#ifndef COMPETITIVE_MOTOR_LAYER_H_INCLUDED
#define COMPETITIVE_MOTOR_LAYER_H_INCLUDED

#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <common/log_messages.h>
#include <common/misc.h>

#include <control/controlparameter.h>

namespace MotorLayerConstants {
    const std::size_t initial_learning_capacity = 1000;
    const control::Minimal_Seed_t seed = {2.0, 0.0, 1.0};

    const double symmetry_ratio = 0.5;
}

class MotorUnit {
public:
    MotorUnit(const control::Control_Parameter& seed, const bool exists = false)
    : weights(seed)
    , selection_count(0) // selections should also be counted by q for better analysis of competition for motor neurons
    , learning_capacity(MotorLayerConstants::initial_learning_capacity)
    , exists(exists)
    {
        dbg_msg("Creating Motor Unit with %u weights: (%s)", seed.size(), to_str(seed.get_parameter()).c_str());
    }

    MotorUnit() : weights(), selection_count(), learning_capacity(MotorLayerConstants::initial_learning_capacity), exists(false) {}

    MotorUnit& operator=(const MotorUnit& other) {
        dbg_msg("Copy motor neuron.");
        weights = other.weights; // copy with flaws?
        exists  = true;
        selection_count = 0;
        learning_capacity = other.learning_capacity;
        return *this;
    }

    control::Control_Parameter weights;
    std::size_t                selection_count;
    std::uint64_t              learning_capacity;
    bool                       exists;
};


class CompetitiveMotorLayer
{
public:
    CompetitiveMotorLayer( const robots::Robot_Interface& robot
                         , static_vector<State_Payload>&  states
                         , const control::Control_Vector& parameter_sets
                         , const std::size_t              number_of_motor_units
                         , const std::size_t              number_of_motor_units_begin
                         , const double                   mutation_rate
                         , const double                   learning_rate
                         , bool                           do_adaption
                         , const control::Minimal_Seed_t& seed = MotorLayerConstants::seed)
    : robot(robot)
    , states(states)
    , parameter_sets(parameter_sets)
    , motor_units()
    , mutated_weights()
    , last_selected_idx()
    , recipient_idx()
    , do_adaption(do_adaption)
    , mutation_rate(mutation_rate)
    , learning_rate(learning_rate)
    {
        dbg_msg("Creating competitive motor layer with %u motor units.", number_of_motor_units);
        assert_in_range(number_of_motor_units, 1ul, 100ul);
        assert_in_range(learning_rate, 0.0010, 0.5);
        assert_in_range(mutation_rate, 0.0001, 0.5);
        motor_units.reserve(number_of_motor_units);

        if (parameter_sets.get_number_of_sets() <= number_of_motor_units)
            sts_msg("Initialize %u of %u parameter sets.", parameter_sets.get_number_of_sets(), number_of_motor_units);
        else
            wrn_msg("Can not initialize all %u parameter sets. Limit is %u.", parameter_sets.get_number_of_sets(), number_of_motor_units);

        for (std::size_t i = 0; i < std::min(parameter_sets.get_number_of_sets(), number_of_motor_units); ++i)
        {
            motor_units.emplace_back(parameter_sets.get(i), true);
        }
        assert(motor_units.size() <= number_of_motor_units);

        //if ?
        for (std::size_t i = motor_units.size(); i < number_of_motor_units; ++i) {
            control::Control_Parameter params = control::get_initial_parameter(robot, seed, /*symmetric?*/(i % 2 == 0));

            control::randomize_control_parameter(params, 0.1, 1.0); /**TODO make to settings, and constrain motor self not not go beyond zero */
            /**TODO also: make settings grouped and only give the local settings as ref */
            motor_units.emplace_back(params, (i < number_of_motor_units_begin));
        }
// else
//        for (std::size_t i = motor_units.size(); i < number_of_motor_units; ++i)
//            motor_units.emplace_back(/*placeholder motor units*/);

        assert(motor_units.size() == number_of_motor_units);
    }

    bool exists(const std::size_t idx) const { assert(idx < motor_units.size()); return motor_units[idx].exists; }

    /** Only the current active motor neuron is going to be adapted. This adaption happens
     *  iff we receive a positive delta from the superordinate learning layer, e.g. sarsa/RL.
     *  That means that only if we notice an increase in our reward expectation we take over
     *  the just evaluated changes to our controller weights.
     *  By using the delta signal we make sure that different policies with different relative
     *  reward levels do not affect the weight adjustment and are treated more or less equally.
     */
    void adapt(bool positive_delta)
    {
        if (not do_adaption) return;
        assert(motor_units[last_selected_idx].exists == true); // must not be executed on non-existing units
        if (positive_delta) {
            do_adapt(last_selected_idx);
        }
    }

    /** Create a randomized variant of the current motor neuron's controller weights.
     *  This mutated weights will be evaluated by the next eigenzeit cycle and in case
     *  of success, i.e. a positive learning delta, the new mutated weights will be adopted.
     */
    void create_mutated_weights(std::size_t selected_idx)
    {
        assert(exists(selected_idx)); // must not be executed on non-existing units

        mutated_weights = motor_units[selected_idx].weights;

        if (do_adaption)
            for (std::size_t i = 0; i < mutated_weights.size(); ++i)
                mutated_weights[i] += random_value(-mutation_rate, +mutation_rate);
        /**TODO
         * I want to use here the same mechanics as in Individual::mutate(void) (evolution)
         */
        ++(motor_units[selected_idx].selection_count);
        last_selected_idx = selected_idx;
    }

    const std::vector<double>& get_weights(std::size_t index) const {
        assert(index < motor_units.size());
        return motor_units[index].weights.get_parameter();
    }

    const control::Control_Parameter& get_mutated_weights(void) const { return mutated_weights; }

    const MotorUnit& get_unit(std::size_t index) const {
        assert(index < motor_units.size());
        return motor_units[index];
    }
    std::size_t get_number_of_motor_units(void) const { return motor_units.size(); }

    std::size_t get_number_of_available_motor_units(void) const
    {
        std::size_t available_units = 0;
        for (std::size_t i = 0; i < motor_units.size(); ++i)
            if (motor_units[i].exists)
                ++available_units;
        return available_units;
    }

    void enable_adaption(bool enable) { do_adaption = enable; }
    bool is_adaption_enabled(void) const { return do_adaption; }

private:

    /** We're using the learning capacity here to decide which module should be replaced
     *  by the cloning. We choose the candidate with highest learning capacity to be
     *  replaced by the one having the lowest learning capacity.
     */
    std::size_t find_replacement_candidate_for(std::size_t min_index)
    {
        /* always returns a valid index */
        std::size_t max_learning_capacity = motor_units[min_index].learning_capacity;
        std::size_t argmax = min_index;
        for (std::size_t idx = 0; idx < motor_units.size(); ++idx) {
            /* if there's a free slot, take it */
            if (not motor_units[idx].exists) return idx;
            if (motor_units[idx].learning_capacity > max_learning_capacity)
            {
                max_learning_capacity = motor_units[idx].learning_capacity;
                argmax = idx;
            }
        }
        return argmax;
    }

    /** Replace the least adapted motor neuron by a copy of the most adapted one.
     *  When cloning the motor neuron, the corresponding q-values and eligibility
     *  trace which the motor neuron is connected with will also be cloned.
     *  TODO: Instead of just cloning, think of using a kind of crossover operation
     *  for the creation of the new unit.
     */
    void clone(std::size_t current_idx)
    {
        std::size_t replace_idx = find_replacement_candidate_for(current_idx);
        dbg_msg("Cloning motor unit %u to %u", current_idx, replace_idx);

        unsigned int learning_capacity_remainder   = (motor_units[current_idx].learning_capacity + motor_units[replace_idx].learning_capacity) % 2;
        motor_units[current_idx].learning_capacity = (motor_units[current_idx].learning_capacity + motor_units[replace_idx].learning_capacity) / 2;

        motor_units[replace_idx] = motor_units[current_idx]; // replace
        motor_units[replace_idx].learning_capacity += learning_capacity_remainder;

        /** Change symmetry to balance the ratio of symmetric and asymmetric joint count */
        if (get_number_of_symmetric_units() < (motor_units.size()/2))
            motor_units[replace_idx].weights = make_symmetric(robot, motor_units[replace_idx].weights);
        else
            motor_units[replace_idx].weights = make_asymmetric(robot, motor_units[replace_idx].weights);

        /* copy associate Q-values and eligibility traces */
        for (std::size_t i = 0; i < states.size(); ++i)
            states[i].copy_payload(current_idx, replace_idx);
    }

    /** Shift the original weight vector of the selected motor neuron a little bit
     *  towards the mutated weight vector. Decrease learning capacity.
     */
    void do_adapt(std::size_t current_idx) {
        --(motor_units[current_idx].learning_capacity);
        assert(mutated_weights.size() == motor_units[current_idx].weights.size()); /** TODO: this was thrown, CHECK!!! somehow large reward values will induce that*/
        for (std::size_t i = 0; i < motor_units[current_idx].weights.size(); ++i) {
            double delta = learning_rate * (mutated_weights[i] - motor_units[current_idx].weights[i]);
            motor_units[current_idx].weights[i] += delta;
        }

        /** When learning capacity is exhausted we clone the motor unit */
        if (motor_units[current_idx].learning_capacity == 0) {
            clone(current_idx);
        }

        /** Redistribute the learning capacity to the group of all (possible) motor units.
         *  This enforces the growing up to the maximal size of the group.
         */
        recipient_idx = random_index(motor_units.size());
        ++(motor_units[recipient_idx].learning_capacity);

        /* assert that learning capacity does not drain */
        unsigned int sum = 0;
        for (std::size_t i = 0; i < motor_units.size(); ++i)
            sum += motor_units[i].learning_capacity;
        assert(sum == motor_units.size() * MotorLayerConstants::initial_learning_capacity);

    }

    std::size_t get_number_of_symmetric_units() {
        std::size_t num_sym = 0;
        for (std::size_t i = 0; i < motor_units.size(); ++i)
            if (motor_units[i].weights.is_symmetric())
                ++num_sym;
        return num_sym;
    }

    const robots::Robot_Interface& robot; // only needed for changing symmetry of controller weights
    static_vector<State_Payload>&  states;
    const control::Control_Vector& parameter_sets;

    std::vector<MotorUnit>         motor_units;
    control::Control_Parameter     mutated_weights;
    std::size_t                    last_selected_idx;
    std::size_t                    recipient_idx;
    bool                           do_adaption;

    const double                   mutation_rate;
    const double                   learning_rate;

    friend class CompetitiveMotorLayer_Graphics;
};



#endif // COMPETITIVE_MOTOR_LAYER_H_INCLUDED
