#ifndef MOTOR_PREDICTOR_H_INCLUDED
#define MOTOR_PREDICTOR_H_INCLUDED

#include <common/log_messages.h>
#include <robots/robot.h>
#include <control/controlparameter.h>
#include <control/jointcontrol.h>
#include <control/control_core.h>
#include <learning/predictor.h>

//#include <draw/display.h>

namespace learning {


class Motor_Predictor : public Predictor_Base {
public:
    Motor_Predictor( robots::Robot_Interface const& robot
                   , sensor_vector const& motor_targets
                   , double learning_rate
                   , double random_weight_range
                   , std::size_t experience_size
                   , control::Control_Parameter const& parameter
                   , double noise_level
                   )
    : Predictor_Base(motor_targets, learning_rate, random_weight_range, experience_size)
    , robot(robot)
    , core(robot)
    , motor_targets(motor_targets)
    , params(control::turn_symmetry(robot, control::make_asymmetric(robot, parameter)))
    , params_changed(false)
    , noise_level(noise_level)
    {
        core.apply_weights(robot, params.get_parameter());
    }

    void copy(Predictor_Base const& other) override {
        Predictor_Base::operator=(other); // copy base members
        Motor_Predictor const& rhs = dynamic_cast<Motor_Predictor const&>(other); /**TODO definitely write a test for that crap :) */
        core = rhs.core;
    }

    double predict(void) override {
        core.prepare_inputs(robot);
        add_noise_to_inputs(core.input, noise_level);
        assert(!(params.is_mirrored() and params.is_symmetric()));
        core.update_outputs(robot, params.is_symmetric(), params.is_mirrored());
        //vector_tanh(core.activation);
        vector_clip(core.activation);
        return calculate_prediction_error();
    }

    double verify(void) override {
        assert(!(params.is_mirrored() and params.is_symmetric()));
        core.update_outputs(robot, params.is_symmetric(), params.is_mirrored());
        vector_clip(core.activation);
        return calculate_prediction_error();
    }

    void initialize_randomized(void) override {

        for (auto& w_k : core.weights)
            for (auto& w_ik : w_k)
                w_ik += random_value(-random_weight_range,+random_weight_range); // don't override weights initialized by params

        auto initial_experience = input.get(); /**TODO this code is the same in state predictor, move to base?*/
        for (auto& w: initial_experience)
            w += random_value(-random_weight_range, random_weight_range);
        experience.assign(experience.size(), initial_experience);

        prediction_error = predictor_constants::error_min;

    }

    void initialize_from_input(void) override { assert(false && "one shot learning not supported."); }

    Predictor_Base::vector_t const& get_prediction(void) const override { return core.activation; }

    control::Control_Parameter const& get_controller_weights() const {
        if (params_changed) {
            params.set_from_matrix(core.weights);
            //params.print();
            params_changed = false;
        }
        return params;
    }

    /*
    void draw(void) const {
        float s = 2.0/core.weights.size();
        unsigned i = 0;
        for (auto const& wi : core.weights)
            draw_vector2(0.0 + s*i++, 0.0, 0.045, s, wi, 3.0);
    }
    */

    vector_t const& get_weights(void) const override { assert(false); return dummy; /*not implemented*/ }
    vector_t      & set_weights(void)       override { assert(false); return dummy; /*not implemented*/ }

private:
    robots::Robot_Interface const&          robot;
    control::Fully_Connected_Symmetric_Core core;
    sensor_vector const&                    motor_targets;

    mutable control::Control_Parameter      params; // for loading, saving, buffering
    mutable bool                            params_changed;
    const double                            noise_level;

    VectorN dummy = {}; // remove when implementing get_weights

    void learn_from_input_sample(void) override {
        /** Regarding the gradient descent on the motor controller weights:
         *  TODO: We handle the 'recurrent' motor connections as just ordinary inputs, this can get stuck,
         *  since it learns to copy only the old motor value's input weights.
         */
        auto const& inputs = core.input;
        VectorN const& predictions = core.activation;
        std::vector<VectorN>& weights = core.weights; // non const ref
        assert(motor_targets.size() == predictions.size());
        assert(motor_targets.size() == weights.size());
        assert(inputs.size() == weights.at(0).size());

        for (std::size_t k = 0; k < motor_targets.size(); ++k) { // for num of motor outputs
            double err = motor_targets[k] - predictions[k];
            for (std::size_t i = 0; i < weights[k].size(); ++i) { // for num of inputs
                weights[k][i] += learning_rate * err * tanh_(predictions[k]) * inputs[i].x;
                /** This gradient is intentionally wrong, derivative of clip transfer function would be not continuous. */
            }
        }
        params_changed = true; /**TODO: move to learn_from_experience, if supported */
    }

    void learn_from_experience(std::size_t /*skip_idx*/) override { assert(false && "Learning from experience is not implemented yet."); }

    void add_noise_to_inputs(std::vector<control::sym_input>& inputs, double sigma) {
        const double s = sigma/sqrt(inputs.size());
        for (auto &in : inputs) {
            const double rndval = rand_norm_zero_mean(s);
            in.x += rndval;
            in.y += rndval;
        }
    }

    friend class Motor_Predictor_Graphics;
};

} // namespace learning

#endif // MOTOR_PREDICTOR_H_INCLUDED
