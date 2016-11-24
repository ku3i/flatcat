#ifndef GMES_H
#define GMES_H

#include <cmath>
#include <vector>
#include <cassert>
#include <common/log_messages.h>
#include <common/modules.h>
#include <common/vector_n.h>

#include <control/statemachine.h>

#include <learning/expert.h>
#include <learning/gmes_constants.h>
#include <learning/q_function.h>

#include <learning/payload.h>
/* first object oriented implementation of GMES
 * 23.02.2015 (Elmar ist heute 16 Monate alt geworden) */


/* TODO:
 * find a better name for 'to_insert'
 * ---
 * Überlege Dir eine geschachtelte Gmes-Schichten-Anordnung, welche zunächst aus Sensordaten (1)
 * Fixpunkte und verschiedene Attraktoren (2) erkennt und dann in "kombinierte Gelenke"-Features (3)
 * bishin zu Körperbewegungen oder -posen (4) erkennt.
 */

/** TODO: namespace learning */
/** TODO: move implementation to .cpp */

template <typename Expert_Vector_t>
class GMES : public control::Statemachine_Interface { /* Growing_Multi_Expert_Structure */
public:
    GMES(Expert_Vector_t& expert, double learning_rate = gmes_constants::global_learning_rate)
    : expert(expert)
    , Nmax(expert.get_max_number_of_experts()) /** TODO: check usage of Nmax */
    , min_prediction_error(.0)
    , learning_progress(.0)
    , sum_capacity(.0)
    , learning_rate(learning_rate)
    , one_shot_learning(true) /** TODO could be a constructor argument */
    , learning_enabled(true)
    , number_of_experts(0)
    , winner(0)
    , last_winner(0)
    , recipient(0)
    , to_insert(0)
    , activations(Nmax)
    , new_node(false)
    {
        assert(gmes_constants::number_of_initial_experts < Nmax);
        assert(gmes_constants::number_of_initial_experts > 0);

        /** TODO create the first one on input? */
        for (std::size_t n = 0; n < gmes_constants::number_of_initial_experts; ++n)
            expert[n].create();
        sts_msg("Created GMES with %u experts and learning rate %.4f", Nmax, learning_rate);
    }

    ~GMES() { dbg_msg("Destroying GMES."); }

    std::size_t get_number_of_experts     (void) const { return number_of_experts;    }
    std::size_t get_max_number_of_experts (void) const { return expert.get_max_number_of_experts(); }
    double      get_learning_progress     (void) const { return learning_progress;    }
    double      get_min_prediction_error  (void) const { return min_prediction_error; }

    std::size_t get_winner                (void) const { return winner;      }
    std::size_t get_recipient             (void) const { return recipient;   }
    std::size_t get_to_insert             (void) const { return to_insert;   }
    const VectorN&  get_activations       (void) const { return activations; }

    /**TODO try to split this long method into smaller bites:
    * 1) Detection part
    * 2) Adaption part
    */

    void execute_cycle(void)
    {
        /* backup last winner */
        last_winner = winner;
        new_node = false;

        /* determine new winner */
        winner = determine_winner();

        /* if needed, insert expert before adaptation takes place */
        if (expert[winner].learning_capacity_is_exhausted() && to_insert != winner)
        {
            expert[to_insert].copy_from(expert[winner], one_shot_learning); // copy weights
            expert.copy_payload(to_insert, winner);

            /* clear transitions emanating from 'to_insert' */
            expert[to_insert].clear_transitions();
            clear_transitions_to(to_insert);

            /* set new transition */
            expert[to_insert].reset_transition(winner);
            winner = to_insert;
            new_node = true;
        }

        /* adapt weights of winner */
        expert[winner].adapt_weights();

        /* estimate learning progress: L = -dE/dt */
        double prediction_error_before_adaption = expert[winner].get_prediction_error();
        learning_progress = prediction_error_before_adaption - expert[winner].make_prediction();

        //dbg_msg("e: %1.20f %s", learning_progress, (new_node)? "new":"");
        assert_in_range(learning_progress, 0.0, 1.0);

        /* reduce learning capacity proportional to progress in learning */ //TODO move to expert
        /** This should be rethought, 'proportional to the progress in learning' or better 'equal steps'? */
        double delta_capacity = expert[winner].learning_capacity - expert[winner].learning_capacity * exp(-learning_rate * learning_progress); // TODO x * (1-exp)
        expert[winner].learning_capacity -= delta_capacity;

        recipient = random_index(Nmax);
        assert(recipient < Nmax);
        expert[recipient].learning_capacity += delta_capacity;

        /* devaluate connections emanating from n1 */
        for (std::size_t n = 0; n < Nmax; ++n) {
            expert[n].transition[winner] *= exp(-learning_rate * learning_progress); /** TODO: this factor can be computed beforehand and used several times*/
            expert[winner].transition[n] *= exp(-learning_rate * learning_progress);
        }

        /* validate the connection from last_winner to winner */
        /** TODO: validate_transition(); */
        expert[winner].transition[last_winner] = gmes_constants::initial_transition_validation;

        /* count experts */
        number_of_experts = count_existing_experts(); /** could be a member method of experts class */

        /* assert learning_capacity does not leak */
        check_learning_capacity();

        /* choose next expert to insert
         * if all available experts are in use,
         * find and take the one with max. learning capacity */
        to_insert = (number_of_experts < Nmax) ? number_of_experts : arg_max_capacity();

        /* testing */
        update_activations();
    }

    void update_activations (void)
    {
        /* compute activations */
        for (std::size_t n = 0; n < Nmax; ++n)
            activations[n] = expert[n].update_and_get_activation();
    }

    bool has_new_node     (void) const { return new_node; }
    bool has_state_changed(void) const { return winner != last_winner; }

    bool is_learning_enabled(void) const { return learning_enabled; }
    void enable_learning(bool enable) { learning_enabled = enable; }

private:

    std::size_t determine_winner(void)
    {
        std::size_t winner = 0;
        double min_error = expert[0].make_prediction();

        for (std::size_t n = 1; n < Nmax; ++n)
        {
            if (expert[n].exists)
            {
                /* compute prediction error and
                * find the best predicting expert */
                if (expert[n].make_prediction() < min_error)
                {
                    winner = n;
                    min_error = expert[n].get_prediction_error();
                }
            }
        }
        assert(winner < Nmax);
        min_prediction_error = min_error;
        return winner;
    }

    std::size_t arg_max_capacity(void) const
    {
        std::size_t with_max_capacity = 0;
        double max_capacity = 0.0;
        for (std::size_t n = 0; n < Nmax; ++n)
        {
            if (expert[n].exists && expert[n].learning_capacity > max_capacity) {
                max_capacity = expert[n].learning_capacity;
                with_max_capacity = n;
            }
        }
        return with_max_capacity;
    }

    std::size_t count_existing_experts(void) const /**TODO move to expert vector class */
    {
        std::size_t num_experts = 0;
        for (std::size_t n = 0; n < Nmax; ++n)
            if (expert[n].exists)
                ++num_experts;
        return num_experts;
    }

    void check_learning_capacity(void) const
    {
        double sum_capacity = 0.0;
        for (std::size_t n = 0; n < Nmax; ++n)
            sum_capacity += expert[n].learning_capacity;

        double leakage = std::abs(sum_capacity - Nmax * gmes_constants::initial_learning_capacity);

        if (leakage > 1e-12)
            wrn_msg("Learning capacity is leaking %e", sum_capacity, leakage);
    }

    void clear_transitions_to(std::size_t to_clear)
    {
        for (std::size_t n = 0; n < Nmax; ++n)
            expert[n].transition[to_clear] = .0;
    }

    Expert_Vector_t& expert;
    const std::size_t Nmax;

    double min_prediction_error;
    double learning_progress;
    double sum_capacity;
    double learning_rate;

    bool one_shot_learning;
    bool learning_enabled;

    std::size_t number_of_experts;
    std::size_t winner;         // winning expert
    std::size_t last_winner;    // winning expert from last timestep
    std::size_t recipient;      // donee of the learning capacity consumed by the winner
    std::size_t to_insert;

    VectorN     activations;
    bool        new_node;

    template <typename T> friend class GMES_Graphics;
    template <typename T> friend class Payload_Graphics;
    template <typename T> friend class Force_Field;
};

#endif // GMES_H
