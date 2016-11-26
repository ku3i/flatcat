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
/** TODO: now that the template argument is gone, move implementation to .cpp */

/** TODO: GMES_Base, Expert_Vector_Base, Predictor_Base */

class GMES : public control::Statemachine_Interface { /* Growing_Multi_Expert_Structure */
public:
    GMES(Expert_Vector& expert, double learning_rate = gmes_constants::global_learning_rate, bool one_shot_learning = true)
    : expert(expert)
    , Nmax(expert.size()) /** TODO: check usage of Nmax */
    , min_prediction_error(.0)
    , learning_progress(.0)
    , learning_rate(learning_rate)
    , one_shot_learning(one_shot_learning)
    , learning_enabled(true)
    , number_of_experts(0)
    , winner(0)
    , last_winner(0)
    , recipient(0)
    , to_insert(0)
    , activations(Nmax)
    , new_node(false)
    {
        assert(in_range(gmes_constants::number_of_initial_experts, std::size_t{1}, Nmax));
        for (std::size_t n = 0; n < gmes_constants::number_of_initial_experts; ++n)
            expert[n].create();
        sts_msg("Created GMES with %u experts and learning rate %.4f", Nmax, learning_rate);
    }

    ~GMES() { dbg_msg("Destroying GMES."); }

    bool        is_learning_enabled       (void) const { return learning_enabled;      }
    bool        has_state_changed         (void) const { return winner != last_winner; }
    bool        has_new_node              (void) const { return new_node;              }

    /** TODO: consider grouping the state variables of gmes to gmes_status and
     *        return this all at once. */
    std::size_t get_number_of_experts     (void) const { return number_of_experts;     }
    std::size_t get_max_number_of_experts (void) const { return expert.size();         }
    std::size_t get_winner                (void) const { return winner;                }
    std::size_t get_recipient             (void) const { return recipient;             }
    std::size_t get_to_insert             (void) const { return to_insert;             }

    double      get_learning_progress     (void) const { return learning_progress;     }
    double      get_min_prediction_error  (void) const { return min_prediction_error;  }

    VectorN const& get_activations        (void) const { return activations;           }


    void enable_learning(bool enable) { learning_enabled = enable; }

    void execute_cycle(void);
    void update_activations(void);

private:

    std::size_t determine_winner          (void);
    std::size_t arg_max_capacity          (void) const;
    std::size_t count_existing_experts    (void) const;
    void        check_learning_capacity   (void) const;

    void        estimate_learning_progress(void);
    void        adjust_learning_capacity  (void);
    void        refresh_transitions       (void);
    void        insert_expert_on_demand   (void);
    void        clear_transitions_to(std::size_t to_clear);

    Expert_Vector& expert;
    const std::size_t Nmax;

    double min_prediction_error;
    double learning_progress;
    const double learning_rate;

    const bool one_shot_learning;
    bool learning_enabled;

    std::size_t number_of_experts;
    std::size_t winner;         // winning expert
    std::size_t last_winner;    // winning expert of last time step
    std::size_t recipient;      // donee of the learning capacity consumed by the winner
    std::size_t to_insert;

    VectorN     activations;
    bool        new_node;

    friend class GMES_Graphics;
    friend class Payload_Graphics;
    friend class Force_Field;
};

#endif // GMES_H
