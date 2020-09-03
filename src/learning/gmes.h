#ifndef GMES_H
#define GMES_H

#include <cmath>
#include <vector>
#include <cassert>
#include <common/log_messages.h>
#include <common/modules.h>
#include <common/vector_n.h>
#include <control/statemachine.h>
#include <learning/expert_vector.h>
#include <learning/gmes_constants.h>
#include <learning/q_function.h>
#include <learning/payload.h>
#include <learning/learning_machine_interface.h>
/* first object oriented implementation of GMES
 * 23.02.2015 (Elmar ist heute 16 Monate alt geworden) */


/**
 * Überlege Dir eine geschachtelte Gmes-Schichten-Anordnung, welche zunächst aus Sensordaten (1)
 * Fixpunkte und verschiedene Attraktoren (2) erkennt und dann in "kombinierte Gelenke"-Features (3)
 * bishin zu Körperbewegungen oder -posen (4) erkennt.
 */

/** TODO: namespace learning */
/** TODO: compute the activations of GMES as softmax activation function (YL)*/


class GMES_Graphics;
class Payload_Graphics;
class Force_Field;

class GMES : public control::Statemachine_Interface, public learning::Learning_Machine_Interface { /* Growing_Multi_Expert_Structure */
    GMES(const GMES& other) = delete; // non construction-copyable

public:
    GMES(GMES&& other) = default;

    explicit GMES( Expert_Vector& expert
        , double learning_rate = gmes_constants::global_learning_rate
        , bool one_shot_learning = true
        , std::size_t number_of_initial_experts = gmes_constants::number_of_initial_experts
        , std::string const& name = "...");

    ~GMES();

    bool        is_learning_enabled       (void) const { return learning_enabled;      }
    bool        has_state_changed         (void) const { return winner != last_winner; }
    bool        has_new_node              (void) const { return new_node;              }

    std::size_t get_number_of_experts     (void) const { return number_of_experts;     }
    std::size_t get_max_number_of_experts (void) const { return expert.size();         }
    std::size_t get_winner                (void) const { return winner;                }
    std::size_t get_state                 (void) const { return winner;                }
    std::size_t get_recipient             (void) const { return recipient;             }
    std::size_t get_to_insert             (void) const { return to_insert;             }

    double      get_learning_progress     (void) const { return learning_progress;     }
    double      get_min_prediction_error  (void) const { return min_prediction_error;  }

    VectorN const& get_activations        (void) const { return activations;           }


    void enable_learning(bool enable);
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

    std::size_t number_of_experts;  // number of existing experts
    std::size_t winner;             // winning expert
    std::size_t last_winner;        // winning expert of last time step
    std::size_t recipient;          // donee of the learning capacity consumed by the winner
    std::size_t to_insert;          // expert to be inserted on demand

    VectorN     activations;
    bool        new_node;

    std::string name;

    friend class GMES_Graphics;
    friend class Payload_Graphics;
    friend class Force_Field;
};

#endif // GMES_H
