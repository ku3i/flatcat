#ifndef EXPERT_H_INCLUDED
#define EXPERT_H_INCLUDED

#include <memory>
#include <learning/gmes_constants.h>
#include <learning/predictor.h>
#include <control/sensorspace.h>

/** TODO
 *  + get 3d graphical representation, note: must provided by the underlying type
 *  + in general, an expert should not care if it exists or not, this should be supervised by the expert_vector.
 */

class Expert {
    Expert(const Expert& other) = delete; // non construction-copyable

public:

    Expert( Predictor_ptr     predictor
          , const std::size_t max_number_of_nodes )
    : exists(false)
    , predictor(std::move(predictor))
    , learning_capacity(gmes_constants::initial_learning_capacity)
    , perceptive_width(gmes_constants::perceptive_width)
    , transition(max_number_of_nodes)
    { }

    Expert(Expert&& other) = default;

    bool   learning_capacity_is_exhausted(void) const { return learning_capacity < gmes_constants::learning_capacity_exhausted; }
    double get_prediction_error          (void) const { return predictor->get_prediction_error(); }
    void   adapt_weights                 (void)       { predictor->adapt();                       }
    void   reinit_predictor_weights      (void)       { predictor->initialize_from_input();       }

    double update_and_get_activation     (void) const {
        if (not exists) return 0.0;
        double e = predictor->get_prediction_error();
        return exp(-e*e/perceptive_width);
    }

    /* make prediction and update prediction error */
    double make_prediction(void) { return predictor->predict(); }

    void clear_transitions(void) { for (auto& t : transition) t = 0.0; }

    void create_randomized(void) {
        exists = true;
        predictor->initialize_randomized();
    }

    bool exists_transition(std::size_t index) const { return transition.at(index) > gmes_constants::transition_exist_treshold; }
    void reset_transition(std::size_t index) { transition.at(index) = gmes_constants::initial_transition_validation; }

    Predictor_Base const& get_predictor(void) const { return *predictor; }
    bool does_exists(void) const { return exists; }


private:
    bool          exists;
    Predictor_ptr predictor;
    double        learning_capacity;
    const double  perceptive_width;
    VectorN       transition;       // validity of connections
    /** TODO some day: restrict to max. k connections */

    friend class Expert_Vector_Base;
    friend class GMES;
    friend class GMES_Graphics;
    friend class Force_Field;
};


#endif // EXPERT_H_INCLUDED
