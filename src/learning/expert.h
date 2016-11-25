#ifndef EXPERT_H_INCLUDED
#define EXPERT_H_INCLUDED

#include <learning/gmes_constants.h>
#include <learning/predictor.h>
#include <learning/sensorspace.h>
#include <common/static_vector.h>


class Expert {
public:

    Expert( const sensor_vector& input
          , const std::size_t    max_number_of_nodes
          , const double         local_learning_rate = gmes_constants::local_learning_rate
          , const std::size_t    experience_size     = gmes_constants::experience_size
          )
    : exists(false)
    , predictor(input, local_learning_rate, gmes_constants::random_weight_range, experience_size)
    , learning_capacity(gmes_constants::initial_learning_capacity)
    , perceptive_width(gmes_constants::perceptive_width)
    , transition(max_number_of_nodes)
    { }

    ~Expert() { }

    /** TODO get 3d coordinates (graphical representation) note: must provided by the underlying type */

    bool   learning_capacity_is_exhausted(void) const { return learning_capacity < gmes_constants::learning_capacity_exhausted; }
    double get_prediction_error          (void) const { return predictor.get_prediction_error();  }
    void   adapt_weights                 (void)       { predictor.adapt_with_experience_replay(); }

    double update_and_get_activation     (void) const {
        if (not exists) return 0.0;
        double e = predictor.get_prediction_error();
        return exp(-e*e/perceptive_width);
    }

    void   copy_predictor_weights_from   (const Expert& other)  { predictor.copy_weights_from(other.predictor); }
    void   reinit_predictor_weights      (void)                 { predictor.init_weights();                     }

    const VectorN& get_weights (void)           const { return predictor.get_weights();             }

    /* make prediction and update prediction error */
    double make_prediction(void) { return predictor.predict(); }

    void clear_transitions(void) {
        for (std::size_t n = 0; n < transition.size(); ++n)
            transition[n] = 0.0;
    }

    void create(void) {
        exists = true;
        predictor.init_random_weights();
    }

    void copy_from(const Expert& other, bool one_shot_learning)
    {
        exists = true;

        /* copy weights */
        if (one_shot_learning) reinit_predictor_weights();
        else copy_predictor_weights_from(other);
    }

    bool exists_transition(std::size_t index) const { assert(index < transition.size()); return transition[index] > gmes_constants::transition_exist_treshold; }

    Predictor const& get_predictor(void) const { return predictor; }
    void reset_transition(std::size_t index) { assert(index < transition.size()); transition[index] = gmes_constants::initial_transition_validation; }
    bool does_exists(void) const { return exists; }


private:
    bool         exists;
    Predictor    predictor;
    double       learning_capacity; /** TODO think of having the learning capacity as integer value and only count discrete learning steps*/
    const double perceptive_width;
    VectorN      transition;              // validity of connections //TODO some day: max k connections

    friend class GMES;
    friend class GMES_Graphics;
    friend class Force_Field;
};


/* The Expert Vector should merely work as a container
 * and should neither carry any information nor functionality
 * regarding the expert modules in it.
 */
class Expert_Vector {
public:

    Expert_Vector( const std::size_t         max_number_of_experts
                 , const sensor_vector&      input
                 , static_vector_interface&  payloads
                 , const double              local_learning_rate
                 , const std::size_t         experience_size
                 )
    : experts()
    , payloads(payloads)
    {
        assert(payloads.size() == max_number_of_experts);
        experts.reserve(max_number_of_experts);
        for (std::size_t i = 0; i < max_number_of_experts; ++i)
            experts.emplace_back(input, max_number_of_experts, local_learning_rate, experience_size);
    }

          Expert& operator[] (const std::size_t index)       { assert(index < experts.size()); return experts[index]; }
    const Expert& operator[] (const std::size_t index) const { assert(index < experts.size()); return experts[index]; }

    std::size_t get_max_number_of_experts(void) const { return experts.size(); }

    void copy_payload(std::size_t to, std::size_t from) {
        assert(from < experts.size());
        assert(to   < experts.size());
        //dbg_msg("Copy from %u to %u", from, to);
        payloads.copy(to, from); /* take a flawed copy of the payload */
    }

private:
    std::vector<Expert> experts;
    static_vector_interface& payloads;
};

#endif // EXPERT_H_INCLUDED
