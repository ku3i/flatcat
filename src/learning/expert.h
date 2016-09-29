#ifndef EXPERT_H_INCLUDED
#define EXPERT_H_INCLUDED

#include <learning/gmes_constants.h>
#include <learning/predictor.h>
#include <learning/sensorspace.h>
#include <common/static_vector.h>


template <typename Payload_t> class Expert_Vector;
template <typename Expert_Vector_t> class GMES;

template <typename Payload_t>
class Expert {
public:

    Expert( const sensor_vector& input
          , const std::size_t    max_number_of_nodes
          , Payload_t&           payload
          , const double         local_learning_rate = gmes_constants::local_learning_rate
          , const std::size_t    experience_size     = gmes_constants::experience_size
          )
    : exists(false)
    , predictor(input, local_learning_rate, gmes_constants::random_weight_range, experience_size)
    , learning_capacity(gmes_constants::initial_learning_capacity)
    , perceptive_width(gmes_constants::perceptive_width)
    , transition(max_number_of_nodes)
    , payload(payload)
    { }

    ~Expert() { }

    /* TODO get 3d coordinates (graphical representation) */

    bool   learning_capacity_is_exhausted(void) const { return learning_capacity < gmes_constants::learning_capacity_exhausted; }
    double get_learning_capacity         (void) const { return learning_capacity; }
    double get_prediction_error          (void) const { return predictor.get_prediction_error();  }
    void   adapt_weights                 (void)       { predictor.adapt_with_experience_replay();   }

    double update_and_get_activation     (void) const {
        if (not exists) return 0.0;
        double e = predictor.get_prediction_error();
        return exp(-e*e/perceptive_width);
    }

    void   copy_predictor_weights_from   (const Expert& other)  { predictor.copy_weights_from(other.predictor); }
    void   reinit_predictor_weights      (void)                 { predictor.init_weights();                     }

    void   copy_payload_from   (const Expert& other)  { payload.copy_with_flaws(other.payload);     }

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

        /* take a flawed copy of the payload */
        copy_payload_from(other);
    }

    bool exists_transition(std::size_t index) const { assert(index < transition.size()); return transition[index] > gmes_constants::transition_exist_treshold; }
    std::size_t get_number_of_transitions()   const { return 0; /* TODO implement */ }

private:
    bool         exists;
    Predictor    predictor;
    double       learning_capacity; /** TODO think of having the learning capacity as integer value and only count discrete learning steps*/
    const double perceptive_width;
    VectorN      transition;              // validity of connections //TODO some day: max k connections
    Payload_t&   payload;

    typedef Expert_Vector<Payload_t> Expert_Vector_t;
    typedef GMES<Expert_Vector_t>    GMES_t;

    template <typename Expert_Vector_t> friend class GMES;
    template <typename Expert_Vector_t> friend class GMES_Graphics;
    template <typename Expert_Vector_t> friend class Payload_Graphics;
    template <typename Expert_Vector_t> friend class Force_Field;
};

template <typename Payload_t>
class Expert_Vector {
public:
    typedef Expert<Payload_t> Expert_t;

    Expert_Vector( const std::size_t         max_number_of_experts
                 , const sensor_vector&      input
                 , static_vector<Payload_t>& payloads
                 , const double              local_learning_rate
                 , const std::size_t         experience_size
                 )
    : expert()
    {
        assert(payloads.size() == max_number_of_experts);
        expert.reserve(max_number_of_experts);
        for (std::size_t i = 0; i < max_number_of_experts; ++i)
            expert.emplace_back(input, max_number_of_experts, payloads[i], local_learning_rate, experience_size);
    }

          Expert_t& operator[] (const std::size_t index)       { assert(index < expert.size()); return expert[index]; }
    const Expert_t& operator[] (const std::size_t index) const { assert(index < expert.size()); return expert[index]; }

    std::size_t get_max_number_of_experts(void) const { return expert.size(); }

private:
    std::vector<Expert_t> expert;
};

#endif // EXPERT_H_INCLUDED
