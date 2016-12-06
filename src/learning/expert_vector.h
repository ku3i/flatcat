#ifndef EXPERT_VECTOR_H_INCLUDED
#define EXPERT_VECTOR_H_INCLUDED

#include <common/static_vector.h>
#include <control/sensorspace.h>
#include <learning/expert.h>

/* The Expert Vector should merely work as a container
 * and should neither carry any information nor functionality
 * regarding the expert modules in it. However this is theory. :)
 */
class Expert_Vector_Base {
public:

    virtual ~Expert_Vector_Base() = default;

    Expert_Vector_Base( const std::size_t max_number_of_experts
                      , static_vector_interface& payloads )
    : experts()
    , payloads(payloads)
    {
        assert(payloads.size() == max_number_of_experts);
        assert(max_number_of_experts > 0);
        experts.reserve(max_number_of_experts);
        dbg_msg("Done constructing expert vector base"); //TODO remove
    }

          Expert& operator[] (const std::size_t index)       { return experts.at(index); }
    const Expert& operator[] (const std::size_t index) const { return experts.at(index); }

    std::size_t size(void) const { return experts.size(); }

    void copy(std::size_t to, std::size_t from, bool one_shot_learning) {

        experts.at(to).exists = true; // create

        if (one_shot_learning) experts.at(to).reinit_predictor_weights();
        else {
            experts.at(to).predictor->copy( *(experts.at(from).predictor) );
        }

        payloads.copy(to, from); /* take a flawed copy of the payload */
    }

protected:

    std::vector<Expert> experts;
    static_vector_interface& payloads;
};


/**TODO move to separate files, find better names */
class Sensor_Experts : public Expert_Vector_Base {
public:

    Sensor_Experts( const std::size_t         max_number_of_experts
                  , static_vector_interface&  payloads
                  , const sensor_vector&      input
                  , const double              local_learning_rate
                  , const std::size_t         experience_size )
    : Expert_Vector_Base(max_number_of_experts, payloads )
    {
        for (std::size_t i = 0; i < max_number_of_experts; ++i)
            experts.emplace_back( Predictor_ptr( new Predictor(input, local_learning_rate, gmes_constants::random_weight_range, experience_size) )
                                , max_number_of_experts );
    }
};


#include <control/control_vector.h>
class Motor_Experts : public Expert_Vector_Base {
public:

    Motor_Experts( const std::size_t              max_number_of_experts
                 , static_vector_interface&       payloads
                 , const sensor_vector&           input
                 , const double                   local_learning_rate
                 , const std::size_t              experience_size
                 , control::Control_Vector const& ctrl_params )
    : Expert_Vector_Base(max_number_of_experts, payloads )
    {
        assert(ctrl_params.size() == max_number_of_experts);
        for (std::size_t i = 0; i < max_number_of_experts; ++i)
            experts.emplace_back( Predictor_ptr( new Motor_Predictor(input, local_learning_rate, gmes_constants::random_weight_range, ctrl_params.get(i)) )
                                , max_number_of_experts );
    }
};

template <typename PredictorType>
class Expert_Vector : Expert_Vector_Base {
public:

    template<typename... Args>
    Expert_Vector( const std::size_t         max_number_of_experts
                 , static_vector_interface&  payloads /**TODO consider to make payloads optional, e.g. with constructor overload */
                 , const sensor_vector&      input
                 , const Args&...            predictor_args)
    : Expert_Vector_Base(max_number_of_experts, payloads)
    {
        for (std::size_t i = 0; i < max_number_of_experts; ++i)
            experts.emplace_back( Predictor_ptr(new PredictorType(predictor_args...)), max_number_of_experts );
    }
};

typedef Expert_Vector<Motor_Predictor> Motor_Expert_Vector;

#endif // EXPERT_VECTOR_H_INCLUDED
