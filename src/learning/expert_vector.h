#ifndef EXPERT_VECTOR_H_INCLUDED
#define EXPERT_VECTOR_H_INCLUDED

#include <common/static_vector.h>
#include <control/sensorspace.h>
#include <learning/expert.h>

/* The Expert Vector should merely work as a container
 * and should neither carry any information nor functionality
 * regarding the expert modules in it. However this is theory. :)
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
            experts.emplace_back( input
                                , Predictor_ptr( new Predictor(input, local_learning_rate, gmes_constants::random_weight_range, experience_size) )
                                , max_number_of_experts
                                , local_learning_rate
                                , experience_size );
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

private:
    std::vector<Expert> experts;
    static_vector_interface& payloads;
};


#endif // EXPERT_VECTOR_H_INCLUDED
