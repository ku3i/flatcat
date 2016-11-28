#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <common/modules.h>
#include <common/vector_n.h>
#include <common/log_messages.h>
#include <learning/sensorspace.h>

/** Notes regarding normalizing the prediction error
 *  N: input size
 *  a: max input range [-a,+a], a =!= 1
 *  The max prediction error e_max = sqrt(N * (2a)^2)
 *  Normalize factor is therefore 1/e_max
 */

/** Experience Replay.
 *  Do not learn directly every sample. But put new sample
 *  on random position into experience replay list and learn the list on every step.
 */
namespace predictor_constants {
    const double error_max = 1.0;
    const double error_min = 0.0;
}

class Predictor {

    Predictor(const Predictor& other) = delete; // non construction-copyable

public:

    Predictor( const sensor_vector& input
             , const double         learning_rate
             , const double         random_weight_range
             , const std::size_t    experience_size = 1 );

    Predictor& operator=(const Predictor& other);

    Predictor(Predictor&& other) = default;

    double predict(void);

    double get_prediction_error(void) const { return prediction_error; }
    const VectorN& get_weights(void) const { return weights; }

    void adapt_with_experience_replay(void);

    void initialize_randomized(void);
    void initialize_from_input(void);

private:

    void adapt(void);
    void adapt_by_experience_replay(std::size_t skip_idx);

    /* constants */
    const sensor_vector& input;
    const double         learning_rate;
    const double         random_weight_range;
    const double         normalize_factor;

    /* non-cost part which must be copied by cloning */
    VectorN              weights;             /** TODO consider: weights and experience can be properties of the expert, and predictor is external and only uses them*/
    std::vector<VectorN> experience; // replay buffer
    double               prediction_error;

    friend class Predictor_Graphics;
};

#endif // PREDICTOR_H
