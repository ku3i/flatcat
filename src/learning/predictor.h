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


class Predictor_Base {
protected:
    /* constants */
    const sensor_vector& input;
    const double         learning_rate;
    const double         random_weight_range;
    const double         normalize_factor;

    /* non-const */
    double               prediction_error;

public:

    Predictor_Base( const sensor_vector& input
                  , const double         learning_rate
                  , const double         random_weight_range )
    : input(input)
    , learning_rate(learning_rate)
    , random_weight_range(random_weight_range)
    , normalize_factor( 1.0 / (sqrt(input.size() * 4)))
    , prediction_error(predictor_constants::error_min)
    { dbg_msg("Creating predictor base"); }

    /* getter */
    double get_prediction_error(void) const { return prediction_error; }

    virtual double predict(void) = 0;
    virtual void   adapt  (void) = 0;
    /** TODO move as much as possible to the base class
     *  e.g. consider: weights and experience
     *  as components of the base class?
     * write the motor predictor in parallel
     */

};



class Predictor : public Predictor_Base {

    Predictor(const Predictor& other) = delete; // non construction-copyable

public:

    Predictor( const sensor_vector& input
             , const double         learning_rate
             , const double         random_weight_range
             , const std::size_t    experience_size = 1 );

    Predictor(Predictor&& other) = default;       /** move to base? */
    Predictor& operator=(const Predictor& other); /** move to base? */


    VectorN const&  get_weights(void) const { return weights; } /** move to base? */

    double predict(void) override;
    void   adapt  (void) override;

    void initialize_randomized(void);/** move to base? */
    void initialize_from_input(void);/** move to base? */

private:

    void learn_from_input_sample(void);
    void learn_from_experience(std::size_t skip_idx);

    /* non-cost part which must be copied by cloning */
    VectorN              weights;
    std::vector<VectorN> experience; // replay buffer

    friend class Predictor_Graphics;
};

#endif // PREDICTOR_H
