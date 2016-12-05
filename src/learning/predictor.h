#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <memory>
#include <common/modules.h>
#include <common/vector_n.h>
#include <common/log_messages.h>
#include <control/sensorspace.h>

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

    Predictor_Base(const Predictor_Base& other) = delete;
    Predictor_Base& operator=(const Predictor_Base& other) = delete;

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

    virtual ~Predictor_Base() = default;

    virtual void copy(Predictor_Base const& other) = 0;
    virtual double predict(void) = 0;
    virtual void   adapt  (void) = 0;
    /** TODO move as much as possible to the base class
     *  e.g. consider: weights and experience
     *  as components of the base class?
     * write the motor predictor in parallel
     */

    virtual void initialize_randomized(void) = 0;
    virtual void initialize_from_input(void) = 0;
    virtual VectorN const&  get_weights(void) const = 0;
    virtual std::vector<VectorN> const& get_experience(void) const = 0;
};


typedef std::unique_ptr<Predictor_Base> Predictor_ptr;


class Predictor : public Predictor_Base {

    Predictor(const Predictor& other) = delete;
    Predictor& operator=(const Predictor& other) = delete;

public:

    Predictor( const sensor_vector& input
             , const double         learning_rate
             , const double         random_weight_range
             , const std::size_t    experience_size = 1 );


    void copy(Predictor_Base const& other) override;

                VectorN  const& get_weights   (void) const override { return weights;    }
    std::vector<VectorN> const& get_experience(void) const override { return experience; }

    double predict(void) override;
    void   adapt  (void) override;

    void initialize_randomized(void) override;
    void initialize_from_input(void) override;

private:

    void learn_from_input_sample(void);
    void learn_from_experience(std::size_t skip_idx);

    /* non-cost part which must be copied by cloning */
    VectorN              weights;
    std::vector<VectorN> experience; // replay buffer

    friend class Predictor_Graphics;
};



class Motor_Predictor : public Predictor_Base {
public:
    Motor_Predictor( const sensor_vector& input
                   , const double learning_rate
                   , const double random_weight_range )
    : Predictor_Base(input, learning_rate, random_weight_range)
    {
        dbg_msg("Creating motor predictor.");
    }
};

#endif // PREDICTOR_H
