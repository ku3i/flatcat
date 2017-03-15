#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <memory>
#include <common/modules.h>
#include <common/vector_n.h>
#include <common/log_messages.h>
#include <common/static_vector.h>
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

class Predictor_Base;
typedef std::unique_ptr<Predictor_Base> Predictor_ptr;


class Predictor_Base {

    Predictor_Base(const Predictor_Base& other) = delete;

protected:
    typedef VectorN vector_t;

    double calculate_prediction_error();

    /* constants */
    const sensor_vector& input;
    const double         learning_rate;
    const double         random_weight_range;
    const double         normalize_factor;

    /* non-const */
    double               prediction_error;
    std::vector<VectorN> experience;       // replay buffer

    Predictor_Base& operator=(const Predictor_Base& other)
    {
        prediction_error = other.prediction_error;
        assert(experience.size() == other.experience.size());
        experience = other.experience;
        return *this;
    }

public:

    Predictor_Base( const sensor_vector& input
                  , const double         learning_rate
                  , const double         random_weight_range
                  , const std::size_t    experience_size )
    : input(input)
    , learning_rate(learning_rate)
    , random_weight_range(random_weight_range)
    , normalize_factor( 1.0 / (sqrt(input.size() * 4)))
    , prediction_error(predictor_constants::error_min)
    , experience(experience_size)
    {
        //dbg_msg("Experience Replay: %s (%ul)", (experience_size > 1 ? "on" : "off"), experience_size);
        //dbg_msg("Input dimension: %u", input.size());
        assert(in_range(input.size(),         1ul,  500ul));
        assert(in_range(experience_size,      1ul, 1000ul));
        assert(in_range(learning_rate,        0.0,   +1.0));
        assert(in_range(random_weight_range, -1.0,   +1.0));

        experience.assign(experience.size(), VectorN(input.size(), .0) ); // zero initialize experience anyhow
    }

    /* non-virtual */
    double get_prediction_error(void) const { return prediction_error; }
    std::vector<VectorN> const& get_experience(void) const { return experience; }
    void adapt(void);

    /* virtual */
    virtual ~Predictor_Base() = default;

    virtual void copy(Predictor_Base const& other) = 0;

    virtual double predict(void) = 0;

    virtual void initialize_randomized(void) = 0;
    virtual void initialize_from_input(void) = 0;

    virtual vector_t const& get_prediction(void) const = 0;

private:
    virtual void learn_from_input_sample(void) = 0;
    virtual void learn_from_experience(std::size_t skip_idx) = 0;
};


/** simple predictor */
class Predictor : public Predictor_Base {

    Predictor(const Predictor& other) = delete;
    Predictor& operator=(const Predictor& other) = delete;

public:

    Predictor( const sensor_vector& input
             , const double         learning_rate
             , const double         random_weight_range
             , const std::size_t    experience_size = 1 );


    virtual ~Predictor() = default;

    void copy(Predictor_Base const& other) override;

    Predictor_Base::vector_t const& get_prediction(void) const override { return weights; }

    double predict(void) override;

    void initialize_randomized(void) override;
    void initialize_from_input(void) override;

private:

    void learn_from_input_sample(void) override;
    void learn_from_experience(std::size_t skip_idx) override;

    VectorN weights;

    friend class Predictor_Graphics;
};



#endif // PREDICTOR_H
