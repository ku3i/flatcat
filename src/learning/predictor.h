#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <common/basic.h>
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
public:
    Predictor( const sensor_vector& input
             , const double         learning_rate
             , const double         random_weight_range
             , const std::size_t    experience_size = 1 )
    : input(input)
    , learning_rate(learning_rate)
    , random_weight_range(random_weight_range)
    , normalize_factor( 1.0 / (sqrt(input.size() * 4)))
    , weights(input.size())
    , delta_w(input.size())
    , experience(experience_size)
    , prediction_error()
    {
        //dbg_msg("Experience Replay: %s (%ul)", (experience_size > 1 ? "on" : "off"), experience_size);
        assert(in_range(input.size(),         1ul,  500ul));
        assert(in_range(experience_size,      1ul, 1000ul));
        assert(in_range(learning_rate,        0.0,   +1.0));
        assert(in_range(random_weight_range, -1.0,   +1.0));

        init_weights();
        predict(); // initialize prediction error
    }

    double predict(void)
    {
        assert(input.size() == weights.size());

        /* sum of squared distances to input */
        double error = .0;
        for (std::size_t m = 0; m < input.size(); ++m)
            error += square(input[m] - weights[m]);

        /** The prediction error is being normalized by the number
         *  of weights/inputs and the max. input range [-1,+1] so
         *  that it is independent of the size of input space.
         *  Also it should be limited [0..1].
         */
        prediction_error = normalize_factor * sqrt(error);
        assert_in_range(prediction_error, predictor_constants::error_min, predictor_constants::error_max);
        return prediction_error;
    }

    double get_prediction_error(void) const { return prediction_error; }

    void adapt(void) {
        for (std::size_t m = 0; m < input.size(); ++m)
            weights[m] += learning_rate * (input[m] - weights[m]) / experience.size();
    }

    void adapt_with_experience_replay(void)
    {
        assert(input.size() == weights.size());
        assert(input.size() == delta_w.size());

        if (experience.size() == 1)
            adapt();
        else {

            /** create random index to skip an arbitrary
             *  sample and replaced it by current input */
            std::size_t rand_idx = random_index(experience.size());

            adapt_by_experience_replay(rand_idx); // adapt without new sample
            predict();                            // refresh prediction error
            adapt();                              // adapt to new sample

            /** Insert current input into random position of experience list.
             *  This must be done after adaptation to guarantee a positive learning progress */
            experience[rand_idx] = input.get();
        }
    }

    void init_random_weights(void)
    {
        for (std::size_t m = 0; m < input.size(); ++m)
            weights[m] = input[m] + random_value(-random_weight_range,
                                                 +random_weight_range);
        experience.assign(experience.size(), weights);
    }

    const VectorN& get_weights(void) const { return weights; }

    void copy_weights_from(const Predictor& other)
    {
        assert(weights   .size() == other.weights   .size());
        assert(experience.size() == other.experience.size());

        weights          = other.weights;
        experience       = other.experience;
        prediction_error = other.prediction_error;
    }

    void init_weights(void)
    {
        /** initialize weights experience replay from first input */
        assert(weights.size() == input.size());
        weights = input.get();
        experience.assign(experience.size(), weights);
        prediction_error = predictor_constants::error_min;
    }

private:

    void adapt_by_experience_replay(std::size_t skip_idx) {
        assert(experience.size() > 1);
        /* learn the list */
        for (std::size_t m = 0; m < delta_w.size(); ++m) {
            delta_w[m] = .0;
            for (std::size_t i = 0; i < experience.size(); ++i) {
                if (i != skip_idx)
                    delta_w[m] += (experience[i][m] - weights[m]);
            }
            delta_w[m] *= learning_rate / (experience.size() - 1);
        }
        /* apply weight changes */
        for (std::size_t m = 0; m < delta_w.size(); ++m)
            weights[m] += delta_w[m];
    }

    const sensor_vector& input;
    const double         learning_rate;
    const double         random_weight_range;
    const double         normalize_factor;

    /* non-cost part which must be copied by cloning */
    VectorN              weights;
    VectorN              delta_w;
    std::vector<VectorN> experience; // replay buffer
    double               prediction_error;

    friend class Predictor_Graphics;
};

#endif // PREDICTOR_H
