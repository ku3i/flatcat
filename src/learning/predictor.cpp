#include <learning/predictor.h>


    Predictor::Predictor( const sensor_vector& input
                        , const double         learning_rate
                        , const double         random_weight_range
                        , const std::size_t    experience_size )
    : input(input)
    , learning_rate(learning_rate)
    , random_weight_range(random_weight_range)
    , normalize_factor( 1.0 / (sqrt(input.size() * 4)))
    , weights(input.size())
    , experience(experience_size)
    , prediction_error()
    {
        //dbg_msg("Experience Replay: %s (%ul)", (experience_size > 1 ? "on" : "off"), experience_size);
        assert(in_range(input.size(),         1ul,  500ul));
        assert(in_range(experience_size,      1ul, 1000ul));
        assert(in_range(learning_rate,        0.0,   +1.0));
        assert(in_range(random_weight_range, -1.0,   +1.0));

        initialize_from_input();
        predict(); // initialize prediction error
    }


    /* initialize weights and experience with random values
     */
    void Predictor::initialize_randomized(void)
    {
        assert(weights.size() == input.size());
        for (std::size_t m = 0; m < input.size(); ++m)
            weights[m] = input[m] + random_value(-random_weight_range,
                                                 +random_weight_range);
        experience.assign(experience.size(), weights);
        prediction_error = predictor_constants::error_min;
    }


    /* initialize weights and experience from first input
     */
    void Predictor::initialize_from_input(void)
    {
        assert(weights.size() == input.size());
        weights = input.get();
        experience.assign(experience.size(), weights);
        prediction_error = predictor_constants::error_min;
    }


    /* copy assignment */
    Predictor& Predictor::operator=(const Predictor& other)
    {
        if (this != &other) {// avoid invalid self-assignment
            dbg_msg("Copying predictor");
            assert(weights   .size() == other.weights   .size());
            assert(experience.size() == other.experience.size());

            weights          = other.weights;
            experience       = other.experience;
            prediction_error = other.prediction_error;
        }
        return *this;
    }


    double Predictor::predict(void)
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


    void Predictor::adapt_with_experience_replay(void)
    {
        assert(input.size() == weights.size());

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



    void Predictor::adapt(void) {
        for (std::size_t m = 0; m < input.size(); ++m)
            weights[m] += learning_rate * (input[m] - weights[m]) / experience.size();
    }



    void Predictor::adapt_by_experience_replay(std::size_t skip_idx) {
        assert(experience.size() > 1);
        assert(experience[0].size() == weights.size());
        /* learn the list */
        for (std::size_t m = 0; m < weights.size(); ++m) {
            double delta = .0;
            for (std::size_t i = 0; i < experience.size(); ++i) {
                if (i != skip_idx)
                    delta += (experience[i][m] - weights[m]);
            }
            delta *= learning_rate / (experience.size() - 1);
            weights[m] += delta;
        }
    }

