#ifndef STATE_PREDICTOR_H_INCLUDED
#define STATE_PREDICTOR_H_INCLUDED

#include <learning/predictor.h>
#include <learning/autoencoder.h>

namespace learning {

namespace constants {
    const std::size_t default_hidden_size = 3;
}

class State_Predictor : public Predictor_Base {

    State_Predictor(const State_Predictor& other) = delete;
    State_Predictor& operator=(const State_Predictor& other) = delete;

public:

    State_Predictor( const sensor_vector& inputs
                   , const double         learning_rate
                   , const double         random_weight_range
                   , const std::size_t    experience_size = 1   /**TODO*/
                   , const std::size_t    hidden_layer_size = constants::default_hidden_size )
    : Predictor_Base(inputs, learning_rate, random_weight_range, experience_size)
    , enc(inputs.size(), hidden_layer_size, random_weight_range )
    {
        dbg_msg("Initialize State Predictor using Auto-encoder.");
    }

    virtual ~State_Predictor() = default;

    void copy(Predictor_Base const& other) override {
        Predictor_Base::operator=(other); // copy base members
        State_Predictor const& rhs = dynamic_cast<State_Predictor const&>(other);
        enc = rhs.enc;
    };

    Predictor_Base::vector_t const& get_prediction(void) const override { return enc.get_outputs(); }

    double predict(void) override {
        enc.propagate(input);
        return calculate_prediction_error();
    };

    void initialize_randomized(void) override {
        dbg_msg("Initialize randomized state predictor.");
        enc.randomize_weight_matrix(random_weight_range);
        auto initial_experience = input.get();
        for (auto& w: initial_experience)
            w += random_value(-random_weight_range, random_weight_range);
        experience.assign(experience.size(), initial_experience);
        prediction_error = predictor_constants::error_min;
    };

    void initialize_from_input(void) override { assert(false && "one shot learning not supported."); }

private:

    void learn_from_input_sample(void) override { enc.adapt(input, learning_rate); };
    void learn_from_experience(std::size_t skip_idx) override { assert(false && "Learning from experience is not implemented yet."); };

    Autoencoder enc;

    friend class Predictor_Graphics;
};

} // namespace learning

#endif // STATE_PREDICTOR_H_INCLUDED
