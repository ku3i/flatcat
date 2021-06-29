#ifndef STATE_ACTION_PREDICTOR_H
#define STATE_ACTION_PREDICTOR_H

#include <control/sensorspace.h>
#include <learning/predictor.h>
#include <learning/autoencoder.h>

namespace learning {

class State_Action_Predictor : public Predictor_Base {

    State_Action_Predictor(const State_Action_Predictor& other) = delete;
    State_Action_Predictor& operator=(const State_Action_Predictor& other) = delete;

public:

    State_Action_Predictor( const time_embedded_sensors<16>& inputs
                          , const double         learning_rate
                          , const double         random_weight_range
                          , const std::size_t    experience_size
                          , const std::size_t    hidden_layer_size // not needed, replace with num joints, remove
                          )
    : Predictor_Base(inputs, learning_rate, random_weight_range, experience_size)
    , enc(inputs.size(), hidden_layer_size, random_weight_range )
    {
        //dbg_msg("Initialize State Action Predictor using Autoencoder and time-embedded inputs.");
    }

    virtual ~State_Action_Predictor() = default;

    void copy(Predictor_Base const& other) override {
        Predictor_Base::operator=(other); // copy base members
        State_Action_Predictor const& rhs = dynamic_cast<State_Action_Predictor const&>(other);
        enc = rhs.enc;
        dbg_msg("Copying state predictor weights.");
    };

    Predictor_Base::vector_t const& get_prediction(void) const override { return enc.get_outputs(); }

    double predict(void) override {
        enc.propagate(input);
        return calculate_prediction_error();
    };

    double verify(void) override {
        enc.propagate(input);
        return calculate_prediction_error();
    }

    void initialize_randomized(void) override {
        enc.randomize_weight_matrix(random_weight_range);
        auto initial_experience = input.get();
        for (auto& w: initial_experience)
            w += random_value(-random_weight_range, random_weight_range);
        experience.assign(experience.size(), initial_experience);
        prediction_error = predictor_constants::error_min;
    };

    void initialize_from_input(void) override { assert(false && "one shot learning not supported."); }

    void draw(void) const { assert(false); /*not implemented*/ }

    Autoencoder const& get_encoder(void) const { return enc; }

    vector_t const& get_weights(void) const override { assert(false); return dummy; /*not implemented*/ }
    vector_t      & set_weights(void)       override { assert(false); return dummy; /*not implemented*/ }

private:

    void learn_from_input_sample(void) override { enc.adapt(input, learning_rate); };
    void learn_from_experience(std::size_t /*skip_idx*/) override { assert(false && "Learning from experience is not implemented yet."); };

    Autoencoder enc;

    VectorN dummy = {}; // remove when implementing get_weights

    friend class Predictor_Graphics;
};

} /* namespace learning */

#endif /* State_Action_Predictor_H */

