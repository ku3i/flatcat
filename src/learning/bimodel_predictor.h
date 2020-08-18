#ifndef BIMODEL_PREDICTOR_H
#define BIMODEL_PREDICTOR_H

#include <control/sensorspace.h>
#include <learning/predictor.h>

namespace learning {

class BiModel_Predictor : public Predictor_Base
{
    typedef NeuralModel<learning::TanhTransfer<>> NeuralModel_t;
    typedef learning::BidirectionalModel<NeuralModel_t,NeuralModel_t> BidirectionalModel_t;

    BidirectionalModel_t mod;
    sensor_input_interface const& ctrl_context; // find better name
    model::vector_t output; // needed?

    BiModel_Predictor(const BiModel_Predictor& other) = delete;
    BiModel_Predictor& operator=(const BiModel_Predictor& other) = delete;

public:

    BiModel_Predictor( sensor_input_interface const& input
                     , sensor_input_interface const& ctrl_context
                     , double learning_rate
                     , double random_weight_range
                     )
    : Predictor_Base(input, learning_rate, random_weight_range, /*experience=*/1)
    , mod(ctrl_context.size(), input.size(), random_weight_range)
    , ctrl_context(ctrl_context)
    , output(ctrl_context.size())
    {
        dbg_msg("Initialize BiModel Predictor.");
    }

    virtual ~BiModel_Predictor() = default;

    void copy(Predictor_Base const& other) override {
        Predictor_Base::operator=(other); // copy base members
        BiModel_Predictor const& rhs = dynamic_cast<BiModel_Predictor const&>(other);
        mod = rhs.mod;
        dbg_msg("Copying homeokinetic pred/ctrl weights.");
    };

    Predictor_Base::vector_t const& get_prediction    (void) const override { return mod.get_forward_result(); }
    Predictor_Base::vector_t const& get_reconstruction(void) const          { return mod.get_inverse_result(); }

    double predict(void) override {
        mod.propagate_forward(ctrl_context);
        mod.propagate_inverse(input);

        return calculate_prediction_error();
    };

    double verify(void) override {
        mod.propagate_forward(ctrl_context);
        return calculate_prediction_error();
    }

    void initialize_randomized(void) override {
        mod.randomize_weights(random_weight_range);
        prediction_error = predictor_constants::error_min;
        /*Note: experience buffer not randomized here. not used */
    };

    void initialize_from_input(void) override { assert(false && "one shot learning not supported."); }

    void draw(void) const { assert(false && "not implemented yet."); }

    void save(std::string /*folder*/) { wrn_msg("FIXME: nothing saved yet."); /*TODO implement */ }
    void load(std::string /*folder*/) { wrn_msg("FIXME: nothing loaded yet.");/*TODO implement */ }

    double get_prediction_error(void) const { return mod.get_forward_error(); } //OK
    double get_reconstruction_error(void) const { return mod.get_inverse_error(); } //OK

    BidirectionalModel_t& get_model(void) { return mod; } // TODO remove

private:

    void learn_from_input_sample(void) override {
        mod.adapt(ctrl_context, input, learning_rate);
    }



};

} /* namespace learning */

#endif /* BIMODEL_PREDICTOR_H */



