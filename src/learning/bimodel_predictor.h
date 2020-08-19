#ifndef BIMODEL_PREDICTOR_H
#define BIMODEL_PREDICTOR_H

#include <control/sensorspace.h>
#include <learning/predictor.h>
#include <learning/forward_inverse_model.hpp>

namespace learning {

class BiModel_Predictor : public Predictor_Base
{
    typedef NeuralModel<learning::TanhTransfer<>> NeuralModel_t;
    typedef learning::BidirectionalModel<NeuralModel_t,NeuralModel_t> BidirectionalModel_t;

    BidirectionalModel_t mod;

    /* inputs from external models */
    sensor_input_interface const& ctrl_context; // base to make prediction from
    model::vector_t& gradient; // vector to connect ext. back-propagation error information

    BiModel_Predictor(const BiModel_Predictor& other) = delete;
    BiModel_Predictor& operator=(const BiModel_Predictor& other) = delete;

public:

    BiModel_Predictor( sensor_input_interface const& input
                     , sensor_input_interface const& ctrl_context
                     , model::vector_t& gradient
                     , double learning_rate
                     , double random_weight_range
                     )
    : Predictor_Base(input, learning_rate, random_weight_range, /*experience=*/1)
    , mod(ctrl_context.size(), input.size(), random_weight_range)
    , ctrl_context(ctrl_context)
    , gradient(gradient)
    {
        assert(gradient.size() == ctrl_context.size());
    }

    virtual ~BiModel_Predictor() = default;

    void copy(Predictor_Base const& other) override {
        Predictor_Base::operator=(other); // copy base members
        BiModel_Predictor const& rhs = dynamic_cast<BiModel_Predictor const&>(other);
        mod = rhs.mod;
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
        /*Note: experience buffer not randomized here. because it is not used */
    };

    void initialize_from_input(void) override { assert(false && "one shot learning not supported."); }

    void draw(void) const { assert(false && "not implemented yet."); }

    void save(std::string /*folder*/) { wrn_msg("FIXME: nothing saved yet."); /*TODO implement */ }
    void load(std::string /*folder*/) { wrn_msg("FIXME: nothing loaded yet.");/*TODO implement */ }

    double get_prediction_error(void) const { return mod.get_forward_error(); } //OK
    double get_reconstruction_error(void) const { return mod.get_inverse_error(); } //OK

    BidirectionalModel_t& get_model(void) { return mod; }

    model::vector_t get_gradient(void) const { return mod.get_backprop_gradient(); }


private:

    void learn_from_input_sample(void) override
    {
        assert( gradient.size() == ctrl_context.size() );
        for (unsigned i = 0; i < gradient.size(); ++i)
            gradient[i] += ctrl_context[i]; /* add the predictor's back-propagated
                                               error information to the training target */
        mod.adapt(gradient, input, learning_rate);
        std::fill(gradient.begin(), gradient.end(), 0); // clear, mark as used
    }

};

} /* namespace learning */

#endif /* BIMODEL_PREDICTOR_H */



