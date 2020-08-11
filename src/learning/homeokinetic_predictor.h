#ifndef HOMEOKINETIC_PREDICTOR_H
#define HOMEOKINETIC_PREDICTOR_H

#include <control/sensorspace.h>
#include <learning/predictor.h>
#include <learning/homeokinesis.h>

namespace learning {

/*--------------------------------------------+
 | Homeokinetic Prediction and Control Core   |
 +--------------------------------------------*/

class Homeokinetic_Core : public Predictor_Base
{
    // settings
    unsigned context = 4;

    Homeokinetic_Control core;

    Homeokinetic_Core(const Homeokinetic_Core& other) = delete;
    Homeokinetic_Core& operator=(const Homeokinetic_Core& other) = delete;

public:

    Homeokinetic_Core( sensor_input_interface const& input
                     , std::size_t number_of_joints
                     , double learning_rate
                     , double random_weight_range
                     )
    : Predictor_Base(input, learning_rate, random_weight_range, 1)
    , core( input, number_of_joints, random_weight_range, context)
    {
        dbg_msg("Initialize Predictor using homeokinetic controller.");
    }

    virtual ~Homeokinetic_Core() = default;

    void copy(Predictor_Base const& other) override {
        Predictor_Base::operator=(other); // copy base members
        Homeokinetic_Core const& rhs = dynamic_cast<Homeokinetic_Core const&>(other);
        core = rhs.core;
        dbg_msg("Copying homeokinetic pred/ctrl weights.");
    };

    Predictor_Base::vector_t const& get_prediction(void) const override { return core.get_prediction(); }

    double predict(void) override {
        core.read_next_state(input); //TODO alt: consider to inject, actual motor commands
        core.predict();
        core.backup_state();
        core.control();
        return calculate_prediction_error();
    };

    double verify(void) override {
        core.predict();
        return calculate_prediction_error();
    }

    void initialize_randomized(void) override {
        core.randomize_weights(random_weight_range);
        prediction_error = predictor_constants::error_min;
        /*Note: experience buffer not randomized here. not used */
    };

    void initialize_from_input(void) override { assert(false && "one shot learning not supported."); }

    void draw(void) const { assert(false && "not implemented yet."); }

    void save(std::string /*folder*/) { wrn_msg("FIXME: nothing saved yet."); /*TODO implement */ }
    void load(std::string /*folder*/) { wrn_msg("FIXME: nothing loaded yet.");/*TODO implement */ }


    Homeokinetic_Control::Vector_t& set_motor_data(void) { return core.set_motor_data(); }

    double get_timeloop_error(void) const { return core.get_timeloop_error(); }
    double get_prediction_error(void) const { return core.get_prediction_error(); }

private:

    void learn_from_input_sample(void) override {
        core.reconstruct();
        core.adapt_prediction();
        core.adapt_controller();
    }

    void learn_from_experience(std::size_t /*skip_idx*/) override {
        assert(false && "Learning from experience is not implemented yet.");
    };



    //friend class Predictor_Graphics;
};

} /* namespace learning */

#endif /* HOMEOKINETIC_PREDICTOR_H */


