#ifndef HOMEOKINESIS_H
#define HOMEOKINESIS_H

#include <robots/robot.h>
#include <control/sensorspace.h>
#include <learning/forward_inverse_model.hpp>

namespace learning {

namespace homeokinetic_constants {
    const float learning_rate_pred = 0.005; //both values 0.001..0.01 is very nice;
    const float learning_rate_ctrl = 0.005;
}

class Homeokinetic_Control {
public:
    typedef NeuralModel<TanhTransfer<>> Forward_t;
    typedef BidirectionalModel<Forward_t,Forward_t> PredictionModel_t;
    typedef BidirectionalModel<Forward_t,Forward_t> ControllerModel_t;
    typedef model::vector_t Vector_t;

    Vector_t x0, x1, y0, y1;
    Vector_t X0, X1, Y0;

    ControllerModel_t ctrl;
    PredictionModel_t pred;

    float learning_rate_pred;
    float learning_rate_ctrl;

    struct Option_t {
        bool prediction_learning = true;
        bool controller_learning = true;
        bool verbose             = true;
    } option = {};


    Homeokinetic_Control( sensor_input_interface const& input
                        , std::size_t number_of_joints
                        , float init_weight_range
                        , unsigned context = 0
                        , float learning_rate_pred = homeokinetic_constants::learning_rate_pred
                        , float learning_rate_ctrl = homeokinetic_constants::learning_rate_ctrl
                        )
    : x0(input.size())
    , x1(input.size())
    , y0(number_of_joints + context)
    , y1(number_of_joints + context)
    , X0(input.size())
    , X1(input.size())
    , Y0(number_of_joints + context)
    , ctrl(/*in=*/x0.size(), /*out=*/y0.size(), init_weight_range)
    , pred(/*in=*/y0.size(), /*out=*/x1.size(), init_weight_range)
    , learning_rate_pred(learning_rate_pred)
    , learning_rate_ctrl(learning_rate_ctrl)
    {
        sts_msg("creating homeokinetic controller with\n%u joints and \n%u context neurons.", number_of_joints, context);
        read_sensor_data(x0, input);
        control();
    }


    void execute_cycle(sensor_input_interface const& input)
    {
        /*--------------------------------------------+
         | Notation:                                  |
         | lower case real sensor/motor data x, y     |
         | upper case prediction/reconstruction X, Y  |
         | time indices: x0 = x(t+0) = x(t)           |
         |               x1 = x(t+1)                  |
         +--------------------------------------------*/

        /* original order changed, because robot.update must be executed between steps 1. + 2. */

        /** 2.) Prediction */
        read_next_state(input);
        predict();

        /** 3.) Reconstruction */
        reconstruct();

        /** 4.) Adaption/Learning */
        adapt_prediction();
        adapt_controller();

        /** 0.) time-step border */
        backup_state();

        /** 1.) Control */
        control();

        if (option.verbose) {
            sts_add("pe=%+.3f, tle=%.3f", pred.get_forward_error(), ctrl.get_inverse_error());
            print_vector(y0,"y");
        }
    }


    /*----------------------------------+
     | time-step border:                |
     | copy sensory state x(t) = x(t+1) |
     +----------------------------------*/
    void backup_state(void) {
        x0 = x1;
    }

    /*------------------------------------+
     | create control command y(t+1) from |
     | current sensory state x(t)         |
     +------------------------------------*/
    void control(void) {
        y1 = ctrl.propagate_forward(x0);
    }

    /*------------------------------------+
     | read next state x(t+1) from inputs |
     +------------------------------------*/
    template <typename Input_t>
    void read_next_state(Input_t const& input) {
        y0 = y1;
        read_sensor_data(x1, input);
    }

    /*--------------------------------------------+
     | make prediction x^(t+1) from x(t) and y(t) |
     +--------------------------------------------*/
    void predict(void) {
        X1 = pred.propagate_forward(y0);  // make prediction
    }

    /*-------------------------------------------------------+
     | make reconstruction from real next state x(t+1)       |
     | to assumed motor commands y^(t),                      |
     | make reconstruction from assumed motor commands y^(t) |
     | to reconstructed sensor state x^(t)                   |
     +-------------------------------------------------------*/
    void reconstruct(void) {
        Y0 = pred.propagate_inverse(x1);
        /* X0 = */ ctrl.propagate_inverse(Y0);
    }

    /* 4.a) ------------------------------+
     | learn predictive model to map from |
     | current motor commands y(t)        |
     | to next sensor state x(t+1)        |
     +------------------------------------*/
    void adapt_prediction(void) {
        if (option.prediction_learning)
            pred.adapt(y0, x1, learning_rate_pred);
    }

    /* 4.b) ---------------------------------------------+
     | learn controller to map from x(t) to y^(t),       |
     | the INVERSE learns to reconstruct x(t) from y^(t) |
     +---------------------------------------------------*/
    void adapt_controller(void) {
        if (option.controller_learning)
            ctrl.adapt(x0, Y0, learning_rate_ctrl);
    }


    Vector_t const& get_curr_state(void) const { return x0; }
    Vector_t const& get_next_state(void) const { return x1; }
    Vector_t const& get_prediction(void) const { return X1; }

    Vector_t const& get_motor_data(void) const { return y1; }
    Vector_t      & set_motor_data(void)       { return y1; }

    void randomize_weights(double range) {
        pred.randomize_weights(range);
        ctrl.randomize_weights(range);
    }

    double get_timeloop_error      (void) const { return ctrl.get_inverse_error(); }
    double get_prediction_error    (void) const { return pred.get_forward_error(); }

    double get_control_error       (void) const { return ctrl.get_forward_error(); }
    double get_reconstruction_error(void) const { return pred.get_inverse_error(); }

private:

    // makes a copy of the sensor data
    template <typename Vector_t, typename Input_t>
    void read_sensor_data(Vector_t& vec, Input_t const& input) {
        assert(input.size() == vec.size());
        for (std::size_t i = 0; i < input.size(); ++i)
            vec[i] = input[i];
    }

    friend class Homeokinetic_Graphics;
};

} /* namespace learning */

#endif /* HOMEOKINESIS_H */
