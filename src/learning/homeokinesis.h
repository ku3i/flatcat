#ifndef HOMEOKINESIS_H
#define HOMEOKINESIS_H

#include <robots/robot.h>
#include <control/sensorspace.h>
#include <learning/forward_inverse_model.hpp>

namespace learning {

class Homeokinetic_Control {
public:
    typedef BidirectionalModel<TanhTransfer<>> Model_t;
    typedef model::vector_t Vector_t;

    // Note: do not use const members
    float learning_rate_ctrl = 0.01; //both 0.01 is very nice;
    float learning_rate_pred = 0.01;
    float joint_range = 0.45; // move to external controller

    Vector_t x0,x1,y0;
    Vector_t X0,Y0;
    twopart_vector<Vector_t> xy0, XY0;

    Model_t ctrl, pred;

    struct Option_t {
        bool external_control    = false;
        bool prediction_learning = true;
        bool controller_learning = true;
        bool motor_outputs       = true;
    } option = {};




    Homeokinetic_Control( sensor_input_interface const& input
                        , std::size_t number_of_joints
                        , float init_weight_range
                        , unsigned context = 0
                        )
    : x0(input.size())
    , x1(input.size())
    , y0(number_of_joints + context)
    , X0(input.size())
    , Y0(number_of_joints + context)
    , xy0(x0,y0)
    , XY0(X0,Y0)
    , ctrl(/*in=*/ x0.size(), /*out=*/y0.size(), init_weight_range)
    , pred(/*in=*/xy0.size(), /*out=*/x1.size(), init_weight_range)
    {

        read_sensor_data(x0, input);
        control();
    }

    /** start with setting up a separate experiment, where only one homeokinetic controller is active.
        Test things as:
            - if it works at all (I mean your idea with the bimodels) CHECK
            - best number of time-embedded steps                      works with 32
            - linear vs. non-linear controller.                       non-linear
            - x+y for prediction vs. y only                           x+x = XY

        local test with a damped so2 oscillator
        Test homeokinesis only with pendulum, crawler and tadpole, fourlegged (TADPOLE
    */

    /*
    template <typename Vector_t>
    void write_motor_data(robots::Robot_Interface& robot, Vector_t & vec, Vector_t const& ext_input) {
        auto& j = robot.set_joints();
        assert(j.size() <= vec.size());

        if (option.external_control) { // inject external control inputs
            const std::size_t N = std::min(ext_input.size(), vec.size());
            assert(N == 4);
            for (unsigned i = 0; i < N; ++i)
                vec[i] += ext_input[i];
        }

        for (std::size_t i = 0; i < j.size(); ++i) {
            j[i].motor.transfer();
            pid[i].set_target_value(joint_range*vec[i]);  // 0.4 because of the 90 deg range of tadpole and we dont want to get stuck in the limits
            j[i].motor = pid[i].step(j[i].s_ang); // find general solution, inc. the limits of the joints
        }
    }
    */



//    void execute_cycle( robots::Robot_Interface& robot
//                      , sensor_input_interface const& input
//                      , Vector_t const& ext_motor_input
//                      )
//    {
//        /*--------------------------------------------+
//         | Notation:                                  |
//         | lower case real sensor/motor data x, y     |
//         | upper case prediction/reconstruction X, Y  |
//         | time indices: x0 = x(t+0) = x(t)           |
//         |               x1 = x(t+1)                  |
//         +--------------------------------------------*/
//
//        /** 1.) Control */
//        control(input);
//
//        //remove these lines from execute_cycle... how to rearrange?
//
//        write_motor_data(robot, y0, ext_motor_input);  // write y(t) TODO make this a small external PID joint-controller
//        robot.execute_cycle(); // apply motor commands y(t) to robot and get next sensory state x(t+1)
//
//
//        /** 2.) Prediction */
//        predict(input);
//
//        /** 3.) Reconstruction */
//        reconstruct();
//
//        /** 4.) Adaption/Learning */
//        adapt_prediction();
//        adapt_controller();
//
//
//        sts_add("pe=%+.3f, tle=%.3f", pred.get_forward_error(), ctrl.get_inverse_error());
//        print_vector(y0,"y");
//    }


    void execute_cycle(sensor_input_interface const& input)
    {
        /*--------------------------------------------+
         | Notation:                                  |
         | lower case real sensor/motor data x, y     |
         | upper case prediction/reconstruction X, Y  |
         | time indices: x0 = x(t+0) = x(t)           |
         |               x1 = x(t+1)                  |
         +--------------------------------------------*/

        /* order changed, because robot.update must be executed between steps 1. + 2. */

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

        sts_add("pe=%+.3f, tle=%.3f", pred.get_forward_error(), ctrl.get_inverse_error());
        print_vector(y0,"y");
    }


    /*----------------------------------+
     | time-step border:                |
     | copy sensory state x(t) = x(t+1) |
     +----------------------------------*/
    void backup_state(void) { x0 = x1; }


    /*-----------------------------------+
     | create control command y(t) from  |
     | current sensory state x(t)        |
     +-----------------------------------*/
    void control(void) {
        y0 = ctrl.propagate_forward(x0);
    }

    /*------------------------------------+
     | read next state x(t+1) from inputs |
     +------------------------------------*/
    template <typename Input_t>
    void read_next_state(Input_t const& input) {
        read_sensor_data(x1, input);
    }

    /*--------------------------------------------+
     | make prediction x^(t+1) from x(t) and y(t) |
     +--------------------------------------------*/
    void predict(void) {
        /* X1 = */ pred.propagate_forward(xy0);  // make prediction
    }

    /*-------------------------------------------------------+
     | make reconstruction from real next state x(t+1)       |
     | to assumed motor commands y^(t), throw away x^(t)     |
     | make reconstruction from assumed motor commands y^(t) |
     | to reconstructed sensor state x^(t)                   |
     +-------------------------------------------------------*/
    void reconstruct(void) {
        XY0 = pred.propagate_inverse(x1); // X0 part of XY0 not used!
        /* X0 = */ ctrl.propagate_inverse(Y0);
    }

    /* 4.a) --------------------------------------+
     | learn predictive model to map from         |
     | current state x(t) and motor commands y(t) |
     | to next sensor state x(t+1)                |
     +--------------------------------------------*/
    void adapt_prediction(void) {
        if (option.prediction_learning)
            pred.adapt(xy0, x1, learning_rate_pred);
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

    Vector_t const& get_motor_data(void) const { return y0; }
    Vector_t      & set_motor_data(void)       { return y0; }

    void randomize_weights(double range) {
        pred.randomize_weights(range);
        ctrl.randomize_weights(range);
    }

private:

    // makes a copy of the sensor data
    template <typename Vector_t, typename Input_t>
    void read_sensor_data(Vector_t& vec, Input_t const& input) {
        assert(input.size() == vec.size());
        for (std::size_t i = 0; i < input.size(); ++i)
            vec[i] = input[i];
    }
};

} /* namespace learning */

#endif /* HOMEOKINESIS_H */
