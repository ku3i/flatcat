#ifndef HOMEOKINESIS_H
#define HOMEOKINESIS_H

#include <common/vector2.h>
#include <control/sensorspace.h>
#include <learning/forward_inverse_model.hpp>
#include <controller/pid_control.hpp>

namespace learning {

class Homeokinetic_Control {

    robots::Robot_Interface& robot;
    sensor_input_interface const& input;
    std::vector<supreme::pid_control>& pid;

public:
    typedef BidirectionalModel<TanhTransfer<>> Model_t;
    typedef model::vector_t Vector_t;

    double learning_rate_ctrl = 0.01;
    double learning_rate_pred = 0.01;

    //both 0.01 is very nice;

    Vector_t x0,x1,y0;
    Vector_t X0,Y0;
    twopart_vector<Vector_t> xy0, XY0;

    Model_t ctrl, pred;

    Vector_t const& ext_input; // ctrl inputs to override motor signal

    struct Option_t {
        bool external_control    = false;
        bool prediction_learning = true;
        bool controller_learning = true;
        bool motor_outputs       = true;
    } option = {};


    const float joint_range = 0.45;

    Homeokinetic_Control( robots::Robot_Interface& robot
                        , sensor_input_interface const& input
                        , std::vector<supreme::pid_control>& pid
                        , double pred_init_range
                        , double ctrl_init_range
                        , Vector_t const& ext_input
                        , unsigned context = 0
                        )
    : robot(robot)
    , input(input)
    , pid(pid)
    , x0(input.size())
    , x1(input.size())
    , y0(robot.get_joints().size()+context)
    , X0(input.size())
    , Y0(robot.get_joints().size()+context)
    , xy0(x0,y0)
    , XY0(X0,Y0)
    , ctrl(/*in=*/ x0.size(), /*out=*/y0.size(), ctrl_init_range)
    , pred(/*in=*/xy0.size(), /*out=*/x1.size(), pred_init_range)
    , ext_input(ext_input)
    {

        read_sensor_data(x0);
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

    template <typename Vector_t>
    void write_motor_data(Vector_t /*const*/& vec) {
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

    // makes a copy of the sensor data
    template <typename Vector_t>
    void read_sensor_data(Vector_t& vec) {
        assert(input.size() == vec.size());
        for (std::size_t i = 0; i < input.size(); ++i)
            vec[i] = input[i];
    }

    void execute_cycle(void)
    {
        /*--------------------------------------------+
         | Notation:                                  |
         | lower case real sensor/motor data x, y     |
         | upper case prediction/reconstruction X, Y  |
         | time indices: x0 = x(t+0) = x(t)           |
         |               x1 = x(t+1)                  |
         +--------------------------------------------*/

        /** 1.) Control */
        read_sensor_data(x0);
        y0 = ctrl.propagate_forward(x0); /*-----------------------------------+
                                          | create control command y(t) from  |
                                          | current sensory state x(t)        |
                                          +-----------------------------------*/

        //TODO how to enable/disable motor output?
        write_motor_data(y0);  // write y(t)
        robot.execute_cycle(); // apply motor commands y(t) to robot and get next sensory state x(t+1)
        read_sensor_data(x1);  // read x(t+1)

        /** 2.) Prediction */
        /* X1 = */pred.propagate_forward(xy0);   /*---------------------------------------------+
                                                  | make prediction x^(t+1) from x(t) and y(t), |
                                                  +---------------------------------------------*/

        /** 3.) Reconstruction */
        XY0 = pred.propagate_inverse(x1);        /*---------------------------------------------------+
                                                  | make reconstruction from real next state x(t+1)   |
                                                  | to assumed motor commands y^(t), throw away x^(t) |
                                                  +---------------------------------------------------*/

        /* X0 = */ ctrl.propagate_inverse(Y0);   /*-------------------------------------------------------+
                                                  | make reconstruction from assumed motor commands y^(t) |
                                                  | to reconstructed sensor state x^(t)                   |
                                                  +-------------------------------------------------------*/

        /** 4.) Adaption/Learning */

        if (option.prediction_learning)
            pred.adapt(xy0, x1, learning_rate_pred);
            /*--------------------------------------------+
             | learn predictive model to map from         |
             | current state x(t) and motor commands y(t) |
             | to next sensor state x(t+1)                |
             +--------------------------------------------*/

        if (option.controller_learning)
            ctrl.adapt(x0, Y0, learning_rate_ctrl);
            /*---------------------------------------------------+
             | learn controller to map from x(t) to y^(t),       |
             | the INVERSE learns to reconstruct x(t) from y^(t) |
             +---------------------------------------------------*/

        sts_add("pe=%+.3f, tle=%.3f", pred.get_forward_error(), ctrl.get_inverse_error());
        print_vector(y0,"y");
    }


    Vector_t const& get_curr_state() const { return x0; }
    Vector_t const& get_next_state() const { return x1; }
    Vector_t const& get_motor_data() const { return y0; }

};

} /* namespace learning */

#endif /* HOMEOKINESIS_H */
