#ifndef HOMEOKINESIS_H
#define HOMEOKINESIS_H

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

    Vector2& j0; // override motor signal
    Vector2& j1;


    Homeokinetic_Control( robots::Robot_Interface& robot
                        , sensor_input_interface const& input
                        , std::vector<supreme::pid_control>& pid
                        , double pred_init_range
                        , double ctrl_init_range
                        , Vector2& j0, Vector2& j1
                        , unsigned context = 0
                        )
    : robot(robot)
    , input(input)
    //, csl(csl)
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
    , j0(j0), j1(j1)
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

        vec.at(0) +=  j0.y;
        vec.at(1) +=  j1.y;
        vec.at(2) +=  j0.x;
        vec.at(3) += -j1.x;

        for (std::size_t i = 0; i < j.size(); ++i) {
            j[i].motor.transfer();
            pid[i].set_target_value(0.5*vec[i]);  // 0.5 because of the 90 deg range of tadpole
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
        pred.adapt(xy0, x1, learning_rate_pred); /*--------------------------------------------+
                                                  | learn predictive model to map from         |
                                                  | current state x(t) and motor commands y(t) |
                                                  | to next sensor state x(t+1)                |
                                                  +--------------------------------------------*/

        ctrl.adapt(x0, Y0, learning_rate_ctrl);  /*---------------------------------------------------+
                                                  | learn controller to map from x(t) to y^(t),       |
                                                  | the INVERSE learns to reconstruct x(t) from y^(t) |
                                                  +---------------------------------------------------*/

        sts_add("pe=%+.3f, tle=%.3f [", pred.get_forward_error(), ctrl.get_inverse_error());

        for (unsigned i = 0; i< y0.size(); ++i)
            sts_add("%+.3f", y0[i]);
        sts_msg("]");
        /**
            Frage: wenn zur Vorhersage von x(t+1) nur y(t) verwendet wird:
            Vorteile: einfacher zu rechnen
            Nachteile: state ist nicht vollständig, model hat kein memory.
        */
    }


    Vector_t const& get_curr_state() const { return x0; }
    Vector_t const& get_next_state() const { return x1; }
    Vector_t const& get_motor_data() const { return y0; }

};

} /* namespace learning */

#endif /* HOMEOKINESIS_H */
