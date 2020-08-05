#ifndef HOMEOKINESIS_H
#define HOMEOKINESIS_H

#include <control/sensorspace.h>
#include <learning/forward_inverse_model.hpp>

#include <controller/pid_control.hpp>
#include <controller/csl_control.hpp>

namespace learning {

class Homeokinetic_Control {

    robots::Robot_Interface& robot;
    sensor_input_interface const& input;

    //std::vector<supreme::pid_control>& pid;
    std::vector<supreme::csl_control>& csl;

public:
    typedef BidirectionalModel<TanhTransfer<>> Model_t;
    typedef model::vector_t Vector_t;

    double learning_rate_ctrl = 0.5;
    double learning_rate_pred = 0.1;

    Model_t ctrl, pred;

    Vector_t x0,x1,y0;
    Vector_t X0,Y0;
    twopart_vector<Vector_t> xy0, XY0;


    //bool control_enabled = false;

    Homeokinetic_Control( robots::Robot_Interface& robot
                        , sensor_input_interface const& input
                        , std::vector<supreme::csl_control>& csl
                        , double random_weight_range
                        )
    : robot(robot)
    , input(input)
    , csl(csl)
    , ctrl(/*in=*/input.size(), /*out=*/robot.get_joints().size(), random_weight_range)
    , pred(/*in=*/input.size()+robot.get_joints().size(), /*out=*/input.size(), random_weight_range)
    , x0(input.size())
    , x1(input.size())
    , y0(robot.get_joints().size())
    , X0(input.size())
    , Y0(robot.get_joints().size())
    , xy0(x0,y0)
    , XY0(X0,Y0)
    {

        read_sensor_data(x0);
    }

    /** start with setting up a separate experiment, where only one homeokinetic controller is active.
        Test things as:
            - if it works at all (I mean your idea with the bimodels)
            - best number of time-embedded steps
            - linear vs. non-linear controller.
            - x+y for prediction vs. y only

        local test with a damped so2 oscillator
        Test homeokinesis only with pendulum, crawler and tadpole, fourlegged
    */

    template <typename Vector_t>
    void write_motor_data(Vector_t const& vec) {
        auto& j = robot.set_joints();
        assert(j.size() == vec.size());
        for (std::size_t i = 0; i < j.size(); ++i) {
            //pid[i].set_target_value(vec[i]);
            //j[i].motor = pid[i].step(3*j[i].s_ang);

            //csl[i].target_csl_mode = 1.0; // contraction
            j[i].motor = csl[i].step(j[i].s_ang, 5*vec[i]);
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
        y0 = ctrl.propagate_forward(x0); // create control command y(t) from current sensory state x(t)

        write_motor_data(y0);  // write y(t)
        robot.execute_cycle(); // apply motor commands y(t) to robot and get next sensory state x(t+1)
        read_sensor_data(x1);  // read x(t+1)

        /** 2.) Prediction */
        /*Vector_t const& X1 = */pred.propagate_forward(xy0); // make prediction x^(t+1) from y(t), TODO use combined vector [y(t).. x(t)]
        // alternatively: X1 = pred.propagate_forward( [y0 + x0] );


        /** 3.) Reconstruction */
        XY0 = pred.propagate_inverse(x1); // make reconstruction from real next state x(t+1) to assumed motor commands y^(t)
        /*Vector_t const& X0 =*/ ctrl.propagate_inverse(Y0); // make reconstruction from assumed motor commands y^(t) to reconstructed sensor state x^(t)
        // alternatively: [Y0, R] = pred.propagate_inverse(x1); // throw away R ?

        /** 4.) Adaption/Learning */
        pred.adapt(xy0, x1, learning_rate_pred); // learn predictive model to map from motor command y(t) to next sensor state x(t+1)
        ctrl.adapt(x0, Y0, learning_rate_ctrl); // learn controller to map from x(t) to y^(t), the INVERSE learns to reconstruct x(t) from y^(t)
        // alternatively: pred.adapt( [y0 + x0] , x1);

        //NOT NEEDED, read sensor data in the beginning
        //x0 = x1; // time step, replace old with new sensor data
        /*sts_msg("pe=%+.5f, pe_=%+.5f, tle=%.5f, tle_=%.5f", pred.get_forward_error()
                                                          , pred.get_inverse_error()
                                                          , ctrl.get_inverse_error()
                                                          , ctrl.get_forward_error());*/
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
