#ifndef FLATCAT_CONTROL_HPP
#define FLATCAT_CONTROL_HPP

#include <control/jointcontrol.h>
#include <control/controlmixer.h>
#include <control/control_vector.h>

#include <flatcat_robot.hpp>
#include <flatcat_settings.hpp>

#include <controller/csl_control.hpp>
#include <controller/pid_control.hpp>
#include <so2_controller.hpp>

namespace supreme {

typedef std::array<float, constants::num_joints> TargetPosition_t;

enum class ControlMode_t : uint8_t
{
    none,
    position,
    csl_hold,
    so2_osc,
    behavior,
    END_ControlMode_t
};

namespace constants {

 /* current default position for Hannah standing */
    const TargetPosition_t default_position =
    { 0.25, /* HEAD */
      0.25, /* BODY */
      0.25, /* TAIL */
    };

    const TargetPosition_t test_position0 =
    { 0.00, /* HEAD */
      0.00, /* BODY */
      0.00, /* TAIL */
    };

    const float Kp = 3.0;
    const float Ki = 0.01;
    const float Kd = 0.0;

    const std::array<const char*, (unsigned) ControlMode_t::END_ControlMode_t> mode_str = { "NONE", "POS", "HOLD", "SO2", "BEHV" };

} /* constants */

class FlatcatControl {
public:

    bool enabled = false;
    bool usr_pos = false;

    float amplitude = 0.f;
    float modulate  = 0.f;
    float inputgain = 0.f;

    unsigned parameter_id = 0;

    FlatcatRobot&             robot;
    //control::Jointcontrol     jointcontrol;
    //control::Control_Vector   parameter_set;  // for joint controller
    TargetPosition_t          usr_params;     // for testing and demo

    ControlMode_t             cur_mode, tar_mode;

    std::vector<csl_control>  csl_ctrl;
    std::vector<pid_control>  pid_ctrl;

    jcl::SO2_Controller       so2_ctrl;

    FlatcatControl(FlatcatRobot& robot, FlatcatSettings const& settings)
    : robot(robot)
    //, jointcontrol(robot)
    //, parameter_set(settings.max_number_of_gaits, settings.lib_folder)
    , usr_params()
    , cur_mode(ControlMode_t::none)
    , tar_mode(ControlMode_t::none)
    , csl_ctrl()
    , pid_ctrl()
    , so2_ctrl(robot, csl_ctrl, usr_params)
    {
        //parameter_set.add(control::get_initial_parameter(robot, {0.1,-0.4, 1.0}, true));
        //parameter_set.add(control::get_initial_parameter(robot, {0.0, -.5, 0.0}, true));
        //assert(parameter_set.size() == 2);

        //auto const& c0 = parameter_set.get(0);
        //auto const& c1 = parameter_set.get(1);

        //parameter_set.add(0.75*c0 + 0.25*c1); // no. 3
        //parameter_set.add(0.50*c0 + 0.50*c1); // no. 4
        //parameter_set.add(0.25*c0 + 0.75*c1); // no. 5

        //jointcontrol.set_control_parameter(parameter_set.get(0));

        //mixed_jointcontrol.set_control_parameter(0, parameter_set.get(1));
        //mixed_jointcontrol.set_control_parameter(1, parameter_set.get(0));

        for (unsigned i = 0; i < constants::num_joints; ++i)
        {
            /* configure CSLs */
            csl_ctrl.emplace_back(i, 0.01f /*100Hz*/);
            auto & c = csl_ctrl.back();
            auto const& j = robot.get_joints()[i];
            c.target_csl_mode = 1.0;
            c.target_csl_fb   = 1.0;
            c.limit_lo = j.limit_lo + 0.05;
            c.limit_hi = j.limit_hi - 0.05;

            c.update_mode();

            /* setup and configure PID controller */
            pid_ctrl.emplace_back(i, 0.01f /*100Hz*/);
            auto & p = pid_ctrl.back();
            p.set_pid(constants::Kp, constants::Ki, constants::Kd);
            //TODO: p.set_dead_band(0.02); //1%
            //TODO: p.set_pulse_mode_threshold(0.05);

            /* general */
			sts_msg("setting controller voltage and type for motor %u", i);
            auto & m = robot.set_motors()[i];
            m.set_voltage_limit(settings.voltage_limit);
            m.set_controller_type(supreme::sensorimotor::Controller_t::voltage);
        }
    }

   /* void set_control_parameter(unsigned idx) {
        if (idx < parameter_set.size())
            control.set_control_parameter(parameter_set.get(idx));
        else wrn_msg("Controller with index %u does not exist.", idx);
    }*/

    void position_control() {

        const float a = clip(modulate, 0.f, 1.f);

        if (!usr_pos) {
			//sts_msg("posenabled");
            for (auto& j : robot.set_joints())
                pid_ctrl.at(j.joint_id).set_target_value( (1.f - a) * constants::default_position.at(j.joint_id)
                                                         +       a  * constants::test_position0  .at(j.joint_id) );
        } else {
            for (auto const& j : robot.get_joints())
                pid_ctrl.at(j.joint_id).set_target_value( usr_params.at(j.joint_id) );
        }

        for (auto& j : robot.set_joints())
        {
            const float out = pid_ctrl.at(j.joint_id).step(j.s_ang);
            j.motor = enabled ? out : .0; // apply only if enabled
        }
    }

    void resetting_pid(void) { for (auto& p : pid_ctrl) p.reset(); }

    void csl_hold_mode(void)
    {
        for (auto& j : robot.set_joints())
        {
            auto &csl = csl_ctrl.at(j.joint_id);
            csl.target_csl_fb = 1.0;
            csl.gi_pos = 2.5;
            float out = csl.step(j.s_ang, usr_params.at(j.joint_id));
            j.motor = enabled ? out : .0; // apply only if enabled
        }
    }

    void csl_behavioral_mode(void) {
        for (auto& j : robot.set_joints())
        {
            auto &csl = csl_ctrl.at(j.joint_id);
            csl.target_csl_mode = clip(usr_params.at(j.joint_id),-1.f,+1.f);
            csl.target_csl_fb = 1.006;
            csl.gi_pos = 2.0;
            float out = csl.step(j.s_ang);
            j.motor = enabled ? out : .0; // apply only if enabled
        }
    }

    void resetting_csl(void) {
        for (auto const& j : robot.get_joints())
        {
            csl_ctrl.at(j.joint_id).reset(j.s_ang);
        }
    }


    void execute_cycle(void)
    {

        if (enabled)
        {
            /* update and reset if mode changes */
            if (cur_mode != tar_mode) {
                resetting_csl();
                resetting_pid();
                so2_ctrl.reset();
                cur_mode = tar_mode;
            }

            switch(cur_mode) {
                case ControlMode_t::csl_hold: csl_hold_mode();          break;
                case ControlMode_t::position: position_control();       break;
                case ControlMode_t::so2_osc : so2_ctrl.execute_cycle(); break;
                case ControlMode_t::behavior: csl_behavioral_mode();    break;
                case ControlMode_t::none    :
                default:                      robot.disable_motors();   break;
            }

        }
        else
            robot.disable_motors();
    }

};

} /* namespace supreme */

#endif /* HANNAH_CONTROL_HPP */
