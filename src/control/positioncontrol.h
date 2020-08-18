#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <common/log_messages.h>
#include <robots/robot.h>
#include <robots/joint.h>
#include <controller/pid_control.hpp>


namespace control {

/** A simple multi-joint PID position controller,
    based on the supreme lib PID controller  */

class position_control // <- rename this... to homeostatic controller ?
{
    robots::Jointvector_t& joints;

    struct JointCtrl_t {

        JointCtrl_t(uint8_t id) : pid(id, 0.01), enabled(true) {}
        supreme::pid_control pid;
        bool enabled;
        //TODO other?
    };

    std::vector<JointCtrl_t> ctrl;


    float joint_range; //TODO make a proper mapping
    // find general solution, inc. the limits of the joints

public:

    bool enable_unruh = false;


    position_control(robots::Robot_Interface& robot, float joint_range = 1.f)
    : joints(robot.set_joints())
    , ctrl()
    , joint_range(joint_range)
    {
        for (unsigned i = 0; i < joints.size(); ++i)
        {
            //PID setup
            ctrl.emplace_back(i);
            auto& c = ctrl.back();
            c.pid.set_pid(6.0,0,0); //tadpole
        }
    }

    template <typename ControlVector_t, typename ExternalControl_t>
    void execute_cycle( ControlVector_t& vec, ExternalControl_t const& ext = {}
                      , const bool enable_external_control = false
                      , const bool disable_internal_control = false )
    {
        assert(joints.size() <= vec.size());

        if (disable_internal_control)
            for (unsigned i = 0; i < vec.size(); ++i) vec[i] = 0;

        if (enable_external_control) { // inject external control inputs
            const std::size_t N = std::min(ext.size(), vec.size()); // match lengths
            for (unsigned i = 0; i < N; ++i)
                vec[i] += ext[i];
        }

        for (std::size_t i = 0; i < joints.size(); ++i) {
            ctrl[i].pid.set_target_value(joint_range * vec[i]);
            joints[i].motor = ctrl[i].enabled ? ctrl[i].pid.step(joints[i].s_ang) : 0.;

            // simulate simple temperature behavior
            const float a = 0.9995;
            joints[i].s_tmp = joints[i].s_tmp*a + (1-a)*joints[i].s_cur;

            if (joints[i].s_tmp > 1.5 && ctrl[i].enabled) {
                ctrl[i].enabled = false;
                sts_msg("motor %u OFF",i);
            }
            if (joints[i].s_tmp < 0.75 && !ctrl[i].enabled) {
                ctrl[i].enabled = true;
                sts_msg("motor %u REACTIVATED",i);
            }

            if (enable_unruh) {
            // This is an unruh controller, use if nothing else works
                float err = vec[i]-joints[i].s_ang;
                joints[i].motor += clip(1*err + joints[i].motor.get_backed(), 1);
                vec[i] = joints[i].s_ang;
            }
        }
    }

};

} /* namespace control */

#endif /* POSITION_CONTROL_H */



