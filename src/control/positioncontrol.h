#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <common/log_messages.h>
#include <robots/robot.h>
#include <robots/joint.h>
#include <controller/pid_control.hpp>


namespace control {

/** A simple multi-joint PID position controller,
    based on the supreme lib PID controller  */

class position_control
{
    robots::Jointvector_t&            joints;
    std::vector<supreme::pid_control> pid;

    float joint_range; //TODO make a proper mapping
    // find general solution, inc. the limits of the joints

public:

    position_control(robots::Robot_Interface& robot, float joint_range = 1.f)
    : joints(robot.set_joints())
    , pid()
    , joint_range(joint_range)
    {
        for (unsigned i = 0; i < joints.size(); ++i)
        {
            //PID setup
            pid.emplace_back(i, 0.01f);// 100Hz
            auto& p = pid.back();
            p.set_pid(6.0,1,0); //tadpole
        }
    }

    template <typename Vector_t>
    void execute_cycle( Vector_t& vec, Vector_t const& ext = {}
                      , const bool enable_external_control = false
                      , const bool disable_internal_control = false )
    {
        assert(joints.size() <= vec.size());

        if (disable_internal_control)
            vec.assign(vec.size(),0);

        if (enable_external_control) { // inject external control inputs
            const std::size_t N = std::min(ext.size(), vec.size()); // match lengths
            for (unsigned i = 0; i < N; ++i)
                vec[i] += ext[i];
        }

        for (std::size_t i = 0; i < joints.size(); ++i) {
            pid[i].set_target_value(joint_range * vec[i]);
            joints[i].motor = pid[i].step(joints[i].s_ang);

            // This is an unruh controller, use if nothing else works
            // float err = vec[i]-joints[i].s_ang;
            // joints[i].motor = clip(1*err + joints[i].motor.get_backed(),1);
        }
    }

};

} /* namespace control */

#endif /* POSITION_CONTROL_H */



