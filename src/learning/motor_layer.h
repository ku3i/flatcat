#ifndef MOTOR_LAYER_H_INCLUDED
#define MOTOR_LAYER_H_INCLUDED

#include <robots/robot.h>
#include <robots/joint.h>
#include <control/jointcontrol.h>
#include <control/control_vector.h>
#include <control/sensorspace.h>

/** WHAT DO WE NEED?

predictor_base
expert_base
gmes_base


robot
joint_control


*/

namespace learning {

namespace constants {
    double      local_learning_rate = 0.01;
    std::size_t experience_size     = 1;
}


class motor_space : public sensor_vector {
public:
    motor_space(const robots::Jointvector_t& joints)
    : sensor_vector(joints.size())
    {
        for (robots::Joint_Model const& j : joints)
            sensors.emplace_back(j.name, [&j](){ return j.motor; });
    }
};

/** set up motor space as "sensor_space&" for experts */

class Motor_Layer {
public:
    Motor_Layer( robots::Robot_Interface& robot, std::size_t max_num_motor_experts )
    : max_num_motor_experts(max_num_motor_experts)
    , control(robot)
    , params(max_num_motor_experts)
    //, experts(max_num_motor_experts, sensors, payloads, constants::local_learning_rate, constants::experience_size)
    //, gmes(experts, constants::gmes_learning_rate)
    {
        dbg_msg("Creating new competitive motor layer.");
    }



    std::size_t                 max_num_motor_experts;
    control::Jointcontrol       control;
    control::Control_Vector     params; /** implements static_vector_interface and can by used as payload for gmes*/


    //Expert_Vector               experts;
    //GMES                        gmes;

};



} // namespace learning


#endif // MOTOR_LAYER_H_INCLUDED

