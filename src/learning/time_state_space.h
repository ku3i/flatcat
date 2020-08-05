#ifndef TIME_STATE_SPACE_H
#define TIME_STATE_SPACE_H

#include <control/sensorspace.h>

/**TODO: provide an easy way to access the vector shifted 1 time-step before */

template <std::size_t NumTaps>
class Time_State_Space : public time_embedded_sensors<NumTaps> {
public:
    Time_State_Space(const robots::Robot_Interface& robot)
    : time_embedded_sensors<NumTaps>(robot.get_joints().size())
    {
        auto const& joints = robot.get_joints();
        auto const& accels = robot.get_accels();

        for (robots::Joint_Model const& j : joints)
            time_embedded_sensors<NumTaps>::sensors.emplace_back(j.name + "_ang", [&j](){ return j.s_ang; });

        for (robots::Joint_Model const& j : joints)
            time_embedded_sensors<NumTaps>::sensors.emplace_back(j.name + "_vel", [&j](){ return j.s_vel; });

        for (robots::Accel_Sensor const& a : accels) {
            time_embedded_sensors<NumTaps>::sensors.emplace_back("acc_x", [&a](){ return a.v.x; });
            time_embedded_sensors<NumTaps>::sensors.emplace_back("acc_y", [&a](){ return a.v.y; });
            time_embedded_sensors<NumTaps>::sensors.emplace_back("acc_z", [&a](){ return a.v.z; });
        }

        /*
        for (robots::Joint_Model const& j : joints)
            time_embedded_sensors<NumTaps>::sensors.emplace_back(j.name + "_mot", [&j](){ return j.motor.get(); });*/

        /**Added bias internally */
    }
};


#endif /* TIME_STATE_SPACE_H */

