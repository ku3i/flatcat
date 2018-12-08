/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | July 2017                       |
 +---------------------------------*/

#ifndef SIMLOID_LOG_H_INCLUDED
#define SIMLOID_LOG_H_INCLUDED

#include <sstream>
#include <robots/simloid.h>

namespace robots {

class Simloid_Log : public Loggable<2048> {

    const Simloid& simloid;

public:
    Simloid_Log(const Simloid& simloid) : simloid(simloid) {}

    const char* log()
    {
        for (auto const& j : simloid.get_joints())
            append("%+e %+e %+e ", j.s_ang, j.s_vel, j.motor.get());

        for (auto const& a : simloid.get_accels())
            append("%+e %+e %+e ", a.v.x, a.v.y, a.v.z);

        append( "%+e %+e %+e %+e %+e %+e %+e %+e %+e %+e %+e"
              , simloid.get_avg_rotation_inf_ang()
              , simloid.get_avg_rotational_speed()
              , simloid.get_avg_velocity_forward()
              , simloid.get_avg_velocity_left()
              , simloid.get_normalized_mechanical_power()
            /** global coordinates */
              , simloid.get_avg_position().x
              , simloid.get_avg_position().y
              , simloid.get_avg_position().z
              , simloid.get_avg_velocity().x
              , simloid.get_avg_velocity().y
              , simloid.get_avg_velocity().z
              );

        return done();
    }
};

} // namespace robots

#endif // SIMLOID_LOG_H_INCLUDED
