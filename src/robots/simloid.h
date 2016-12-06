#ifndef SIMLOID_H
#define SIMLOID_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <float.h>
#include <assert.h>
#include <algorithm>

#include <common/lock.h>
#include <common/modules.h>
#include <common/socket_client.h>
#include <common/basic.h>
#include <common/log_messages.h>
#include <common/robot_conf.h>
#include <robots/robot.h>
#include <robots/joint.h>
#include <basic/vector3.h>

namespace robots {

/**TODO think about how get simloid thread save:
 * analyze which resources are accessed simultaneously and lock only them instead of locking every method.*/

class Simloid : public Robot_Interface
{
public:
    const unsigned short port;
    const unsigned int robot_ID;
    const unsigned int scene_ID;
    const bool visuals;
    const bool realtime;

private:
    pid_t child_pid;
    common::mutex_t mtx;

    Socket_Client client;
    bool connection_established;
    Robot_Configuration configuration;

    double timestamp;
    std::vector<Vector3> body_position0;

    Vector3 average_position;
    Vector3 average_position0;
    Vector3 average_velocity;
    double  average_rotation;
    double  avg_rot_inf_ang;
    double  avg_rot_inf_ang_last;

    double  avg_velocity_forward;
    double  avg_velocity_left;

    bool open_connection(void);
    void close_connection(void);
    void simulation_idle(double sec);
    void set_robot_to_default_position(void);
    void init_robot(void);
    void read_robot_configuration(void);
    void read_sensor_data(void);
    void write_motor_data(void);
    void reset(void);
    void update_avg_position(void);
    void update_avg_velocity(void);
    void update_rotation_z(void);
    void update_robot_velocity(void);

public:
    Simloid(const unsigned short port, const unsigned int robot_ID, const unsigned int scene_ID, const bool visuals, const bool realtime = true);
    ~Simloid(void);

    bool update(void); //locking
    bool is_connected(void) const { return connection_established; }
    void restore_state(void); //locking
    void save_state(void);
    void finish(void);

    void set_force(unsigned int body_index, const Vector3& force) {
        assert(body_index < configuration.number_of_bodies);
        configuration.bodies[body_index].force = force;
    }
    void reset_all_forces(void) {
        for (std::size_t i = 0; i < configuration.bodies.size(); ++i)
            configuration.bodies[i].force.zero();
    }

    /* implements the robot interface */
    bool execute_cycle(void) { return update(); }

    std::size_t get_number_of_joints          (void) const { return configuration.number_of_joints;                 }
    std::size_t get_number_of_symmetric_joints(void) const { return configuration.get_number_of_symmetric_joints(); }
    std::size_t get_number_of_accel_sensors   (void) const { return configuration.number_of_accelsensors;           }
    std::size_t get_number_of_bodies          (void) const { return configuration.number_of_bodies;                 }

    const Jointvector_t& get_joints(void) const { return configuration.joint; }
          Jointvector_t& set_joints(void)       { return configuration.joint; }

    const Accelvector_t& get_accels(void) const { return configuration.s_acc; }
          Accelvector_t& set_accels(void)       { return configuration.s_acc; }

    const Bodyvector_t& get_bodies(void) const { return configuration.bodies; }
          Bodyvector_t& get_bodies(void)       { return configuration.bodies; }

    Vector3 get_min_position(void) const;
    Vector3 get_max_position(void) const;

    const Vector3& get_avg_position(void) const { return average_position; }
    const Vector3& get_avg_velocity(void) const { return average_velocity; }
    double         get_avg_rotation(void) const { return average_rotation; }

    double get_avg_rotation_inf_ang(void) const { return avg_rot_inf_ang;  }
    double get_avg_rotational_speed(void) const { return (avg_rot_inf_ang - avg_rot_inf_ang_last)*100; }

    double get_avg_velocity_forward(void) const { return avg_velocity_forward; }
    double get_avg_velocity_left   (void) const { return avg_velocity_left;    }

    double         get_bodyheight0 (void) const { return body_position0[0].z; }

    double get_normalized_mechanical_power(void) const;

    bool motion_stopped(double thrsh) const; //TODO make use of this!
    bool dropped(double level = 0.5)  const;
    bool out_of_track_x(void)         const;
    bool out_of_track_y(void)         const;

};

} // namespace robots

#endif /* SIMLOID_H */
