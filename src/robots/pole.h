#ifndef POLE_H
#define POLE_H

#include <cmath>
#include <vector>
#include <cassert>
#include <common/modules.h>
#include <common/static_vector.h>
#include <common/vector_n.h>

#include <draw/draw.h>

#include <robots/joint.h>
#include <robots/accel.h>
#include <robots/robot.h>

namespace robots {

namespace pole_constants
{
    const double gravity   = 9.81;
    const double mass      = 0.1;
    const double length    = 0.5;   /* actually half the pole's length */

    const double force_mag = 2.0;
    const double dt        = 0.001; /* step size */
    const double max_x     = 10.0;  /* meters */
    const double friction  = 0.5;

    const double five_deg  = 0.087263889;
}

class pole : public Robot_Interface
{
private:
    double theta;       /* pole angle [rad] */
    double theta_dot;   /* pole angular velocity [rad/s] */
    double force;       /* force exerted to hinge joint */

    Jointvector_t joints;

    void reset_state(bool tilted = false);
    void update_dynamics(const double action);

    Accelvector_t accels;

public:
    pole(bool tilted = false)
    : theta()
    , theta_dot()
    , force()
    , joints()
    , accels() //empty
    {
        joints.reserve(1);
        joints.emplace_back();
        assert(joints.size() == 1);
        reset_state(tilted);
    }

    bool execute_cycle(void);

    void draw(const float pos_x, const float pos_y, const float size) const; //TODO make independent of pos and size
    bool top(double range = 0.01) const;
    double height(void) const { return -cos(theta); }

    /* implement the robot interface */
    std::size_t get_number_of_joints          (void) const { return 1; }
    std::size_t get_number_of_symmetric_joints(void) const { return 0; }
    std::size_t get_number_of_accel_sensors   (void) const { return 0; }

    const Jointvector_t& get_joints(void) const { return joints; }
          Jointvector_t& set_joints(void)       { return joints; }

    const Accelvector_t& get_accels(void) const { return accels; }
          Accelvector_t& set_accels(void)       { return accels; }

    double get_normalized_mechanical_power(void) const { return force * force; };
};

} // namespace robots
#endif /* POLE_H */

