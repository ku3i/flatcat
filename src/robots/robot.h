#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <robots/joint.h>
#include <robots/accel.h>

namespace robots {

/**
 * This robot interface should unify all types of robots and make them available
 * to other components as a pure structure of joints, i.e. angles, velocities and torques.
 * This shall ensure the independence of the individual segment lengths, masses or other
 * construction details. All components should prefer to use this interface instead of
 * using a certain robot class directly.
 */
class Robot_Interface {
public:
    virtual std::size_t get_number_of_joints           (void) const = 0;
    virtual std::size_t get_number_of_symmetric_joints (void) const = 0;
    virtual std::size_t get_number_of_accel_sensors    (void) const = 0;

    virtual const Jointvector_t& get_joints(void) const = 0;
    virtual       Jointvector_t& set_joints(void)       = 0;

    virtual const Accelvector_t& get_accels(void) const = 0;
    virtual       Accelvector_t& set_accels(void)       = 0;

    virtual ~Robot_Interface() {}

    virtual bool execute_cycle(void) = 0;

    virtual double get_normalized_mechanical_power(void) const = 0;

};

} // namespace robots


#endif // ROBOT_H_INCLUDED
