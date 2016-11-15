#ifndef JOINTCONTROL_H_INCLUDED
#define JOINTCONTROL_H_INCLUDED

#include <vector>
#include <cassert>

#include <common/modules.h>
#include <common/config.h>

#include <control/controlparameter.h>

#include <robots/robot.h>
#include <robots/joint.h>

namespace control {

struct Minimal_Seed_t {
    double pgain;
    double damping;
    double motor_self;
};

Control_Parameter initialize_anyhow(bool is_symmetric, const Minimal_Seed_t params_pdm, const std::string& filename );

namespace constants {
    const double initial_bias = 0.1;
}

/** TODO: check for initialization problem in controller when using csl hold */

class Jointcontrol
{
public:
    Jointcontrol(robots::Robot_Interface& robot)
    : robot(robot) /* angle, velocity, motor output + xyz-acceleration + bias */
    , number_of_inputs(3 * robot.get_number_of_joints() + 3 * robot.get_number_of_accel_sensors() + 1)
    , number_of_params_sym(number_of_inputs * (robot.get_number_of_joints() - robot.get_number_of_symmetric_joints()))
    , number_of_params_asym(number_of_inputs * robot.get_number_of_joints())
    , get_joints(robot.get_joints())
    , set_joints(robot.set_joints())
    , get_accels(robot.get_accels())
    , set_accels(robot.set_accels())
    , weights(robot.get_number_of_joints(), std::vector<double>(number_of_inputs, 0.0))
    , X(number_of_inputs)
    , Y(number_of_inputs)
    , activation(robot.get_number_of_joints())
    , initial_bias(0.1)
    , symmetric_controller(false)
    , is_switched(false)
    {
        sts_msg("Creating joint controller.");
        if (robot.get_number_of_joints() < 1) err_msg(__FILE__, __LINE__, "No motor outputs.");
        if (0 == robot.get_number_of_accel_sensors()) wrn_msg("No use of acceleration sensors in controller.");

        sts_msg("Number of symmetric joints is %u.", robot.get_number_of_symmetric_joints());

        assert(weights   .size() == robot.get_number_of_joints());
        assert(weights[0].size() == number_of_inputs);

        sts_msg( "Created controller with: \n   %u inputs\n   %u outputs\n   %u symmetric params\n   %u asymmetric params."
               , number_of_inputs
               , robot.get_number_of_joints()
               , number_of_params_sym
               , number_of_params_asym);
        reset();
    }

    void loop(void);
    void reset(void);

    void switch_symmetric(bool switched) {
        if (not symmetric_controller)
            is_switched = switched;
        else wrn_msg("Switching symmetry does not have any effect on symmetrical controller weights.");
    }
    void switch_symmetric() { switch_symmetric(not is_switched); }

    void set_control_parameter(const Control_Parameter& controller)
    {
        if (controller.is_symmetric()) apply_symmetric_weights(controller.get_parameter());
        else apply_weights(controller.get_parameter());
        symmetric_controller = controller.is_symmetric();
    }

    void set_control_parameter(const std::vector<double>& params) {
        dbg_msg("Apply raw control parameter.");
        if (symmetric_controller) apply_symmetric_weights(params);
        else apply_weights(params);
    }

    double get_normalized_mechanical_power(void) const;

    /**TODO move outside this class if no members are used*/
    Control_Parameter get_initial_parameter(const Minimal_Seed_t& seed) const;
    Control_Parameter make_symmetric (const Control_Parameter& other) const;
    Control_Parameter make_asymmetric(const Control_Parameter& other) const;

    void print_parameter(void) const;

    std::size_t get_number_of_parameter          (void) const { return number_of_params_asym; }
    std::size_t get_number_of_symmetric_parameter(void) const { return number_of_params_sym;  }

private:

    void apply_symmetric_weights(const std::vector<double>& params);
    void apply_weights          (const std::vector<double>& params);

    robots::Robot_Interface&          robot;
    const std::size_t                 number_of_inputs;
    const std::size_t                 number_of_params_sym;
    const std::size_t                 number_of_params_asym;

    const robots::Jointvector_t&      get_joints;
          robots::Jointvector_t&      set_joints;

    const robots::Accelvector_t&      get_accels;
          robots::Accelvector_t&      set_accels;

    std::vector<std::vector<double> > weights;
    std::vector<double>               X, Y, activation;

    const double                      initial_bias;
    bool                              symmetric_controller;
    bool                              is_switched;

    friend class Jointcontrol_Graphics;
};


class Behavior_Switcher
{
public:
    Behavior_Switcher(const Control_Vector& parameter_set, Jointcontrol& control)
    : parameter_set(parameter_set)
    , control(control)
    , current_behavior(0)
    {
        dbg_msg("Creating Behavior Switcher.");
    }

    void next(void) {
        ++current_behavior;
        if (current_behavior >= parameter_set.get_number_of_sets())
            current_behavior = 0;
        dbg_msg("Current behavior: %u", current_behavior);
        control.set_control_parameter(parameter_set.get(current_behavior));

        control.switch_symmetric(parameter_set.get(current_behavior).is_mirrored());
    }

private:
    const Control_Vector& parameter_set;
    Jointcontrol&         control;
    std::size_t           current_behavior;
};

} // namespace control

#endif // JOINTCONTROL_H_INCLUDED
