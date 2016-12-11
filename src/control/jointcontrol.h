#ifndef JOINTCONTROL_H_INCLUDED
#define JOINTCONTROL_H_INCLUDED

#include <vector>
#include <cassert>

#include <common/modules.h>
#include <common/config.h>
#include <common/log_messages.h>

#include <control/controlparameter.h>
#include <control/control_vector.h>
#include <control/control_core.h>

#include <robots/robot.h>
#include <robots/joint.h>

namespace control {

class Jointcontrol;

struct Minimal_Seed_t {
    double pgain;
    double damping;
    double motor_self;
};

std::size_t get_number_of_inputs(robots::Robot_Interface const& robot);
Control_Parameter get_initial_parameter(robots::Robot_Interface const& robot, const Minimal_Seed_t&    seed , bool symmetric);
Control_Parameter make_symmetric       (robots::Robot_Interface const& robot, const Control_Parameter& other);
Control_Parameter make_asymmetric      (robots::Robot_Interface const& robot, const Control_Parameter& other);

Control_Parameter initialize_anyhow    ( robots::Robot_Interface const& robot, Jointcontrol const& control
                                       , bool is_symmetric, const Minimal_Seed_t params_pdm, const std::string& filename);



/** TODO: check for initialization problem in controller when using csl hold */

class Jointcontrol
{
public:
    Jointcontrol(robots::Robot_Interface& robot);

    void loop(void);
    void reset(void);

    void switch_symmetric(bool switched);
    void switch_symmetric() { switch_symmetric(not is_switched); }

    void set_control_parameter(const Control_Parameter& controller);
    void set_control_parameter(const std::vector<double>& params);

    double get_normalized_mechanical_power(void) const;

    void print_parameter(void) const;

    std::size_t get_number_of_parameter          (void) const { return number_of_params_asym; }
    std::size_t get_number_of_symmetric_parameter(void) const { return number_of_params_sym;  }

private:

    void apply_symmetric_weights(const std::vector<double>& params);
    void apply_weights          (const std::vector<double>& params);
    void integrate_accels       (void);

    robots::Robot_Interface&          robot;
    Fully_Connected_Symmetric_Core    core;

    const std::size_t                 number_of_params_sym;
    const std::size_t                 number_of_params_asym;

    bool                              symmetric_controller;
    bool                              is_switched;

    friend class Jointcontrol_Graphics;
};


} // namespace control

#endif // JOINTCONTROL_H_INCLUDED
