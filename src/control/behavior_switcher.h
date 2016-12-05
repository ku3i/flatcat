#ifndef BEHAVIOR_SWITCHER_H_INCLUDED
#define BEHAVIOR_SWITCHER_H_INCLUDED

#include <control/controlparameter.h>
#include <control/control_vector.h>
#include <control/jointcontrol.h>

namespace control {


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
        if (current_behavior >= parameter_set.size())
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

#endif // BEHAVIOR_SWITCHER_H_INCLUDED
