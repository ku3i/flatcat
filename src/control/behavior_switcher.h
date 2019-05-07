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

    void random(void) {
        current_behavior = random_index(parameter_set.size());
        sts_msg("Switching to random behavior: %u", current_behavior);
        switch_behavior();
    }

    void next(void) {
        ++current_behavior;
        if (current_behavior >= parameter_set.size())
            current_behavior = 0;
        //sts_msg("Current behavior: %u", current_behavior);
        switch_behavior();
    }

    bool step(void) {
        if (triggered) {
            next();
            triggered = false;
            return true;
        }
        else return false;
    }

    void trigger(void) { triggered = true; }

    Control_Parameter const& get_current_behavior(void) const { return parameter_set.get(current_behavior); }

private:

    void switch_behavior(void) {
        control.set_control_parameter(parameter_set.get(current_behavior));

        //TODO is that still needed?
        control.switch_symmetric(parameter_set.get(current_behavior).is_mirrored());
    }

    const Control_Vector& parameter_set;
    Jointcontrol&         control;
    std::size_t           current_behavior;
    bool                  triggered = false;
};


} // namespace control

#endif // BEHAVIOR_SWITCHER_H_INCLUDED
