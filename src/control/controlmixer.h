#ifndef CONTROLMIXER_H
#define CONTROLMIXER_H

#include <control/jointcontrol.h>
#include <robots/robot.h>

namespace control {

class Controlmixer {

std::vector<Jointcontrol> control;

public:
    Controlmixer(robots::Robot_Interface& robot, std::size_t num_controller)
    : control()
    {
        for (std::size_t i = 0; i < num_controller; ++i)
            control.emplace_back(robot);
    }

    std::size_t size(void) const { return control.size(); }

    /* 'one-hot' switching */
    void set_active(std::size_t index) {
        for (std::size_t i = 0; i < control.size(); ++i)
            control[i].set_output_gain((index == i) ? 1. : 0.);
    }

    void fade(std::size_t i, std::size_t j, float val) {
        const float g = clip(val, 0.f, 1.f);
        control.at(i).set_output_gain(1.f - g); // active if g -> 0
        control.at(j).set_output_gain(      g); // active if g -> 1
    }

    void set_control_parameter(std::size_t index, const Control_Parameter& controller) {
        control.at(index).set_control_parameter(controller);
    }

          Jointcontrol& operator[] (std::size_t index)       { return control.at(index); }
    const Jointcontrol& operator[] (std::size_t index) const { return control.at(index); }

    void execute_cycle(void) { for (auto& c: control) c.execute_cycle(); }

    void reset(void) { for (auto& c: control) c.reset(); }

};

} /* namespace control */

#endif /* CONTROLMIXER_H */
