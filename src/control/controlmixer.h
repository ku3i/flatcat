#ifndef CONTROLMIXER_H
#define CONTROLMIXER_H

#include <control/jointcontrol.h>
#include <robots/robot.h>

namespace control {

class Controlmixer {

std::vector<Jointcontrol> control;

public:
    Controlmixer(robots::Robot_Interface& robot, std::size_t num_controller)
    {
        for (std::size_t i = 0; i < num_controller; ++i)
            control.emplace_back(robot);
    }

    std::size_t size(void) const { return control.size(); }

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
