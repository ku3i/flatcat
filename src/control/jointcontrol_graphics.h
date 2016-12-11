#ifndef JOINTCONTROL_GRAPHICS_H_INCLUDED
#define JOINTCONTROL_GRAPHICS_H_INCLUDED

#include <draw/draw.h>

#include <robots/robot.h>
#include <control/jointcontrol.h>

namespace control {

class Jointcontrol_Graphics : public Graphics_Interface
{
    const robots::Robot_Interface& robot;
    const control::Jointcontrol& control;

public:
    Jointcontrol_Graphics(const robots::Robot_Interface& robot, const control::Jointcontrol& control) : robot(robot), control(control) {}

    void draw(const pref& p) const
    {
        const double line_height = 0.02;
        const double row_width = 0.15;
        const double xstart = -1.0;
        const double ystart = +1.0;

        double xpos = xstart;
        double ypos = ystart;

        glColor3f(1.0,1.0,1.0);

        /*print header*/
        glprintc(xpos, ypos, 0.0, 0.5*line_height, "#");
        for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i) {
            if (robot.get_joints()[i].type == robots::Joint_Type_Normal) {
                xpos += row_width;
                glprintf(xpos, ypos, 0.0, 0.5*line_height, "%2lu_%s", i, robot.get_joints()[i].name.substr(0,16).c_str());
            }
        }
        for (std::size_t k = 0; k < control.core.num_inputs; ++k)
        {
            xpos = xstart;
            ypos -= line_height;

            glColor3f(1.0,1.0,1.0);
            glprintf(xpos, ypos, 0.0, line_height, "%2lu: ", k);
            for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
            {
                if (robot.get_joints()[i].type == robots::Joint_Type_Normal) {
                    const double w = control.core.weights[i][k];

                    if (w == 0.0) glColor3f(0.5, 0.5, 0.5);
                    else if (w > 0.0) glColor3f(1.0, 0.5, 0.5);
                    else glColor3f(0.5, 0.5, 1.0);

                    xpos += row_width;
                    glprintf(xpos, ypos, 0.0, line_height, "%+1.3f", control.core.weights[i][k]);
                }
            }

        }
    }
};

} // namespace control

#endif // JOINTCONTROL_GRAPHICS_H_INCLUDED
