#ifndef SIMLOID_GRAPHICS_H_INCLUDED
#define SIMLOID_GRAPHICS_H_INCLUDED

#include <robots/simloid.h>
#include <draw/draw.h>
#include <draw/axes.h>
#include <draw/plot2D.h>

/**TODO:
 * make position of that graphic changeable
 */

namespace robots {
namespace constants {
    const double bsize = 0.01;
}

class Simloid_Graphics : Graphics_Interface {
    const Simloid& simloid;
    axes           axis_position;
    plot2D         plot_position;

public:
    Simloid_Graphics(const Simloid& simloid)
    : simloid(simloid)
    , axis_position(2.0, 0.0, 0.0, 1.0, 1.0, 0, "pos xy")
    , plot_position(1000, axis_position, colors::magenta)
    {}

    /**TODO: extend the drawing area to -1,+1*/
    void draw_body_position(void) const
    {
        const Bodyvector_t& bodies = simloid.get_bodies();
        const Vector3& bodypos = simloid.get_avg_position();
        Vector3 bodyvel_absolute = simloid.get_avg_velocity();

        double bodyvel_forward = simloid.get_avg_velocity_forward();
        double bodyvel_left    = simloid.get_avg_velocity_left();

        glColor3f(1.0, 1.0, 1.0);
        for (auto const& b : bodies)
        {
            draw_rect( (b.position.x - bodypos.x)
                     , (b.position.y - bodypos.y)
                     , constants::bsize, constants::bsize);
        }

        /* velocity vector */
        glColor3f(1.0, 0.75, 0.0);
        bodyvel_absolute.clip(0.5);
        draw_line(0.0, 0.0, bodyvel_absolute.x, bodyvel_absolute.y);

        glColor3f(0.0, 1.0, 1.0);
        draw_line(0.0, 0.0, bodyvel_left, -bodyvel_forward);

        axis_position.draw();
        plot_position.draw();
    }

    void draw_body_rotation(void) const
    {
        const double rot_norm  = -simloid.get_avg_rotation();
        const double rot_inf   = simloid.get_avg_rotation_inf_ang();
        const double rot_speed = -clip(simloid.get_avg_rotational_speed(), 1.0)/2;

        const float sin_rot = sin(rot_norm)/2;
        const float cos_rot = cos(rot_norm)/2;

        /* compass */
        glColor3f(1.0, 0.0, 0.0);
        draw_line(0.0, 0.0,  sin_rot,  cos_rot); // north
        glColor3f(0.0, 1.0, 0.0);
        draw_line(0.0, 0.0, -sin_rot, -cos_rot); // south

        /* speed */
        glColor3f(1.0, 0.0, 1.0);
        draw_line( sin_rot
                 , cos_rot
                 , sin_rot + cos_rot * rot_speed
                 , cos_rot - sin_rot * rot_speed);// rot_speed

        glColor3f(1.0, 1.0, 1.0);
        glprintf(-.5, .5, 0.0, 0.05, "rot: %+1.2f (%+1.2f pi)", rot_norm/M_PI, rot_inf/M_PI);

    }

    void draw(const pref& p) const {
        glPushMatrix();
        glTranslatef(0.0, 2.0, 0.); /**TODO make dependent on pref p */

        glColor4f(1.0, 1.0, 1.0, 0.25);
        draw_rect(0.0, 0.0, 1.0, 1.0);

        draw_body_position();
        draw_body_rotation();
        glPopMatrix();
    }

    void execute_cycle(void) {
        const Vector3& bodypos = simloid.get_avg_position();
        plot_position.add_sample(bodypos.x, bodypos.y);
    }
};

} // namespace robots

#endif // SIMLOID_GRAPHICS_H_INCLUDED
