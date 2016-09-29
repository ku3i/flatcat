
#include "pole.h"

namespace robots {

bool
pole::execute_cycle(void)
{
    /* apply action and update pole cart*/
    assert(joints.size() == 1);

    update_dynamics(joints[0].motor);

    joints[0].s_ang = wrap2(theta) / M_PI;
    joints[0].s_vel = tanh(theta_dot / (2*M_PI));
    joints[0].motor = force;
    assert(std::abs(joints[0].s_ang) <= 1.0);
    return true;
}

void
pole::reset_state(bool tilted)
{
    theta_dot = 0.0;
    if (tilted)
        theta = M_PI + random_value( -pole_constants::five_deg
                                   , +pole_constants::five_deg);
    else
        theta = M_PI;
}


/*-----------------------------------------------------------------------*
 * Pole: Takes an action [-1,+1] and the current values of the two state *
 * variables theta and theta_dot and updates their values by estimating  *
 * the state dt seconds later, using Explicit Euler's Method.            *
 *-----------------------------------------------------------------------*/
void
pole::update_dynamics(const double action)
{
    assert(std::abs(action) <= 1.0);
    force = pole_constants::force_mag * clip(action, 1.0);

    for (std::size_t t = 0; t < 10; ++t)
    {
        double theta_dotdot = -0.2 * sign(theta_dot)                                          /* dry friction                  */
                            - pole_constants::friction * theta_dot                            /* fluid friction                */
                            - (pole_constants::gravity / pole_constants::length) * sin(theta) /* torque induced by gravity     */
                            + force * (pole_constants::length / pole_constants::mass);        /* torque induced by motor force */

        /* Update using Euler's method */
        theta     += theta_dot    * pole_constants::dt;
        theta_dot += theta_dotdot * pole_constants::dt;
    }
}

bool
pole::top(double top_range) const
{
    /* [-1,+1] is top, 0 is bottom */
    assert_in_range(top_range, 0.0, 0.5);
    const double angle = wrap2(theta) / M_PI;

    return std::abs(angle) > (1.0 - top_range);
}

void
pole::draw(const float pos_x, const float pos_y, const float size) const
{
    const double s = size/pole_constants::max_x;

    double length = clip(force)*0.1;

    if (force < 0) glColor4f(1.0, .5, 0.0, 0.7);
    else glColor4f(0.5, 0.0, 1.0, 0.7);
    draw_fill_rect(pos_x+length/2, pos_y, std::abs(length), 0.01);

    glColor3f(1.0, 1.0, 1.0);
    glprintf(pos_x, pos_y + 0.1 * s, 0.0, 0.025, "F=%+1.2f", force / pole_constants::force_mag);

    glLineWidth(2.0f);
    draw_line( pos_x
             , pos_y
             , 0.0
             , pos_x + 2 * s * pole_constants::length * sin(theta)
             , pos_y - 2 * s * pole_constants::length * cos(theta)
             , 0.0);
}

} // namespace robots
