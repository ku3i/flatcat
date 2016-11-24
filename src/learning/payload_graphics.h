#ifndef PAYLOAD_GRAPHICS_H_INCLUDED
#define PAYLOAD_GRAPHICS_H_INCLUDED

#include <common/static_vector.h>
#include <draw/color_table.h>
#include <draw/graphics.h>

#include <learning/gmes.h>
#include <learning/expert.h>
#include <learning/payload.h>
#include <learning/sarsa.h>

template <typename Expert_Vector_t>
class Payload_Graphics : public Graphics_Interface {

    const GMES<Expert_Vector_t>&          gmes;
    const GMES_Graphics<Expert_Vector_t>& gmes_graphics;
    const static_vector<State_Payload>&   payload;
    const SARSA&                          sarsa;
    const ColorTable                      table;

public:
    Payload_Graphics( const GMES<Expert_Vector_t>&          gmes
                    , const GMES_Graphics<Expert_Vector_t>& gmes_graphics
                    , const static_vector<State_Payload>&   payload
                    , const SARSA&                          sarsa )
    : gmes(gmes)
    , gmes_graphics(gmes_graphics)
    , payload(payload)
    , sarsa(sarsa)
    , table(3)
    {
        dbg_msg("Creating payload graphics");
    }

    void draw(const pref& p) const
    {
        glPushMatrix();
        glRotatef(p.y_angle, 1.f, 0.f, 0.f);
        glRotatef(p.x_angle, 0.f, 1.f, 0.f);

        for (std::size_t n = 0; n < gmes.get_max_number_of_experts(); ++n) {
            if (gmes.expert[n].does_exists()) {
                const Point& point = gmes_graphics.graph.get_position(n);

                unsigned int argmax_q = payload[n].policies[sarsa.get_current_policy()].get_argmax_q();
                const Color4& c = table.get_color(argmax_q);

                glColor3f(c.r, c.g, c.b);
                glprintf(point.x, point.y, point.z, 0.03, "%u", argmax_q);

                /** make that switchable */
            }
        }
        glEnd();
        glPopMatrix();
    }

};

#endif // PAYLOAD_GRAPHICS_H_INCLUDED
