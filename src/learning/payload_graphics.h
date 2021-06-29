#ifndef PAYLOAD_GRAPHICS_H_INCLUDED
#define PAYLOAD_GRAPHICS_H_INCLUDED

#include <common/static_vector.h>
#include <draw/color_table.h>
#include <draw/graphics.h>

#include <learning/gmes.h>
#include <learning/gmes_graphics.h>
#include <learning/expert.h>
#include <learning/payload.h>
#include <learning/sarsa.h>

class Payload_Graphics : public Graphics_Interface {

    const GMES&                         gmes;
    const GMES_Graphics&                gmes_graphics;
    const static_vector<State_Payload>& payload;
    const SARSA&                        sarsa;
    const ColorTable                    table;

public:
    Payload_Graphics( const GMES&                         gmes
                    , const GMES_Graphics&                gmes_graphics
                    , const static_vector<State_Payload>& payload
                    , const SARSA&                        sarsa )
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

#include <draw/graphics.h>
#include <draw/display.h>
#include <common/vector2.h>

class State_Payload_Graphics : public Graphics_Interface {

    typedef static_vector<State_Payload> Payload_Vector_t;

    const Payload_Vector_t& payloads;
    const Action_Module_Interface& actions;
    const std::size_t num_policies, num_states;

public:
    State_Payload_Graphics(const Payload_Vector_t& payloads, const Action_Module_Interface& actions)
    : payloads(payloads)
    , actions(actions)
    , num_policies(payloads[0].policies.size())
    , num_states(payloads.size())
    {
        dbg_msg("payload graphics for %u states and %u policies.", num_states, num_policies);

    }
    void draw(const pref& /*p*/) const {

        /**TODO draw user selected policy a little bigger */
        const float space = 1.8/num_policies;
        const float height = 0.6*space/num_states;
        const float offy = space * num_policies/2;
        for (std::size_t i = 0; i < num_policies; ++i)
            for (std::size_t s = 0; s < num_states; ++s) {
                draw::vec3( 0.0
                          , -(i*space) - s*height + offy
                          , height
                          , 2.0
                          , payloads[s].policies[i].qvalues
                          , actions.get_number_of_actions_available()
                          );
            }
    }

    void draw2(const pref& /*p*/, unsigned real_num_states, unsigned cur_policy, unsigned cur_state, unsigned cur_action, unsigned RL_sel_action ) const {

        const unsigned N = real_num_states;
        assert(real_num_states<=num_states);
        const unsigned K = actions.get_number_of_actions_available();

        // draw rectangles
        const float ds = 2./(float) N;
        const float da = 2./(float) K;
        const float size = 0.05;

        const float Ds = 1 - ds/2;
        const float Da = 1 - da/2;

        const unsigned n = cur_state; // current state
        const unsigned k = cur_action; // current action

        for (unsigned s = 0; s < N; ++s)
            draw_square(-1, Ds - s*ds, size);
        draw_fill_square(-1, Ds - n*ds, size);

        for (unsigned a = 0; a < K; ++a)
            draw_square(1, Da - a*da, size);
        draw_fill_square(1, Da - k*da, size);

        draw_square(1, Da - RL_sel_action*da, 1.5*size);

        unsigned p = cur_policy;

        for (unsigned s = 0; s < N; ++s)
        {
            auto const& pay = payloads[s].policies[p];
            const unsigned a_maxq = pay.get_argmax_q();
            const unsigned a_minq = pay.get_argmin_q();

            const float qmax = pay.qvalues[a_maxq];
            const float qmin = pay.qvalues[a_minq];



            const float d = clip(1.f/(qmax-qmin), 0.001, 1000);

            for (unsigned a = 0; a < K; ++a)
            {
                glLineWidth(a==a_maxq? 1.5f: 0.5f);
                float val = clip( (pay.qvalues[a] - qmin) * d,0,1);
                //val =clip(val,-1.f,1.f);
                //sts_msg("val=%1.5f", val);
                //assert(in_range(val, -0.1f,1.f));

                if (a == RL_sel_action and s == n)
                    set_color(colors::yellow);
                else //if (val>0)
                    set_color(colors::cyan, val);
                //else
                  //  set_color(colors::magenta, -val);
                draw_line(-1, Ds - s*ds, 1, Da - a*da);
            }
        }

    }
};

#endif // PAYLOAD_GRAPHICS_H_INCLUDED
