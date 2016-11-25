#ifndef GMES_GRAPHICS_H_INCLUDED
#define GMES_GRAPHICS_H_INCLUDED

#include <draw/draw.h>
#include <draw/axes3D.h>
#include <draw/plot3D.h>
#include <draw/network3D.h>
#include <draw/graphics.h>
#include <draw/display.h>
#include <learning/gmes.h>
#include <learning/predictor_graphics.h>

class GMES_Graphics : public Graphics_Interface {
public:
    GMES_Graphics(const GMES& gmes, const sensor_vector& input)
    : gmes(gmes)
    , expert(gmes.expert)
    , input(input)
    , axis(.0, .0, .0, 1., 1., 1., 0)
    , plot(100, axis, LineColorMix0[0])
    , graph(expert.get_max_number_of_experts(), axis, white)
    , predictor_graphics()
    {
        predictor_graphics.reserve(expert.get_max_number_of_experts());

        int dsize = (int) ceil(sqrt(expert.get_max_number_of_experts()));
        for (unsigned int n = 0; n < expert.get_max_number_of_experts(); ++n)
        {
            graph.update_node(n,
                              -1.0 + 2.0/dsize*(n%dsize) + 1.0/dsize,
                              -1.0 + 2.0/dsize*(n/dsize) + 1.0/dsize,
                              -1.0,
                              gmes_constants::initial_learning_capacity);

            predictor_graphics.emplace_back(expert[n].get_predictor());
        }
        sts_msg("Created GMES Graphics Extension");
    }

    void execute_cycle(uint64_t cycle)
    {
        assert(input.size() == 3);
        plot.add_sample((float) input[0], (float) input[1], (float) input[2]);

        for (unsigned int n = 0; n < expert.get_max_number_of_experts(); ++n) {
            graph.update_edge(n, gmes.get_winner(), (unsigned char) 255 * expert[n].transition[gmes.get_winner()]);
            graph.update_edge(gmes.get_winner(), n, (unsigned char) 255 * expert[gmes.get_winner()].transition[n]);
        }

        graph.update_node(gmes.get_recipient(),
                          fmin(2.0, expert[gmes.get_recipient()].learning_capacity));

        graph.update_node(gmes.get_winner(),
                          expert[gmes.get_winner()].predictor.get_weights()[0],
                          expert[gmes.get_winner()].predictor.get_weights()[1],
                          expert[gmes.get_winner()].predictor.get_weights()[2],
                          fmin(2.0, expert[gmes.get_winner()].learning_capacity));

        graph.activated(gmes.get_winner());
        graph.special(gmes.get_to_insert());
    }

    void draw_experience(const pref& p) const {
        glPushMatrix();
        glRotatef(p.y_angle, 1.0, 0.0, 0.0);
        glRotatef(p.x_angle, 0.0, 1.0, 0.0);
        glScalef(0.5, 0.5, 0.5);
        for (std::size_t i = 0; i < gmes.get_max_number_of_experts(); ++i)
            if (expert[i].exists)
                predictor_graphics[i].draw(p);
        glPopMatrix();
    }

    void draw(const pref& p) const
    {
        draw_experience(p);

        glColor4f(1.0, 1.0, 1.0, 1.0);
        glprintf(0.8, -0.7, 0.0, 0.03, "%u/%u", gmes.get_number_of_experts(), gmes.get_max_number_of_experts());
        glprintf(0.8, -0.8, 0.0, 0.03, "%u"   , gmes.get_to_insert());

        glLineWidth(2.0f);
        glColor4f(1.0, 1.0, 1.0, 0.2);
        axis.draw(p.x_angle, p.y_angle);

        glColor4f(1.0, 1.0, 1.0, 1.0);
        plot.draw(p.x_angle, p.y_angle);
        graph.draw(p.x_angle, p.y_angle);

        draw_vector2(-1.0, 1.2, 0.1, 2.0, gmes.get_activations());
    }

    const GMES&          gmes;
    const Expert_Vector& expert;
    const sensor_vector& input;

    const axes3D         axis;
    plot3D               plot;
    network3D            graph;

    std::vector<Predictor_Graphics> predictor_graphics;
};

#endif // GMES_GRAPHICS_H_INCLUDED
