#ifndef PREDICTOR_GRAPHICS_H_INCLUDED
#define PREDICTOR_GRAPHICS_H_INCLUDED

#include <common/modules.h>
#include <draw/graphics.h>
#include <basic/color.h>
#include <learning/predictor.h>

class Predictor_Graphics : public Graphics_Interface
{
    const Predictor& predictor;
    const Color4     color;

public:
    Predictor_Graphics(const Predictor& predictor)
    : predictor(predictor)
    , color(random_value(0.2, 1.0), random_value(0.2, 1.0), random_value(0.2, 1.0), 0.2)
    {
        assert(predictor.experience[0].size() == 3);
    }

    void draw(const pref& p) const
    {
        if (predictor.experience.size() > 1) {
            set_color(color);
            for (std::size_t i = 0; i < predictor.experience.size(); ++i)
                draw_solid_cube( predictor.experience[i][0]
                               , predictor.experience[i][1]
                               , predictor.experience[i][2]
                               , 0.02 );
        }
    }
};

#endif // PREDICTOR_GRAPHICS_H_INCLUDED
