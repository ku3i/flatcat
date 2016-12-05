#ifndef PREDICTOR_GRAPHICS_H_INCLUDED
#define PREDICTOR_GRAPHICS_H_INCLUDED

#include <common/modules.h>
#include <draw/graphics.h>
#include <basic/color.h>
#include <learning/predictor.h>

class Predictor_Graphics : public Graphics_Interface
{
    const Predictor_Base& predictor;
    const Color4     color;

public:
    Predictor_Graphics(const Predictor_Base& predictor)
    : predictor(predictor)
    , color(random_value(0.2, 1.0), random_value(0.2, 1.0), random_value(0.2, 1.0), 0.2)
    {
        assert(predictor.get_experience()[0].size() == 3);
    }

    void draw(const pref& p) const
    {
        std::vector<VectorN> const& experience = predictor.get_experience();
        if (experience.size() > 1) {
            set_color(color);
            for (std::size_t i = 0; i < experience.size(); ++i)
                draw_solid_cube( experience[i][0]
                               , experience[i][1]
                               , experience[i][2]
                               , 0.02 );
        }
    }
};

#endif // PREDICTOR_GRAPHICS_H_INCLUDED
