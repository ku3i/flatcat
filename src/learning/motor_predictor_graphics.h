#ifndef MOTOR_PREDICTOR_GRAPHICS_H_INCLUDED
#define MOTOR_PREDICTOR_GRAPHICS_H_INCLUDED

#include <common/modules.h>
#include <draw/graphics.h>
#include <draw/display.h>
#include <basic/color.h>
#include <learning/motor_predictor.h>

namespace learning {

class Motor_Predictor_Graphics : public Graphics_Interface
{
    const Motor_Predictor& predictor;
    const Color4           color;

public:
    Motor_Predictor_Graphics(const Motor_Predictor& predictor, Color4 const& color = colors::white)
    : predictor(predictor)
    , color(color)
    {
    }

    void draw(const pref&) const {
        float s = 2.0/predictor.core.weights.size();
        unsigned i = 0;
        for (auto const& wi : predictor.core.weights)
            draw_vector2(0.0 + s*i++, 0.0, 0.045, s, wi, 3.0);
    }

};

}

#endif /* MOTOR_PREDICTOR_GRAPHICS_H_INCLUDED */
