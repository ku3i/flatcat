#ifndef DISPLAY_H_INCLUDED
#define DISPLAY_H_INCLUDED

#include <vector>
#include <draw/draw.h>
#include <common/modules.h>
#include "../../simloidTCP/src/basic/color.h"

template <typename Element_t = double, typename Vector_t = std::vector<Element_t> > inline void
draw_vector( const double posx
           , const double posy
           , const double height
           , const double width
           , const Vector_t& vec
           , const Element_t& max_val = 1.0 )
{
    glColor4f(1.0, 1.0, 1.0, 1.0);
    const double wbar = width/vec.size();
    const double hbar = height/std::abs(max_val);
    for (std::size_t i = 0; i < vec.size(); ++i)
        draw_fill_rect(posx + i*wbar, posy, wbar, hbar*clip(vec[i], 0.0, max_val));
}

template <typename Element_t = double, typename Vector_t = std::vector<Element_t> > inline void
draw_vector2( const double posx
            , const double posy
            , const double height
            , const double width
            , const Vector_t& vec
            , const Element_t& max_val = 1.0 )
{
    //const double wbar = std::min(width/vec.size(), 0.05);
    const double wbar = width/vec.size();

    for (std::size_t i = 0; i < vec.size(); ++i) {

        if (vec[i] > max_val)
            glColor3f(1.0, 0.0, 0.0);
        else if (vec[i] < -max_val)
            glColor3f(0.0, 0.0, 1.0);
        else
            glColor4f((vec[i] > 0) ? 1.0 : 0.5
                    , 0.5
                    , (vec[i] < 0) ? 1.0 : 0.5
                    , 0.1 + 0.9 * fabs(clip(vec[i]/max_val , 1.0)));

        draw::fill_rect(posx + i*wbar, posy, 0.9*wbar, height);
    }
}

namespace draw {


void hbar(float px, float py, float dx, float dy, float value, float max_value);
void vbar(float px, float py, float dx, float dy, float value, float max_value);
void block(float px, float py, float sx, float sy, float value, float max_value);

template <typename Vector_t = std::vector<double> > inline void
vec3( const double posx
    , const double posy
    , const double height
    , const double width
    , const Vector_t& vec )
{
    const double max_value = std::max( fabs(vec.get_max()), fabs(vec.get_min()) );
    const double wbar = width/vec.size();

    for (std::size_t i = 0; i < vec.size(); ++i) {
        block(posx + i*wbar, posy, 0.9*wbar, 0.9*height, vec[i], max_value);
        glColor3f(0.8,0.8,0.8);
        glprintf(posx + i*wbar-0.05, posy+0.01, 0, 0.01, "%1.3f", vec[i]);
    }

}

}

#endif // DISPLAY_H_INCLUDED
