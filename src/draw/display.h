#ifndef DISPLAY_H_INCLUDED
#define DISPLAY_H_INCLUDED

#include <vector>
#include <limits>
#include <draw/draw.h>
#include <common/modules.h>
#include <basic/color.h>

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



void hbar(float px, float py, float dx, float dy, float value, float max_value, Color4 const& color = colors::white);
void vbar(float px, float py, float dx, float dy, float value, float max_value, Color4 const& color = colors::white);
void block(float px, float py, float sx, float sy, float value, float max_value);

template <typename Element_t = double, typename Vector_t = std::vector<Element_t> > inline void
vector_dual( const double posx
                , const double posy
                , const double height
                , const double width
                , const Vector_t& vec1
                , const Vector_t& vec2
                , const Element_t& max_val = 1.0
                , bool with_numbers = true )
{
    assert(vec1.size() == vec2.size());
    const double wbar = width/vec1.size();

    for (std::size_t i = 0; i < vec1.size(); ++i)
    {
        if (vec1[i] > 0.f) draw::vbar(posx+wbar*i, posy, wbar*.4f, height, +vec1[i], max_val, colors::cyan);
        else draw::vbar(posx+wbar*i, posy, wbar*.4f, height, -vec1[i], max_val, colors::magenta);
        if (with_numbers)
            glprintf(posx+wbar*i, posy-height*0.5, .0f, 0.1*wbar, "%+3.1f", vec1[i]);

        if (vec2[i] > 0.f) draw::vbar(posx+wbar*i+wbar*.4f, posy, wbar*.4f, height, +vec2[i], max_val, colors::cyan_l);
        else draw::vbar(posx+wbar*i+wbar*.4f, posy, wbar*.4f, height, -vec2[i], max_val, colors::magenta_l);
        if (with_numbers)
            glprintf(posx+wbar*i+wbar*.4f, posy-height*0.5, .0f, 0.1*wbar, "%+3.1f", vec2[i]);

    }
}

template <typename Vector_t = std::vector<double> > inline void
vec3( const float posx
    , const float posy
    , const float height
    , const float width
    , const Vector_t& vec
    , std::size_t max_elements = std::numeric_limits<std::size_t>::max())
{
    const std::size_t len = std::min(vec.size(), max_elements);
    double max_value = 0;
    for (std::size_t i = 0; i < len; ++i)
        max_value = std::max(max_value, fabs(vec[i]));

    const float wbar = width/vec.size();
    const float px = posx - 0.5*width + 0.5*wbar;
    const float py = posy;

    for (std::size_t i = 0; i < len; ++i) {
        block(px + i*wbar, py, 0.9*wbar, 0.9*height, vec[i], max_value);
    }
    glColor3f(0.8,0.8,0.8);
    for (std::size_t i = 0; i < len; ++i) {
        glprintf(px + (i-0.40)*wbar, py, 0, 0.008, "%.2f", vec[i]);
    }

}

} /* namespace draw */

#endif // DISPLAY_H_INCLUDED
