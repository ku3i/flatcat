/* plot2D.h */

#ifndef plot2D_H
#define plot2D_H

#include <cassert>
#include <vector>
#include <algorithm>
#include <basic/color.h>
#include <common/vector2.h>
#include <draw/axes.h>
#include <draw/axes3D.h>
#include <draw/color_table.h>

//TODO autoscale.
// * use gltranslate and glscale for autoscale
// * also autoadjust the offset
// Draw in time?

class plot2D
{
public:
    plot2D(unsigned int number_of_samples, axes& axis, const Color4& color)
    : number_of_samples(number_of_samples)
    , pointer(0)
    , axis(axis)
    , signal(number_of_samples)
    , color(color)
    , decrement(0.001)
    { }

    void draw(void) const;
    //void draw_in_time(void) const;
    void add_sample(float s0, float s1);
    void add_sample(const std::vector<double>& sample);
    void reset(void) { std::fill(signal.begin(), signal.end(), Vector2()); }

protected:
    void adjust_amplitude(float s0, float s1) const;
    void autoscale(void) const;
    void increment_pointer(void) { ++pointer; pointer %= number_of_samples; }

    const unsigned int number_of_samples;
    unsigned int pointer;
    axes& axis;

    std::vector<Vector2> signal;
    const Color4 color;

    const float decrement;
};


class colored_plot2D : public plot2D {
public:
    colored_plot2D( unsigned int number_of_samples
                  , axes& a
                  , ColorTable const& colortable
                  /*, const char* name = ""*/)
    : plot2D(number_of_samples, a, colors::white0) //TODO, name)
    , colors(number_of_samples)
    , colortable(colortable)
    {}

    void add_colored_sample(float s0, float s1, unsigned color_index) {
        add_sample(s0, s1);
        colors.at(pointer) = color_index;
    }

    void draw_colored(void) const;

private:
    //void draw_colored_line_strip(void) const;

    std::vector<unsigned> colors;
    ColorTable const& colortable;

};

#endif /*plot2D_H*/
