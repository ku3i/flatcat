/* plot1D.h */

#ifndef plot1D_H
#define plot1D_H

#include <vector>
#include <algorithm>
#include <draw/axes.h>
#include <draw/draw.h>
#include <draw/color_table.h>
#include <common/modules.h>
#include <common/log_messages.h>
#include <basic/color.h>

class plot1D {
public:
    plot1D(unsigned int number_of_samples, axes& a, const Color4& c = colors::white0, const char* name = "")
    : number_of_samples(number_of_samples)
    , pointer(0)
    , axis(a)
    , axis_id(axis.countNum++)
    , signal(number_of_samples)
    , color(c)
    , decrement(0.99)
    , name(name)
    { }

    virtual ~plot1D() = default;

    void draw(void) const;
    void add_sample(const float s);
    void reset(void) { std::fill(signal.begin(), signal.end(), .0); pointer = 0; }

protected:
    void increment_pointer(void) { ++pointer; pointer %= number_of_samples; }

    void auto_scale     (void) const;
    void draw_statistics(void) const;
    void draw_line_strip(void) const;

    const unsigned int number_of_samples;
    unsigned int pointer;
    axes& axis;
    unsigned int axis_id;
    std::vector<float> signal;
    Color4 color;
    const float decrement;
    const std::string name;
};


class colored_plot1D : public plot1D {
public:
    colored_plot1D( unsigned int number_of_samples
                  , axes& a
                  , ColorTable const& colortable )
    : plot1D(number_of_samples, a)
    , colors(number_of_samples)
    , colortable(colortable)
    {}

    void add_colored_sample(float s, unsigned color_index) {
        add_sample(s),
        colors.at(pointer) = color_index;
    }

    void draw_colored(void) const;

private:
    void draw_colored_line_strip(void) const;

    std::vector<unsigned> colors;
    ColorTable const& colortable;

};
#endif /*plot1D_H*/
