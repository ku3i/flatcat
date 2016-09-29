/* plot1D.h */

#ifndef plot1D_H
#define plot1D_H

#include <vector>
#include <algorithm>
#include <draw/axes.h>
#include <draw/draw.h>
#include <common/modules.h>
#include <common/log_messages.h>
#include <basic/color.h>

class plot1D {
public:
    plot1D(unsigned int number_of_samples, axes& a, const Color4& c = colors::white0)
    : number_of_samples(number_of_samples)
    , pointer(0)
    , axis(a)
    , axis_id(axis.countNum++)
    , signal(number_of_samples)
    , color(c)
    , decrement(0.001)
    { }

    void draw(void) const;
    void add_sample(const float s);
    void reset(void) { std::fill(signal.begin(), signal.end(), .0); }

private:
    void increment_pointer(void) { ++pointer; pointer %= number_of_samples; }

    const unsigned int number_of_samples;
    unsigned int pointer;
    axes& axis;
    unsigned int axis_id;
    std::vector<float> signal;
    Color4 color;
    const float decrement;
};

#endif /*plot1D_H*/
