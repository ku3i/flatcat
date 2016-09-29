/* plot2D.h */

#ifndef plot2D_H
#define plot2D_H

#include <cassert>
#include <vector>
#include <algorithm>
#include "axes.h"
#include "axes3D.h"
#include <common/vector2.h>

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

private:
    void increment_pointer(void) { ++pointer; pointer %= number_of_samples; }

    const unsigned int number_of_samples;
    unsigned int pointer;
    axes& axis;

    std::vector<Vector2> signal;
    const Color4 color;

    const float decrement;
};


#endif /*plot2D_H*/
