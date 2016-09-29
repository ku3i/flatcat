/* plot3D.h */

#ifndef plot3D_H
#define plot3D_H

#include <vector>
#include <cassert>
#include "axes3D.h"

class plot3D
{
public:
    plot3D(unsigned int number_of_samples, const axes3D& axis, const GLubyte c[4])
    : number_of_samples(number_of_samples)
    , pointer(0)
    , axis(axis)
    , signal0(number_of_samples)
    , signal1(number_of_samples)
    , signal2(number_of_samples)
    {
        for (unsigned int i = 0; i < 4; ++i) color[i] = c[i];
    }

    void draw(float x_angle, float y_angle) const;

    void add_sample(float s0, float s1, float s2);
    void add_sample(const std::vector<double>& sample);

private:
    void increment_pointer(void) { ++pointer; pointer %= number_of_samples; }

    const unsigned int number_of_samples;
    unsigned int pointer;
    const axes3D& axis;
    std::vector<float> signal0;
    std::vector<float> signal1;
    std::vector<float> signal2;
    GLubyte color[4];
};

#endif /*plot3D_H*/
