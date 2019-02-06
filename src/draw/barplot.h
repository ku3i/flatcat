#ifndef BARPLOT_H_INCLUDED
#define BARPLOT_H_INCLUDED

#include <draw/draw.h>
#include <draw/display.h>

namespace draw {

class BarPlot
{
    const float x, y, w, h, maxval;
    float curval;
    Color4 const& color, max_color;

public:

    BarPlot(float x, float y, float w, float h, float maxval = 1.0, Color4 const& color = colors::white, Color4 const& max_color = colors::redorange)
    : x(x), y(y), w(w), h(h), maxval(maxval), curval(0.0), color(color), max_color(max_color)
    {}

    void add_sample(float x) { curval=x; }

    void draw() const {
        if (curval<maxval)
            vbar(x, y, w, h, curval, maxval, color);
        else
            vbar(x, y, w, h, curval, maxval, max_color);
    }
};

} /* namespace draw */

#endif // BARPLOT_H_INCLUDED
