#include <draw/display.h>

namespace draw {

void hbar(float px, float py, float dx, float dy, float value, float max_value, Color4 const& color)
{
    const float frac = clip(value/max_value, 0.f, 1.f);
    set_color(color, 0.3);
    fill_rect(px, py, dx, dy);
    set_color(color);
    fill_rect(px, py, dx*frac, dy);
}

void vbar(float px, float py, float dx, float dy, float value, float max_value)
{
    const float frac = clip(value/max_value, 0.f, 1.f);
    glColor3f(.3f, .3f, .3f);
    fill_rect(px, py, dx, dy);
    glColor3f(.9f, .9f, .9f);
    fill_rect(px, py, dx, dy*frac);
}

void block(float px, float py, float sx, float sy, float value, float max_value)
{
    float frac = clip(fabs(value/max_value), 0.f, 1.f);

    if (value >= 0.f) glColor4f(.0f, .5f, 1.f, frac);
    else              glColor4f(1.f, .5f, .0f, frac);

    draw_fill_rect(px, py, sx, sy);
    glColor4f(.9f, .9f, .9f, .5f);
    draw_rect(px, py, sx, sy);
}

} /* namespace draw */


