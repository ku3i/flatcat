#include <draw/display.h>

namespace draw {

void hbar(float px, float py, float dx, float dy, float value, float max_value)
{
    const float frac = clip(value/max_value, 0.f, 1.f);
    glColor3f(.3f, .3f, .3f);
    fill_rect(px, py, dx, dy);
    glColor3f(.9f, .9f, .9f);
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

}


