/* axes.cpp */

#include "axes.h"

axes::axes(float x, float y, float z, float w, float h, int flags, std::string namestr)
: px(x), py(y), pz(z)
, width(w), height(h)
, flag(flags)
, countNum(0)
, max_amplitude(+1.0)
, min_amplitude(-1.0)
, font_height(clip(0.75 * height, .01, .04))
, name(namestr)
{
    /* init axes coordinates */
    a[0][0] = -0.5*width;
    a[0][1] = 0;

    a[1][0] = 0.5*width;
    a[1][1] = 0;

    a[2][0] = 0;
    a[2][1] = 0.5*height;

    a[3][0] = 0;
    a[3][1] = -0.5*height;
}

void axes::draw(void) const
{
    glPushMatrix();
    glTranslatef(px, py, pz);

    glColor4ubv(white_trans);
    glLineWidth(1.0f);

    draw_rect(width, height);

    glColor4ubv(white_trans2);
    glprints(-0.5 * width, -0.5 * height + 0.5 * font_height, 0.0,
             font_height,
             name);

    switch(flag)
    {
        case 0:
            draw_line2D(a[0], a[1]);
            draw_line2D(a[2], a[3]);
            break;
        case 1:
            draw_line2D(a[0], a[1]);
            break;
        default:
            break;
    }
    glPopMatrix();
}

/* axes.cpp */

