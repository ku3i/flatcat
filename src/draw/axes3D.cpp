/* axes3D.cpp */

#include "axes3D.h"
#include "draw.h"

axes3D::axes3D(float x, float y, float z, float w, float h, float d, int f)
: px(x), py(y), pz(z), width(w), height(h), depth(d), flag(f)
{
    /* init frame coordinates */
    r[0][0] = -.5*width;
    r[0][1] = +.5*height;
    r[0][2] = -.5*depth;

    r[1][0] = +.5*width;
    r[1][1] = +.5*height;
    r[1][2] = -.5*depth;

    r[2][0] = +.5*width;
    r[2][1] = -.5*height;
    r[2][2] = -.5*depth;

    r[3][0] = -.5*width;
    r[3][1] = -.5*height;
    r[3][2] = -.5*depth;

    r[4][0] = -.5*width;
    r[4][1] = +.5*height;
    r[4][2] = +.5*depth;

    r[5][0] = +.5*width;
    r[5][1] = +.5*height;
    r[5][2] = +.5*depth;

    r[6][0] = +.5*width;
    r[6][1] = -.5*height;
    r[6][2] = +.5*depth;

    r[7][0] = -.5*width;
    r[7][1] = -.5*height;
    r[7][2] = +.5*depth;


    /* init axes coordinates */
    a[0][0] = -.5*width;
    a[0][1] = 0;
    a[0][2] = 0;

    a[1][0] = +.5*width;
    a[1][1] = 0;
    a[1][2] = 0;

    a[2][0] = 0;
    a[2][1] = -.5*height;
    a[2][2] = 0;

    a[3][0] = 0;
    a[3][1] = +.5*height;
    a[3][2] = 0;

    a[4][0] = 0;
    a[4][1] = 0;
    a[4][2] = -.5*depth;

    a[5][0] = 0;
    a[5][1] = 0;
    a[5][2] = +.5*depth;
}

void axes3D::draw(float x_angle, float y_angle) const
{
    glColor4ubv(white_trans);

    glPushMatrix();
    glTranslatef(px, py, pz);
    glRotatef(y_angle, 1.0, 0.0, 0.0);
    glRotatef(x_angle, 0.0, 1.0, 0.0);

    draw_cube(r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7]);

    switch(flag)
    {
        case 0:
            draw_line(a[0],a[1]);
            draw_line(a[2],a[3]);
            draw_line(a[4],a[5]);
            break;
        case 1:
            draw_line(a[0],a[1]);
            break;
        default:
            break;
    }
    glPopMatrix();
}

void axes3D::axesflag(int f)
{
    flag = f;
}

/* axes.cpp */

