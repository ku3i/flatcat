#ifndef GRAPHICS_H_INCLUDED
#define GRAPHICS_H_INCLUDED

#include <draw/draw.h>
#include <basic/vector3.h>

/**TODO namespace draw and rename to graphics_base */

struct pref {
    float x_angle;
    float y_angle;
};

class Graphics_Interface
{
    Vector3 pos;
    double  scale;

public:

    Graphics_Interface()
    : pos(0.0), scale(1.0) {}

    Graphics_Interface(double px, double py, double pz = 0.0, double s = 1.0)
    : pos(px, py, pz), scale(s) {}

    virtual ~Graphics_Interface() {}
    virtual void draw(const pref&) const = 0;

    void drawing(const pref& p) const {
        glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        glScalef(scale, scale, scale);
        draw(p);
        glPopMatrix();
    }

    Graphics_Interface& set_position(double px, double py, double pz = 0.0) { pos = Vector3(px, py, pz); return *this; }
    Graphics_Interface& set_scale   (double s                             ) { scale = s;                 return *this; }

};

#endif // GRAPHICS_H_INCLUDED
