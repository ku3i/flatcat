/* axes.h */

#ifndef AXES_H
#define AXES_H

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <string>

#include "draw.h"
#include "../common/modules.h"

class axes
{
    friend class plot1D;
    friend class plot2D;
    friend class network2D;

private:
    float px, py, pz;
    float width, height;
    GLfloat a[4][2];
    int flag; /* axes flag */
    unsigned int countNum;
    float max_amplitude;
    float min_amplitude;
    float font_height;
    std::string name;

public:
    axes(float x, float y, float z, float w, float h, int flags, std::string name, float def_amp = 1.0f);
    void set_fontheight(float f) { font_height = f; }
    void draw(void) const;
};

#endif /*AXES_H*/
