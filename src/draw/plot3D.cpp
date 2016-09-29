/* plot3D.cpp */

#include "plot3D.h"

void plot3D::draw(float x_angle, float y_angle) const
{
    glPushMatrix();
    glTranslatef(axis.px, axis.py, axis.pz);
    glRotatef(y_angle, 1.0, 0.0, 0.0);
    glRotatef(x_angle, 0.0, 1.0, 0.0);

    glBegin(GL_LINE_STRIP);
    //glColor4ubv(color);
    for (unsigned int i = number_of_samples - 1; i != 0; --i) {
//      color[3] = 255*i/N; //TODO
        glColor4ubv(color);
        glVertex3f(0.5 * axis.width  * signal0[(i + pointer) % number_of_samples],
                   0.5 * axis.height * signal1[(i + pointer) % number_of_samples],
                   0.5 * axis.depth  * signal2[(i + pointer) % number_of_samples]);
    }
    glEnd();
    glPopMatrix();
}

void plot3D::add_sample(float s0, float s1, float s2)
{
    increment_pointer();
    signal0[pointer] = s0;
    signal1[pointer] = s1;
    signal2[pointer] = s2;
}

void plot3D::add_sample(const std::vector<double>& sample)
{
    assert(sample.size() >= 3);
    increment_pointer();
    signal0[pointer] = sample[0];
    signal1[pointer] = sample[1];
    signal2[pointer] = sample[2];
}


/* plot3D.cpp */

