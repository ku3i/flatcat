/* plot1D.cpp */

#include "plot1D.h"

void
plot1D::draw(void) const
{
    float scale  = 2.0 / (axis.max_amplitude - axis.min_amplitude);
    float offset = 0.0;//      (axis.max_amplitude + axis.min_amplitude) / 2;

    glPushMatrix();
    glTranslatef(axis.px - 0.5 * axis.width, axis.py - offset, axis.pz);
    glScalef( axis.width / number_of_samples
            , 0.5 * axis.height * scale /2 //TODO scale is wrong, remove 0.5?
            , 1.0);


    set_color(color);
    glLineWidth(1.0f);

    glBegin(GL_LINE_STRIP);
    for (unsigned i = number_of_samples; i-- > 1; ) { // zero omitted
        glVertex2f( i, signal[(i + pointer) % number_of_samples]);
    }

    glEnd();
    glPopMatrix();

    glprintf( axis.px - 0.5 * axis.width
            , axis.py + 0.5 * axis.height - 1.1 * axis.font_height * (axis_id + 1)
            , axis.pz
            , axis.font_height
            , "%+.4f"
            , signal[pointer] );
}

void
plot1D::add_sample(const float s)
{
    increment_pointer();
    signal[pointer] = s;

    /* autoscale //TODO überarbeiten */
    if (pointer == 0) {
        axis.max_amplitude -= decrement;
        axis.min_amplitude += decrement;
    }
    axis.max_amplitude = std::max(axis.max_amplitude, s);
    axis.min_amplitude = std::min(axis.min_amplitude, s);
}

