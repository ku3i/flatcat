/* plot2D.cpp */

#include "plot2D.h"

void plot2D::autoscale(void) const
{
    float scale  = 2.0 / (axis.max_amplitude - axis.min_amplitude);
    float offset = 0.0;//TODO      (axis.max_amplitude + axis.min_amplitude) / 2;

    glTranslatef(axis.px - offset, axis.py - offset, axis.pz);
    glScalef( 0.5 * axis.width  * scale
            , 0.5 * axis.height * scale
            , 1.0);
}

void plot2D::draw(void) const
{
    glPushMatrix();
    autoscale();

    glLineWidth(1.0f);

    glBegin(GL_LINE_STRIP);
    for (unsigned i = number_of_samples; i-- > 1; ) { // zero omitted
        set_color(color, (float) i/number_of_samples);
        glVertex2f((signal[(i + pointer) % number_of_samples].x),
                   (signal[(i + pointer) % number_of_samples].y));
    }
    glEnd();
    glPopMatrix();
}

//void plot2D::draw_in_time(void) const
//{
//    glPushMatrix();
//    glTranslatef(px,py,pz);
//
//    glBegin(GL_LINE_STRIP);
//    for (unsigned int i = number_of_samples - 1; i != 0; --i) {
//        glVertex3f(0.5 * width + i * width / number_of_samples,
//                   0.5 * height * signal[0][(i + pointer) % number_of_samples],
//                   0.5 * depth  * signal[1][(i + pointer) % number_of_samples]);
//    }
//    glEnd();
//    glPopMatrix();
//}

void plot2D::adjust_amplitude(float s0, float s1) const
{
    if (pointer == 0) {
        axis.max_amplitude -= decrement;
        axis.min_amplitude += decrement;
    }

    axis.max_amplitude = std::max(axis.max_amplitude, std::max(s0, s1));
    axis.min_amplitude = std::min(axis.min_amplitude, std::min(s0, s1));
}

void plot2D::add_sample(float s0, float s1)
{
    increment_pointer();
    signal[pointer].x = s0;
    signal[pointer].y = s1;

    adjust_amplitude(s0,s1);
}

void plot2D::add_sample(const std::vector<double>& sample)
{
    assert(sample.size() >= 2);
    increment_pointer();
    signal[pointer].x = sample[0];
    signal[pointer].y = sample[1];

    adjust_amplitude(sample[0],sample[1]);
}

void colored_plot2D::draw_colored(void) const
{
    glPushMatrix();
    autoscale();

    glLineWidth(1.0f);

    glBegin(GL_LINE_STRIP);
    for (unsigned i = number_of_samples; i-- > 1; ) { // zero omitted
        const unsigned pos = (i + pointer) % number_of_samples;
        set_color(colortable.get_color(colors[pos]), (float) i/number_of_samples);
        glVertex2f( signal[pos].x, signal[pos].y );
    }
    glEnd();
    glPopMatrix();
}

/* plot2D.cpp */
