/* plot1D.cpp */

#include "plot1D.h"

void
plot1D::draw(void) const
{
    glPushMatrix();
    auto_scale();

    set_color(color);
    draw_line_strip();

    glPopMatrix();
    draw_statistics();
}

void
colored_plot1D::draw_colored(void) const
{
    glPushMatrix();
    auto_scale();

    draw_colored_line_strip();

    glPopMatrix();
    set_color(color);
    draw_statistics();
}

void
plot1D::auto_scale(void) const
{
    const float scale  = 2.0 / (axis.max_amplitude - axis.min_amplitude);
    const float offset = 0.0;//      (axis.max_amplitude + axis.min_amplitude) / 2;

    glTranslatef(axis.px - 0.5 * axis.width, axis.py - offset, axis.pz);
    glScalef( axis.width / number_of_samples
            , 0.5 * axis.height * scale /2 //TODO scale is wrong, remove 0.5?
            , 1.0);

}

void
plot1D::draw_line_strip(void) const
{
    glLineWidth(1.0f);
    glBegin(GL_LINE_STRIP);
    for (unsigned i = number_of_samples+1; i-- > 1; ) { // zero omitted
        const unsigned pos = (i + pointer) % number_of_samples;
        glVertex2f( i, signal[pos]);
    }
    glEnd();
}

void
colored_plot1D::draw_colored_line_strip(void) const
{
    glLineWidth(1.0f);
    glBegin(GL_LINE_STRIP);
    for (unsigned i = number_of_samples+1; i-- > 1; ) { // zero omitted
        const unsigned pos = (i + pointer) % number_of_samples;
        set_color(colortable.get_color(colors[pos]));
        glVertex2f( i, signal[pos]);
    }
    glEnd();
}

void
plot1D::draw_statistics(void) const
{
    glprintf( axis.px - 0.5 * axis.width
            , axis.py + 0.5 * axis.height - 1.1 * axis.font_height * (axis_id + 1)
            , axis.pz
            , axis.font_height
            , "%+.4f %s"
            , signal[pointer], name.c_str() );
}

void
plot1D::add_sample(const float s)
{
    increment_pointer();
    //assert(pointer < number_of_samples);
    signal[pointer] = s;

    /* autoscale //TODO überarbeiten */
    if (pointer == 0) {
        axis.max_amplitude *= decrement;
        axis.min_amplitude *= decrement;
    }
    axis.max_amplitude = std::max(axis.max_amplitude, s);
    axis.min_amplitude = std::min(axis.min_amplitude, s);
}

