/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | August 2020                     |
 +---------------------------------*/

#ifndef HOMEOKINESIS_GRAPHICS_H
#define HOMEOKINESIS_GRAPHICS_H

#include <draw/draw.h>
#include <draw/axes.h>
#include <draw/axes3D.h>
#include <draw/plot1D.h>
#include <draw/plot2D.h>
#include <draw/plot3D.h>
#include <draw/network3D.h>
#include <draw/graphics.h>

#include <learning/homeokinesis.h>

namespace learning {

class Homeokinesis_Graphics : public Graphics_Interface
{
    Homeokinetic_Control const& ctrl;

public:
    Homeokinesis_Graphics(Homeokinetic_Control const& ctrl)
    : ctrl(ctrl)
    , axes_err(0., -0.5, 0., 2.0, 1.0, 1, "error"  , 0.001)
    , plot_pre(1000, axes_err, colors::cyan    , "pre")
    , plot_tle(1000, axes_err, colors::magenta , "tle")
    , plot_rec(1000, axes_err, colors::yellow  , "rec")
    , plot_ctr(1000, axes_err, colors::green   , "ctr")
    {}

    void draw(const pref& /*p*/) const {
        axes_err.draw();
        plot_pre.draw();
        plot_tle.draw();
        plot_rec.draw();
        plot_ctr.draw();
        draw_motor_context();
    }

    void draw_motor_context(void) const
    {
        draw::vector_dual(-1,-0.8,0.06,1.0, ctrl.y0, ctrl.Y0);
    }

    void execute_cycle(uint64_t /*cycle*/) {
        plot_pre.add_sample(ctrl.get_prediction_error    ());
        plot_tle.add_sample(ctrl.get_timeloop_error      ());
        plot_rec.add_sample(ctrl.get_reconstruction_error());
        plot_ctr.add_sample(ctrl.get_control_error       ());
    }

    axes   axes_err;
    plot1D plot_pre;
    plot1D plot_tle;
    plot1D plot_rec;
    plot1D plot_ctr;
};

} /* namespace learning */

#endif /* HOMEOKINESIS_GRAPHICS_H */

