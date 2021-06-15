#ifndef FLATCAT_GRAPHICS_HPP
#define FLATCAT_GRAPHICS_HPP

#include <draw/axes.h>
#include <draw/plot1D.h>
#include <draw/graphics.h>
#include <draw/display.h>

#include <motorcord.hpp>
#include <flatcat_control.hpp>


namespace supreme {

//TODO move
class BarPlot
{
    const float x, y, w, h, maxval;
    float curval;
    Color4 const& color;

public:

    BarPlot(float x, float y, float w, float h, float maxval, Color4 const& color)
    : x(x), y(y), w(w), h(h), maxval(maxval), curval(0.0), color(color)
    {}

    void add_sample(float x) { curval=x; }

    void draw() const {
        draw::vbar(x, y, w, h, curval, maxval, colors::yellow);
    }
};


class MotorPlot : public Graphics_Interface {
public:

    struct PlotConf {
        unsigned id;
        std::string name;
        float x, y, w, h;
    };

    typedef supreme::interface_data MotorData_t;

    MotorData_t const& ux;
    float const& ctrl_target;

    PlotConf const& conf;
    axes    axis;
    plot1D  plot_target;
    plot1D  plot_position;
    plot1D  plot_velocity;
    plot1D  plot_current;
    plot1D  plot_tmp;
    plot1D  plot_target_voltage;

    BarPlot bar_temperature;
    BarPlot bar_power;

    float temperature;
    float supply_voltage;
    float consumed_power;
    unsigned connection_losses;

    MotorPlot(MotorData_t const& ux, PlotConf const& conf, float const& target)
    : ux(ux)
    , ctrl_target(target)
    , conf(conf)
    , axis(conf.x, conf.y, .0, conf.w, conf.h, 1, conf.name, 0.01f)
    , plot_target        (1000u, axis, colors::magenta)
    , plot_position      (1000u, axis, colors::white  )
    , plot_velocity      (1000u, axis, colors::orange )
    , plot_current       (1000u, axis, colors::yellow )
    , plot_tmp           (1000u, axis, colors::white0 )
    , plot_target_voltage(1000u, axis, colors::cyan   )
    , bar_temperature(conf.x+conf.w/2+0.01, conf.y-conf.h/2, 0.01, conf.h, /*maxval=*/90, colors::yellow)
    , bar_power      (conf.x+conf.w/2+0.02, conf.y-conf.h/2, 0.01, conf.h, /*maxval=*/10, colors::magenta)
    , temperature   (.0)
    , supply_voltage(.0)
    , consumed_power(.0)
    , connection_losses(0)
    {
//TODO:        sts_msg("Created plot for motor %u", ux.get_id());
        axis.set_fontheight(0.03);
    }

    void update_samples(void)
    {
        temperature       = ux.temperature;
        supply_voltage    = ux.voltage_supply;
        consumed_power    = ux.voltage_supply * ux.current;
        //TODO::connection_losses = ux.connection_losses;

        plot_target        .add_sample(ctrl_target     );
        plot_position      .add_sample(ux.position      );
        plot_velocity      .add_sample(ux.velocity      );
        plot_current       .add_sample(ux.current       );
        plot_target_voltage.add_sample(ux.output_voltage);
        bar_temperature    .add_sample(ux.temperature   );
        bar_power          .add_sample(consumed_power  );
    }

    void draw(const pref& /*p*/) const {
        axis.draw();
       //TODO: if (ux.is_active) {
            plot_target        .draw();
            plot_position      .draw();
            plot_velocity      .draw();
            plot_current       .draw();
            plot_tmp           .draw();
            plot_target_voltage.draw();

            /* bar plots */
            bar_temperature.draw();
            bar_power      .draw();

            set_color(colors::white);
            glprintf(conf.x + conf.w/2 + 0.05, conf.y + conf.h/2 - 0.03, 0.0, 0.025, "%3.1f C", temperature);
            glprintf(conf.x + conf.w/2 + 0.05, conf.y + conf.h/2 - 0.06, 0.0, 0.025, "%4.2fV" , supply_voltage);
            glprintf(conf.x + conf.w/2 + 0.05, conf.y + conf.h/2 - 0.09, 0.0, 0.025, "%4.2fW" , consumed_power);
            glprintf(conf.x + conf.w/2 + 0.05, conf.y + conf.h/2 - 0.12, 0.0, 0.025, "%u"     , connection_losses);

            /*
            const int ctrl = (int) ux.get_controller_type();
            if (ctrl == 0) set_color(colors::orange);
            glprintf(conf.x + conf.w/2 + 0.05, conf.y + conf.h/2 - 0.12, 0.0, 0.025, "%d%s" , ctrl, ctrl == 0 ? " (OFF)":"");
            */
            /* statistics */
            /*
            set_color(colors::white);
            auto const& s = ux.get_stats();
            glprintf(conf.x + conf.w/2 + 0.05, conf.y + conf.h/2 - 0.15, 0.0, 0.025, "e=%u t=%u" , s.errors, s.timeouts);
            */
       //TODO: }
    }
};

/** TODO:
class AccelPlot : public Graphics_Interface {
    axes    axis;
    plot1D  le_x;
    plot1D  fw_y, up_z;
    robots::Accelvector_t const& accels;

public:

    AccelPlot(robots::Accelvector_t const& accels, MotorPlot::PlotConf const& conf)
    : axis(conf.x, conf.y, .0, conf.w, conf.h, 1, conf.name)
    , le_x(1000u, axis, colors::cyan   )
    , fw_y(1000u, axis, colors::magenta)
    , up_z(1000u, axis, colors::yellow )
    , accels(accels)
    {}

    void add_sample(void) {
        auto const& data = accels[0].a;
        le_x.add_sample(data.x);
        fw_y.add_sample(data.y);
        up_z.add_sample(data.z);
    }

    void draw(const pref& p) const {
        axis.draw();
        le_x.draw();
        fw_y.draw();
        up_z.draw();
    }
};
*/

class FlatcatGraphics : public Graphics_Interface {
public:

    std::vector<MotorPlot> plots;

    std::vector<MotorPlot::PlotConf> confs = {{0x0, "0 Head", 0.0, +0.50, 1.2, 0.5}
                                             ,{0x1, "1 Body", 0.0, +0.00, 1.2, 0.5}
                                             ,{0x2, "2 Tail", 0.0, -0.50, 1.2, 0.5}
                                             };

    //MotorPlot::PlotConf  accel_conf = {0, "acceleration", +0.0, +1.10, 1.8, 0.20};
    //TODO AccelPlot accel_plot;

    template <typename RobotType, typename TargetPositionType>
    FlatcatGraphics(RobotType const& robot, TargetPositionType const& targets)
    : plots()
    //TODO, accel_plot(robot.get_accels(), accel_conf)
    {
        sts_msg("Creating Flatcat Graphics");
        auto const& motors = robot.get_motors();
        assert(motors.size() > 0);
        plots.reserve(motors.size());
        assert(targets.size() >= motors.size());
        assert(confs  .size() >= motors.size());

        for (std::size_t i = 0; i < motors.size(); ++i) {
            plots.emplace_back(motors[i], confs[i], targets[i]);
        }
    }

    void update_samples(void) {
        for (auto& plot : plots)
            plot.update_samples();
        //TODO: accel_plot.add_sample();
    }

    void draw(const pref& p) const {
        for (auto& plot : plots)
            plot.draw(p);
        //TODO:: accel_plot.draw(p);

        set_color(colors::white);
        /*glprintf(-.05f, +.90f, 0.0, 0.04, "Fore");
        glprintf(-.04f, -.94f, 0.0, 0.04, "Aft");
        glprintf(-.90f, -.02f, 0.0, 0.04, "Port");
        glprintf(+.60f, -.02f, 0.0, 0.04, "Starboard");
        */
    }
};

} /* namespace supreme */

#endif /* FLATCAT_GRAPHICS_HPP */
