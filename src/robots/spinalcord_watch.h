/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | July 2017                       |
 +---------------------------------*/

#ifndef SPINALCORD_WATCH_H_INCLUDED
#define SPINALCORD_WATCH_H_INCLUDED

#include <draw/draw.h>
#include <draw/axes.h>
#include <draw/axes3D.h>
#include <draw/plot1D.h>
#include <draw/plot2D.h>
#include <draw/plot3D.h>
#include <draw/network3D.h>
#include <draw/graphics.h>

/** TODO:
 * make this class more compatible to different (simloid) robots, e.g. with different draw-setups (use new data reader for reading this setup)
 * draw acceleration sensors
 */

namespace robots {

namespace constants {
    const double vel_amp = 1.0;///3.0;
}

// consider using Motorplot from Hannah

class Spinalcord_Watch : public Graphics_Interface
{
public:
    Spinalcord_Watch(const robots::Robot_Interface& robot, const std::size_t num_datapoints)
    : joints(robot.get_joints())
    , num_joints(robot.get_number_of_joints())
    , accels(robot.get_accels())
    , plot_axs()
    , plot_pos()
    , plot_vel()
    , plot_vol()
    , plot_cur()
    , plot_tmp()
    , subspace_axes()
    , subspace_portrait()
    , axes_accel(0.,-1.25, 0., 2.0, 0.25, 1, "Accel")
    , plot_accel_x(1000, axes_accel, colors::cyan   , "ax")
    , plot_accel_y(1000, axes_accel, colors::magenta, "ay")
    , plot_accel_z(1000, axes_accel, colors::yellow , "az")
    {
        plot_axs.reserve(num_joints);
        plot_pos.reserve(num_joints);
        plot_vel.reserve(num_joints);
        plot_vol.reserve(num_joints);
        plot_cur.reserve(num_joints);
        plot_tmp.reserve(num_joints);

        subspace_axes.reserve(num_joints);
        subspace_portrait.reserve(num_joints);

        for (std::size_t i = 0; i < num_joints; ++i)
        {
            const double width  =  1.0;
            const double height =  4.0 / num_joints;
            const double posx   =  (i%2==0)? -1.5 : 1.5;
            const double posy   =  1.0 - height * (i/2 + 0.5);

            plot_axs.emplace_back(posx, posy, 0., width, height, 1, std::to_string(i) + ' ' + joints[i].name);
            plot_cur.emplace_back(num_datapoints, plot_axs[i], colors::yellow, "cur");
            plot_tmp.emplace_back(num_datapoints, plot_axs[i], colors::orange, "tmp");
            plot_vol.emplace_back(num_datapoints, plot_axs[i], colors::cyan  , "vol");
            plot_vel.emplace_back(num_datapoints, plot_axs[i], colors::pidgin, "vel");
            plot_pos.emplace_back(num_datapoints, plot_axs[i], colors::white , "pos");

            subspace_axes    .emplace_back(posx + ((i%2==0) ? -1:1) * (width*0.5+0.5*height), posy, 0., height, height, 0, 'j' + std::to_string(i));
            subspace_portrait.emplace_back(num_datapoints, subspace_axes[i], colors::white);

        }
       //TODO: assert(robot.get_number_of_accel_sensors() >= 1); //TODO make applicable for more than one sensor
    }

    void draw(const pref& /*p*/) const {
        for (std::size_t i = 0; i < num_joints; ++i) {
            plot_axs[i].draw();
            plot_cur[i].draw();
            plot_tmp[i].draw();
            plot_pos[i].draw();
            plot_vel[i].draw();
            plot_vol[i].draw();

            subspace_axes[i]    .draw();
            subspace_portrait[i].draw();
        }
        axes_accel  .draw();
        plot_accel_x.draw();
        plot_accel_y.draw();
        plot_accel_z.draw();
    }

    void execute_cycle(uint64_t /*cycle*/) {
        for (std::size_t i = 0; i < num_joints; ++i) {
            plot_cur[i].add_sample(joints[i].s_cur);
            plot_tmp[i].add_sample(joints[i].s_tmp);
            plot_pos[i].add_sample(joints[i].s_ang);
            plot_vel[i].add_sample(joints[i].s_vel * constants::vel_amp);
            plot_vol[i].add_sample(joints[i].motor.get_backed());

            subspace_portrait[i].add_sample(joints[i].s_ang,
                                            joints[i].s_vel * constants::vel_amp);
        }
        if (accels.size() > 0) {
            plot_accel_x.add_sample(accels[0].a.x);
            plot_accel_y.add_sample(accels[0].a.y);
            plot_accel_z.add_sample(accels[0].a.z);
        }
    }

    const robots::Jointvector_t& joints;
    const std::size_t            num_joints;

    const robots::Accelvector_t& accels;

    std::vector<axes>   plot_axs;

    std::vector<plot1D> plot_pos;
    std::vector<plot1D> plot_vel;
    std::vector<plot1D> plot_vol;
    std::vector<plot1D> plot_cur;
    std::vector<plot1D> plot_tmp;

    std::vector<axes>   subspace_axes;
    std::vector<plot2D> subspace_portrait;

    axes   axes_accel;
    plot1D plot_accel_x;
    plot1D plot_accel_y;
    plot1D plot_accel_z;
};

} // namespace robots

#endif // SPINALCORD_WATCH_H_INCLUDED
