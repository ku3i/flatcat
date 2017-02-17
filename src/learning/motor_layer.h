#ifndef MOTOR_LAYER_H_INCLUDED
#define MOTOR_LAYER_H_INCLUDED

#include <robots/robot.h>
#include <robots/joint.h>
#include <control/jointcontrol.h>
#include <control/control_vector.h>
#include <control/sensorspace.h>
#include <learning/expert.h>
#include <learning/gmes.h>
#include <learning/payload.h>
#include <learning/motor_predictor.h>

/* graphics */
#include <draw/draw.h>
#include <draw/axes.h>
#include <draw/axes3D.h>
#include <draw/plot1D.h>
#include <draw/plot2D.h>
#include <draw/plot3D.h>
#include <draw/network3D.h>
#include <draw/graphics.h>


namespace learning {

namespace constants {
    const double number_of_experts    = 20;
    const double local_learning_rate  = 0.01;
    const double gmes_learning_rate   = 1.0;//10.0;
    const std::size_t experience_size = 100;
}


class Motor_Space : public sensor_vector {
public:
    Motor_Space(const robots::Jointvector_t& joints)
    : sensor_vector(joints.size())
    {
        for (robots::Joint_Model const& j : joints)
            sensors.emplace_back(j.name, [&j](){ return j.motor.get() + rand_norm_zero_mean(0.01); });
    }
};


class Motor_Layer {
public:
    Motor_Layer( robots::Robot_Interface const& robot
               , std::size_t max_num_motor_experts = constants::number_of_experts
               , std::size_t experience_size = constants::experience_size )
    : max_num_motor_experts(max_num_motor_experts)
    , params(control::param_factory(robot, max_num_motor_experts, "", {0.,0.,0.}))
    , payloads(max_num_motor_experts)
    , motorspace(robot.get_joints())
    , experts(max_num_motor_experts, payloads, motorspace, constants::local_learning_rate, experience_size, params, robot)
    , gmes(experts, constants::gmes_learning_rate, /* one shot learning = */false)
    {
        dbg_msg("Creating new competitive motor layer.");
    }

    void execute_cycle(void) {
        motorspace.execute_cycle();
        gmes.execute_cycle();
    }

    std::size_t                  max_num_motor_experts;
    control::Control_Vector      params;
    static_vector<Empty_Payload> payloads;
    Motor_Space                  motorspace;
    Expert_Vector                experts;
    GMES                         gmes;

    friend class Motor_Layer_Graphics;
};



namespace constants {
    const unsigned subspace_num_datapoints = 200; // 2s of data at 100Hz
}

struct subspace_graphics : public Graphics_Interface
{
    subspace_graphics( robots::Joint_Model const& j0
                     , robots::Joint_Model const& j1
                     , Vector3 pos, float size )
    : j0(j0)
    , j1(j1)
    , axis_xy(pos.x + size/2        , pos.y + size/2, pos.z,     size, size, 0, std::string("foo"))
    , axis_dt(pos.x + (2.0 + size)/2, pos.y + size/2, pos.z, 2.0-size, size, 1, std::string("bar"))
    , plot_xy(constants::subspace_num_datapoints, axis_xy, colors::magenta )
    , plot_j0(constants::subspace_num_datapoints, axis_dt, colors::cyan    )
    , plot_j1(constants::subspace_num_datapoints, axis_dt, colors::orange  )
    , plot_p0(constants::subspace_num_datapoints, axis_dt, colors::cyan_t  )
    , plot_p1(constants::subspace_num_datapoints, axis_dt, colors::orange_t)
    {
        dbg_msg("Initialize subspace for joints:\n\t%2u: %s\n\t%2u: %s", j0.joint_id, j0.name.c_str()
                                                                       , j1.joint_id, j1.name.c_str() );
    }

    void draw(const pref&) const {
        axis_xy.draw();
        plot_xy.draw();

        axis_dt.draw();
        plot_j0.draw(); // signals
        plot_j1.draw();
        plot_p0.draw(); // predictions
        plot_p1.draw();
    }

    void execute_cycle(float s0, float s1) {
        plot_xy.add_sample(j0.motor.get(), j1.motor.get());
        plot_j0.add_sample(j0.motor.get());
        plot_j1.add_sample(j1.motor.get());
        plot_p0.add_sample(s0);
        plot_p1.add_sample(s1);
    }

    const robots::Joint_Model& j0;
    const robots::Joint_Model& j1;

    axes   axis_xy;
    axes   axis_dt;

    plot2D plot_xy;
    plot1D plot_j0, plot_j1; // joint motor commands
    plot1D plot_p0, plot_p1; // predictions

};

class Motor_Layer_Graphics : public Graphics_Interface {
public:

    Motor_Layer_Graphics( Motor_Layer const& motor_layer
                        , robots::Robot_Interface const& robot )
    : motor_layer(motor_layer)
    , num_experts()
    , max_experts(motor_layer.gmes.get_max_number_of_experts())
    , winner()
    , subspace()
    {
        /** TODO
            + also for the rest of the joints
            + consider grouping the graphs in legs (4x 3D) instead of 6x 2D
        */
        const unsigned N = robot.get_number_of_symmetric_joints();
        const double size = 2.0/N;
        subspace.reserve(N);
        Vector3 pos(-1.0, 1.0, 0.);
        for (auto const& j0: robot.get_joints()) {
            if (j0.is_symmetric()) {
                robots::Joint_Model const& j1 = robot.get_joints()[j0.symmetric_joint];
                pos.y -= size;
                subspace.emplace_back(j1, j0, pos, size);
            }
        }
    }

    void draw(const pref& p) const {
        glColor3f(1.0, 1.0, 1.0);
        glprintf(-0.95, 0.95, 0., 0.025, "%03u (%03u/%03u)", winner, num_experts, max_experts);

        for (auto& s: subspace)
            s.draw(p);
    };

    void execute_cycle() {

        num_experts = motor_layer.gmes.get_number_of_experts(); // update number of experts
        winner = motor_layer.gmes.get_winner();
        auto const& predictions = motor_layer.experts[winner].get_predictor().get_prediction();

        for (auto& s: subspace) {
            auto s0 = predictions.at(s.j0.joint_id);
            auto s1 = predictions.at(s.j1.joint_id);
            s.execute_cycle(s0, s1);
        }
    };

    Motor_Layer const& motor_layer;
    std::size_t        num_experts;
    std::size_t        max_experts;
    std::size_t        winner;

    std::vector<subspace_graphics> subspace;

};

} // namespace learning


#endif // MOTOR_LAYER_H_INCLUDED

