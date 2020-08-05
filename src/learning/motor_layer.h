#ifndef MOTOR_LAYER_H_INCLUDED
#define MOTOR_LAYER_H_INCLUDED

#include <robots/robot.h>
#include <robots/joint.h>
#include <control/jointcontrol.h>
#include <control/control_vector.h>
#include <control/controlparameter.h>
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
#include <draw/display.h>


namespace learning {

namespace motor_layer_constants {
    const double number_of_experts    = 19;
    const double learning_rate        = 0.001;
    const double growth_rate          = 1.0;
    const std::size_t experience_size = 100;
    const double noise_level          = 0.005; // 0.01 is maybe too high
}


class Motor_Space : public sensor_vector {
public:
    Motor_Space(const robots::Jointvector_t& joints)
    : sensor_vector(joints.size())
    {
        for (robots::Joint_Model const& j : joints)
            sensors.emplace_back(j.name, [&j](){ return clip(j.motor.get()); });
            //sensors.emplace_back(j.name, [&j](){ return clip(j.motor.get() + rand_norm_zero_mean(0.01), 1.0); });
    }
};


std::size_t getnum(void){
    return random_index(5);
}

class Motor_Layer : public learning::Learning_Machine_Interface {
public:
    Motor_Layer( robots::Robot_Interface const& robot
               , std::size_t max_num_motor_experts = motor_layer_constants::number_of_experts
               , const double learning_rate = motor_layer_constants::learning_rate
               , const double growth_rate = motor_layer_constants::growth_rate
               , std::size_t experience_size = motor_layer_constants::experience_size
               , double noise_level = motor_layer_constants::noise_level
               , std::string const& initial_parameter_folder = ""
               , control::Minimal_Seed_t seed = {0.,0.,0.}
               , std::size_t num_initial_experts = 1)
    : params(control::param_factory(robot, max_num_motor_experts, initial_parameter_folder, seed))
    , payloads(max_num_motor_experts)
    , motorspace(robot.get_joints())
    , experts(max_num_motor_experts, payloads, motorspace, learning_rate, experience_size, noise_level, params, robot )
    , gmes(experts, growth_rate, /* one shot learning = */false, num_initial_experts)
    {
        sts_msg("Creating motor layer with max. %u experts and growth rate: %1.4f", max_num_motor_experts, growth_rate);
        if (initial_parameter_folder != "")
            sts_msg("Reading initial parameters from '%s'", initial_parameter_folder.c_str());

        sts_msg("Number of initial experts: %u (loaded and created: %u)", num_initial_experts, params.size());
        assert(params.size() == max_num_motor_experts);
        assert(get_max_number_of_experts() == max_num_motor_experts);
        assert(get_cur_number_of_experts() == num_initial_experts);

        assert(learning_rate > 0.);
        assert(growth_rate > 0.);
    }

    void execute_cycle(void) {
        motorspace.execute_cycle();
        gmes.execute_cycle();
    }

    void enable_learning(bool b) override { gmes.enable_learning(b); }
    void toggle_learning(void)   { gmes.enable_learning(not gmes.is_learning_enabled()); }
    double get_learning_progress(void) const override { return gmes.get_learning_progress(); }

    std::size_t get_state(void) const { return gmes.get_state(); }

    bool is_learning_enabled(void) const { return gmes.is_learning_enabled(); }

    /** This is somewhat ugly. But the only way I have figured out to do that. */
    const control::Control_Parameter& get_controller_weights(std::size_t id) const {
//        dbg_msg("Fetching controller weights.");
        Predictor_Base const& other = experts[id].get_predictor();
        Motor_Predictor const& motor_pred = dynamic_cast<Motor_Predictor const&>(other);
        return motor_pred.get_controller_weights();
    }


    std::size_t get_max_number_of_experts() const { return experts.size(); }
    std::size_t get_cur_number_of_experts() const { return gmes.get_number_of_experts(); }

    bool has_expert(std::size_t id) const { return experts[id].does_exists(); }

    void save(const std::string& foldername) const {
        const std::size_t num_experts = get_cur_number_of_experts();
        sts_msg("Saving %lu motor expert%s to folder %s", num_experts, (num_experts>1?"s":""), foldername.c_str());
        const std::string folder = basic::make_directory((foldername + "/motor").c_str());
        for (std::size_t i = 0; i < num_experts; ++i)
        {
            auto const& ctrl = get_controller_weights(i);
            ctrl.save_to_file(folder + "/motor_expert_" + std::to_string(i) + ".dat", i);
        }
    }

    void connect_payloads(static_vector<State_Payload>* s) {
        dbg_msg("Connecting payloads");
        for (std::size_t i = 0; i < payloads.size(); ++i)
            payloads[i].connect(i, s);
    }

    control::Control_Vector      params;
    static_vector<Motor_Payload> payloads;
    Motor_Space                  motorspace;
    Expert_Vector                experts;
    GMES                         gmes;

    friend class Motor_Layer_Graphics;
};



namespace constants {
    const unsigned subspace_num_datapoints = 200; // 2s of data at 100Hz
}

struct motor_subspace_graphics : public Graphics_Interface
{
    motor_subspace_graphics( robots::Joint_Model const& j0
                           , robots::Joint_Model const& j1
                           , Vector3 pos, float size
                           , ColorTable const& colortable )
    : j0(j0)
    , j1(j1)
    , axis_xy(pos.x + size/2        , pos.y + size/2, pos.z,     size, size, 0, std::string("xy"))
    , axis_dt(pos.x + (2.0 + size)/2, pos.y + size/2, pos.z, 2.0-size, size, 1, std::string(j0.name + "/" + j1.name))
    , plot_xy(constants::subspace_num_datapoints, axis_xy, colors::magenta )
    , plot_j0(constants::subspace_num_datapoints, axis_dt, colors::light0  )
    , plot_j1(constants::subspace_num_datapoints, axis_dt, colors::light1  )
    , plot_p0(constants::subspace_num_datapoints, axis_dt, colortable      )
    , plot_p1(constants::subspace_num_datapoints, axis_dt, colortable      )
    {
        dbg_msg("Initialize motor subspace for joints:\n\t%2u: %s\n\t%2u: %s", j0.joint_id, j0.name.c_str()
                                                                             , j1.joint_id, j1.name.c_str() );
    }

    void draw(const pref&) const {
        axis_xy.draw();
        plot_xy.draw();

        axis_dt.draw();
        plot_j0.draw(); // signals
        plot_j1.draw();
        plot_p0.draw_colored(); // predictions
        plot_p1.draw_colored();
    }

    void execute_cycle(float s0, float s1, unsigned expert_id) {
        plot_xy.add_sample(j0.motor.get(), j1.motor.get());
        plot_j0.add_sample(j0.motor.get());
        plot_j1.add_sample(j1.motor.get());
        plot_p0.add_colored_sample(s0, expert_id);
        plot_p1.add_colored_sample(s1, expert_id);
    }

    const robots::Joint_Model& j0;
    const robots::Joint_Model& j1;

    axes   axis_xy;
    axes   axis_dt;

    plot2D plot_xy;
    plot1D plot_j0, plot_j1; // joint motor commands
    colored_plot1D plot_p0, plot_p1; // predictions

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
    , colortable(5, /*randomized*/true)
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
                subspace.emplace_back(j1, j0, pos, size, colortable);
            }
        }
    }

    void draw(const pref& p) const {
        glColor3f(1.0, 1.0, 1.0);
        glprintf(1.05, 0.95, 0.0, 0.05, "no. exp. = %03u (%03u/%03u)", winner, num_experts, max_experts);

        auto const learning = motor_layer.is_learning_enabled();
        if (!learning) glColor3f(.3f,.3f,.3f);
        glprintf(1.05, 0.90, 0.0, 0.05, "learning: %s", learning ? "enabled" : "disabled");


        for (auto& s: subspace)
            s.draw(p);

        for (std::size_t i = 0; i < motor_layer.experts.size(); ++i)
        {
            if (winner == i) glColor3f(1.0, 0.5, 1.0);
            else if (motor_layer.experts[i].does_exists())
                glColor3f(.9f, .9f, .9f);
            else
                glColor3f(.3f,.3f,.3f);
            glprintf(-1.1, -1.5 - 0.05*i, 0.0, 0.04, "%2u" , i);

            auto const& col = motor_layer.experts[i].does_exists() ? colortable[i] : colors::gray;
            draw::hbar(-1.4, -1.5 - 0.05*i, 0.3, 0.02, 0.5* motor_layer.experts[i].get_learning_capacity(), gmes_constants::initial_learning_capacity, col);

            auto const& pred = motor_layer.experts[i].get_predictor();
            glPushMatrix();
            glTranslatef(0.0, -1.5 - 0.05*i , 0.0);
            pred.draw();
            glPopMatrix();
        }

    };

    void execute_cycle() {

        num_experts = motor_layer.gmes.get_number_of_experts(); // update number of experts
        winner = motor_layer.gmes.get_winner();
        auto const& predictions = motor_layer.experts[winner].get_predictor().get_prediction();

        for (auto& s: subspace) {
            auto s0 = predictions.at(s.j0.joint_id);
            auto s1 = predictions.at(s.j1.joint_id);
            s.execute_cycle(s0, s1, winner);
        }
    };

    Motor_Layer const& motor_layer;
    std::size_t        num_experts;
    std::size_t        max_experts;
    std::size_t        winner;

    std::vector<motor_subspace_graphics> subspace;
    ColorTable                           colortable;
};

} // namespace learning


#endif // MOTOR_LAYER_H_INCLUDED












