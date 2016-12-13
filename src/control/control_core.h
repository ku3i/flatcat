#ifndef CONTROL_CORE_H_INCLUDED
#define CONTROL_CORE_H_INCLUDED

#include <vector>
#include <common/log_messages.h>
#include <robots/robot.h>


namespace control {

namespace constants {
    const double initial_bias = 0.1;
}

inline std::size_t get_number_of_inputs(robots::Robot_Interface const& robot) {
    /* angle, velocity, motor output + xyz-acceleration + bias */
    return 3 * robot.get_number_of_joints() + 3 * robot.get_number_of_accel_sensors() + 1;
}

struct sym_input { double x,y; };

class Fully_Connected_Symmetric_Core
{
public:
    const std::size_t                 num_inputs, num_outputs;
    std::vector<std::vector<double> > weights;
    std::vector<sym_input>            input;
    std::vector<double>               activation;


    Fully_Connected_Symmetric_Core(robots::Robot_Interface const& robot)
    : num_inputs(get_number_of_inputs(robot))
    , num_outputs(robot.get_number_of_joints())
    , weights(num_outputs, std::vector<double>(num_inputs, 0.0))
    , input(num_inputs)
    , activation(num_outputs)
    {
        dbg_msg("Creating fully connected symmetric control core.");
        assert(num_inputs > 0);
        assert(num_outputs > 0);
        assert(input.size() == num_inputs);
    }


    void prepare_inputs(const robots::Robot_Interface& robot)
    {
        std::size_t index = 0;
        for (auto const& jx : robot.get_joints())
        {
            auto const& jy = robot.get_joints()[jx.symmetric_joint];

            /**TODO consider using a virtual (integrated) angle */
            input[index++] = {jx.s_ang, jy.s_ang};
            input[index++] = {jx.s_vel, jy.s_vel};
            input[index++] = {jx.motor, jy.motor};
        }

        for (auto const& a : robot.get_accels())
        {
            input[index++] = {a.v.x, -a.v.x}; // mirror the x-axes
            input[index++] = {a.v.y, a.v.y};
            input[index++] = {a.v.z, a.v.z};
        }

        input[index++] = {constants::initial_bias, constants::initial_bias};
        assert(index == input.size());
    }

    void update_outputs(const robots::Robot_Interface& robot, bool is_symmetric, bool is_switched)
    {
        assert(input.size() == weights[0].size());
        assert(activation.size() == robot.get_number_of_joints());
        for (std::size_t i = 0; i < activation.size(); ++i)
        {
            activation[i] = .0;
            bool swap_inputs = is_switched == (is_symmetric and robot.get_joints()[i].type == robots::Joint_Type_Symmetric);

            for (std::size_t k = 0; k < input.size(); ++k)
                activation[i] += weights[i][k] * (swap_inputs ? input[k].x : input[k].y);
        }
    }

    void write_motors(robots::Robot_Interface& robot, bool is_switched)
    {
        robots::Jointvector_t& joints = robot.set_joints();
        assert(activation.size() == joints.size());

        for (std::size_t i = 0; i < activation.size(); ++i)
            joints[(is_switched ? joints[i].symmetric_joint : i)].motor = clip(activation[i], 1.0);
    }

    void apply_weights(robots::Robot_Interface const& robot, std::vector<double> const& params)
    {
        dbg_msg("Apply weights.");
        assert(params.size() == weights.size() * weights.at(0).size());
        std::size_t param_index = 0;
        for (auto& w_i : weights)
            for (auto& w_ik : w_i)
                w_ik = params[param_index++];

        assert(param_index == params.size());
    }

    void apply_symmetric_weights(robots::Robot_Interface const& robot, std::vector<double> const& params)
    {
        dbg_msg("Apply symmetric weights.");
        robots::Jointvector_t const& joints = robot.get_joints();

        std::size_t param_index = 0;
        for (std::size_t ix = 0; ix < robot.get_number_of_joints(); ++ix)
        {
            if (joints[ix].type == robots::Joint_Type_Normal)
            {
                std::size_t iy = joints[ix].symmetric_joint; // get symmetric counter part of ix
                assert(iy < robot.get_number_of_joints());
                for (std::size_t k = 0; k < weights[ix].size(); ++k)
                {
                    weights[ix][k] = params[param_index++];
                    weights[iy][k] = weights[ix][k];
                }
            }
        }
        assert(param_index == params.size());
    }

};


} // namespace control

#endif // CONTROL_CORE_H_INCLUDED
