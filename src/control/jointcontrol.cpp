#include "jointcontrol.h"

namespace control {

std::size_t get_number_of_inputs(robots::Robot_Interface const& robot) {
    return 3 * robot.get_number_of_joints() + 3 * robot.get_number_of_accel_sensors() + 1;
}

void
Jointcontrol::reset(void)
{
    for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
        robot.set_joints()[i].motor = random_value(-0.01, 0.01);

    /* reset integrated velocities from acceleration sensors */
    for (std::size_t i = 0; i < robot.get_number_of_accel_sensors(); ++i)
        robot.set_accels()[i].reset();
}

void
Jointcontrol::loop(void)
{
    std::size_t index = 0;

    for (std::size_t ix = 0; ix < robot.get_number_of_joints(); ++ix)
    {
        //virt_ang[i] = clip(virt_ang[i] + x[i][0]);
        std::size_t iy = get_joints[ix].symmetric_joint;

        X[index]   = get_joints[ix].s_ang;
        Y[index++] = get_joints[iy].s_ang;

        X[index]   = get_joints[ix].s_vel;
        Y[index++] = get_joints[iy].s_vel;

        X[index]   = get_joints[ix].motor;
        Y[index++] = get_joints[iy].motor; //ganz wichtig!, bringt irre viel dynamic für den anfang
    }

    for (std::size_t i = 0; i < robot.get_number_of_accel_sensors(); ++i)
    {
        /* integrate velocities from acceleration sensors */
        set_accels[i].integrate();

        X[index]   = get_accels[i].v.x;
        Y[index++] = -get_accels[i].v.x; // mirror the x-axes

        X[index]   = get_accels[i].v.y;
        Y[index++] = get_accels[i].v.y;

        X[index]   = get_accels[i].v.z;
        Y[index++] = get_accels[i].v.z;
    }

    X[index]   = initial_bias; // bias
    Y[index++] = initial_bias;

    assert(index == number_of_inputs);
    assert(activation.size() == robot.get_number_of_joints());

    for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
    {
        activation[i] = .0;
        if (symmetric_controller and get_joints[i].type == robots::Joint_Type_Symmetric)
        {
            for (std::size_t k = 0; k < number_of_inputs; ++k)
                activation[i] += weights[i][k] * (is_switched ? X[k] : Y[k]);
        }
        else
        {
            for (std::size_t k = 0; k < number_of_inputs; ++k)
                activation[i] += weights[i][k] * (is_switched ? Y[k] : X[k]);
        }

        set_joints[(is_switched ? get_joints[i].symmetric_joint : i)].motor = clip(activation[i], 1.0);
    }
}

void
Jointcontrol::apply_symmetric_weights(const std::vector<double>& params)
{
    //dbg_msg("Apply symmetric weights.");
    assert(params.size() == number_of_params_sym);
    assert(weights.size() == robot.get_number_of_joints());
    assert(weights[0].size() == number_of_inputs);

    std::size_t param_index = 0;
    for (std::size_t ix = 0; ix < robot.get_number_of_joints(); ++ix)
    {
        if (get_joints[ix].type == robots::Joint_Type_Normal)
        {
            std::size_t iy = get_joints[ix].symmetric_joint; // get symmetric counterpart of ix
            assert(iy < robot.get_number_of_joints());
            for (std::size_t k = 0; k < number_of_inputs; ++k)
            {
                weights[ix][k] = params[param_index++];
                weights[iy][k] = weights[ix][k];
            }
        }
    }
    assert(param_index == params.size());
}

void
Jointcontrol::apply_weights(const std::vector<double>& params)
{
    //dbg_msg("Apply weights.");
    assert(params.size() == number_of_params_asym);
    assert(weights.size() == robot.get_number_of_joints());
    assert(weights[0].size() == number_of_inputs);

    std::size_t param_index = 0;
    for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
    {
        for (std::size_t k = 0; k < number_of_inputs; ++k)
            weights[i][k] = params[param_index++];
    }
    assert(param_index == params.size());
}

void
Jointcontrol::print_parameter(void) const
{
    sts_msg("Printing controller parameter:");
    printf("joints:%lu inputs:%lu\n", robot.get_number_of_joints(), number_of_inputs);
    /*print header*/
    printf(" #  |");
    for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
        if (!symmetric_controller or get_joints[i].type == robots::Joint_Type_Normal)
            printf("%4lu  |", i);
    printf("\n");

    for (std::size_t k = 0; k < number_of_inputs; ++k)
    {
        printf("%2lu: |", k);
        for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
        {
            if (!symmetric_controller or get_joints[i].type == robots::Joint_Type_Normal)
                printf("% 1.3f|", weights[i][k]);
        }
        printf("\n");
    }
    printf("\n");
}

double
Jointcontrol::get_normalized_mechanical_power(void) const
{
    //dbg_msg("Get normalized mechanical power.");
    double power = .0;
    for (unsigned int i = 0; i < robot.get_number_of_joints(); ++i)
        power += square(get_joints[i].motor);
    return power/robot.get_number_of_joints();
}

Control_Parameter
get_initial_parameter(robots::Robot_Interface const& robot, const Minimal_Seed_t& seed, bool symmetric = false)
{
    const std::size_t number_of_joints = robot.get_number_of_joints();
    const std::size_t number_of_inputs = get_number_of_inputs(robot);
    std::vector<double> params(number_of_joints*number_of_inputs);

    /* set default weights for asymmetric joints */
    for (std::size_t i = 0; i < number_of_joints; ++i)
    {
        const std::size_t pos = i*number_of_inputs + i * 3;

        params[pos + 0] = -seed.pgain;      // spring
        params[pos + 1] =  seed.damping;    // positive friction
        params[pos + 2] =  seed.motor_self; // motor neuron's self coupling

        /** Divide by initial bias, because bias in the control input will be < 1.
         *  If pgain or default_pos is zero this bias is also zero.
         */
        params[(i+1)*number_of_inputs - 1] = -params[pos + 0] * robot.get_joints()[i].default_pos * 1.0 / constants::initial_bias;
    }

    if (symmetric)
        return make_symmetric(robot, Control_Parameter(params));

    return Control_Parameter(params); /* asymmetric */
}

/**TODO: instead of throwing away the not used weights, we could average the corresponding weight pairs*/
Control_Parameter
make_symmetric(robots::Robot_Interface const& robot, const Control_Parameter& other) {
    if (other.is_symmetric())
        return other;

    dbg_msg("Making controller symmetric.");
    assert(robot.get_number_of_joints() > robot.get_number_of_symmetric_joints());
    const std::size_t num_joints = robot.get_number_of_joints() - robot.get_number_of_symmetric_joints();
    const std::size_t number_of_inputs = get_number_of_inputs(robot);
    const std::vector<double>& other_params = other.get_parameter();
    std::size_t p = 0;

    std::vector<double> params(number_of_inputs * num_joints);
    for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
    {
        if (robot.get_joints()[i].type == robots::Joint_Type_Normal)
            for (std::size_t k = 0; k < number_of_inputs; ++k)
                params[p++] = other_params[i*number_of_inputs + k];
    }
    assert(p == params.size());
    wrn_msg("Not tested yet.");
    return Control_Parameter(params, Control_Parameter::symmetric);

}

Control_Parameter
make_asymmetric(robots::Robot_Interface const& robot, const Control_Parameter& other) {
    if (not other.is_symmetric())
        return other;

    dbg_msg("Making controller asymmetric.");
    wrn_msg("Not tested yet.");

    const std::size_t number_of_inputs = get_number_of_inputs(robot);
    std::vector<double> params(number_of_inputs * robot.get_number_of_joints());
    const std::vector<double>& other_params = other.get_parameter();


    assert(params.size() >= other_params.size());
    for (std::size_t i = 0; i < other_params.size(); ++i)
        params[i] = other_params[i];

    /* copy weights for symmetric joints */
    for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
        if (robot.get_joints()[i].type == robots::Joint_Type_Symmetric)
        {
            std::size_t j = robot.get_joints()[i].symmetric_joint;
            for (std::size_t k = 0; k < number_of_inputs; ++k)
                params[i*number_of_inputs + k] = other_params[j*number_of_inputs + k];
        }

    return Control_Parameter(params, Control_Parameter::asymmetric);
}

/** this initializer is capable of reading a symmetric file and transform it to asymmetric */
Control_Parameter
initialize_anyhow(robots::Robot_Interface const& robot, Jointcontrol const& control, bool is_symmetric, const Minimal_Seed_t params_pdm, const std::string& filename ) {

    if (filename.empty())
        return get_initial_parameter(robot, params_pdm, is_symmetric);

    sts_msg("Reading seed from file.");
    Control_Parameter params( filename
                            , is_symmetric ? control.get_number_of_symmetric_parameter()
                                           : control.get_number_of_parameter()
                            , is_symmetric ? Control_Parameter::Symmetry::symmetric
                                           : Control_Parameter::Symmetry::asymmetric
                            , Control_Parameter::Propagation::original );

    if (not is_symmetric)
        return make_asymmetric(robot, Control_Parameter(params));

    return params;
}

} // namespace control
