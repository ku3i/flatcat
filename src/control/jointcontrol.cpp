#include "jointcontrol.h"

namespace control {


std::size_t get_number_of_inputs(robots::Robot_Interface const& robot) {
    return 3 * robot.get_number_of_joints() + 3 * robot.get_number_of_accel_sensors() + 1;
}



Jointcontrol::Jointcontrol(robots::Robot_Interface& robot)
: robot(robot) /* angle, velocity, motor output + xyz-acceleration + bias */
, number_of_inputs(get_number_of_inputs(robot))
, number_of_params_sym(number_of_inputs * (robot.get_number_of_joints() - robot.get_number_of_symmetric_joints()))
, number_of_params_asym(number_of_inputs * robot.get_number_of_joints())
, get_joints(robot.get_joints())
, set_joints(robot.set_joints())
, get_accels(robot.get_accels())
, set_accels(robot.set_accels())
, weights(robot.get_number_of_joints(), std::vector<double>(number_of_inputs, 0.0))
, X(number_of_inputs)
, Y(number_of_inputs)
, activation(robot.get_number_of_joints())
, initial_bias(0.1)
, symmetric_controller(false)
, is_switched(false)
{
    sts_msg("Creating joint controller.");
    if (robot.get_number_of_joints() < 1) err_msg(__FILE__, __LINE__, "No motor outputs.");
    if (0 == robot.get_number_of_accel_sensors()) wrn_msg("No use of acceleration sensors in controller.");

    sts_msg("Number of symmetric joints is %u.", robot.get_number_of_symmetric_joints());

    assert(weights   .size() == robot.get_number_of_joints());
    assert(weights[0].size() == number_of_inputs);

    sts_msg( "Created controller with: \n   %u inputs\n   %u outputs\n   %u symmetric params\n   %u asymmetric params."
           , number_of_inputs
           , robot.get_number_of_joints()
           , number_of_params_sym
           , number_of_params_asym);
    reset();
}


void
Jointcontrol::switch_symmetric(bool switched) {
    if (not symmetric_controller)
        is_switched = switched;
    else wrn_msg("Switching symmetry does not have any effect on symmetrical controller weights.");
}

void
Jointcontrol::set_control_parameter(const Control_Parameter& controller)
{
    if (controller.is_symmetric()) apply_symmetric_weights(controller.get_parameter());
    else apply_weights(controller.get_parameter());
    symmetric_controller = controller.is_symmetric();
}

void
Jointcontrol::set_control_parameter(const std::vector<double>& params) {
    //dbg_msg("Apply raw control parameter to %s controller.", symmetric_controller? "symmetric" : "asymmetric");
    if (symmetric_controller) apply_symmetric_weights(params);
    else apply_weights(params);
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
    return Control_Parameter(params, true);

}

std::size_t swap_sym_joint_pos(robots::Robot_Interface const& robot, std::size_t joint_id, std::size_t input_pos) {

    std::size_t sym_id = robot.get_joints()[joint_id].symmetric_joint;
    assert(sym_id < robot.get_number_of_joints());
    assert(input_pos < get_number_of_inputs(robot));

    if (joint_id == sym_id) /* not symmetric */
        return input_pos;

    if (input_pos >= 3 * robot.get_number_of_joints()) /* no joint pos, angle or torque */
        return input_pos;

    std::size_t jid = input_pos/3;
    std::size_t rem = input_pos%3;
    assert(jid < robot.get_number_of_joints());


    std::size_t result = 0;
    if (joint_id == jid)
        result = sym_id*3 + rem;
    else if (sym_id == jid)
        result = joint_id*3 + rem;
    else return input_pos; /* not the right id */

    assert(result < get_number_of_inputs(robot));
    assert(input_pos != result);
    dbg_msg("Transforming %u to %u", input_pos, result);
    return result;
}


Control_Parameter
make_asymmetric(robots::Robot_Interface const& robot, const Control_Parameter& other) {
    if (not other.is_symmetric()) {
        dbg_msg("Skipping, already asymmetric.");
        return other;
    }

    dbg_msg("Making controller asymmetric.");
    const std::size_t number_of_inputs = get_number_of_inputs(robot);

    /**TODO it must be asserted that joint IDs are unique and contiguous over the full range, different places, each constructor*/

    std::vector<std::vector<double> > temp_weights(robot.get_number_of_joints(), std::vector<double>(number_of_inputs, 0.0));

    const std::vector<double>& other_params = other.get_parameter();
    const std::size_t expected_other_size = number_of_inputs * (robot.get_number_of_joints() - robot.get_number_of_symmetric_joints());

    /* check the correct number of input params */
    //dbg_msg("num other params %u (%u)", other_params.size(), expected_other_size);
    assert(other_params.size() == expected_other_size);

    std::size_t param_index = 0;
    for (std::size_t ix = 0; ix < robot.get_number_of_joints(); ++ix)
        if (robot.get_joints()[ix].type == robots::Joint_Type_Normal)
        {
            std::size_t iy = robot.get_joints()[ix].symmetric_joint; // get symmetric counterpart of ix
            assert(iy < robot.get_number_of_joints());
            for (std::size_t k = 0; k < number_of_inputs; ++k)
            {
                temp_weights[ix][k] = other_params[param_index++];
                if (iy != ix) {
                    temp_weights[iy][swap_sym_joint_pos(robot, ix, k)] = temp_weights[ix][k];
                }
            }
        }
    assert(param_index == other_params.size());

    // get them out
    std::vector<double> params;
    params.reserve(number_of_inputs * robot.get_number_of_joints());
    for (std::size_t ix = 0; ix < robot.get_number_of_joints(); ++ix)
        for (std::size_t k = 0; k < number_of_inputs; ++k)
            params.push_back(temp_weights[ix][k]);

    assert(params.size() == number_of_inputs * robot.get_number_of_joints());

    return Control_Parameter(params, false);
}

/** this initializer is capable of reading a symmetric file and transform it to asymmetric */
Control_Parameter
initialize_anyhow(robots::Robot_Interface const& robot, Jointcontrol const& control, bool force_symmetric, const Minimal_Seed_t params_pdm, const std::string& filename ) {

    if (filename.empty())
        return get_initial_parameter(robot, params_pdm, force_symmetric);

    sts_msg("Reading seed from file: %s", filename.c_str());
    sts_msg("Trying to fetch %s controller.", force_symmetric? "a symmetric" : "an asymmetric");

    std::size_t num_params = force_symmetric ? control.get_number_of_symmetric_parameter() : control.get_number_of_parameter();

    Control_Parameter param0( filename, num_params, force_symmetric );
    if (param0.get_parameter().size() == num_params)
        return param0; /* success */

    /* try again with different symmetry */
    bool new_symmetry = not force_symmetric;

    num_params = new_symmetry ? control.get_number_of_symmetric_parameter() : control.get_number_of_parameter();

    Control_Parameter param1( filename, num_params, new_symmetry );
    if (param1.get_parameter().size() == num_params)
    {/* success, now force correct symmetry */

        if (force_symmetric) return make_symmetric(robot, param1);
        else return make_asymmetric(robot, param1);
    }
    else
        err_msg(__FILE__,__LINE__,"Could not read controller weights from file: %s.", filename.c_str());

    return Control_Parameter();
}

} // namespace control
