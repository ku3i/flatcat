#include "jointcontrol.h"

namespace control {


Jointcontrol::Jointcontrol(robots::Robot_Interface& robot)
: robot(robot)
, core(robot)
, number_of_params_sym(get_number_of_inputs(robot) * (robot.get_number_of_joints() - robot.get_number_of_symmetric_joints()))
, number_of_params_asym(get_number_of_inputs(robot) * robot.get_number_of_joints())
, symmetric_controller(false)
, is_switched(false)
{
    sts_msg("Creating joint controller.");
    if (robot.get_number_of_joints() < 1) err_msg(__FILE__, __LINE__, "No motor outputs.");
    if (0 == robot.get_number_of_accel_sensors()) wrn_msg("No use of acceleration sensors in controller.");

    sts_msg("Number of symmetric joints is %u.", robot.get_number_of_symmetric_joints());

    sts_msg( "Created controller with: \n   %u inputs\n   %u outputs\n   %u symmetric params\n   %u asymmetric params."
           , get_number_of_inputs(robot)
           , robot.get_number_of_joints()
           , number_of_params_sym
           , number_of_params_asym);
    reset();
}


void
Jointcontrol::switch_symmetric(bool switched)
{
    if (not symmetric_controller)
        is_switched = switched;
    else
        is_switched = false;
    /* switching symmetry does not have any effect on symmetrical controller weights */
}


void
Jointcontrol::set_control_parameter(const Control_Parameter& controller)
{
    if (controller.is_symmetric()) apply_symmetric_weights(controller.get_parameter());
    else apply_weights(controller.get_parameter());
    symmetric_controller = controller.is_symmetric();

    switch_symmetric(controller.is_mirrored());
}


void
Jointcontrol::set_control_parameter(const std::vector<double>& params)
{
    if (symmetric_controller) apply_symmetric_weights(params);
    else apply_weights(params);
}


void
Jointcontrol::reset(void)
{
    for (auto& j : robot.set_joints()) j.motor.set( random_value(-0.01, 0.01) );
    for (auto& a : robot.set_accels()) a.reset(); // reset integrated velocities from acceleration sensors
}


void
Jointcontrol::insert_motor_command(unsigned index, double value)
{
    assert(index < robot.set_joints().size());
    auto& j = robot.set_joints()[index];
    j.motor.set(robot.get_joints()[index].motor.get() + value);
}


void
Jointcontrol::integrate_accels(void)
{
    for (auto& a : robot.set_accels()) a.integrate(); // integrate velocities from acceleration sensors
}


void
Jointcontrol::execute_cycle(void)
{
    integrate_accels();
    core.prepare_inputs(robot);
    core.update_outputs(robot, symmetric_controller, is_switched);
    core.write_motors  (robot, is_switched);
}


void
Jointcontrol::apply_symmetric_weights(const std::vector<double>& params)
{
    assert(params.size() == number_of_params_sym);
    core.apply_symmetric_weights(robot, params);
}

void
Jointcontrol::apply_weights(const std::vector<double>& params)
{
    assert(params.size() == number_of_params_asym);
    core.apply_weights(robot, params);
}

void
Jointcontrol::print_parameter(void) const
{
    robots::Jointvector_t const& joints = robot.get_joints();

    sts_msg("Printing controller parameter:");
    printf("joints:%zu inputs:%zu\n", robot.get_number_of_joints(), get_number_of_inputs(robot));
    /*print header*/
    printf(" #  |");
    for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
        if (!symmetric_controller or joints[i].type == robots::Joint_Type_Normal)
            printf("%4zu  |", i);
    printf("\n");

    for (std::size_t k = 0; k < get_number_of_inputs(robot); ++k)
    {
        printf("%2zu: |", k);
        for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
        {
            if (!symmetric_controller or joints[i].type == robots::Joint_Type_Normal)
                printf("% 1.3f|", core.weights[i][k]);
        }
        printf("\n");
    }
    printf("\n");
}

double
Jointcontrol::get_normalized_mechanical_power(void) const
{
    double power = .0;
    for (auto const& j : robot.get_joints())
        power += square(j.motor.get());
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

bool is_sagittal_accel_sensor(robots::Robot_Interface const& robot, std::size_t input_pos) {
    return (input_pos == 3 * robot.get_number_of_joints());
}

std::size_t swap_sym_joint_pos(robots::Robot_Interface const& robot, std::size_t joint_id, std::size_t input_pos) {

    std::size_t sym_id = robot.get_joints()[joint_id].symmetric_joint;
    assert(sym_id < robot.get_number_of_joints());
    assert(input_pos < get_number_of_inputs(robot));

    if (joint_id == sym_id){ /* not symmetric */
        //dbg_msg("1) not changed %u", input_pos);
        return input_pos;
    }

    if (input_pos >= 3 * robot.get_number_of_joints()){ /* no joint pos, angle or torque */
        //dbg_msg("2) not changed %u", input_pos);
        return input_pos;
    }

    std::size_t jid = input_pos/3;
    std::size_t rem = input_pos%3;
    assert(jid < robot.get_number_of_joints());

    std::size_t result = 0;
    sym_id = robot.get_joints()[jid].symmetric_joint;
    result = sym_id*3 + rem;

    //dbg_msg("Transforming %u to %u", input_pos, result);
    assert(result < get_number_of_inputs(robot));

    return result;
}


Control_Parameter
make_asymmetric(robots::Robot_Interface const& robot, const Control_Parameter& other) {
    if (not other.is_symmetric()) {
        //dbg_msg("Skipping, already asymmetric.");
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
            assert(robot.get_joints()[iy].symmetric_joint == ix);
            if (ix!=iy)
                assert(robot.get_joints()[iy].type == robots::Joint_Type_Symmetric);
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
    for (std::size_t ix = 0; ix < robot.get_number_of_joints(); ++ix) {
        printf("\n");
        for (std::size_t k = 0; k < number_of_inputs; ++k) {
            params.push_back(temp_weights[ix][k]);
            printf("  %+5.2f", temp_weights[ix][k]);
        }
    }
    printf("\n");

    other.print();

    assert(params.size() == number_of_inputs * robot.get_number_of_joints());

    return Control_Parameter(params, false);
}


Control_Parameter
turn_symmetry(robots::Robot_Interface const& robot, const Control_Parameter& other) {
    if (other.is_symmetric() or !other.is_mirrored()) {
        dbg_msg("Nothing to turn to original.");
        return other; /* nothing to do*/
    }

    dbg_msg("Turning controller to original.");
    Control_Parameter orig = other; // make a safe copy
    auto& params = orig.set_parameter();
    orig.print();
    printf("\n");
    std::size_t number_of_inputs = get_number_of_inputs(robot);

    /* copy params to temp weight matrix */
    std::vector<std::vector<double> > weights{ robot.get_number_of_joints()
                                              , std::vector<double>(number_of_inputs, 0.0)};
    std::size_t p = 0;
        for (auto& w_i : weights)
            for (auto& w_ik : w_i)
                w_ik = params[p++];
    assert(p == params.size());

    p = 0;
    for (std::size_t ix = 0; ix < robot.get_number_of_joints(); ++ix) {
        for (std::size_t k = 0; k < number_of_inputs; ++k) {
            unsigned j = swap_sym_joint_pos(robot, ix, k);
            int sign = is_sagittal_accel_sensor(robot,k)? -1:1;
            params[p++] = sign*weights[robot.get_joints()[ix].symmetric_joint][j];
        }
    }

    assert(p == params.size());

    orig.print();

    return Control_Parameter(params, false, false);
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

Control_Vector param_factory( const robots::Robot_Interface& robot
                            , std::size_t number_of_motor_units
                            , const std::string& folder
                            , const control::Minimal_Seed_t& seed )
{
    sts_msg("Launching motor control parameter vector factory.");
    control::Control_Vector params(number_of_motor_units, folder);

    sts_msg("Number of elements loaded from files: %u", params.size());
    assert(number_of_motor_units>=params.size());

    if (number_of_motor_units > params.size()) {
        sts_msg("Added the rest (%u) as randomized values.", number_of_motor_units - params.size());
        for (std::size_t i = params.size(); i < number_of_motor_units; ++i) {
            sts_msg("Create randomized motor control parameter no.: %u", i);
            control::Control_Parameter p = control::get_initial_parameter(robot, seed, false/*symmetric?*/);//(i % 2 == 0));
            control::randomize_control_parameter(p, 0.1, 1.0);
                /**TODO make random parameters to settings, and constrain motor self not not go beyond zero */
                /**TODO also: make settings grouped and only give the local settings as ref */
            params.add(p);
        }
    }
    assert(params.size() == number_of_motor_units);
    sts_msg("Done creating %u motor control vectors.", params.size());
    return params;
}

double Jointcontrol::get_L1_norm(void)
{
    /** think about: shall we exclude bias weights*/
    double sum_abs_weights = .0;
    for (auto const& wi: core.weights)
        for (auto const& wik: wi)
            sum_abs_weights += std::abs(wik);
    return sum_abs_weights/number_of_params_asym;
}

} // namespace control
