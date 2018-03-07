#include <tests/catch.hpp>
#include <vector>

#include <common/modules.h>
#include <common/log_messages.h>
#include <robots/robot.h>
#include <robots/joint.h>
#include <control/controlparameter.h>
#include <control/jointcontrol.h>
#include <tests/test_robot.h>


static void printf_param(control::Control_Parameter const& ctrl, robots::Robot_Interface const& robot) {
    auto num_inputs = control::get_number_of_inputs(robot);
    auto p = ctrl.get_parameter();
    unsigned j = 0;
    printf("%s ", (robot.get_joints().at(j).type == robots::Joint_Type_Symmetric)?"~":"=");
    for (std::size_t i = 0; i < p.size(); ++i) {
        if ((i > 0) and (i % num_inputs == 0)) printf("\n%s ", (robot.get_joints().at(++j).type == robots::Joint_Type_Symmetric)?"/":"=");
        printf("% 5.2f ", p[i]);
    }
    printf("\n");
}



TEST_CASE( "Make Asymmetric" , "[jointcontrol]") {

    std::vector<std::pair<unsigned, unsigned>> vec = {{3,1}, {5,2}, {4,2}, {3,0}};
    for (auto& s : vec)
    { /* LOOP OVER TESTCASE */

    std::size_t num_joints = s.first;
    std::size_t num_sym_joints = s.second;

    Test_Robot robot(num_joints, num_sym_joints);

    control::Control_Parameter asym_ctrl = control::get_initial_parameter(robot, {1.1, 2.2, 3.3}, false);

    REQUIRE( not asym_ctrl.is_symmetric() );
    REQUIRE( asym_ctrl.get_parameter().size() == control::get_number_of_inputs(robot) * num_joints );
    printf_param(asym_ctrl, robot);

    control::Control_Parameter sym_ctrl = control::make_symmetric(robot, asym_ctrl);
    REQUIRE( sym_ctrl.is_symmetric() );
    REQUIRE( sym_ctrl.get_parameter().size() == control::get_number_of_inputs(robot) * (num_joints-num_sym_joints) );
    if (num_sym_joints > 0)
        REQUIRE( sym_ctrl.get_parameter().size() < asym_ctrl.get_parameter().size() );
    printf_param(sym_ctrl, robot);

    control::Control_Parameter new_asym_ctrl = control::make_asymmetric(robot, sym_ctrl);
    REQUIRE( not new_asym_ctrl.is_symmetric() );
    REQUIRE( new_asym_ctrl.get_parameter().size() == asym_ctrl.get_parameter().size() );
    REQUIRE( new_asym_ctrl.get_parameter()        == asym_ctrl.get_parameter()        );
    if (num_sym_joints > 0)
        REQUIRE( new_asym_ctrl.get_parameter().size()  >  sym_ctrl.get_parameter().size() );
    printf_param(new_asym_ctrl, robot);

    control::Control_Parameter new_sym_ctrl = control::make_symmetric(robot, asym_ctrl);
    REQUIRE( new_sym_ctrl.is_symmetric() );
    REQUIRE( new_sym_ctrl.get_parameter().size() == sym_ctrl.get_parameter().size() );
    REQUIRE( new_sym_ctrl.get_parameter()        == sym_ctrl.get_parameter()        );
    printf_param(new_sym_ctrl, robot);

    } /* END TEST CASE LOOP */
}

TEST_CASE( "Making Asymmetric keeps structure" , "[jointcontrol]") {

    printf("\n____\nASYMMETRIC\n");

    std::vector<std::pair<unsigned, unsigned>> vec = {{4,2}, {8,4}, {4,0}, {5,2}, {3,1}};

    for (auto& s : vec)
    { /* LOOP OVER TESTCASE */
        printf("\n------------------------\n");

    const std::size_t num_joints = s.first;
    const std::size_t num_sym_joints = s.second;

    Test_Robot robot(num_joints, num_sym_joints);
    const std::size_t num_inputs = control::get_number_of_inputs(robot);

    const std::size_t num_sym_params = num_inputs*(num_joints-num_sym_joints);

    control::Fully_Connected_Symmetric_Core core(robot);
    robot.set_random_inputs();
    core.prepare_inputs(robot);

    printf("Inputs: \n");
    for (auto const& jx : robot.get_joints())
        printf("%5.2f %5.2f %5.2f\n", jx.s_ang, jx.s_vel, jx.motor.get_backed());
    printf("\n");


    //TODO control::Control_Parameter sym_ctrl = control::get_initial_parameter(robot, {1.1, 2.2, 3.3}, true);

    std::vector<double> rnd_parameter;


    for (std::size_t i = 0; i < num_sym_params; ++i)
        rnd_parameter.push_back(random_value());

    control::Control_Parameter sym_ctrl = control::Control_Parameter( rnd_parameter, /*symmetric*/true);



    /* randomize parameters */
    /*for (auto& p : sym_ctrl.set_parameter())
        p += random_value();*/

    REQUIRE( sym_ctrl.is_symmetric() );
    REQUIRE( sym_ctrl.get_parameter().size() == num_sym_params );
    core.apply_symmetric_weights(robot, sym_ctrl.get_parameter());
    core.update_outputs(robot, true, false);
    std::vector<double> sym_activation = core.activation;

    control::Control_Parameter asym_ctrl = control::make_asymmetric(robot, sym_ctrl);

    printf("\n sym: ");  sym_ctrl.print();
    printf("\n asm: "); asym_ctrl.print();

    REQUIRE( not asym_ctrl.is_symmetric() );

    if (num_sym_joints > 0)
        REQUIRE( asym_ctrl.get_parameter().size()  >  sym_ctrl.get_parameter().size() );

    core.prepare_inputs(robot);
    core.apply_weights(robot, asym_ctrl.get_parameter());
    core.update_outputs(robot, false, false);

    printf("\nActivations:");
    for (auto s: sym_activation) printf("%e ", s);
    printf("\n");

    REQUIRE( close(sym_activation, core.activation, 0.000001) );
    //printf("%e =?= %e", sym_activation, core.activation);

    printf_param( sym_ctrl, robot);
    printf("\nasymmetric\n");
    printf_param(asym_ctrl, robot);

    }

    printf("\n____\nDONE\n");
}

