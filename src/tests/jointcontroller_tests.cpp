#include <tests/catch.hpp>
#include <vector>

#include <common/modules.h>
#include <common/log_messages.h>
#include <robots/robot.h>
#include <robots/joint.h>
#include <control/controlparameter.h>
#include <control/jointcontrol.h>
#include <tests/test_robot.h>


TEST_CASE( "Make Asymmetric" , "[jointcontrol]") {

    std::vector<std::pair<unsigned, unsigned>> vec = {{3,1}, {5,2}, {4,2}, {3,0}};
    for (auto& s : vec)
    { /* LOOP OVER TESTCASE */

    std::size_t num_joints = s.first;
    std::size_t num_sym_joints = s.second;

    auto printf_param = [](control::Control_Parameter const& ctrl, robots::Robot_Interface const& robot) {
        auto num_inputs = control::get_number_of_inputs(robot);
        auto p = ctrl.get_parameter();
        for (std::size_t i = 0; i < p.size(); ++i) {
            if ((i > 0) and (i % num_inputs == 0)) printf("\n");
            printf("% .1f ", p[i]);

        }
        printf("\n");
    };

    Test_Robot robot(num_joints, num_sym_joints);

    control::Control_Parameter asym_ctrl = control::get_initial_parameter(robot, {1.1, 2.2, 3.3}, false);
    REQUIRE( not asym_ctrl.is_symmetric() );
    REQUIRE( asym_ctrl.get_parameter().size() == control::get_number_of_inputs(robot) * num_joints );
    printf_param(asym_ctrl, robot);

    control::Control_Parameter sym_ctrl = control::make_symmetric(robot, asym_ctrl);
    REQUIRE( sym_ctrl.is_symmetric() );
    REQUIRE( sym_ctrl.get_parameter().size() == control::get_number_of_inputs(robot) * (num_joints-num_sym_joints) );
    printf_param(sym_ctrl, robot);

    control::Control_Parameter new_asym_ctrl = control::make_asymmetric(robot, sym_ctrl);
    REQUIRE( not new_asym_ctrl.is_symmetric() );
    REQUIRE( new_asym_ctrl.get_parameter().size() == asym_ctrl.get_parameter().size() );
    REQUIRE( new_asym_ctrl.get_parameter()        == asym_ctrl.get_parameter()        );
    printf_param(new_asym_ctrl, robot);

    control::Control_Parameter new_sym_ctrl = control::make_symmetric(robot, asym_ctrl);
    REQUIRE( new_sym_ctrl.is_symmetric() );
    REQUIRE( new_sym_ctrl.get_parameter().size() == sym_ctrl.get_parameter().size() );
    REQUIRE( new_sym_ctrl.get_parameter()        == sym_ctrl.get_parameter()        );
    printf_param(new_sym_ctrl, robot);

    } /* END TEST CASE LOOP */
}
