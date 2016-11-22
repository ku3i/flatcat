#include <tests/catch.hpp>
#include <vector>

#include <common/modules.h>
#include <common/log_messages.h>
#include <robots/robot.h>
#include <robots/joint.h>
#include <control/controlparameter.h>
#include <control/jointcontrol.h>



class test_robot : public robots::Robot_Interface {
public:

    test_robot(std::size_t num_joints, std::size_t num_sym_joints)
    : num_joints(num_joints)
    , num_sym_joints(num_sym_joints)
    , joints()
    , accels(0)
    {
        for (unsigned i = 0; i < num_joints; ++i)
            joints.emplace_back(i, robots::Joint_Type_Normal, i, "joint"+std::to_string(i), -1.0, +1.0, 0.0);

        /* assign symmetric joints */
        REQUIRE( num_joints >= num_sym_joints*2 );
        for (unsigned i = 0; i < num_sym_joints*2; i+=2) {
            joints[i  ].type = robots::Joint_Type_Normal;
            joints[i+1].type = robots::Joint_Type_Symmetric;
            joints[i  ].symmetric_joint = i + 1;
            joints[i+1].symmetric_joint = i;
            REQUIRE( i < joints.size() );
        }
        for (unsigned i = num_sym_joints*2; i < num_joints; ++i) {
            joints[i].type = robots::Joint_Type_Normal;
            joints[i].symmetric_joint = i;
            REQUIRE( i < joints.size() );
        }

        for (unsigned i = 0; i < num_joints; ++i) {
            dbg_msg("creating joint jID = %u symID= %u type = %s", joints[i].joint_id
                                                                 , joints[i].symmetric_joint
                                                                 , joints[i].type==robots::Joint_Type_Normal? "normal":"symmetric");
        }
    }

    std::size_t get_number_of_joints           (void) const { return num_joints; }
    std::size_t get_number_of_symmetric_joints (void) const { return num_sym_joints; }
    std::size_t get_number_of_accel_sensors    (void) const { return 0; }

    const robots::Jointvector_t& get_joints(void) const { return joints; }
          robots::Jointvector_t& set_joints(void)       { return joints; }

    const robots::Accelvector_t& get_accels(void) const { return accels; }
          robots::Accelvector_t& set_accels(void)       { return accels; }

    bool execute_cycle(void) { return true; }

    double get_normalized_mechanical_power(void) const { return .0; }

    std::size_t num_joints;
    std::size_t num_sym_joints;
    robots::Jointvector_t joints;
    robots::Accelvector_t accels;

};


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

    test_robot robot(num_joints, num_sym_joints);

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
