#ifndef TEST_ROBOT_H_INCLUDED
#define TEST_ROBOT_H_INCLUDED

#include <robots/robot.h>

class Test_Robot : public robots::Robot_Interface {
public:

    Test_Robot(std::size_t num_joints, std::size_t num_sym_joints)
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

#endif // TEST_ROBOT_H_INCLUDED
