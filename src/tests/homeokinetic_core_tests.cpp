#include <tests/catch.hpp>
#include <tests/test_robot.h>

#include <common/log_messages.h>
#include <control/sensorspace.h>
#include <learning/predictor.h>
#include <learning/time_state_space.h>
#include <learning/homeokinetic_predictor.h>
#include <controller/pid_control.hpp>

TEST_CASE( "homeokinetic predictor construction", "[homeokinetic_core]" )
{
    typedef learning::Homeokinetic_Control::Vector_t ExtInputVector_t;
    typedef std::vector<supreme::pid_control> PID_joint_control_vector_t;

    Test_Robot robot(5,2);
    learning::Time_State_Space<8> inputs{robot};
    PID_joint_control_vector_t    pid;
    ExtInputVector_t              ext_input;

    //test_space sensors(0.01);
    learning::Homeokinetic_Core core(inputs, robot.get_joints().size(), /*learning_rate=*/0.01, /*random_weight_range=*/0.1, /*context=*/4);
}
