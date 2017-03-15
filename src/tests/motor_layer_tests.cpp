#include <tests/catch.hpp>

#include <learning/motor_layer.h>
#include <tests/test_robot.h>

TEST_CASE(" motor layer ", "[Motor_Layer]") {

    Test_Robot  robot(5,2);
    learning::Motor_Layer motor(robot, 10, 0.01, 1.0, 1);

    for (unsigned int t = 0; t < 10; ++t) {
        robot.execute_cycle();
        motor.execute_cycle();
    }
}
