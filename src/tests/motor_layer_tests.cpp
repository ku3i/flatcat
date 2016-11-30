#include <tests/catch.hpp>

#include <learning/motor_layer.h>
#include <tests/test_robot.h>

TEST_CASE(" motor layer ", "[Motor_Layer]") {


    Test_Robot  robot(5,2);
    learning::Motor_Layer motor(robot, 5);

}
