#include <tests/catch.hpp>
#include <tests/test_robot.h>

#include <common/modules.h>
#include <learning/homeokinesis.h>
#include <controller/csl_control.hpp>

namespace local_tests {

namespace homeokinesis_tests {

struct Test_Sensor_Space : public sensor_vector {
    Test_Sensor_Space(const robots::Jointvector_t& joints)
    : sensor_vector(2*joints.size() + 1)
    {
        for (robots::Joint_Model const& j : joints)
            sensors.emplace_back(j.name + "_ang", [&j](){ return j.s_ang; });
        for (robots::Joint_Model const& j : joints)
            sensors.emplace_back(j.name + "_vel", [&j](){ return j.s_vel; });

        sensors.emplace_back("bias", [&](){ return 0.01; });
        assert(sensors.size() == 2*joints.size() + 1);
    }

};



TEST_CASE( "homeokinetic controller construction + basic stuff" , "[homeokinesis]")
{
    srand(time(0)); // set random seed

    Test_Robot robot(5,2);
    robot.set_random_inputs(); // random initialize sensors

    Test_Sensor_Space sensors{robot.get_joints()};
    sensors.execute_cycle();

    std::vector<supreme::csl_control> csl(robot.get_joints().size());

    double random_range = 0.1;
    learning::Homeokinetic_Control homeoctrl(robot, sensors, csl, random_range, random_range);

    REQUIRE( homeoctrl.get_curr_state().size() == sensors.size() );
    REQUIRE( homeoctrl.get_next_state().size() == sensors.size() );

//    REQUIRE( homeoctrl.control_enabled == false );

    for (std::size_t i = 0; i < sensors.size(); ++i)
        REQUIRE ( sensors[i] != 0.0 );

    auto const& x0 = homeoctrl.get_curr_state();
    for (std::size_t i = 0; i < x0.size(); ++i)
        REQUIRE ( x0[i] != 0.0 );

    // check predictor and controller weights are random initialized
    auto const& weights = homeoctrl.pred.get_weights();
    double sum = .0;
    int diff = 0;
    for (std::size_t i = 0; i < weights.size(); ++i)
        for (std::size_t j = 0; j < weights[i].size(); ++j) {
            diff += ( weights[i][j] != .0 )? 0 : 1;
            sum += weights[i][j];
        }
    REQUIRE( diff == 0 );
    const double max_range = 0.5 * random_range * weights.size()*weights[0].size();
    dbg_msg("Max rand: %e < %e", std::abs(sum), max_range);
    REQUIRE( std::abs(sum) <= max_range ); // check small
    REQUIRE( std::abs(sum) != 0. ); // but not zero


    // check motor outputs are NON-ZERO
    sensors.execute_cycle();
    homeoctrl.execute_cycle();
    sts_add("[");
    for (auto const& e: homeoctrl.get_motor_data()) {
        sts_add("%+1.2f", e);
        REQUIRE ( e != 0.0 );
    }
    sts_msg("]");

}


} /* homeokinesis_tests */

} /* local_tests */

