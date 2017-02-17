#include <tests/catch.hpp>
#include <tests/test_robot.h>

#include <common/modules.h>
#include <learning/autoencoder.h>


namespace local_tests {

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



TEST_CASE( "autoencoder construction" , "[autoencoder]")
{
    srand(time(0)); // set random seed

    Test_Robot robot(5,2);
    Test_Sensor_Space inputs{robot.get_joints()};
    const double random_range = 0.1;
    learning::Autoencoder autoenc(inputs.size(), 3, random_range);

    auto const& outputs = autoenc.get_outputs();
    REQUIRE( outputs.size() == inputs.size() );

    for (std::size_t i = 0; i < outputs.size(); ++i) {
        REQUIRE( outputs[i] == .0 );
        REQUIRE( inputs[i] == .0 );
    }

    autoenc.propagate(inputs);

    for (std::size_t i = 0; i < outputs.size(); ++i)
        REQUIRE( outputs[i] == .0 );

    /* check weights are not zero */
    auto const& weights = autoenc.get_weights();
    double sum = .0;
    int diff = 0;
    for (std::size_t i = 0; i < weights.size(); ++i)
        for (std::size_t j = 0; j < weights[i].size(); ++j) {
            diff += ( weights[i][j] != .0 )? 0 : 1;
            sum += weights[i][j];
        }

    /* check randomize_weight_matrix() is executed */
    REQUIRE( diff == 0 );
    const double max_range = 0.5* random_range * weights.size()*weights[0].size();
    dbg_msg("Max rand: %e < %e", std::abs(sum), max_range);
    REQUIRE( std::abs(sum) <= max_range ); // check small
    REQUIRE( std::abs(sum) != 0. ); // but not zero


    /** check that autoencoder is copyable **/
    learning::Autoencoder autoenc2 = autoenc;
}


TEST_CASE( "auto encoder learning", "[autoencoder]")
{
    srand(time(0)); // set random seed

    Test_Robot robot(5,2);
    Test_Sensor_Space inputs{robot.get_joints()};

    for (auto& j: robot.set_joints())
        j.s_ang = 1.0;

    const double learning_rate = 0.01;
    learning::Autoencoder autoenc(inputs.size(), 3, 0.1);
    REQUIRE ( squared_distance(inputs, autoenc.get_outputs()) == .0 );

    for (std::size_t trials = 0; trials < 10; ++trials) {
        inputs.execute_cycle();
        autoenc.propagate(inputs);
        double err0 = squared_distance(inputs, autoenc.get_outputs());
        autoenc.adapt(inputs, learning_rate);
        autoenc.propagate(inputs);
        double err1 = squared_distance(inputs, autoenc.get_outputs());

        dbg_msg("Prediction Error before %e and %e after adaption.", err0, err1);
        REQUIRE( err0 > err1 );
    }
}

} // namespace local_tests

