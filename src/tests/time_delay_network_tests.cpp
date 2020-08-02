#include <tests/catch.hpp>
#include <tests/test_robot.h>

#include <common/modules.h>
#include <learning/time_delay_network.h>


namespace local_tests {
namespace time_delay_network_tests {


struct TD_Sensor_Space : public sensor_vector {
    TD_Sensor_Space(const robots::Jointvector_t& joints)
    : sensor_vector(3*joints.size() + 1)
    {
        for (robots::Joint_Model const& j : joints)
            sensors.emplace_back(j.name + "_ang", [&j](){ return j.s_ang; });
        for (robots::Joint_Model const& j : joints)
            sensors.emplace_back(j.name + "_vel", [&j](){ return j.s_vel; });
        for (robots::Joint_Model const& j : joints)
            sensors.emplace_back(j.name + "_mot", [&j](){ return j.motor.get_backed(); });

        sensors.emplace_back("bias", [&](){ return 0.13; });
        assert(sensors.size() == 3*joints.size() + 1);
    }
};


TEST_CASE( "FIR Synapse Test" , "[Time Delay Network]")
{

    Test_Robot robot(1,0);
    TD_Sensor_Space inputs{robot.get_joints()};

    const unsigned num_taps = 3;

    learning::FIR_type_synapse td_input(inputs.size(), num_taps);

    for (auto& j: robot.set_joints()) {
        j.s_ang = 1.1;
        j.s_vel = 2.2;
        j.motor = 3.3;
        j.motor.transfer();
    }

    inputs.execute_cycle();
    td_input.propagate(inputs);

    REQUIRE( td_input.get().size() == num_taps*inputs.size() );

    auto const& vec = td_input.get();

//    for (auto const& v : vec)
//        printf("%5.2f ", v);

    auto const& target = std::vector<double>{1.1, 2.2, 3.3, 0.13, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    REQUIRE( vec == target );

    for (auto& j: robot.set_joints()) {
        j.s_ang = 1.0;
        j.s_vel = 2.0;
        j.motor = 3.0;
        j.motor.transfer();
    }

    inputs.execute_cycle();
    td_input.propagate(inputs);
    auto const& target2 = std::vector<double>{1.0, 2.0, 3.0, 0.13, 1.1, 2.2, 3.3, 0.13, 0.0, 0.0, 0.0, 0.0};
    REQUIRE( vec == target2 );

    inputs.execute_cycle();
    td_input.propagate(inputs);
    auto const& target3 = std::vector<double>{1.0, 2.0, 3.0, 0.13, 1.0, 2.0, 3.0, 0.13, 1.1, 2.2, 3.3, 0.13};
    REQUIRE( vec == target3 );
}


TEST_CASE( "time delay network construction" , "[Time Delay Network]")
{
    srand(time(0)); // set random seed

    Test_Robot robot(5,2);
    robot.set_random_inputs();
    TD_Sensor_Space inputs{robot.get_joints()};
    const double random_range = 0.1;
    learning::Timedelay_Network tdn(inputs.size(), inputs.size(), 3, 10, random_range);

    auto const& outputs = tdn.get_outputs();
    REQUIRE( outputs.size() == inputs.size() );

    for (std::size_t i = 0; i < outputs.size(); ++i) {
        REQUIRE( outputs[i] == .0 );
        REQUIRE( inputs[i] == .0 );
    }

    tdn.propagate_and_shift(inputs);

    for (std::size_t i = 0; i < outputs.size(); ++i)
        REQUIRE( outputs[i] == .0 );

    /* check weights are not zero */
    auto const& w1 = tdn.get_weights().hi;
    double sum = .0;
    int diff = 0;
    for (std::size_t i = 0; i < w1.size(); ++i)
        for (std::size_t j = 0; j < w1[i].size(); ++j) {
            diff += ( w1[i][j] != .0 )? 0 : 1;
            sum += w1[i][j];
        }
    auto const& w2 = tdn.get_weights().oh;
    sum = .0;
    diff = 0;
    for (std::size_t i = 0; i < w2.size(); ++i)
        for (std::size_t j = 0; j < w2[i].size(); ++j) {
            diff += ( w2[i][j] != .0 )? 0 : 1;
            sum += w2[i][j];
        }

    /* check randomize_weight_matrix() is executed */
    REQUIRE( diff == 0 );
    const double max_range = 0.5* random_range * w1.size()*w1[0].size();
    dbg_msg("Max rand: %e < %e", std::abs(sum), max_range);
    REQUIRE( std::abs(sum) <= max_range ); // check small
    REQUIRE( std::abs(sum) != 0. ); // but not zero


    /** check that autoencoder is copyable **/
    learning::Timedelay_Network tdn2 = tdn;
}


TEST_CASE( "time delay network learning", "[Time Delay Network]")
{
    srand(time(0)); // set fixed seed

    Test_Robot robot(5,2);
    TD_Sensor_Space inputs{robot.get_joints()};

    robot.set_random_inputs();
    const unsigned number_of_taps = 10;

    const double learning_rate = 0.02;
    learning::Timedelay_Network tdn(inputs.size(), inputs.size(), 3, number_of_taps, 0.1);

    SECTION( "vector_tanh computes tanh element-wise") {
        std::vector<double> vec = {1.1, -5.0, 10.0};
        vector_tanh(vec);
        REQUIRE( not is_vector_zero(vec) );
        REQUIRE( vec[0] == tanh( 1.1) );
        REQUIRE( vec[1] == tanh(-5.0) );
        REQUIRE( vec[2] == tanh(10.0) );
    }

    SECTION(" outputs are zero at initialization time.") {

        REQUIRE( is_vector_zero(tdn.get_hidden()) );
        REQUIRE( is_vector_zero(tdn.get_outputs()) );

        inputs.execute_cycle();
        tdn.propagate_and_shift(inputs);

        print_vector(tdn.get_hidden() ,"hidden");
        print_vector(tdn.get_outputs(),"output");

        REQUIRE( not is_vector_zero(tdn.get_hidden()) );
        REQUIRE( not is_vector_zero(tdn.get_outputs()) );

        inputs.execute_cycle();
        tdn.propagate_and_shift(inputs);

        REQUIRE( not is_vector_zero(tdn.get_hidden()) );
        REQUIRE( not is_vector_zero(tdn.get_outputs()) );
    }

    SECTION(" reducing squared distance over training time.") {

        /* shift all inputs */
        for (std::size_t tap = 0; tap < number_of_taps; ++tap) {
            inputs.execute_cycle();
            tdn.propagate_and_shift(inputs);
        }

        /* adapt without shifting and see if error drops */
        sts_msg("Error before training: %e ", squared_distance(inputs, tdn.get_outputs()));

        std::size_t max_trials = 10000;
        std::size_t trials = 0;
        double err1,err0;
        for (; trials < max_trials; ++trials) {
            tdn.propagate(); /** without shifting */
            err0 = squared_distance(inputs, tdn.get_outputs());
            tdn.adapt(inputs, learning_rate);
            tdn.propagate();
            err1 = squared_distance(inputs, tdn.get_outputs());

            if (err0 <= err1) {
                dbg_msg("Prediction Error before %e and %e after adaption.", err0, err1);
                break;
            }
        }
        sts_msg("Error after training of %lu trials: %e ",trials,squared_distance(inputs, tdn.get_outputs()));
        if (err1 > 10e-25)
            REQUIRE( (trials == max_trials) );
        else REQUIRE( true );

    }
}

}} // namespace local_tests


