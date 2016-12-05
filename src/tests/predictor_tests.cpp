#include <tests/catch.hpp>

#include <control/sensorspace.h>
#include <learning/predictor.h>
#include <common/log_messages.h>

class test_space : public sensor_vector {
public:
    test_space(const double sigma) : sensor_vector(3) {
        sensors.emplace_back("Foo", [sigma](){ return  0.42 + rand_norm_zero_mean(sigma); });
        sensors.emplace_back("Bar", [sigma](){ return  0.37 + rand_norm_zero_mean(sigma); });
        sensors.emplace_back("Baz", [sigma](){ return -0.23 + rand_norm_zero_mean(sigma); });
    }
};

TEST_CASE( "adapt" , "[predictor]")
{
    test_space sensors{0.0};
    Predictor pred{ sensors, 0.1, 0.01, 1 };

    const std::vector<double>& w = pred.get_weights();
    REQUIRE( w.size() == 3 );

    for (unsigned i = 0; i < 100; ++i) {
        sensors.execute_cycle();
        pred.adapt();
        const std::vector<double>& w = pred.get_weights();
    }
    REQUIRE( close(w[0], 0.42, 0.001) );
    REQUIRE( close(w[1], 0.37, 0.001) );
    REQUIRE( close(w[2],-0.23, 0.001) );

    dbg_msg("%6.4f %6.4f %6.4f", w[0], w[1], w[2]);
}

TEST_CASE( "adapt with experience replay" , "[predictor]")
{
    srand((unsigned) time(0));
    test_space sensors(0.01); /** with random */
    Predictor pred{ sensors, 0.1, 0.01, 100 };

    const std::vector<double>& w = pred.get_weights();
    REQUIRE( w.size() == 3 );

    for (unsigned i = 0; i < 1000; ++i) {
        sensors.execute_cycle();
        pred.adapt();
        //const std::vector<double>& w = pred.get_weights();
        //dbg_msg("%6.4f %6.4f %6.4f", sensors[0], sensors[1], sensors[2]);
    }
    REQUIRE( close(w[0], 0.42, 0.1) );
    REQUIRE( close(w[1], 0.37, 0.1) );
    REQUIRE( close(w[2],-0.23, 0.1) );

    dbg_msg("%6.4f %6.4f %6.4f", w[0], w[1], w[2]);
}
