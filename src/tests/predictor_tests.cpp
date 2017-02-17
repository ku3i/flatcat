#include <tests/catch.hpp>

#include <control/sensorspace.h>
#include <learning/predictor.h>
#include <learning/state_predictor.h>
#include <common/log_messages.h>

class test_space : public sensor_vector {
public:
    test_space(const double sigma) : sensor_vector(3) {
        sensors.emplace_back("Foo", [sigma](){ return  0.42 + rand_norm_zero_mean(sigma); });
        sensors.emplace_back("Bar", [sigma](){ return  0.37 + rand_norm_zero_mean(sigma); });
        sensors.emplace_back("Baz", [sigma](){ return -0.23 + rand_norm_zero_mean(sigma); });
    }
};

TEST_CASE( "predictor adapts" , "[predictor]")
{
    test_space sensors{0.0};
    sensors.execute_cycle();
    Predictor pred{ sensors, 0.1, 0.01, 1 };
    pred.initialize_randomized();
    const std::vector<double>& w = pred.get_prediction();
    REQUIRE( w.size() == 3 );

    for (unsigned i = 0; i < 100; ++i) {
        sensors.execute_cycle();
        pred.adapt();
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
    sensors.execute_cycle();
    Predictor pred{ sensors, 0.1, 0.01, 100 };
    pred.initialize_randomized();

    const std::vector<double>& w = pred.get_prediction();
    REQUIRE( w.size() == 3 );

    for (unsigned i = 0; i < 1000; ++i) {
        sensors.execute_cycle();
        pred.adapt();
    }
    REQUIRE( close(w[0], 0.42, 0.1) );
    REQUIRE( close(w[1], 0.37, 0.1) );
    REQUIRE( close(w[2],-0.23, 0.1) );

    dbg_msg("%6.4f %6.4f %6.4f", w[0], w[1], w[2]);
}

TEST_CASE( "prediction error must be constant without learning step" )
{
    srand((unsigned) time(0));
    test_space sensors(0.0); /** without random */
    sensors.execute_cycle();
    Predictor pred{ sensors, 0.1, 0.01, 1 };
    pred.initialize_randomized();

    REQUIRE( pred.get_prediction_error() == 0.0 );
    pred.predict();
    double pred_err_start = pred.get_prediction_error();
    dbg_msg("Prediction error start: %1.5f", pred_err_start);
    REQUIRE( pred_err_start > 0.0 );

    for (unsigned i = 0; i < 13; ++i) {
        sensors.execute_cycle();
        double pred_err_new = pred.predict();
        REQUIRE( pred_err_new == pred.get_prediction_error() );
        dbg_msg("Prediction error %u: %1.5f %1.5f", i, pred_err_start, pred_err_new);
        REQUIRE( close(pred_err_start, pred_err_new, 0.001) );
    }
}

TEST_CASE( "prediction error must decrease after learning step" )
{
    srand((unsigned) time(0));
    test_space sensors(0.0); /** without random */
    sensors.execute_cycle();
    Predictor pred{ sensors, 0.1, 0.01, 1 };
    pred.initialize_randomized();

    REQUIRE( pred.get_prediction_error() == 0.0 );
    pred.predict();

    double pred_err_start = pred.get_prediction_error();
    dbg_msg("Prediction error start: %1.3f", pred_err_start);
    REQUIRE( pred_err_start > 0.0 );

    for (unsigned i = 0; i < 42; ++i) {
        sensors.execute_cycle();
        double pred_err_before = pred.predict();
        pred.adapt();
        double pred_err_after = pred.predict();
        REQUIRE( pred_err_after == pred.get_prediction_error() );
        dbg_msg("Prediction error %u: %1.5f %1.5f", i, pred_err_before, pred_err_after);
        REQUIRE( pred_err_before > pred_err_after );
    }
}

TEST_CASE( "prediction error is reset on (re-)initialization" )
{
    srand((unsigned) time(0));
    test_space sensors(0.1); /** with random */
    sensors.execute_cycle();
    Predictor pred{ sensors, 0.1, 0.01, 1 };
    REQUIRE( pred.get_prediction_error() == 0.0 );
    pred.initialize_randomized();
    REQUIRE( pred.get_prediction_error() == 0.0 );
    pred.predict();
    REQUIRE( pred.get_prediction_error() >  0.0 );
    pred.initialize_from_input();
    REQUIRE( pred.get_prediction_error() == 0.0 );
    sensors.execute_cycle();
    pred.predict();
    REQUIRE( pred.get_prediction_error() >  0.0 );
}


TEST_CASE( "state predictor construction", "[predictor]" )
{
    test_space inputs(0.01);
    learning::State_Predictor pred(inputs, 0.01, 0.1, 3, 1);
}
