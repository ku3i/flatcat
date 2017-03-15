#include <tests/catch.hpp>

#include <learning/predictor.h>
#include <learning/motor_predictor.h>
#include <common/log_messages.h>
#include <tests/test_robot.h>

namespace local_tests {

class Test_Motor_Space : public sensor_vector {
public:
    Test_Motor_Space(const robots::Jointvector_t& joints, const double sigma)
    : sensor_vector(joints.size())
    {
        for (robots::Joint_Model const& j : joints)
            sensors.emplace_back(j.name, [&j, sigma](){ return j.motor.get() + rand_norm_zero_mean(sigma); });
    }
};



TEST_CASE( "motor predictor adapts" , "[motor_predictor]")
{
    srand(2310); // set random seed

    Test_Robot robot(5,2);
    Test_Motor_Space motors{robot.get_joints(), 0.01};
    control::Control_Parameter params = control::get_initial_parameter(robot,{0.,0.,0.}, false);

    /* set constant outputs to learn from */
    auto& joints = robot.set_joints();
    REQUIRE( joints.size() == 5 );
    joints[0].motor.set(0.4223);
    joints[1].motor.set(0.3771);
    joints[2].motor.set(0.2342);
    joints[3].motor.set(0.1337);
    joints[4].motor.set(0.9876);

    /* assert that motor space is correct */
    motors.execute_cycle();
    REQUIRE( motors.size() == 5 );
    for (unsigned i = 0; i < 5; ++i)
        REQUIRE( close(motors[i], joints[i].motor.get(), 0.1) );

    /* initialize predictors */
    motors.execute_cycle();
    learning::Motor_Predictor pred{ robot, motors, 0.1, 0.01, 1, params };
    pred.initialize_randomized();

    /* adapt a 'few' cycles */
    for (unsigned i = 0; i < 1000; ++i) {
        motors.execute_cycle();
        pred.predict();
        pred.adapt();
    }

    /* compare predictions with constant motor output */
    auto const& predictions = pred.get_prediction();
    for (auto& p : predictions)
        dbg_msg("%+e", p);

    REQUIRE( predictions.size() == 5 );
    REQUIRE( close(predictions[0], 0.4223, 0.01) );
    REQUIRE( close(predictions[1], 0.3771, 0.01) );
    REQUIRE( close(predictions[2],-0.2342, 0.01) );
    REQUIRE( close(predictions[3],-0.1337, 0.01) );
    REQUIRE( close(predictions[4], 0.9876, 0.01) );

}

//TEST_CASE( "motor predictor adapts with experience replay" , "[motor_predictor]") /**TODO EXPERIENCE REPLAY */
//{
//    srand((unsigned) time(0));
//    Test_Robot robot(5,2);
//    Test_Motor_Space motors(robot.get_joints(), 0.01); /** with random */
//    control::Control_Parameter params = control::get_initial_parameter(robot,{0.,0.,0.}, false);
//    motors.execute_cycle();
//    learning::Motor_Predictor pred{ robot, motors, 0.1, 0.01, 1 /**TODO*/, params};
//    pred.initialize_randomized();
//
//    const std::vector<double>& w = pred.get_prediction();
//    REQUIRE( w.size() == 3 );
//
//    for (unsigned i = 0; i < 1000; ++i) {
//        motors.execute_cycle();
//        pred.adapt();
//    }
//    REQUIRE( close(w[0], 0.42, 0.1) );
//    REQUIRE( close(w[1], 0.37, 0.1) );
//    REQUIRE( close(w[2],-0.23, 0.1) );
//
//    dbg_msg("%6.4f %6.4f %6.4f", w[0], w[1], w[2]);
//}

TEST_CASE( "motor prediction error must be constant without learning step", "[motor_predictor]" )
{
    Test_Robot robot(5,2);
    srand((unsigned) time(0));
    Test_Motor_Space motors(robot.get_joints(), 0.0); /** without random */
    control::Control_Parameter params = control::get_initial_parameter(robot,{0.,0.,0.}, false);
    motors.execute_cycle();
    learning::Motor_Predictor pred{ robot, motors, 0.1, 0.01, 1, params };
    pred.initialize_randomized();

    REQUIRE( pred.get_prediction_error() == 0.0 );
    pred.predict();
    double pred_err_start = pred.get_prediction_error();
    dbg_msg("Prediction error start: %e", pred_err_start);
    REQUIRE( pred_err_start > 0.0 );

    for (unsigned i = 0; i < 13; ++i) {
        motors.execute_cycle();
        double pred_err_new = pred.predict();
        REQUIRE( pred_err_new == pred.get_prediction_error() );
        dbg_msg("Prediction error %u: %e %e", i, pred_err_start, pred_err_new);
        REQUIRE( close(pred_err_start, pred_err_new, 0.001) );
    }
}

TEST_CASE( "motor prediction error must decrease after learning step", "[motor_predictor]" )
{
    srand((unsigned) time(0));
    Test_Robot robot(5,2);
    Test_Motor_Space motors(robot.get_joints(), 0.0); /** without random */
    control::Control_Parameter params = control::get_initial_parameter(robot,{0.,0.,0.}, false);
    motors.execute_cycle();
    learning::Motor_Predictor pred{ robot, motors, 0.1, 0.01, 1, params };
    pred.initialize_randomized();

    REQUIRE( pred.get_prediction_error() == 0.0 );
    pred.predict();

    double pred_err_start = pred.get_prediction_error();
    dbg_msg("Prediction error start: %e", pred_err_start);
    REQUIRE( pred_err_start > 0.0 );

    for (unsigned i = 0; i < 42; ++i) {
        motors.execute_cycle();
        double pred_err_before = pred.predict();
        pred.adapt();
        double pred_err_after = pred.predict();
        REQUIRE( pred_err_after == pred.get_prediction_error() );
        dbg_msg("Prediction error %u: %e %e", i, pred_err_before, pred_err_after);
        REQUIRE( pred_err_before > pred_err_after );
    }
}

TEST_CASE( "motor prediction error is reset on (re-)initialization", "[motor_predictor]")
{
    srand((unsigned) time(0));
    Test_Robot robot(5,2);
    Test_Motor_Space motors(robot.get_joints(), 0.0); /** without random */
    control::Control_Parameter params = control::get_initial_parameter(robot,{0.,0.,0.}, false);
    motors.execute_cycle();
    learning::Motor_Predictor pred{ robot, motors, 0.1, 0.01, 1, params };
    pred.initialize_randomized();

    REQUIRE( pred.get_prediction_error() == 0.0 );
    pred.predict();

    REQUIRE( pred.get_prediction_error() >  0.0 );
    //not supported pred.initialize_from_input();
    REQUIRE( pred.get_prediction_error() == 0.0 );
    motors.execute_cycle();
    pred.predict();
    REQUIRE( pred.get_prediction_error() >  0.0 );
}

} // namespace local_tests
