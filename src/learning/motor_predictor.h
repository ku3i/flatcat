#ifndef MOTOR_PREDICTOR_H_INCLUDED
#define MOTOR_PREDICTOR_H_INCLUDED

#include <common/log_messages.h>
#include <robots/robot.h>
#include <control/controlparameter.h>
#include <control/control_core.h>
#include <learning/predictor.h>

class Motor_Predictor : public Predictor_Base {
public:
    Motor_Predictor( const robots::Robot_Interface& robot
                   , const sensor_vector& input
                   , const double learning_rate
                   , const double random_weight_range
                   , const control::Control_Parameter& parameter )
    : Predictor_Base(input, learning_rate, random_weight_range, 1/**TODO*/)
    , robot(robot)
    , core(robot)
    , asym_params(make_asymmetric(parameter))
    {
        dbg_msg("Creating motor predictor.");
        core.apply_weights(robot, asym_params);
    }

    void   copy(Predictor_Base const& other) { /**TODO implement*/ }


    double predict(void) {
        core.prepare_inputs(robot);
        core.update_outputs(robot, params.is_symmetric(), params.is_mirrored());
        return calculate_prediction_error(core.activation);
    }

    void   adapt  (void) {
        /**TODO what shall be done here?
         * gradient descent on the motor controller weights
         * do we handle the non-smooth transfer function (clip), or do we assume linearity
         * do we handle the 'recurrent' motor connections as just weights
         */
    }


    void initialize_randomized(void)       override { /**TODO implement*/ }
    void initialize_from_input(void)       override { /**TODO implement*/ }
    VectorN const& get_weights(void) const override { /**TODO use weights*/ return asym_params.get_parameter(); }

private:
    robots::Robot_Interface const&          robot;
    control::Fully_Connected_Symmetric_Core core;
    control::Control_Parameter              asym_params; // currently just for loading
};


#endif // MOTOR_PREDICTOR_H_INCLUDED
