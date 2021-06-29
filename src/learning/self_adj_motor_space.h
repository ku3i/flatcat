#ifndef SELF_ADJ_MOTOR_SPACE_H_INCLUDED
#define SELF_ADJ_MOTOR_SPACE_H_INCLUDED

#include <control/control_vector.h>

namespace control {

class self_adjusting_motor_space : public Action_Module_Interface
{
    control::Jointcontrol              control;

    std::size_t                        applied_policy;
    std::size_t                        applied_action;
    std::size_t                        applied_state;

    const learning::RL_Interface&      learner;
    const bool                         self_adjusting;
    CompetitiveMotorLayer              motor_layer;

public:

    self_adjusting_motor_space( robots::Robot_Interface&       robot
                              , const Control_Vector&          parameter_set
                              , const std::size_t              max_actions
                              , const std::size_t              num_actions_begin
                              , static_vector<State_Payload>&  state_payload
                              , const learning::RL_Interface&  learner
                              , bool                           self_adjusting
                              , const double                   mutation_rate
                              , const double                   learning_rate
                              , const control::Minimal_Seed_t& seed )
    : control(robot)
    , applied_policy(0)
    , applied_action(0)
    , applied_state(0)
    , learner(learner)
    , self_adjusting(self_adjusting)
    , motor_layer(robot, state_payload, parameter_set, max_actions, num_actions_begin, mutation_rate, learning_rate, self_adjusting, seed)
    {
        dbg_msg("Creating self adjusting motor space.");
        control.set_control_parameter(motor_layer.get_unit(0).weights); // initialize non-mutated start controller
        control.print_parameter();
        control.reset();
    }

    void execute_cycle(bool state_has_changed)
    {
        if (state_has_changed) { /* apply action learning on state change only */
            /* adjust previously selected weights*/
            motor_layer.adapt(learner.positive_current_delta(applied_policy));

            /* update and check state + action from learner */
            applied_policy = learner.get_current_policy();
            applied_action = learner.get_current_action();
            applied_state  = learner.get_current_state();

            if (self_adjusting)
                motor_layer.enable_adaption(applied_policy == 0); /**TODO use enums for policies*/
            motor_layer.create_mutated_weights(applied_action);

            /* apply new weights */
            control.set_control_parameter(motor_layer.get_mutated_weights());
        }
        control.execute_cycle();
    }

    std::size_t get_number_of_actions          (void) const { return motor_layer.get_number_of_motor_units();           }
    std::size_t get_number_of_actions_available(void) const { return motor_layer.get_number_of_available_motor_units(); }

    bool exists(const std::size_t action_index) const { return motor_layer.exists(action_index); }

    friend class self_adjusting_motor_space_graphics;
};

#include <draw/draw.h>

/**TODO refactor and move to gmes_action_module*/
class self_adjusting_motor_space_graphics : public Graphics_Interface {
    const self_adjusting_motor_space& space;
    CompetitiveMotorLayer_Graphics    motor_layer_graphics;
public:
    self_adjusting_motor_space_graphics(const self_adjusting_motor_space& space)
    : space(space)
    , motor_layer_graphics(space.motor_layer, -1.0, 0.0, 0.5) {}

    void execute_cycle(bool state_has_changed) {
        if (state_has_changed)
            motor_layer_graphics.execute_cycle();
    }
    void draw(const pref& p) const
    {
        motor_layer_graphics.draw(p);

        const MotorUnit& motor = space.motor_layer.get_unit(space.applied_action);
        glColor3f(1.0, 1.0, 1.0);
        glprintf(-0.9, 0.65, 0.0, 0.03, "%3u:%2u"
                                      , space.applied_state
                                      , space.applied_action );

        draw_vector2(-0.9, 0.6, 0.05, 1.0, motor.weights.get_parameter(), 3.0);
    }
};


} /* namespace control */

#endif /* SELF_ADJ_MOTOR_SPACE_H_INCLUDED */
