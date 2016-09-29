#ifndef BEHAVIOR_STATE_MACHINE_H_INCLUDED
#define BEHAVIOR_STATE_MACHINE_H_INCLUDED

#include <common/log_messages.h>
#include <robots/simloid.h>
#include <control/controlparameter.h>
#include <control/jointcontrol.h>

/*
   \ | /
  -- o --
   / | \
*/

//class State_Base {
//public:
//    State_Base(const std::string& name)
//    : name(name)
//    { dbg_msg("Creating state: %", name.c_str()); }
//
//    virtual void entry(void) = 0;
//    virtual void step(void) = 0;
//
//    const std::string& get_name(void) const { return name; }
//
//protected:
//    const std::string name;
//};

enum Behaviorial_State {
//    standing,
//    walking_forwards,
//    stop_walking_forwards,
//    walking_backwards,
//    walking_left,
//    walking_right,
//    running_forwards,
//    running_backwards,
//    running_left,
//    running_right,
//    stop_from_running,
//    roll_over_left,
//    roll_over_right,
//
    turning_left,
    stop_turning_left,
//
//    turning_right,
//    stop_turning_right,
};

/*
TODO: table with allowed transitions:
*/


enum Direction { forwards, backwards, left, right };
enum Gait { stand, walk, turn, run, stop };

class Behavioral_Statemachine {
public:
    Behavioral_Statemachine( const control::Control_Vector& parameter_set
                           , robots::Simloid&               robot
                           , control::Jointcontrol&         control
                           , Behaviorial_State              initial_state = Behaviorial_State::stop_turning_left/*walking_forwards*/)
    : parameter_set(parameter_set)
    , robot(robot)
    , control(control)
    , state(initial_state)
    , last_state(state)
    , command()
    {
        dbg_msg("Creating behavioral state machine: \n   Initial state is %u", state);
        //TODO: how make the mapping from enum (state) to parameter_setID

    }

    void loop(void)
    {
        switch(state)
        {
        case turning_left:
            if (command.gait == Gait::stop)
                state = stop_turning_left;
            break;

        case stop_turning_left:
            if (command.gait == Gait::turn)
                state = turning_left;
            break;

//        case standing:
//            if (command.gait == Gait::walk)
//                state = walking_forwards;
//            break;
//
//        case walking_forwards:
//            if (command.gait == Gait::stop)
//                state = stop_walking_forwards;
//
//                //TODO control.set_control_parameter(parameter_set.get(current_behavior).get_parameter(), true);
//            break;
//
//        case stop_walking_forwards:
//            /* TODO: wait for stop : robot.motion_stopped(threshold) */
//            if (command.gait == Gait::walk) {
//                state = walking_forwards;
//            }
//            else if (robot.motion_stopped(0.01))
//            {
//                state = standing;
//                dbg_msg("goto state: standing");
//            }
//            break;

        default:
            assert(false);
        }

        if (last_state != state) {
            dbg_msg("State has changed from %u to %u", last_state, state);
            last_state = state;
        }
    }


    /* command functions */
    void walk(void) { command.gait = Gait::walk; dbg_msg("walk"); }
    void turn(void) { command.gait = Gait::turn; dbg_msg("turn"); }
    void run (void) { command.gait = Gait::run;  dbg_msg("run" ); }
    void stop(void) { command.gait = Gait::stop; dbg_msg("stop"); }
    /* stand is implicit and will be activated when stopping has succeeded */

    void forwards (void) { command.dir = Direction::forwards;  dbg_msg("forwards" ); }
    void backwards(void) { command.dir = Direction::backwards; dbg_msg("backwards"); }
    void left     (void) { command.dir = Direction::left;      dbg_msg("left"     ); }
    void right    (void) { command.dir = Direction::right;     dbg_msg("right"    ); }

    Behaviorial_State get_state(void) const { return state; }

private:
    const control::Control_Vector& parameter_set;
    robots::Simloid&               robot;
    control::Jointcontrol&         control;
    Behaviorial_State              state, last_state;

    struct control_commands {
        control_commands()
        : gait(Gait::stand)
        , dir(Direction::forwards)
        { dbg_msg("Creating control commands."); }

        Gait gait;
        Direction dir;
    } command;


};
#endif // BEHAVIOR_STATE_MACHINE_H_INCLUDED
