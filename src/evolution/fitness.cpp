
#include "./fitness.h"

Fitness_ptr assign_fitness( const robots::Simloid& robot
                          , const Setting& settings)
{
    Fitness_ptr fitness_function;
    const std::string& fitness(settings.fitness_function);

         if ("FORWARDS"     == fitness) fitness_function = Fitness_ptr(new Fitness_Forwards (robot, settings.drop_penalty, settings.out_of_track_penalty, true , settings.target));
    else if ("FORWARDS_MIN" == fitness) fitness_function = Fitness_ptr(new Fitness_Forwards (robot, settings.drop_penalty, settings.out_of_track_penalty, false));
    else if ("FORWARDS_FEET"== fitness) fitness_function = Fitness_ptr(new Fitness_Forwards_Feet(robot, settings.drop_penalty, settings.out_of_track_penalty, settings.stop_penalty));
    else if ("BACKWARDS"    == fitness) fitness_function = Fitness_ptr(new Fitness_Backwards(robot, settings.drop_penalty, settings.out_of_track_penalty));
    else if ("BACKWARDS_MIN"== fitness) fitness_function = Fitness_ptr(new Fitness_Backwards(robot, settings.drop_penalty, settings.out_of_track_penalty, false));
    else if ("STOPPING"     == fitness) fitness_function = Fitness_ptr(new Fitness_Stopping (robot, settings.drop_penalty));
    else if ("STANDING"     == fitness) fitness_function = Fitness_ptr(new Fitness_Standing (robot, settings.stop_penalty));
    else if ("SIDEWARDS"    == fitness) fitness_function = Fitness_ptr(new Fitness_Sidewards(robot, settings.drop_penalty, settings.out_of_track_penalty));
    else if ("SIDEWARDS_MIN"== fitness) fitness_function = Fitness_ptr(new Fitness_Sidewards(robot, settings.drop_penalty, settings.out_of_track_penalty, false));
    else if ("TURNING"      == fitness) fitness_function = Fitness_ptr(new Fitness_Turning  (robot, settings.drop_penalty, settings.out_of_track_penalty));
    else err_msg(__FILE__, __LINE__, "Wrong name of fitness function.");

    return fitness_function;
}
