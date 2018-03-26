#ifndef PUSHER_H_INCLUDED
#define PUSHER_H_INCLUDED

#include <basic/vector3.h>

class robot_pusher {

    robots::Simloid& robot;
    double probability;
    double strength;
    unsigned body_index;
    Vector3 force;
    unsigned duration;
    bool active;

public:
    robot_pusher(robots::Simloid& robot, double probability, double strength)
    : robot(robot)
    , probability(probability)
    , strength(strength)
    , body_index(0)
    , force(0.)
    , duration(0)
    , active(false)
    {
        assert(probability < 1.0 and probability >= 0.);
    }

    void execute_cycle(void) {
        if (!active) {
            if (random_value() > 1.0 - probability)
            {
                force.random(-strength,+strength);
                body_index = random_index(robot.get_number_of_bodies());
                robot.set_force(body_index, force);
                duration = random_int(0, 25);
                active = true;
            }
        }
        else { /* still active */
            if (duration == 0)
            {
                force.zero();
                robot.set_force(body_index, force); // set force to zero
                active = false;
            } else
                --duration;
        }
    }
};

#endif // PUSHER_H_INCLUDED
