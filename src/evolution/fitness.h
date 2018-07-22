#ifndef FITNESS_H
#define FITNESS_H

#include <math.h>
#include <memory>

#include <common/modules.h>
#include <common/config.h>

#include <robots/simloid.h>

#include <evolution/setting.h>


struct fitness_data
{
    fitness_data()
    : fit  (.0)
    , power(.0)
    , temp (.0)
    , steps( 0)
    , dropped(false)
    , out_of_track(false)
    , stopped(false)
    {}
    double      fit;
    double      power;
    double      temp;
    std::size_t steps;
    bool        dropped;
    bool        out_of_track;
    bool        stopped;
};

class Fitness_Base {
public:
    Fitness_Base(const std::string& name, const robots::Simloid& robot, bool drop_penalty = false, bool out_of_track_penalty = false, bool stop_penalty = false)
    : name(name)
    , robot(robot)
    , drop_penalty(drop_penalty)
    , out_of_track_penalty(out_of_track_penalty)
    , stop_penalty(stop_penalty)
    {
        sts_msg("Fitness function: %s", name.c_str());
        //TODO print flags
    }

    virtual void start (fitness_data& /*data*/) {};
    virtual void step  (fitness_data& data) = 0;
    virtual void finish(fitness_data& data) = 0;

    const std::string& get_name(void) const { return name; }

private:
    const std::string name;

protected:
    const robots::Simloid& robot;
    bool                   drop_penalty;
    bool                   out_of_track_penalty;
    bool                   stop_penalty;
};

typedef std::shared_ptr<Fitness_Base> Fitness_ptr;

Fitness_ptr assign_fitness(const robots::Simloid& robot, const Setting& settings);

class Fitness_Forwards : public Fitness_Base
{
public:
    Fitness_Forwards(const robots::Simloid& robot, bool drop_penalty, bool out_of_track_penalty, bool use_avg = true)
    : Fitness_Base("FORWARDS", robot, drop_penalty, out_of_track_penalty)
    , use_avg(use_avg)
    { sts_msg("Evolve walking forwards."); }

    void step(fitness_data& data) override
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;

        if (out_of_track_penalty && robot.out_of_track_x())
            data.out_of_track = true;
    }

    void finish(fitness_data& data) override
    {
        /* forwards has negative sign on y axis */
        if (use_avg)
            data.fit = -robot.get_avg_position().y;
        else
            data.fit = -robot.get_max_position().y;

        if (data.dropped)
            data.fit -= 2*robot.get_bodyheight0();

        if (data.out_of_track)
            data.fit -= .5;
    }

private:
    bool use_avg;
};

class Fitness_Forwards_Feet : public Fitness_Base
{
public:
    Fitness_Forwards_Feet(const robots::Simloid& robot, bool drop_penalty, bool out_of_track_penalty, bool stop_penalty)
    : Fitness_Base("FORWARDS_FEET", robot, drop_penalty, out_of_track_penalty, stop_penalty)
    { sts_msg("Evolve walking forwards, taking only feet position into account."); }

    void step(fitness_data& data) override
    {
        if (drop_penalty && robot.dropped(0.9))
            data.dropped = true;

        if (out_of_track_penalty && robot.out_of_track_x())
            data.out_of_track = true;

        if (data.steps > 100 && stop_penalty && robot.motion_stopped(0.0005))
            data.stopped = true; /**TODO this penalty is not yet included in the other fitness functions*/
    }

    void finish(fitness_data& data) override
    {
        /* forwards has negative sign on y axis */
        data.fit = -robot.get_max_feet_pos().y;

        if (data.dropped) {
            data.fit -= 2*robot.get_bodyheight0() + exp(-(data.steps/1000.0));
        }

        if (data.out_of_track)
            data.fit -= .5;

        if (data.stopped)
            data.fit -= .5;// + exp(-(data.steps/1000.0));
    }
};

class Fitness_Backwards : public Fitness_Base
{
public:
    Fitness_Backwards(const robots::Simloid& robot, bool drop_penalty, bool out_of_track_penalty, bool use_avg = true)
    : Fitness_Base("BACKWARDS", robot, drop_penalty, out_of_track_penalty)
    , use_avg(use_avg)
    { sts_msg("Evolve walking backwards."); }

    void step(fitness_data& data) override
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;

        if (out_of_track_penalty && robot.out_of_track_x())
            data.out_of_track = true;
    }

    void finish(fitness_data& data) override
    {
        /* backwards has positive sign on y axis */
        if (use_avg)
            data.fit = robot.get_avg_position().y;
        else
            data.fit = robot.get_min_position().y;

        if (data.dropped)
            data.fit -= 2*robot.get_bodyheight0();

        if (data.out_of_track)
            data.fit -= .5;
    }

private:
    bool use_avg;
};


class Fitness_Stopping : public Fitness_Base
{
public:
    Fitness_Stopping(const robots::Simloid& robot, bool drop_penalty)
    : Fitness_Base("STOPPING", robot, drop_penalty)
    { sts_msg("Evolve stopping."); }

    void step(fitness_data& data) override
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;
    }

    void finish(fitness_data& data) override
    {
        data.fit = 10.0/(1 + data.power);

        if (data.dropped)
            data.fit -= 10;
    }
};

class Fitness_Standing : public Fitness_Base
{
public:
    Fitness_Standing(const robots::Simloid& robot, bool stop_penalty)
    : Fitness_Base("STANDING", robot, false, false, stop_penalty)
    { sts_msg("Evolve standing."); }

    void step(fitness_data& data) override
    {
        if (robot.dropped())
            data.dropped = true;

        if (data.steps > 100 && stop_penalty && robot.motion_stopped(0.0005))
            data.stopped = true; /**TODO this penalty is not yet included in the other fitness functions*/
    }

    void finish(fitness_data& data) override
    {
        data.fit = data.steps;
        /* TODO include power consumption */
    }
};

class Fitness_Sidewards : public Fitness_Base
{
public:
    Fitness_Sidewards(const robots::Simloid& robot, bool drop_penalty, bool out_of_track_penalty)
    : Fitness_Base("SIDEWARDS", robot, drop_penalty, out_of_track_penalty)
    { sts_msg("Evolve walking sidewards."); }

    void step(fitness_data& data) override
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;

        if (out_of_track_penalty && robot.out_of_track_y())
            data.out_of_track = true;

        data.temp = fmax((data.temp), fabs(robot.get_avg_position().y));
    }

    void finish(fitness_data& data) override
    {
        /* left has positive sign on x axis */
        data.fit = robot.get_avg_position().x;
        data.fit -= data.temp;

        if (data.dropped)
            data.fit -= 2*robot.get_bodyheight0(); //TODO make the penalty configurable
        /* no penalty for out of track here */
    }
};

class Fitness_Turning : public Fitness_Base
{
public:
    Fitness_Turning(const robots::Simloid& robot, bool drop_penalty, bool out_of_track_penalty)
    : Fitness_Base("TURNING", robot, drop_penalty, out_of_track_penalty)
    { sts_msg("Evolve spinning or turning around."); }

    void start(fitness_data& data) override {
        data.temp = robot.get_avg_rotation();
    }

    void step(fitness_data& data) override
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;

        if (out_of_track_penalty && (robot.out_of_track_y() || robot.out_of_track_x()))
            data.out_of_track = true;

        /* sum of rotation */
        data.temp = unwrap(robot.get_avg_rotation(), data.temp);
    }

    void finish(fitness_data& data) override
    {
        data.fit = data.temp;

        if (data.dropped)
            data.fit -= 2*M_PI;
    }
};

class Fitness_Moving : public Fitness_Base
{
public:
    Fitness_Moving(const robots::Simloid& robot, bool drop_penalty)
    : Fitness_Base("TURNING", robot, drop_penalty, false)
    { sts_msg("Evolve moving."); }

    void start(fitness_data& data) override {
        data.temp = .0;
    }

    void step(fitness_data& data) override
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;

        /* sum up movements */
        data.temp += robot.sum_abs_velocities();
    }

    void finish(fitness_data& data) override
    {
        data.fit = data.temp/10.0;

        //if (data.dropped)
          //  data.fit -= 1.0 + exp(-(data.steps/1000.0));
    }
};

#endif /* FITNESS_H */
