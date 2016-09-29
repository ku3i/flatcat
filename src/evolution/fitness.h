#ifndef FITNESS_H
#define FITNESS_H

#include <math.h>
#include <tr1/memory>

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
    {}
    double      fit;
    double      power;
    double      temp;
    std::size_t steps;
    bool        dropped;
    bool        out_of_track;
};

class Fitness_Base {
public:
    Fitness_Base(const robots::Simloid& robot, const std::string& name)
    : robot(robot)
    , name(name)
    {
        sts_msg("Fitness function: %s", name.c_str());
    }

    virtual void start (fitness_data& data) = 0;
    virtual void step  (fitness_data& data) = 0;
    virtual void finish(fitness_data& data) = 0;

    const std::string& get_name(void) const { return name; }

protected:
    const robots::Simloid& robot;

private:
    const std::string name;
};

typedef std::tr1::shared_ptr<Fitness_Base> Fitness_ptr;

Fitness_ptr assign_fitness(const robots::Simloid& robot, const Setting& settings);

class Fitness_Forwards : public Fitness_Base
{
public:
    Fitness_Forwards(const robots::Simloid& robot, bool drop_penalty, bool out_of_track_penalty, bool use_avg = true)
    : Fitness_Base(robot, "FORWARDS")
    , drop_penalty(drop_penalty)
    , out_of_track_penalty(out_of_track_penalty)
    , use_avg(use_avg)
    { sts_msg("Evolve walking forwards."); }

    void start(fitness_data& data) {}

    void step(fitness_data& data)
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;

        if (out_of_track_penalty && robot.out_of_track_x())
            data.out_of_track = true;
    }

    void finish(fitness_data& data)
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
    bool drop_penalty;
    bool out_of_track_penalty;
    bool use_avg;
};

class Fitness_Backwards : public Fitness_Base
{
public:
    Fitness_Backwards(const robots::Simloid& robot, bool drop_penalty, bool out_of_track_penalty, bool use_avg = true)
    : Fitness_Base(robot, "BACKWARDS")
    , drop_penalty(drop_penalty)
    , out_of_track_penalty(out_of_track_penalty)
    , use_avg(use_avg)
    { sts_msg("Evolve walking backwards."); }

    void start(fitness_data& data) {}

    void step(fitness_data& data)
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;

        if (out_of_track_penalty && robot.out_of_track_x())
            data.out_of_track = true;
    }

    void finish(fitness_data& data)
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
    bool drop_penalty;
    bool out_of_track_penalty;
    bool use_avg;
};

class Fitness_Jumping : public Fitness_Base
{
public:
    Fitness_Jumping(const robots::Simloid& robot)
    : Fitness_Base(robot, "JUMPING")
    { sts_msg("Evolve jumping."); }

    void start(fitness_data& data) {}

    void step(fitness_data& data)
    {
        if (data.steps > 20)
            data.fit = fmax((data.fit), robot.get_avg_position().z);
    }

    void finish(fitness_data& data) {}
};

class Fitness_Stopping : public Fitness_Base
{
public:
    Fitness_Stopping(const robots::Simloid& robot, bool drop_penalty)
    : Fitness_Base(robot, "STOPPING")
    , drop_penalty(drop_penalty)
    { sts_msg("Evolve stopping."); }

    void start(fitness_data& data) {}

    void step(fitness_data& data)
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;
    }

    void finish(fitness_data& data)
    {
        data.fit = 10.0/(1 + data.power);

        if (data.dropped)
            data.fit -= 10;
    }
private:
    bool drop_penalty;
};

class Fitness_Standing : public Fitness_Base
{
public:
    Fitness_Standing(const robots::Simloid& robot)
    : Fitness_Base(robot, "STANDING")
    { sts_msg("Evolve standing."); }

    void start(fitness_data& data) {}

    void step(fitness_data& data)
    {
        if (robot.dropped())
            data.dropped = true;
    }

    void finish(fitness_data& data)
    {
        data.fit = data.steps;
        /* TODO include power consumption */
    }
};

class Fitness_Sidewards : public Fitness_Base
{
public:
    Fitness_Sidewards(const robots::Simloid& robot, bool drop_penalty, bool out_of_track_penalty)
    : Fitness_Base(robot, "SIDEWARDS")
    , drop_penalty(drop_penalty)
    , out_of_track_penalty(out_of_track_penalty)
    { sts_msg("Evolve walking sidewards."); }

    void start(fitness_data& data) {}

    void step(fitness_data& data)
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;

        if (out_of_track_penalty && robot.out_of_track_y())
            data.out_of_track = true;

        data.temp = fmax((data.temp), fabs(robot.get_avg_position().y));
    }

    void finish(fitness_data& data)
    {
        /* left has positive sign on x axis */
        data.fit = robot.get_avg_position().x;
        data.fit -= data.temp;

        if (data.dropped)
            data.fit -= 2*robot.get_bodyheight0(); //TODO make the penalty configurable
        /* no penalty for out of track here */
    }
private:
    bool drop_penalty;
    bool out_of_track_penalty;
};

class Fitness_Turning : public Fitness_Base
{
public:
    Fitness_Turning(const robots::Simloid& robot, bool drop_penalty, bool out_of_track_penalty)
    : Fitness_Base(robot, "TURNING")
    , drop_penalty(drop_penalty)
    , out_of_track_penalty(out_of_track_penalty)
    { sts_msg("Evolve spinning or turning around."); }

    void start(fitness_data& data) {
        data.temp = robot.get_avg_rotation();
    }

    void step(fitness_data& data)
    {
        if (drop_penalty && robot.dropped())
            data.dropped = true;

        if (out_of_track_penalty && (robot.out_of_track_y() || robot.out_of_track_x()))
            data.out_of_track = true;

        /* sum of rotation */
        data.temp = unwrap(robot.get_avg_rotation(), data.temp);
    }

    void finish(fitness_data& data)
    {
        data.fit = data.temp;

        if (data.dropped)
            data.fit -= 2*M_PI;
    }

private:
    bool drop_penalty;
    bool out_of_track_penalty;
};

class Fitness_Standup : public Fitness_Base
{
public:
    Fitness_Standup(const robots::Simloid& robot)
    : Fitness_Base(robot, "STANDUP")
    { sts_msg("Evolve standing up."); }

    void start(fitness_data& data) {}
    void step(fitness_data& data)
    {
        /* TODO use body height to have a minimal value of let's say 80% */
    }
    void finish(fitness_data& data) {}
};

class Fitness_Sitdown : public Fitness_Base
{
public:
    Fitness_Sitdown(const robots::Simloid& robot)
    : Fitness_Base(robot, "SITDOWN")
    { sts_msg("Evolve sitting down."); }

    void start (fitness_data& data) {}
    void step  (fitness_data& data) {}
    void finish(fitness_data& data) {}
};

class Fitness_Rollover : public Fitness_Base
{
public:
    Fitness_Rollover(const robots::Simloid& robot)
    : Fitness_Base(robot, "ROLLOVER")
    { sts_msg("Evolve sitting down."); } //TODO

    void start (fitness_data& data) {} // start standing
    void step  (fitness_data& data) {} // no drop penalty
    void finish(fitness_data& data) {} // max distance left.
};

#endif /* FITNESS_H */
