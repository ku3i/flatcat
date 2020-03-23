#ifndef FITNESS_H
#define FITNESS_H

#include <math.h>
#include <memory>

#include <common/modules.h>
#include <common/config.h>

#include <robots/simloid.h>

#include <evolution/setting.h>

/**TODO: how to incorporate L1 normalization into fitness?
   How to access the weight matrix?
   we only need to access the norm value L1 = sum |w_i|
   start with computing this value and print it to terminal after each evaluation.
   */

struct fitness_data
{
    fitness_data()
    : fit  (.0)
    , power(.0)
    , dctrl(.0)
    , temp (.0)
    , steps( 0)
    , max_steps(0)
    , dropped(false)
    , out_of_track(false)
    , stopped(false)
    {}
    double      fit;
    double      power;
    double      dctrl;
    double      temp;
    std::size_t steps;
    std::size_t max_steps;
    bool        dropped;
    bool        out_of_track;
    bool        stopped;
};


inline double step_ratio(fitness_data const& data) { return clip(static_cast<double>(data.steps)/data.max_steps, 0.0, 1.0); }

class Fitness_Base {
public:
    Fitness_Base(const std::string& name, const robots::Simloid& robot, const Setting& settings)
    : robot(robot)
    , s(settings)
    {
        sts_msg("Fitness Name = %s", name.c_str());
        sts_msg("DROPPED      = %s", s.drop_penalty         ? "YES":"NO");
        sts_msg("OUT-OF-TRACK = %s", s.out_of_track_penalty ? "YES":"NO");
        sts_msg("STOPPED      = %s", s.stop_penalty         ? "YES":"NO");
        sts_msg("TARGET       = %1.2f", s.target);
        sts_msg("DROP_LEVEL   = %1.2f", s.drop_level);
        sts_msg("STOP_LEVEL   = %1.2f", s.stop_level);
        sts_msg("CORRIDOR     = %1.2f", s.corridor);

        assert(s.target >= .0);
        assert(s.corridor >= .0);
        assert_in_range(s.drop_level, 0., 1.);
        assert_in_range(s.stop_level, 0., 1.);
    }

    virtual void start (fitness_data& /*data*/) {};
    virtual void step  (fitness_data& data) = 0;
    virtual void finish(fitness_data& data) = 0;
    virtual ~Fitness_Base() {};

private:
    const unsigned min_steps = 100;

protected:
    const robots::Simloid& robot;
    const Setting&         s; //settings

    bool dropped(void) { return s.drop_penalty && robot.dropped(s.drop_level); }
    bool stopped(fitness_data& data) { return s.stop_penalty && (data.steps > min_steps) && robot.motion_stopped(s.stop_level); }

    bool out_of_track_x(void) { return s.out_of_track_penalty && (fabs(robot.dx_from_origin()) > s.corridor); }
    bool out_of_track_y(void) { return s.out_of_track_penalty && (fabs(robot.dy_from_origin()) > s.corridor); }


};

typedef std::shared_ptr<Fitness_Base> Fitness_ptr;

Fitness_ptr assign_fitness(const robots::Simloid& robot, const Setting& settings);

class Fitness_Forwards : public Fitness_Base
{
public:
    Fitness_Forwards(const robots::Simloid& robot, const Setting& s, bool use_avg = true)
    : Fitness_Base("FORWARDS", robot, s)
    , use_avg(use_avg)
    { sts_msg("Evolve getting forwards."); }

    void step(fitness_data& data) override
    {
        if (dropped()) data.dropped = true;
        if (stopped(data)) data.stopped = true;
        if (out_of_track_x()) data.out_of_track = true;
    }

    void finish(fitness_data& data) override
    {
        /* forwards has negative sign on y axis */
        data.fit = use_avg ? -robot.get_avg_position().y : -robot.get_max_position().y;

        if (s.target != .0 and data.fit > s.target) {
            data.fit = clip(data.fit, std::abs(s.target));
            //dbg_msg("Target overshot. \n");
            data.fit *= step_ratio(data);
        }

        if (data.dropped || data.out_of_track || data.stopped)
            data.fit -= 1.0;// - step_ratio(data);
    }

private:
    bool use_avg;
};


class Fitness_Forwards_Feet : public Fitness_Base
{
public:
    Fitness_Forwards_Feet(const robots::Simloid& robot, const Setting& s)
    : Fitness_Base("FORWARDS_FEET", robot, s)
    { sts_msg("Evolve walking forwards, taking only feet position into account."); }

    void step(fitness_data& data) override
    {
        if (dropped()) data.dropped = true;
        if (stopped(data)) data.stopped = true;
        if (out_of_track_x()) data.out_of_track = true;
    }

    void finish(fitness_data& data) override
    {
        /* forwards has negative sign on y axis */
        data.fit = -robot.get_max_feet_pos().y;

        if (data.dropped || data.out_of_track || data.stopped)
            data.fit -= 1.0 - step_ratio(data);
    }
};


class Fitness_Backwards : public Fitness_Base
{
public:
    Fitness_Backwards(const robots::Simloid& robot, const Setting& s, bool use_avg = true)
    : Fitness_Base("BACKWARDS", robot, s)
    , use_avg(use_avg)
    { sts_msg("Evolve getting backwards."); }

    void step(fitness_data& data) override
    {
        if (dropped()) data.dropped = true;
        if (stopped(data)) data.stopped = true;
        if (out_of_track_x()) data.out_of_track = true;
    }

    void finish(fitness_data& data) override
    {
        /* backwards has positive sign on y axis */
        data.fit = use_avg ? robot.get_avg_position().y : robot.get_min_position().y;

        if (data.dropped || data.out_of_track || data.stopped)
            data.fit -= 1.0;// - step_ratio(data);
    }

private:
    bool use_avg;
};


class Fitness_Stopping : public Fitness_Base
{
public:
    Fitness_Stopping(const robots::Simloid& robot, const Setting& s)
    : Fitness_Base("STOPPING", robot, s)
    { sts_msg("Evolve stopping."); }

    void step(fitness_data& data) override
    {
        if (dropped()) data.dropped = true;
    }

    void finish(fitness_data& data) override
    {
        data.fit = 10.0/(1 + data.power);
        data.fit+= 10.0/(1 + data.dctrl);

        if (data.dropped)
            data.fit -= 10.0 * (1.0 - step_ratio(data));
    }
};


class Fitness_Sidewards : public Fitness_Base
{
public:
    Fitness_Sidewards(const robots::Simloid& robot, const Setting& s, bool use_avg = true)
    : Fitness_Base("SIDEWARDS", robot, s)
    , use_avg(use_avg)
    { sts_msg("Evolve walking sidewards."); }

    void step(fitness_data& data) override
    {
        if (dropped()) data.dropped = true;
        if (stopped(data)) data.stopped = true;
        if (out_of_track_y()) data.out_of_track = true;

        data.temp = fmax((data.temp), fabs(robot.get_avg_position().y));
    }

    void finish(fitness_data& data) override
    {
        /* left has positive sign on x axis */
        data.fit = use_avg ? robot.get_avg_position().x : robot.get_min_position().x;
        data.fit -= data.temp;

        if (data.dropped)
            data.fit -= 1.0;// - step_ratio(data);

        /* no penalty for out of track here */
    }
private:
    bool use_avg;
};

class Fitness_Turning : public Fitness_Base
{
public:
    Fitness_Turning(const robots::Simloid& robot, const Setting& s)
    : Fitness_Base("TURNING", robot, s)
    { sts_msg("Evolve turning around."); }

    void start(fitness_data& data) override {
        data.temp = robot.get_avg_rotation();
    }

    void step(fitness_data& data) override
    {
        if (dropped()) data.dropped = true;
        if (stopped(data)) data.stopped = true;

        if (out_of_track_y() || out_of_track_x())
            data.out_of_track = true;

        /* sum of rotation */
        data.temp = unwrap(robot.get_avg_rotation(), data.temp);
    }

    void finish(fitness_data& data) override
    {
        data.fit = data.temp;

        if (data.dropped)
            data.fit -= 2*M_PI;// * (1.0 - step_ratio(data));
    }
};

#endif /* FITNESS_H */
