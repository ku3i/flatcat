#ifndef SARSA_GRAPHICS_H_INCLUDED
#define SARSA_GRAPHICS_H_INCLUDED

#include <common/integrator.h>

#include <draw/draw.h>
#include <draw/axes.h>
#include <draw/axes3D.h>
#include <draw/plot1D.h>
#include <draw/plot3D.h>
#include <draw/network3D.h>
#include <draw/color_table.h>
#include <draw/graphics.h>

#include "sarsa.h"

/** REWARD DISPLAY

 Shows the rewards on different time scales

 1) Immediate reward r(t), according to system time t.
 2) Immediate reward r(T), according to eigentime T.
 3) Trial reward, accumulated eigenstep rewards for a full trial (Episode), before switching to another policy
 4) Bunch reward, trial rewards over time.

 */

class SARSA_Graphics : public Graphics_Interface {
public:
    SARSA_Graphics(const SARSA& sarsa) //TODO make position on the screen configurable
    : sarsa(sarsa)
    , num_policies(sarsa.get_number_of_policies())
    , last_policy(sarsa.get_current_policy())
    , table(4)
    , axis_reward_total(-0.0, -0.5, 0.0, 1.98, 0.98, 0, "total", 0.1)
    , axis_reward_systemstep()
    , axis_reward_eigenstep ()
    , axis_reward_trial     ()
    , axis_reward_bunch()
    , plot_reward_total(2000, axis_reward_total, Color4::set_transparency(colors::cyan_l, 0.30), "total")
    , plot_reward_t_avg(2000, axis_reward_total, Color4::set_transparency(colors::cyan  , 0.60), "t_avg")
    , plot_reward_t_max(2000, axis_reward_total, Color4::set_transparency(colors::orange, 0.60), "t_max")
    , plot_reward_systemstep()
    , plot_reward_eigenstep ()
    , plot_reward_trial     ()
    , plot_reward_bunch     ()
    //, plot_reward_trial_long()
    , reward_trial(num_policies)
    , reward_bunch(num_policies)
    {
        axis_reward_eigenstep .reserve(num_policies);
        axis_reward_systemstep.reserve(num_policies);
        axis_reward_trial     .reserve(num_policies);
        axis_reward_bunch     .reserve(num_policies);
        plot_reward_systemstep.reserve(num_policies);
        plot_reward_eigenstep .reserve(num_policies);
        plot_reward_trial     .reserve(num_policies);
        plot_reward_bunch     .reserve(num_policies);

        unsigned N = sarsa.get_number_of_policies();
        for (std::size_t i = 0; i < N; ++i) {
            float ypos = N/2 *0.2 - 0.2*i + 0.5;
            axis_reward_systemstep.emplace_back(+0.75, ypos, 0.0, 0.48, 0.18, 0, "r(t)"   , 0.001);
            axis_reward_eigenstep .emplace_back(+0.25, ypos, 0.0, 0.48, 0.18, 0, "r(T)"   , 0.001);
            axis_reward_trial     .emplace_back(-0.25, ypos, 0.0, 0.48, 0.18, 0, "R/trial", 0.001);
            axis_reward_bunch     .emplace_back(-0.75, ypos, 0.0, 0.48, 0.18, 0, sarsa.rewards.get_reward_name(i).substr(0,14), 0.001);

            plot_reward_systemstep.emplace_back(400 , axis_reward_systemstep[i], Color4::set_transparency(table.get_color(i), 1.00));
            plot_reward_eigenstep .emplace_back(400 , axis_reward_eigenstep [i], Color4::set_transparency(table.get_color(i), 0.80));
            plot_reward_trial     .emplace_back(100 , axis_reward_trial     [i], Color4::set_transparency(table.get_color(i), 0.60));
            plot_reward_bunch     .emplace_back(100 , axis_reward_bunch     [i], Color4::set_transparency(table.get_color(i), 0.40));
            //plot_reward_trial_long.emplace_back(2000, axis_reward_total        , Color4::set_transparency(table.get_color(i), 0.75), "L-Trial");
        }

        dbg_msg("Creating Sarsa Graphics for %lu policies.", num_policies);
    }

    void execute_cycle(uint64_t /*cycle*/, bool state_changed, bool trial_ended)
    {

        for (std::size_t i = 0; i < num_policies; ++i)
            plot_reward_systemstep[i].add_sample(sarsa.rewards.get_current_reward(i));


        if (state_changed)
        {
            assert(sarsa.rewards.get_number_of_policies() == num_policies);

            for (std::size_t i = 0; i < num_policies; ++i)
            {
                if (sarsa.get_current_policy() == i) {
                    const double value = sarsa.rewards.get_aggregated_last_reward(i);
                    plot_reward_eigenstep[i].add_sample(value);
                    reward_trial[i].add(value);
                }
            }
        }

        if (/*last_policy != sarsa.get_current_policy() or*/ trial_ended) {  // policy changed, trial ended
            std::size_t i = last_policy;
            last_policy = sarsa.get_current_policy();

            const float trial_reward_i = reward_trial[i].get_avg_value_and_reset();
            reward_bunch[i].add(trial_reward_i);
            plot_reward_trial[i].add_sample(trial_reward_i);

            if (reward_bunch[i].get_number_of_samples() >= 10)
                plot_reward_bunch[i].add_sample(reward_bunch[i].get_avg_value_and_reset());

            //plot_reward_trial_long[i].add_sample(trial_reward_i);

            ++trial_count;
            if (trial_count % num_policies == 0) {
                double total = .0;
                for (auto const& r: reward_trial)
                    total += r.get_last();
                reward_t_avg = 0.95*reward_t_avg + 0.05*total;
                reward_t_max = std::max(reward_t_max, reward_t_avg);
                plot_reward_total.add_sample(total);
                plot_reward_t_avg.add_sample(reward_t_avg);
                plot_reward_t_max.add_sample(reward_t_max);
            }
        }
    }

    void draw(const pref& /*p*/) const
    {
        axis_reward_total.draw();
        plot_reward_total.draw();
        plot_reward_t_max.draw();
        plot_reward_t_avg.draw();
        for (std::size_t i = 0; i < num_policies; ++i) {
            axis_reward_systemstep[i].draw();
            axis_reward_eigenstep [i].draw();
            axis_reward_trial     [i].draw();
            axis_reward_bunch     [i].draw();
            plot_reward_systemstep[i].draw();
            plot_reward_eigenstep [i].draw();
            plot_reward_trial     [i].draw();
            plot_reward_bunch     [i].draw();
            //plot_reward_trial_long[i].draw();
        }

        glColor3f(1.0, 1.0, 1.0);
        glprintf(-0.9, 1.0, 0.0, 0.03, "cur: %+.4f", sarsa.get_current_reward(sarsa.get_current_policy()));
    }

private:

    const SARSA&        sarsa;
    const std::size_t   num_policies;
    std::size_t         last_policy;

    std::size_t         trial_count = 0;
    double              reward_t_avg = .0;
    double              reward_t_max = .0;

    const ColorTable    table;
    axes                axis_reward_total;
    std::vector<axes>   axis_reward_systemstep;
    std::vector<axes>   axis_reward_eigenstep;
    std::vector<axes>   axis_reward_trial;
    std::vector<axes>   axis_reward_bunch;

    plot1D              plot_reward_total;
    plot1D              plot_reward_t_avg;
    plot1D              plot_reward_t_max;
    std::vector<plot1D> plot_reward_systemstep;
    std::vector<plot1D> plot_reward_eigenstep;
    std::vector<plot1D> plot_reward_trial;
    std::vector<plot1D> plot_reward_bunch;
    //std::vector<plot1D> plot_reward_trial_long;

    std::vector<Integrator> reward_trial;
    std::vector<Integrator> reward_bunch;

};

class Policy_Selector_Graphics : public Graphics_Interface {
    const Policy_Selector& policy_selector;
public:
    Policy_Selector_Graphics(const Policy_Selector& policy_selector) : policy_selector(policy_selector) {}

    void draw(const pref& /*p*/) const {
        unsigned int time_left = policy_selector.get_trial_time_left();
        unsigned int minutes = (time_left / 6000);
        unsigned int seconds = (time_left / 100) % 60;
        unsigned int hsecs   = (time_left % 100);
        glColor3f(1.0,1.0,1.0);
        glprintf(0.0,-0.05, 0.0, 0.03, "[%u] %s", policy_selector.current_policy
                                                , policy_selector.sarsa.rewards.get_reward_name(policy_selector.current_policy).c_str());
        glprintf(0.0,-0.10, 0.0, 0.03, "left: %u:%02u:%02u [%c]", minutes, seconds, hsecs, policy_selector.profile.play ? '+' :
                                                                                           policy_selector.random_policy_mode ? '~' : '=');
    }
};

#endif // SARSA_GRAPHICS_H_INCLUDED

