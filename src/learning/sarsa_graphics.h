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

class SARSA_Graphics : public Graphics_Interface {
public:
    SARSA_Graphics(const SARSA& sarsa) //TODO make position on the screen configurable
    : sarsa(sarsa)
    , num_policies(sarsa.get_number_of_policies())
    , last_policy(sarsa.get_current_policy())
    , axis_reward_systemstep()
    , axis_reward_eigenstep ()
    , axis_reward_trial     ()
    , axis_reward_bunch()
    , plot_reward_systemstep()
    , plot_reward_eigenstep ()
    , plot_reward_trial     ()
    , plot_reward_bunch     ()
    , reward_trial(num_policies)
    , reward_bunch(num_policies)
    , table(4)
    {
        axis_reward_eigenstep .reserve(num_policies);
        axis_reward_systemstep.reserve(num_policies);
        axis_reward_trial     .reserve(num_policies);
        axis_reward_bunch     .reserve(num_policies);
        plot_reward_systemstep.reserve(num_policies);
        plot_reward_eigenstep .reserve(num_policies);
        plot_reward_trial     .reserve(num_policies);
        plot_reward_bunch     .reserve(num_policies);

        for (std::size_t i = 0; i < sarsa.get_number_of_policies(); ++i) {
            axis_reward_systemstep.emplace_back(-1.0, 0.00 - 0.2*i, 0.0, 0.45, .18, 0, "");
            axis_reward_eigenstep .emplace_back(-1.5, 0.00 - 0.2*i, 0.0, 0.45, .18, 0, "");
            axis_reward_trial     .emplace_back(-2.0, 0.00 - 0.2*i, 0.0, 0.45, .18, 0, sarsa.rewards.get_reward_name(i));
            axis_reward_bunch     .emplace_back(-2.5, 0.00 - 0.2*i, 0.0, 0.45, .18, 0, "");

            plot_reward_systemstep.emplace_back(400   , axis_reward_systemstep[i], Color4::set_transparency(table.get_color(i), 1.00));
            plot_reward_eigenstep .emplace_back(400   , axis_reward_eigenstep [i], Color4::set_transparency(table.get_color(i), 0.75));
            plot_reward_trial     .emplace_back(100   , axis_reward_trial     [i], Color4::set_transparency(table.get_color(i), 0.50));
            plot_reward_bunch     .emplace_back(100   , axis_reward_bunch     [i], Color4::set_transparency(table.get_color(i), 1.00));
        }

        dbg_msg("Creating Sarsa Graphics.");
    }

    void execute_cycle(uint64_t cycle, bool state_changed)
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

        if (last_policy != sarsa.get_current_policy()) {  // policy changed, trial ended
            last_policy = sarsa.get_current_policy();

            std::size_t i = last_policy;

            reward_bunch[i].add(reward_trial[i].get_avg_value());
            plot_reward_trial[i].add_sample(reward_trial[i].get_avg_value_and_reset());

            if (reward_bunch[i].get_number_of_samples() >= 10)
                plot_reward_bunch[i].add_sample(reward_bunch[i].get_avg_value_and_reset());
        }
    }

    void draw(const pref& p) const
    {
        for (std::size_t i = 0; i < num_policies; ++i) {
            axis_reward_systemstep[i].draw();
            axis_reward_eigenstep [i].draw();
            axis_reward_trial     [i].draw();
            axis_reward_bunch     [i].draw();
            plot_reward_systemstep[i].draw();
            plot_reward_eigenstep [i].draw();
            plot_reward_trial     [i].draw();
            plot_reward_bunch     [i].draw();
        }

        const std::size_t cur_policy = sarsa.get_current_policy();
        glColor3f(1.0, 1.0, 1.0);
        glprintf(-0.9, 0.95, 0.0, 0.03, "[%u] %s", cur_policy, sarsa.rewards.get_reward_name(cur_policy).c_str());
        glprintf(-0.9, 0.75, 0.0, 0.03, "cur: %+.4f", sarsa.get_current_reward(sarsa.get_current_policy()));

        //TODO draw the eligibilities, draw the path
    }

private:

    const SARSA&        sarsa;
    const std::size_t   num_policies;
    std::size_t         last_policy;

    std::vector<axes>   axis_reward_systemstep;
    std::vector<axes>   axis_reward_eigenstep;
    std::vector<axes>   axis_reward_trial;
    std::vector<axes>   axis_reward_bunch;

    std::vector<plot1D> plot_reward_systemstep;
    std::vector<plot1D> plot_reward_eigenstep;
    std::vector<plot1D> plot_reward_trial;
    std::vector<plot1D> plot_reward_bunch;

    std::vector<Integrator> reward_trial;
    std::vector<Integrator> reward_bunch;
    const ColorTable    table;
};

#endif // SARSA_GRAPHICS_H_INCLUDED

