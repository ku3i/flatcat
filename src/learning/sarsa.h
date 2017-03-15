#ifndef SARSA_H_INCLUDED
#define SARSA_H_INCLUDED

#include <common/modules.h>
#include <common/static_vector.h>

#include <learning/reinforcement_learning.h>
#include <learning/sarsa_constants.h>
#include <learning/gmes.h>
#include <learning/reward.h>
#include <learning/payload.h>
#include <learning/action_selection.h>

#include <robots/joint.h>

/**
    "There is a chance that that mouse is going to say ‘yes I see the best move, but...the hell with it’ and jump over the edge!
    All in the name of exploration."

    --Travis DeWolf--
*/

namespace RL {
    typedef std::size_t State;
    typedef std::size_t Action;
}

/** TODO: überprüfe, ob für die off-policies Q-learning bessere Ergebnisse liefert als SARSA,
 *  Antwort: Möglicherweise gibt es Probleme den Max-Operator zusammen mit nichtstationären Aktionen zu verwenden. */
class SARSA : public learning::RL_Interface {
public:

    SARSA( static_vector<State_Payload>& states
         , const reward_base& rewards
         , /*const*/ Action_Selection_Base& action_selection
         , const std::size_t number_of_actions
         , const std::vector<double>& learning_rates
         , RL::State initial_state = 0
         , RL::Action initial_action = 0 )
    : states(states)
    , rewards(rewards)
    , action_selection(action_selection)
    , current_state(initial_state)
    , last_state()
    , current_action(initial_action)
    , last_action()
    , number_of_policies(states[0].policies.size())
    , number_of_actions(number_of_actions)
    , current_policy(0)
    , deltaQ(number_of_policies)
    , learning_rates(learning_rates)
    {
        sts_msg("Creating discrete Reinforcement Learner: SARSA.\
                \n  states = %u\n  actions = %u\n  policies = %u"
                , states.size(), number_of_actions, number_of_policies);
        assert(number_of_policies > 0);
        assert(number_of_actions > 0);
        assert_in_range(learning_rates, 0.0001, 0.5);
        assert(learning_rates.size() == number_of_policies);
    }

    double      get_current_reward    (std::size_t index) const { return rewards.get_current_reward(index); }
    bool        positive_current_delta(std::size_t index) const { assert(index < number_of_policies); return deltaQ[index] > 0.0; }
    std::size_t get_current_policy    (void) const { return current_policy;     }
    std::size_t get_current_action    (void) const { return current_action;     }
    std::size_t get_current_state     (void) const { return current_state;      }
    std::size_t get_number_of_policies(void) const { return number_of_policies; }
    std::size_t get_number_of_actions (void) const { return number_of_actions;  }

    void select_policy(std::size_t index, bool print_status = true)
    {
        if (index < number_of_policies) {
            current_policy = index;
            if (print_status)
                sts_msg("Selecting policy: %s", rewards.get_reward_name(current_policy).c_str());
        }
        else wrn_msg("Invalid policy");
    }

    void decay_eligibility_traces(void) {
        /**TODO non existing states should be reset to zero */
        for (std::size_t s = 0; s < states.size(); ++s)
            for (std::size_t a = 0; a < number_of_actions; ++a)
                states[s].eligibility_trace[a].decay();
    }

    void execute_cycle(RL::State new_state)
    {
        /* save old states */
        last_state  = current_state;
        last_action = current_action;

        /* set new state */
        current_state = new_state;

        /* action selection (e.g.Epsilon Greedy or Boltzmann)  */
        current_action = action_selection.select_action(current_state, current_policy);

        /* Q-learning (SARSA) */
        assert(deltaQ.size() == number_of_policies);
        assert(states[last_state].eligibility_trace.size() == number_of_actions);

        for (std::size_t pi = 0; pi < number_of_policies; ++pi)
            deltaQ[pi] = rewards.get_aggregated_last_reward(pi) + sarsa_constants::GAMMA * states[current_state].policies[pi].qvalues[current_action]
                                                                                         - states[last_state   ].policies[pi].qvalues[last_action   ];

        /* decay eligibility traces */
        if (current_state != last_state)
            decay_eligibility_traces();

        /* reset trace */
        states[last_state].eligibility_trace[last_action].reset();

        /* update all Q-Values according to their trace */
        /**TODO this method is inherently slow. try to improve */
        for (std::size_t s = 0; s < states.size(); ++s)
            for (std::size_t a = 0; a < number_of_actions; ++a)
                for (std::size_t pi = 0; pi < number_of_policies; ++pi)
                    states[s].policies[pi].qvalues[a] += learning_rates[pi] * deltaQ[pi] * states[s].eligibility_trace[a].get();
    }

private:

    static_vector<State_Payload>& states;
    const reward_base&            rewards;
    Action_Selection_Base&        action_selection;

    RL::State current_state;
    RL::State last_state;

    RL::Action current_action;
    RL::Action last_action;

    const std::size_t number_of_policies;
    const std::size_t number_of_actions;
          std::size_t current_policy;

    std::vector<double> deltaQ;

    const std::vector<double> learning_rates;

    friend class SARSA_Graphics;
    friend class Policy_Selector_Graphics;
};

class Policy_Selector /**TODO: move to separate file */
{
    SARSA&                sarsa;
    const std::size_t     number_of_policies;
          std::size_t     current_policy;

    std::vector<uint64_t> policy_trial_duration;
    uint64_t              cycles;
    bool                  random_policy_mode;

public:
    Policy_Selector(SARSA& sarsa, const std::size_t number_of_policies, bool random_policy_mode = true, uint64_t default_duration_s = 10)
    : sarsa(sarsa)
    , number_of_policies(number_of_policies)
    , current_policy(0)
    , policy_trial_duration(number_of_policies, default_duration_s * 100) /* make constant in setup */
    , cycles(0)
    , random_policy_mode(random_policy_mode)
    {
        dbg_msg("Creating Policy Selector with %u policies.", number_of_policies);
    }

    void toggle_random_policy_mode(void) {
        random_policy_mode = !random_policy_mode;
        sts_msg("Random Policy Mode: %s", random_policy_mode ? "on" : "off");
    }

    void select_random_policy(void) {
        /* select a random policy which is different from the previous one */
        if (number_of_policies > 1) {
            current_policy += random_int(1, number_of_policies - 1);
            current_policy %= number_of_policies;
            assert(current_policy < number_of_policies);
        }
        sarsa.select_policy(current_policy, false);
        cycles = 0;
    }

    void select_policy(std::size_t new_policy) {
        if (new_policy >= number_of_policies) {
            dbg_msg("No such policy: %u", new_policy);
            return;
        }
        current_policy = new_policy;
        sarsa.select_policy(current_policy);
        cycles = 0;
    }

    void execute_cycle(void) {
        if (random_policy_mode) {
            ++cycles;
            if (cycles >= policy_trial_duration[current_policy])
                select_random_policy();
        }
    }

    void set_policy_trial_duration(std::size_t index, uint64_t duration) {
        assert(index < policy_trial_duration.size());
        policy_trial_duration[index] = duration;
    }

    uint64_t get_trial_time_left(void) const {
        return policy_trial_duration[current_policy] - cycles;
    }

    std::size_t size() const { return number_of_policies; }

    friend class Policy_Selector_Graphics;
};

class Policy_Selector_Graphics : public Graphics_Interface {
    const Policy_Selector& policy_selector;
public:
    Policy_Selector_Graphics(const Policy_Selector& policy_selector) : policy_selector(policy_selector) {}

    void draw(const pref& p) const {
        unsigned int time_left = policy_selector.get_trial_time_left();
        unsigned int seconds = (time_left / 100) % 60;
        unsigned int hsecs   = (time_left % 100);
        glColor3f(1.0,1.0,1.0);
        glprintf(-0.9, 0.95, 0.0, 0.03, "[%u] %s", policy_selector.current_policy
                                                 , policy_selector.sarsa.rewards.get_reward_name(policy_selector.current_policy).c_str());
        glprintf(-0.9, 0.90, 0.0, 0.03, "left: %02u:%02u [%c]" , seconds, hsecs, policy_selector.random_policy_mode? '~':'=');
    }
};
#endif // SARSA_H_INCLUDED
