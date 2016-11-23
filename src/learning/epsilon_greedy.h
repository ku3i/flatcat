#ifndef EPSILON_GREEDY_H_INCLUDED
#define EPSILON_GREEDY_H_INCLUDED

#include <vector>
#include <common/static_vector.h>
#include <common/log_messages.h>
#include <learning/action_selection.h>
#include <learning/action_module.h>
#include <learning/payload.h>

class Epsilon_Greedy : public Action_Selection_Base
{

public:
    Epsilon_Greedy( const static_vector<State_Payload>& states
                  , const Action_Module_Interface&     actions
                  , const double              exploration_rate )
    : Action_Selection_Base(states, actions, exploration_rate)
    {
        dbg_msg("Creating 'Epsilon Greedy' action selection.");
        assert_in_range(exploration_rate, 0.01, 0.99);
    }

private:
    void update_distribution(const std::size_t greedy_action)
    {
        assert(actions.exists(greedy_action));
        assert(actions.get_number_of_actions_available() > 1);

        const double non_greedy_portion = exploration_rate / (actions.get_number_of_actions_available() - 1);

        for (std::size_t i = 0; i < selection_probabilities.size(); ++i)
            if (actions.exists(i) and (i != greedy_action))
                selection_probabilities[i] = non_greedy_portion;

        selection_probabilities[greedy_action] = 1.0 - exploration_rate;
    }

public:
    std::size_t select_action(std::size_t current_state, std::size_t current_policy) override
    {
        update_distribution(states[current_state].policies[current_policy].get_argmax_q());

        /* create random variable and select */
        return select_from_distribution(selection_probabilities);
    }
};

#endif // EPSILON_GREEDY_H_INCLUDED
