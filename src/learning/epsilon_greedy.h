#ifndef EPSILON_GREEDY_H_INCLUDED
#define EPSILON_GREEDY_H_INCLUDED

#include <vector>
#include <common/static_vector.h>
#include <common/log_messages.h>
#include <learning/action_selection.h>
#include <learning/action_module.h>
#include <learning/payload.h>

/**TODO move tests away from here, they dirt EVERYTHING.*/

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

//        dbg_msg("Begin Testing of Epsilon Greedy Module");
//        const unsigned avail_actions = actions.get_number_of_actions_available();
//        const unsigned rand_idx = random_index(avail_actions);
//        update_distribution(rand_idx);
//        print_distribution(selection_probabilities);
//        for (unsigned i = 0; i < selection_probabilities.size(); ++i)
//            if (i == rand_idx)
//                assert(selection_probabilities[rand_idx] == 1 - exploration_rate);
//            else {
//                printf("%+1.3f ~ %+1.3f\n", selection_probabilities[i], actions.exists(i) * exploration_rate / (avail_actions-1));
//                assert_close(selection_probabilities[i], actions.exists(i) * exploration_rate / (avail_actions-1), 0.01);
//            }
//        dbg_msg("Testing binning.");
//        std::vector<std::size_t> bins(selection_probabilities.size());
//        for (unsigned i = 0; i < 1000; ++i)
//            ++bins[select_from_distribution(selection_probabilities)];
//
//        for (unsigned i = 0; i < bins.size(); ++i) {
//            printf("%+1.3f ~ %+1.3f\n", bins[i]/1000.0, selection_probabilities[i]);
//            assert_close(bins[i]/1000.0, selection_probabilities[i], 0.05);
//        }
//
//        dbg_msg("End Testing of Epsilon Greedy Module");
    }

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

    std::size_t select_action(std::size_t current_state, std::size_t current_policy)
    {
        update_distribution(states[current_state].policies[current_policy].get_argmax_q());

        /* create random variable and select */
        return select_from_distribution(selection_probabilities);
    }
};

#endif // EPSILON_GREEDY_H_INCLUDED
