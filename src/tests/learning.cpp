#include <tests/catch.hpp>
#include <vector>

#include <common/modules.h>
#include <common/static_vector.h>
#include <learning/payload.h>
#include <learning/epsilon_greedy.h>


class no_actions : public Action_Module_Interface {
public:
    std::size_t get_number_of_actions(void) const { return 7; }
    std::size_t get_number_of_actions_available(void) const { return 6; }
    bool exists(const std::size_t action_index) const {
        if (action_index == 3) return false; // id 3 is not available
        else return true;
    }
};

TEST_CASE( "Epsilon Greedy" , "[learning]") {
    REQUIRE( true );

    const unsigned num_states = 10;
    const unsigned num_policies = 3;
    const no_actions actions;
    static_vector<State_Payload> states(num_states, actions, num_policies, 0.0);

    learning::Epsilon_Greedy greedy(states, actions, 0.01);

    const unsigned avail_actions = actions.get_number_of_actions_available();
    const unsigned rand_idx = random_index(avail_actions);

    Action_Selection_Base::Vector_t const& selection_probabilities = greedy.get_distribution();
    print_distribution(selection_probabilities);

    /*check that probabilities are restricted to [0,1] and sum up to 1 */
    auto check_probabilities = [&](Action_Selection_Base::Vector_t const& prob) {
        double sum = .0;
        for (unsigned i = 0; i < prob.size(); ++i) {
            sum += prob[i];
            REQUIRE( in_range(prob[i], 0.0, 1.0) );
        }
        REQUIRE( in_range(sum, 0.0, 1.0) );
    };

    check_probabilities(selection_probabilities);

    /* check distribution of selected actions */
    auto test_selection = [&](learning::Epsilon_Greedy g, unsigned state_id, unsigned policy_id, unsigned action_id) {
        states[state_id].policies[policy_id].qvalues[action_id] = 1.0; // set Q-value
        unsigned result = g.select_action(state_id, policy_id);
        states[state_id].policies[policy_id].qvalues[action_id] = 0.0; // clear it
        return result;
    };

    auto check_distribution = [&](learning::Epsilon_Greedy g, unsigned state_id, unsigned policy_id, unsigned action_id, double expected) {
        REQUIRE( state_id < states.size() );
        REQUIRE( policy_id < states[0].policies.size() );
        REQUIRE( action_id < states[0].policies[0].qvalues.size() );
        const unsigned total = 1000;
        unsigned counter = 0;
        for (unsigned i = 0; i < total; ++i)
            if (action_id == test_selection(g, state_id, policy_id, action_id)) ++counter;
        dbg_msg("Selection rate: %3u/%4u  %4.1f ~ %4.1f", counter, total, 100.0*counter/total, expected);
        REQUIRE( close(100.0*counter/total, expected, 2.0) ); // values in 2% tolerance
    };

    learning::Epsilon_Greedy greedy_10(states, actions, 0.10);
    learning::Epsilon_Greedy greedy_40(states, actions, 0.40);
    learning::Epsilon_Greedy greedy_70(states, actions, 0.70);

    check_distribution(greedy_10, 0,0,0, 90.0); check_probabilities(greedy_10.get_distribution());
    check_distribution(greedy_10, 1,2,3,  0.0); check_probabilities(greedy_10.get_distribution()); // action not available
    check_distribution(greedy_40, 9,1,6, 60.0); check_probabilities(greedy_40.get_distribution());
    check_distribution(greedy_70, 3,0,1, 30.0); check_probabilities(greedy_70.get_distribution());
}



TEST_CASE( "select_from_distribution" ,"[eps_greedy]")
{
    Action_Selection_Base::Vector_t selection_probabilities{5};
    selection_probabilities[0] = 0.50; //50
    selection_probabilities[1] = 0.25; //75
    selection_probabilities[2] = 0.10; //85
    selection_probabilities[3] = 0.10; //95
    selection_probabilities[4] = 0.05; //100

    /* Testing binning.*/
    std::vector<std::size_t> bins(selection_probabilities.size());
    for (unsigned i = 0; i < 1000; ++i)
        ++bins[select_from_distribution(selection_probabilities)];

    for (unsigned i = 0; i < bins.size(); ++i) {
        dbg_msg("%+1.3f ~ %+1.3f", bins[i]/1000.0, selection_probabilities[i]);
        REQUIRE( close(bins[i]/1000.0, selection_probabilities[i], 0.05) );
    }
}
