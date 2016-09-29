#ifndef BOLTZMANN_SOFTMAX_H_INCLUDED
#define BOLTZMANN_SOFTMAX_H_INCLUDED

#include <vector>
#include <common/static_vector.h>
#include <common/log_messages.h>
#include <learning/action_selection.h>
#include <learning/action_module.h>
#include <learning/payload.h>

class Boltzmann_Softmax : public Action_Selection_Base
{
    /** TODO this does not work for non-existing actions,
     ** rethink variably sized action space, this makes EVERYTHING way too complicated */

    template <typename VectorType>
    static void boltzmann_layer(VectorType& output, const VectorType& input, const double inv_temp) {
        assert(output.size() == input.size());
        assert(output.size() > 0);

        const double maxq = input.get_max();
        double sum = .0;
        for (std::size_t i = 0; i < input.size(); ++i)
            sum += exp(inv_temp * (input[i] - maxq));

        assert(sum > .0);
        for (std::size_t i = 0; i < input.size(); ++i)
            output[i] = exp(inv_temp * (input[i] - maxq)) / sum;
    }

public:
    Boltzmann_Softmax( const static_vector<State_Payload>& states
                     , const Action_Module_Interface&     actions
                     , const double              exploration_rate )
    : Action_Selection_Base(states, actions, exploration_rate)
    {
        dbg_msg("Creating 'Boltzmann/Softmax' action selection.");
        assert_in_range(exploration_rate, 0.01, 0.99);

//        dbg_msg("Begin Testing of Boltzmann/Softmax Module");
//        const unsigned avail_actions = actions.get_number_of_actions_available();
//
//        std::vector<double> qval = random_vector(avail_actions, -10.0, +10.0);
//
//        boltzmann_layer(selection_probabilities, qval, 1.0);
//
//        print_distribution(selection_probabilities);
//        double sump = .0;
//        for (unsigned i = 0; i < selection_probabilities.size(); ++i) {
//            sump = selection_probabilities[i];
//            assert(selection_probabilities[i] > 0 and selection_probabilities[i] < 1.0);
//        }
//        assert_close(sump, 1.0, 0.0001);
//
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
//        dbg_msg("End Testing of Boltzmann/Softmax Module");
    }

    std::size_t select_action(std::size_t current_state, std::size_t current_policy)
    {
        assert(actions.get_number_of_actions_available() > 1);
        double inv_temp = 1.0;//TODO
        boltzmann_layer(selection_probabilities, states[current_state].policies[current_policy].qvalues, inv_temp);

        /* create random variable and select */
        return select_from_distribution(selection_probabilities);
    }
};

#endif // BOLTZMANN_SOFTMAX_H_INCLUDED
