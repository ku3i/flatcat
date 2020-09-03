#ifndef ACTION_SELECTION_H_INCLUDED
#define ACTION_SELECTION_H_INCLUDED

#include <common/static_vector.h>
#include <common/log_messages.h>
#include <learning/action_module.h>
#include <learning/payload.h>

template <typename Vector_t> std::size_t select_from_distribution(Vector_t const& distribution);

class Action_Selection_Base
{
public:
    typedef static_vector<double>       Vector_t;

protected:
    Vector_t                            selection_probabilities;
    const static_vector<State_Payload>& states;
    const Action_Module_Interface&      actions;
    const double                        exploration_rate;

    bool                                explorative_selection = false;

public:
    Action_Selection_Base( const static_vector<State_Payload>& states
                         , const Action_Module_Interface&     actions
                         , const double              exploration_rate )
    : selection_probabilities(actions.get_number_of_actions())
    , states(states)
    , actions(actions)
    , exploration_rate(exploration_rate)
    {}
    const Vector_t& get_distribution(void) const { return selection_probabilities; }

    virtual ~Action_Selection_Base() = default;
    virtual std::size_t select_action( std::size_t current_state, std::size_t current_policy) = 0;

    bool is_exploring(void) { return explorative_selection; }

    std::size_t select_randomized(void) {
        selection_probabilities.zero(); // set all zeros
        const double portion = 1.0 / actions.get_number_of_actions_available();

        for (std::size_t i = 0; i < selection_probabilities.size(); ++i)
            if (actions.exists(i))
                selection_probabilities[i] = portion;

        explorative_selection = true;

        return select_from_distribution(selection_probabilities); // uniform, only available actions
    }

};

void print_distribution(const Action_Selection_Base::Vector_t& distribution);


/** Selects an index from given discrete probability distribution.
 *  The given distribution must sum up to 1.
 */
template <typename Vector_t>
std::size_t
select_from_distribution(Vector_t const& distribution)
{
    const double x = random_value(0.0, 1.0);
    double sum = 0.0;
    for (std::size_t i = 0; i < distribution.size(); ++i)
    {
        assert((0.0 <= distribution[i]) and (distribution[i] <= 1.0));
        sum += distribution[i];
        if (x < sum)
            return i;
        assert(sum < 1.0);
    }
    dbg_msg("sum: %1.4f", sum);
    assert(false);
}

#endif // ACTION_SELECTION_H_INCLUDED
