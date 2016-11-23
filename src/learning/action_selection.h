#ifndef ACTION_SELECTION_H_INCLUDED
#define ACTION_SELECTION_H_INCLUDED

#include <common/static_vector.h>
#include <common/log_messages.h>
#include <learning/action_module.h>
#include <learning/payload.h>

class Action_Selection_Base
{
public:
    typedef static_vector<double>       Vector_t;

protected:
    Vector_t                            selection_probabilities;
    const static_vector<State_Payload>& states;
    const Action_Module_Interface&      actions;
    const double                        exploration_rate;

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
};

std::size_t select_from_distribution(const Action_Selection_Base::Vector_t& distribution);
void print_distribution(const Action_Selection_Base::Vector_t& distribution);


#endif // ACTION_SELECTION_H_INCLUDED
