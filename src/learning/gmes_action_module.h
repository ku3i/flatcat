#ifndef GMES_ACTION_MODULE_H_INCLUDED
#define GMES_ACTION_MODULE_H_INCLUDED

#include <common/log_messages.h>
#include <learning/action_module.h>
#include <learning/motor_layer.h>
#include <learning/reinforcement_learning.h>

namespace learning {

/** Maybe we don't need this class at all. Consider to move that into motor layer*/

class gmes_action_module : public Action_Module_Interface {

    Motor_Layer const& motor_layer;

public:
    gmes_action_module( Motor_Layer const& motor_layer )
    : motor_layer(motor_layer)
    {
        dbg_msg("Creating gmes_action_module with max. number of actions %u:", get_number_of_actions());
    }

    ~gmes_action_module() = default;

    control::Control_Parameter const& get_controller_weights(std::size_t id) const { return motor_layer.get_controller_weights(id); }

    std::size_t get_number_of_actions(void)           const { return motor_layer.get_max_number_of_experts(); }
    std::size_t get_number_of_actions_available(void) const { return motor_layer.get_cur_number_of_experts(); }
    bool exists(const std::size_t action_index)       const { return motor_layer.has_expert(action_index); }
};

} // namespace learning

#endif // GMES_ACTION_MODULE_H_INCLUDED
