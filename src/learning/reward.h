#ifndef REWARD_H_INCLUDED
#define REWARD_H_INCLUDED

#include <common/integrator.h>
#include <control/sensorspace.h>

class reward_base
{
protected:
    std::vector<sensor_signal> rewards;
    std::vector<Integrator>    aggregated_rewards;

public:
    reward_base(std::size_t number_of_policies_to_reserve)
    : rewards()
    , aggregated_rewards(number_of_policies_to_reserve)
    {
        dbg_msg("Creating reward base, reserving %u items.", number_of_policies_to_reserve);
        rewards.reserve(number_of_policies_to_reserve);
    }

    virtual ~reward_base() {}

    const double       get_current_reward(std::size_t index) const { assert(index < rewards.size()); return rewards[index].current; }
    const double       get_last_reward   (std::size_t index) const { assert(index < rewards.size()); return rewards[index].last;    }
    const std::string& get_reward_name   (std::size_t index) const { assert(index < rewards.size()); return rewards[index].name;    }
    const std::size_t  get_number_of_policies(void)          const { return rewards.size(); }

    const double get_aggregated_last_reward(std::size_t index) const {
        assert(index < rewards.size());
        return aggregated_rewards[index].get_avg_value();
    }

    void clear_aggregations(void) {
        for (std::size_t i = 0; i < aggregated_rewards.size(); ++i)
            aggregated_rewards[i].reset();
    }

    void execute_cycle(void) {
        assert(aggregated_rewards.size() >= rewards.size());
        for (std::size_t i = 0; i < rewards.size(); ++i) {
            rewards[i].execute_cycle();
            aggregated_rewards[i].add(rewards[i].current);
        }
    }
};


#endif // REWARD_H_INCLUDED
