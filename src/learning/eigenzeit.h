#ifndef LEARNING_EIGENZEIT_H
#define LEARNING_EIGENZEIT_H

#include <common/log_messages.h>
#include <control/statemachine.h>

namespace learning {

/** Eigenzeit.h
 ** The concept of eigenzeit constitutes a mechanism for translating the inherent time step (cycle)
 ** of usually 10ms (100Hz) into a 'time' or cycle which only depends on gmes' state changes. Eigenzeit
 ** passes only when a new state is recognized by gmes. If a state does not change within a given time
 ** it is recognized as a state transition into itself and another eigenzeit cycle is executed. */

class Eigenzeit
{
    const control::Statemachine_Interface&  statemachine;
    const uint64_t timeout_10ms;
          uint64_t system_cycle;
          uint64_t eigenzeit_cycle;

public:
    Eigenzeit(const control::Statemachine_Interface& statemachine, const uint64_t timeout_10ms = 100)
    : statemachine(statemachine)
    , timeout_10ms(timeout_10ms)
    , system_cycle(0)
    , eigenzeit_cycle(0)
    {
        dbg_msg("Creating Eigenzeit with max. timeout of %1.2f s", timeout_10ms/(100.0));
        assert(timeout_10ms > 0);
    }

    void execute_cycle(void)
    {
        ++system_cycle;
        if (statemachine.has_state_changed() or (system_cycle >= timeout_10ms)) {
            system_cycle = 0;
            ++eigenzeit_cycle;
        }
    }

    bool has_progressed(void) const { return system_cycle == 0; }
    uint64_t get_cycle (void) const { return eigenzeit_cycle;   }
};

} /* namespace learning */

#endif /* LEARNING_EIGENZEIT_H */
