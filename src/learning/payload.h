#ifndef PAYLOAD_H_INCLUDED
#define PAYLOAD_H_INCLUDED

#include <vector>
#include <common/static_vector.h>
#include <common/log_messages.h>
#include <learning/q_function.h>
#include <learning/eligibility.h>

class Empty_Payload
{
public:
    Empty_Payload() {}
    Empty_Payload& operator=(const Empty_Payload& /*other*/) { return *this; };
};

class State_Payload
{
public:
    State_Payload(const Action_Module_Interface& actions, std::size_t number_of_policies, double q_initial)
    : policies(number_of_policies, actions, q_initial)
    , eligibility_trace(actions.get_number_of_actions())
    {}

    State_Payload& operator=(const State_Payload& other) {
        assert(this != &other); // no self-assignment
        copy_with_flaws(other); // redirect copy-assignment
        return *this;
    }

    /* This must be used by the motor layer to copy states associated to the copied action.*/
    void copy_payload(std::size_t from_idx, std::size_t to_idx) {
        for (std::size_t policy_idx = 0; policy_idx < policies.size(); ++policy_idx)
            policies[policy_idx].copy_q_value(from_idx, to_idx);
        eligibility_trace[to_idx] = eligibility_trace[from_idx];
    }

private:
    void copy_with_flaws(const State_Payload& other)
    {
        /* inherit flawed Q-values, i.e. mutate, hidden in the copy assignment [!] */
        policies = other.policies;

        /* inherit flawless eligibility traces */
        eligibility_trace = other.eligibility_trace;
    }
public:
    copyable_static_vector<Policy> policies;
    copyable_static_vector<Eligibility> eligibility_trace;

    friend class SARSA;
    friend class Epsilon_Greedy;
    friend class Boltzmann_Softmax;
    friend class State_Payload_Graphics;
    friend class Payload_Graphics;
};


class Motor_Payload
{
    typedef static_vector<State_Payload> state_vector_t;

    std::size_t index = 0;
    state_vector_t* states = nullptr;

public:
    Motor_Payload(){}
    Motor_Payload& operator=(const Motor_Payload& other) {
        assert(this != &other); // no self-assignment
        //copy_with_flaws(other); // redirect copy-assignment
        sts_msg("Copy action value from %lu to %lu.", other.index, index);
        assert(other.index != index); /* If this was thrown, check payloads connected. */
        assert(states != nullptr);
        for (std::size_t i = 0; i < (*states).size(); ++i)
            (*states)[i].copy_payload(other.index, index);
        return *this;
    }

    /* Two-phase initialization for connecting payloads.*/
    void connect(std::size_t idx, state_vector_t* s) {
        index = idx;
        states = s;
    }
};


#endif // PAYLOAD_H_INCLUDED
