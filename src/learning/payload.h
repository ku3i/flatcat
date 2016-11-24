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
    Empty_Payload& operator=(const Empty_Payload& other) { return *this; };
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

    void copy_payload(std::size_t from_idx, std::size_t to_idx) {
        for (std::size_t policy_idx = 0; policy_idx < policies.size(); ++policy_idx)
            policies[policy_idx].copy_q_value(from_idx, to_idx);
        eligibility_trace[to_idx] = eligibility_trace[from_idx];
    }

private:
    void copy_with_flaws(const State_Payload& other)
    {
        //dbg_msg("Copy with flaws");
        /* inherit flawed Q-values, i.e. mutate, hidden in the copy assignment [!] */
        policies = other.policies;

        /* inherit flawless eligibility traces */
        eligibility_trace = other.eligibility_trace;
    }

    copyable_static_vector<Policy> policies;
    copyable_static_vector<Eligibility> eligibility_trace;

    friend class SARSA;
    friend class Epsilon_Greedy;
    friend class Boltzmann_Softmax;
    friend class State_Payload_Graphics;
    template <typename T> friend class Payload_Graphics;
};

#include <draw/graphics.h>
#include <draw/display.h>
#include <common/vector2.h>

class State_Payload_Graphics : public Graphics_Interface {

    typedef static_vector<State_Payload> Payload_Vector_t;

    const Payload_Vector_t& payloads;
    const std::size_t       num_policies, num_states;

public:
    State_Payload_Graphics(const Payload_Vector_t& payloads)
    : payloads(payloads)
    , num_policies(payloads[0].policies.size())
    , num_states(payloads.size())
    {}
    void draw(const pref& p) const {
        for (std::size_t i = 0; i < num_policies; ++i)
            for (std::size_t s = 0; s < std::min(10ul,num_states); ++s)
                draw::vec3(i*0.6, 0.0 - s*0.06, 0.05, 0.5, payloads[s].policies[i].qvalues);
    }
};
#endif // PAYLOAD_H_INCLUDED
