#ifndef SARSA_CONSTANTS_H_INCLUDED
#define SARSA_CONSTANTS_H_INCLUDED

namespace sarsa_constants {

    /* sarsa */
    const double EPSILON = 0.1;  // probability for non-greedy action
    const double GAMMA   = 0.99; // discount factor
    const double LAMBDA  = 0.9;  // eligibility trace factor (better do not touch)
    const double ALPHA   = 0.1;  // Reinforcement Learning Rate

    const unsigned int policy_change_cycle = 5000; // 10 sec. @ 100Hz
    const unsigned int number_of_policies = 3;
    const unsigned int number_of_actions = 3;
}

#endif // SARSA_CONSTANTS_H_INCLUDED
