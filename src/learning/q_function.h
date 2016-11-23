#ifndef Q_FUNCTION_H_INCLUDED
#define Q_FUNCTION_H_INCLUDED

#include <cassert>
#include <common/modules.h>
#include <common/static_vector.h>
#include <common/log_messages.h>
#include <learning/action_module.h>

class Policy {

    const Action_Module_Interface& actions;

public:

    static_vector<double> qvalues;

    Policy(const Action_Module_Interface& actions, double initial)
    : actions(actions)
    , qvalues(actions.get_number_of_actions())
    {
        assert(actions.get_number_of_actions() >= 1);
        for (std::size_t i = 0; i < qvalues.size(); ++i)
            qvalues[i] = initial;
    }

    double get_max_q(void) const
    {
        double max_q = qvalues[0];
        for (std::size_t i = 1; i < qvalues.size(); ++i)
            if (actions.exists(i) and qvalues[i] > max_q)
                max_q = qvalues[i];
        return max_q;
    }
    std::size_t get_argmax_q(void) const
    {
        double max_q = qvalues[0];
        std::size_t argmax = 0;

        for (std::size_t i = 1; i < qvalues.size(); ++i)
            if (actions.exists(i) and qvalues[i] > max_q) {
                max_q = qvalues[i];
                argmax = i;
            }
        return argmax;
    }

    void copy_with_flaws(const Policy& other) {
        assert(qvalues.size() == other.qvalues.size());
        for (std::size_t a = 0; a < qvalues.size(); ++a)
            if (actions.exists(a))
                qvalues[a] = other.qvalues[a]
                           + random_value(-0.05 * other.qvalues[a],
                                          +0.05 * other.qvalues[a]); /** TODO: put into method and use gaussian noise */
    }

    void copy_q_value(std::size_t from_idx, std::size_t to_idx) {
        qvalues[to_idx] = qvalues[from_idx];
    }

    Policy& operator=(const Policy& other) {
        this->copy_with_flaws(other);
        return *this;
    }
};

#endif // Q_FUNCTION_H_INCLUDED
