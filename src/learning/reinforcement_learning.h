#ifndef REINFORCEMENT_LEARNING_H_INCLUDED
#define REINFORCEMENT_LEARNING_H_INCLUDED

namespace learning {

class RL_Interface {
public:
    virtual ~RL_Interface() {}
    virtual std::size_t get_current_state (void) const = 0;
    virtual std::size_t get_current_action(void) const = 0;
    virtual std::size_t get_current_policy(void) const = 0;
    virtual bool        is_exploring      (void) const = 0;
    virtual bool positive_current_delta(std::size_t policy) const = 0;
};

class Non_Learning : public RL_Interface {
public:
    Non_Learning(void) {}
    std::size_t get_current_state (void) const { return 0; }
    std::size_t get_current_action(void) const { return 0; }
    std::size_t get_current_policy(void) const { return 0; }
    bool        is_exploring      (void) const { return false; }
    bool positive_current_delta(std::size_t /*policy*/) const { return false; }
};

} // namespace learning

#endif // REINFORCEMENT_LEARNING_H_INCLUDED
