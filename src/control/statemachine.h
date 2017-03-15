#ifndef STATEMACHINE_H_INCLUDED
#define STATEMACHINE_H_INCLUDED

namespace control {

class Statemachine_Interface {
public:
    virtual bool has_state_changed(void) const = 0;
    virtual std::size_t get_state(void) const = 0;
    virtual ~Statemachine_Interface() {}
};


} // namespace control

#endif // STATEMACHINE_H_INCLUDED
