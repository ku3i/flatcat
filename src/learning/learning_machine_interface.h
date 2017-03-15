#ifndef LEARNING_MACHINE_INTERFACE_HPP_INCLUDED
#define LEARNING_MACHINE_INTERFACE_HPP_INCLUDED

namespace learning {

class Learning_Machine_Interface {
public:
    virtual double get_learning_progress(void) const = 0;
    virtual void enable_learning(bool b) = 0;
    virtual ~Learning_Machine_Interface() {}
};

}

#endif // LEARNING_MACHINE_INTERFACE_HPP_INCLUDED
