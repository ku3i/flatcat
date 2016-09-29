#ifndef ACTION_MODULE_H_INCLUDED
#define ACTION_MODULE_H_INCLUDED

class Action_Module_Interface
{
public:
    virtual std::size_t get_number_of_actions(void) const = 0;
    virtual std::size_t get_number_of_actions_available(void) const = 0;
    virtual bool exists(const std::size_t action_index) const = 0;
    virtual ~Action_Module_Interface() {}
};


#endif // ACTION_MODULE_H_INCLUDED
