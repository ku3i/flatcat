#ifndef GLOBALFLAG_H
#define GLOBALFLAG_H
#include <common/lock.h>

class GlobalFlag
{
    GlobalFlag( const GlobalFlag& other ) = delete;      // non construction-copyable
    GlobalFlag& operator=( const GlobalFlag& ) = delete; // non copyable

    common::mutex_t mtx;
    bool flag;

public:
    GlobalFlag()          : mtx(), flag(false) {}
    GlobalFlag(bool flag) : mtx(), flag(flag)  {}

    void toggle(void) {
        common::lock_t lock(mtx);
        flag = !flag;
    }
    void enable(void) {
        if (!flag) {
            common::lock_t lock(mtx);
            flag = true;
        }
    }
    void disable(void) {
        if (flag) {
            common::lock_t lock(mtx);
            flag = false;
        }
    }
    bool status(void) const
    {
        return flag;
    }
};

#endif // GLOBALFLAG

