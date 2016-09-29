#ifndef LOCK_H_INCLUDED
#define LOCK_H_INCLUDED

#include <mutex>

/** Self-unlocking LOCK */
namespace common {

typedef std::mutex               mutex_t;
typedef std::lock_guard<mutex_t> lock_t;

} // common

#endif // LOCK_H_INCLUDED
