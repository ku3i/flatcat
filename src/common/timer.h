/*

 +---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | November 2018                   |
 +---------------------------------+

*/

#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

class SimpleTimer {
    uint64_t timeout_us;
    std::chrono::high_resolution_clock::time_point time_0; //what type?
    bool enabled;

public:
    SimpleTimer(uint64_t timeout_us, bool enabled = false)
    : timeout_us(timeout_us)
    , time_0(std::chrono::high_resolution_clock::now())
    , enabled(enabled)
    {}

    void start(void) { enabled = true; }
    void stop(void) { enabled = false; }

    bool check_if_timed_out_and_restart(uint64_t new_timeout_us) {
        timeout_us = new_timeout_us;
        return check_if_timed_out_and_restart();
    }

    bool check_if_timed_out_and_restart(void) {
        if (not enabled) return false;
        const auto time_1 = std::chrono::high_resolution_clock::now();
        uint64_t elapsed_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(time_1 - time_0).count());
        bool is_timed_out = elapsed_us >= timeout_us;
        if (is_timed_out) {
            time_0 = time_1; // reset
            return true;
        } else return false;
    }

};

#endif /* TIMER_HPP */
