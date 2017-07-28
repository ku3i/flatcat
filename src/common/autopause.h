/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | July 2017                       |
 +---------------------------------*/

#ifndef AUTOPAUSE_H_INCLUDED
#define AUTOPAUSE_H_INCLUDED

/**
 * AUTO PAUSE
 * Used to pause the application automatically after certain time steps (cycles).
 * Steps have to be provided as a list.
 */

class Auto_Pause {

    GlobalFlag&           do_pause;
    std::vector<uint64_t> times;
    std::size_t           ptr;

public:

    Auto_Pause(GlobalFlag& do_pause, std::vector<uint64_t> const& times)
    : do_pause(do_pause)
    , times(times)
    , ptr(0)
    {
        std::sort(this->times.begin(), this->times.end());
    }

    void execute_cycle(uint64_t current_cycles) {
        if (ptr < times.size() and current_cycles >= times[ptr]) {
            ++ptr;
            do_pause.enable();
            sts_msg("Automatically paused at step %llu", current_cycles);
        }
    }
};

#endif // AUTOPAUSE_H_INCLUDED
