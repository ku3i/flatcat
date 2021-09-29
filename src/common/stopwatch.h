#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <sys/time.h>
#include <unistd.h>


class Stopwatch
{
public:
    Stopwatch()
    : last_time_us(0)
    , current_time_us(0)
    , start_time_us(0)
    , timestamp()
    {
        reset();
    }

    void reset(void)
    {
        get_time();
        start_time_us = current_time_us;
    }

    unsigned long long get_time_passed_us(void)
    {
        last_time_us = current_time_us; // save last time stamp
        get_time();
        return current_time_us - last_time_us;
    }

    unsigned long long get_current_time_ms(void) {
        get_time();
        return current_time_us/1000;
    }

private:
    void get_time()
    {
        gettimeofday(&timestamp, NULL);
        current_time_us = timestamp.tv_sec*1000*1000 + timestamp.tv_usec;
    }
    unsigned long long last_time_us; // last time
    unsigned long long current_time_us; // current time
    unsigned long long start_time_us;
    struct timeval timestamp;
};

/*
unsigned int get_hours(unsigned long long time_ms)
{
    return time_ms / (1000*3600);
}

unsigned int get_minutes(unsigned long long time_ms)
{
    return time_ms % (1000*3600);
}
*/

#endif // STOPWATCH_H
