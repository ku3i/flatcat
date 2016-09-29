#ifndef SENSORSPACE_H_INCLUDED
#define SENSORSPACE_H_INCLUDED

#include <vector>
#include <cassert>
#include <common/vector_n.h>

/**TODO move to control or robots */

class sensor_signal
{
public:
    sensor_signal(const std::string& name, std::function<double()> lambda)
    : name(name)
    , lambda(lambda)
    , current(.0)
    , last(current)
    {}

    void execute_cycle(void) {
        last = current;
        current = lambda();
    }

    const std::string       name;
    std::function<double()> lambda;

    const double operator() (void) const { return current; }

    double current;
    double last;
};

class sensor_vector
{
protected:
    std::vector<sensor_signal> sensors;

public:
    sensor_vector(std::size_t number_of_elements = 0)
    : sensors()
    {
        //dbg_msg("Creating sensor vector, reserving space for %u elements.", number_of_elements);
        sensors.reserve(number_of_elements);
    }

    /* embed 'normal' vector */
    explicit sensor_vector(const VectorN& plain)
    : sensor_vector(plain.size())
    {
        for (std::size_t i = 0; i < plain.size(); ++i)
            sensors.emplace_back(std::to_string(i) , [&plain, i](){ return plain[i]; });
    }

    virtual ~sensor_vector() { /*dbg_msg("Destroying sensor vector base.");*/ };

    void execute_cycle(void) {
        for (std::size_t i = 0; i < sensors.size(); ++i)
            sensors[i].execute_cycle();
    }

    const std::size_t size(void) const { return sensors.size(); }

          double operator[] (std::size_t index)       { assert(index < sensors.size()); return sensors[index](); }
    const double operator[] (std::size_t index) const { assert(index < sensors.size()); return sensors[index](); }

    VectorN get(void) const {
        VectorN result(sensors.size());
        for (std::size_t i = 0; i < sensors.size(); ++i)
            result[i] = sensors[i]();
        return result;
    }
};

#endif // SENSORSPACE_H_INCLUDED
