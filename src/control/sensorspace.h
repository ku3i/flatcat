#ifndef SENSORSPACE_H_INCLUDED
#define SENSORSPACE_H_INCLUDED

#include <vector>
#include <string>
#include <queue>
#include <cassert>
#include <functional>
#include <common/vector_n.h>

/**TODO namespace */

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

    std::string             name;
    std::function<double()> lambda;

    double operator() (void) const { return current; }

    double current;
    double last;
};


class sensor_input_interface {
public:
    virtual std::size_t size(void) const = 0;
    virtual VectorN get(void) const = 0;
    virtual ~sensor_input_interface() {}
    virtual double operator[] (std::size_t index) const = 0;
};


class sensor_vector : public sensor_input_interface
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
        for (auto& s : sensors) s.execute_cycle();
    }

    std::size_t size(void) const { return sensors.size(); }

    double operator[] (std::size_t index)       { return sensors.at(index)(); }
    double operator[] (std::size_t index) const { return sensors.at(index)(); }

    VectorN get(void) const {
        VectorN result(sensors.size());
        for (std::size_t i = 0; i < sensors.size(); ++i)
            result[i] = sensors[i]();
        return result;
    }
};

template <std::size_t NumTaps>
class time_embedded_signal
{
public:
    time_embedded_signal(const std::string& name, std::function<double()> lambda)
    : name(name)
    , lambda(lambda)
    , buffer()
    {
        static_assert(NumTaps > 0, "Min. length of tapped delay line is 1.");
        buffer.assign(NumTaps, .0);
        assert(buffer.size() == NumTaps);
    }

    void execute_cycle(void) {
        buffer.pop_front();
        buffer.push_back(lambda());
    }

    std::string             name;
    std::function<double()> lambda;

    double operator() (void) const { return buffer.front(); }

    double operator[] (std::size_t index)       { return buffer.at(index); }
    double operator[] (std::size_t index) const { return buffer.at(index); }

    std::deque<double> buffer;
};

template <std::size_t NumTaps>
class time_embedded_sensors : public sensor_input_interface
{
protected:
    std::vector<time_embedded_signal<NumTaps>> sensors;

public:
    time_embedded_sensors(std::size_t number_of_elements)
    : sensors()
    {
        //dbg_msg("Creating time-embedded sensors, reserving space for %u delay lines of size %u.", number_of_elements, NumTaps);
        sensors.reserve(number_of_elements);
    }

    virtual ~time_embedded_sensors() {};

    void execute_cycle(void) {
        for (auto& s : sensors) s.execute_cycle(); // propagate delay lines
    }

    std::size_t size(void) const { return sensors.size()*NumTaps + 1 /*bias*/; }


    double operator[] (std::size_t index) const {
        assert(index < size());
        const std::size_t i = index/NumTaps; // get buffer index
        const std::size_t j = index%NumTaps; // get element index
        if (i >= sensors.size()) return 0.1;
        return sensors[i][j];
    }

    VectorN get(void) const {
        VectorN result(size());
        for (std::size_t i = 0; i < size(); ++i)
            result[i] = (*this)[i];
        return result;
    }

};

#endif // SENSORSPACE_H_INCLUDED
