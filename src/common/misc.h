#ifndef MISC_H_INCLUDED
#define MISC_H_INCLUDED

#include <iostream>
#include <vector>
#include <string>

std::string get_time_from_cycle_counter(unsigned long long cycles);

std::string to_str(const std::vector<double>& vect);

template <typename vector_t> void print(const vector_t& content);

namespace common {

template <typename T>
std::string to_string(std::vector<T> const& vect)
{
    if (vect.size() == 0) return "";

    std::string result;
    for (auto const& v: vect) result.append(std::to_string(v)+" ");
    return result;
}

} /* namespace common */


#endif // MISC_H_INCLUDED
