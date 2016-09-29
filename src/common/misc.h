#ifndef MISC_H_INCLUDED
#define MISC_H_INCLUDED

#include <iostream>
#include <vector>
#include <string>

std::string get_time_from_cycle_counter(unsigned long long cycles);

std::string to_str(const std::vector<double>& vect);

template <typename vector_t> void print(const vector_t& content);

#endif // MISC_H_INCLUDED
