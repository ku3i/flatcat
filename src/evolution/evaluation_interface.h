#ifndef EVALUATION_INTERFACE_H_INCLUDED
#define EVALUATION_INTERFACE_H_INCLUDED

#include <vector>

class Evaluation_Interface
{
public:
    virtual ~Evaluation_Interface() = default;
    virtual bool evaluate(Fitness_Value &fitness, const std::vector<double>& genome, double rand_value) = 0;
    virtual void prepare_generation(unsigned cur_generation, unsigned max_generation) = 0;
};

#endif // EVALUATION_INTERFACE_H_INCLUDED
