#ifndef EVALUATION_INTERFACE_H_INCLUDED
#define EVALUATION_INTERFACE_H_INCLUDED

#include <vector>

class Evaluation_Interface
{
public:
    Evaluation_Interface() {}
    virtual ~Evaluation_Interface() {}
    virtual bool evaluate(Fitness_Value &fitness, const std::vector<double>& genome, double rand_value) = 0;
    virtual void prepare(void) = 0;
};

#endif // EVALUATION_INTERFACE_H_INCLUDED
