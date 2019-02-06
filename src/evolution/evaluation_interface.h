#ifndef EVALUATION_INTERFACE_H_INCLUDED
#define EVALUATION_INTERFACE_H_INCLUDED

#include <vector>

class Evaluation_Interface
{
public:
    typedef std::vector<double> genome_t;

    virtual ~Evaluation_Interface() = default;
    virtual bool evaluate(Fitness_Value &fitness, const genome_t& genome, double rand_value) = 0;
    virtual void prepare_generation(unsigned cur_generation, unsigned max_generation) = 0;
    virtual void prepare_evaluation(unsigned cur_trial, unsigned max_trial) = 0;

    virtual void constrain(genome_t& /*genome*/) { /* implement optionally */ };
};

#endif // EVALUATION_INTERFACE_H_INCLUDED
