#ifndef INTEGRATOR_H_INCLUDED
#define INTEGRATOR_H_INCLUDED

class Integrator {

    double      sum;
    std::size_t number;

    double      last;
    /* TODO incremental average */
public:
    Integrator() : sum(0.0), number(0), last(0.0) {}

    void add(double sample) {
        sum += sample;
        ++number;
    }

    double get_avg_value(void) const {
        if (number > 0) return sum/number;
        else return .0;
    }

    double get_avg_value_and_reset(void) {
        double result = get_avg_value();
        reset();
        last = result;
        return result;
    }

    std::size_t get_number_of_samples() const { return number; }

    double get_last(void) const { return last; }

    void reset(void) {
        sum = 0.0;
        number = 0;
    }
};

#endif // INTEGRATOR_H_INCLUDED
