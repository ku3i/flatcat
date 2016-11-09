#ifndef INCREMENTAL_AVERAGE_H
#define INCREMENTAL_AVERAGE_H

class incremental_average {
public:

    explicit incremental_average(void) : mean{.0}, num_samples{0} {}

    void sample(double value) {
        ++num_samples;
        mean = mean + (value - mean) / num_samples;
    }

    double get(void) const { return mean; }
    std::size_t get_num_samples(void) const { return num_samples; }

    void reset(void) {
        mean = .0;
        num_samples = 0;
    }

private:
    double mean;
    std::size_t num_samples;
};


#endif // INCREMENTAL_AVERAGE_H


