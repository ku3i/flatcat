#ifndef INCREMENTAL_AVERAGE_H
#define INCREMENTAL_AVERAGE_H

class incremental_average {
public:

    incremental_average(double mean = 0.0, std::size_t num_samples = 0)
    : mean(mean)
    , num_samples(num_samples)
    {}

    void sample(double value) {
        ++num_samples;
        mean = mean + (value - mean) / num_samples;
    }

    void reset(double m = 0.0, std::size_t n = 0) {
        mean = m;
        num_samples = n;
    }

    double mean{.0};
    std::size_t num_samples{0};
};


#endif // INCREMENTAL_AVERAGE_H


