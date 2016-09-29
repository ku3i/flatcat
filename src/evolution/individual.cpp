#include "individual.h"

void
crossover(const Individual& mother, const Individual& father, Individual& child)
{
    assert(child.genome.size() == father.genome.size());
    assert(child.genome.size() == mother.genome.size());

    /* 50% chance to inherit each feature from either mother or father */
    for (unsigned int j = 0; j < child.genome.size(); j++)
        child.genome[j] = (random_value(0, 1.0) < 0.5) ? mother.genome[j] : father.genome[j];

    /* average the mutation rate */
    child.fitness.reset();
    child.mutation_rate      = .5 * mother.mutation_rate       + .5 * father.mutation_rate;
    child.meta_mutation_rate = .5 * mother.meta_mutation_rate  + .5 * father.meta_mutation_rate;
}

void
Individual::initialize_from_seed(const std::vector<double>& seed)
{
    assert(genome.size() == seed.size());
    double sigma = mutation_rate / sqrt(genome.size()); // normalize
    for (unsigned int j = 0; j < genome.size(); ++j)
        genome[j] = seed[j] + rand_norm_zero_mean(sigma);
    fitness.reset();
}

void
Individual::mutate(void)
{
    assert_in_range(mutation_rate     , 0.0001, 1.0  );
    assert_in_range(meta_mutation_rate, 0.0   , M_LN2);
    assert(genome.size() > 0);
    // TODO search for literature reference of that.
    const double rnd_val = random_value_norm(0.0, meta_mutation_rate, -M_LN2, M_LN2);
    const double factor = exp(rnd_val); // mutate the mutation rate
    assert_in_range(factor, 0.5, 2.0);

    mutation_rate *= factor;
    mutation_rate = clip(mutation_rate, 0.0001, 1.0);
//    sts_msg(" mutating: mu = %1.5f, d mu = %1.3f", mutation_rate, factor);
    const double sigma = mutation_rate / sqrt(genome.size()); // normalize

    for (unsigned int j = 0; j < genome.size(); ++j)
        genome[j] += rand_norm_zero_mean(sigma);
}
