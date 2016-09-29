#include <evolution/pool_strategy.h>

/** TODO think about to include a check of number of evaluations? */
std::size_t get_replacement_candidate_for(Individual& opponent, Population& population)
{
    if (opponent.fitness < population.get_last_individual().fitness)
        return population.get_size() - 1;

    std::size_t i = population.get_size();
    std::size_t candidate_idx = i-1;

    while (i > 0) {
        --i;
        if (opponent.fitness > population[i].fitness)
            candidate_idx = i;
        else break;
    }
    //dbg_msg("Replacement candidate is %u", candidate_idx);
    assert(candidate_idx < population.get_size());
    return candidate_idx;
}
