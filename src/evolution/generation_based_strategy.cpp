#include "generation_based_strategy.h"

bool
Generation_Based_Evolution::show_selection(void) /**TODO show_sel and evaluate have many lines of code in common!*/
{
    double max_fitness = -DBL_MAX;
    double min_fitness = +DBL_MAX;
    double avg_fitness = 0.0;
    double generation_rnd = random_value(0.0, 1.0); // provide a random value used by evaluation, same for every individual of a generation,

    sts_msg("Showing selection");
    evaluation.prepare_generation(cur_generation, max_generation); // prepare generation
    for (std::size_t i = 0; i < selection_size; ++i)
    {
        sts_msg("  showing individual no. %3u/%u ", i + 1, selection_size);
        if (evaluation.evaluate(population[i].fitness, population[i].genome, generation_rnd))
        {
            sts_msg("  = %1.3f (% e)", population[i].fitness, population[i].fitness.get_value());
            avg_fitness += population[i].fitness.get_value();
            max_fitness = fmax(max_fitness, population[i].fitness.get_value());
            min_fitness = fmin(min_fitness, population[i].fitness.get_value());
        }
        else return false;
    }
    avg_fitness /= selection_size;
    sts_msg("Result: max: %1.2f min: %1.2f avg: %1.2f\n", max_fitness, min_fitness, avg_fitness);
    fitness_stats.max = max_fitness;
    fitness_stats.avg = avg_fitness;
    fitness_stats.min = min_fitness;
    return true;
}


bool
Generation_Based_Evolution::evaluate_generation(void)
{
    double max_fitness = -DBL_MAX;
    double min_fitness = +DBL_MAX;
    double avg_fitness = 0.0;
    double generation_rnd = random_value(0.0, 1.0); // provide a random value used by evaluation, same for every individual of a generation,

    if (verbose) sts_msg("Evaluate generation %u/%u:", cur_generation, max_generation);
    evaluation.prepare_generation(cur_generation, max_generation); // prepare generation

    for (std::size_t i = 0; i < population.get_size(); ++i)
    {
        if (verbose) sts_msg(" testing individual no.%3lu/%lu [%lu/%lu]", i + 1, population.get_size(), cur_generation, max_generation);
        else printf("\rtesting individual no.%3lu/%lu [%lu/%lu]", i + 1, population.get_size(), cur_generation, max_generation);

        population[i].fitness.reset();
        if (evaluation.evaluate(population[i].fitness, population[i].genome, generation_rnd))
        {
            if (verbose) sts_msg(" fit=%+07.3f  mu=%1.5f\n", population[i].fitness.get_value(), population[i].mutation_rate);
            max_fitness = fmax(max_fitness, population[i].fitness.get_value());
            min_fitness = fmin(min_fitness, population[i].fitness.get_value());
            avg_fitness += population[i].fitness.get_value();
        }
        else
        {
            sts_msg("Stopped in generation %u, max=%+07.3ff", cur_generation, max_fitness);
            return false;
        }
    }
    avg_fitness /= population.get_size();
    sts_msg("\rGeneration result: max=%+07.3f avg=%+07.3f min=%+07.3f", max_fitness, avg_fitness, min_fitness);

    fitness_stats.max = max_fitness;
    fitness_stats.avg = avg_fitness;
    fitness_stats.min = min_fitness;

    return true;
}

void
Generation_Based_Evolution::recombination_crossover(void)
{
    if (selection_size < 2) {
        wrn_msg("Recombination skipped, selection size is only 1.");
        return;
    }
    if (verbose) sts_msg("Doing recombination with crossover.");
    assert(selection_size > 1);
    for (std::size_t child_idx = selection_size; child_idx < population.get_size(); ++child_idx)
    {
        std::size_t mother_idx = random_index(selection_size);
        std::size_t father_idx = random_index(selection_size);
        while (mother_idx == father_idx)
            mother_idx = random_index(selection_size); // (!) this gets an infinite loop with selection_size = 1

        crossover(population[mother_idx],
                  population[father_idx],
                  population[ child_idx]);
    }
}

void
Generation_Based_Evolution::mutation(void)
{
    if (verbose) sts_msg("Mutating");
    for (std::size_t i = selection_size; i < population.get_size(); ++i)
        population[i].mutate();
}
