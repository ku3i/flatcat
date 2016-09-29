#include "population.h"


void
Population::initialize_from_seed(const std::vector<double>& seed)
{
    for (std::size_t i = 0; i < individuals.size(); ++i)
        individuals[i].initialize_from_seed(seed);
}

bool greater_than(const Individual& ind1, const Individual& ind2) { return (ind1.fitness > ind2.fitness); }

void
Population::sort_by_fitness(void)
{
    std::sort(individuals.begin(), individuals.end(), greater_than);
}

void
save_population(Population& population, file_io::CSV_File<double>& csv_population)
{
    //sts_msg("Saving population.");
    for (std::size_t idx = 0; idx < population.individuals.size(); ++idx)
        csv_population.set_line(idx, population.individuals[idx].genome);
    csv_population.write();
}

void
load_population(Population& population, file_io::CSV_File<double>& csv_population)
{
    if (csv_population.read())
    {
        sts_msg("Loading population.");
        for (std::size_t idx = 0; idx < population.individuals.size(); ++idx)
            csv_population.get_line(idx, population.individuals[idx].genome);
    }
    else wrn_msg("Could not load population.");
}

void
save_mutation_rates(Population& population, file_io::CSV_File<double>& csv_mutation)
{
    //sts_msg("Saving mutation rates.");
    for (std::size_t idx = 0; idx < population.individuals.size(); ++idx)
        csv_mutation.set_line(idx, population.individuals[idx].mutation_rate);
    csv_mutation.write();
}

void
load_mutation_rates(Population& population, file_io::CSV_File<double>& csv_mutation)
{
    if (csv_mutation.read()) {
        sts_msg("Loading mutation rates.");
        for (std::size_t idx = 0; idx < population.individuals.size(); ++idx)
            csv_mutation.get_line(idx, population.individuals[idx].mutation_rate);
    }
    else wrn_msg("Could not load mutation rates.");
}

void
save_fitness_values(Population& population, file_io::CSV_File<double>& csv_fitness)
{
    //sts_msg("Saving fitness values.");
    for (std::size_t idx = 0; idx < population.individuals.size(); ++idx)
        csv_fitness.set_line(idx, population.individuals[idx].fitness.get_value());
    csv_fitness.write();
}

void
load_fitness_values(Population& population, file_io::CSV_File<double>& csv_fitness)
{
    if (csv_fitness.read()) {
        sts_msg("Loading fitness values.");
        for (std::size_t idx = 0; idx < population.individuals.size(); ++idx)
        {
            double fitness = -DBL_MIN;
            csv_fitness.get_line(idx, fitness);
            population.individuals[idx].fitness.set_value(fitness);
        }
    }
    else wrn_msg("Could not load fitness values.");
}

void
print_population(Population& population)
{
    sts_msg("Print population:");
    for (std::size_t i = 0; i < population.individuals.size(); ++i) {
        sts_msg("%2u. %1.2f", i, population.individuals[i].fitness);
        for (std::size_t k = 0; k < population.individuals[i].genome.size(); ++k)
            sts_msg("genome %u %1.3f", k, population.individuals[i].genome[k]);
    }
}
