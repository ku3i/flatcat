#ifndef POPULATION_H_INCLUDED
#define POPULATION_H_INCLUDED

#include <vector>
#include <algorithm>
#include <common/file_io.h>
#include <common/log_messages.h>
#include <evolution/individual.h>

class Population
{
    std::vector<Individual> individuals;

public:
    Population(std::size_t population_size, std::size_t individual_size, double init_mutation_rate, double meta_mutation_rate)
    : individuals(population_size, Individual(individual_size, init_mutation_rate, meta_mutation_rate))
    {
        assert(population_size > 0);
        sts_msg("[Population:] Initializing population with %u individuals of size %u", population_size, individual_size);
    }

    ~Population() { dbg_msg("[Population:] Destroying population with %u individuals.", get_size()); }

    void initialize_from_seed(const std::vector<double>& seed);
    void sort_by_fitness(void);

    const Individual& get_best_individual(void) const { return individuals.front(); }
    const Individual& get_last_individual(void) const { return individuals.back (); }

    std::size_t get_size           (void) const { return individuals.size();           }
    std::size_t get_individual_size(void) const { return individuals[0].genome.size(); }

          Individual& operator[] (std::size_t index)       { assert(index < individuals.size()); return individuals[index]; }
    const Individual& operator[] (std::size_t index) const { assert(index < individuals.size()); return individuals[index]; }

    friend void save_population    (Population& population, file_io::CSV_File<double>& csv_population);
    friend void load_population    (Population& population, file_io::CSV_File<double>& csv_population);
    friend void save_mutation_rates(Population& population, file_io::CSV_File<double>& csv_mutation);
    friend void load_mutation_rates(Population& population, file_io::CSV_File<double>& csv_mutation);
    friend void save_fitness_values(Population& population, file_io::CSV_File<double>& csv_fitness);
    friend void load_fitness_values(Population& population, file_io::CSV_File<double>& csv_fitness);
    friend void print_population   (Population& population);

};

#endif // POPULATION_H_INCLUDED
