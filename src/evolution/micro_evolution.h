#ifndef MICRO_EVOLUTION_H_INCLUDED
#define MICRO_EVOLUTION_H_INCLUDED

//#include <evolution/fitness.h> //TODO check for circular dependency
#include <evolution/individual.h>
#include <evolution/population.h>
#include <evolution/pool_strategy.h>

/** 29.09.15: Elmar hat heute: "Papa, mehr Gurke bitte, ohne Schale" gesagt. */

class MicroEvolution {

    Population  population;
    Individual  child;     // temp individual for crossover operation

    std::size_t candidate_idx;

    double      moving_rate;
    double      selection_bias;

    enum Trial_t {
        trial_initial,
        trial_crossover,
        trial_refreshing
    };

    Trial_t trial;
    uint32_t trial_count;
    std::string name;

public:
    MicroEvolution( const std::string& name
                  , std::size_t population_size
                  , std::size_t individual_size
                  , double      init_mutation_rate
                  , double      meta_mutation_rate
                  , double      moving_rate
                  , double      selection_bias )
    : population(population_size, individual_size, init_mutation_rate, meta_mutation_rate)
    , child(population[0])
    , candidate_idx(0)
    , moving_rate(moving_rate)
    , selection_bias(selection_bias)
    , trial(trial_initial)
    , trial_count(0)
    , name(name)
    { /* dbg_msg("Creating micro evolution."); */ }

    ~MicroEvolution() { /* dbg_msg("Destroying micro evolution."); */ }

    void generate_start_population(const VectorN& seed) {
        //dbg_msg("Generate start population.");
        for (std::size_t i = 0; i < seed.size(); ++i)
            dbg_msg("[%u]: %+1.3f", i, seed[i]); // TODO remove

        population.initialize_from_seed(seed);
    }

    const VectorN& get_next_candidate_genome(void) {

        //sts_msg("%s Trial: %2u (%d)", name.c_str(), trial_count, trial);

        if (trial_count < population.get_size())
        {
            trial = trial_initial;
            candidate_idx = trial_count;
            return population[trial_count].genome;
        }
        else if (random_value(0.0, 1.0) > moving_rate) /**TODO: this is actually the inverse moving rate, rename!*/
        {
            /* select parents from pool and crossover */
            std::size_t parent_1 = biased_random_index_inv(population.get_size(), selection_bias);
            std::size_t parent_2 = biased_random_index_inv(population.get_size(), selection_bias);
            //sts_msg(" crossing %2u and %2u", parent_1, parent_2);

            trial = trial_crossover;
            candidate_idx = parent_1;
            crossover( population[parent_1]
                     , population[parent_2]
                     , child );

            child.mutate();
            return child.genome;
        }
        else
        {
            trial = trial_refreshing;
            candidate_idx = random_index(population.get_size());
            return population[candidate_idx].genome;
        }
    }

    /** no state machine */
    void set_candidate_fitness(const double fitness) {

        if (trial == trial_crossover) {
            child.fitness = fitness; // assign fitness to child in crossover trial
            std::size_t replace_idx = get_replacement_candidate_for(child, population);
            if (child.fitness > population[replace_idx].fitness) {
                //dbg_msg("Got replacement candidate %u (%1.3f > %1.3f)", replace_idx, child.fitness, population[replace_idx].fitness);
                population[replace_idx] = child;
            } //else dbg_msg("not replaced (%1.3f < %1.3f)", child.fitness, population[replace_idx].fitness);
        }
        else {
            population[candidate_idx].fitness.set_value(fitness); //averaging
            //dbg_msg("refreshing id:%u (%1.2f)", candidate_idx, population[candidate_idx].fitness.get_value());
        }
        ++trial_count;
        dbg_msg("fit(%1.2f)", population[candidate_idx].fitness.get_value());
    }

};

#endif // MICRO_EVOLUTION_H_INCLUDED
