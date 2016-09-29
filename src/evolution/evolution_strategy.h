#ifndef EVOLUTION_POLICY_H_INCLUDED
#define EVOLUTION_POLICY_H_INCLUDED

#include <common/config.h>
#include <evolution/population.h>
#include <evolution/evaluation_interface.h>


enum Evolution_State
{
    stopped = 0,
    running,
    aborted,
    finished,
    playback
};

/* base class */
class Evolution_Strategy
{
public:
    Evolution_Strategy(Population& population, Evaluation_Interface& evaluation, config& configuration, const std::string& project_folder_path, const bool verbose)
    : population(population)
    , evaluation(evaluation)
    , configuration(configuration)
    , csv_population(project_folder_path + "/population.log", population.get_size(), population.get_individual_size())
    , csv_mutation  (project_folder_path + "/mutation.log"  , population.get_size(), 1)
    , csv_fitness   (project_folder_path + "/fitness.log"   , population.get_size(), 1)
    , last_max_fitness(.0)
    , last_avg_fitness(.0)
    , last_min_fitness(.0)
    , verbose(verbose)
    {
        dbg_msg("created evolution policy (base).");
    }

    virtual ~Evolution_Strategy() { };

    virtual Evolution_State execute_trial(void) = 0;
    virtual Evolution_State playback(void) = 0;
    virtual void resume(void) = 0;

    virtual void save_config(config& configuration) = 0;

    double get_max_fitness(void) const { return last_max_fitness; }
    double get_avg_fitness(void) const { return last_avg_fitness; }
    double get_min_fitness(void) const { return last_min_fitness; }

    virtual std::size_t get_max_trials   (void) const = 0;
    virtual std::size_t get_current_trial(void) const = 0;

    virtual const Individual& get_best_individual(void) const = 0;
    virtual bool   is_there_a_new_best_individual(void)       = 0;

    void save_state(void) {
        if (verbose) sts_msg("Saving population state:");
        save_population    (population, csv_population);
        save_mutation_rates(population, csv_mutation);
        save_fitness_values(population, csv_fitness);
    }
    void load_state(void) {
        if (verbose) sts_msg("Loading population state:");
        load_population    (population, csv_population);
        load_mutation_rates(population, csv_mutation);
        load_fitness_values(population, csv_fitness);
    }

    void generate_start_population(const std::vector<double>& seed) {
        sts_msg("Generate start population.");
        population.initialize_from_seed(seed);
    }

    Population&           population;
    Evaluation_Interface& evaluation;
    config&               configuration;

    file_io::CSV_File<double> csv_population;
    file_io::CSV_File<double> csv_mutation;
    file_io::CSV_File<double> csv_fitness;

    double last_max_fitness;
    double last_avg_fitness;
    double last_min_fitness;

    const bool verbose;
};


#endif // EVOLUTION_POLICY_H_INCLUDED
