#ifndef EVOLUTION_H
#define EVOLUTION_H

#include <vector>
#include <algorithm>
#include <float.h>
#include <math.h>
#include <tr1/memory>

#include <common/config.h>
#include <common/stopwatch.h>
#include <common/modules.h>
#include <common/log_messages.h>
#include <common/file_io.h>

#include <robots/simloid.h>

#include <evolution/individual.h>
#include <evolution/population.h>
#include <evolution/evaluation_interface.h>
#include <evolution/evolution_strategy.h>
#include <evolution/generation_based_strategy.h>
#include <evolution/pool_strategy.h>
#include <evolution/setting.h>

typedef std::tr1::shared_ptr<Evolution_Strategy> Strategy_Pointer;

class Evolution
{
    Evolution( const Evolution& other ) = delete;      // non construction-copyable
    Evolution& operator=( const Evolution& ) = delete; // non copyable

public:
    Evolution(Evaluation_Interface &evaluation, const Setting& settings, const std::vector<double>& seed_genome); // new
    Evolution(Evaluation_Interface &evaluation, const Setting& settings, bool playback_only); // resume or watch

    ~Evolution() { sts_msg("Evolution shut down."); }

    bool loop(void);
    void finish(void);

    double get_last_max_fitness(void) const { return strategy->get_max_fitness(); }
    double get_last_avg_fitness(void) const { return strategy->get_avg_fitness(); }
    double get_last_min_fitness(void) const { return strategy->get_min_fitness(); }

    std::size_t get_number_of_trials(void) const { return strategy->get_max_trials();    }
    std::size_t get_current_trial   (void) const { return strategy->get_current_trial(); }
    std::size_t get_population_size (void) const { return population.get_size();         }

private:
    Evaluation_Interface& evaluation;
    const std::string     projectname;
    const std::string     conffilename;
    config                configuration;

    Evolution_State       state;

    std::vector<double>   seed;
    Population            population;
    Strategy_Pointer      strategy;

    file_io::Logfile      evolution_log;
    file_io::Logfile      bestindiv_log;

    const bool            verbose;

    void common_setup(void);
    void save_best_individual(void);
    void save_statistics(void);
    void write_config(void);

    void prepare_quit();
};

std::string create_project_name_and_folder(std::string name);

#endif // EVOLUTION_H
