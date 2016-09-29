#ifndef GENERATION_BASED_STRATEGY_H_INCLUDED
#define GENERATION_BASED_STRATEGY_H_INCLUDED

#include <evolution/evolution_strategy.h>
#include <common/stopwatch.h>
#include <common/modules.h>
#include <common/log_messages.h>

/** TODO:
 *  create object time statistics */

class Generation_Based_Evolution: public Evolution_Strategy
{
public:
    Generation_Based_Evolution(Population& population,
                               Evaluation_Interface& evaluation,
                               config& configuration,
                               std::size_t max_generation,
                               std::size_t cur_generation,
                               const std::size_t selection_size,
                               const std::string& project_folder_path,
                               const bool verbose = true)
    : Evolution_Strategy(population, evaluation, configuration, project_folder_path, verbose)
    , max_generation(max_generation)
    , cur_generation(cur_generation)
    , selection_size(selection_size)
    , verbose(verbose)
    {
        assert(max_generation > 0);
        assert(max_generation >= cur_generation);
        assert(selection_size < population.get_size());
        assert(selection_size > 1);

        sts_msg("created generation-based policy.");
        sts_msg("selection size is %u.", selection_size);
    }

    ~Generation_Based_Evolution() { sts_msg("destroyed generation-based policy."); }

    bool evaluate_generation(void);

    void selection(void) {
        if (verbose) sts_msg("Selecting.");
        population.sort_by_fitness();
    }

    void recombination_crossover(void);
    void mutation(void);
    bool show_selection(void);

    Evolution_State playback(void) { return show_selection()? Evolution_State::playback : Evolution_State::stopped; }

    void resume(void)
    {
        if (cur_generation > 0) {
            std::size_t max_gen_old = max_generation;
            max_generation += (cur_generation - (cur_generation % max_generation));
            if (max_generation > max_gen_old)
                wrn_msg("max. generation increased from %u to %u.", max_gen_old, max_generation);

            load_state();
            recombination_crossover();
            mutation();
            sts_msg("Generation-based strategy is ready to resume.");
        } else
            wrn_msg("Nothing to resume. Skip.");
    }

    void save_config(config& configuration)
    {
        sts_msg("Saving generation-based strategy settings.");
        configuration.writeUINT("MAX_GENERATIONS"   , max_generation);
        configuration.writeUINT("CURRENT_GENERATION", cur_generation);
        configuration.writeUINT("SELECTION_SIZE"    , selection_size);
    }

    std::size_t get_max_generation(void) const { return max_generation; }
    std::size_t get_cur_generation(void) const { return cur_generation; }
    std::size_t get_selection_size(void) const { return selection_size; }

    std::size_t get_max_trials    (void) const { return max_generation * population.get_size(); }
    std::size_t get_current_trial (void) const { return cur_generation * population.get_size(); }

    Evolution_State execute_trial(void)
    {
        if (!evaluate_generation()) {
            return Evolution_State::aborted;
        }
        selection();
        save_state();

        ++cur_generation;
        if (cur_generation >= max_generation) {
            return Evolution_State::finished;
        } // else
        recombination_crossover();
        mutation();
        return Evolution_State::running;
    }

    const Individual& get_best_individual(void) const { return population[0]; }
    bool   is_there_a_new_best_individual(void)       { return true; /* TODO implement */}

private:
    std::size_t max_generation;
    std::size_t cur_generation;
    const std::size_t selection_size;
    const bool verbose;
};

#endif // GENERATION_BASED_STRATEGY_H_INCLUDED
