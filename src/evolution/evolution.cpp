#include "evolution.h"

/* constructor for a new evolution */
Evolution::Evolution(Evaluation_Interface &evaluation, const Setting& settings, const std::vector<double>& seed_genome)
: evaluation(evaluation)
, projectname(create_project_name_and_folder(settings.project_name))
, conffilename(FOLDER_PREFIX + projectname + "/evolution.conf")
, configuration(settings.save_to_projectfile(conffilename))
, state(Evolution_State::stopped)
, seed(seed_genome)
, population(settings.population_size, seed.size(), settings.init_mutation_rate, settings.meta_mutation_rate)
, strategy()
, evolution_log(FOLDER_PREFIX + projectname + "/evolution.log")
, bestindiv_log(FOLDER_PREFIX + projectname + "/bestindiv.log")
, verbose(settings.visuals)
, playback_only(false)
{
    common_setup();

    sts_msg("Setting up a new evolution project: %s", projectname.c_str());

    if (seed.empty()) err_msg(__FILE__, __LINE__, "Seed genome is missing.");

    /*TODO move to initializer list? */
    if (settings.strategy == "GENERATION")
        strategy = Strategy_Pointer(new Generation_Based_Evolution( population,
                                                                    evaluation,
                                                                    configuration,
                                                                    settings.max_generations,
                                                                    0,
                                                                    settings.selection_size,
                                                                    FOLDER_PREFIX + projectname,
                                                                    settings.visuals ));
    else if (settings.strategy == "POOL")
        strategy = Strategy_Pointer(new Pool_Evolution( population,
                                                        evaluation,
                                                        configuration,
                                                        settings.max_trials,
                                                        0,
                                                        settings.moving_rate,
                                                        settings.selection_bias,
                                                        FOLDER_PREFIX + projectname));
    else
        err_msg(__FILE__, __LINE__, "Unknown type of evolution strategy.");

    write_config();


    assert(strategy != nullptr);
    assert(not settings.fitness_function.empty());

    dbg_msg("Init population filename: %s", settings.initial_population.c_str());
    if (not settings.initial_population.empty())
        strategy->load_start_population(settings.initial_population);
    else
        strategy->generate_start_population(seed);

    state = Evolution_State::running;
    sts_msg("Ready to create.");

}

/* constructor for resuming previous evolution */
Evolution::Evolution(Evaluation_Interface &evaluation, const Setting& settings, bool playback_only)
: evaluation(evaluation)
, projectname(settings.project_name)
, conffilename(FOLDER_PREFIX + projectname + "/evolution.conf")
, configuration(conffilename)
, state(Evolution_State::stopped)
, seed()
, population(settings.population_size,
             configuration.readUINT("INDIVIDUAL_SIZE"),
             settings.init_mutation_rate,
             settings.meta_mutation_rate)
, strategy()
, evolution_log(FOLDER_PREFIX + projectname + "/evolution.log", true)
, bestindiv_log(FOLDER_PREFIX + projectname + "/bestindiv.log", true)
, verbose(settings.visuals)
, playback_only(playback_only)
{
    common_setup();

    if (settings.strategy == "GENERATION")
        strategy = Strategy_Pointer(new Generation_Based_Evolution( population,
                                                                    evaluation,
                                                                    configuration,
                                                                    settings.max_generations,
                                                                    configuration.readUINT("CURRENT_GENERATION"),
                                                                    settings.selection_size,
                                                                    FOLDER_PREFIX + projectname,
                                                                    settings.visuals ));
    else if (settings.strategy == "POOL")
        strategy = Strategy_Pointer(new Pool_Evolution( population,
                                                        evaluation,
                                                        configuration,
                                                        settings.max_trials,
                                                        configuration.readUINT("CURRENT_TRIAL"),
                                                        configuration.readDBL ("MOVING_RATE"),
                                                        configuration.readDBL ("SELECTION_BIAS"),
                                                        FOLDER_PREFIX + projectname));
    else
        err_msg(__FILE__, __LINE__, "Unknown type of evolution strategy.");

    assert(strategy != nullptr);
    assert(not settings.fitness_function.empty());

    switch (configuration.readINT("STATUS")) {
        case 1: sts_msg("Former evolution has not finished."); break;
        case 2: sts_msg("Former evolution was finished.    "); break;
        default:
            wrn_msg("Project has invalid status. Abort.");
            state = Evolution_State::stopped;
            return;
    }

    sts_msg("Resume existing evolution project: %s", projectname.c_str());
    strategy->resume();

    if (not playback_only) {
        state = Evolution_State::running;
        sts_msg("Ready for resuming.");
    } else {
        state = Evolution_State::playback;
        sts_msg("Ready for playback.");
    }
}


void
Evolution::common_setup(void)
{
    /* since evolution uses mutation,
     * we have to be for sure that random values
     * are initialized properly */
    sts_msg("Initializing random number generator.");
    srand((unsigned) time(NULL));
}

void
Evolution::write_config()
{
    /* test if the project already exists */
    if (configuration.readUINT("STATUS") > 0)
        err_msg(__FILE__, __LINE__, "Project already exists. Please use '--resume' instead. Exit.");
    else
        sts_msg("Safely override existing configuration file.");

    configuration.writeUINT("STATUS"         , 0); // evolution has not started yet
    configuration.writeUINT("INDIVIDUAL_SIZE", population.get_individual_size());

    assert(strategy != nullptr);
    strategy->save_config(configuration);

    configuration.finish();
    dbg_msg("Done writing configuration file.");
}

bool Evolution::loop(void)
{
    switch(state)
    {
        case running:
            state = strategy->execute_trial();
            save_best_individual();
            save_statistics();
            /** TODO update GUI status */
            break;

        case finished:
            sts_msg("Finished.");
            configuration.load();
            configuration.writeINT("STATUS", 2);
            strategy->save_config(configuration);
            configuration.finish();
            state = Evolution_State::stopped;
            break;

        case playback:
            state = strategy->playback();
            break;

        case aborted: /* by user */
            wrn_msg("Aborted.");
            prepare_quit();
            break;

        case stopped:
            sts_msg("Stopped.");
            return false;

        default:
            err_msg(__FILE__, __LINE__, "Invalid Evolution status.");
            break;
    }

    return true;
}

void
Evolution::prepare_quit(void)
{
    if (not playback_only) {
        sts_msg("Saving data.");
        configuration.load();
        configuration.writeINT("STATUS", 1);
        strategy->save_config(configuration);
        configuration.finish();
    }
    sts_msg("Sending evolution stop signal.");
    state = Evolution_State::stopped;
}

/* abort evolution externally before finished */
void
Evolution::finish(void)
{
    sts_msg("Closing.");
    dbg_msg("Current state is: %d", state);

    if ((Evolution_State::running == state) || (Evolution_State::aborted == state))
        prepare_quit();
    else if (Evolution_State::playback == state)
        state = Evolution_State::stopped;

}

/* save best individual, write/append to file */
void Evolution::save_best_individual(void)
{
    if (not (Evolution_State::running == state or Evolution_State::finished == state)) {
        wrn_msg("Evolution status is NOT running or finished. Saving best individual aborted.");
        return;
    }

    if (strategy->is_there_a_new_best_individual()) {
        const Individual& best = strategy->get_best_individual();
        if (verbose) sts_msg("Saving best individual.");
        bestindiv_log.append(best.genome);
        bestindiv_log.flush(); // flush, to save data in case of error
    } //else
        //dbg_msg("Saving best individual skipped.");
}

void Evolution::save_statistics(void)
{
    assert(strategy != nullptr);
    statistics_t const& fstats = strategy->get_fitness_statistics();
    statistics_t const& mstats = strategy->get_mutation_statistics();
    evolution_log.append( "%+1.8e %+1.8e %+1.8e %+1.8e %+1.8e %+1.8e"
                         , fstats.max, fstats.avg, fstats.min
                         , mstats.max, mstats.avg, mstats.min );
    evolution_log.flush();
}

std::string
create_project_name_and_folder(std::string name)
{
    if (name.empty()) // no name given yet? create project name as time stamp
    {
        time_t t0 = time(NULL);                 // initialize time
        struct tm * timeinfo = localtime(&t0);  // get time info
        char timestamp[256];
        snprintf(
            timestamp, 256, "%02d%02d%02d%02d%02d%02d",
            timeinfo->tm_year-100, timeinfo->tm_mon + 1,
            timeinfo->tm_mday, timeinfo->tm_hour,
            timeinfo->tm_min, timeinfo->tm_sec
            );
        name = timestamp;
        wrn_msg("Create project's name from current time stamp.");
    }
    sts_msg("Project name is '%s'", name.c_str());
    make_directory("%s%s", FOLDER_PREFIX, name.c_str());
    return name;
}

