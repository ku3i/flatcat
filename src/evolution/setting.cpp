
#include "./setting.h"

bool         read_option_bool  (int argc, char **argv, const std::string long_name, const std::string short_name, bool def = false);
unsigned int read_option_uint  (int argc, char **argv, const std::string long_name, const std::string short_name, unsigned int def);
std::string  read_option_string(int argc, char **argv, const std::string long_name, const std::string short_name);

Setting::Setting( int argc, char **argv )
                : project_name()
                , project_status(NEW)
                , visuals(not read_option_bool(argc, argv, "--blind", "-b"))
                , tcp_port(read_option_uint(argc, argv, "--port", "-p", DEFAULTPORT))
                , robot_ID(31)
                , scene_ID(0)
                , max_steps(1000)
                , max_power(100)
                , initial_steps(0)
                , efficient(true)
                , drop_penalty(true)
                , out_of_track_penalty(true)
                , stop_penalty(true)
                , symmetric_controller(true)
                , strategy("GENERATION")
                , population_size(500)
                , selection_size(100)
                , max_generations(500)
                , max_trials(max_generations * population_size)
                , init_mutation_rate(0.01)
                , meta_mutation_rate(0.5)
                , moving_rate(0.5)
                , selection_bias(1.0) // (0,...,5]
                , seed()
                , initial_population()
                , param_p(3.0)
                , param_d(-1.0)
                , param_m(1.0)
                , push_mode(0)
                , push_body(0)
                , push_cycle(0)
                , push_steps(0)
                , push_strength(0.0)
                , fitness_function("FORWARDS")
{
    if (read_option_bool(argc, argv, "--help", "-h"))
    {
        print_options();
        exit(EXIT_SUCCESS);
    }
    else if (read_option_bool(argc, argv, "--new"   , "-n"))
    {
        project_name = read_option_string(argc, argv, "--new"   , "-n");
        read_setting_file(read_option_string(argc, argv, "--settings", "-s"));
        project_status = NEW;
    }
    else if (read_option_bool(argc, argv, "--resume"   , "-r"))
    {
        project_name = read_option_string(argc, argv, "--resume", "-r");
        read_project_file(project_name);
        project_status = RESUME;
    }
    else if (read_option_bool(argc, argv, "--watch", "-w"))
    {
        project_name = read_option_string(argc, argv, "--watch" , "-w");
        read_project_file(project_name);
        project_status = WATCH;
    }

    assert(project_name != "");
}

void Setting::read_setting_file(const std::string& setting_name)
{
    if (setting_name == "") {
        wrn_msg("No settings file provided. Using default settings.");
        return;
    }
    read_configuration(setting_name);
}

void Setting::read_project_file(const std::string& project_name) {
    read_configuration(FOLDER_PREFIX + project_name + "/evolution.conf");
}

void
Setting::read_configuration(const std::string& filename)
{
    /* read all static settings */
    sts_msg("Reading configuration: %s", filename.c_str());
    config settings_file(filename, true /*quit on fail*/);

    robot_ID = settings_file.readINT("ROBOT", robot_ID);
    scene_ID = settings_file.readINT("SCENE", scene_ID);
    dbg_msg("   Robot No. %d and Scene No. %d", robot_ID, scene_ID);

    max_steps = settings_file.readUINT("MAX_STEPS", max_steps);
    max_power = settings_file.readUINT("MAX_POWER", max_power);
    initial_steps = settings_file.readUINT("INITIAL_STEPS");
    dbg_msg("   Max_Power is %u, max. %u steps a trial and %u initial steps", max_power, max_steps, initial_steps);

    push_mode     = settings_file.readUINT("PUSH_MODE"    , push_mode);
    push_body     = settings_file.readUINT("PUSH_BODY"    , push_body);
    push_cycle    = settings_file.readUINT("PUSH_CYCLE"   , push_cycle);
    push_steps    = settings_file.readUINT("PUSH_STEPS"   , push_steps);
    push_strength = settings_file.readDBL ("PUSH_STRENGTH", push_strength);

    dbg_msg("   push steps = %u/%u with strength = %lf (mode=%u) on body %u", push_steps, push_cycle, push_strength, push_mode, push_body);
    assert(push_steps <= push_cycle);

    efficient    = settings_file.readBOOL("EFFICIENT", efficient);
    drop_penalty = settings_file.readBOOL("DROP_PENALTY", drop_penalty);
    out_of_track_penalty = settings_file.readBOOL("OUT_OF_TRACK_PENALTY", out_of_track_penalty);
    stop_penalty = settings_file.readBOOL("STOP_PENALTY", stop_penalty);
    symmetric_controller = settings_file.readBOOL("SYMMETRIC_CONTROLLER", symmetric_controller);
    dbg_msg("   Efficient: %s, Dropping: %s, Track: %s, Stop: %s, Symmetric: %s", (efficient?"Yes":"No"), (drop_penalty?"Yes":"No"), (out_of_track_penalty?"Yes":"No"), (stop_penalty?"Yes":"No"), (symmetric_controller?"Yes":"No"));

    strategy = settings_file.readSTR("STRATEGY", strategy);
    dbg_msg("   Strategy is: %s", strategy.c_str());

    max_generations = settings_file.readUINT("MAX_GENERATIONS", max_generations);
    population_size = settings_file.readUINT("POPULATION_SIZE", population_size);
    selection_size  = settings_file.readUINT("SELECTION_SIZE" , selection_size );
    max_trials      = settings_file.readUINT("MAX_TRIALS"     , max_trials);
    dbg_msg("   Generations: %u, Population Size: %u, Selection Size: %u", max_generations, population_size, selection_size);

    init_mutation_rate = settings_file.readDBL("INIT_MUTATION_RATE", init_mutation_rate);
    meta_mutation_rate = settings_file.readDBL("META_MUTATION_RATE", meta_mutation_rate);
           moving_rate = settings_file.readDBL("MOVING_RATE"       ,        moving_rate);
        selection_bias = settings_file.readDBL("SELECTION_BIAS"    ,     selection_bias);
    dbg_msg("   Initial Random: %1.2f, Meta Mutation Rate: %1.2f", init_mutation_rate, meta_mutation_rate);
    dbg_msg("   Moving Rate: %1.2f, Selection Bias: %1.2f", moving_rate, selection_bias);

    param_p = settings_file.readDBL("PARAM_P", param_p);
    param_d = settings_file.readDBL("PARAM_D", param_d);
    param_m = settings_file.readDBL("PARAM_M", param_m);

    seed = settings_file.readSTR("SEED");
    if (seed == "") dbg_msg("   No seed file.");
    else dbg_msg("   Seed file name: %s", seed.c_str());

    initial_population = settings_file.readSTR("INIT_POPULATION");
    if (initial_population == "") dbg_msg("   No initial population file.");
    else dbg_msg("   Initial population file name: %s", initial_population.c_str());

    fitness_function = settings_file.readSTR("FITNESS_FUNCTION");
    assert(not fitness_function.empty());
}

const std::string&
Setting::save_to_projectfile(const std::string& filename) const
{
    sts_msg("Saving to project file.");
    config project_file(filename);
    project_file.writeUINT("ROBOT"               , robot_ID);
    project_file.writeUINT("SCENE"               , scene_ID);
    project_file.writeUINT("MAX_STEPS"           , max_steps);
    project_file.writeUINT("MAX_POWER"           , max_power);
    project_file.writeUINT("INITIAL_STEPS"       , initial_steps);
    project_file.writeUINT("PUSH_MODE"           , push_mode);
    project_file.writeUINT("PUSH_BODY"           , push_body);
    project_file.writeUINT("PUSH_CYCLE"          , push_cycle);
    project_file.writeUINT("PUSH_STEPS"          , push_steps);
    project_file.writeUINT("PUSH_STRENGTH"       , push_strength);

    project_file.writeSTR ("STRATEGY"            , strategy);

    assert(not fitness_function.empty());
    project_file.writeSTR ("FITNESS_FUNCTION"    , fitness_function);

    if ("" != seed) project_file.writeSTR("SEED" , seed);

    project_file.writeBOOL("EFFICIENT"           , efficient);
    project_file.writeBOOL("DROP_PENALTY"        , drop_penalty);
    project_file.writeBOOL("OUT_OF_TRACK_PENALTY", out_of_track_penalty);
    project_file.writeBOOL("STOP_PENALTY"        , stop_penalty);
    project_file.writeBOOL("SYMMETRIC_CONTROLLER", symmetric_controller);

    project_file.writeDBL ("INIT_MUTATION_RATE"  , init_mutation_rate);
    project_file.writeDBL ("META_MUTATION_RATE"  , meta_mutation_rate);

    project_file.writeUINT("POPULATION_SIZE"     , population_size);

    project_file.finish();
    return filename;
}


void
Setting::print_options(void)
{
    printf("Help, Options: \n");
    printf("    -p  --port     change TCP port\n");
    printf("    -n  --new      create new evolution\n");
    printf("    -r  --resume   resume with existing evolution\n");
    printf("    -w  --watch    watch existing evolution\n");
    printf("    -s  --settings settings file\n");
    printf("    -b  --blind    no visuals\n");
    printf("    -h  --help     display this help\n");
    printf("\n");
}


bool read_option_bool(int argc, char **argv, const std::string long_name, const std::string short_name, bool def)
{
    bool value = def;

    for (int i = 1; i < argc; ++i)
    {
        const std::string option_name = argv[i];
        value = (long_name == option_name || short_name == option_name);
        if (value) break;
    }
    dbg_msg("Reading option '%s' = %s", long_name.c_str(), value? "true":"false");
    return value;
}

unsigned int
read_option_uint(int argc, char **argv, const std::string long_name, const std::string short_name, unsigned int def = 0)
{
    unsigned int value = def;

    for (int i = 1; i < argc; ++i)
    {
        const std::string option_name = argv[i];
        if (long_name == option_name || short_name == option_name)
        {
            if (argc < i+2) {
                printf("usage: %s %s <uint>\n", argv[0], long_name.c_str());
                exit(0);
            } else {
                value = abs(atoi(argv[i+1]));
                ++i;
            }
            break;
        }
    }
    dbg_msg("Reading option '%s' = %u", long_name.c_str(), value);
    return value;
}

std::string
read_option_string(int argc, char **argv, const std::string long_name, const std::string short_name)
{
    std::string value;

    for (int i = 1; i < argc; ++i)
    {
        const std::string option_name = argv[i];
        if (long_name == option_name || short_name == option_name)
        {
            if (argc < i+2) {
                printf("usage: %s %s <string>\n", argv[0], long_name.c_str());
                exit(0);
            } else {
                value = argv[i+1];
                ++i;
            }
            break;
        }
    }
    dbg_msg("Reading option '%s' = '%s'", long_name.c_str(), value.c_str());
    return value;
}
