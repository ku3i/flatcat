
#include "./setting.h"

/**TODO: this needs an update someday.*/


bool         read_option_bool  (int argc, char **argv, const std::string long_name, const std::string short_name, bool def = false);
unsigned int read_option_uint  (int argc, char **argv, const std::string long_name, const std::string short_name, unsigned int def);
std::string  read_option_string(int argc, char **argv, const std::string long_name, const std::string short_name);

Setting::Setting( int argc, char **argv )
                : project_name()
                , project_status(NEW)
                , visuals(not read_option_bool(argc, argv, "--blind", "-b"))
                , interlaced_mode(false)
                , tcp_port(read_option_uint(argc, argv, "--port", "-p", network::constants::default_port))
                , robot_ID(31)
                , scene_ID(0)
                , max_steps(1000)
                , max_power(100)
                , max_dctrl(100)
                , initial_steps(0)
                , efficient(true)
                , drop_penalty(true)
                , out_of_track_penalty(true)
                , stop_penalty(false)
                , symmetric_controller(true)
                , strategy("GENERATION")
                , population_size(500)
                , selection_size(100)
                , max_generations(500)
                , cur_generations(0)
                , max_trials(max_generations * population_size)
                , cur_trials(0)
                , init_mutation_rate(0.01)
                , meta_mutation_rate(0.5)
                , moving_rate(0.5)
                , selection_bias(1.0) // (0,...,5]
                , seed()
                , initial_population()
                , param{3.0, -1.0, 1.0}
                , push()
                , fitness_function("FORWARDS")
                , rnd({"NONE", 0.0, 0})
                , growth({1.0, 0.0})
                , friction(1.0)
                , low_sensor_quality(false)
                , L1_normalization(false)
                , target(.0)
                , drop_level(.5)
                , stop_level(0.0005)
                , corridor(.5)
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
Setting::read_configuration(std::string const& filename)
{
    /* read all static settings */
    sts_msg("Reading configuration: %s", filename.c_str());
    config settings_file(filename, true /*quit on fail*/);

    robot_ID             = settings_file.readINT ("ROBOT"               , robot_ID            );
    scene_ID             = settings_file.readINT ("SCENE"               , scene_ID            );

    initial_steps        = settings_file.readUINT("INITIAL_STEPS"       , initial_steps       );
    max_steps            = settings_file.readUINT("MAX_STEPS"           , max_steps           );

    push.mode            = settings_file.readUINT("PUSH_MODE"           , push.mode           );
    push.body            = settings_file.readUINT("PUSH_BODY"           , push.body           );
    push.cycle           = settings_file.readUINT("PUSH_CYCLE"          , push.cycle          );
    push.steps           = settings_file.readUINT("PUSH_STEPS"          , push.steps          );
    push.strength        = settings_file.readDBL ("PUSH_STRENGTH"       , push.strength       );

    assert ( push.steps <= push.cycle );

    efficient            = settings_file.readBOOL("EFFICIENT"           , efficient           );
    max_power            = settings_file.readUINT("MAX_POWER"           , max_power           );
    max_dctrl            = settings_file.readUINT("MAX_DCTRL"           , max_dctrl           );

    drop_penalty         = settings_file.readBOOL("DROP_PENALTY"        , drop_penalty        );
    drop_level           = settings_file.readDBL ("DROP_LEVEL"          , drop_level          );

    out_of_track_penalty = settings_file.readBOOL("OUT_OF_TRACK_PENALTY", out_of_track_penalty);
    corridor             = settings_file.readDBL ("CORRIDOR"            , corridor            );

    stop_penalty         = settings_file.readBOOL("STOP_PENALTY"        , stop_penalty        );
    stop_level           = settings_file.readDBL ("STOP_LEVEL"          , stop_level          );

    symmetric_controller = settings_file.readBOOL("SYMMETRIC_CONTROLLER", symmetric_controller);
    strategy             = settings_file.readSTR ("STRATEGY"            , strategy            );

    max_generations      = settings_file.readUINT("MAX_GENERATIONS"     , max_generations     );
    cur_generations      = settings_file.readUINT("CURRENT_GENERATION"  , cur_generations     );
    population_size      = settings_file.readUINT("POPULATION_SIZE"     , population_size     );
    selection_size       = settings_file.readUINT("SELECTION_SIZE"      , selection_size      );

    max_trials           = settings_file.readUINT("MAX_TRIALS"          , max_trials          );
    cur_trials           = settings_file.readUINT("CURRENT_TRIAL"       , cur_trials          );
    moving_rate          = settings_file.readDBL ("MOVING_RATE"         , moving_rate         );
    selection_bias       = settings_file.readDBL ("SELECTION_BIAS"      , selection_bias      );

    init_mutation_rate   = settings_file.readDBL ("INIT_MUTATION_RATE"  , init_mutation_rate  );
    meta_mutation_rate   = settings_file.readDBL ("META_MUTATION_RATE"  , meta_mutation_rate  );

    param.pgain          = settings_file.readDBL ("PARAM_P"             , param.pgain         );
    param.damping        = settings_file.readDBL ("PARAM_D"             , param.damping       );
    param.motor_self     = settings_file.readDBL ("PARAM_M"             , param.motor_self    );

    seed                 = settings_file.readSTR ("SEED"                , seed                );
    initial_population   = settings_file.readSTR ("INIT_POPULATION"     , initial_population  );
    fitness_function     = settings_file.readSTR ("FITNESS_FUNCTION"    , fitness_function    );

    rnd.mode             = settings_file.readSTR ("RANDOM_MODE"         , rnd.mode            );
    rnd.value            = settings_file.readDBL ("RANDOM_VALUE"        , rnd.value           );
    rnd.init             = settings_file.readUINT("RANDOM_INIT"         , rnd.init            );

    growth.init          = settings_file.readDBL ("GROWTH_INIT"         , growth.init         );
    growth.rate          = settings_file.readDBL ("GROWTH_RATE"         , growth.rate         );

    friction             = settings_file.readDBL ("FRICTION"            , friction            );
    target               = settings_file.readDBL ("TARGET"              , target              );

    interlaced_mode      = settings_file.readBOOL("INTERLACED"          , interlaced_mode     );
    low_sensor_quality   = settings_file.readBOOL("LOW_SENSOR_QUALITY"  , low_sensor_quality  );

    L1_normalization     = settings_file.readBOOL("L1_NORMALIZATION"    , L1_normalization    );
    assert(false == L1_normalization);
}

const std::string&
Setting::save_to_projectfile(const std::string& filename) const
{
    sts_msg("Saving to project file.");
    config project_file(filename);

    if (interlaced_mode) /* default is false for evolution, so only write if true */
        project_file.writeBOOL("INTERLACED", true);

    project_file.writeUINT("ROBOT"               , robot_ID        );
    project_file.writeUINT("SCENE"               , scene_ID        );
    project_file.writeUINT("MAX_STEPS"           , max_steps       );
    project_file.writeUINT("MAX_POWER"           , max_power       );
    project_file.writeUINT("MAX_DCTRL"           , max_dctrl       );
    project_file.writeUINT("INITIAL_STEPS"       , initial_steps   );
    project_file.writeUINT("PUSH_MODE"           , push.mode       );
    project_file.writeUINT("PUSH_BODY"           , push.body       );
    project_file.writeUINT("PUSH_CYCLE"          , push.cycle      );
    project_file.writeUINT("PUSH_STEPS"          , push.steps      );
    project_file.writeUINT("PUSH_STRENGTH"       , push.strength   );
    project_file.writeSTR ("STRATEGY"            , strategy        );

    assert(not fitness_function.empty());
    project_file.writeSTR ("FITNESS_FUNCTION"    , fitness_function);

    if ("" != seed) project_file.writeSTR("SEED" , seed);

    project_file.writeBOOL("EFFICIENT"           , efficient);
    project_file.writeBOOL("DROP_PENALTY"        , drop_penalty);
    project_file.writeDBL ("DROP_LEVEL"          , drop_level);
    project_file.writeBOOL("OUT_OF_TRACK_PENALTY", out_of_track_penalty);
    project_file.writeDBL ("CORRIDOR"            , corridor);
    project_file.writeBOOL("STOP_PENALTY"        , stop_penalty);
    project_file.writeDBL ("STOP_LEVEL"          , stop_level);
    project_file.writeBOOL("SYMMETRIC_CONTROLLER", symmetric_controller);

    project_file.writeDBL ("INIT_MUTATION_RATE"  , init_mutation_rate);
    project_file.writeDBL ("META_MUTATION_RATE"  , meta_mutation_rate);

    project_file.writeUINT("POPULATION_SIZE"     , population_size);

    project_file.writeSTR ("RANDOM_MODE"         , rnd.mode);
    project_file.writeDBL ("RANDOM_VALUE"        , rnd.value);
    project_file.writeUINT("RANDOM_INIT"         , rnd.init);

    project_file.writeDBL ("GROWTH_INIT"         , growth.init);
    project_file.writeDBL ("GROWTH_RATE"         , growth.rate);

    project_file.writeDBL ("FRICTION"            , friction);

    project_file.writeBOOL("LOW_SENSOR_QUALITY"  , low_sensor_quality);
    project_file.writeBOOL("L1_NORMALIZATION"    , L1_normalization);
    project_file.writeDBL ("TARGET"              , target);

    /** NOTE: Strategy-specific settings are saved separately!
        e.g. selection bias, moving rate, max generations, current_trial */

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
