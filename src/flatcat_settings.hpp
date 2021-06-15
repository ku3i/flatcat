#ifndef FLATCAT_SETTINGS_HPP
#define FLATCAT_SETTINGS_HPP

#include <string.h>
#include <common/settings.h>
#include <common/vector_n.h>


namespace supreme {

namespace defaults {
    const std::string lib_folder = "./data/lib_flatcat/";
    const std::string settings_filename = "flatcat.dat";
    const std::size_t max_number_of_gaits = 8;
    const std::string group = "224.0.0.1";//"239.255.255.252";
    const unsigned port = 1900;
    const VectorN joint_offsets = { .0, /* HEAD 0 */
                                    .0, /* BODY 1 */
                                    .0  /* TAIL 2 */
                                  };

    const float voltage_limit = 0.25;
}

class FlatcatSettings : public Settings_Base
{
public:

    unsigned max_number_of_gaits;
    std::string lib_folder;
    std::string group;
    unsigned port;
    VectorN joint_offsets;
    float voltage_limit;

    VectorN sarsa_learning_rates = {0.05, 0.05, 0.005, 0.005};
    uint64_t trial_time_s = 60;
    uint64_t eigenzeit_steps = 1000; // 10 seconds max.

    float epsilon_exploration = 0.1;

    unsigned save_cycles_s = 120;

    std::string save_state_name;
    std::string save_folder = "./data/";
    bool clear_state;

    FlatcatSettings(int argc, char **argv)
    : Settings_Base       (argc, argv                          , defaults::settings_filename.c_str())
    , max_number_of_gaits (read_uint ("max_number_of_gaits"    , defaults::max_number_of_gaits     ))
    , lib_folder          (read_str  ("lib_folder"             , defaults::lib_folder              ))
    , group               (read_str  ("group"                  , defaults::group                   ))
    , port                (read_uint ("port"                   , defaults::port                    ))
    , joint_offsets       (read_vec  ("joint_offsets"          , defaults::joint_offsets           ))
    , voltage_limit       (read_float("voltage_limit"          , defaults::voltage_limit           ))
    , save_state_name     (read_string_option(argc, argv, "-n", "--name", "default"                ))
    , clear_state         (read_option_flag  (argc, argv, "-c", "--clear"                          ))
    {

        save_folder += save_state_name + "/";
    }
};

} /* namespace supreme */

#endif /* FLATCAT_SETTINGS_HPP */
