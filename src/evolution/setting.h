/* setting.h */

#ifndef SETTING_H
#define SETTING_H

#include <stdio.h>

#include <common/log_messages.h>
#include <common/socket_client.h>
#include <evolution/evolution_strategy.h>

#define FOLDER_PREFIX "../data/exp/"

enum PStatus {NEW, RESUME, WATCH}; /** TODO rename */

class Setting
{
    private:
        void        print_options(void);

    public:
        /* general */
        std::string project_name;
        PStatus     project_status;
        bool        visuals;
        bool        interlaced_mode;

        /* simloid */
        unsigned short tcp_port;
        unsigned int   robot_ID;
        unsigned int   scene_ID;

        /* evaluation */
        unsigned int max_steps;
        unsigned int max_power;
        unsigned int max_dctrl;
        unsigned int initial_steps;

        bool efficient;
        bool drop_penalty;
        bool out_of_track_penalty;
        bool stop_penalty;
        bool symmetric_controller;

        /* evolution */
        std::string  strategy;
        unsigned int population_size;
        unsigned int selection_size;
        unsigned int max_generations;
        unsigned int max_trials;
        double       init_mutation_rate;
        double       meta_mutation_rate;
        double       moving_rate;
        double       selection_bias;
        std::string  seed;
        std::string  initial_population;


        double       param_p;
        double       param_d;
        double       param_m;

        struct Push_Settings {
            unsigned int mode;
            unsigned int body;
            unsigned int cycle;
            unsigned int steps;
            double       strength;
        } push;

        std::string  fitness_function;

        struct Random_Mode_Settings {
            std::string mode;
            double value;
            mutable uint64_t init;
        } rnd;

        bool         low_sensor_quality;
        bool         L1_normalization;

        double       target;

        Setting(int argc, char **argv);
        void read_setting_file(const std::string& setting_name);
        void read_project_file(const std::string& project_name);
        void read_configuration(const std::string& filename);
        const std::string& save_to_projectfile(const std::string& filename) const;

};

#endif //SETTING_H
