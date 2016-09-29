/* setting.h */

#ifndef SETTING_H
#define SETTING_H

#include <stdio.h>

#include <common/log_messages.h>
#include <common/socket_client.h>
#include <evolution/evolution_strategy.h>

#define FOLDER_PREFIX "./data/exp/"

enum PStatus {NEW, RESUME, WATCH}; // TODO rename

class Setting
{
    private:
        void        print_options(void);

    public:
        /* general */
        std::string project_name;
        PStatus     project_status;
        bool        visuals;

        /* simloid */
        unsigned short tcp_port;
        unsigned int   robot_ID;
        unsigned int   scene_ID;

        /* evaluation */
        unsigned int max_steps;
        unsigned int max_power;
        unsigned int initial_steps;

        bool efficient;
        bool drop_penalty;
        bool out_of_track_penalty;
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

        double       param_p;
        double       param_d;
        double       param_m;

        unsigned int push_mode; //TODO struct?
        unsigned int push_body;
        unsigned int push_cycle;
        unsigned int push_steps;
        double       push_strength;

        std::string  fitness_function;

        Setting(int argc, char **argv);
        void read_setting_file(const std::string& setting_name);
        void read_project_file(const std::string& project_name);
        void read_configuration(const std::string& filename);
        const std::string& save_to_projectfile(const std::string& filename) const;

};

#endif //SETTING_H
