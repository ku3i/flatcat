#ifndef JOINTCONTROLLER_H
#define JOINTCONTROLLER_H

#include <vector>
#include <cmath>
#include <cassert>

#include <common/modules.h>
#include <common/config.h>
#include <common/robot_conf.h>
#include <common/file_io.h>

#define INITIAL_BIAS 0.1

/** WARNING: This class is outdated, do not use it anymore.
  * Use Jointcontrol instead.
  */

class Jointcontroller
{
public:
    Jointcontroller( Robot_Configuration& configuration, bool symmetric_controller
                   , double param_p, double param_d, double param_m, const std::string& seed_filename) __attribute_deprecated__;

    ~Jointcontroller() { sts_msg("Destroying joint controller."); }

    void loop(void);
    void reset(void);

    const std::vector<double> get_control_parameter(void) const;
    unsigned int get_number_of_parameter(void) const { return total_num_params; }

    void set_control_parameter(const std::vector<double>& params);
    void set_seed_parameter(void);
    double get_normalized_mechanical_power(void) const;
    void print_parameter(void) const;

private:
    Robot_Configuration& robot;

    const unsigned int num_inputs;
    unsigned int total_num_params;

    std::vector<double> activation;
    std::vector<std::vector<double> > weights;

    std::vector<double> X;
    std::vector<double> Y;

    std::vector<double> seed_from_file;

    void set_initial_parameter(double p, double d, double m);
    void load_seed(const std::string& filename);

};


#endif // JOINTCONTROLLER
