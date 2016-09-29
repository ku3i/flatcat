#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include "../main.h"

class pidcontrol
{
    private:
        double kp, ki, kd; // PID parameter
        double u;  // output

    public:
        pid(void);
        void loop(const double set_point, const double process_variable);
        void reset(void);

        void set_control_parameter(const double kp, const double ki, const double kd);
        void set_limits(const double llower, const double lupper);
};


#endif // PIDCONTROL


