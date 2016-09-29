#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include "../main.h"

class positioncontrol
{
    // this is a cascaded position/velocity control

    private:
        double pKp, pKi, pKd; // PID parameter position
        double vKp, vKi, vKd; // PID parameter velocity

        double u;  // output

    public:
        pid(void);
        void loop(const double target_velocity, const double current_velocity);
        void reset(void);

        void set_control_parameter(const double kp, const double ki, const double kd);
        void set_limits(const double llower, const double lupper);
};


#endif // POSITIONCONTROL



