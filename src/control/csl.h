#ifndef CSL_H
#define CSL_H

//#include "../main.h"

class csl
{
    private:
        double z;  // csl hidden state
        double gi; // loop gain
        double gf; // feedback gain
        double u;  // output

    public:
        csl(void);
        csl(double *motor_output, double *angle_sensor);
        csl(double *motor_output, double *angle_sensor, double *angular_velocity_sensor);
        void loop(const double mode);
        void reset(const double initial);

        void set_control_parameter(const double gi, const double gf);
        void set_joint_limits(const double llower, const double lupper);
};


#endif // CSL

