#ifndef SO2_CONTROLLER_HPP_INCLUDED
#define SO2_CONTROLLER_HPP_INCLUDED

#include <math.h>
#include <common/modules.h>
#include <robots/robot.h>
#include <controller/csl_control.hpp>

namespace jcl {

namespace so2 {

    struct Params_t {
        float pctrl;
        float off;
        float amp;
        float phs;
        float minv;
        float maxv;
    };

    const unsigned num_joints = 3;

    const float max_val = 0.25f;
    const float preamp = 2.f;

    /*                             pctrl |   off |  amp | phase |  min |  max | joint name */
    Params_t pset[num_joints] = { {  0.5 ,  0.00 ,  0.2 ,     0 , -1.0 , +1.0 } // 0 head
                                , {  0.5 ,  0.00 ,  0.2 ,    60 , -1.0 , +1.0 } // 1 body
                                , {  0.5 ,  0.00 ,  0.2 ,   120 , -1.0 , +1.0 } // 2 tail

                                };

}


class SO2_Controller {

    typedef std::array<float, so2::num_joints> UserParameter_t;


    robots::Robot_Interface& robot;
    std::vector<supreme::csl_control>& controls;
    UserParameter_t const& usr_params;

    float x1 = .001f;
    float x2 = .0f;

    float w11 = .0f;
    float w12 = .0f;
    float w21 = .0f;
    float w22 = .0f;

    float freq= 0.85f;
    float volume = 1.f; //remove if not needed

    float get_amp(unsigned idx) { return usr_params.at(idx    ); }
    float get_phs(unsigned /*idx*/) { return 0.0; } //TODO: usr_params.at(idx + 3); }


public:

    SO2_Controller(robots::Robot_Interface& robot, std::vector<supreme::csl_control>& controls, UserParameter_t const& usr_params)
    : robot(robot)
    , controls(controls)
    , usr_params(usr_params) {

        sts_msg("num usr params = %u", usr_params.size());
        //TODO assert(usr_params.size() == robot.get_joints().size()*2);
    }

    void set_frequency(float f /*-1..1*/) { freq = 0.75f + 0.2f*clip(f); }

    void reset(void) { x1 = .001f; x2 = .0f; }

    void execute_cycle(void)
    {
        update_weights(freq);

        float y1 = tanh(w11*x1 + w12*x2);
        float y2 = tanh(w21*x1 + w22*x2);
        x1 = y1;
        x2 = y2;

        robots::Jointvector_t& joints = robot.set_joints();

        assert(joints.size() == controls.size());
        for (std::size_t i = 0; i < joints.size(); ++i)
        {
            /* get desired phase and create phase shifted motor signals */
            const float q = M_PI*(so2::pset[i].phs/180.f + get_phs(i));
            const float a = cos(q);
            const float b = sin(q);
            const float u = a*x1 + b*x2;

            const float A = so2::preamp * so2::pset[i].amp * get_amp(i); // amplitude
            const float out = clip( A*u, so2::max_val * so2::pset[i].minv
                                       , so2::max_val * so2::pset[i].maxv );

            /* amplitude  * ( soll - ist) */
            const float target = clip(so2::pset[i].pctrl * (so2::pset[i].off - joints[i].s_ang),0.5)
                               + out;

            joints[i].motor = controls.at(i).step(joints[i].s_ang, clip(target));
        }



    }

    void update_weights(float freq)
    {
        /*
            (x1,x2)^T  <-- tanh[ (1+eps) * r(dp) * (x1,x2)^T ]

            r(dp) = [ cos(dp), sin(dp)
                     -sin(dp), cos(dp) ]
        */
        const float val = clip(freq, .0f, 1.f);

        /* step size of oscillator (determines frequency) */
        const float dp = M_PI/(8.f + 42.f*val);  // pi/8 .. pi/50

        /* add non-linearity */
        const float k = 1.0f + 1.5f*dp; // useful range: 1 + 1dp ... 1 + 2dp

        /* determine self and ring coupling */
        const float s = cos(dp) * k;
        const float r = sin(dp) * k;

        /* rotation matrix */
        w11 =  s; w12 = r;
        w21 = -r; w22 = s;
    }

};


} // namespace jcl

#endif // SO2_CONTROLLER_HPP_INCLUDED
