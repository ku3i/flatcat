#ifndef FLATCAT_ROBOT_HPP
#define FLATCAT_ROBOT_HPP

#include <cassert>
#include <array>
#include <common/log_messages.h>

#include <robots/robot.h>
#include <robots/joint.h>
#include <robots/accel.h>

#include <communication_ctrl.hpp>
#include <motorcord.hpp>
#include <flatcat_settings.hpp>

namespace supreme {

namespace constants {
    const unsigned num_joints = 3;

    const std::array<int16_t, num_joints> dir = { +1, +1, +1};
    const double position_scale = 270.0/360.0;

} /* namespace constants */

class FlatcatRobot : public robots::Robot_Interface
{
    supreme::motorcord motorcord;

    std::size_t number_of_joints;
    std::size_t number_of_joints_sym;
    std::size_t number_of_accels;

    robots::Jointvector_t joints;
    robots::Accelvector_t accels;

    double voltage_amp = 0.;
    bool   enabled = false;

public:


    FlatcatRobot(FlatcatSettings const& settings)
    : motorcord(constants::num_joints, 100.0/*Hz*/, false)
    , number_of_joints(motorcord.size())
    , number_of_joints_sym(/* will be counted */)
    , number_of_accels(1)
    , joints()
    , accels()
    {

        assert(motorcord.size() == constants::num_joints);
        sts_msg("Creating Flatcat Robot Interface");
        joints.reserve(number_of_joints);

        /* define joints */
        joints.emplace_back(  0, robots::Joint_Type_Normal,  0, "HEAD" , -0.75, +0.75, .0 );
        joints.emplace_back(  1, robots::Joint_Type_Normal,  1, "MIDL" , -0.75, +0.75, .0 );
        joints.emplace_back(  2, robots::Joint_Type_Normal,  2, "TAIL" , -0.75, +0.75, .0 );

        unsigned i = 0;
        for (auto const& j : joints) {
            assert(j.joint_id == i++);
            if (j.is_symmetric()) ++number_of_joints_sym; // count symmetric joints
        }

        accels.emplace_back();

        /* configure joints */
        assertion(settings.joint_offsets.size() == joints.size(), "%u =!= %u", settings.joint_offsets.size(), joints.size());
        for (auto const& j : joints) {
            unsigned i = j.joint_id;
            assert(i < constants::num_joints);
            auto& m  = motorcord[i];
            m.set_direction  (constants::dir.at(i)        );
            m.set_scalefactor(constants::position_scale   );
            m.set_offset     (settings.joint_offsets.at(i));
        }
    }

    std::size_t get_number_of_joints           (void) const { return number_of_joints;     }
    std::size_t get_number_of_symmetric_joints (void) const { return number_of_joints_sym; }
    std::size_t get_number_of_accel_sensors    (void) const { return number_of_accels;     }

    const robots::Jointvector_t& get_joints(void) const { return joints; }
          robots::Jointvector_t& set_joints(void)       { return joints; }

    const robots::Accelvector_t& get_accels(void) const { return accels; }
          robots::Accelvector_t& set_accels(void)       { return accels; }

    bool execute_cycle(void)
    {
        write_motorcord();          /* write motors     */
        motorcord.execute_cycle();  /* read motor cord  */
        read_motorcord();           /* read sensors     */
        return true;
    }

    double get_normalized_mechanical_power(void) const
    {
        double power = .0;
        for (unsigned i = 0; i < motorcord.size(); ++i) {
            auto const& m = motorcord[i];
            power += m.get_data().voltage_supply * m.get_data().current;
        }
        return power;
    }

    void read_motorcord(void)
    {
        for (auto& j : joints) {
            auto const& m = motorcord[j.joint_id];
            j.s_ang = m.get_data().position;
            j.s_vel = m.get_data().velocity;
        }

        /**TODO accelerometer
        auto const& r0 = spinalcord.status_data.motors[0];
        accels[0].a.x = +r0.acceleration.x;
        accels[0].a.y = -r0.acceleration.z;
        accels[0].a.z = +r0.acceleration.y; */
    }

    void write_motorcord(void) {
        for (auto& j : joints) {
            auto& m = motorcord[j.joint_id];
            //TODO important? m.data.last_output = 0.f;
            if (enabled) {
                m.set_target_voltage( clip(voltage_amp*j.motor.get(), voltage_amp) );
            }
            j.motor.transfer();
            j.motor = .0f;
        }
    }

    void set_voltage_amplitude(double value) { voltage_amp = clip(value, 0.0, 1.0); }
    void set_enable(bool value) { enabled = value; }

    void disable_motors(void) {
		//motorcord.disable_all();
		for (auto& j : joints)
            motorcord[j.joint_id].set_target_voltage(.0);
	}

    /* non-robot interface member function */
    //SpinalCord::TimingStats const& get_motorcord_timing(void) const { return spinalcord.get_timing(); }
    void reset_motor_statistics(void) { /**TODO spinalcord.reset_statistics();*/ }

    supreme::motorcord const& get_motors(void) const { return motorcord; }
    supreme::motorcord      & set_motors(void)       { return motorcord; }
};

} /* namespace supreme */

#endif /* FLATCAT_ROBOT_HPP */
