#ifndef FLATCAT_CONTROL_UDP_HPP
#define FLATCAT_CONTROL_UDP_HPP

#include <array>
#include <experimental/filesystem>

#include <common/basic.h>
#include <common/application_base.h>
#include <common/event_manager.h>
#include <common/log_messages.h>
#include <common/basic.h>
#include <common/globalflag.h>
#include <common/modules.h>
#include <common/udp.hpp>
#include <common/stopwatch.h>
#include <common/socket_client.h>
#include <common/view_manager.h>

#include <draw/draw.h>
#include <midi/midi_in.h>

#include <flatcat_graphics.hpp>
#include <flatcat_control.hpp>

#include <robots/robot.h>
#include <robots/accel.h>

#include <learning/gmes.h>
#include <learning/gmes_graphics.h>
#include <learning/sarsa.h>
#include <learning/sarsa_graphics.h>
#include <learning/reward.h>
#include <learning/forcefield.h>
#include <learning/payload.h>
#include <learning/payload_graphics.h>
#include <learning/eigenzeit.h>
#include <learning/action_selection.h>
#include <learning/epsilon_greedy.h>

#include "gmes_joint_group.hpp"
#include "flatcat_settings.hpp"

extern GlobalFlag do_pause;
extern GlobalFlag fast_forward;

namespace supreme {

namespace constants {

    /* maps the motor ids to channel numbers of korg kontrol */
    const std::array<uint8_t, num_joints> FlatcatMidiMap = { 14, 15, 16 };
}



class FlatcatUDPRobot : public robots::Robot_Interface {
public:
    typedef std::array<float, constants::num_joints> TargetPosition_t;

    network::UDPReceiver<113> receiver;

    uint16_t sync   = 0;
    uint64_t cycles = 0;
    uint8_t  chksum = 0;

    typedef std::vector<supreme::interface_data> Motordata_t;
    Motordata_t motors;

    robots::Jointvector_t      joints;
    robots::Accelvector_t      accels;

    struct Control_t {
        bool enabled = false;
        bool def_pos = false;
        float amplitude = 0.f;
        float modulate  = 0.f;
        float inputgain = 0.f;
        supreme::ControlMode_t mode = ControlMode_t::none;
        TargetPosition_t user_target_position = TargetPosition_t{.0};
    } control;


    FlatcatUDPRobot()
    : receiver("239.255.255.252", 7331)
    , motors(3 /**TODO determine automatically*/)
    , joints()
    , accels()
    , control()
    {
        sts_msg("Creating Flatcat UDP Robot.");

        /* define joints */
        joints.emplace_back( 0, robots::Joint_Type_Normal,  0, "head" , -0.75, +0.75, .0 );
        joints.emplace_back( 1, robots::Joint_Type_Normal,  1, "body" , -0.75, +0.75, .0 );
        joints.emplace_back( 2, robots::Joint_Type_Normal,  2, "tail" , -0.75, +0.75, .0 );
    }

    Motordata_t const& get_motors(void) const { return motors; }

    bool execute_cycle(void) {
        bool result = get_UDP_data();
        read_spinalcord();
        return result;
    }

    bool get_UDP_data(void) {
        receiver.receive_message();
        if (receiver.data_received())
        {
            const uint8_t* msg = receiver.get_message();
            std::size_t n = 0;
            n = network::getfrom(sync  , msg, n);
            n = network::getfrom(cycles, msg, n);

            /* sensorimotor data */
            for (unsigned i = 0; i < motors.size(); ++i) {
                auto& m = motors[i];
                n = network::getfrom(m.id               , msg, n);
                n = network::getfrom(m.position         , msg, n);
                n = network::getfrom(m.last_p           , msg, n);
                n = network::getfrom(m.velocity         , msg, n);
                n = network::getfrom(m.current          , msg, n);
                n = network::getfrom(m.voltage_supply   , msg, n);
                n = network::getfrom(m.output_voltage   , msg, n);
            //  n = network::getfrom(m.voltage_backemf  , msg, n);
            //  n = network::getfrom(m.last_output      , msg, n);
                n = network::getfrom(m.temperature      , msg, n);
            //  n = network::getfrom(m.is_connected     , msg, n);
            //  n = network::getfrom(m.acceleration.x   , msg, n);
            //  n = network::getfrom(m.acceleration.y   , msg, n);
            //  n = network::getfrom(m.acceleration.z   , msg, n);
            //  n = network::getfrom(m.connection_losses, msg, n);
            //  n = network::getfrom(m.dir              , msg, n);
            //  n = network::getfrom(m.scale            , msg, n);
            //  n = network::getfrom(m.offset           , msg, n);
            } /* for each motor */

            /* timing */
            /*
            auto& t = timing;
            n = network::getfrom(t.mean                     , msg, n);
            n = network::getfrom(t.maxv                     , msg, n);
            n = network::getfrom(t.syncfaults               , msg, n);
            n = network::getfrom(t.board_dropouts           , msg, n);
            n = network::getfrom(t.transparent_errors       , msg, n);
            n = network::getfrom(t.transparent_packets_recv , msg, n);
            n = network::getfrom(t.collected_ids            , msg, n);
            */

            /* control read back */
            auto& c = control;
            n = network::getfrom(c.enabled  , msg, n);
            n = network::getfrom(c.def_pos  , msg, n);
            n = network::getfrom(c.amplitude, msg, n);
            n = network::getfrom(c.modulate , msg, n);
            n = network::getfrom(c.inputgain, msg, n);
            n = network::getfrom(c.mode     , msg, n);

            /* checksum */
            n = network::getfrom(chksum, msg, n);

            assertion(network::validate(msg, n), "Invalid checksum: 0x%x for %u bytes", chksum, n);

            receiver.acknowledge();
            //sts_msg("%ub, 0x%x: %lu ", n, sync, cycles);
            return true;
        }
        return false;
    }

    std::size_t get_number_of_joints(void) const { return motors.size(); }
    std::size_t get_number_of_symmetric_joints(void) const { return 0; }
    virtual std::size_t get_number_of_accel_sensors(void) const { return 0; }


    const robots::Jointvector_t& get_joints(void) const { return joints; }
          robots::Jointvector_t& set_joints(void)       { return joints; }

    const robots::Accelvector_t& get_accels(void) const { return accels; }
          robots::Accelvector_t& set_accels(void)       { return accels; }



    double get_normalized_mechanical_power(void) const { return .0; /*TODO implement */ };

    void read_spinalcord(void)
    {
        for (auto& j : joints) {
            auto const& r = motors.at( j.joint_id );
            j.s_ang = r.position;
            j.s_vel = r.velocity;
            j.motor = r.output_voltage;
        }

        /*auto const& r0 = motors[0];
        accels[0].a.x = +r0.acceleration.x;
        accels[0].a.y = -r0.acceleration.z;
        accels[0].a.z = +r0.acceleration.y;*/
    }

};

} /* namespace supreme */


class flatcat_reward_space : public reward_base
{
public:
    flatcat_reward_space(  )
    : reward_base(2)
    {
        rewards.emplace_back( "intrinsic basic", []() { assert(false); return 0; } );
        rewards.emplace_back( "intrinsic super", []() { assert(false); return 0; } );
    }

    void add_intrinsic_rewards( learning::Learning_Machine_Interface const& basic_learner
                              , learning::Learning_Machine_Interface const& super_learner )
    {
        rewards.at(0) = { "intrinsic basic", [&basic_learner]() { return 1000*basic_learner.get_learning_progress(); } };
        rewards.at(1) = { "intrinsic super", [&super_learner]() { return 1000*super_learner.get_learning_progress(); } };
    }
};


class RemoteRobotActions : public Action_Module_Interface {


    network::Socket_Client& remote;

    unsigned applied_policy = 0;
    unsigned applied_action = 0;
    unsigned applied_state  = 0;

    struct CSL_params {
        float head, body, tail;
    };

    std::vector<CSL_params> modes = { {0.0, 0.0, 0.0} // 0
                                    , {0.0, 0.0, 1.0} // 1
                                    , {0.0, 1.0, 0.0} // 2
                                    , {0.0, 1.0, 1.0} // 3
                                    , {1.0, 0.0, 0.0} // 4
                                    , {1.0, 0.0, 1.0} // 5
                                    , {1.0, 1.0, 0.0} // 6
                                    , {1.0, 1.0, 1.0} // 7
                                    };

public:

    RemoteRobotActions(network::Socket_Client& remote) : remote(remote) {}

    std::size_t get_number_of_actions(void) const { return modes.size(); }
    std::size_t get_number_of_actions_available(void) const { return modes.size(); }
    bool exists(const std::size_t action_index) const {return true; }

    void execute_cycle(learning::RL_Interface const& learner)
    {
       /* update and check state + action from learner */
       applied_policy = learner.get_current_policy();
       applied_action = learner.get_current_action();
       applied_state  = learner.get_current_state();
       sts_add("policy=%u, action=%u, state=%u",applied_policy,applied_action,applied_state);
       sts_msg("%3.1f %3.1f %3.1f ", modes.at(applied_action).head
                                   , modes.at(applied_action).body
                                   , modes.at(applied_action).tail );


       remote.append("MDI00=%f\nMDI01=%f\nMDI02=%f\n", modes.at(applied_action).head
                                                     , modes.at(applied_action).body
                                                     , modes.at(applied_action).tail );
    }

};

class Application : public Application_Base
{
public:
    Application(int argc, char** argv, Event_Manager& em)
    : Application_Base(argc, argv, em, "Flatcat UDP Learning", 1024, 1024)
    , settings(argc, argv)
    , midi(1, /*verbose=*/true)
    , remote()
    , robot()
    , actions(remote)
    , reward()
    , gmes_joint_group( robot.get_joints()
                      , 64     // settings.number_of_experts
                      , 100.0  // settings.joint_gmes_learning_rate
                      , 0.001  // settings.local_learning_rate
                      , 1      // settings.experience_size
                      )
    , super_layer( 16
                , gmes_joint_group.get_activations()
                , actions
                , reward.get_number_of_policies()
                , /*initialQ=*/ .1
                , 10.0  // settings.joint_gmes_learning_rate
                , 0.0005 // settings.local_learning_rate
                , 1    // settings.experience_size
                )
    /* reinforcement learning */
    , epsilon_greedy(super_layer.payload, actions, settings.epsilon_exploration)
    , agent(super_layer.payload, reward, epsilon_greedy, actions.get_number_of_actions(), settings.sarsa_learning_rates)
    , policy_selector(agent, reward.get_number_of_policies(), /*random_policy_mode = */true, settings.trial_time_s)
    , eigenzeit(super_layer.gmes, settings.eigenzeit_steps)

    /* utilities */
    , watch()
    , views(4)

    /* graphics */
    , gfx_robot(robot, robot.control.user_target_position)
    , gfx_gmes_joint_group(gmes_joint_group)
    , gfx_super_gmes(super_layer.gmes)
    , gfx_agent(agent)
    , gfx_policy_selector(policy_selector)
    , gfx_super_payload(super_layer.payload, actions)
    {
        reward.add_intrinsic_rewards(gmes_joint_group, super_layer);


        if (std::experimental::filesystem::exists(settings.save_folder)) {
            if (settings.clear_state) {
                wrn_msg("Overriding state: %s", settings.save_state_name.c_str());
                save(settings.save_folder);

            } else
                load(settings.save_folder);
        }
        else {
            basic::make_directory(settings.save_folder.c_str());
            save(settings.save_folder);
        }

        do_pause.disable(); // do not start in pause mode
        fast_forward.enable(); /**TODO why is that needed... cycle time seems to be 3 times as fast*/

        assert(robot.control.user_target_position.size() == supreme::constants::FlatcatMidiMap.size());
        remote.open_connection(network::hostname_to_ip("flatcat2.local").c_str()/*"192.168.1.106"*/, 7332);
        remote.send("HELLO\n");
        gfx_super_gmes.set_position(0.5,-0.5).set_scale(1.0);


    }

    bool loop();
    void finish();

    void draw(const pref&) const;

    void user_callback_key_pressed (SDL_Keysym const& key);
  //void user_callback_key_released(SDL_Keysym const& key);

    void user_callback_joystick_button_pressed (SDL_JoyButtonEvent const& e);
    void user_callback_joystick_button_released(SDL_JoyButtonEvent const& e);
    void user_callback_joystick_motion_axis    (SDL_JoyAxisEvent   const& e);
  //void user_callback_joystick_motion_hat     (SDL_JoyHatEvent    const& e);

    void send_control_mode(supreme::ControlMode_t mode) { remote.send("CTL=%u\n", mode); }
    void send_parameter_id(unsigned id) { remote.send("PAR=%u\n", id); }

    void save(std::string f) {
        sts_msg("Saving state: %s", settings.save_state_name.c_str());
        gmes_joint_group.save(f);
        super_layer.save(f);
    }

    void load(std::string f) {
        sts_msg("Loading state: %s", settings.save_state_name.c_str());
        gmes_joint_group.load(f);
        super_layer.load(f);

        gfx_gmes_joint_group.update_on_load();
    }

private:
    supreme::FlatcatSettings             settings;
    MidiIn                               midi;
    network::Socket_Client               remote;

    supreme::FlatcatUDPRobot             robot;
    RemoteRobotActions                   actions;
    flatcat_reward_space                 reward;

    /* state space learning */
    learning::GMES_Joint_Group           gmes_joint_group;
    learning::GMES_Layer                 super_layer;

    /* reinforcement learning */
    learning::Epsilon_Greedy             epsilon_greedy;
    SARSA                                agent;
    Policy_Selector                      policy_selector;
    learning::Eigenzeit                  eigenzeit;

    /* utilities */
    Stopwatch                            watch;
    View_Manager                         views;

    /* graphics */
    supreme::FlatcatGraphics             gfx_robot;
    learning::GMES_Joint_Group_Graphics  gfx_gmes_joint_group;
    Force_Field                          gfx_super_gmes;
    SARSA_Graphics                       gfx_agent;
    Policy_Selector_Graphics             gfx_policy_selector;
    State_Payload_Graphics               gfx_super_payload;

    double                     j_val[32]; // joystick
    bool                       j_enable = false;
    bool                       j_axis_changed = false;

    bool connection_status = false;

    float time_passed_ms = 0.f;
};



#endif /* FLATCAT_CONTROL_UDP_HPP */
