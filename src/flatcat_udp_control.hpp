#ifndef FLATCAT_CONTROL_UDP_HPP
#define FLATCAT_CONTROL_UDP_HPP

#include <array>

#include <common/application_base.h>
#include <common/event_manager.h>
#include <common/log_messages.h>
#include <common/basic.h>
#include <common/globalflag.h>
#include <common/modules.h>
#include <common/udp.hpp>
#include <common/stopwatch.h>
#include <common/socket_client.h>

#include <draw/draw.h>
#include <midi/midi_in.h>

#include <flatcat_graphics.hpp>
#include <flatcat_control.hpp>
#include <robots/accel.h>


/**TODOs:
    + think about auto-calibration method
*/


extern GlobalFlag do_pause;
extern GlobalFlag fast_forward;

namespace supreme {

namespace constants {

    /* maps the motor ids to channel numbers of korg kontrol */
    const std::array<uint8_t, num_joints> FlatcatMidiMap = { 17, 16 };
}



class FlatcatUDPRobot {
public:
    typedef std::array<float, constants::num_joints> TargetPosition_t;

    network::UDPReceiver<888> receiver;

    uint16_t sync   = 0;
    uint64_t cycles = 0;
    uint8_t  chksum = 0;

    typedef std::vector<supreme::interface_data> Motordata_t;
    Motordata_t motors;

    //robots::Accelvector_t accels; /**TODO*/

    //typedef supreme::SpinalCord::TimingStats timestats_t;
    //TODO: timestats_t timing;

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
    , motors(2 /**TODO*/)
    //, accels(1)/**TODO*/
    //, timing()
    , control()
    {


    }

    Motordata_t const& get_motors(void) const { return motors; }
    //const robots::Accelvector_t& get_accels(void) const { return accels; }

    //TODO SpinalCord::TimingStats const& get_spinalcord_timing(void) const { return timing; }

    void execute_cycle(void) {
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
//TODO                n = network::getfrom(m.id               , msg, n);
                n = network::getfrom(m.position         , msg, n);
                n = network::getfrom(m.last_p           , msg, n);
                n = network::getfrom(m.velocity         , msg, n);
                n = network::getfrom(m.current          , msg, n);
                n = network::getfrom(m.voltage_supply   , msg, n);
                n = network::getfrom(m.voltage_backemf  , msg, n);
//TODO                n = network::getfrom(m.last_output      , msg, n);
                n = network::getfrom(m.temperature      , msg, n);
//TODO                n = network::getfrom(m.is_connected     , msg, n);
                n = network::getfrom(m.acceleration.x   , msg, n);
                n = network::getfrom(m.acceleration.y   , msg, n);
                n = network::getfrom(m.acceleration.z   , msg, n);
//TODO                n = network::getfrom(m.connection_losses, msg, n);
//TODO                n = network::getfrom(m.target_voltage   , msg, n);
 //TODO               n = network::getfrom(m.dir              , msg, n);
 //TODO               n = network::getfrom(m.scale            , msg, n);
 //TODO               n = network::getfrom(m.offset           , msg, n);
            } /* for each motor */

            /* timing */
            /**TODO
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
        }

    }

};

} /* namespace supreme */


class Application : public Application_Base
{
public:
    Application(int argc, char** argv, Event_Manager& em)
    : Application_Base(argc, argv, em, "Flatcat UDP Terminal", 1024, 1024)
    , midi(1, /*verbose=*/false)
    //, settings(argc, argv)
    , remote()
    , flatcat_UDP()
    , flatcat_gfx(flatcat_UDP, flatcat_UDP.control.user_target_position)
    , watch()
    {
        do_pause.disable(); // do not start in pause mode
        fast_forward.enable();

       assert(flatcat_UDP.control.user_target_position.size() == supreme::constants::FlatcatMidiMap.size());
       remote.open_connection(network::hostname_to_ip("flatcat.local").c_str()/*"192.168.1.106"*/, 7332);
       remote.send("HELLO\n");
    }

    bool loop();
    void finish();

    void draw(const pref&) const;

    void user_callback_key_pressed (SDL_Keysym const& key);
    //void user_callback_key_released(SDL_Keysym const& key);

    void user_callback_joystick_button_pressed (SDL_JoyButtonEvent const& e);
    void user_callback_joystick_button_released(SDL_JoyButtonEvent const& e);
    //void user_callback_joystick_motion_axis    (SDL_JoyAxisEvent   const& e);
    //void user_callback_joystick_motion_hat     (SDL_JoyHatEvent    const& e);




    void send_control_mode(supreme::ControlMode_t mode) { remote.send("CTL=%u\n", mode); }
    void send_parameter_id(unsigned id) { remote.send("PAR=%u\n", id); }

private:
    MidiIn                     midi;

    network::Socket_Client     remote;

    supreme::FlatcatUDPRobot    flatcat_UDP;
    supreme::FlatcatGraphics    flatcat_gfx;

    Stopwatch                  watch;
};



#endif /* FLATCAT_CONTROL_UDP_HPP */
