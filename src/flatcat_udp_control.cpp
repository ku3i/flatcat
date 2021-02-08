#include "./flatcat_udp_control.hpp"

#include <common/modules.h>
#include <common/setup.h>

DEFINE_GLOBALS()


void
Application::draw(const pref& p) const {

    flatcat_gfx.draw(p);

    auto const& ctrl = flatcat_UDP.control;
   /**TODO:
    auto const& ts = flatcat_UDP.get_spinalcord_timing();
    glprintf(-1.f, 0.97f, 0.f, .025f, "(stat) <%x> sync_err=%u (dropped=%u) [trans_error=%u] %4.1f %5.1f\n"
                                    , ts.collected_ids
                                    , ts.syncfaults
                                    , ts.board_dropouts
                                    , ts.transparent_errors
                                    , ts.mean, ts.maxv );
    */
    glprintf(+.4f, 0.97f, 0.f, .025f, "I = %4.2f M = %4.2f A = %4.2f %s\n", ctrl.inputgain
                                                                          , ctrl.modulate
                                                                          , ctrl.amplitude
                                                                          , (ctrl.enabled) ? "EN" : "--");
    glprintf(+.7f, 0.94f, 0.f, .025f, "MODE = %s", supreme::constants::mode_str[(unsigned)ctrl.mode]);

}

bool
Application::loop(void)
{
    sts_msg("%05.2f ms %llu", watch.get_time_passed_us()/1000.0, cycles);

    /* get controller status from midi buttons and pots */
    if (midi.has_changed(33)) remote.append("POS=%u\n", (unsigned) midi[33]);
    if (midi.has_changed(23)) remote.append("ENA=%u\n", (unsigned) midi[23]);
    if (midi.has_changed(22)) remote.append("AMP=%f\n", clip(midi.get(22), 0.f, 1.f));
    if (midi.has_changed(21)) remote.append("MOD=%f\n", clip(midi.get(21), 0.f, 1.f));
    if (midi.has_changed(20)) remote.append("ING=%f\n", clip(midi.get(20), 0.f, 1.f));

    auto& ctrl = flatcat_UDP.control;

    for (std::size_t i = 0; i < ctrl.user_target_position.size(); ++i) {
        auto const& idx = supreme::constants::FlatcatMidiMap[i];
        if (midi.has_changed(idx)) {
            ctrl.user_target_position[i] = midi.get(idx);
            remote.append("MDI%02u=%f\n", i, clip(ctrl.user_target_position[i], -1.f, 1.f));
        }
    }

    flatcat_UDP.execute_cycle();   /* recv UDP raw sensor signals */

    /* add samples to plot */
    flatcat_gfx.update_samples();

    cycles++;

    remote.flush();
    midi.fetch();
    return true;
}

void
Application::user_callback_key_pressed(const SDL_Keysym& key)
{

    switch (key.sym)
    {
        case SDLK_1 : send_control_mode(supreme::ControlMode_t::none    ); break;
        case SDLK_2 : send_control_mode(supreme::ControlMode_t::position); break;
        case SDLK_3 : send_control_mode(supreme::ControlMode_t::csl_hold); break;
        case SDLK_4 : send_control_mode(supreme::ControlMode_t::so2_osc ); break;
        case SDLK_5 : send_control_mode(supreme::ControlMode_t::walking ); break;

        case SDLK_q : send_parameter_id(0); break; // stop
        case SDLK_w : send_parameter_id(1); break; // walk

        case SDLK_r : remote.send("RST\n"); break;

        case SDLK_c : remote.send("CEN\n"); break; // calibration enable
        case SDLK_v : remote.send("CID\n"); break; // calibration index
        case SDLK_b : remote.send("CAB\n"); break; // calibration abort

        default:
            break;
    }
}


void Application::user_callback_joystick_button_pressed(SDL_JoyButtonEvent const& joystick)
{
    switch (joystick.button)
    {
    case 0: send_parameter_id(1); break;
    case 1: send_parameter_id(2); break;
    case 2: send_parameter_id(3); break;
    case 3: send_parameter_id(4); break;
    default: break;
    }

}

void Application::user_callback_joystick_button_released(SDL_JoyButtonEvent const& joystick)
{
    switch (joystick.button)
    {
    case 0: case 1: case 2: case 3: send_parameter_id(0); break;
    default: break;
    }

}

/*void Application::user_callback_joystick_motion_axis(SDL_JoyAxisEvent const& joystick)
{
    switch (joystick.axis) {
    case 3: break;
    default: break;
    }
}*/


void
Application::finish(void)
{
    sts_msg("Finished shutting down all subsystems.");
    quit();
    remote.send("EXIT\n");
}


APPLICATION_MAIN()
