#include "./flatcat_udp_learning.hpp"

#include <common/modules.h>
#include <common/setup.h>

DEFINE_GLOBALS()


void
Application::draw(const pref& p) const
{
    switch(views.get()) {
        case 0:
            gfx_robot           .drawing(p);
            break;
        case 1:
            gfx_gmes_joint_group.drawing(p);
            gfx_super_gmes      .drawing(p);
            break;
        case 2:
            gfx_policy_selector .drawing(p);
            gfx_agent           .drawing(p);
            break;
        case 3:
            gfx_super_payload   .drawing(p);
            break;
        default: break;
    }

    set_color(colors::white);

    auto const& ctrl = robot.control;
   /**TODO:
    auto const& ts = robot.get_spinalcord_timing();
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
    glprintf(+.7f, .94f, .0f, .025f, "MODE = %s", supreme::constants::mode_str[(unsigned)ctrl.mode]);
    glprintf(+.7f, .90f, .0f, .025f, "CONN = %s", connection_status? "OK":"NO");


    glprintf(+.4f, -0.97f, .0f, .025f, "%05.2f ms %llu", time_passed_ms, cycles);
}

bool
Application::loop(void)
{


    /* get controller status from midi buttons and pots */
    if (midi.has_changed(33)) remote.append("POS=%u\n", (unsigned) midi[33]);
    if (midi.has_changed(23)) remote.append("ENA=%u\n", (unsigned) midi[23]);
    if (midi.has_changed(22)) remote.append("AMP=%f\n", clip(midi.get(22), 0.f, 1.f));
    if (midi.has_changed(21)) remote.append("MOD=%f\n", clip(midi.get(21), 0.f, 1.f));
    if (midi.has_changed(20)) remote.append("ING=%f\n", clip(midi.get(20), 0.f, 1.f));

    auto& ctrl = robot.control;

    for (std::size_t i = 0; i < ctrl.user_target_position.size(); ++i) {
        auto const& idx = supreme::constants::FlatcatMidiMap[i];
        if (midi.has_changed(idx)) {
            ctrl.user_target_position[i] = midi.get(idx);
            remote.append("MDI%02u=%f\n", i, clip(ctrl.user_target_position[i], -1.f, 1.f));
        }
        if (j_axis_changed) {
            ctrl.user_target_position[i] = j_val[i];
            remote.append("MDI%02u=%f\n", i, clip(ctrl.user_target_position[i], -1.f, 1.f));
        }
    }
    j_axis_changed = false;


    connection_status = robot.execute_cycle();
    gmes_joint_group         .execute_cycle();
    super_layer              .execute_cycle();

    eigenzeit                .execute_cycle();
    reward                   .execute_cycle();
    policy_selector          .execute_cycle();

    if (eigenzeit.has_progressed()) {
        agent.execute_cycle(super_layer.gmes.get_winner());
        actions.execute_cycle(agent); //note: must be processed after agent's step.
    }

    /* graphics */
    gfx_robot           .update_samples();
    gfx_gmes_joint_group.execute_cycle(cycles);
    gfx_super_gmes      .execute_cycle(cycles);
    gfx_agent           .execute_cycle(cycles, eigenzeit.has_progressed(), policy_selector.has_trial_ended());

    if (eigenzeit.has_progressed())
        reward.clear_aggregations();

    remote.flush();
    midi.fetch();

    cycles++;
    time_passed_ms = watch.get_time_passed_us()/1000.0;

    if (cycles % (settings.save_cycles_s*100) == 0)
        save(settings.save_folder); // save each minute

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
        case SDLK_5 : send_control_mode(supreme::ControlMode_t::behavior); break;

        case SDLK_r : remote.send("RST\n"); break;

        case SDLK_c : remote.send("CEN\n"); break; // calibration enable
        case SDLK_v : remote.send("CID\n"); break; // calibration index
        case SDLK_b : remote.send("CAB\n"); break; // calibration abort

        default:
            break;
    }

    views.key_pressed(key);
}


void Application::user_callback_joystick_button_pressed(SDL_JoyButtonEvent const& joystick)
{
    switch (joystick.button)
    {
    case 0: send_parameter_id(1); break;
    case 1: send_parameter_id(2); break;
    case 2: send_parameter_id(3); break;
    case 3: send_parameter_id(4); break;
    case 4: /* toggle enable */ j_enable = 1 - j_enable; break;
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

void Application::user_callback_joystick_motion_axis(SDL_JoyAxisEvent const& joystick)
{
    switch (joystick.axis) {
    case 3: { j_val[3] = -clip(em.get_joystick().y1, -1.,1.); } j_axis_changed = true; break;
    case 2: { j_val[2] = -clip(em.get_joystick().x1, -1.,1.); } j_axis_changed = true; break;
    case 1: { j_val[1] = +clip(em.get_joystick().y0, -1.,1.); } j_axis_changed = true; break;
    case 0: { j_val[0] = +clip(em.get_joystick().x0, -1.,1.); } j_axis_changed = true; break;
    default: break;
    }
}


void
Application::finish(void)
{
    sts_msg("Finished shutting down all subsystems.");
    quit();
    remote.send("EXIT\n");
}


APPLICATION_MAIN()
