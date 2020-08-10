
#include "./event_manager.h"

extern GlobalFlag do_quit;
extern GlobalFlag do_pause;
extern GlobalFlag fast_forward;
extern GlobalFlag draw_grid;
extern GlobalFlag screenshot;

extern Visuals screen;
extern float t_delay_ms;

void
quit()
{
    static bool q = false;
    if (!q) // prevent second call
    {
        q = true;
        sts_msg("Sending signal to exit.");
        do_quit.enable();
    }
}

void
on_button_pressed_ESCAPE(void)
{
    sts_msg("Pressed key ESC. Quit.");
    quit();
}

void
on_button_pressed_SPACE(void)
{
    do_pause.toggle();
    if (do_pause.status()) sts_msg("Paused.");
    else sts_msg("Continuing.");
}

void
on_button_pressed_BACKSPACE(void)
{
    fast_forward.toggle();
    if (fast_forward.status()) sts_msg("Fast mode enabled.");
    else sts_msg("Fast mode disabled.");
}

void
on_button_pressed_RETURN(void)
{
    dbg_msg("Pressed key RETURN.");
}

void
on_button_pressed_G(void)
{
    sts_msg("Toggled drawing grid.");
    draw_grid.toggle();
}

void
on_button_pressed_S(void)
{
    screenshot.enable();
}

void
on_button_pressed_F(void)
{
    screen.show_fps = not screen.show_fps;
    sts_msg("Statistics: %s", screen.show_fps ? "ON":"OFF");
}

void
on_button_pressed_PAGEUP(void)
{
    if (!screen.rotate_view)
        screen.rotate_view = true;
    else if (screen.rot_factor < 20)
        ++screen.rot_factor;
    sts_msg("Rotation factor: %d", screen.rot_factor);
}

void
on_button_pressed_PAGEDOWN(void)
{
    if (!screen.rotate_view)
        screen.rotate_view = true;
    else if (screen.rot_factor > -20)
        --screen.rot_factor;
    sts_msg("Rotation factor: %d", screen.rot_factor);
}

void
on_button_pressed_PLUS(void)
{
    if (t_delay_ms > 2) t_delay_ms *= 0.5;
    else fast_forward.enable();
    sts_msg("Delay: %.2f ms (ff %d)", t_delay_ms, fast_forward.status());
}

void
on_button_pressed_MINUS(void)
{
    if (fast_forward.status())
        fast_forward.disable();
    else if (t_delay_ms < 1000) t_delay_ms *= 2.0;
    sts_msg("Delay: %.2f ms (ff %d)", t_delay_ms, fast_forward.status());
}
void
Event_Manager::on_left_mouse_button_pressed(void)
{
    mouse_button_left.clicked = true;
    mouse_button_left.position_x = event.button.x;
    mouse_button_left.position_y = event.button.y;
    screen.rotate_view = false;
}

void
Event_Manager::on_left_mouse_button_released(void)
{
    mouse_button_left.clicked = false;

    screen.x_angle += screen.x_angle_disp;
    screen.y_angle += screen.y_angle_disp;
    screen.x_angle_disp = 0.0f;
    screen.y_angle_disp = 0.0f;

    /* snap to grid */
    screen.x_angle = screen.snap*round(screen.x_angle/screen.snap);
    screen.y_angle = screen.snap*round(screen.y_angle/screen.snap);

    sts_msg("new pos: % 7.2f, % 7.2f", screen.x_angle, screen.y_angle);
}

void
Event_Manager::on_right_mouse_button_pressed(void)
{
    mouse_button_right.clicked = true;
    mouse_button_right.position_x = event.button.x;
    mouse_button_right.position_y = event.button.y;
}

void
Event_Manager::on_right_mouse_button_released(void)
{
    mouse_button_right.clicked = false;
    screen.x_position += screen.x_position_disp;
    screen.y_position += screen.y_position_disp;
    screen.x_position_disp = 0.0f;
    screen.y_position_disp = 0.0f;
}

void
Event_Manager::on_middle_mouse_button_pressed(void)
{
    mouse_button_middle.clicked = true;
    mouse_button_middle.position_x = event.button.x;
    mouse_button_middle.position_y = event.button.y;
    screen.reset();
}

void
Event_Manager::on_middle_mouse_button_released(void)
{
    mouse_button_middle.clicked = false;
}

void
Event_Manager::on_mouse_wheel_up(void)
{
    ++mouse_wheel_position;
    if (screen.zdist * visuals_defaults::zoom_factor > visuals_defaults::gl_zNear)
        screen.zdist *= visuals_defaults::zoom_factor;
}

void
Event_Manager::on_mouse_wheel_down(void)
{
    --mouse_wheel_position;
    if (screen.zdist < 8)
        screen.zdist /= visuals_defaults::zoom_factor;
}

void
Event_Manager::handle_mouse_wheel(SDL_MouseWheelEvent const& wheel)
{
    if (wheel.y > 0)
        on_mouse_wheel_up();
    else if (wheel.y < 0)
        on_mouse_wheel_down();
}

void
Event_Manager::handle_key_pressed(SDL_Keysym const& key)
{
    switch (key.sym)
    {
        case SDLK_ESCAPE:    on_button_pressed_ESCAPE();    break;
        case SDLK_SPACE:     on_button_pressed_SPACE();     break;
        case SDLK_BACKSPACE: on_button_pressed_BACKSPACE(); break;
        case SDLK_RETURN:    on_button_pressed_RETURN();    break;
        /*
        case SDLK_TAB:       break;
        case SDLK_UP:        break;
        case SDLK_F1:        break; */
        case SDLK_g:         on_button_pressed_G();         break;
        case SDLK_f:         on_button_pressed_F();         break;
        case SDLK_s:         on_button_pressed_S();         break;
        /*
        case SDLK_0:         break;
        case SDLK_2:         break;
        */
        case SDLK_PLUS:      on_button_pressed_PLUS();      break;
        case SDLK_MINUS:     on_button_pressed_MINUS();     break;

        case SDLK_PAGEUP:    on_button_pressed_PAGEUP();    break;
        case SDLK_PAGEDOWN:  on_button_pressed_PAGEDOWN();  break;

        default: break;
    }

    if (nullptr != user_callback.key_pressed) user_callback.key_pressed(key);
}

void
Event_Manager::handle_key_released(SDL_Keysym const& key)
{
    switch (key.sym)
    {
        /*case SDLK_ESCAPE:    break;*/
        /*case SDLK_SPACE:     break;*/
        /*case SDLK_BACKSPACE: break;*/
        /*
        case SDLK_TAB:       break;
        case SDLK_UP:        break;
        case SDLK_F1:        break;
        case SDLK_1:         break;
        case SDLK_0:         break;
        case SDLK_2:         break;
        case SDLK_RETURN:    break;
        case SDLK_PLUS:      break;
        case SDLK_MINUS:     break;

        case SDLK_PAGEUP:    break;
        case SDLK_PAGEDOWN:  break;
        */
        default: break;
    }
    if (nullptr != user_callback.key_released) user_callback.key_released(key);
}

void
Event_Manager::handle_mouse_button_pressed(SDL_MouseButtonEvent const& m)
{
    switch (m.button)
    {
        case SDL_BUTTON_LEFT   : on_left_mouse_button_pressed  (); break;
        case SDL_BUTTON_RIGHT  : on_right_mouse_button_pressed (); break;
        case SDL_BUTTON_MIDDLE : on_middle_mouse_button_pressed(); break;
        default: break;
    }
}

void
Event_Manager::handle_mouse_button_released(SDL_MouseButtonEvent const& m)
{
    switch (m.button)
    {
        case SDL_BUTTON_LEFT   : on_left_mouse_button_released  (); break;
        case SDL_BUTTON_RIGHT  : on_right_mouse_button_released (); break;
        case SDL_BUTTON_MIDDLE : on_middle_mouse_button_released(); break;
        default: break;
    }
}

void
Event_Manager::handle_mouse_motion(SDL_MouseMotionEvent const& m)
{
    mouse_position_x = m.x;
    mouse_position_y = m.y;

    if (mouse_button_left.clicked) {
        screen.mdx = (float) (mouse_button_left.position_x - mouse_position_x) / screen.window_size_x;
        screen.mdy = (float) (mouse_button_left.position_y - mouse_position_y) / screen.window_size_y;
        screen.x_angle_disp = -90 * screen.mdx;
        screen.y_angle_disp = -90 * screen.mdy;
        //dbg_msg("(%1.2f,%1.2f)", screen.mdx, screen.mdy);
    }

    if (mouse_button_right.clicked) {
        screen.x_position_disp = -(float) (mouse_button_right.position_x - mouse_position_x) * 2 / std::min(screen.window_size_y, screen.window_size_x);
        screen.y_position_disp = +(float) (mouse_button_right.position_y - mouse_position_y) * 2 / std::min(screen.window_size_y, screen.window_size_x);
    }
}

void
Event_Manager::handle_joystick_motion_axis(SDL_JoyAxisEvent const& j)
{
    switch(j.axis)
    {
        case 0: joystick.x0 = (float) (+j.value / 32767.0); /*dbg_msg("jx0");*/ break;
        case 1: joystick.y0 = (float) (-j.value / 32767.0); /*dbg_msg("jy0");*/ break;
        case 2: joystick.x1 = (float) (+j.value / 32767.0); /*dbg_msg("jx1");*/ break;
        case 3: joystick.y1 = (float) (-j.value / 32767.0); /*dbg_msg("jy1");*/ break;
        default: dbg_msg("unknown axis %u", j.axis); break;
    }
    if (nullptr != user_callback.joystick_motion_axis) user_callback.joystick_motion_axis(j);
}

void
Event_Manager::handle_joystick_motion_hat(SDL_JoyHatEvent const& j)
{
    switch(j.value)
    {
         case SDL_HAT_LEFTUP    : /*dbg_msg("L-U");*/ break;
         case SDL_HAT_UP        : /*dbg_msg("  U");*/ break;
         case SDL_HAT_RIGHTUP   : /*dbg_msg("R-U");*/ break;
         case SDL_HAT_LEFT      : /*dbg_msg("L  ");*/ break;
         case SDL_HAT_CENTERED  : /*dbg_msg(" C ");*/ break;
         case SDL_HAT_RIGHT     : /*dbg_msg("R  ");*/ break;
         case SDL_HAT_LEFTDOWN  : /*dbg_msg("L-D");*/ break;
         case SDL_HAT_DOWN      : /*dbg_msg("  D");*/ break;
         case SDL_HAT_RIGHTDOWN : /*dbg_msg("R-U");*/ break;
         default: dbg_msg("unknown hat %u", j.hat); break;
    }
    if (nullptr != user_callback.joystick_motion_hat) user_callback.joystick_motion_hat(j);
}

void
Event_Manager::handle_joystick_button_pressed(SDL_JoyButtonEvent const& joystick)
{
    switch (joystick.button) //IDEA: consider making names for rumblepad
    {
        case  0: dbg_msg("Button 0 pressed"); break;
        case  1: dbg_msg("Button 1 pressed"); break;
        case  2: dbg_msg("Button 2 pressed"); break;
        case  3: dbg_msg("Button 3 pressed"); break;
        case  4: dbg_msg("Button 4 pressed"); break;
        case  5: dbg_msg("Button 5 pressed"); break;
        case  6: dbg_msg("Button 6 pressed"); break;
        case  7: dbg_msg("Button 7 pressed"); break;
        case  8: dbg_msg("Button 8 pressed. Escape.");       quit();            break;
        case  9: dbg_msg("Button 9 pressed. Toggle Pause."); do_pause.toggle(); break;
        case 10: dbg_msg("Button A pressed"); break;
        case 11: dbg_msg("Button B pressed"); break;
        default: dbg_msg("unknown button %u", joystick.button); break;
    }

    if (nullptr != user_callback.joystick_button_pressed) user_callback.joystick_button_pressed(joystick);
}

void
Event_Manager::handle_joystick_button_released(SDL_JoyButtonEvent const& joystick)
{
    switch (joystick.button)
    {
        case  0: break;
        case  1: break;
        case  2: break;
        case  3: break;
        case  4: break;
        case  5: break;
        case  6: break;
        case  7: break;
        case  8: break;
        case  9: break;
        case 10: break;
        case 11: break;
        default: break;
    }
    if (nullptr != user_callback.joystick_button_released) user_callback.joystick_button_released(joystick);
}

void
handle_application_quit(void)
{
    sts_msg("handle application quit");
    quit();
}

void
Event_Manager::process_events(void)
{
    /* Grab all the events off the queue. */
    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
            /* all possible events */
            case SDL_KEYDOWN         : handle_key_pressed             (event.key.keysym); break;
            case SDL_KEYUP           : handle_key_released            (event.key.keysym); break;
            case SDL_MOUSEBUTTONDOWN : handle_mouse_button_pressed    (event.button    ); break;
            case SDL_MOUSEBUTTONUP   : handle_mouse_button_released   (event.button    ); break;
            case SDL_MOUSEMOTION     : handle_mouse_motion            (event.motion    ); break;
            case SDL_MOUSEWHEEL      : handle_mouse_wheel             (event.wheel     ); break;
            case SDL_JOYAXISMOTION   : handle_joystick_motion_axis    (event.jaxis     ); break;
            case SDL_JOYHATMOTION    : handle_joystick_motion_hat     (event.jhat      ); break;
            case SDL_JOYBUTTONDOWN   : handle_joystick_button_pressed (event.jbutton   ); break;
            case SDL_JOYBUTTONUP     : handle_joystick_button_released(event.jbutton   ); break;
            case SDL_QUIT            : handle_application_quit();                         break;
            default:
                /*assertion(false, "unhandled SDL event: %u", event.type); */ break;
        }
    } /* while */
}

void
Event_Manager::reg_usr_cb_key_pressed(Keysym_t callback_function)
{
    if (callback_function == nullptr)
        wrn_msg("Could not register user callback function for 'keyboard pressed'.");
    else
        user_callback.key_pressed = callback_function;
}

void
Event_Manager::reg_usr_cb_key_released(Keysym_t func)
{
    if (func == nullptr)
        wrn_msg("Could not register user callback function for 'keyboard released'.");
    else
        user_callback.key_released = func;
}

void
Event_Manager::reg_usr_cb_joystick_button_pressed(Joybutton_t func)
{
    if (func == nullptr)
        wrn_msg("Could not register user callback function for joystick button pressed.");
    else
        user_callback.joystick_button_pressed = func;
}

void
Event_Manager::reg_usr_cb_joystick_button_released(Joybutton_t func)
{
    if (func == nullptr)
        wrn_msg("Could not register user callback function for joystick button released.");
    else
        user_callback.joystick_button_released = func;
}

void
Event_Manager::reg_usr_cb_joystick_motion_axis(Joyaxis_t func)
{
    if (func == nullptr)
        wrn_msg("Could not register user callback function for joystick motion axis.");
    else
        user_callback.joystick_motion_axis = func;
}

void
Event_Manager::reg_usr_cb_joystick_motion_hat(Joyhat_t func)
{
    if (func == nullptr)
        wrn_msg("Could not register user callback function for joystick motion hat.");
    else
       user_callback.joystick_motion_hat = func;
}

/*void
Event_Manager::register_user_callback_mouse(callback_type func)
{
    if (func == nullptr)
        wrn_msg("Could not register user callback function for mouse.");
    else
        user_callback_mouse = func;
}*/
