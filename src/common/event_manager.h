#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keyboard.h>
#include <SDL2/SDL_keycode.h>
#include <algorithm>
#include <functional>
#include <common/log_messages.h>
#include <common/globalflag.h>
#include <common/visuals.h>

struct Mouse_Click_Event
{
    bool clicked;
    int position_x;
    int position_y;
};

struct Joystick
{
    float x0, y0;
    float x1, y1;
};

class Event_Manager
{

public:
    Event_Manager()
    : event()
    , joystick()
    , user_callback()
    , mouse_button_left()
    , mouse_button_right()
    , mouse_button_middle()
    , mouse_position_x()
    , mouse_position_y()
    , mouse_wheel_position()
    , mouse_wheel_position_last()
    {}

    void process_events(void);

    typedef std::function<void(SDL_Keysym         const& key     )> Keysym_t;
    typedef std::function<void(SDL_JoyButtonEvent const& joystick)> Joybutton_t;
    typedef std::function<void(SDL_JoyAxisEvent   const& joystick)> Joyaxis_t;
    typedef std::function<void(SDL_JoyHatEvent    const& joystick)> Joyhat_t;


    void reg_usr_cb_key_pressed             (Keysym_t    cb);
    void reg_usr_cb_key_released            (Keysym_t    cb);
    void reg_usr_cb_joystick_button_pressed (Joybutton_t cb);
    void reg_usr_cb_joystick_button_released(Joybutton_t cb);
    void reg_usr_cb_joystick_motion_axis    (Joyaxis_t   cb);
    void reg_usr_cb_joystick_motion_hat     (Joyhat_t    cb);
    //void reg_usr_cb_mouse   (callback_type callback_function);

    Joystick const& get_joystick(void) const { return joystick; }

    int  get_mouse_wheel_position  (void) const { return mouse_wheel_position; }

private:
    void handle_key_pressed (SDL_Keysym const& key);
    void handle_key_released(SDL_Keysym const& key);

    void handle_mouse_motion         (SDL_MouseMotionEvent const& motion);
    void handle_mouse_wheel          (SDL_MouseWheelEvent  const& wheel );
    void handle_mouse_button_pressed (SDL_MouseButtonEvent const& button);
    void handle_mouse_button_released(SDL_MouseButtonEvent const& button);

    void handle_joystick_motion_axis    (SDL_JoyAxisEvent   const& joystick);
    void handle_joystick_motion_hat     (SDL_JoyHatEvent    const& joystick);
    void handle_joystick_button_pressed (SDL_JoyButtonEvent const& joystick);
    void handle_joystick_button_released(SDL_JoyButtonEvent const& joystick);

    /* private member callbacks */
    void on_left_mouse_button_pressed  (void);
    void on_right_mouse_button_pressed (void);
    void on_middle_mouse_button_pressed(void);

    void on_left_mouse_button_released  (void);
    void on_right_mouse_button_released (void);
    void on_middle_mouse_button_released(void);

    void on_mouse_wheel_up  (void);
    void on_mouse_wheel_down(void);


    SDL_Event event;
    Joystick joystick;

    /* callbacks */
    struct UserCallbacks {
        Keysym_t    key_pressed              = nullptr; // think about a general callback register function for all events
        Keysym_t    key_released             = nullptr;
        Joybutton_t joystick_button_pressed  = nullptr;
        Joybutton_t joystick_button_released = nullptr;
        Joyaxis_t   joystick_motion_axis     = nullptr;
        Joyhat_t    joystick_motion_hat      = nullptr;
    } user_callback;

    //callback_type user_callback_mouse;

    Mouse_Click_Event mouse_button_left;
    Mouse_Click_Event mouse_button_right;
    Mouse_Click_Event mouse_button_middle;

    int mouse_position_x;
    int mouse_position_y;
    int mouse_wheel_position;
    int mouse_wheel_position_last;

};

void on_button_pressed_ESCAPE   (void);
void on_button_pressed_SPACE    (void);
void on_button_pressed_BACKSPACE(void);
void on_button_pressed_RETURN   (void);
void quit(void);


#endif /* EVENT_MANAGER_H */
