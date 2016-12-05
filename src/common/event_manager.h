#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keyboard.h>
#include <SDL2/SDL_keycode.h>
#include <algorithm>
#include <functional>
#include <common/log_messages.h>

struct Mouse_Click_Event
{
    bool clicked;
    int position_x;
    int position_y;
};

struct Joystick
{
    double x;
    double y;
};

class Event_Manager
{

public:
    Event_Manager()
    : event()
    , joystick0()
    , joystick1()
    , user_callback_key_pressed()
    , user_callback_key_released()
    , user_callback_joystick()
    , user_callback_mouse   ()
    , mouse_button_left()
    , mouse_button_right()
    , mouse_button_middle()
    , mouse_position_x()
    , mouse_position_y()
    , mouse_wheel_position()
    , mouse_wheel_position_last()
    {}

    void process_events(void);

    typedef std::function<void(SDL_Keysym &keysym)> callback_type;

    void register_user_callback_key_pressed (callback_type callback_function);
    void register_user_callback_key_released(callback_type callback_function);
    void register_user_callback_joystick(callback_type callback_function);
    void register_user_callback_mouse   (callback_type callback_function);

    Joystick get_joystick(unsigned int index) const { return (index==0) ? joystick0:joystick1; }

    int  get_mouse_wheel_position  (void) const { return mouse_wheel_position; }

private:
    void handle_key_pressed (SDL_Keysym &keysym);
    void handle_key_released(SDL_Keysym &keysym);

    void handle_mouse_motion         (SDL_MouseMotionEvent &mouse_motion);
    void handle_mouse_wheel          (SDL_MouseWheelEvent  &mouse_wheel);
    void handle_mouse_button_pressed (SDL_MouseButtonEvent &mouse_button_event);
    void handle_mouse_button_released(SDL_MouseButtonEvent &mouse_button_event);

    void handle_joystick_motion         (SDL_JoyAxisEvent &joystick);
    void handle_joystick_button_pressed (SDL_JoyButtonEvent &joystick);
    void handle_joystick_button_released(SDL_JoyButtonEvent &joystick);

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
    Joystick joystick0, joystick1;

    /* callbacks */
    callback_type user_callback_key_pressed; // think about a general callback register function for all events
    callback_type user_callback_key_released;
    callback_type user_callback_joystick;
    callback_type user_callback_mouse;

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

#endif // EVENT_MANAGER_H
