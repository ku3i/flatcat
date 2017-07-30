/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | July 2017                       |
 +---------------------------------*/

#ifndef APPLICATION_BASE_H
#define APPLICATION_BASE_H

#include <string>

#include <common/event_manager.h>
#include <draw/graphics.h>

namespace constants {
    const unsigned default_window_width  = 400;
    const unsigned default_window_height = 400;
}


class Application_Base
{
public:
    virtual bool loop() = 0;                                 /* for everything which has to be done repeatedly */
    virtual void finish() = 0;                               /* for things to be done before exiting */
    virtual void draw(const pref&) const = 0;                /* things to draw */
    virtual bool visuals_enabled() { return true; };         /* returns true if the application was started with visuals */
    uint64_t get_cycle_count(void) const { return cycles; }  /* returns the current application cycle */

    virtual void user_callback_key_pressed (const SDL_Keysym& /*keysym*/) {};
    virtual void user_callback_key_released(const SDL_Keysym& /*keysym*/) {};


    Application_Base( Event_Manager& em
                    , const std::string& name
                    , unsigned width  = constants::default_window_width
                    , unsigned height = constants::default_window_height )
    : em(em)
    , name(name)
    , window_width(width)
    , window_height(height)
    , cycles(0)
    {
        sts_msg("Loading application...");
        /* register key event */
        em.register_user_callback_key_pressed (std::bind(&Application_Base::user_callback_key_pressed , this, std::placeholders::_1));
        em.register_user_callback_key_released(std::bind(&Application_Base::user_callback_key_released, this, std::placeholders::_1));
    }

    Event_Manager&    em;
    const std::string name;
    const unsigned    window_width;
    const unsigned    window_height;

    uint64_t          cycles;

protected:
    virtual ~Application_Base() = default;
};


#endif // APPLICATION_BASE_H
