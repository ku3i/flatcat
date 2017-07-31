/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | July 2017                       |
 +---------------------------------*/

#ifndef APPLICATION_BASE_H
#define APPLICATION_BASE_H

#include <string>

#include <common/event_manager.h>
#include <common/datalog.h>

#include <draw/graphics.h>

namespace constants {
    const unsigned default_window_width  = 400;
    const unsigned default_window_height = 400;
    const std::string logfolder = "./data/";
    const std::string logfileext = ".log";
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
    , logger( basic::make_directory(constants::logfolder.c_str()) // folder
            + basic::get_timestamp()                              // name as time stamp
            + constants::logfileext )                             // file extension
    {
        sts_msg("Loading application...");
        /* register key event */
        em.register_user_callback_key_pressed (std::bind(&Application_Base::base_callback_key_pressed , this, std::placeholders::_1));
        em.register_user_callback_key_released(std::bind(&Application_Base::base_callback_key_released, this, std::placeholders::_1));
    }

    Event_Manager&    em;
    const std::string name;
    const unsigned    window_width;
    const unsigned    window_height;

    uint64_t          cycles;

    Datalog           logger;

protected:
    virtual ~Application_Base() = default;

private:
    void base_callback_key_pressed (const SDL_Keysym& keysym) {
        switch (keysym.sym) {
            case SDLK_RSHIFT: logger.toggle_logging(); break;
            default: break;
        }
        user_callback_key_pressed (keysym);
    };

    void base_callback_key_released(const SDL_Keysym& keysym) {
        user_callback_key_released(keysym);
    };
};


#endif // APPLICATION_BASE_H
