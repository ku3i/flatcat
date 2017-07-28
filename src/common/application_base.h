/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | July 2017                       |
 +---------------------------------*/

#ifndef APPLICATION_BASE_H
#define APPLICATION_BASE_H

#include <string>
#include <draw/graphics.h>


namespace constants {
    const unsigned default_window_width  = 400;
    const unsigned default_window_height = 400;
}


class Application_Base
{
public:
    virtual bool loop() = 0;                          /* for everything which has to be done repeatedly */
    virtual void finish() = 0;                        /* for things to be done before exiting */
    virtual void draw(const pref&) const = 0;         /* things to draw */
    virtual bool visuals_enabled() = 0;               /* returns true if the application was started without visuals */
    virtual uint64_t get_cycle_count(void) const = 0; /* returns the current application cycle */

    Application_Base( const std::string& name
                    , unsigned width  = constants::default_window_width
                    , unsigned height = constants::default_window_height )
    : window_width(width)
    , window_height(height)
    , name(name)
    { }

    const unsigned    window_width;
    const unsigned    window_height;
    const std::string name;

protected:
    virtual ~Application_Base() = default;
};


#endif // APPLICATION_BASE_H
