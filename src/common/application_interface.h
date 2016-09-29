#ifndef APPLICATION_INTERFACE_H
#define APPLICATION_INTERFACE_H

#include <string>
#include <draw/graphics.h>

class Application_Interface
{
public:
    virtual void setup()  = 0; //Think about using this method for unit tests
    virtual bool loop()   = 0;
    virtual void finish() = 0;
    virtual void draw(const pref&) const = 0;
    virtual bool visuals_enabled() = 0;
    virtual uint64_t get_cycle_count(void) const = 0;

protected:
    virtual ~Application_Interface() {};
};

class Application_Base
{
public:
    Application_Base(const std::string& name, std::size_t width = 400, std::size_t height = 400)
    : window_width(width)
    , window_height(height)
    , name(name)
    { }

    virtual ~Application_Base() {}

    const unsigned int window_width;
    const unsigned int window_height;
    const std::string  name;
};


#endif // APPLICATION_INTERFACE_H
