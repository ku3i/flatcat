#ifndef SETUPSDL_H
#define SETUPSDL_H

#include <sys/time.h>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <SDL2/SDL.h>
#include <thread>

#include <common/globalflag.h>
#include <common/basic.h>
#include <common/event_manager.h>
#include <common/stopwatch.h>
#include <common/application_base.h>
#include <common/visuals.h>
#include <common/misc.h>
#include <common/gui.h>
#include <common/modules.h>
#include <draw/draw.h>

void init_SDL(const bool visuals, const std::size_t window_width, const std::size_t window_height, const std::string& name = "working title");
void deinit_SDL(void);
void init_OpenGL(const std::size_t window_width, const std::size_t window_height);
void init_controls(void);
void signal_terminate_handler(int signum);
int  process_application(void *data);

void ui_main_loop(GlobalFlag &do_quit, const GlobalFlag &do_drawing, Event_Manager &em, const Application_Base &app);
void draw_screen(const double &fps, const Application_Base &app, const Event_Manager &em);
void fps_controller(double &fps, const double &sp_fps);
void quit(void);


/* get rid of that macros, prevents gtk from linking!*/

#define DEFINE_GLOBALS()                                                                                    \
                                                                                                            \
extern float progress_val1;                                                                                 \
extern float progress_val2;                                                                                 \
                                                                                                            \
extern GlobalFlag do_pause;                                                                                 \
extern GlobalFlag do_quit;                                                                                  \
extern GlobalFlag fast_forward;                                                                             \
extern GlobalFlag draw_grid;                                                                                \
extern GlobalFlag do_drawing;                                                                               \



#define APPLICATION_MAIN()                                                                                  \
                                                                                                            \
int main(int argc, char* argv[])                                                                            \
{                                                                                                           \
    srand((unsigned) time(NULL));                                                                           \
    signal(SIGINT, signal_terminate_handler);                                                               \
                                                                                                            \
    Event_Manager em;                                                                                       \
    Application app(argc, argv, em);                                                                        \
                                                                                                            \
    init_SDL(app.visuals_enabled(), app.window_width, app.window_height, app.name);                         \
                                                                                                            \
    /*GUI_Starter gui(app.visuals_enabled());*/                                                                 \
                                                                                                            \
    atexit(quit);                                                                                           \
    std::thread app_thread(process_application, &app);                                                      \
                                                                                                            \
    if (app.visuals_enabled()) ui_main_loop(do_quit, do_drawing, em, app);                                  \
                                                                                                            \
    app_thread.join();                                                                                      \
                                                                                                            \
    sts_msg("Bye.");                                                                                        \
    return 0;                                                                                               \
}                                                                                                           \

#endif // SETUPSDL_H
