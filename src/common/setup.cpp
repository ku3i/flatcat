#include <common/setup.h>

GlobalFlag do_pause    (true );
GlobalFlag do_quit     (false);
GlobalFlag fast_forward(false);
GlobalFlag draw_grid   (false);
GlobalFlag do_drawing  (false);
GlobalFlag screenshot  (false);

static SDL_Window *window;
static SDL_GLContext glcontext;

/* screen properties, TODO find a better place */
Visuals screen;

float t_delay_ms = 10.0f; // convert to unsigned int
double speed_factor = 1.0;

int
process_application(void *data)
{
    auto cur_time = std::chrono::high_resolution_clock::now();
    auto lst_time = cur_time;

    SimpleTimer tim = SimpleTimer(10*1000/*us*/, true); // 10 ms timer

    Application_Base *a = static_cast<Application_Base *>(data);
    sts_msg("Setting up application.");
    do_drawing.enable(); // enable drawing

    sts_msg("Entering application main loop.");
    while (!do_quit.status())
    {
        if (do_pause.status()) {
            usleep(10000); // 10ms
            a->paused();
        }
        else
        {
            if (a->get_cycle_count() % 100 == 0) {
                lst_time = cur_time;
                cur_time = std::chrono::high_resolution_clock::now();
                long long elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - lst_time).count();
                speed_factor = 1000 / static_cast<double>(elapsed_ms);
            }

            if (!a->loop())
            {
                sts_msg("Application main loop has terminated.");
                break;
            }

            if (!fast_forward.status()) {
                while(!tim.check_if_timed_out_and_restart((unsigned)t_delay_ms*1000))
                    usleep(100);
            }
        }
    }
    sts_msg("Finishing application.");
    a->finish();
    return 0;
}

void
fps_controller(double &fps, const double &sp_fps)
{
    static double delay_ms = 1.0;
    static double time_passed_s;
    static Stopwatch stopwatch;

    /* controller */
    delay_ms += 0.01 * (fps - sp_fps);
    if (delay_ms < 0.0) delay_ms = 0.0;

    usleep((int) round(1000 * delay_ms));
    time_passed_s = stopwatch.get_time_passed_us() / 1000000.0;

    if (time_passed_s > 0)
    fps = 0.9 * fps + 0.1 / time_passed_s; // low pass filter
}

void
ui_main_loop(GlobalFlag& do_quit, const GlobalFlag& do_drawing, Event_Manager& em, const Application_Base& app)
{
    sts_msg("Entering UI main loop.");
    double fps = 0.0;

    while (!do_quit.status())
    {
        em.process_events();

        if (do_drawing.status())
            draw_screen(fps, app);

        fps_controller(fps, visuals_defaults::frames_per_second); // delay to reach stable 25 f/s

    }
    sts_msg("Leaving UI main loop.");
    return;
}

void
init_SDL(const bool visuals, const std::size_t window_width, const std::size_t window_height, const std::string& name)
{
    if (visuals)
    {
        sts_msg("Initializing SDL.");
        /* Dimensions of our window. */
        assert_in_range(window_width , 100ul, 2048ul);
        assert_in_range(window_height, 100ul, 1024ul);

        screen.window_size_x = window_width;
        screen.window_size_y = window_height;

        /* First, initialize SDL's video subsystem. */
        sts_msg("Initializing SDL VIDEO.");
        if (SDL_Init(SDL_INIT_VIDEO))
            err_msg(__FILE__, __LINE__, "SDL Video initialization failed: %s", SDL_GetError());

        sts_msg("Initializing SDL Window.");
        window = SDL_CreateWindow( name.c_str()             // title
                                 , SDL_WINDOWPOS_UNDEFINED  // initial x position
                                 , SDL_WINDOWPOS_UNDEFINED  // initial y position
                                 , window_width             // width, in pixels
                                 , window_height            // height, in pixels
                                 , SDL_WINDOW_OPENGL
                                // | SDL_WINDOW_RESIZABLE     // flags
                                 );

        if (window == NULL)
            err_msg(__FILE__, __LINE__, "SDL Window initialization failed: %s", SDL_GetError());

        sts_msg("Creating GL context.");
        glcontext = SDL_GL_CreateContext(window);

        /* Color depth in bits of our window. */
        //TODO int bpp = info->vfmt->BitsPerPixel;

//        SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 5);
//        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 5);
//        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 5);
//        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
//        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

        /* Flags we will pass into SDL_SetVideoMode. */
        //int flags = SDL_OPENGL;// | SDL_FULLSCREEN;

        /* set the video mode */
        //if (SDL_SetVideoMode(screen.window_size_x, screen.window_size_y, bpp, flags) == 0)
        //    err_msg(__FILE__, __LINE__, "Initialization of video mode failed: %s", SDL_GetError());

        init_OpenGL(screen.window_size_x, screen.window_size_y);

        /* Initialize GLUT */
        sts_msg("Initialize GLUT.");
        int argc = 1; // fake arguments for glutInit()
        char * argv[] = {strdup("sim"), NULL};
        glutInit(&argc, argv);

    } // done initializing visuals

    /* initializing control pad */
    init_controls();

    atexit(deinit_SDL);
}

void
deinit_SDL(void) {
    sts_msg("De-initialize SDL on exit.");
    SDL_GL_DeleteContext(glcontext);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void
init_OpenGL(const std::size_t window_width, const std::size_t window_height)
{
    sts_msg("Initialize OpenGL.");
    double ratio = (double) window_width / (double) window_height;

    dbg_msg("GLBlendFunc");
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //glEnable(GL_LINE_SMOOTH);
    //glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);

    dbg_msg("GLShadeModel");
    glShadeModel(GL_SMOOTH);  // shading model Gouraud (smooth)

    /* Culling. */
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glEnable(GL_CULL_FACE);

    /* Set the clear color. */
    glClearColor(0.0, 0.0, 0.0, 0.0);//glClearColor(0.05, 0.0, 0.1, 0.0);

    /* Setup our view port. */
    glViewport(0, 0, window_width, window_height);

    /* Change to the projection matrix and set our viewing volume. */
    dbg_msg("GLProjection");
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(visuals_defaults::gl_fovy,
                   ratio,
                   visuals_defaults::gl_zNear,
                   visuals_defaults::gl_zFar);
}

void
init_controls()
{
    sts_msg("Initializing controls.");
    /* Initialize the SDL joystick subsystem */
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK))
        err_msg(__FILE__, __LINE__, "SDL joystick initialization failed: %s", SDL_GetError());

    int num_buttons = 0;     // number of control pad buttons
    int num_axes = 0;        // number of control pad axes pairs
    SDL_Joystick *ctrl;

    /* initialize control pad */
    int JNum = SDL_NumJoysticks();
    int joystick_ID = 0;

    if (JNum > 0)
    {
        if (JNum > 1) sts_msg("Available control pads: %d", JNum);

        /* open control pad with jID 0 */
        ctrl = SDL_JoystickOpen(joystick_ID);
        if (NULL != ctrl)
        {
            num_buttons = SDL_JoystickNumButtons(ctrl);
            num_axes = SDL_JoystickNumAxes(ctrl);

            if (num_buttons < 1) wrn_msg("Control pad \"%s\" has no buttons available.", SDL_JoystickName(ctrl));
            if (num_axes < 2)    wrn_msg("Control pad \"%s\" has insufficient axes."   , SDL_JoystickName(ctrl));

            sts_msg("Control pad \"%s\" found with %d axes and %d buttons.", SDL_JoystickName(ctrl), num_axes, num_buttons);
        }
        else
            err_msg(__FILE__, __LINE__, "control pad error: %s", SDL_GetError());

    }
    else wrn_msg("There is no control pad available.");

    return;
}

void
draw_screen(const double& fps, const Application_Base& app)
{
    static FILE *fp;
    const unsigned long long cycles = app.get_cycle_count();

    /* prepare screen shot */
    if (screenshot.status())
    {

        int buffsize = 1024*1024;

        fp = open_file("wb", "screenshot_%llu.svg", cycles);
        sts_msg("Prepare screen shot at cycle: %llu",cycles);

        gl2psBeginPage("screenshot", "gl2ps", NULL, GL2PS_SVG, GL2PS_SIMPLE_SORT,
                       GL2PS_DRAW_BACKGROUND | GL2PS_USE_CURRENT_VIEWPORT,
                       GL_RGBA, 0, NULL, 0, 0, 0, buffsize, fp, "foo.svg");
    }

    /* Clear the color and depth buffers. */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* We don't want to modify the projection matrix. */
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslatef( screen.x_position + screen.x_position_disp
                , screen.y_position + screen.y_position_disp
                ,-screen.zdist );

    if (screen.rotate_view && (screen.x_angle += 0.1f * screen.rot_factor) > 360.0f)
        screen.x_angle = 0.0f;

    const pref p = {screen.x_angle + screen.x_angle_disp, screen.y_angle + screen.y_angle_disp};
    app.draw(p); // draw application dependent content

    /* overlay grid */
    if (draw_grid.status()) {
        glColor4f(1.0f, 1.0f, 1.0f, 0.1f);
        glLineWidth(1.0f);
        draw_grid2D(1.0, 5);
    }

    if (screen.show_fps) {
        glColor3f(1.0, 1.0, 1.0);
        glprintf(-1.0,-1.0, 0.0, .025, "%s (%lu), %d fps %1.2fx", get_time_from_cycle_counter(cycles).c_str(), cycles, (int) round(fps), speed_factor);
    }

    /* finish screen shot */
    if (screenshot.status()) {
        if (GL2PS_OVERFLOW == gl2psEndPage())
            wrn_msg("Overflow while creating screen shot. Increase buffer size.");
        fclose(fp);
        screenshot.disable();
    }

    SDL_GL_SwapWindow(window); // swap the buffers
}

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
signal_terminate_handler(int signum)
{
    sts_msg("Got a SIGINT(%d) from user\n", signum);
    quit();
}
