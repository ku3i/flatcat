#ifndef VISUALS_H_INCLUDED
#define VISUALS_H_INCLUDED

namespace visuals_defaults {
    const unsigned int window_width  = 512;
    const unsigned int window_height = 512;
    const unsigned int frames_per_second = 25;
    const float        zdist = 1.7320508f + 0.01f; // z = sin(pi/3) / sin(pi/6) for 60 deg view angle to be 2 units width;
    const unsigned int rot_factor = 1;
    const float        zoom_factor = 0.95;

    const float        gl_fovy  = 60.0;
    const float        gl_zNear = 0.05;
    const float        gl_zFar  = 1024.0;
}

//TODO improve this
struct Visuals
{
    Visuals()
    : window_size_x(visuals_defaults::window_width)
    , window_size_y(visuals_defaults::window_height)
    , mdy(0.0f)
    , mdx(0.0f)
    , x_angle(0.0f)
    , y_angle(20.0f)
    , x_angle_disp(0.0f)
    , y_angle_disp(0.0f)
    , x_position(.0f)
    , y_position(.0f)
    , x_position_disp(.0f)
    , y_position_disp(.0f)
    , zdist(visuals_defaults::zdist)
    , rotate_view(true)
    , rot_factor(visuals_defaults::rot_factor)
    , show_fps(true)
    , snap(1.0)
    {
        sts_msg("Initialized settings for visuals.");
    }

    void reset(void) {
        x_angle =   .0f;
        y_angle = 20.0f;
        x_position = .0f;
        y_position = .0f;
        zdist   = visuals_defaults::zdist;
        rotate_view = true;
        rot_factor  = visuals_defaults::rot_factor;
        mdx = .0f;
        mdy = .0f;
    }

    void set_background_color(Color4 color) { glClearColor(color.r, color.g, color.b, color.a); }

    unsigned int window_size_x;
    unsigned int window_size_y;
    float mdy;
    float mdx;
    float x_angle;
    float y_angle;
    float x_angle_disp;
    float y_angle_disp;
    float x_position;
    float y_position;
    float x_position_disp;
    float y_position_disp;
    float zdist;
    bool  rotate_view;
    int   rot_factor;
    bool  show_fps;
    float snap;
};

#endif // VISUALS_H_INCLUDED
