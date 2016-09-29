/* simloid.cpp */
#include "./simloid.h"

namespace robots {

Simloid::Simloid( const unsigned short port,
                  const unsigned int robot_ID,
                  const unsigned int scene_ID,
                  const bool visuals,
                  const bool realtime)
                : port(port)
                , robot_ID(robot_ID)
                , scene_ID(scene_ID)
                , visuals(visuals)
                , realtime(realtime)
                , child_pid()
                , mtx()
                , client()
                , connection_established(open_connection())
                , configuration(client)
                , timestamp()
                , body_position0(configuration.number_of_bodies)
                , average_position()
                , average_position0()
                , average_velocity()
                , average_rotation()
                , avg_rot_inf_ang()
                , avg_rot_inf_ang_last()
                , avg_velocity_forward()
                , avg_velocity_left()
{
    assert(configuration.number_of_bodies > 0);

    if (connection_established)
    {
        sts_msg("Connection established.");
        init_robot();
    }
    else
        wrn_msg("Cannot connect to robot.");
}

Simloid::~Simloid()
{
    if (connection_established)
        close_connection();
}

bool
Simloid::open_connection(void)
{
    sts_msg("Forking process.");

    child_pid = fork();
    if (child_pid >= 0) // fork was successful
    {
        if (child_pid == 0) // child process
        {
            sts_msg("Child process says \'hello\'.");
            sts_msg("Checking if processor is available...");
            if (system(NULL)) sts_msg("OK.");
            else err_msg(__FILE__, __LINE__, "FAILED.");

            char portarg[8], robotarg[8], scenearg[8], nographics[8] = "", norealtime[8] = ""; //TODO: aufnäumen, extra funktion oder so.
            snprintf(portarg,    8, "%d", port);
            snprintf(robotarg,   8, "%d", robot_ID);
            snprintf(scenearg,   8, "%d", scene_ID);

            if (!visuals) snprintf(nographics, 8, "-ng");
            if (!realtime) snprintf(norealtime, 8, "-nr");

            sts_msg("Starting simloid on port %s with robot '%s' and scene '%s'.", portarg, robotarg, scenearg);
            if (!visuals) sts_msg("Starting without graphics.");
            if (!realtime) sts_msg("Starting with maximal speed (no realtime).");

            int i = execl("../simloidTCP/bin/Release/simloid", "simloid", "--port", portarg, "--robot", robotarg, "--scene", scenearg, nographics, norealtime, (char *) 0);
            err_msg(__FILE__, __LINE__, "\n +++ Could not start simloid. Status: %d +++ \n%s\n", i, strerror(errno));
        }
        else //Parent process
        {
            sts_msg("Parent process says \'hello\'.");
        }
    }
    else
    {
        wrn_msg("Forking process failed.");
        return false;
    }

    sts_msg("Waiting a second for simloid to start.");
    sleep(1);
    sts_msg("Continuing.");

    return client.open_connection(port);
}

void
Simloid::simulation_idle(double sec)
{
    sts_msg("Waiting for %1.1f seconds.", sec);
    /* pass 100 time steps per second */
    for (unsigned int i = 0; i < round(100.0 * sec); ++i)
    {
        read_sensor_data();
        client.send("DONE\n");
    }
}

void
Simloid::set_robot_to_default_position(void)
{
    double sec = 2; // should be enough
    sts_msg("Setting robot to default joint position.");

    char msg[MSGLEN];
    /* pass 100 timesteps per second */
    for (unsigned int i = 0; i < round(100.0 * sec); ++i)
    {
        read_sensor_data();

        short n = snprintf(msg, MSGLEN, "PX");
        for (unsigned int k = 0; k < configuration.number_of_joints; ++k)
            n += snprintf(msg + n, MSGLEN - n, " %lf", configuration.joint[k].default_pos);

        snprintf(msg + n, MSGLEN - n, "\nDONE\n");
        client.send(msg);
    }
    read_sensor_data();
    client.send("GRAVITY ON\nDONE\n");
    read_sensor_data();
}

void
Simloid::reset(void) //non-public
{
    /* resetting simloid, resetting motor output */
    for (unsigned int i = 0; i < configuration.number_of_joints; ++i)
        configuration.joint[i].motor = .0;
    client.send("UA 0\nRESET\nDONE\n");
    read_sensor_data(); // note: a reset must be followed by an update
}

void
Simloid::save_state(void)
{
    if (!connection_established)
    {
        wrn_msg("Cannot save state. Not connected.");
        return;
    }
    /* saving state of simloid */
    client.send("UA 0\nNEWTIME\nSAVE\nDONE\n");
    read_sensor_data();

    // save initial position
    for (std::size_t i = 0; i < configuration.bodies.size(); ++i)
        body_position0[i] = configuration.bodies[i].position;

    update_avg_position();
    update_avg_velocity();
    average_position0 = average_position;
}


void
Simloid::restore_state(void)
{
    common::lock_t lock(mtx);

    if (!connection_established) {
        wrn_msg("Cannot restore state. Not connected.");
        return;
    }

    /* restoring last snapshot of simloid, resetting motor output */
    for (unsigned int i = 0; i < configuration.number_of_joints; ++i)
        configuration.joint[i].motor = .0;
    reset_all_forces();
    client.send("UA 0\nNEWTIME\nRESTORE\nDONE\n");
    read_sensor_data(); // note: a reset must be followed by an update

    /* TODO: irgendwie ist nachm restore und sensorupdate noch die alten werte da */
}

void
Simloid::finish(void)
{
    if (connection_established) {
        sts_msg("Closing connection to simloid.");
        close_connection();
    }
    else wrn_msg("Cannot finish simloid. Already disconnected.");
    return;
}

bool
Simloid::update(void)
{
    common::lock_t lock(mtx);

    if (!connection_established) {
        wrn_msg("Cannot update sensor values. Not connected.");
        return false;
    }

    write_motor_data();
    read_sensor_data();

    update_avg_position();
    update_avg_velocity();
    update_rotation_z();
    update_robot_velocity();
    return true;
}


void Simloid::init_robot(void)
{
    common::lock_t lock(mtx);

    sts_msg("Initializing robot.");

    /* initialize motor voltages */
    for (unsigned int i = 0; i < configuration.number_of_joints; ++i)
        configuration.joint[i].motor = .0;
    set_robot_to_default_position();
    save_state();
}

void
Simloid::close_connection(void)
{
    /* close connection and socket */
    sts_msg("Closing connection to server.");
    client.send("EXIT\n");
    client.close_connection();
    sts_msg("TCP connection terminated. Waiting for simloid to exit.");

    int status;
    while (waitpid(child_pid, &status, 0) != child_pid)
    {
        sts_msg("Tick...");
        sleep(1);
    }
    sts_msg("Simloid finished.");
    connection_established = false;
}

double
read_double(const char *msg_buffer, unsigned int *offset)
{
    double value = .0;
    const char *msg = msg_buffer + (*offset);
    int chars_read = 0;

    if (1 == sscanf(msg, " %lf%n", &value, &chars_read))
    {
        msg += chars_read;
        *offset += chars_read;
    }
    else wrn_msg("Cannot read expected (double) value from TCP message at offset %d.", *offset);

    return value;
}

Vector3
read_vector3(const char *msg_buffer, unsigned int *offset)
{
    const double x = read_double(msg_buffer, offset);
    const double y = read_double(msg_buffer, offset);
    const double z = read_double(msg_buffer, offset);
    return Vector3(x, y, z);
}

void
Simloid::read_sensor_data(void)
{
    static std::string srv_msg;
    unsigned int charcount = 0;

    srv_msg = client.recv(60*seconds_us);

    if (0 == srv_msg.length())
    {
        wrn_msg("Received no more bytes. Cancel reading sensory data.");
        close_connection();
        return;
    }

    /* read time stamp */
    const char *server_message = srv_msg.c_str(); // transitional, remove TODO:

    timestamp = read_double(server_message, &charcount);

    /* read angles */
    for (unsigned int i = 0; i < configuration.number_of_joints; ++i)
        configuration.joint[i].s_ang = clip(read_double(server_message, &charcount));

    /* read angle rate */
    for (unsigned int i = 0; i < configuration.number_of_joints; ++i)
        configuration.joint[i].s_vel = clip(read_double(server_message, &charcount));

    /* read acceleration sensors */
    for (unsigned int i = 0; i < configuration.number_of_accelsensors; ++i)
        configuration.s_acc[i].a = read_vector3(server_message, &charcount);

    /* read body positions + velocities */
    for (unsigned int i = 0; i < configuration.number_of_bodies; ++i)
    {
        configuration.bodies[i].position = read_vector3(server_message, &charcount);
        configuration.bodies[i].velocity = read_vector3(server_message, &charcount);
    }
}

void
Simloid::write_motor_data(void)
{
    static char msg[MSGLEN];
    unsigned int n = snprintf(msg, MSGLEN, "UX");

    for (unsigned int i = 0; i < configuration.number_of_joints; ++i)
        n += snprintf(msg + n, MSGLEN - n, " %lf", clip(configuration.joint[i].motor));

    for (unsigned int i = 0; i < configuration.number_of_bodies; ++i)
        if (configuration.bodies[i].force.length() > .0)
            n += snprintf(msg + n, MSGLEN - n, "\nFI %u %lf %lf %lf", i, configuration.bodies[i].force.x,
                                                                         configuration.bodies[i].force.y,
                                                                         configuration.bodies[i].force.z);
    snprintf(msg + n, MSGLEN - n, "\nDONE\n");
    client.send(msg);
    return;
}

//double
//Simloid::get_min_distance_from_start(void)
//{
//    double minx = DBL_MAX;
//    double miny = DBL_MAX;
//    double minz = DBL_MAX;
//    for (unsigned int i = 0; i < configuration.number_of_bodies; ++i)
//    {
//        if (configuration.bodies[i].x < minx) minx = fabs(configuration.bodies[i].x);
//        if (configuration.bodies[i].y < miny) miny = fabs(configuration.bodies[i].y);
//        if (configuration.bodies[i].z < minz) minz = fabs(configuration.bodies[i].z);
//    }
//    return sqrt(square(minx) + square(miny) + square(minz));
//}

//double
//Simloid::get_median_position_y(void)
//{
//    /* sort body positions and return the one in the middle */
//    double median = 0.0;
//    switch (configuration.number_of_bodies)
//    {
//    case 0: median = 0.0;
//    case 1: median = configuration.bodies[0].y;
//    case 2: median = 0.5 * (configuration.bodies[0].y + configuration.bodies[1].y);
//    default:
//        std::vector<double> body_positions;
//        body_positions.reserve(configuration.number_of_bodies);
//        for (unsigned int i = 0; i < configuration.number_of_bodies; ++i)
//            body_positions.emplace_back(configuration.bodies[i].y);
//        std::sort(body_positions.begin(), body_positions.end());
//
//        if (is_even(configuration.number_of_bodies))
//            median = .5 * (body_positions[configuration.number_of_bodies/2 -1 ] + body_positions[configuration.number_of_bodies/2]);
//        else
//            median = body_positions[configuration.number_of_bodies/2];
//    }
//
//    return median;
//    return get_mean_position_y();
//}



//double
//Simloid::get_median_position_x(void)
//{
//    /* sort body positions and return the one in the middle */
//    double median = 0.0;
//    switch (configuration.number_of_bodies)
//    {
//    case 0: median = 0.0;
//    case 1: median = configuration.bodies[0].x;
//    case 2: median = 0.5 * (configuration.bodies[0].x + configuration.bodies[1].x);
//    default:
//        std::vector<double> body_positions;
//        body_positions.reserve(configuration.number_of_bodies);
//        for (unsigned int i = 0; i < configuration.number_of_bodies; ++i)
//            body_positions.emplace_back(configuration.bodies[i].x);
//        std::sort(body_positions.begin(), body_positions.end());
//
//        if (is_even(configuration.number_of_bodies))
//            median = .5 * (body_positions[configuration.number_of_bodies/2 -1 ] + body_positions[configuration.number_of_bodies/2]);
//        else
//            median = body_positions[configuration.number_of_bodies/2];
//    }
//
//    return median;
//}

//double
//Simloid::get_min_distance_up(void)
//{
//    /* upwards has positive sign on y axis */
//    assert(configuration.number_of_bodies > 0);
//    double min_z = configuration.bodies[0].z;
//    for (unsigned int i = 1; i < configuration.number_of_bodies; ++i)
//        min_z = std::min(min_z, configuration.bodies[i].z);
//
//    return min_z;
//}

void
Simloid::update_avg_position(void)
{
    Vector3 position(.0);
    for (unsigned int i = 0; i < configuration.number_of_bodies; ++i)
        position += configuration.bodies[i].position;

    average_position = position / configuration.number_of_bodies;
}
void
Simloid::update_avg_velocity(void)
{
    Vector3 velocity(.0);
    for (unsigned int i = 0; i < configuration.number_of_bodies; ++i)
        velocity += configuration.bodies[i].velocity;

    average_velocity = velocity / configuration.number_of_bodies;
}

Vector3
Simloid::get_min_position(void) const
{
    Vector3 min_position(DBL_MAX);
    for (unsigned int i = 0; i < configuration.number_of_bodies; ++i)
    {
        min_position.x = std::min(min_position.x, configuration.bodies[i].position.x);
        min_position.y = std::min(min_position.y, configuration.bodies[i].position.y);
        min_position.z = std::min(min_position.z, configuration.bodies[i].position.z);
    }
    return min_position;
}

Vector3
Simloid::get_max_position(void) const
{
    Vector3 max_position(-DBL_MAX);
    for (unsigned int i = 0; i < configuration.number_of_bodies; ++i)
    {
        max_position.x = std::max(max_position.x, configuration.bodies[i].position.x);
        max_position.y = std::max(max_position.y, configuration.bodies[i].position.y);
        max_position.z = std::max(max_position.z, configuration.bodies[i].position.z);
    }
    return max_position;
}

bool
Simloid::motion_stopped(double thrsh) const
{
    double sum_v = .0;
    for (unsigned int i = 0; i < configuration.number_of_joints; ++i)
        sum_v += fabs(configuration.joint[i].s_vel);

    return ((sum_v/configuration.number_of_joints) < thrsh);
}

bool
Simloid::dropped(double level) const
{
    assert(level >= 0 && level <= 1.0);
    return (configuration.bodies[0].position.z < level * body_position0[0].z);
}

bool
Simloid::out_of_track_x(void) const
{
    return (fabs(configuration.bodies[0].position.x) > 0.5);
}

bool
Simloid::out_of_track_y(void) const
{
    return (fabs(configuration.bodies[0].position.y - body_position0[0].y) > 0.5);
}

/* for mean calculation of circular quantities see:
   https://en.wikipedia.org/wiki/Mean_of_circular_quantities
 */
void
Simloid::update_rotation_z(void)
{
    assert(configuration.number_of_bodies == configuration.bodies.size());
    double sum_y = 0.0;
    double sum_x = 0.0;

    for (std::size_t i = 0; i < configuration.number_of_bodies; ++i) {
        Vector3 relative_position = configuration.bodies[i].position - average_position; // avg free position
        relative_position.normalize(); // TODO: needed?
        assert_close(relative_position.length(), 1.0, 0.001);

        const Vector3 relpos0 = body_position0[i] - average_position0;
        const double relative_rotation = relative_position.angle_phi() - relpos0.angle_phi();

        sum_y += sin(relative_rotation);
        sum_x += cos(relative_rotation);
    }

    average_rotation = atan2( sum_y/configuration.number_of_bodies
                            , sum_x/configuration.number_of_bodies );

    avg_rot_inf_ang_last = avg_rot_inf_ang;
    avg_rot_inf_ang = unwrap(average_rotation, avg_rot_inf_ang);
}

void
Simloid::update_robot_velocity(void)
{
    const double& a = average_rotation;
    const double& x = average_velocity.x;
    const double& y = -average_velocity.y;

    avg_velocity_left    = cos(a)*x - sin(a)*y;
    avg_velocity_forward = sin(a)*x + cos(a)*y;
}

double
Simloid::get_normalized_mechanical_power(void) const
{
    double power = .0;
    for (unsigned int i = 0; i < configuration.number_of_joints; ++i)
        power += square(configuration.joint[i].motor);
    return power/configuration.number_of_joints;
}

} // namespace robots
