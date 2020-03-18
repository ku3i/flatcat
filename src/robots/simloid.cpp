/* simloid.cpp */
#include "./simloid.h"

namespace robots {

Simloid::Simloid( bool interlaced_mode,
                  unsigned short port,
                  unsigned int robot_ID,
                  unsigned int scene_ID,
                  bool visuals,
                  bool realtime,
                  std::vector<double> modelparams
                )
                : port(port)
                , robot_ID(robot_ID)
                , scene_ID(scene_ID)
                , visuals(visuals)
                , realtime(realtime)
                , child_pid()
                , mtx()
                , client()
                , connection_established(open_connection())
                , record_frame(false)
                , configuration(client.recv(5*network::constants::seconds_us), interlaced_mode)
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
                , left_id(get_body_id_by_name(configuration.bodies, "left"))
                , rift_id(get_body_id_by_name(configuration.bodies, "rift"))
{
    if (interlaced_mode) client.send("INTERLACED MODE\n");
    else                 client.send("SEQUENTIAL MODE\n");

    sts_msg("Done reading robot configuration. Sending acknowledge.");
    client.send("ACK\n");

    assert(configuration.number_of_bodies > 0);

    if (connection_established)
    {
        sts_msg("Connection established.");

        if (0 == modelparams.size()) init_robot();
        else {
            read_sensor_data();
            reinit_robot_model(modelparams);
        }
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

    return client.open_connection("127.0.0.1", port);
}

void
Simloid::simulation_idle(double sec)
{
    sts_msg("Waiting for %1.1f seconds.", sec);
    /* pass 100 time steps per second */
    for (unsigned i = 0; i < round(100.0 * sec); ++i)
    {
        read_sensor_data();
        client.send("DONE\n");
    }
}

void
Simloid::set_robot_to_default_position(void)
{
    client.send("GRAVITY OFF\nFIXED 0\n");

    double sec = 2; // should be enough
    sts_msg("Setting robot to default joint position.");

    char msg[network::constants::msglen];
    /* pass 100 time steps per second */
    for (unsigned i = 0; i < round(100.0 * sec); ++i)
    {
        read_sensor_data();

        short n = snprintf(msg, network::constants::msglen, "PX");
        for (auto& j: configuration.joints)
            n += snprintf(msg + n, network::constants::msglen - n, " %lf", j.default_pos);

        snprintf(msg + n, network::constants::msglen - n, "\nDONE\n");
        client.send(msg);
    }
    read_sensor_data();
    client.send("FIXED 0\nGRAVITY ON\nDONE\n");
    read_sensor_data();
}

void
Simloid::reset(void) //non-public
{
    /* resetting simloid, resetting motor output */
    for (auto& j: configuration.joints)
        j.motor.reset();
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
    for (auto& j: configuration.joints) j.motor.reset();
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
Simloid::idle(void)
{
    common::lock_t lock(mtx);

    if (!connection_established) {
        wrn_msg("Not connected.");
        return false;
    }

    send_pause_command();
    return true;
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
    for (auto& j: configuration.joints) j.motor.reset();
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
    unsigned charcount = 0;

    srv_msg = client.recv(60*network::constants::seconds_us);

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
    for (auto& j: configuration.joints)
        j.s_ang = clip(read_double(server_message, &charcount));

    /* read angle rate */
    for (auto& j: configuration.joints)
        j.s_vel = clip(read_double(server_message, &charcount));

    /* read acceleration sensors */
    for(auto& s: configuration.accels)
        s.a = read_vector3(server_message, &charcount);

    /* read body positions + velocities */
    for (auto& b: configuration.bodies)
    {
        b.position = read_vector3(server_message, &charcount);
        b.velocity = read_vector3(server_message, &charcount);
    }
}


void
Simloid::write_motor_data(void)
{
    static char msg[network::constants::msglen];
    unsigned n = snprintf(msg, network::constants::msglen, "UX");

    for (auto& j: configuration.joints)
        n += snprintf(msg + n, network::constants::msglen - n, " %lf", clip(j.motor.get()));

    auto const& bodies = configuration.bodies;
    for (unsigned i = 0; i < bodies.size(); ++i)
        if (bodies[i].force.length() > .0)
            n += snprintf(msg + n, network::constants::msglen - n, "\nFI %u %lf %lf %lf", i, bodies[i].force.x,
                                                                                             bodies[i].force.y,
                                                                                             bodies[i].force.z);
    snprintf(msg + n, network::constants::msglen - n, "\n%sDONE\n", record_frame ? "RECORD\n" : "");
    client.send(msg);

    /* transfer motor data u(t) to u(t-1) and reset value */
    for (auto& j: configuration.joints) {
        j.motor.transfer();
        j.motor = .0;
    }

    record_frame = false;
    return;
}

void
Simloid::send_pause_command(void) { client.send("PAUSE\nDONE\n"); }

void
Simloid::set_low_sensor_quality(bool low_quality) {
    if (low_quality)
        client.send("SENSORS POOR\n");
    else
        client.send("SENSORS GOOD\n");
}

void
Simloid::update_avg_position(void)
{
    Vector3 position(.0);
    for (auto& b: configuration.bodies)
        position += b.position;

    average_position = position / configuration.bodies.size();
}

void
Simloid::update_avg_velocity(void)
{
    Vector3 velocity(.0);
    for (auto& b: configuration.bodies)
        velocity += b.velocity;

    average_velocity = velocity / configuration.bodies.size();
}

Vector3
Simloid::get_min_position(void) const
{
    Vector3 min_position(DBL_MAX);
    for (auto& b: configuration.bodies)
    {
        min_position.x = std::min(min_position.x, b.position.x);
        min_position.y = std::min(min_position.y, b.position.y);
        min_position.z = std::min(min_position.z, b.position.z);
    }
    return min_position;
}

Vector3
Simloid::get_max_position(void) const
{
    Vector3 max_position(-DBL_MAX);
    for (auto& b: configuration.bodies)
    {
        max_position.x = std::max(max_position.x, b.position.x);
        max_position.y = std::max(max_position.y, b.position.y);
        max_position.z = std::max(max_position.z, b.position.z);
    }
    return max_position;
}

bool
Simloid::motion_stopped(double thrsh) const
{
    double sum_v = .0;
    for (auto& j: configuration.joints)
        sum_v += fabs(j.s_vel);
    return ((sum_v/configuration.number_of_joints) < thrsh);
}

bool
Simloid::dropped(double level) const
{
    assert(level >= 0 && level <= 1.0);
    return (configuration.bodies[0].position.z < level * body_position0[0].z);
}

double
Simloid::dx_from_origin(void) const
{
    return configuration.bodies[0].position.x - body_position0[0].x;
}

double
Simloid::dy_from_origin(void) const
{
    return configuration.bodies[0].position.y - body_position0[0].y;
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
        relative_position.normalize();
        const double len = relative_position.length();

        if (len != 0) assert_close(len, 1.0, 0.001, "relative position");

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
    for (auto& j: configuration.joints)
        power += square(j.motor.get());
    return power/configuration.number_of_joints;
}

unsigned
Simloid::get_body_id_by_name(const Bodyvector_t& bodies, const std::string& name) const {
    for (std::size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].name == name) return i;
    return bodies.size();
}

Vector3
Simloid::get_min_feet_pos(void) const {
    assert(left_id < configuration.bodies.size());
    assert(rift_id < configuration.bodies.size());
    return Vector3{ std::min(configuration.bodies[left_id].position.x, configuration.bodies[rift_id].position.x)
                  , std::min(configuration.bodies[left_id].position.y, configuration.bodies[rift_id].position.y)
                  , std::min(configuration.bodies[left_id].position.z, configuration.bodies[rift_id].position.z) };
}

Vector3
Simloid::get_max_feet_pos(void) const {
    assert(left_id < configuration.bodies.size());
    assert(rift_id < configuration.bodies.size());
    return Vector3{ std::max(configuration.bodies[left_id].position.x, configuration.bodies[rift_id].position.x)
                  , std::max(configuration.bodies[left_id].position.y, configuration.bodies[rift_id].position.y)
                  , std::max(configuration.bodies[left_id].position.z, configuration.bodies[rift_id].position.z) };
}

uint64_t
Simloid::randomize_model(double rnd_amp, double growth, uint64_t inst)
{
    if (0 == inst) {/* not initialized yet? */
        inst = time(NULL);
        sts_msg("Initializing random seed, instance is: %lu", inst);
    }

    sts_msg("Req. new model for robot %u and inst. %lu, amp. %lf, growth %lf", robot_ID, inst, rnd_amp, growth);
    client.send("MODEL %u 3 %lu %lf %lf\nDONE\n", robot_ID, inst, rnd_amp, growth);
    configuration.read_robot_info( client.recv(5*network::constants::seconds_us) );
    client.send("ACK\n");
    assert(configuration.number_of_bodies > 0);
    init_robot();
    return inst;
}

void
Simloid::reinit_robot_model(std::vector<double> const& params)
{
    sts_msg("Requesting new model for robot_id %u with %u params", robot_ID, params.size());
    client.send("MODEL %u %u %s\nDONE\n", robot_ID, params.size(), common::to_string(params).c_str());
    configuration.read_robot_info( client.recv(5*network::constants::seconds_us) );
    client.send("ACK\n");
    assert(configuration.number_of_bodies > 0);
    init_robot();
}

void
Simloid::reinit_motor_model(std::vector<double> const& params)
{
    sts_msg("Requesting new motor model with %u params", params.size());
    client.send("MOTOR %u %s\nDONE\n", params.size(), common::to_string(params).c_str());
}

} // namespace robots
