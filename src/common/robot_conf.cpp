
#include "./robot_conf.h"

namespace { //TODO maybe set up a file constants.h
    const unsigned int max_number_of_joints = 64;
    const unsigned int max_number_of_accels = 50;
    const unsigned int max_number_of_bodies = max_number_of_joints + 1;
}

//TODO refactor this method... horrible
Robot_Configuration::Robot_Configuration(Socket_Client &client)
: number_of_joints(0)
, number_of_accelsensors(0)
, number_of_bodies(0)
, joint()
, s_acc()
, bodies()
{
    sts_msg("Receiving robot's configuration.");

    int offset = 0;
    unsigned int tmp_id, tmp_type, tmp_sym;
    float tmp_jslo, tmp_jshi, tmp_jdef;
    char tmp_name[256];

    sts_msg("Waiting for simloid's answer.");
    const std::string server_message = client.recv(5*constants::seconds_us);
    sts_msg("Received and checking configuration.");

    if (0 == server_message.length())
        err_msg(__FILE__, __LINE__, "Received no more bytes. Close socket and exit. (robot conf)");

    if (sscanf(server_message.c_str(), "%u %u %u\n%n", &number_of_bodies, &number_of_joints, &number_of_accelsensors, &offset) == 3) {
        sts_msg("Robot's configuration: \n  %3u Bodies\n  %3u Joints\n  %3u Acceleration Sensors\n\n", number_of_bodies, number_of_joints, number_of_accelsensors);
    } else
        err_msg(__FILE__, __LINE__, "Unable to read the robot's configuration. Exit.");

    if ((number_of_joints < 1) || (number_of_joints > max_number_of_joints))
        err_msg(__FILE__, __LINE__, "Number of joints (%u) out of range (1..%u). Exit.", number_of_joints, max_number_of_joints);

    if (number_of_accelsensors > max_number_of_accels)
        err_msg(__FILE__, __LINE__, "Number of acceleration sensors (%u) out of range (0..%u). Exit.", number_of_accelsensors, max_number_of_accels);

    if ((number_of_bodies < 2) || (number_of_bodies > max_number_of_bodies))
        err_msg(__FILE__, __LINE__, "Number of bodies (%u) out of range (2..%u). Exit.", number_of_bodies, max_number_of_bodies);

    joint .reserve(number_of_joints);
    s_acc .assign(number_of_accelsensors, robots::Accel_Sensor());
    bodies.reserve(number_of_bodies);

    const char *msg = server_message.c_str();

    for (unsigned int i = 0; i < number_of_joints; ++i)
    {
        msg += offset;
        if (sscanf(msg, "%u %u %u %e %e %e %256s\n%n", &tmp_id, &tmp_type, &tmp_sym, &tmp_jslo, &tmp_jshi, &tmp_jdef, tmp_name, &offset) == 7) {
            dbg_msg("Joint %02u (%s), Type %u, a.w. %u, limits(%+1.2f, %+1.2f, %+1.2f)", tmp_id, tmp_name, tmp_type, tmp_sym, tmp_jslo, tmp_jdef, tmp_jshi);
        } else
            err_msg(__FILE__, __LINE__, "could not parse joint %u", i);

        if (tmp_type > 1)
            err_msg(__FILE__, __LINE__, "FIXME: Invalid Joint Type.");

        joint.emplace_back(i, tmp_type?robots::Joint_Type_Symmetric:robots::Joint_Type_Normal,
                           tmp_sym, tmp_name, tmp_jslo, tmp_jshi, tmp_jdef);

        assert(i == joint.size()-1);
    }

    for (unsigned int i = 0; i < number_of_bodies; ++i)
    {
        msg += offset;
        if (sscanf(msg, "%u %256s\n%n", &tmp_id, tmp_name, &offset) == 2) {
            dbg_msg("Body %02u (%s)", tmp_id, tmp_name);
        } else
            err_msg(__FILE__, __LINE__, "could not parse body %u", i);

        bodies.emplace_back(tmp_name);
        assert(i == bodies.size()-1);
    }

    sts_msg("Done reading robot configuration. Sending acknowledge.");
    client.send("ACK\n");

}

unsigned int
Robot_Configuration::get_number_of_symmetric_joints(void) const
{
    unsigned int num_sym_joints = 0;
    for (unsigned int i = 0; i < number_of_joints; ++i)
        if (robots::Joint_Type_Symmetric == joint[i].type)
            ++num_sym_joints;

    assert(num_sym_joints * 2 <= number_of_joints);
    return num_sym_joints;
}
