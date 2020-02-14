#ifndef ROBOT_CONF_H
#define ROBOT_CONF_H

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <assert.h>

#include <robots/joint.h>
#include <robots/accel.h>
#include <common/socket_client.h>
#include <basic/vector3.h>


struct Body_Segment /**TODO move to separate header */
{
    Body_Segment(const char* name) : position(), velocity(), force(), name(name) {}
    Vector3 position;
    Vector3 velocity;
    Vector3 force;
    std::string name;
};
typedef std::vector<Body_Segment> Bodyvector_t;

class Robot_Configuration
{

public:
    unsigned number_of_joints = 0;
    unsigned number_of_accels = 0;
    unsigned number_of_bodies = 0;

    robots::Jointvector_t joints;
    robots::Accelvector_t accels;
    Bodyvector_t          bodies;

    bool const interlaced;

    Robot_Configuration(std::string const& server_message, bool interlaced)
    : joints()
    , accels()
    , bodies()
    , interlaced(interlaced)
    {
        read_robot_info(server_message);
    }

    void delete_symmetry_information(void) { //TODO get rid of that method
        sts_msg("Deleting symmetry information.");
        for (unsigned int i = 0; i < joints.size(); ++i) {
            joints[i].type = robots::Joint_Type_Normal;
            joints[i].symmetric_joint = i; // delete reference to other joints
        }
        assert(get_number_of_symmetric_joints() == 0);
    }

    unsigned int get_number_of_symmetric_joints(void) const;


    void read_robot_info(std::string const& server_message);

private:
    const char* read_joints(const char* msg, int* offset);
    const char* read_bodies(const char* msg, int* offset);
};


#endif // ROBOT_CONF_H
