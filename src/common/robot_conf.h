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
    Body_Segment() : position(), velocity(), force() {}
    Vector3 position;
    Vector3 velocity;
    Vector3 force;
};
typedef std::vector<Body_Segment> Bodyvector_t;

class Robot_Configuration
{
public:
    Robot_Configuration(Socket_Client &client); //robot conf should not depend on socket client, reading configuration should be done outside

    void delete_symmetry_information(void) { //TODO get rid of that method
        sts_msg("Deleting symmetry information.");
        for (unsigned int i = 0; i < joint.size(); ++i) {
            joint[i].type = robots::Joint_Type_Normal;
            joint[i].symmetric_joint = i; // delete reference to other joints
        }
        assert(get_number_of_symmetric_joints() == 0);
    }

    unsigned int get_number_of_symmetric_joints(void) const;

    unsigned int number_of_joints;
    unsigned int number_of_accelsensors;
    unsigned int number_of_bodies;

    robots::Jointvector_t joint;
    robots::Accelvector_t s_acc;
    Bodyvector_t          bodies;
};


#endif // ROBOT_CONF_H
