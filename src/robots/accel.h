#ifndef ACCEL_H_INCLUDED
#define ACCEL_H_INCLUDED

#include "../../simloidTCP/src/basic/vector3.h"

namespace robots {

struct Accel_Sensor
{
    Accel_Sensor() : a(), v() {}
    void integrate(void) { v = 0.1 * a + 0.9 * v; }
    void reset    (void) { v = 0.0; a = 0.0;      }

    Vector3 a;
    Vector3 v;
};

typedef std::vector<Accel_Sensor> Accelvector_t;

} // namespace

#endif // ACCEL_H_INCLUDED
