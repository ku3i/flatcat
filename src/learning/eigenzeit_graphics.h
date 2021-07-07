#ifndef LEARNING_EIGENZEIT_GRAPHICS_H
#define LEARNING_EIGENZEIT_GRAPHICS_H

#include <learning/eigenzeit.h>

namespace learning {

class Eigenzeit_Graphics : public Graphics_Interface {
    const Eigenzeit& eigenzeit;
public:
    Eigenzeit_Graphics(const Eigenzeit& eigenzeit) : eigenzeit(eigenzeit) {}
    void draw(const pref& /*p*/) const { /*TODO*/ }
};

} /* namespace learning */

#endif /* LEARNING_EIGENZEIT_GRAPHICS_H */
