/* axes3D.h */

#ifndef AXES3D_H
#define AXES3D_H

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

class axes3D
{
    friend class plot2D;
    friend class plot3D;
    friend class network3D;
    friend class Vertices;
    friend class Graph;

private:
    float px, py, pz;
    float width, height, depth;
    GLfloat r[8][3], a[6][3];
    int flag; /* axes flag */

public:
    axes3D(float, float, float, float, float, float, int);
    void draw(float x_angle, float y_angle) const;
    void axesflag(int);
};

#endif /*AXES3D_H*/
