/* network2D.h */

#ifndef NETWORK2D_H
#define NETWORK2D_H

#include "axes.h"


class network2D
{
   private:
   float **X; // position vector
   float *size;
   /* size == 0, kein Knoten */
   unsigned char **edges;

   int pointer;
   int special_node;
   int activated_node;
   int N;
   float px, py;
   float width, height;
   GLubyte color[4];
   axes *A;

   public:
   network2D(int, axes*, const GLubyte[4]);
   void draw(void);

   void update_node(int, float, float, float);
   void update_edge(int, int, unsigned char);
   void special(int);
   void activated(int);
};

#endif /*NETWORK2D_H*/
