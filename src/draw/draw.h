/* draw.h */
#ifndef DRAW_H
#define DRAW_H

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <string.h>
#include <string>

#include <basic/color.h>

const GLubyte red           [] = {255,   0,   0, 255};
const GLubyte green         [] = {  0, 255,   0, 255};
const GLubyte blue          [] = {  0,   0, 255, 255};
const GLubyte white         [] = {255, 255, 255, 255};
const GLubyte white_trans   [] = {255, 255, 255,  64};
const GLubyte white_trans2  [] = {255, 255, 255, 128};
const GLubyte cyan          [] = {  0, 255, 255, 255};
const GLubyte yellow        [] = {255, 255,   0, 255};
const GLubyte black         [] = {  0,   0,   0, 255};
const GLubyte orange        [] = {255, 128,   0, 255};
const GLubyte magenta       [] = {255,   0, 255, 255};

const GLubyte LineColorMix0[4][4] = {{  0, 128, 255, 255},
                                     {  0, 128, 255, 196},
                                     {  0, 128, 255, 128},
                                     {  0, 128, 255,  64}};


const GLubyte LineColorMix1[4][4] = {{255,   0,   0, 255},
                                     {255,   0,   0, 196},
                                     {255,   0,   0, 128},
                                     {255,   0,   0,  64}};

inline void set_color(Color4 c)          { glColor4d(c.r, c.g, c.b, c.a); }
inline void set_color(Color4 c, float a) { glColor4d(c.r, c.g, c.b,   a); }


struct Point
{
    float x;
    float y;
    float z;
};


void draw_line(const GLfloat x[3], const GLfloat y[3]);
void draw_line(const Point& p1, const Point& p2);
void draw_line(const float x1, const float y1, const float z1,
               const float x2, const float y2, const float z2);
void draw_line(const float x1, const float y1,
               const float x2, const float y2);

void draw_line2D(const GLfloat x[2], const GLfloat y[2]);

void draw_rect(const GLfloat x1[3], const GLfloat x2[3], const GLfloat x3[3], const GLfloat x4[3]) __attribute__ ((deprecated));

void draw_rect(const float size_x, const float size_y);
void draw_rect(const float x, const float y, const float size_x, const float size_y);

void draw_square(const float size);
void draw_square(const float x, const float y, const float size);

void draw_cube(const GLfloat x1[3], const GLfloat x2[3], const GLfloat x3[3], const GLfloat x4[3],
               const GLfloat x5[3], const GLfloat x6[3], const GLfloat x7[3], const GLfloat x8[3]);

void draw_solid_cube(const float x, const float y, const float z, const float size);
void draw_wire_cube(const float x, const float y, const float z, const float size);

void draw_fill_rect(const float size_x, const float size_y);
void draw_fill_rect(const float x, const float y, const float size_x, const float size_y);

void draw_fill_square(const float size);
void draw_fill_square(const float x, const float y, const float size);



void fill_rect(const GLfloat x1[3], const GLfloat x2[3], const GLfloat x3[3], const GLfloat x4[3]);

void draw_grid2D(const float range, const int lines);
void draw_grid3D(const float range, const int lines);

void draw_text_small(const float x, const float y, const float z, const char *str) __attribute__ ((deprecated));
void draw_text_medium(const float x, const float y, const float z, const char *str) __attribute__ ((deprecated));

void output(const GLfloat x, const GLfloat y, const GLfloat z, const char *text) __attribute__ ((deprecated));

void gl_msg(const float px, const float py, const float pz, const char* format, ...) __attribute__ ((deprecated));

void glprintf(const GLfloat x, const GLfloat py, const GLfloat pz, const float scale_factor, const char* format, ...);
void glprintc(const GLfloat x, const GLfloat py, const GLfloat pz, const float scale_factor, const char* str);
void glprints(const GLfloat x, const GLfloat py, const GLfloat pz, const float scale_factor, const std::string str);

namespace draw {
    void fill_rect(float px, float py, float dx, float dy);
}
#endif /*DRAW_H*/
