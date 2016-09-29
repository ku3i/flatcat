/* draw.c */

#include "draw.h"

void set_color(Color4 c)          { glColor4d(c.r, c.g, c.b, c.a); }
void set_color(Color4 c, float a) { glColor4d(c.r, c.g, c.b,   a); }

void draw_line(const GLfloat x[3], const GLfloat y[3])
{
   glBegin(GL_LINE_STRIP);
   glVertex3fv(x);
   glVertex3fv(y);
   glEnd();
}

void draw_line(const float x1, const float y1, const float z1,
               const float x2, const float y2, const float z2)
{
   glBegin(GL_LINE_STRIP);
   glVertex3f(x1, y1, z1);
   glVertex3f(x2, y2, z2);
   glEnd();
}

void draw_line(const float x1, const float y1,
               const float x2, const float y2)
{
   glBegin(GL_LINE_STRIP);
   glVertex2f(x1, y1);
   glVertex2f(x2, y2);
   glEnd();
}

void draw_line(const Point& p1, const Point& p2)
{
   glBegin(GL_LINE_STRIP);
   glVertex3f(p1.x, p1.y, p1.z);
   glVertex3f(p2.x, p2.y, p2.z);
   glEnd();
}

void draw_line2D(const GLfloat x[2], const GLfloat y[2])
{
   glBegin(GL_LINE_STRIP);
   glVertex2fv(x);
   glVertex2fv(y);
   glEnd();
}

void draw_rect(const GLfloat x1[3], const GLfloat x2[3], const GLfloat x3[3], const GLfloat x4[3])
{
   glBegin(GL_LINE_STRIP);
   glVertex3fv(x1);
   glVertex3fv(x2);
   glVertex3fv(x3);
   glVertex3fv(x4);
   glVertex3fv(x1);
   glEnd();
}

void draw_cube(const GLfloat x1[3], const GLfloat x2[3], const GLfloat x3[3], const GLfloat x4[3],
               const GLfloat x5[3], const GLfloat x6[3], const GLfloat x7[3], const GLfloat x8[3])
{
   glBegin(GL_LINE_STRIP);
   glVertex3fv(x1);
   glVertex3fv(x2);
   glVertex3fv(x3);
   glVertex3fv(x4);
   glVertex3fv(x1);
   glVertex3fv(x5);
   glVertex3fv(x6);
   glVertex3fv(x7);
   glVertex3fv(x8);
   glVertex3fv(x5);
   glEnd();
   glBegin(GL_LINE_STRIP);
   glVertex3fv(x2);
   glVertex3fv(x6);
   glEnd();
   glBegin(GL_LINE_STRIP);
   glVertex3fv(x3);
   glVertex3fv(x7);
   glEnd();
   glBegin(GL_LINE_STRIP);
   glVertex3fv(x4);
   glVertex3fv(x8);
   glEnd();


}

void draw_wire_cube(const float x, const float y, const float z, const float size)
{
    glPushMatrix();
    glTranslatef(x, y, z);
    glutWireCube(size);
    glPopMatrix();
}

void draw_solid_cube(const float x, const float y, const float z, const float size)
{
    glPushMatrix();
    glTranslatef(x, y, z);
    glutSolidCube(size);
    glPopMatrix();
}


void draw_grid2D(const float range, const int lines)
{
    float v0[2], v1[2];
    float F = range/lines;

    for (int i = -lines; i <= lines; ++i)
    {
        v0[0] = i*F;
        v0[1] = -range;
        v1[0] = i*F;
        v1[1] = +range;
        draw_line2D(v0, v1);

        v0[0] = -range;
        v0[1] = i*F;
        v1[0] = +range;
        v1[1] = i*F;
        draw_line2D(v0, v1);
    }
}

void draw_grid3D(const float range, const int LINES)
{
    float v0[3], v1[3];
    float F = range/LINES;

    for (int i = -LINES; i <= LINES; i++)
        for (int j = -LINES; j <= LINES; j++)
        {
            v0[0] = i*F;
            v0[1] = j*F;
            v0[2] = -range;
            v1[0] = i*F;
            v1[1] = j*F;
            v1[2] = +range;
            draw_line(v0, v1);

            v0[0] = j*F;
            v0[1] = -range;
            v0[2] = i*F;
            v1[0] = j*F;
            v1[1] = +range;
            v1[2] = i*F;
            draw_line(v0, v1);

            v0[0] = -range;
            v0[1] = i*F;
            v0[2] = j*F;
            v1[0] = +range;
            v1[1] = i*F;
            v1[2] = j*F;
            draw_line(v0, v1);
        }
}

void
draw_fill_rect(const float size_x, const float size_y)
{
    const float sx = size_x/2;
    const float sy = size_y/2;

    glBegin(GL_QUADS);
    glVertex2f( sx, sy);
    glVertex2f(-sx, sy);
    glVertex2f(-sx,-sy);
    glVertex2f( sx,-sy);
    glEnd();
}

void
draw_fill_square(const float size)
{
    draw_fill_rect(size, size);
}

void
draw_fill_square(const float x, const float y, const float size)
{
    glPushMatrix();
    glTranslatef(x, y, .0f);
    draw_fill_rect(size, size);
    glPopMatrix();
}

void draw_fill_rect(const float x, const float y, const float size_x, const float size_y)
{
    glPushMatrix();
    glTranslatef(x, y, .0f);
    draw_fill_rect(size_x, size_y);
    glPopMatrix();
}

void draw_rect(const float size_x, const float size_y)
{
    const float sx = size_x/2;
    const float sy = size_y/2;

    glBegin(GL_LINE_STRIP);
    glVertex2f( sx, sy);
    glVertex2f(-sx, sy);
    glVertex2f(-sx,-sy);
    glVertex2f( sx,-sy);
    glVertex2f( sx, sy);
    glEnd();
}

void draw_square(const float size)
{
    draw_rect(size, size);
}

void draw_square(const float x, const float y, const float size)
{
    glPushMatrix();
    glTranslatef(x, y, .0f);
    draw_rect(size, size);
    glPopMatrix();
}

void draw_rect(const float x, const float y, const float size_x, const float size_y)
{
    glPushMatrix();
    glTranslatef(x, y, .0f);
    draw_rect(size_x, size_y);
    glPopMatrix();
}


void fill_rect(const GLfloat x1[3], const GLfloat x2[3], const GLfloat x3[3], const GLfloat x4[3])
{
   glBegin(GL_TRIANGLE_STRIP);
   glVertex3fv(x2);
   glVertex3fv(x1);
   glVertex3fv(x3);
   glVertex3fv(x4);
   glEnd();
}

//GLUT_BITMAP_9_BY_15
//GLUT_BITMAP_8_BY_13
//GLUT_BITMAP_HELVETICA_10
//GLUT_BITMAP_HELVETICA_18
void draw_text_small(const float x, const float y, const float z, const char *str)
{
   glRasterPos3f(x,y,z);
   for (unsigned int j = 0; j < strlen(str); j++)
     glutBitmapCharacter(GLUT_BITMAP_8_BY_13,str[j]);

}


void output(const GLfloat x, const GLfloat y, const GLfloat z, char *text)
{
    char *p;

    glPushMatrix();
    glTranslatef(x, y, z);
    for (p = text; *p; p++)
        glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
    glPopMatrix();
}

void draw_text_medium(const float x, const float y, const float z, const char *str)
{
   glRasterPos3f(x,y,z);
   for (unsigned int j = 0; j < strlen(str); j++)
     glutBitmapCharacter(GLUT_BITMAP_9_BY_15,str[j]);
}

void gl_msg(const float px, const float py, const float pz, const char* format, ...)
{
    char text[256];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(text, 256, format, args);
    va_end(args);

    if (length > 0) {
        glRasterPos3f(px, py, pz);
        for (int i = 0; i < length; ++i)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
    }

}

void
glprintf(const GLfloat x, const GLfloat y, const GLfloat z, const float line_height, const char* format, ...)
{
    char text[256];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(text, 256, format, args);
    va_end(args);

    if (length > 0)
    {
        glPushMatrix();
        glTranslatef(x, y, z);
        const float size = line_height / 128.0;
        glScalef(size, size, size);
        glLineWidth(1.0f);
        for (int i = 0; i < length; ++i)
            glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, text[i]);
        glPopMatrix();
    }
}

void
glprintc(const GLfloat x, const GLfloat y, const GLfloat z, const float line_height, const char* str)
{
    if (strlen(str) > 0)
    {
        glPushMatrix();
        glTranslatef(x, y, z);
        const float size = line_height / 128.0;
        glScalef(size, size, size);
        glLineWidth(1.0f);
        for (unsigned int i = 0; i < strlen(str); ++i)
            glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, str[i]);
        glPopMatrix();
    }
}

void
glprints(const GLfloat x, const GLfloat y, const GLfloat z, const float line_height, const std::string str)
{
    if (str.size() > 0)
    {
        glPushMatrix();
        glTranslatef(x, y, z);
        const float size = line_height / 128.0;
        glScalef(size, size, size);
        glLineWidth(1.0f);
        for (std::size_t i = 0; i < str.size(); ++i)
            glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, str[i]);
        glPopMatrix();
    }
}

namespace draw {

void fill_rect(float px, float py, float dx, float dy) {
    glBegin(GL_QUADS);
    glVertex2f(px   , py);
    glVertex2f(px+dx, py);
    glVertex2f(px+dx, py+dy);
    glVertex2f(px   , py+dy);
    glEnd();
}

}

/* draw.c */

