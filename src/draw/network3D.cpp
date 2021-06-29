/* network3D.cpp */

#include "network3D.h"

void network3D::draw(float x_angle, float y_angle) const
{
    glPushMatrix();
    glTranslatef(axis.px, axis.py, axis.pz);
    glRotatef(y_angle, 1.0, 0.0, 0.0);
    glRotatef(x_angle, 0.0, 1.0, 0.0);

    /* draw nodes */
    glColor4ub(255,255,255,160);

    for (unsigned int i = 0; i < number_of_nodes; ++i)
        draw_solid_cube(n_pos[i].x, n_pos[i].y, n_pos[i].z, 0.005);

    glColor4ub(255,255,255,96);

    for (unsigned int i = 0; i < number_of_nodes; ++i)
        draw_solid_cube(n_pos[i].x, n_pos[i].y, n_pos[i].z, 0.005 + 0.01 * n_size[i]);


    glColor4ub(255,192,0,128);
    if (n_size[special_node] > 0)
        draw_solid_cube(n_pos[special_node].x, n_pos[special_node].y, n_pos[special_node].z, 0.02);

    glColor4ub(0,128,255,128);
    if (n_size[activated_node] > 0)
        draw_solid_cube(n_pos[activated_node].x, n_pos[activated_node].y, n_pos[activated_node].z, 0.015);

    /* draw edges */
    glLineWidth(1.0f);
    for (unsigned int i = 0; i < number_of_nodes; ++i)
        for (unsigned int j = i+1; j < number_of_nodes; ++j)
            if (n_edges[j][i] || n_edges[i][j])
            {
                glColor4ub(255, 128, 32, std::max(n_edges[j][i],n_edges[i][j]));
                draw_line(n_pos[i], n_pos[j]);
            }
    glPopMatrix();
}

void network3D::special(unsigned int n)
{
    assert(n < number_of_nodes);
    special_node = n;
}

void network3D::activated(unsigned int n)
{
    assert(n < number_of_nodes);
    activated_node = n;
}

void network3D::update_node(unsigned int n, float x0, float x1, float x2, float s)
{
    assert(n < number_of_nodes);
    n_pos[n].x = x0 * axis.width  / 2;
    n_pos[n].y = x1 * axis.height / 2;
    n_pos[n].z = x2 * axis.depth  / 2;
    n_size[n] = s;
}

void network3D::update_node(unsigned int n, float s)
{
    assert(n < number_of_nodes);
    n_size[n] = s;
}

void network3D::update_edge(unsigned int i, unsigned int j, unsigned char op)
{
    assert(i < number_of_nodes && j < number_of_nodes);
    n_edges[i][j] = op;
}

void network3D::update_all_edges_of(unsigned int i, unsigned char op)
{
    assert(i < number_of_nodes);
    for (unsigned int k = 0; k < number_of_nodes; ++k) {
        n_edges[i][k] = op;
        n_edges[k][i] = op;
    }
}

/* network3D.cpp */
