/* network3D.h */

#ifndef NETWORK3D_H
#define NETWORK3D_H

#include <cassert>
#include <vector>

#include "draw.h"
#include "axes3D.h"
#include "../common/log_messages.h"

class network3D
{
public:
    network3D(unsigned int number_of_nodes, const axes3D& axis)
    : number_of_nodes(number_of_nodes)
    , axis(axis)
    , pointer(0)
    , special_node(0)
    , activated_node(0)
    , n_pos(number_of_nodes)
    , n_size(number_of_nodes)
    , n_edges(number_of_nodes)
    {
        for (unsigned int i = 0; i < number_of_nodes; ++i)
            n_edges.at(i).assign(number_of_nodes, 0);
    }

    void draw(float x_angle, float y_angle) const;

    void update_node(unsigned int n, float x0, float x1, float x2, float s);
    void update_node(unsigned int n, float s);
    void update_edge(unsigned int i, unsigned int j, unsigned char op);
    void update_all_edges_of(unsigned int i, unsigned char op);
    void special    (unsigned int n);
    void activated  (unsigned int n);

    const Point& get_position(unsigned int index) const { assert(index < n_pos.size()); return n_pos[index]; }

private:
    const unsigned int number_of_nodes;
    const axes3D& axis;

    unsigned int pointer;
    unsigned int special_node;
    unsigned int activated_node;

    std::vector<Point> n_pos;
    std::vector<float> n_size;

    /* size == 0, kein Knoten */
    std::vector<std::vector<unsigned char> > n_edges;

};

#endif /*NETWORK3D_H*/
