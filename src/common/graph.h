/* graph.h */

#ifndef GRAPH_H
#define GRAPH_H

#include "../common/basic.h"
#include "../common/modules.h"

#include "../draw/draw.h"
#include "../draw/axes3D.h"

///////////////////////////////////
////// TODO REFACTOR THAT /////////
///////////////////////////////////

struct properties
{
    double position[3];
    double value;
    double reward;
    unsigned long uptime;
}; // dont forget to reset variables

class Vertices
{
    friend class Graph;

        int N;  // number of vertices
        bool *vertex_exists;
        properties *property; /* associated properties */

        void create(int n);
        void destroy(int n);
        void check_index(int n, const char* s);


    public:
        Vertices(int number_of_vertices)
        : N(number_of_vertices)
        , vertex_exists((bool*) calloc(N, sizeof(bool))) /* allocate memory for vertices */
        , property((properties*) calloc(N, sizeof(properties)))
        {}

        int get_number_of_vertices(void);
        int exists(int n);

        /* property functions */
        unsigned long get_uptime(int n);
        void clock(void);
        double get_value(int n);
        double get_reward(int n);
        void set_value(int n, double value);
        void set_reward(int n, double value);
        void set_position(int n, double px, double py, double pz);

        /* drawing functions */
        void draw(const axes3D& a, const float x_angle, const float y_angle) const;
};

class Graph
{
    private:
        Vertices *V;
        int N, K;
        int **edge;         // N x K Edges
        int *neighbours;    // current number of neigbours of vertex n
        int index;
        void check_index(int n, const char* s);
        int get_free_connection(int n);
        int is_free(int n);

    public:
        Graph(Vertices *Ver, int max_edges_per_vertex);
        //int get_vertex_state(int n);
        //int get_max_vertices(void);
        Vertices* get_vertex_pointer(void);
        int get_max_edges_per_vertex(void);
        int get_number_of_edges(int n);
        int add_vertex(void);
        void remove_vertex(int n);
        int is_connected_with(int from, int to);
        void add_connection(int from, int to);
        void remove_connection(int from, int to);
        int get_next_connection(int n);
        int is_registered(int n);
        void draw(const axes3D& a, const float x_angle, const float y_angle) const;
};

#endif /*GRAPH_H*/

