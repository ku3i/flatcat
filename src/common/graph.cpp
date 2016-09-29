/* graph.cpp */

#include "graph.h"

int Vertices::get_number_of_vertices()
{
    return N;
}

int Vertices::exists(int n)
{
    check_index(n, "exists");
    return vertex_exists[n];
}

void Vertices::create(int n)
{
    check_index(n, "create");
    if (!vertex_exists[n])
        vertex_exists[n] = 1;
    else
        dbg_msg("Cannot create. Vertex with index %d already exists.", n);
}

void Vertices::destroy(int n)
{
    check_index(n, "destroy");
    if (vertex_exists[n])
    {
        vertex_exists[n] = 0;
        property[n].uptime = 0;
        property[n].value = 0;
        property[n].reward = 0;
        property[n].position[0] = 0.0f;
        property[n].position[1] = 0.0f;
        property[n].position[2] = 0.0f;
    } else
        dbg_msg("Cannot destroy. Vertex with index %d does not exist.", n);
}

void Vertices::clock(void)
{
    for (int n = 0; n < N; ++n)
        if (vertex_exists[n])
            ++property[n].uptime;
}

/* checks if index is out of bounds */
inline void Vertices::check_index(int n, const char* s)
{
    if (n < 0 || n >= N)
        err_msg(__FILE__, __LINE__, "Index out of bounds error in class 'Vertices', method '%s'.\nIndex %d is not in bounds (0, %d)\n", s, n, N-1);
}

void Vertices::set_position(int n, double px, double py, double pz)
{
    check_index(n, "set_position");
    if (vertex_exists[n])
    {
        property[n].position[0] = px;
        property[n].position[1] = py;
        property[n].position[2] = pz;

        if (abs(px) > 1.0f) dbg_msg("Vertex %d position [x = %1.2f] out of bounds [-1,+1].", n, px);
        if (abs(py) > 1.0f) dbg_msg("Vertex %d position [y = %1.2f] out of bounds [-1,+1].", n, py);
        if (abs(py) > 1.0f) dbg_msg("Vertex %d position [z = %1.2f] out of bounds [-1,+1].", n, pz);
    }
    else dbg_msg("Cannot set position. Vertex with index %d does not exists.", n);

}

void Vertices::set_value(int n, double value)
{
    check_index(n, "set_value");
    if (vertex_exists[n]) property[n].value = value;
    else dbg_msg("Cannot set value. Vertex with index %d does not exists.", n);
}

void Vertices::set_reward(int n, double reward)
{
    check_index(n, "set_value");
    if (vertex_exists[n]) property[n].reward = reward;
    else dbg_msg("Cannot set reward. Vertex with index %d does not exists.", n);
}


unsigned long Vertices::get_uptime(int n)
{
    // 1,36 Jahre = long @ 100Hz
    check_index(n, "get_uptime");
    return property[n].uptime;
}

double Vertices::get_value(int n)
{
    check_index(n, "get_value");
    return property[n].value;
}

double Vertices::get_reward(int n)
{
    check_index(n, "get_reward");
    return property[n].reward;
}

void Vertices::draw(const axes3D& a, const float x_angle, const float y_angle) const
{
    glPushMatrix();
    //glTranslatef(a.px,a.py,a.pz);
    glRotatef(y_angle, 1.0, 0.0, 0.0);
    glRotatef(x_angle, 0.0, 1.0, 0.0);

    for (int n = 0; n < N; ++n)
        if (vertex_exists[n])
        {
            glColor4f(1.0, 1.0, 1.0, 0.3);
            draw_solid_cube(
                property[n].position[0] * a.width  + a.px,
                property[n].position[1] * a.height + a.py,
                property[n].position[2] * a.depth  + a.pz,
                0.05 * clip(fabs(property[n].value)) + 0.01);
            glColor4f(1.0, 0.0, 1.0, 0.3);
            draw_wire_cube(
                property[n].position[0] * a.width  + a.px,
                property[n].position[1] * a.height + a.py,
                property[n].position[2] * a.depth  + a.pz,
                0.05 * clip(fabs(property[n].reward)) + 0.01);
        }
    glPopMatrix();
}



/* Graph */

inline int index(int n);

Vertices* Graph::get_vertex_pointer(void)
{
    return V;
}

Graph::Graph(Vertices *Ver, int max_edges_per_vertex)
: V(Ver)
, N(V->get_number_of_vertices())
, K(max_edges_per_vertex)
{
    if (N <= 0) err_msg(__FILE__, __LINE__, "Number of vertices must be greater than 0");
    if ((K > N) || (K < 1)) err_msg(__FILE__, __LINE__, "Number of edges per vertex must be between 1 and the number of vertices.");

    /* set index of first vertex */
    index = 0;

    /* initialize neighbour counter */
    neighbours = (int*) calloc(N, sizeof(int));

    /* allocate memory for edges */
	edge = (int **) malloc(N * sizeof(int *));
    if (NULL == edge) {
		fprintf(stderr, "Error while allocating memory for edges in class 'Graph' (1).\n");
		exit(-1);
	}

	for (int i = 0; i < N; ++i) {
		edge[i] = (int *) calloc(K, sizeof(int));
		if (NULL == edge[i]) {
			fprintf(stderr, "Error while allocating memory for edges in class 'Graph' (2).\n");
			exit(-1);
		}
		for (int k = 0; k < K; ++k)
            edge[i][k] = -1;
	}
	dbg_msg("Finished creating Graph with %d vertices and %d x %d edges.", N, N, K);
}

/*int Graph::get_max_vertices(void)
{
    return N;
}*/

int Graph::get_max_edges_per_vertex(void)
{
    return K;
}

int Graph::get_number_of_edges(int n)
{
    check_index(n, "get_number_of_edges");
    return neighbours[n];
}

int Graph::is_connected_with(int from, int to)
{
    if (!V->exists(from))
    {
        dbg_msg("Vertex %d does not exist. (is_connected_with)", from);
        return 0;
    }
    if (!V->exists(to))
    {
        dbg_msg("Vertex %d does not exist. (is_connected_with)", to);
        return 0;
    }

    for (int k = 0; k < K; k++)
        if (edge[from][k] == to) return 1;
    return 0;
}

void Graph::add_connection(int from, int to)
{
    if (!V->exists(from))
    {
        dbg_msg("Vertex %d does not exist. (connect_to)", from);
        return;
    }
    if (!V->exists(to))
    {
        dbg_msg("Vertex %d does not exist. (connect_to)", to);
        return;
    }

    int k = get_free_connection(from);

    if (k >= 0)
    {
        edge[from][k] = to;
        neighbours[from]++;
    } else dbg_msg("No free edge at vertex %d. (connect_to)", from);
}

void Graph::remove_connection(int from, int to)
{
    if (!V->exists(from))
    {
        dbg_msg("Vertex %d does not exist. (connect_to)", from);
        return;
    }
    if (!V->exists(to))
    {
        dbg_msg("Vertex %d does not exist. (connect_to)", to);
        return;
    }

    for (int k = 0; k < K; k++)
        if (edge[from][k] == to) edge[from][k] = -1;
    return;
}
/* checks for free space in connection list
   returns 1st available index or -1 */
int Graph::get_free_connection(int n)
{
    for (int k = 0; k < K; k++)
        if (-1 == edge[n][k]) return k;
    return -1;
}

int Graph::get_next_connection(int n)
{
    static int c = -1; // index counter
    check_index(n, "get_next_connection");

    if (neighbours[n] > 0)
    {
        for (int k = 0; k < K; k++) // max k searches
        {
            c++;
            if (c >= K) c = 0;
            if (edge[n][c] >= 0) return edge[n][c];
        }
        err_msg(__FILE__, __LINE__, "Inconsistency in class 'Graph', method 'get_next_connection'.\n");
    }
    return -1;
}

int Graph::add_vertex(void)
{
    int n = -1;
    for (int i = 0; i < N; i++) // max N searches
    {
        if (!V->exists(index))
        {
            n = index;
            break; // stop searching
        }
        index++;
        if (index >= N) index = 0;
    }

    if (-1 == n)
        dbg_msg("Vertex list is full (max %d vertices).", N);
    else
        V->create(n);

    return n;
}

void Graph::remove_vertex(int n)
{
    check_index(n, "remove_vertex");
    if (V->exists(n))
    {
        V->destroy(n);
        if (n < index) index = n;

        // remove all outgoing connections
        for (int k = 0; k < K; k++) edge[n][k] = -1;
        neighbours[n] = 0;

        // remove all incoming connections
        for (int i = 0; i < N; i++)
            for (int k = 0; k < K; k++)
                if (edge[i][k] == n)
                {
                    edge[i][k] = -1;
                    neighbours[i]--;
                }
    }
    else
        dbg_msg("Vertex %d does not exist. (remove vertex)", n);

}

/* checks if index is out of bounds */
inline void Graph::check_index(int n, const char* s)
{
    if (n < 0 || n >= N)
        err_msg(__FILE__, __LINE__, "Index out of bounds error in class 'Graph', method '%s'.\nIndex %d is not in bounds (0, %d)\n", s, n, N-1);
}


void Graph::draw(const axes3D& a, const float x_angle, const float y_angle) const
{
    glPushMatrix();
    //glTranslatef(a.px, a.py, a.pz);
    glRotatef(y_angle, 1.0, 0.0, 0.0);
    glRotatef(x_angle, 0.0, 1.0, 0.0);

    glColor4f(1.0, 1.0, 1.0, 0.2);
    glBegin(GL_LINES);
    for (int n = 0; n < N; n++)
        for (int k = 0; k < K; k++)
        {
            int m = edge[n][k];
            if (m >= 0) // connection exits
            {
                glVertex3f(
                    V->property[n].position[0] * a.width  + a.px,
                    V->property[n].position[1] * a.height + a.py,
                    V->property[n].position[2] * a.depth  + a.pz);
                glVertex3f(
                    V->property[m].position[0] * a.width  + a.px,
                    V->property[m].position[1] * a.height + a.py,
                    V->property[m].position[2] * a.depth  + a.pz);
            }
        }
	glEnd();
	glPopMatrix();
}
/* graph.cpp */

