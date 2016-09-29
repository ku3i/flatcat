/* rl.cpp */

#include "rl.h"

/* these functions distribute the values  *
 * gained from reward through the states */

/* this is asynchrononous value iteration:      *
 * means that values are overridden immediately */
void
value_distribution(Graph *G, float rate, float discount)
{
    Vertices *V = G->get_vertex_pointer();

    for (int n = 0; n < V->get_number_of_vertices(); n++)
    {
        double Value = 0;
        for (int k = 0; k < G->get_number_of_edges(n); k++)
        {
            int m = G->get_next_connection(n);
            Value = fmax(V->get_reward(n) + discount * V->get_value(m), Value);
        }

        double err = Value - V->get_value(n);
        V->set_value(n, V->get_value(n) + rate * err);
    }
}

void
incremental_value_distribution(Graph *G, float rate, float discount, int n)
{
    Vertices *V = G->get_vertex_pointer();

    double Value = 0;
    for (int k = 0; k < G->get_number_of_edges(n); k++)
    {
        int m = G->get_next_connection(n);
        Value = fmax(V->get_reward(n) + discount * V->get_value(m), Value);
    }

    double err = Value - V->get_value(n);
    V->set_value(n, V->get_value(n) + rate * err);

}

void
random_order_value_distribution(Graph *G, float rate, float discount)
{
    Vertices *V = G->get_vertex_pointer();

    int n = floor(random_value(0, V->get_number_of_vertices()));

    double Value = 0;
    for (int k = 0; k < G->get_number_of_edges(n); k++)
    {
        int m = G->get_next_connection(n);
        Value = fmax(V->get_reward(n) + discount * V->get_value(m), Value);
    }

    double err = Value - V->get_value(n);
    V->set_value(n, V->get_value(n) + rate * err);

}

/* rl.cpp */
