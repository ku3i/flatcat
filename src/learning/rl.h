/* rl.h */

#ifndef RL_H
#define RL_H

//#include "../main.h"
#include "../common/graph.h"

void value_distribution(Graph *G, float rate, float discount);
void incremental_value_distribution(Graph *G, float rate, float discount, int n);
void random_order_value_distribution(Graph *G, float rate, float discount);

#endif /*RL_H*/
