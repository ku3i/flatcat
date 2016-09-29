#ifndef GMES_CONSTANTS_H_INCLUDED
#define GMES_CONSTANTS_H_INCLUDED

namespace gmes_constants
{
    const double global_learning_rate = 35.0; // used for edges
    const double local_learning_rate = 0.005; // used for predictors

    const double initial_learning_capacity     = 1.0;
    const double learning_capacity_exhausted   = 0.01;
    const double initial_transition_validation = 1.0;
    const double transition_exist_treshold     = 0.01;
    const double perceptive_width              = 0.1; /** TODO: derive this factor */

    const std::size_t experience_size          = 100;

    /* This setting could be used in combination with initial weights,
     * if we want to insert prior domain knowledge to the system.
     * Otherwise this must be 1. */
    const std::size_t number_of_initial_experts = 1;

    /* general */
    const double random_weight_range = 0.1;
}

#endif // GMES_CONSTANTS_H_INCLUDED
