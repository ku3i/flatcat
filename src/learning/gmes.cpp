#include <learning/gmes.h>

    /* gmes main loop
     * TODO: split to smaller subroutines, e.g.
     * 1) Detection part
     * 2) Adaption part
     */
    void GMES::execute_cycle(void)
    {
        /* backup last winner */
        last_winner = winner;
        new_node = false;

        /* determine new winner */
        winner = determine_winner();

        /* if needed, insert expert before adaptation takes place */
        if (expert[winner].learning_capacity_is_exhausted() && to_insert != winner)
        {
            expert[to_insert].copy_from(expert[winner], one_shot_learning); // copy weights
            expert.copy_payload(to_insert, winner);

            /* clear transitions emanating from 'to_insert' */
            expert[to_insert].clear_transitions();
            clear_transitions_to(to_insert);

            /* set new transition */
            expert[to_insert].reset_transition(winner);
            winner = to_insert;
            new_node = true;
        }

        /* adapt weights of winner */
        expert[winner].adapt_weights();

        /* estimate learning progress: L = -dE/dt */
        double prediction_error_before_adaption = expert[winner].get_prediction_error();
        learning_progress = prediction_error_before_adaption - expert[winner].make_prediction();

        //dbg_msg("e: %1.20f %s", learning_progress, (new_node)? "new":"");
        assert_in_range(learning_progress, 0.0, 1.0);

        /* reduce learning capacity proportional to progress in learning */ //TODO move to expert
        /** This should be rethought, 'proportional to the progress in learning' or better 'equal steps'? */
        double delta_capacity = expert[winner].learning_capacity - expert[winner].learning_capacity * exp(-learning_rate * learning_progress); // TODO x * (1-exp)
        expert[winner].learning_capacity -= delta_capacity;

        recipient = random_index(Nmax);
        assert(recipient < Nmax);
        expert[recipient].learning_capacity += delta_capacity;

        /* devaluate connections emanating from n1 */
        for (std::size_t n = 0; n < Nmax; ++n) {
            expert[n].transition[winner] *= exp(-learning_rate * learning_progress); /** TODO: this factor can be computed beforehand and used several times*/
            expert[winner].transition[n] *= exp(-learning_rate * learning_progress);
        }

        /* validate the connection from last_winner to winner */
        /** TODO: validate_transition(); */
        expert[winner].transition[last_winner] = gmes_constants::initial_transition_validation;

        /* count experts */
        number_of_experts = count_existing_experts(); /** could be a member method of experts class */

        /* assert learning_capacity does not leak */
        check_learning_capacity();

        /* choose next expert to insert
         * if all available experts are in use,
         * find and take the one with max. learning capacity */
        to_insert = (number_of_experts < Nmax) ? number_of_experts : arg_max_capacity();

        update_activations();
    }


    /* refreshes the activation vector with the current
     * experts' activations, i.e. inverse prediction error
     */
    void GMES::update_activations(void)
    {
        /* compute activations */
        for (std::size_t n = 0; n < Nmax; ++n)
            activations[n] = expert[n].update_and_get_activation();
    }


    /* compute all predictions and determine
     * the expert with minimal prediction error
     */
    std::size_t GMES::determine_winner(void)
    {
        std::size_t winner = 0;
        double min_error = expert[0].make_prediction();

        for (std::size_t n = 1; n < Nmax; ++n)
        {
            if (expert[n].exists)
            {
                /* compute prediction error and
                * find the best predicting expert */
                if (expert[n].make_prediction() < min_error)
                {
                    winner = n;
                    min_error = expert[n].get_prediction_error();
                }
            }
        }
        assert(winner < Nmax);
        min_prediction_error = min_error;
        return winner;
    }


    /* find the expert for which the
     * learning capacity is maximal.
     */
    std::size_t GMES::arg_max_capacity(void) const
    {
        std::size_t with_max_capacity = 0;
        double max_capacity = 0.0;
        for (std::size_t n = 0; n < Nmax; ++n)
        {
            if (expert[n].exists && expert[n].learning_capacity > max_capacity) {
                max_capacity = expert[n].learning_capacity;
                with_max_capacity = n;
            }
        }
        return with_max_capacity;
    }


    /* count number of experts where
     * the 'exists' flag is set to true
     */
    std::size_t GMES::count_existing_experts(void) const
    {
        std::size_t num_experts = 0;
        for (std::size_t n = 0; n < Nmax; ++n)
            if (expert[n].exists)
                ++num_experts;
        return num_experts;
    }


    /* double check that the total learning
     * capacity stays constants all the time
     */
    void GMES::check_learning_capacity(void) const
    {
        double sum_capacity = 0.0;
        for (std::size_t n = 0; n < Nmax; ++n)
            sum_capacity += expert[n].learning_capacity;

        double leakage = std::abs(sum_capacity - Nmax * gmes_constants::initial_learning_capacity);

        if (leakage > 1e-12)
            wrn_msg("Learning capacity is leaking %e", sum_capacity, leakage);
    }


    /* remove all transitions emanating
     * from a a certain node 'to_clear'
     */
    void GMES::clear_transitions_to(std::size_t to_clear)
    {
        for (std::size_t n = 0; n < Nmax; ++n)
            expert[n].transition[to_clear] = .0;
    }
