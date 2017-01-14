#include <learning/gmes.h>

    GMES::GMES( Expert_Vector& expert
              , double learning_rate
              , bool one_shot_learning )
    : expert(expert)
    , Nmax(expert.size()) /** TODO: check usage of Nmax */
    , min_prediction_error(.0)
    , learning_progress(.0)
    , learning_rate(learning_rate)
    , one_shot_learning(one_shot_learning)
    , learning_enabled(true)
    , number_of_experts(0)
    , winner(0)
    , last_winner(0)
    , recipient(0)
    , to_insert(0)
    , activations(Nmax)
    , new_node(false)
    {
        assert(in_range(gmes_constants::number_of_initial_experts, std::size_t{1}, Nmax));
        for (std::size_t n = 0; n < gmes_constants::number_of_initial_experts; ++n)
            expert[n].create_randomized();
        sts_msg("Created GMES with %u experts and learning rate %.4f", Nmax, learning_rate);
    }

    GMES::~GMES() { dbg_msg("Destroying GMES."); }

    /* gmes main loop
     */
    /** TODO: consider changing the call pattern of private member
     *        methods to functions like x = f(y) and make them
     */
    void GMES::execute_cycle(void)
    {
        /* backup last winner */
        last_winner = winner;

        /* determine new winner */
        winner = determine_winner();

        /* if needed, insert expert before adaptation takes place */
        insert_expert_on_demand();

        /* adapt weights of winner */
        expert[winner].adapt_weights();

        /* estimate learning progress: L = -dE/dt */
        estimate_learning_progress();

        /* shift learning capacity proportional to progress in learning */
        adjust_learning_capacity();

        /* refreshes transitions according to adaptation */
        refresh_transitions();

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


    /* if needed, insert expert
     * before adaptation takes place
     */
    void GMES::insert_expert_on_demand(void)
    {
        new_node = false;

        if (expert[winner].learning_capacity_is_exhausted() && to_insert != winner)
        {
            /* copy weights and payload */
            expert.copy(to_insert, winner, one_shot_learning);

            /* clear transitions emanating from 'to_insert' */
            expert[to_insert].clear_transitions();
            clear_transitions_to(to_insert);

            /* set new transition */
            expert[to_insert].reset_transition(winner);
            winner = to_insert;
            new_node = true;
        }
    }


    /* estimate learning progress: L = -dE/dt
     * TODO: this is a method of the expert, right?
     */
    void GMES::estimate_learning_progress(void)
    {
        const double prediction_error_before_adaption = expert[winner].get_prediction_error();
        learning_progress = prediction_error_before_adaption - expert[winner].make_prediction();
        assert_in_range(learning_progress, 0.0, 1.0);
    }


    /* adjust learning capacity by shifting a fraction of learning capacity
     * away from the winner towards a randomly picked expert
     */
    void GMES::adjust_learning_capacity(void)
    {
        const double delta_capacity = expert[winner].learning_capacity
                                    - expert[winner].learning_capacity * exp(-learning_rate * learning_progress); /** TODO: reorder eq. to: x * (1-exp) */

        recipient = random_index(Nmax);

        expert[winner   ].learning_capacity -= delta_capacity;
        expert[recipient].learning_capacity += delta_capacity;
    }


    /* refreshes transitions according to adaptations
     */
    void GMES::refresh_transitions(void)
    {
        /* invalidate connections emanating from winner */
        for (std::size_t n = 0; n < Nmax; ++n) {
            expert[n].transition[winner] *= exp(-learning_rate * learning_progress); /** TODO: this factor can be computed beforehand and used several times*/
            expert[winner].transition[n] *= exp(-learning_rate * learning_progress);
        }

        /* validate the connection from last_winner to winner */
        expert[winner].reset_transition(last_winner);
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

        if (leakage > 1e-10) //1e-12
            wrn_msg("Learning capacity (%e) is leaking %e", sum_capacity, leakage);
    }


    /* remove all transitions emanating
     * from a a certain node 'to_clear'
     */
    void GMES::clear_transitions_to(std::size_t to_clear)
    {
        for (std::size_t n = 0; n < Nmax; ++n)
            expert[n].transition[to_clear] = .0;
    }
