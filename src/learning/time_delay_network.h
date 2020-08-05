#ifndef TIME_DELAY_NETWORK_H_INCLUDED
#define TIME_DELAY_NETWORK_H_INCLUDED

#include <deque>

#include <common/modules.h>
#include <common/static_vector.h>
#include <control/sensorspace.h>


/** TODO consider a time-delay autoencoder */

namespace learning {

/*
    + time expansion through FIR-type synapses DONE
    + non-linear expansion through tanh or ReLU neurons
    + configurable number of hidden layers
*/



/* FIR-type Synapse
   Expands the input vector v(t) by its recent values v(t-1), v(t-2),... , with each
   tapped delay line having the same size of "number of taps" = N.

   v1(t) ---> [v1(t-1)] --->[v1(t-2)]---> ... --->[v1(t-N)]
   v2(t) ---> [v2(t-1)] --->[v2(t-2)]---> ... --->[v2(t-N)]
         ...                              ...
   vK(t) ---> [vK(t-1)] --->[vK(t-2)]---> ... --->[vK(t-N)]

   plus additional bias

*/
class FIR_type_synapse {

    typedef std::vector<double>  vector_t;
    typedef std::deque<vector_t> buffer_t;   // replace by something which is more efficient

    std::size_t number_of_taps;
    std::size_t input_size;    /* size of raw inputs */
    vector_t    td_inputs;     /* time delayed inputs */
    buffer_t    buffer;        /* structure holding all FIFOs */


    void expand(void) {
        std::size_t p = 0;
        for (auto const& vec : buffer)
            for (auto const& el : vec)
                td_inputs[p++] = el;
        assert(p == td_inputs.size());
    }

public:

    FIR_type_synapse(std::size_t input_size, std::size_t number_of_taps)
    : number_of_taps(number_of_taps)
    , input_size(input_size)
    , td_inputs(input_size*number_of_taps)
    , buffer(input_size)
    {
        /* initialize buffer with zero vectors */
        buffer.assign(number_of_taps, vector_t(input_size));

        assert(buffer.size() == number_of_taps);
        assert(buffer.size()*buffer[0].size() == td_inputs.size());
        dbg_msg("Created FIR-type Synapse of size %u x %u.", input_size, number_of_taps);
    }

    vector_t const& get       (void) const { return td_inputs; }
    buffer_t const& get_buffer(void) const { return buffer; }


    std::size_t size(void) const { return td_inputs.size(); }

    template <typename InputVector_t>
    void propagate(const InputVector_t& inputs)
    {
        assert(input_size == inputs.size());

        /* shift */
        buffer.pop_back();       // remove oldest input vector
        buffer.push_front(inputs.get()); // fill in new values
        assert(buffer.size() == number_of_taps);

        /*
        for (auto const& vec : buffer) {
            printf("[ ");
            for (auto const& el : vec)
                printf("%5.2f ", el);
            printf("]\n");
        }
        printf("\n");
        */

        expand(); // prepare vector for returning in 'get()'
    }



};


/**TODO move the tapped delay line to input space, in order to have all experts share the same tapping line. */

/* Time-Delay feed-forward network
   with a single tanh-type hidden layer.
*/
typedef std::vector<double> vector_t;
typedef copyable_static_vector<copyable_static_vector<double>> matrix_t;

struct TDNWeights {
    TDNWeights(std::size_t inp, std::size_t hid, std::size_t out): hi(hid, inp), oh(out, hid) /*, biases_oh(output.size())*/{}

    matrix_t hi;
    matrix_t oh;
        //TODO vector_t biases_oh;
};


class Timedelay_Network
{

    FIR_type_synapse td_input; /* time delayed inputs */
    vector_t hidden;
    vector_t output;
    vector_t delta;

    TDNWeights weights;

    void randomize(matrix_t& mat, double std_dev) {
        assert_in_range(std_dev, 0.0, 0.1);
        const double normed_stddev = std_dev / sqrt(mat[0].size()); // normalize by sqrt(N), N:#inputs
        for (std::size_t i = 0; i < mat.size(); ++i)
            for (std::size_t j = 0; j < mat[i].size(); ++j)
                mat[i][j] = rand_norm_zero_mean(normed_stddev);
    }

    void propagate_forward(vector_t& out, matrix_t const& mat, vector_t const& in) {
        assert(out.size() == mat.size());
        assert( in.size() == mat[0].size());

        for (std::size_t i = 0; i < out.size(); ++i) {
            double act = 0.;
            for (std::size_t j = 0; j < in.size(); ++j)
                act += mat[i][j] * in[j];
            out[i] = act;
        }
    }

public:

    Timedelay_Network( std::size_t input_size
                     , std::size_t target_size
                     , std::size_t hidden_size
                     , std::size_t number_of_taps
                     , double rnd_init_range)
    : td_input(input_size, number_of_taps)
    , hidden(hidden_size)
    , output(target_size)
    , delta (output.size())
    , weights(td_input.size(), hidden.size(), output.size())
    {
        randomize_weight_matrix(rnd_init_range);
    }


    void propagate(void) {
        /* time delayed input to hidden layer */
        propagate_forward(hidden, weights.hi, td_input.get());
        vector_tanh(hidden);

        /* hidden to output layer */
        propagate_forward(output, weights.oh, hidden);
        vector_tanh(output);
    }

    template <typename InputVector_t>
    void propagate_and_shift(const InputVector_t& inputs)
    {
        /* shift and time expand next inputs */
        td_input.propagate(inputs);
        propagate();
    }

    template <typename TargetVector_t>
    void adapt(const TargetVector_t& targets, double learning_rate)
    {
        assert_in_range(learning_rate, 0.0, 0.5);
        assert(targets.size() == output.size());

        /* delta error */
        for (std::size_t i = 0; i < output.size(); ++i)
            delta[i] = (targets[i] - output[i]) * tanh_(output[i]);


        /* estimate error for hidden units and adapt input to hidden weights */
        for (std::size_t i = 0; i < hidden.size(); ++i)
        {
            /* estimate 'hidden' errors */
            double error_i = .0;
            for (std::size_t j = 0; j < output.size(); ++j)
                error_i += weights.oh[j][i] * delta[j];

            /* adapt weights.hi */
            const double delta_i = error_i * tanh_(hidden[i]);
            for (std::size_t j = 0; j < output.size(); ++j)
                weights.hi[i][j] += learning_rate * delta_i * td_input.get()[j];
        }


        /* adapt weight_oh */
        for (std::size_t i = 0; i < output.size(); ++i)
            for (std::size_t j = 0; j < hidden.size(); ++j)
                weights.oh[i][j] += learning_rate * delta[i] * hidden[j];
    }

    /*
        x_j : inputs[j]
        t_j : targets[j]
        y_j : outputs[j]
        h_i : hidden[i]

        eta : learning_rate

        second layer weight change:
        error : e_j = (t_j - y_j)
        delta : d2_j = e_j * (1.0 + y_j) * (1.0 - y_j)
        weight: dw2_ji = eta * d2_j * h_i

        first layer weight change:
        error : eh_i = sum_j w2_ji * d2_j
        delta : d1_i = eh_i * (1.0 + h_i) * (1.0 - h_i)
        weight: dw1_ji = eta * d1_i * x_j

    */

    vector_t const& get_outputs() const { return output; }
    vector_t const& get_hidden() const { return hidden; }
    TDNWeights const& get_weights() const { return weights; }

    void randomize_weight_matrix(double random_weight_range)
    {
        randomize(weights.hi, random_weight_range);
        randomize(weights.oh, random_weight_range);
    }
};




} // namespace learning

#endif // TIME_DELAY_NETWORK_H_INCLUDED
