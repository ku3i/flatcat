#ifndef BIDIRECTIONAL_MODELS_H_INCLUDED
#define BIDIRECTIONAL_MODELS_H_INCLUDED

#include <vector>
#include <common/static_vector.h>
#include <common/modules.h>


namespace learning {



namespace model {
    typedef double scalar_t;
    typedef std::vector<scalar_t> vector_t;
    typedef copyable_static_vector<copyable_static_vector<scalar_t>> matrix_t;

    template <typename A_t, typename B_t>
    void check_vectors(A_t const& a, B_t const& b) {
        assertion(a.size() == b.size(), "Incompatible vector lengths %u =/= %u", a.size(), b.size());
    }

    template <typename MatrixType>
    void randomize_weights(MatrixType& mat, double random_weight_range) {
        assert_in_range(random_weight_range, 0.0, 5.0);
        const double normed_std_dev = random_weight_range / sqrt(mat[0].size());
        assert(normed_std_dev != 0.0);

        for (std::size_t i = 0; i < mat.size(); ++i)
            for (std::size_t j = 0; j < mat[i].size(); ++j)
                mat[i][j] = rand_norm_zero_mean(normed_std_dev); // normalized by sqrt(N), N:#inputs

    }
}

template <typename Vector_t>
class twopart_vector {
public:

    Vector_t& part0, part1;

    twopart_vector(Vector_t& part0, Vector_t& part1) : part0(part0), part1(part1) {
        assert(part0.size() > 0);
        assert(part1.size() > 0);
    }

    twopart_vector& operator=(Vector_t const& vec) {
        assert(vec.size() == part0.size() + part1.size());
        for (std::size_t i = 0; i < size(); ++i)
            this->operator[](i) = vec[i];
        return *this;
    }

    std::size_t size(void) const { return part0.size() + part1.size(); }

    model::scalar_t& operator[] (std::size_t index) {
        const auto s0 = part0.size();
        if (index < s0) return part0[index];
        else if (index < s0 + part1.size()) return part1[index-s0];
        else {
            assert(false);
            return part0[0];
        }
    }

    const model::scalar_t& operator[] (std::size_t index) const {
        const auto s0 = part0.size();
        if (index < s0) return part0[index];
        else if (index < s0 + part1.size()) return part1[index-s0];
        else {
            assert(false);
            return part0[0];
        }
    }
};

template <typename T = model::scalar_t>
class LinearTransfer {
public:
    static T transfer(T const& x) { return x; }
    static T derive(T const& /*y*/) { return T{1}; }
};

template <typename T = model::scalar_t>
class TanhTransfer {
public:
    static T transfer(T const& x) { return tanh(x); }
    static T derive(T const& y) { return (1.0 + y) * (1.0 - y); } // this is y' with y=tanh(x)
};


template <typename Transfer_t>
class NeuralModel {

    model::vector_t y; // output, predictions
    model::matrix_t W; // Weights
    model::scalar_t e; // prediction error

public:
    NeuralModel(std::size_t size_in, std::size_t size_out, double random_weight_range)
    : y(size_out)
    , W(size_out, size_in)
    , e()
    {
        model::randomize_weights(W, random_weight_range);
    }

    template <typename InputVector_t>
    model::vector_t const& propagate(InputVector_t const& in)
    {
        model::check_vectors(in, W[0]);

        for (std::size_t i = 0; i < y.size(); ++i) {
            model::scalar_t a = .0;
            for (std::size_t j = 0; j < in.size(); ++j)
                a += W[i][j] * in[j];
            y[i] = Transfer_t::transfer(a);
        }
        return y;
    }

    /* adapt should follow a propagation to fill y */
    template <typename InputVector_From_t, typename InputVector_To_t>
    void adapt(InputVector_From_t const& in, InputVector_To_t const& tar, double learning_rate) {
        model::check_vectors(tar, y);
        model::check_vectors(in, W[0]);
        assert_in_range(learning_rate, 0.0, 5.0);
        e = .0;
        for (std::size_t i = 0; i < y.size(); ++i) {
            model::scalar_t err_i = learning_rate * (tar[i] - y[i]) * Transfer_t::derive(y[i]);
            e += square(tar[i] - y[i]);
            for (std::size_t j = 0; j < in.size(); ++j)
                W[i][j] += err_i * in[j];
        }
    }

    model::matrix_t const& get_weights() const { return W; }
    model::vector_t const& get_outputs() const { return y; }
    model::scalar_t        get_error  () const { return e; }


}; /* LinearModel */


template <typename Transfer_t>
class BidirectionalModel {

    NeuralModel<Transfer_t> m_forward, m_inverse;

public:
    BidirectionalModel(std::size_t size_x, std::size_t size_y, double random_weight_range)
    : m_forward(size_x, size_y, random_weight_range)
    , m_inverse(size_y, size_x, random_weight_range)
    {}

    template <typename InputVector_t> model::vector_t const& propagate_forward(InputVector_t const& X) { return m_forward.propagate(X); }
    template <typename InputVector_t> model::vector_t const& propagate_inverse(InputVector_t const& Y) { return m_inverse.propagate(Y); }

    template <typename InputVector_X_t, typename InputVector_Y_t>
    void adapt(InputVector_X_t const& X, InputVector_Y_t const& Y, double learning_rate)
    {
        m_forward.adapt(/*from*/X, /*to*/Y, learning_rate);
        m_inverse.adapt(/*from*/Y, /*to*/X, learning_rate);
    }

    model::matrix_t const& get_weights        () const { return m_forward.get_weights(); }
    model::matrix_t const& get_weights_inverse() const { return m_inverse.get_weights(); }

    model::vector_t const& get_outputs() const { return m_forward.get_outputs(); }
    model::vector_t const& get_inputs () const { return m_inverse.get_outputs(); }

    model::scalar_t get_forward_error() const { return m_forward.get_error(); }
    model::scalar_t get_inverse_error() const { return m_inverse.get_error(); }

}; /* BidirectionalModel */


} /* namespace learning */

#endif /* BIDIRECTIONAL_MODELS_H_INCLUDED */
