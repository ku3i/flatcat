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

    template <typename Float_t>
    struct AdamData {
        static constexpr Float_t b1 = 0.9;
        static constexpr Float_t b2 = 0.999;
        static constexpr Float_t e0 = 10e-8;

        Float_t m, v, bt1, bt2;

        AdamData(void) : m(.0), v(.0), bt1(1.), bt2(1.) { }

        Float_t get(Float_t grad) {
            bt1 *= b1;
            bt2 *= b2;

            m = b1 * m + (1. - b1) * grad;        // 1st momentum
            v = b2 * v + (1. - b2) * grad * grad; // 2nd momentum

            const Float_t M = m / (1. - bt1);
            const Float_t V = v / (1. - bt2);

            return (M / (sqrt(V) + e0));

        }
    };

    typedef copyable_static_vector<copyable_static_vector<AdamData<scalar_t>>> gradient_t;



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

    Vector_t& part0;
    Vector_t& part1;

    twopart_vector(Vector_t& part0, Vector_t& part1) : part0(part0), part1(part1) {
        assert(part0.size() > 0);
        assert(part1.size() > 0);
    }


    twopart_vector& operator=(Vector_t const& vec) {
        assert(vec.size() == size());
        for (std::size_t i = 0; i < size(); ++i)
            this->operator[](i) = vec[i];
        return *this;
    }

    twopart_vector& operator=(twopart_vector const& other) {
        assert(part0.size() == other.part0.size());
        assert(part1.size() == other.part1.size());
        part0 = other.part0;
        part1 = other.part1;
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
    static T transfer(T const& x) { return tanh(x);               } // normal tangens hyperbolicus
    static T derive  (T const& y) { return (1.0 + y) * (1.0 - y); } // this is y' with y=tanh(x)
    static T inverse (T const& x) { return atanh(x); /*log((1+x)/(1-x))/2*/    } // area tangens hyperbolicus = tanh^-1
};

struct Weight_Statistics_t {

    static constexpr float zero_thrsh = 0.001;

    unsigned num = 0, total = 0;
    float avg = 0.f, ext = 0.f, vol = 0.f;

};

template <typename Transfer_t>
class NeuralModel {

    model::vector_t y; // output, predictions
    model::matrix_t W; // Weights
    model::scalar_t E; // prediction error
    model::vector_t d; // delta error (used for back-propagation)

    model::gradient_t G; // Adam's Gradient Statistics

public:
    NeuralModel(std::size_t size_in, std::size_t size_out, double random_weight_range)
    : y(size_out)
    , W(size_out, size_in)
    , E()
    , d(size_out)
    , G(size_out, size_in)
    {
        model::randomize_weights(W, random_weight_range);
    }

    /* calculate and get back-propagated delta error */
    model::vector_t get_backprop_err(void) const
    {
        model::vector_t r(W[0].size());
        for (std::size_t j = 0; j < r.size(); ++j)
            for (std::size_t i = 0; i < d.size(); ++i)
                r[j] += d[i] * W[i][j];
        return r;
    }


    template <typename InputVector_t>
    model::vector_t const& propagate(InputVector_t const& in)
    {
        check_vectors(in, W[0]);

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
    void adapt(InputVector_From_t const& in, InputVector_To_t const& tar, double learning_rate, double regularization_rate)
    {
        check_vectors(tar, y);
        check_vectors(in, W[0]);
        assert_in_range(learning_rate, 0.0, 1.0);
        E = .0; // reset total sum of prediction errors
        for (std::size_t i = 0; i < y.size(); ++i) {
            const model::scalar_t e_i = tar[i] - y[i];
            E += square(e_i);
            d[i] = Transfer_t::derive(y[i]) * e_i; // remember delta error for back-propagation signal
            for (std::size_t j = 0; j < in.size(); ++j)
                //W[i][j] += learning_rate * d[i] * in[j] - regularization_rate * W[i][j];//* sign(W[i][j]);
                /*              target learning                  L1 regularization     */
                W[i][j] += learning_rate * G[i][j].get(d[i] * in[j]) - regularization_rate * W[i][j];
        }
    }

    model::matrix_t const& get_weights() const { return W; }
    model::vector_t const& get_outputs() const { return y; }
    model::scalar_t        get_error  () const { return E/y.size(); }

    void randomize_weights(double random_weight_range) { model::randomize_weights(W, random_weight_range); }


    void constrain_weights(void) {
        for (std::size_t i = 0; i < W.size(); ++i)
            for (std::size_t j = 0; j < W[i].size(); ++j)
                W[i][j] = clip(W[i][j], 5);
    }

    //move to w_statistics class
    Weight_Statistics_t get_weight_statistics(void) const {
        Weight_Statistics_t stat = {};
        stat.num = 0;
        stat.total = W.size() * W[0].size();
        for (std::size_t i = 0; i < W.size(); ++i)
            for (std::size_t j = 0; j < W[i].size(); ++j) {
                const float w = W[i][j];
                stat.avg += w;
                stat.vol += std::abs(w);
                if (fabs(w) > Weight_Statistics_t::zero_thrsh) ++stat.num;
                if (fabs(w) > fabs(stat.ext)) stat.ext = w;
            }

        stat.avg /= stat.total; // normalize by number of weights
        return stat;
    }

}; /* LinearModel */


class InverseNeuralModel {

    model::vector_t y, a; // output, temp
    model::matrix_t M;    // Weights
    model::scalar_t e;    // prediction error

public:
    static constexpr auto& G = TanhTransfer<>::inverse;

    InverseNeuralModel(std::size_t size_in, std::size_t size_out, double random_weight_range)
    : y(size_out)
    , a(size_in)
    , M(size_out, size_in)
    , e()
    {
        model::randomize_weights(M, random_weight_range);
    }

    template <typename InputVector_t>
    model::vector_t const& propagate(InputVector_t const& in)
    {
        check_vectors(in, a);
        for (std::size_t j = 0; j < in.size(); ++j)
            a[j] = G(in[j]);

        for (std::size_t i = 0; i < y.size(); ++i) {
            y[i] = .0;
            for (std::size_t j = 0; j < a.size(); ++j)
                y[i] += M[i][j] * a[j];
        }

        return y;
    }

    /* adapt should follow a propagation to fill y */
    template <typename InputVector_From_t, typename InputVector_To_t>
    void adapt(InputVector_From_t const& in, InputVector_To_t const& tar, double learning_rate, double normalize_rate) {
        check_vectors(tar, y);
        check_vectors(in, a);
        assert_in_range(learning_rate, 0.0, 5.0);

        e = .0;
        for (std::size_t j = 0; j < in.size(); ++j)
            a[j] = G(in[j]);

        for (std::size_t i = 0; i < y.size(); ++i) {
            model::scalar_t err_i = learning_rate * (tar[i] - y[i]);
            e += square(tar[i] - y[i]);
            for (std::size_t j = 0; j < in.size(); ++j)
                M[i][j] += err_i * a[j] - normalize_rate * sign(M[i][j]);
        }
    }

    model::matrix_t const& get_weights() const { return M; }
    model::vector_t const& get_outputs() const { return y; }
    model::scalar_t        get_error  () const { return e/y.size(); }

    void randomize_weights(double random_weight_range) { model::randomize_weights(M, random_weight_range); }


}; /* LinearModel */


template <typename ForwardType, typename InverseType>
class BidirectionalModel {

    ForwardType m_forward;
    InverseType m_inverse;

public:
    BidirectionalModel(std::size_t size_x, std::size_t size_y, double random_weight_range)
    : m_forward(size_x, size_y, random_weight_range)
    , m_inverse(size_y, size_x, random_weight_range)
    {}

    template <typename InputVector_t> model::vector_t const& propagate_forward(InputVector_t const& X) { return m_forward.propagate(X); }
    template <typename InputVector_t> model::vector_t const& propagate_inverse(InputVector_t const& Y) { return m_inverse.propagate(Y); }

    template <typename InputVector_X_t, typename InputVector_Y_t>
    void adapt(InputVector_X_t const& X, InputVector_Y_t const& Y, double learning_rate, double regularization_rate)
    {
        m_forward.adapt(/*from*/X, /*to*/Y, learning_rate, regularization_rate);
        m_inverse.adapt(/*from*/Y, /*to*/X, learning_rate, regularization_rate);
    }

    model::matrix_t const& get_weights        () const { return m_forward.get_weights(); }
    model::matrix_t const& get_weights_inverse() const { return m_inverse.get_weights(); }

    model::vector_t const& get_forward_result() const { return m_forward.get_outputs(); }
    model::vector_t const& get_inverse_result() const { return m_inverse.get_outputs(); }

    model::scalar_t get_forward_error() const { return m_forward.get_error(); }
    model::scalar_t get_inverse_error() const { return m_inverse.get_error(); }


    void randomize_weights(double range) {
        m_forward.randomize_weights(range);
        m_forward.randomize_weights(range);
    }

    model::vector_t get_backprop_gradient(void) const { return m_forward.get_backprop_err(); }

    ForwardType const& get_forward_model(void) const { return m_forward; }
    InverseType const& get_inverse_model(void) const { return m_inverse; }


    void constrain_weights(void) {
        m_forward.constrain_weights();
        m_inverse.constrain_weights();
    }

}; /* BidirectionalModel */


} /* namespace learning */

#endif /* BIDIRECTIONAL_MODELS_H_INCLUDED */
