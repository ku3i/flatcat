#ifndef CONTROLPARAMETER_H_INCLUDED
#define CONTROLPARAMETER_H_INCLUDED

#include <vector>
#include <memory>
#include <common/basic.h>
#include <common/modules.h>
#include <common/log_messages.h>
#include <common/noncopyable.h>
#include <common/file_io.h>
#include <common/datareader.h>

namespace control {

template <typename T> inline std::vector<T>& operator+=(std::vector<T>& lhs, std::vector<T>const & rhs);
template <typename T> inline std::vector<T>& operator-=(std::vector<T>& lhs, std::vector<T>const & rhs);
template <typename T> inline std::vector<T>& operator*=(std::vector<T>& lhs, T const& rhs);
template <typename T> inline std::vector<T>& operator/=(std::vector<T>& lhs, T const& rhs);

class Control_Parameter : public noncopyable
{
public:
    typedef std::vector<std::vector<double>> matrix_t;

    explicit Control_Parameter( const std::string& filename
                              , std::size_t number_of_params
                              , bool symmetric
                              , bool mirrored = false
                              , unsigned robot_id = 0
                              , uint64_t rnd_init = 0 );

    explicit Control_Parameter(const std::string& filename);

    explicit Control_Parameter( const std::vector<double>& parameter
                              , bool symmetric = false
                              , bool mirrored = false
                              , unsigned robot_id = 0
                              , uint64_t rnd_init = 0 );

    void set_from_matrix( matrix_t const& weights );

    explicit Control_Parameter() : parameter(), symmetric(), mirrored(), robot_id(), rnd_init() { assert(false and "Should not be used."); }

    Control_Parameter(const Control_Parameter& other);

    Control_Parameter& operator=(const Control_Parameter& other);

    ~Control_Parameter() { /*dbg_msg("Destroying control parameters.");*/ }

    const std::vector<double>& get_parameter(void) const { return parameter;        }
          std::vector<double>& set_parameter(void)       { return parameter;        }
          std::size_t          size         (void) const { return parameter.size(); }

    const double& operator[](std::size_t idx) const { return parameter.at(idx); }
          double& operator[](std::size_t idx)       { return parameter.at(idx); }

    bool is_symmetric(void) const { return symmetric; }
    bool is_mirrored (void) const { return mirrored;  }

    unsigned get_robot_id(void) const { return robot_id; }
    uint64_t get_rnd_init(void) const { return rnd_init; }

    void add_gaussian_noise(double sigma);

    void print() const;

    void save_to_file(const std::string& filename, std::size_t id) const;

    /* arithmetic overloads */

    Control_Parameter& operator+=(const Control_Parameter& rhs) {
        this->parameter += rhs.parameter;
        return *this;
    }
    Control_Parameter& operator-=(const Control_Parameter& rhs) {
        this->parameter -= rhs.parameter;
        return *this;
    }
    Control_Parameter& operator*=(const double& rhs)  {
        this->parameter *= rhs;
        return *this;
    }
    Control_Parameter& operator/=(const double& rhs)  {
        this->parameter /= rhs;
        return *this;
    }


private:

    std::vector<double> parameter;
    bool                symmetric;
    bool                mirrored;
    unsigned            robot_id;
    uint64_t            rnd_init;

};


inline Control_Parameter operator+(Control_Parameter lhs, const Control_Parameter& rhs) { lhs += rhs; return lhs; }
inline Control_Parameter operator-(Control_Parameter lhs, const Control_Parameter& rhs) { lhs -= rhs; return lhs; }
inline Control_Parameter operator*(Control_Parameter lhs, const double&            rhs) { lhs *= rhs; return lhs; }
inline Control_Parameter operator*(const double&     lhs,       Control_Parameter  rhs) { rhs *= lhs; return rhs; }
inline Control_Parameter operator/(Control_Parameter lhs, const double&            rhs) { lhs /= rhs; return lhs; }

template <typename T>
inline std::vector<T>& operator+=(std::vector<T>& lhs, std::vector<T>const& rhs) {
    assert(lhs.size() == rhs.size());
    for (std::size_t i = 0; i < lhs.size(); ++i)
        lhs[i] += rhs[i];
    return lhs;
}

template <typename T>
inline std::vector<T>& operator-=(std::vector<T>& lhs, std::vector<T>const& rhs) {
    assert(lhs.size() == rhs.size());
    for (std::size_t i = 0; i < lhs.size(); ++i)
        lhs[i] -= rhs[i];
    return lhs;
}

template <typename T>
inline std::vector<T>& operator*=(std::vector<T>& lhs, T const& rhs) {
    for (T& e: lhs) e *= rhs;
    return lhs;
}

template <typename T>
inline std::vector<T>& operator/=(std::vector<T>& lhs, T const& rhs) {
    for (T& e: lhs) e /= rhs;
    return lhs;
}



} // namespace control

#endif // CONTROLPARAMETER_H_INCLUDED
