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

class Control_Parameter : public noncopyable
{
public:
    enum Symmetry { symmetric, asymmetric };
    enum Propagation { original, mirrored };

    explicit Control_Parameter( const std::string& filename
                              , const std::size_t number_of_params
                              , const Symmetry symmetry
                              , const Propagation propagation = Propagation::original );

    explicit Control_Parameter(const std::string& filename);

    explicit Control_Parameter( const std::vector<double>& parameter
                              , const Symmetry             symmetry    = asymmetric
                              , const Propagation          propagation = original );

    explicit Control_Parameter() : parameter(), symmetry(), propagation() {} /** TODO should not be used */

    Control_Parameter(const Control_Parameter& other);

    Control_Parameter& operator=(const Control_Parameter& other);

    ~Control_Parameter() { dbg_msg("Destroying control parameter set."); }

    const std::vector<double>& get_parameter(void) const { return parameter;        }
    const std::size_t          size         (void) const { return parameter.size(); }

    const double& operator[](std::size_t idx) const { return parameter.at(idx); }
          double& operator[](std::size_t idx)       { return parameter.at(idx); }

    bool is_symmetric(void) const { return symmetry    == Symmetry   ::symmetric; }
    bool is_mirrored (void) const { return propagation == Propagation::mirrored;  }

private:

    std::vector<double> parameter;
    Symmetry            symmetry;
    Propagation         propagation;


};


} // namespace control

#endif // CONTROLPARAMETER_H_INCLUDED
