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

    explicit Control_Parameter( const std::string& filename
                              , std::size_t number_of_params
                              , bool symmetric
                              , bool mirrored = false );

    explicit Control_Parameter(const std::string& filename);

    explicit Control_Parameter( const std::vector<double>& parameter
                              , bool symmetric = false
                              , bool mirrored = false );

    explicit Control_Parameter() : parameter(), symmetric(), mirrored() { assert(false and "Should not be used."); }

    Control_Parameter(const Control_Parameter& other);

    Control_Parameter& operator=(const Control_Parameter& other);

    ~Control_Parameter() { dbg_msg("Destroying control parameters."); }

    const std::vector<double>& get_parameter(void) const { return parameter;        }
    const std::size_t          size         (void) const { return parameter.size(); }

    const double& operator[](std::size_t idx) const { return parameter.at(idx); }
          double& operator[](std::size_t idx)       { return parameter.at(idx); }

    bool is_symmetric(void) const { return symmetric; }
    bool is_mirrored (void) const { return mirrored;  }

private:

    std::vector<double> parameter;
    bool                symmetric;
    bool                mirrored;


};


} // namespace control

#endif // CONTROLPARAMETER_H_INCLUDED
