#ifndef CONTROL_VECTOR_H_INCLUDED
#define CONTROL_VECTOR_H_INCLUDED

#include <string>
#include <memory>
#include <common/basic.h>
#include <common/modules.h>
#include <control/controlparameter.h>

namespace control {


    void randomize_control_parameter(Control_Parameter& params, double std_dev, double max_dev);


class Control_Vector
{
public:
    Control_Vector(std::size_t max_number_of_parameter_sets, const std::string& foldername = "");

    std::size_t              get_number_of_sets(void) const { return controls.size(); }
    const Control_Parameter& get(std::size_t index)   const { return *(controls.at(index)); }

    void add( const std::string& filename
            , const std::size_t number_of_params
            , bool symmetric
            , bool mirrored );

    void add(const std::string& filename);
    void reload(std::size_t index, const std::string& filename);


private:
    const std::size_t max_number_of_parameter_sets;
    std::vector<std::unique_ptr<Control_Parameter> >controls; /**TODO why pointers? */
};


} // namespace control

#endif // CONTROL_VECTOR_H_INCLUDED
