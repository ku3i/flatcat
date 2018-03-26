#ifndef CONTROL_VECTOR_H_INCLUDED
#define CONTROL_VECTOR_H_INCLUDED

#include <string>
#include <memory>
#include <common/basic.h>
#include <common/modules.h>
#include <common/static_vector.h>
#include <control/controlparameter.h>

namespace control {


void randomize_control_parameter(Control_Parameter& params, double std_dev, double max_dev);


class Control_Vector : public static_vector_interface
{
public:
    Control_Vector(std::size_t max_number_of_parameter_sets, const std::string& foldername = "", bool include_mirrored = false);

    const Control_Parameter& get(std::size_t index) const { return controls.at(index); }

    void add( const std::string& filename
            , const std::size_t number_of_params
            , bool symmetric
            , bool mirrored );

    void add(const std::string& filename);
    void add(const Control_Parameter& params);

    void reload(std::size_t index, const std::string& filename);

    std::size_t size(void) const override { return controls.size(); }
    void copy(std::size_t dst, std::size_t src) override { controls.at(dst) = controls.at(src); }

private:
    const std::size_t max_number_of_parameter_sets;
    std::vector<Control_Parameter> controls;
};


} // namespace control

#endif // CONTROL_VECTOR_H_INCLUDED
