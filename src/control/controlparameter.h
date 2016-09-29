#ifndef CONTROLPARAMETER_H_INCLUDED
#define CONTROLPARAMETER_H_INCLUDED

#include <vector>
#include <memory>
#include <common/basic.h>
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
                              , const Symmetry symmetry //provide this until the file format supports this
                              , const Propagation propagation)
    : parameter(number_of_params)
    , symmetry(symmetry)
    , propagation(propagation)
    {
        sts_msg("Loading controller weights from CSV file:\n   '%s'", filename.c_str());
        file_io::CSV_File<double> csv_file(filename, 1, number_of_params);
        csv_file.read();
        csv_file.get_line(0, parameter);
    }

    explicit Control_Parameter(const std::string& filename)
    : parameter()
    , symmetry()
    , propagation()
    {
        sts_msg("Loading controller weights from DAT file:\n   '%s'", filename.c_str());
        file_io::Data_Reader dat_file(filename);
        assert(dat_file.read("parameter", parameter));

        symmetry    = ("symmetric" == dat_file.read_string("symmetry"   )) ? Symmetry   ::symmetric : Symmetry   ::asymmetric;
        propagation = ("original"  == dat_file.read_string("propagation")) ? Propagation::original  : Propagation::mirrored;
    }

    explicit Control_Parameter( const std::vector<double>& parameter
                              , const Symmetry             symmetry    = asymmetric
                              , const Propagation          propagation = original )
    : parameter(parameter)
    , symmetry(symmetry)
    , propagation(propagation)
    {}

    explicit Control_Parameter() : parameter(), symmetry(), propagation() {}

    Control_Parameter(const Control_Parameter& other)
    : parameter(other.parameter)
    , symmetry(other.symmetry)
    , propagation(other.propagation)
    { dbg_msg("Copying control parameter."); }

    Control_Parameter& operator=(const Control_Parameter& other)
    {
        if (this != &other) // avoid invalid self-assignment
        {
            parameter   = other.parameter;
            symmetry    = other.symmetry;
            propagation = other.propagation;
        }
        return *this;
    }

    ~Control_Parameter() { dbg_msg("Destroying control parameter set."); }

    const std::vector<double>& get_parameter(void) const { return parameter;        }
    const std::size_t          size         (void) const { return parameter.size(); }

    const double& operator[](std::size_t idx) const { assert(idx < parameter.size()); return parameter[idx]; }
          double& operator[](std::size_t idx)       { assert(idx < parameter.size()); return parameter[idx]; }

    bool is_symmetric(void) const { return symmetry    == Symmetry   ::symmetric; }
    bool is_mirrored (void) const { return propagation == Propagation::mirrored;  }

private:

    std::vector<double> parameter;
    Symmetry            symmetry;
    Propagation         propagation;


};

class Control_Vector
{
public:
    Control_Vector(std::size_t max_number_of_parameter_sets, const std::string& foldername = "")
    : max_number_of_parameter_sets(max_number_of_parameter_sets)
    , controls()
    {
        controls.reserve(max_number_of_parameter_sets);
        if (not foldername.empty())
        {
            basic::Filelist files = basic::list_directory(foldername.c_str(), ".dat");
            /* add files successively */
            if (files.size() > 0) {
                for (std::size_t i = 0; i < files.size(); ++i)
                {
                    sts_msg("Adding file %s", files[i].c_str());
                    controls.emplace_back(new Control_Parameter(foldername + files[i]));
                }
            } else
                sts_msg("No files found in '%s'", foldername.c_str());
        } else
            sts_msg("Initialize empty control vector.");
    }

    std::size_t              get_number_of_sets(void) const { return controls.size(); }
    const Control_Parameter& get(std::size_t index)   const { assert(index < controls.size()); return *(controls[index]); }

    void add( const std::string& filename
            , const std::size_t number_of_params
            , Control_Parameter::Symmetry symmetry
            , Control_Parameter::Propagation propagation )
    {
        assert(controls.size() < max_number_of_parameter_sets);
        controls.emplace_back(new Control_Parameter(filename, number_of_params, symmetry, propagation));
    }

    void add(const std::string& filename)
    {
        controls.emplace_back(new Control_Parameter(filename));
    }

    void reload(std::size_t index, const std::string& filename) {
        assert(index < controls.size());
        *(controls[index]) = Control_Parameter(filename);
    }
private:
    const std::size_t max_number_of_parameter_sets;
    std::vector<std::unique_ptr<Control_Parameter> >controls;
};


inline void
randomize_control_parameter(Control_Parameter& params, double std_dev, double max_dev) {
    assert_in_range(std_dev, 0.0, max_dev);
    for (std::size_t i = 0; i < params.size(); ++i)
        params[i] += random_value_norm(.0, std_dev, -max_dev, +max_dev);
}

} // namespace control

#endif // CONTROLPARAMETER_H_INCLUDED
