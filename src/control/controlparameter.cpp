
#include <control/controlparameter.h>

namespace control {


    Control_Parameter::Control_Parameter( const std::string& filename
                                        , const std::size_t number_of_params
                                        , const Symmetry symmetry
                                        , const Propagation propagation )
    : parameter()
    , symmetry(symmetry)
    , propagation(propagation)
    {
        sts_msg("Loading controller weights from CSV file:\n   '%s'", filename.c_str());
        file_io::CSV_File<double> csv_file(filename, 1, number_of_params);
        if (csv_file.read()) {
            parameter.assign(number_of_params, 0.0);
            csv_file.get_line(0, parameter);
        } else
            wrn_msg("Could not read from file: %s", filename.c_str());
    }


    Control_Parameter::Control_Parameter(const std::string& filename)
    : parameter()
    , symmetry()
    , propagation()
    {
        sts_msg("Loading controller weights from DAT file:\n   '%s'", filename.c_str());
        file_io::Data_Reader dat_file(filename);
        assert(dat_file.read("parameter", parameter)); /** TODO: remove assert dependence */

        symmetry    = ("symmetric" == dat_file.read_string("symmetry"   )) ? Symmetry   ::symmetric : Symmetry   ::asymmetric;
        propagation = ("original"  == dat_file.read_string("propagation")) ? Propagation::original  : Propagation::mirrored;
    }


    Control_Parameter::Control_Parameter( const std::vector<double>& parameter
                                        , const Symmetry             symmetry
                                        , const Propagation          propagation )
    : parameter(parameter)
    , symmetry(symmetry)
    , propagation(propagation)
    {}


    Control_Parameter::Control_Parameter(const Control_Parameter& other)
    : parameter(other.parameter)
    , symmetry(other.symmetry)
    , propagation(other.propagation)
    { dbg_msg("Copying control parameter."); }

    Control_Parameter& Control_Parameter::operator=(const Control_Parameter& other)
    {
        if (this != &other) // avoid invalid self-assignment
        {
            parameter   = other.parameter;
            symmetry    = other.symmetry;
            propagation = other.propagation;
        }
        return *this;
    }


} // namespace control
