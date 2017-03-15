
#include <control/controlparameter.h>

namespace control {


    Control_Parameter::Control_Parameter( const std::string& filename
                                        , std::size_t number_of_params
                                        , bool symmetric
                                        , bool mirrored )
    : parameter()
    , symmetric(symmetric)
    , mirrored(mirrored)
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
    , symmetric()
    , mirrored()
    {
        sts_msg("Loading controller weights from DAT file:\n   '%s'", filename.c_str());
        file_io::Data_Reader dat_file(filename);
        assert(dat_file.read("parameter", parameter)); /** TODO: remove assert dependence */

        symmetric = ("symmetric" == dat_file.read_string("symmetry"   )) ? true : false;
        mirrored  = ("original"  == dat_file.read_string("propagation")) ? false : true;
    }


    Control_Parameter::Control_Parameter(const std::vector<double>& parameter, bool symmetric, bool mirrored)
    : parameter(parameter)
    , symmetric(symmetric)
    , mirrored(mirrored)
    {}

    void Control_Parameter::set_from_matrix(matrix_t const& weights)
    {
        assert(symmetric == false and mirrored == false);
        assert(weights.size() > 0 and weights[0].size() > 0);
        assert(parameter.size() == weights.size() * weights[0].size());

        std::size_t p = 0;
        for (auto const& wi : weights)
            for (auto const& wij : wi)
                parameter[p++] = wij;
        assert( p == parameter.size());
    }

    Control_Parameter::Control_Parameter(const Control_Parameter& other)
    : parameter(other.parameter)
    , symmetric(other.symmetric)
    , mirrored(other.mirrored)
    { /*dbg_msg("Copying control parameter.");*/ }


    Control_Parameter& Control_Parameter::operator=(const Control_Parameter& other)
    {
        if (this != &other) // avoid invalid self-assignment
        {
            parameter = other.parameter;
            symmetric = other.symmetric;
            mirrored  = other.mirrored;
        }
        return *this;
    }

    void Control_Parameter::print() const {
        for ( auto const& p : parameter ) printf("% 5.2f ", p);
        printf("\n");
    }

} // namespace control
