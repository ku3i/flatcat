#include <control/control_vector.h>


namespace control {

    void randomize_control_parameter(Control_Parameter& params, double std_dev, double max_dev)
    {
        assert_in_range(std_dev, 0.0, max_dev);
        for (std::size_t i = 0; i < params.size(); ++i)
            params[i] += random_value_norm(.0, std_dev, -max_dev, +max_dev);
    }



    Control_Vector::Control_Vector( std::size_t max_number_of_parameter_sets
                                  , const std::string& foldername )
    : max_number_of_parameter_sets(max_number_of_parameter_sets)
    , controls()
    {
        controls.reserve(max_number_of_parameter_sets);
        if (not foldername.empty())
        {
            basic::Filelist files = basic::list_directory(foldername.c_str(), ".dat");
            /* add files successively */
            if (files.size() > 0) {
                for (auto& f : files)
                {
                    sts_msg("Adding file %s", f.c_str());
                    controls.emplace_back(foldername + f);
                }
            } else
                sts_msg("No files found in '%s'", foldername.c_str());
        } else
            sts_msg("Initialize empty control vector.");
    }


    void Control_Vector::add( const std::string& filename
                            , const std::size_t number_of_params
                            , bool symmetric
                            , bool mirrored )
    {
        assert(controls.size() < max_number_of_parameter_sets);
        controls.emplace_back(filename, number_of_params, symmetric, mirrored);
    }


    void Control_Vector::add(const std::string& filename) {
        assert(controls.size() < max_number_of_parameter_sets);
        controls.emplace_back(filename);
    }


    void Control_Vector::reload(std::size_t index, const std::string& filename) {
        controls.at(index) = Control_Parameter(filename);
    }

} // namespace control
