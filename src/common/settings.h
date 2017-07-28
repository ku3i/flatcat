#ifndef SETTINGS_H_INCLUDED
#define SETTINGS_H_INCLUDED

#include <string.h>
#include <common/datareader.h>


std::string read_options(int argc, char **argv, const char* default_path);

class Settings_Base {

    const std::string    filename;
    file_io::Data_Reader settings_file;

public:

    Settings_Base(int argc, char **argv, const char* default_path)
    : filename(read_options(argc, argv, default_path))
    , settings_file(filename)
    {}

    file_io::uint_t   read_uint (const file_io::key_t name, file_io::uint_t   default_value) { return settings_file.read_unsigned(name, default_value); }
    file_io::float_t  read_float(const file_io::key_t name, file_io::float_t  default_value) { return settings_file.read_float   (name, default_value); }
    file_io::string_t read_str  (const file_io::key_t name, file_io::string_t default_value) { return settings_file.read_string  (name, default_value); }
    file_io::vector_t read_vec  (const file_io::key_t name, file_io::vector_t default_value) { return settings_file.read_vector  (name, default_value); }

    virtual ~Settings_Base() {}
};


#endif // SETTINGS_H_INCLUDED
