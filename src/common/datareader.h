#ifndef DATAREADER_H_INCLUDED
#define DATAREADER_H_INCLUDED

#include <string>
#include <vector>
#include <map>

#include <common/noncopyable.h>
#include <common/basic.h>

namespace file_io {

/*
    i = 5         int
    s = "hallo"   string
    f = {0.1}     float
    v = {0 1 1 2} vector
    # comment
*/

    typedef std::string            key_t;

    typedef uint64_t              uint_t;
    typedef int64_t               sint_t;
    typedef double               float_t;
    typedef std::vector<double> vector_t;
    typedef std::string         string_t;

namespace {

    const std::size_t key_size = 256;
    const std::size_t val_size = 1048576; // 1 kbyte
    const char separator[] = " ";

    const char pattern_vec[] = " %256[a-z_] = { %1048576[\n\t eE0-9.+-] } %n";
    const char pattern_str[] = " %256[a-z_] = \"%1048576[^\"]\" %n";
    const char pattern_int[] = " %256[a-z_] = %ld %n";
}

inline bool is_empty(const char* msg) {
    while(*msg != '\0') {
        if (not isspace(*msg))
            return false;
        msg++;
    }
    return true;
}

class Data_Reader : public noncopyable{

    FILE* fd;
    const std::size_t file_size;
    char* txtbuf;
    char* refto_txtbuf;

    char key[key_size];

    char  str_val[val_size];
    long int int_val;

    Data_Reader           ( const Data_Reader& ) = delete; // non construction-copyable
    Data_Reader& operator=( const Data_Reader& ) = delete; // non copyable

    /* maps */
    std::map<key_t, vector_t> map_vec;
    std::map<key_t, string_t> map_str;
    std::map<key_t,   sint_t> map_int;

    bool verbose;

public:
    Data_Reader(const std::string& filename, bool verbose = true)
    : fd(open_file("r", filename.c_str()))
    , file_size(basic::get_file_size(fd))
    , txtbuf((char*) malloc (sizeof(char) * file_size))
    , refto_txtbuf(txtbuf)
    , int_val(0)
    , map_vec()
    , map_str()
    , map_int()
    , verbose(verbose)
    {
        if (verbose) sts_msg("Reading file %s \n      with size: %lu bytes.", filename.c_str(), file_size);

        if (nullptr == txtbuf)
            err_msg(__FILE__, __LINE__, "Cannot allocate memory.");

        // copy complete file into buffer
        std::size_t result = fread(txtbuf, 1, file_size, fd);
        if (result != file_size)
            err_msg(__FILE__, __LINE__, "Cannot read full file.");

        fclose(fd);
        parse();
    }

    ~Data_Reader() {
        if (refto_txtbuf != nullptr)
            free(refto_txtbuf);
    }

private:

    vector_t decode(char *value)
    {
        vector_t data;
        char* token = strtok(value, separator); // get first token
        while (nullptr != token) {
            if (not is_empty(token))
                data.emplace_back(atof(token));
            token = strtok(nullptr, separator); // get next token
        }
        return data;
    }

    void remove_comments(void)
    {
        bool comment_found = false;
        for (std::size_t i = 0; i < file_size; ++i)
        {
            if ('#' == txtbuf[i])
                comment_found = true;
            else if (comment_found && ('\n' == txtbuf[i]))
                comment_found = false;

            if (comment_found)
                txtbuf[i] = ' '; // clear comments
        }
    }

    void parse(void) {

        remove_comments();
        int offset = 0;
        while (true)
        {
            if (2 == sscanf(txtbuf, pattern_vec, key, str_val, &offset))
            {
                vector_t data = decode(str_val);
                if (verbose) {
                    printf("\t%s = ( ", key);
                    for (std::size_t i = 0; i < std::min((std::size_t)8, data.size()); ++i)
                        printf("%1.2f ", data[i]);
                    if (data.size() > 8)
                        printf("... ) N=%lu\n", data.size());
                    else
                        printf(") \n");
                }
                if (0 == map_vec.count(key))
                    map_vec.emplace(key, data);
                else wrn_msg("\tSkipping vector %s, already in list.", key);
            }
            else if (2 == sscanf(txtbuf, pattern_str, key, str_val, &offset)) {
                if (verbose) sts_msg("\t%s = \'%s\'", key, str_val);
                if (0 == map_str.count(key))
                    map_str.emplace(key, str_val);
                else wrn_msg("\tSkipping string %s, already in list.", key);
            }
            else if (2 == sscanf(txtbuf, pattern_int, key, &int_val, &offset)) {
                if (verbose) sts_msg("\t%s = <%ld>", key, int_val);
                if (0 == map_int.count(key))
                    map_int.emplace(key, int_val);
                else wrn_msg("\tSkipping integer %s, already in list.", key);
            }
            else {
                if (verbose) sts_msg("Done reading");
                break;
            }
            txtbuf += offset;
        }
    }

public:
    bool read (const key_t& key,   uint_t& value)
    {
        if (map_int.count(key)) {
            value = static_cast<uint_t>(map_int[key]);
            return true;
        }
        return false;
    }
    bool read (const key_t& key,   sint_t& value)
    {
        if (map_int.count(key)) {
            value = map_int[key];
            return true;
        }
        return false;
    }
    bool read (const key_t& key,  float_t& value)
    {
        if (map_vec.count(key)) {
            const vector_t& vec = map_vec[key];
            if (vec.size() == 1) {
                value = vec[0];
                return true;
            }
        }
        return false;
    }
    bool read (const key_t& key, string_t& value)
    {
        if (map_str.count(key)) {
            value = map_str[key];
            return true;
        }
        return false;
    }
    bool read (const key_t& key, vector_t& value)
    {
        if (map_vec.count(key)) {
            value = map_vec[key];
            return true;
        }
        return false;
    }

    uint_t read_unsigned(const key_t& key, const uint_t default_value = uint_t()) {
        uint_t result;
        if (not read(key, result)) {
            wrn_msg("Cannot read unsigned '%s'. Taking default: %u", key.c_str(), default_value);
            return default_value;
        }
        return result;
    }

    sint_t read_signed(const key_t& key, const sint_t default_value = sint_t()) {
        sint_t result;
        if (not read(key, result)) {
            wrn_msg("Cannot read signed '%s'. Taking default: %d", key.c_str(), default_value);
            return default_value;
        }
        return result;
    }

    float_t read_float(const key_t& key, const float_t default_value = float_t()) {
        float_t result;
        if (not read(key, result)) {
            wrn_msg("Cannot read float '%s'. Taking default: %f", key.c_str(), default_value);
            return default_value;
        }
        return result;
    }

    string_t read_string(const key_t& key, const string_t default_value = string_t()) {
        string_t result;
        if (not read(key, result)) {
            wrn_msg("Cannot read string '%s'. Taking default: %s", key.c_str(), default_value.c_str());
            return default_value;
        }
        return result;
    }

    vector_t read_vector(const key_t& key, const vector_t default_value = vector_t()) {
        vector_t result;
        if (not read(key, result)) {
            wrn_msg("Cannot read vector '%s'.", key.c_str());
            return default_value;
        }
        return result;
    }

    template <typename result_t>
    result_t read_type(const key_t& key, const result_t default_value = result_t()) {
        result_t result;
        if (not read(key, result)) {
            wrn_msg("Cannot read '%s'", key.c_str());
            return default_value;
        }
        return result;
    }

//    void write(const std::string& key, const   uint_t& value) { }
//    void write(const std::string& key, const   sint_t& value) { }
//    void write(const std::string& key, const  float_t& value) { }
//    void write(const std::string& key, const string_t& value) { }
//    void write(const std::string& key, const vector_t& value) { }

};


} // namespace file_io

#endif // DATAREADER_H_INCLUDED
