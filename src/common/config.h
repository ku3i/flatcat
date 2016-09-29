/* config.h */

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <fstream>
#include <vector>

#include "./log_messages.h"
#include "./modules.h"

#define MAX_LINES 32
#define MAX_NAME_LENGTH 32
#define MAX_VALUE_LENGTH 1024

struct cfg_entry //TODO refactor that
{
    char name[MAX_NAME_LENGTH];
    char value[MAX_VALUE_LENGTH];
};

class config
{
    config( const config& other ) = delete;      // non construction-copyable
    config& operator=( const config& ) = delete; // non copyable

public:
    config(const std::string& filename, bool quit_on_fail = false)
    : fd()
    , filename(filename)
    , configuration(MAX_LINES)
    , num_entries()
    {
        if (not filename.empty())
            sts_msg("Configuration filename is: %s", filename.c_str());
        else
            err_msg(__FILE__, __LINE__, "No file name provided for configuration.");

        if (not load() and quit_on_fail)
            err_msg(__FILE__, __LINE__, "Configuration file does not exits, but was expected.");
    }

    bool load(void);
    void clear(void);
    void finish(void);
    int get_num_entries(void);

    int          readINT (const std::string& name, const int          default_value = 0);
    unsigned int readUINT(const std::string& name, const unsigned int default_value = 0);

    bool         readBOOL(const std::string& name, const bool         default_value = false);
    double       readDBL (const std::string& name, const double       default_value = .0);
    double       readDBL (const std::string& name, const double minval, const double maxval, const double default_value = .0);
    std::string  readSTR (const std::string& name, const std::string& default_value = "");

    void writeINT (const std::string& name, const int          value);
    void writeUINT(const std::string& name, const unsigned int value);
    void writeBOOL(const std::string& name, const bool         value);
    void writeDBL (const std::string& name, const double       value);
    void writeSTR (const std::string& name, const std::string& value);

private:
    FILE* fd;
    const std::string filename;
    std::vector<cfg_entry> configuration;
    unsigned int num_entries;
    unsigned int parse(void);
};

#endif //CONFIG_H
