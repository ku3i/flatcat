/* config.h */

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <fstream>
#include <map>

#include <common/log_messages.h>
#include <common/modules.h>


class config
{
    config( const config& other ) = delete;      // non construction-copyable
    config& operator=( const config& ) = delete; // non copyable

    typedef std::pair<std::string,std::string> element_t;

public:
    config(const std::string& filename, bool quit_on_fail = false)
    : fd()
    , filename(filename)
    , configuration()
    {
        if (not filename.empty())
            sts_msg("Configuration filename is: %s", filename.c_str());
        else
            err_msg(__FILE__, __LINE__, "No file name provided for configuration.");

        if (not load() and quit_on_fail)
            err_msg(__FILE__, __LINE__, "Configuration file does not exits, but was expected.");
    }

    bool load(void);
    void clear(void) { configuration.clear(); }
    void finish(void);
    unsigned get_num_entries(void) const { return configuration.size(); }

    int         readINT (std::string const& name, int                def = {});
    unsigned    readUINT(std::string const& name, unsigned           def = {});
    bool        readBOOL(std::string const& name, bool               def = {});
    double      readDBL (std::string const& name, double             def = {});
    std::string readSTR (std::string const& name, std::string const& def = {});

    void writeINT (std::string const& name, int                value);
    void writeUINT(std::string const& name, unsigned int       value);
    void writeBOOL(std::string const& name, bool               value);
    void writeDBL (std::string const& name, double             value);
    void writeSTR (std::string const& name, std::string const& value);

private:
    FILE* fd;
    const std::string filename;
    std::map<std::string,std::string> configuration;
    void parse(void);
};

#endif //CONFIG_H
