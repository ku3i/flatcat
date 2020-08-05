
#include "./config.h"

bool
config::load(void)
{
    /* try to open file for reading */
    fd = fopen(filename.c_str(), "r");
    if (fd == NULL) {
        wrn_msg("Cannot load configuration file %s", filename.c_str());
        return false;
    }

    sts_msg("Configuration file %s is opened, parsing...", filename.c_str());
    parse();
    fclose(fd); // and close
    sts_msg("Done.");
    return true;
}

void
config::parse(void)
{
    unsigned number_of_entries = 0;

    char name[64];
    char value[1024];

    if (NULL == fd)
        err_msg(__FILE__, __LINE__, "Cannot parse config file. File is not open.");
    fseek(fd, 0, SEEK_SET);

    while(true) {
        if (2 == fscanf(fd, "%64s = %1024s\n", name, value)) {
            sts_msg("| %2u %s = %s", ++number_of_entries, name, value);
            configuration.insert(element_t(std::string{name}, std::string{value}));
        }
        else break;
    }
}

int
config::readINT(std::string const& name, int def)
{
    int result = def;
    if (configuration.find(name) != configuration.end())
        result = std::stoi(configuration[name]);

    sts_msg("Reading %s <int> = %d %s", name.c_str(), result, (result == def)? "(DEFAULT)":"");
    return result;
}

unsigned
config::readUINT(std::string const& name, unsigned def)
{
    unsigned result = def;
    if (configuration.find(name) != configuration.end())
        result = std::stoul(configuration[name]);

    sts_msg("Reading %s <unsigned> = %u %s", name.c_str(), result, (result == def)? "(DEF)":"");
    return result;
}

bool
config::readBOOL(std::string const& name, bool def)
{
    bool result = def;

    if (configuration.find(name) != configuration.end())
    {
        if      ("YES" == configuration[name] or "TRUE" == configuration[name]) result = true;
        else if ("NO" == configuration[name] or "FALSE" == configuration[name]) result = false;
        else
            wrn_msg("unrecognized boolean value.");
    }

    sts_msg("Reading %s <bool> = %s %s", name.c_str(), result? "TRUE":"FALSE", (result == def)? "(DEF)":"");
    return result;
}

double
config::readDBL(std::string const& name, double def)
{
    double result = def;
    if (configuration.find(name) != configuration.end())
        result = std::stod(configuration[name]);

    sts_msg("Reading %s <double> = %f %s", name.c_str(), result, (result == def)? "(DEF)":"");
    return result;
}

std::string
config::readSTR(std::string const& name, std::string const& def)
{
    std::string result = def;
    if (configuration.find(name) != configuration.end())
        result = configuration[name];

    sts_msg("Reading %s <string> = \"%s\" %s", name.c_str(), result.c_str(), (result == def)? "(DEF)":"");
    return result;
}

void
config::writeINT(std::string const& name, int value)
{
    if (configuration.find(name) != configuration.end()) {
        sts_msg("Replacing entry %s with value %d", name.c_str(), value);
        configuration[name] = std::to_string(value);
        return;
    }

    sts_msg("Adding entry %s with value %d", name.c_str(), value);
    configuration.insert(element_t(name, std::to_string(value)));
}

void
config::writeUINT(std::string const& name, unsigned value)
{
    if (configuration.find(name) != configuration.end()) {
        sts_msg("Replacing entry %s with value %u", name.c_str(), value);
        configuration[name] = std::to_string(value);
        return;
    }

    sts_msg("Adding entry %s with value %u", name.c_str(), value);
    configuration.insert(element_t(name, std::to_string(value)));
}

void
config::writeBOOL(std::string const& name, bool value)
{
    if (configuration.find(name) != configuration.end()) {
        sts_msg("Replacing entry %s with value %s", name.c_str(), value ? "YES" : "NO");
        configuration[name] = value ? "YES" : "NO";
        return;
    }

    sts_msg("Adding entry %s with value %s", name.c_str(), value ? "YES" : "NO");
    configuration.insert(element_t(name, value ? std::string{"YES"} : std::string{"NO"}));
}

void
config::writeDBL(std::string const& name, double value)
{
    if (configuration.find(name) != configuration.end()) {
        sts_msg("Replacing entry %s with value %f", name.c_str(), value);
        configuration[name] = std::to_string(value);
        return;
    }

    sts_msg("Adding entry %s with value %f", name.c_str(), value);
    configuration.insert(element_t(name, std::to_string(value)));
}

void
config::writeSTR(std::string const& name, std::string const& value)
{
    if (configuration.find(name) != configuration.end()) {
        sts_msg("Replacing entry %s with value %s", name.c_str(), value.c_str());
        configuration[name] = value;
        return;
    }

    sts_msg("Adding entry %s with value %s", name.c_str(), value.c_str());
    configuration.insert(element_t(name, value));
}

void
config::finish(void)
{
    /* open configuration file for writing */
    std::fstream fs;
    fs.open(filename.c_str(), std::fstream::out);

    if (fs.is_open())
    {
        sts_add("Writing configuration file...");
        for (auto const& c: configuration)
            fs << c.first << " = " << c.second << std::endl;
        sts_msg("Done.");
        fs.close();
    }
    else wrn_msg("Cannot initialize configuration file %s", filename.c_str());
}
