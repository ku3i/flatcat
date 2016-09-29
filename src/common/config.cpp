
#include "./config.h"

void
config::clear()
{
    for (unsigned int i = 0; i < configuration.size(); ++i)
    {
        configuration[i].name[0] = '\0';
        configuration[i].value[0] = '\0';
    }
    num_entries = 0;
}

bool
config::load(void)
{
    /* try to open file for reading */
    fd = fopen(filename.c_str(), "r");
	if (fd == NULL) {
        num_entries = 0;
        wrn_msg("Cannot load configuration file %s", filename.c_str());
        return false;
    }

	sts_msg("Configuration file %s is opened", filename.c_str());
    num_entries = parse();
    fclose(fd); // and close
    return true;
}


unsigned int
config::parse()
{
    unsigned int number_of_entries = 0;

    if (NULL == fd)
        err_msg(__FILE__, __LINE__, "Cannot parse config file. File is not open.");
    fseek(fd, 0, SEEK_SET);

    for (unsigned int i = 0; i < configuration.size(); ++i)
    {
        if (2 == fscanf(fd, "%32s = %1024s\n", configuration[i].name, configuration[i].value)) {
            dbg_msg("  %2u: %s = %s", number_of_entries, configuration[i].name, configuration[i].value);
            ++number_of_entries;
        }
        else break;
    }

    return number_of_entries; // number of items read
}

int
config::get_num_entries(void)
{
    return num_entries;
}


int
config::readINT(const std::string& name, const int default_value)
{
    dbg_msg("Reading %s (int)", name.c_str());
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
            return atoi(configuration[i].value);
    wrn_msg("No value found for key '%s', keeping default: %d", name.c_str(), default_value);
    return default_value;
}

unsigned int
config::readUINT(const std::string& name, const unsigned int default_value)
{
    dbg_msg("Reading %s (uint)", name.c_str());
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
        {
            int tmp = atoi(configuration[i].value);
            if (tmp < 0) wrn_msg("Could not read %s without loss of information.", name.c_str());
            return static_cast<unsigned int> (tmp);
        }
    wrn_msg("No value found for key '%s', keeping default: %u", name.c_str(), default_value);
    return default_value;
}

bool
config::readBOOL(const std::string& name, const bool default_value)
{
    dbg_msg("Reading %s (bool)", name.c_str());
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
        {
            if ((0 == strcmp(configuration[i].value, "YES")) || (0 == strcmp(configuration[i].value, "TRUE")))
                return true;
            if ((0 == strcmp(configuration[i].value, "NO")) || (0 == strcmp(configuration[i].value, "FALSE")))
                return false;
        }
    wrn_msg("No value found for key '%s', keeping default: %s", name.c_str(), default_value? "true":"false");
    return default_value;
}

double
config::readDBL(const std::string& name, const double default_value)
{
    dbg_msg("Reading %s (double)", name.c_str());
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
            return atof(configuration[i].value);
    wrn_msg("No value found for key '%s', keeping default: %f", name.c_str(), default_value);
    return default_value;
}

double
config::readDBL(const std::string& name, const double minval, const double maxval, const double default_value)
{
    dbg_msg("Reading %s (double)", name.c_str());
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
            return clip(atof(configuration[i].value), minval, maxval);
    wrn_msg("No value found for key '%s', keeping default: %f", name.c_str(), default_value);
    return default_value;
}

std::string
config::readSTR(const std::string& name, const std::string& default_value)
{
    dbg_msg("Reading %s (string)", name.c_str());
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
            return std::string(configuration[i].value);
    wrn_msg("No value found for key '%s', keeping default: '%s'", name.c_str(), default_value.c_str());
    return default_value;
}

void
config::writeINT(const std::string& name, const int value)
{
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
        {
            dbg_msg("Replacing entry %s with [%d]", configuration[i].name, value);
            snprintf(configuration[i].value, MAX_VALUE_LENGTH, "%d", value);
            return;
        }

    if (num_entries < MAX_LINES)
    {
        snprintf(configuration[num_entries].name, MAX_NAME_LENGTH, "%s", name.c_str());
        snprintf(configuration[num_entries].value, MAX_VALUE_LENGTH, "%d", value);
        dbg_msg("Adding entry %s", configuration[num_entries].name);
        ++num_entries;
    } else dbg_msg("Cannot add further entries.");
}

void
config::writeUINT(const std::string& name, const unsigned int value)
{
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
        {
            dbg_msg("Replacing entry %s with [%d]", configuration[i].name, value);
            snprintf(configuration[i].value, MAX_VALUE_LENGTH, "%d", value);
            return;
        }

    if (num_entries < MAX_LINES)
    {
        snprintf(configuration[num_entries].name, MAX_NAME_LENGTH, "%s", name.c_str());
        snprintf(configuration[num_entries].value, MAX_VALUE_LENGTH, "%d", value);
        dbg_msg("Adding entry %s", configuration[num_entries].name);
        ++num_entries;
    } else dbg_msg("Cannot add further entries.");
}

void
config::writeBOOL(const std::string& name, const bool value)
{
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
        {
            dbg_msg("Replacing entry %s with [%s]", configuration[i].name, (value)?"YES":"NO");
            snprintf(configuration[i].value, MAX_VALUE_LENGTH, "%s", (value)?"YES":"NO");
            return;
        }

    if (num_entries < MAX_LINES)
    {
        snprintf(configuration[num_entries].name, MAX_NAME_LENGTH, "%s", name.c_str());
        snprintf(configuration[num_entries].value, MAX_VALUE_LENGTH, "%s", (value)?"YES":"NO");
        dbg_msg("Adding entry %s", configuration[num_entries].name);
        ++num_entries;
    } else dbg_msg("Cannot add further entries.");
}


void
config::writeDBL(const std::string& name, const double value)
{
    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
        {
            dbg_msg("Replacing entry %s with [%f]", configuration[i].name, value);
            snprintf(configuration[i].value, MAX_VALUE_LENGTH, "%f", value);
            return;
        }

    if (num_entries < MAX_LINES)
    {
        snprintf(configuration[num_entries].name, MAX_NAME_LENGTH, "%s", name.c_str());
        snprintf(configuration[num_entries].value, MAX_VALUE_LENGTH, "%f", value);
        dbg_msg("Adding entry %s", configuration[num_entries].name);
        ++num_entries;
    } else dbg_msg("Cannot add further entries.");
}

void
config::writeSTR(const std::string& name, const std::string& value)
{
    //if (NULL == value.c_str()) log.err_msg(__FILE__, __LINE__, "Could not write NULL String to file.");

    for (unsigned int i = 0; i < num_entries; ++i)
        if (0 == strcmp(configuration[i].name, name.c_str()))
        {
            dbg_msg("Replacing entry %s with [%s]", configuration[i].name, value.c_str());
            snprintf(configuration[i].value, MAX_VALUE_LENGTH, "%s", value.c_str());
            return;
        }

    if (num_entries < MAX_LINES)
    {
        snprintf(configuration[num_entries].name, MAX_NAME_LENGTH, "%s", name.c_str());
        snprintf(configuration[num_entries].value, MAX_VALUE_LENGTH, "%s", value.c_str());
        dbg_msg("Adding entry %s", configuration[num_entries].name);
        ++num_entries;
    } else dbg_msg("Cannot add further entries.");
}

void
config::finish()
{
    /* try to open config file for writing */

    std::fstream fs;
    fs.open(filename.c_str(), std::fstream::out);

	if (fs.is_open())
	{
        dbg_msg("Writing configuration file.");
        for (unsigned int i = 0; i < num_entries; ++i)
        {
            fs << configuration[i].name << " = " << configuration[i].value << std::endl;
            dbg_msg("%s = %s", configuration[i].name, configuration[i].value);
        }
        dbg_msg("Closing configuration file.");
        fs.close();
    }
    else dbg_msg("Cannot initialize configuration file %s", filename.c_str());
}
