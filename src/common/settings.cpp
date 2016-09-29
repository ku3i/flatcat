
#include <common/settings.h>

bool
read_option_flag(int argc, char **argv, const char* short_name, const char* ext_name)
{
    for (int i = 1; i < argc; ++i) {
        if ((strncmp(argv[i], ext_name, strlen(ext_name)) == 0) || (strncmp(argv[i], short_name, strlen(short_name)) == 0))
            return true;
    }
    return false;
}

std::string
read_string_option(int argc, char **argv, const char* short_name, const char* ext_name, const char* default_value)
{
    std::string result = default_value;

    for (int i = 1; i < argc; ++i)
    {
        if ((strncmp(argv[i], ext_name, strlen(ext_name)) == 0) || (strncmp(argv[i], short_name, strlen(short_name)) == 0))
        {
            if (argc < i+2) {
                printf("usage: %s %s <value>\n", argv[0], short_name);
                exit(EXIT_FAILURE);
            } else {
                result = argv[i+1];
                ++i;
            }
        }
    }
    return result;
}

std::string
read_options(int argc, char **argv, const char* default_path)
{
    if (read_option_flag(argc, argv, "-h", "--help"))
    {
        printf("Help, Options: \n");
        printf("    -s  --settings : filename of settings file\n");
        printf("    -h  --help     : show help                \n");
        printf("\n");
        exit(EXIT_SUCCESS);
    }
    return read_string_option(argc, argv, "-s", "--settings", default_path);
}
