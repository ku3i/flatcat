/* basic.cpp */

#include "basic.h"

double *
init_double(unsigned int N) //TODO save
{
    double *var = (double *) calloc(N, sizeof(double));
    if (NULL == var) err_msg(__FILE__, __LINE__, "Cannot allocate memory.");
    return var;
}

double **
init_double(unsigned int N, unsigned int M)
{
    double **var = (double **) calloc(N, sizeof(double*));
    if (NULL == var) err_msg(__FILE__, __LINE__, "Cannot allocate memory (1).");

    for (unsigned int i = 0; i < N; ++i)
    {
        var[i] = (double *) calloc(M, sizeof(double));
        if (NULL == var[i]) err_msg(__FILE__, __LINE__, "Cannot allocate memory (2).");
    }
    return var;
}

void
clear_double(double *values, unsigned int N)
{
    if (NULL == values) err_msg(__FILE__, __LINE__, "Pointer is NULL.");
    for (unsigned int i = 0; i < N; i++)
        values[i] = .0;
}

FILE*
open_file(const char* mode, const char* format, ...)
{
    char filename[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(filename, 1024, format, args);
    va_end(args);

    FILE * fd = fopen(filename, mode);
    if (NULL == fd)
    {
        perror("Error");
        err_msg(__FILE__, __LINE__, "could not open file %s in mode %s.", filename, mode);
    }

    return fd;
}

void
make_directory(const char *format, ...)
{
    char foldername[256];
    va_list args;
    va_start(args, format);
    vsnprintf(foldername, 256, format, args);
    va_end(args);

    int md = mkdir(foldername, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (!md) sts_msg("create folder %s", foldername);
    else if (errno != EEXIST) err_msg(__FILE__, __LINE__, "could not create folder %s.\n %s\n", foldername, strerror(errno));
    return;
}

namespace basic {

Filelist list_directory(const char* target_dir, const char* filter)
{
    const unsigned max_files = 64;
    std::vector<std::string> files_in_directory;
    struct dirent *epdf;
    DIR *dpdf;

    if (0 == strcmp(target_dir,""))
        dpdf = opendir("./");
    else
        dpdf = opendir(target_dir);

    if (dpdf != NULL) {
        while ((epdf = readdir(dpdf)) and (files_in_directory.size() < max_files)) {
            if (strcmp(epdf->d_name, ".") and strcmp(epdf->d_name, "..") and (nullptr != strstr(epdf->d_name, filter)))
                files_in_directory.emplace_back(epdf->d_name);
        }
        if (files_in_directory.size() > 1)
            std::sort(files_in_directory.begin(), files_in_directory.end());
        dbg_msg("Read %u files in directory %s", files_in_directory.size(), target_dir);
    } else err_msg(__FILE__, __LINE__, "Could not open directory %s", target_dir);

    //for (unsigned i = 0; i < files_in_directory.size(); ++i) dbg_msg("%s", files_in_directory[i].c_str());
    return files_in_directory;
}

std::size_t get_file_size(FILE* fd)
{
    // obtain file size
    fseek(fd, 0, SEEK_END);
    std::size_t file_size = ftell(fd);
    rewind(fd);
    return file_size;
}

} // namespace basic
