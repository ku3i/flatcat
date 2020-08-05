#ifndef BASIC_H
#define BASIC_H

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include "log_messages.h"


/* file opening, formatted yeah! */
FILE * open_file(const char *mode, const char *format, ...);

namespace basic { /**IDEA: Consider moving this to file_io*/

std::string make_directory(const char *format, ...);

typedef std::vector<std::string> Filelist;
Filelist list_directory(const char* target_dir = "", const char* filename_ending = "");

std::size_t get_file_size(FILE* fd);

std::string get_timestamp();

} // namespace basic

#endif // BASIC_H
