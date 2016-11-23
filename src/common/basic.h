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

/* memory allocation */
double* init_double(unsigned int N) __attribute_deprecated__;
double** init_double(unsigned int N, unsigned int M) __attribute_deprecated__;

void clear_double(double *values, unsigned int N) __attribute_deprecated__;

/* file opening, formatted yeah! */
FILE * open_file(const char *mode, const char *format, ...)__attribute_deprecated__;
void make_directory(const char *format, ...);

namespace basic { /**TODO: move to file_io*/

typedef std::vector<std::string> Filelist;
Filelist list_directory(const char* target_dir = "", const char* filename_ending = "");

std::size_t get_file_size(FILE* fd);

} // namespace basic

#endif // BASIC_H
