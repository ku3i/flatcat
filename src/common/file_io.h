#ifndef FILE_IO_H_INCLUDED
#define FILE_IO_H_INCLUDED

#include <cstdio>
#include <vector>
#include <string>
#include <assert.h>
#include <common/basic.h>
#include <common/noncopyable.h>
#include <common/log_messages.h>

namespace file_io {

namespace constants {
    const size_t format_length = 32;
    const char separator[] = " ";
}

template <typename T>
class CSV_File : public noncopyable
{
    CSV_File(const CSV_File& other) = delete;      // non construction-copyable
    CSV_File& operator=(const CSV_File&) = delete; // non copyable
public:
    CSV_File(const std::string& filename, const std::size_t rows, const std::size_t cols)
    : nbytes(cols * constants::format_length)
    , txtbuf((char *) malloc(nbytes + 1))
    , data(rows, std::vector<T>(cols))
    , max_rows(rows)
    , max_cols(cols)
    , filename(filename)
    {}

    ~CSV_File() { free(txtbuf); }

    bool read(void)
    {
        FILE* csv_file = open_file("r", filename.c_str());
        bool result = true;
        unsigned int row = 0;
        while (row < max_rows)
        {   // read next line
            if (getline(&txtbuf, &nbytes, csv_file) < 0) {
                wrn_msg("Error reading csv file %s in line %u\n", filename.c_str(), row + 1);
                result = false;
                break;
            }
            char* token = strtok(txtbuf, constants::separator); // get first token
            unsigned int col = 0;
            while (col < max_cols) {
                if (token != NULL)
                    data[row][col++] = atof(token);
                else {
                    wrn_msg("Unexpected columns size: %u (%u expected)", col, max_cols);
                    result = false;
                    break;
                }
                token = strtok(NULL, constants::separator); // get next token
            }
            ++row;
        }
        fclose(csv_file);
        return result;
    }

    void write(void)
    {
        FILE* csv_file = open_file("w", filename.c_str());
        for (std::size_t row = 0; row < max_rows; ++row)
        {
            for (std::size_t col = 0; col < max_cols; ++col)
                fprintf(csv_file, "%+1.8e ", data[row][col]);
            fprintf(csv_file, "\n");
        }
        fclose(csv_file);
    }

    void get_line(std::size_t row_index, std::vector<T>& line) {
        assert(row_index < data.size());
        //dbg_msg("reading line %2u, size: %u", row_index, max_cols);
        assert(line.size() == max_cols);
        line = data[row_index];
    }
    void get_line(std::size_t row_index, T& value) {
        assert(row_index < data.size());
        //dbg_msg("reading line %2u, size: 1", row_index);
        value = data[row_index][0];
    }

    void set_line(std::size_t row_index, const std::vector<T>& line) {
        assert(row_index < data.size() && line.size() == max_cols);
        //dbg_msg("writing line %2u, size: %u", row_index, max_cols);
        data[row_index] = line;
    }
    void set_line(std::size_t row_index, const T& value) {
        assert(row_index < data.size());
        //dbg_msg("writing line %2u, size: 1", row_index);
        data[row_index][0] = value;
    }

private:
    std::size_t nbytes;
    char* txtbuf;
    std::vector< std::vector<T> > data;
    const std::size_t max_rows;
    const std::size_t max_cols;
    const std::string filename;
};


class Logfile
{
    Logfile(const Logfile& other) = delete;      // non construction-copyable
    Logfile& operator=(const Logfile&) = delete; // non copyable
public:
    Logfile(const std::string& filename, bool append = false)
    : filename(filename)
    , fd(open_file(append ? "a":"w", filename.c_str()))
    {}

    ~Logfile() {
        std::size_t fs = basic::get_file_size(fd);
        fclose(fd);
        if (fs == 0) { // no data written
            sts_msg("No data written, removing file: %s", filename.c_str());
            remove(filename.c_str());
        }
    }

    void append(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        vfprintf(fd, format, args);
        fprintf(fd, "\n");
        va_end(args);
    }
    void append(const std::vector<double> Vector)
    {
        for (std::size_t idx = 0; idx < Vector.size(); ++idx)
            fprintf(fd, "%+1.8e ", Vector[idx]);
        fprintf(fd, "\n");
    }

    void flush(void) { fflush(fd); }


    const std::string filename;
    FILE* fd;
};


} // namespace file_io

#endif // FILE_IO_H_INCLUDED
