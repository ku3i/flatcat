#ifndef DATALOG_H_INCLUDED
#define DATALOG_H_INCLUDED

#include <string>
#include <common/basic.h>
#include <common/file_io.h>
#include <common/log_messages.h>
#include <common/settings.h>

namespace constants {
    const std::string logfolder = "./data/";
    const std::string logfileext = ".log";
}

class Datalog {
public:

    Datalog(int argc, char** argv, std::string const& folder = constants::logfolder )
    : filename  ( read_string_option(argc, argv, "--outfile"       , "-o", get_timestamped_file_name(folder)) )
    , logfile   ( filename )
    , enabled   ( read_option_flag  (argc, argv, "--enable_logging", "-l") )
    , incl_video( read_option_flag  (argc, argv, "--include_video" , "-i") )
    {
        sts_msg("Created data log file: %s (%s)", filename.c_str(), enabled ? "Enabled":"Disabled");
        sts_msg("Video will%s be recorded.", incl_video ? "":" NOT");
    }

    template<typename... Args>
    void log(const char* format, const Args&... args)
    {
        if (!enabled) return;

        logfile.append(format, args...);
    }

    void toggle_logging(void) {
        enabled = !enabled;
        sts_msg("Logging: %s", enabled ? "ON":"OFF");
        if (not enabled)
            logfile.flush();
    }

    bool is_enabled(void) const { return enabled; }
    bool is_video_included(void) const { return enabled and incl_video; }

private:
    const std::string filename;
    file_io::Logfile logfile;
    bool enabled;
    bool incl_video;


    std::string get_timestamped_file_name(const std::string& folder) {
        return basic::make_directory(folder.c_str()) // folder
             + basic::get_timestamp()                // name as time stamp
             + constants::logfileext;                // extension
    }
};


template <unsigned BufferSize = 1024>
class Loggable {

    char buffer[BufferSize];
    std::size_t total_bytes = 0;

protected:

    template<typename... Args>
    void append(const char* format, const Args&... args)
    {
        if (total_bytes + 1 < BufferSize) {
            auto n = snprintf(buffer+total_bytes, BufferSize-total_bytes, format, args...);
            if (n < 0) err_msg(__FILE__,__LINE__," Encoding error.");
            if (static_cast<unsigned>(n) < BufferSize-total_bytes) {
                total_bytes += n;
                return;
            }
        }
        err_msg(__FILE__,__LINE__," Buffer size of %u too small.", BufferSize);
    }

    const char* done(void) {
        //dbg_msg("Bytes written %u", total_bytes);
        total_bytes = 0;
        return buffer;
    }

public:
    virtual ~Loggable() {}
    virtual const char* log(void) = 0;

};


#endif // DATALOG_H_INCLUDED
