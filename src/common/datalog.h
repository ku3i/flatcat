#ifndef DATALOG_H_INCLUDED
#define DATALOG_H_INCLUDED

#include <common/basic.h>
#include <common/file_io.h>
#include <common/log_messages.h>

class Datalog {
public:

    Datalog(std::string const& filename, bool enabled = false)
    : logfile(filename)
    , enabled(enabled)
    {
        sts_msg("Created data log file: %s", filename.c_str());
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

private:
    file_io::Logfile logfile;
    bool enabled;
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
            if (n < BufferSize-total_bytes) {
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
