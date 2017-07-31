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

#endif // DATALOG_H_INCLUDED
