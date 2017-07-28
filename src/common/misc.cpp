#include "misc.h"


std::string //TODO make stepsize as parameter
get_time_from_cycle_counter(unsigned long long cycles)
{
    char time_str[1024];
    unsigned int days    =  cycles / (100 * 3600  * 24);
    unsigned int hours   =  cycles / (100 * 3600) % 24;
    unsigned int minutes = (cycles / (100 * 60))  % 60;
    unsigned int seconds = (cycles /  100)        % 60;
    unsigned int hsecs   = (cycles %  100);

    if (days > 0)
        snprintf(time_str, 1024, "%3u:%02u:%02u:%02u:%02u", days, hours, minutes, seconds, hsecs);
    else
        snprintf(time_str, 1024, "%02u:%02u:%02u:%02u", hours, minutes, seconds, hsecs);
    return std::string(time_str);
}

std::string to_str(const std::vector<double>& vect)
{
    char buf[256] = "";
    int len = 0;
    if (vect.size() == 0) return buf;
    int n = snprintf(buf, sizeof(buf), "%+1.2f", vect[0]);
    for (std::size_t i = 1; i < vect.size(); ++i) {
        if (n >= 0 && n < ((int)sizeof(buf) - len)) {
            len += n;
            n = snprintf(buf + len, sizeof(buf) - len, " %+1.2f", vect[i]);
        } else break;
    }
    return buf;
}

template <typename vector_t> void
print(const vector_t& content) {
    for (std::size_t i = 0; i < content.size(); ++i)
        std::cout << content[i] << " ";
    std::cout << std::endl;
}
