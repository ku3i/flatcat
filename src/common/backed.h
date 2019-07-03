#ifndef BACKED_H_INCLUDED
#define BACKED_H_INCLUDED

namespace common {

template <typename T>
class backed_t
{
    T actual, backup;

public:
    explicit backed_t(T const& actual = T(), T const& backup = T()) : actual(actual), backup(backup) {}

    void reset(void) { actual = T{}; backup = T{}; }

    void set_backed(T const& value) { backup = value; }

    void transfer() { backup = actual; }

    T const& get()        const { return actual; }
    T const& get_backed() const { return backup; }

    backed_t& operator=(const T& value) {
        this->actual = value;
        return *this;
    }

    backed_t& operator+=(const T& value) {
        this->actual += value;
        return *this;
    }
};

} // common

#endif // BACKED_H_INCLUDED
