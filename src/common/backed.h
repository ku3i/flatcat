#ifndef BACKED_H_INCLUDED
#define BACKED_H_INCLUDED

#include <cassert>
#include <deque>

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

template <typename T>
class delayed_t
{
    T actual;
    const unsigned steps;

    typedef std::deque<T> buffer_t;
    buffer_t backup;

public:
    explicit delayed_t(T const& actual = T(), const unsigned steps = 1)
    : actual(actual)
    , steps(steps)
    , backup()
    {
        assert(steps > 0);
        backup.assign(steps, T{});
    }

    void reset(void) {
        backup.assign(steps, T{});
        actual = T{};
        assert(backup.size() == steps);
    }

    void transfer() {
        backup.push_back(actual);
        backup.pop_front();
        assert(backup.size() == steps);
    }

    T const& get()        const { return actual; }
    T const& get_backed() const { return backup.front(); }

    delayed_t& operator=(const T& value) {
        this->actual = value;
        return *this;
    }

    delayed_t& operator+=(const T& value) {
        this->actual += value;
        return *this;
    }
};


} // common

#endif // BACKED_H_INCLUDED
