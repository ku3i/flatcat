#ifndef BACKED_H_INCLUDED
#define BACKED_H_INCLUDED

namespace common {

template <typename T>
class backed_t
{
    T actual, backup;

public:
    explicit backed_t(T const& actual = T(), T const& backup = T()) : actual(actual), backup(backup) {}

    void set(T const& value) { actual = value; }
    void transfer() { backup = actual; }

    T const& get()        const { return actual; }
    T const& get_backed() const { return backup; }
};

} // common

#endif // BACKED_H_INCLUDED
