#ifndef CHANGE_LIMITER_H
#define CHANGE_LIMITER_H

namespace common {

template <typename T>
class ChangeLimiter {
	T last_pos, last_vel;
	const T max_value;
public:
	ChangeLimiter(T max_value) : last_pos(), last_vel(), max_value(max_value) {}

	T step(T current) {
	    const double vel = current - last_pos;
		last_pos = last_pos + 0.5*clip((vel + last_vel)/2, max_value);
	    last_vel = vel;
		return last_pos;
	}
};

} /* namespace common */

#endif // CHANGE_LIMITER_H
