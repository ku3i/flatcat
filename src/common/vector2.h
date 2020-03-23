#ifndef Vector2_H_INCLUDED
#define Vector2_H_INCLUDED

#include <common/modules.h>

class Vector2
{
public:

    Vector2(const Vector2& rhs) : x(rhs.x), y(rhs.y) {}
    Vector2(void) : x(.0), y(.0) {}
    Vector2(double x, double y) : x(x), y(y) {}
    Vector2(double val) : x(val), y(val) {}
    Vector2(const double val[2]) : x(val[0]), y(val[1]) {}

    Vector2& operator=(const Vector2& rhs) {
        this->x = rhs.x;
        this->y = rhs.y;
        return *this;
    }

    Vector2& operator=(const double& rhs) {
        this->x = rhs;
        this->y = rhs;
        return *this;
    }

    Vector2& operator+=(const Vector2& rhs) {
        this->x += rhs.x;
        this->y += rhs.y;
        return *this;
    }
    Vector2& operator-=(const Vector2& rhs) {
        this->x -= rhs.x;
        this->y -= rhs.y;
        return *this;
    }
    Vector2& operator*=(const double& rhs)  {
        this->x *= rhs;
        this->y *= rhs;
        return *this;
    }
    Vector2& operator/=(const double& rhs)  {
        this->x /= rhs;
        this->y /= rhs;
        return *this;
    }
//    void clip(double max_val) {
//        x = clip(x, max_val);
//        y = clip(y, max_val);
//    }
    double length() { return sqrt(x*x + y*y); }

    void normalize(void) {
        double l = 1.0 / length();
        x *= l;
        y *= l;
    }
    double angle_phi  (void) const { return atan2(y,x); }

    void random(double lower, double upper)
    {
        x = random_value(lower, upper);
        y = random_value(lower, upper);
    }

    void zero(void) { x = .0; y = .0; }

    bool is_zero(void) const { return (x == .0 && y == .0); }

    double x, y;
};

inline Vector2 operator+(Vector2 lhs, const Vector2& rhs) {
    lhs += rhs;
    return lhs;
}
inline Vector2 operator-(Vector2 lhs, const Vector2& rhs) {
    lhs -= rhs;
    return lhs;
}
inline Vector2 operator*(Vector2 lhs, const double& rhs)  {
    lhs *= rhs;
    return lhs;
}
inline Vector2 operator*(const double& lhs, Vector2 rhs)  {
    rhs *= lhs;
    return rhs;
}
inline double operator*(const Vector2& lhs, const Vector2& rhs) { //scalar multiplication
    return lhs.x * rhs.x
         + lhs.y * rhs.y;
}
inline Vector2 operator/(Vector2 lhs, const double& rhs)  {
    lhs /= rhs;
    return lhs;
}
inline double distance(const Vector2& lhs, const Vector2& rhs) {
    return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x)
              + (lhs.y - rhs.y) * (lhs.y - rhs.y));
}
inline Vector2 clip(const Vector2& v, double max_val) {
    Vector2 result;
    result.x = clip(v.x, max_val);
    result.y = clip(v.y, max_val);
    return result;
}

#endif // Vector2_H_INCLUDED
