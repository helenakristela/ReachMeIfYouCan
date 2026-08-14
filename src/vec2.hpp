#pragma once
#include <cmath>
struct Vec2 {
    double x = 0, y = 0;
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(double s) const { return Vec2(x * s, y * s); }
    double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    double length() const { return std::sqrt(x * x + y * y); }
    Vec2 normalized() const { double l = length(); return l > 1e-9 ? Vec2(x / l, y / l) : Vec2(0, 0); }
};
