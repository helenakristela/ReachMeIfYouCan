#pragma once
#include "vec2.hpp"
struct Obstacle {
    Vec2 pos;
    double radius;
    Obstacle(Vec2 p, double r) : pos(p), radius(r) {}
};
inline Vec2 closestPointOnSegment(const Vec2& a, const Vec2& b, const Vec2& p) {
    Vec2 ab = b - a;
    double len2 = ab.dot(ab);
    if (len2 < 1e-12) return a;
    double t = (p - a).dot(ab) / len2;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return a + ab * t;
}
inline bool segmentIntersectsObstacle(const Vec2& a, const Vec2& b, const Obstacle& o) {
    Vec2 cp = closestPointOnSegment(a, b, o.pos);
    return (cp - o.pos).length() <= o.radius;
}
