#pragma once

// Small 3-vector and axis-aligned box for meshkit.

#include <algorithm>
#include <cmath>

namespace meshkit {

struct V3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator-(V3 a) { return {-a.x, -a.y, -a.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline V3 operator*(double s, V3 a) { return a * s; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double length(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 normalize(V3 a) {
    const double l = length(a);
    return l > 0.0 ? a * (1.0 / l) : V3{};
}
inline V3 lerp(V3 a, V3 b, double t) { return a + (b - a) * t; }
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }
inline double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

/// The distance of a solid whose section lies at d2 and which ends w past its
/// caps (both negative inside): an extrusion's distance, exact outside.
inline double extrude(double d2, double w) {
    if (d2 > 0.0 && w > 0.0) return std::hypot(d2, w);
    return std::max(d2, w);
}

struct Box {
    V3 lo{1e30, 1e30, 1e30};
    V3 hi{-1e30, -1e30, -1e30};

    bool empty() const { return lo.x > hi.x; }
    void add(V3 p) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    void merge(const Box& b) {
        if (b.empty()) return;
        add(b.lo);
        add(b.hi);
    }
    Box padded(double r) const {
        Box b = *this;
        if (!b.empty()) {
            b.lo = b.lo - V3{r, r, r};
            b.hi = b.hi + V3{r, r, r};
        }
        return b;
    }
    Box intersect(const Box& b) const {
        Box r;
        r.lo = {std::max(lo.x, b.lo.x), std::max(lo.y, b.lo.y), std::max(lo.z, b.lo.z)};
        r.hi = {std::min(hi.x, b.hi.x), std::min(hi.y, b.hi.y), std::min(hi.z, b.hi.z)};
        return r;
    }
    /// Distance from p to the box (0 inside): a lower bound of the distance
    /// to anything inside it.
    double distance(V3 p) const {
        const double dx = std::max({lo.x - p.x, 0.0, p.x - hi.x});
        const double dy = std::max({lo.y - p.y, 0.0, p.y - hi.y});
        const double dz = std::max({lo.z - p.z, 0.0, p.z - hi.z});
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

} // namespace meshkit
