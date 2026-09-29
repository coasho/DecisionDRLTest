// Operational zones' geometry (docs/flight-autonomy.md, 4.43; A-GRA's ZoneType): see Zones.h.
#include "control/Zones.h"

#include "core/Geodesy.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control::zones {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool code(double v, double count) noexcept { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); }
bool finiteOr(double v) noexcept { return isHold(v) || std::isfinite(v); }

/// Segment ab crosses segment cd (touching counts; a shared end of neighbours is left to the caller).
bool cross(const double* a, const double* b, const double* c, const double* d) noexcept {
    auto side = [](const double* p, const double* q, const double* r) { return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]); };
    const double d1 = side(c, d, a), d2 = side(c, d, b), d3 = side(a, b, c), d4 = side(a, b, d);
    if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))) return true;
    auto on = [](const double* p, const double* q, const double* r) {
        return std::min(p[0], q[0]) <= r[0] && r[0] <= std::max(p[0], q[0]) && std::min(p[1], q[1]) <= r[1] && r[1] <= std::max(p[1], q[1]);
    };
    return (d1 == 0.0 && on(c, d, a)) || (d2 == 0.0 && on(c, d, b)) || (d3 == 0.0 && on(a, b, c)) || (d4 == 0.0 && on(a, b, d));
}

/// A closed ring of `n` points: simple (no two edges meet but neighbours at their shared vertex) and not flat.
bool simple(const double (*p)[2], std::size_t n) noexcept {
    double twiceArea = 0.0;
    for (std::size_t i = 0; i < n; ++i) twiceArea += p[i][0] * p[(i + 1) % n][1] - p[(i + 1) % n][0] * p[i][1];
    if (!(std::abs(twiceArea) > 1e-6)) return false;
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j) {
            const bool neighbours = j == i + 1 || (i == 0 && j == n - 1);
            if (neighbours) continue;
            if (cross(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n])) return false;
        }
    return true;
}

/// Inside a ring, by its crossings.
bool inside(const double (*p)[2], std::size_t n, double x, double y) noexcept {
    bool in = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++)
        if ((p[i][1] > y) != (p[j][1] > y) && x < (p[j][0] - p[i][0]) * (y - p[i][1]) / (p[j][1] - p[i][1]) + p[i][0]) in = !in;
    return in;
}

/// The nearest point of a ring's edges to (x, y), and how far.
double nearestOnRing(const double (*p)[2], std::size_t n, double x, double y, double& nx, double& ny) noexcept {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
        const double* a = p[i];
        const double* b = p[(i + 1) % n];
        const double dx = b[0] - a[0], dy = b[1] - a[1], len2 = dx * dx + dy * dy;
        const double t = len2 > 0.0 ? std::clamp(((x - a[0]) * dx + (y - a[1]) * dy) / len2, 0.0, 1.0) : 0.0;
        const double qx = a[0] + t * dx, qy = a[1] + t * dy, d = std::hypot(x - qx, y - qy);
        if (d < best) best = d, nx = qx, ny = qy;
    }
    return best;
}

/// (x, y) in an axis frame turned `bearing` from north: along it, and across it (to its right).
void along(double bearing, double x, double y, double& a, double& c) noexcept {
    a = x * std::cos(bearing) + y * std::sin(bearing);
    c = -x * std::sin(bearing) + y * std::cos(bearing);
}
void unalong(double bearing, double a, double c, double& x, double& y) noexcept {
    x = a * std::cos(bearing) - c * std::sin(bearing);
    y = a * std::sin(bearing) + c * std::cos(bearing);
}

/// A slant range area's bearing (from its point, turned by its orientation) within its extent.
bool withinBearings(const MustFlyArea& a, double bearing) noexcept {
    const double width = geo::wrapTwoPi(a.azimuthMaxRad - a.azimuthMinRad);
    return width == 0.0 && a.azimuthMaxRad != a.azimuthMinRad ? true : geo::wrapTwoPi(bearing - a.orientationRad - a.azimuthMinRad) <= width;
}

/// The turn of a frame's axes (2D): its yaw, or its track (its yaw where it has none).
double turnOf(const MustFlyArea& a, const FramePose& pose) noexcept {
    switch (a.rotation) {
    case FrameRotation::Unrotated: return 0.0;
    case FrameRotation::Heading: return std::hypot(pose.northMs, pose.eastMs) > 0.1 ? std::atan2(pose.eastMs, pose.northMs) : pose.yawRad;
    default: return pose.yawRad; // (yaw, or its attitude's: a zone is flat)
    }
}

/// A ring of vertices given, as plane points from the reference (`toXY`), into `out`.
template <typename ToXY>
std::size_t ring(const std::vector<ZoneVertex>& vs, double (*out)[2], ToXY toXY) noexcept {
    const std::size_t n = std::min(vs.size(), MustFlyArea::kVertices);
    for (std::size_t i = 0; i < n; ++i) toXY(vs[i], out[i][0], out[i][1]);
    return n;
}

} // namespace

int fault(const OpZone& z, bool frameKnown) noexcept {
    if (isHold(z.shape) || !code(z.shape, static_cast<double>(ZoneShape::Count))) return 0;
    const auto shape = static_cast<ZoneShape>(static_cast<int>(z.shape));
    const bool framed = !isHold(z.frame);
    if (framed && !(z.frame == std::floor(z.frame) && z.frame >= 1.0 && frameKnown)) return 7;
    if (!code(z.frameRotation, static_cast<double>(FrameRotation::Count)) || (!framed && !isHold(z.frameRotation))) return 7;
    // its velocity: both ways or neither, finite; a frame moves it instead
    if (isHold(z.northMs) != isHold(z.eastMs) || !finiteOr(z.northMs) || !finiteOr(z.eastMs) || !finiteOr(z.timeS)) return 8;
    if ((framed && !isHold(z.northMs)) || (isHold(z.northMs) && !isHold(z.timeS))) return 8;
    // its band
    if (!finiteOr(z.altitudeMinM) || !finiteOr(z.altitudeMaxM) || !code(z.altitudeReference, static_cast<double>(AltitudeReference::Count))) return 6;
    if (!isHold(z.altitudeMinM) && !isHold(z.altitudeMaxM) && z.altitudeMinM > z.altitudeMaxM) return 6;
    if (!isHold(z.altitudeReference) && isHold(z.altitudeMinM) && isHold(z.altitudeMaxM)) return 6;
    // a place: on the Earth or in the frame, never both
    auto placed = [&](double lat, double lon, double x, double y) {
        if (framed) return isHold(lat) && isHold(lon) && std::isfinite(x) && std::isfinite(y);
        return isHold(x) && isHold(y) && std::isfinite(lat) && std::isfinite(lon) && std::abs(lat) <= 0.5 * kPi;
    };
    const bool centred = !isHold(z.latitudeRad) || !isHold(z.longitudeRad) || !isHold(z.xM) || !isHold(z.yM);
    const double dims[] = {z.semiMajorM, z.semiMinorM, z.widthM, z.heightM, z.rangeMinM, z.rangeMaxM};
    for (const double d : dims)
        if (!finiteOr(d) || d < 0.0) return 4;
    if (!finiteOr(z.orientationRad) || !finiteOr(z.azimuthMinRad) || !finiteOr(z.azimuthMaxRad)) return 5;
    const auto given = [](double v) { return !isHold(v); };
    if (shape == ZoneShape::Polygon) {
        if (centred) return 3;
        if (std::any_of(std::begin(dims), std::end(dims), given)) return 4;
        if (given(z.orientationRad) || given(z.azimuthMinRad) || given(z.azimuthMaxRad)) return 5;
        if (z.vertices.size() < 3 || z.vertices.size() > MustFlyArea::kVertices) return 1;
        for (const ZoneVertex& v : z.vertices)
            if (!placed(v.latitudeRad, v.longitudeRad, v.xM, v.yM)) return 1;
        if (z.holes.size() > MustFlyArea::kHoles) return 2;
        for (const auto& h : z.holes) {
            if (h.size() < 3 || h.size() > MustFlyArea::kVertices) return 2;
            for (const ZoneVertex& v : h)
                if (!placed(v.latitudeRad, v.longitudeRad, v.xM, v.yM)) return 2;
        }
        // laid out as flown, then checked there: simple, its holes inside it and apart
        MustFlyArea a;
        FrameSpec none;
        layOut(z, framed ? &none : nullptr, 0.0, a);
        if (!simple(a.vertices, a.vertexCount)) return 1;
        for (std::size_t k = 0; k < a.holeCount; ++k) {
            const auto* hole = a.holes[k];
            const std::size_t n = a.holeSizes[k];
            if (!simple(hole, n)) return 2;
            for (std::size_t i = 0; i < n; ++i) {
                if (!inside(a.vertices, a.vertexCount, hole[i][0], hole[i][1])) return 2;
                for (std::size_t j = 0; j < a.vertexCount; ++j)
                    if (cross(hole[i], hole[(i + 1) % n], a.vertices[j], a.vertices[(j + 1) % a.vertexCount])) return 2;
                for (std::size_t m = k + 1; m < a.holeCount; ++m)
                    for (std::size_t j = 0; j < a.holeSizes[m]; ++j)
                        if (cross(hole[i], hole[(i + 1) % n], a.holes[m][j], a.holes[m][(j + 1) % a.holeSizes[m]])) return 2;
            }
        }
        return -1;
    }
    if (!z.vertices.empty()) return 1;
    if (!z.holes.empty()) return 2;
    if (!placed(z.latitudeRad, z.longitudeRad, z.xM, z.yM)) return 3;
    auto only = [&](std::initializer_list<int> mine) {
        for (int i = 0; i < 6; ++i)
            if (given(dims[i]) && std::find(mine.begin(), mine.end(), i) == mine.end()) return false;
        return true;
    };
    const bool quarter = isHold(z.orientationRad) || std::abs(z.orientationRad) <= 0.5 * kPi;
    switch (shape) {
    case ZoneShape::Ellipse:
        if (!only({0, 1}) || !(z.semiMajorM > 0.0) || !(z.semiMinorM > 0.0) || z.semiMinorM > z.semiMajorM) return 4;
        if (!quarter || given(z.azimuthMinRad) || given(z.azimuthMaxRad)) return 5;
        return -1;
    case ZoneShape::Rectangle:
        if (!only({2, 3}) || !(z.widthM > 0.0) || !(z.heightM > 0.0)) return 4;
        if (!quarter || given(z.azimuthMinRad) || given(z.azimuthMaxRad)) return 5;
        return -1;
    default: // a slant range area: its bearings both, within half a turn
        if (!only({4, 5}) || !(z.rangeMaxM > 0.0) || (given(z.rangeMinM) && z.rangeMinM >= z.rangeMaxM)) return 4;
        if (isHold(z.azimuthMinRad) || isHold(z.azimuthMaxRad) || std::abs(z.azimuthMinRad) > kPi || std::abs(z.azimuthMaxRad) > kPi || !quarter)
            return 5;
        return -1;
    }
}

void layOut(const OpZone& z, const FrameSpec* spec, double nowS, MustFlyArea& a) noexcept {
    a = MustFlyArea{};
    a.shape = static_cast<ZoneShape>(static_cast<int>(z.shape));
    a.framed = spec != nullptr;
    if (spec)
        a.frame = *spec, a.frameId = static_cast<FrameId>(z.frame),
        a.rotation = isHold(z.frameRotation) ? FrameRotation::Unrotated : static_cast<FrameRotation>(static_cast<int>(z.frameRotation));
    a.northMs = orHold(z.northMs, 0.0), a.eastMs = orHold(z.eastMs, 0.0), a.timeS = orHold(z.timeS, nowS);
    a.altitudeMinM = z.altitudeMinM, a.altitudeMaxM = z.altitudeMaxM;
    a.altitudeReference = isHold(z.altitudeReference) ? AltitudeReference::Msl : static_cast<AltitudeReference>(static_cast<int>(z.altitudeReference));
    if (a.shape == ZoneShape::Polygon) {
        // its reference: a framed polygon's the frame's origin; else its vertices' mean
        double lat0 = 0.0, lon0 = 0.0;
        if (!spec) {
            const double first = z.vertices.empty() ? 0.0 : z.vertices[0].longitudeRad;
            for (const ZoneVertex& v : z.vertices) lat0 += v.latitudeRad, lon0 += first + geo::wrapPi(v.longitudeRad - first);
            if (!z.vertices.empty()) lat0 /= static_cast<double>(z.vertices.size()), lon0 /= static_cast<double>(z.vertices.size());
            a.latitudeRad = lat0, a.longitudeRad = geo::wrapPi(lon0);
        }
        auto toXY = [&](const ZoneVertex& v, double& x, double& y) {
            if (spec) x = v.xM, y = v.yM;
            else geo::localNorthEastM(lat0, lon0, v.latitudeRad, lon0 + geo::wrapPi(v.longitudeRad - lon0), x, y);
        };
        a.vertexCount = static_cast<std::uint8_t>(ring(z.vertices, a.vertices, toXY));
        const std::size_t holes = std::min(z.holes.size(), MustFlyArea::kHoles);
        a.holeCount = static_cast<std::uint8_t>(holes);
        for (std::size_t k = 0; k < holes; ++k) a.holeSizes[k] = static_cast<std::uint8_t>(ring(z.holes[k], a.holes[k], toXY));
        return;
    }
    if (spec) a.frameXM = z.xM, a.frameYM = z.yM;
    else a.latitudeRad = z.latitudeRad, a.longitudeRad = geo::wrapPi(z.longitudeRad);
    a.orientationRad = orHold(z.orientationRad, 0.0);
    a.semiMajorM = orHold(z.semiMajorM, 0.0), a.semiMinorM = orHold(z.semiMinorM, 0.0);
    a.widthM = orHold(z.widthM, 0.0), a.heightM = orHold(z.heightM, 0.0);
    a.rangeMinM = orHold(z.rangeMinM, 0.0), a.rangeMaxM = orHold(z.rangeMaxM, 0.0);
    a.azimuthMinRad = orHold(z.azimuthMinRad, 0.0), a.azimuthMaxRad = orHold(z.azimuthMaxRad, 0.0);
}

bool toPlane(const MustFlyArea& a, const FramePose* pose, double nowS, double lat, double lon, double& x, double& y) noexcept {
    if (!a.framed) {
        double rlat = a.latitudeRad, rlon = a.longitudeRad; // (carried on at its velocity)
        const double t = nowS - a.timeS;
        if (a.northMs != 0.0 || a.eastMs != 0.0) geo::offsetLatLon(a.latitudeRad, a.longitudeRad, a.northMs * t, a.eastMs * t, rlat, rlon);
        geo::localNorthEastM(rlat, rlon, lat, rlon + geo::wrapPi(lon - rlon), x, y);
        return true;
    }
    if (!pose) return false;
    const double turn = turnOf(a, *pose);
    double rn = 0.0, re = 0.0, pn = 0.0, pe = 0.0;
    unalong(turn, a.frameXM, a.frameYM, rn, re); // (its reference from the origin, north and east)
    geo::localNorthEastM(pose->latitudeRad, pose->longitudeRad, lat, pose->longitudeRad + geo::wrapPi(lon - pose->longitudeRad), pn, pe);
    along(turn, pn - rn, pe - re, x, y);
    return true;
}

double turnNow(const MustFlyArea& a, const FramePose* pose) noexcept { return a.framed && pose ? turnOf(a, *pose) : 0.0; }

bool fromPlane(const MustFlyArea& a, const FramePose* pose, double nowS, double x, double y, double& lat, double& lon) noexcept {
    if (!a.framed) {
        double rlat = a.latitudeRad, rlon = a.longitudeRad;
        const double t = nowS - a.timeS;
        if (a.northMs != 0.0 || a.eastMs != 0.0) geo::offsetLatLon(a.latitudeRad, a.longitudeRad, a.northMs * t, a.eastMs * t, rlat, rlon);
        geo::offsetLatLon(rlat, rlon, x, y, lat, lon);
        lon = geo::wrapPi(lon);
        return true;
    }
    if (!pose) return false;
    const double turn = turnOf(a, *pose);
    double n = 0.0, e = 0.0;
    unalong(turn, a.frameXM + x, a.frameYM + y, n, e);
    geo::offsetLatLon(pose->latitudeRad, pose->longitudeRad, n, e, lat, lon);
    lon = geo::wrapPi(lon);
    return true;
}

bool contains(const MustFlyArea& a, double x, double y) noexcept {
    switch (a.shape) {
    case ZoneShape::Polygon: {
        if (!inside(a.vertices, a.vertexCount, x, y)) return false;
        for (std::size_t k = 0; k < a.holeCount; ++k)
            if (inside(a.holes[k], a.holeSizes[k], x, y)) return false;
        return true;
    }
    case ZoneShape::Ellipse: {
        double u = 0.0, v = 0.0;
        along(a.orientationRad, x, y, u, v);
        return (u / a.semiMajorM) * (u / a.semiMajorM) + (v / a.semiMinorM) * (v / a.semiMinorM) <= 1.0;
    }
    case ZoneShape::Rectangle: {
        double u = 0.0, v = 0.0;
        along(a.orientationRad, x, y, u, v);
        return std::abs(u) <= 0.5 * a.heightM && std::abs(v) <= 0.5 * a.widthM;
    }
    case ZoneShape::SlantRange: {
        const double r = std::hypot(x, y);
        return r >= a.rangeMinM && r <= a.rangeMaxM && withinBearings(a, std::atan2(y, x));
    }
    default: return false;
    }
}

void centre(const MustFlyArea& a, double& x, double& y) noexcept {
    x = y = 0.0;
    if (a.shape == ZoneShape::Polygon && a.vertexCount) {
        for (std::size_t i = 0; i < a.vertexCount; ++i) x += a.vertices[i][0], y += a.vertices[i][1];
        x /= a.vertexCount, y /= a.vertexCount;
    }
}

void nearest(const MustFlyArea& a, double x, double y, double& nx, double& ny, double& inX, double& inY) noexcept {
    nx = x, ny = y;
    switch (a.shape) {
    case ZoneShape::Polygon: {
        std::size_t k = 0; // (inside its outline, it is in a hole: that hole's edge)
        while (k < a.holeCount && !(inside(a.vertices, a.vertexCount, x, y) && inside(a.holes[k], a.holeSizes[k], x, y))) ++k;
        if (k < a.holeCount) nearestOnRing(a.holes[k], a.holeSizes[k], x, y, nx, ny);
        else nearestOnRing(a.vertices, a.vertexCount, x, y, nx, ny);
        break;
    }
    case ZoneShape::Ellipse: {
        // in its axes, the nearest of 720 points round it, then closed in on (its distance is smooth near there)
        double u = 0.0, v = 0.0;
        along(a.orientationRad, x, y, u, v);
        auto d2 = [&](double t) {
            const double pu = a.semiMajorM * std::cos(t) - u, pv = a.semiMinorM * std::sin(t) - v;
            return pu * pu + pv * pv;
        };
        double best = 0.0, bestD = std::numeric_limits<double>::infinity();
        for (int i = 0; i < 720; ++i) {
            const double t = 2.0 * kPi * i / 720.0, d = d2(t);
            if (d < bestD) bestD = d, best = t;
        }
        double lo = best - 2.0 * kPi / 720.0, hi = best + 2.0 * kPi / 720.0;
        for (int i = 0; i < 40; ++i) {
            const double m1 = lo + (hi - lo) / 3.0, m2 = hi - (hi - lo) / 3.0;
            if (d2(m1) < d2(m2)) hi = m2;
            else lo = m1;
        }
        const double t = 0.5 * (lo + hi);
        unalong(a.orientationRad, a.semiMajorM * std::cos(t), a.semiMinorM * std::sin(t), nx, ny);
        break;
    }
    case ZoneShape::Rectangle: {
        double u = 0.0, v = 0.0;
        along(a.orientationRad, x, y, u, v);
        u = std::clamp(u, -0.5 * a.heightM, 0.5 * a.heightM), v = std::clamp(v, -0.5 * a.widthM, 0.5 * a.widthM);
        unalong(a.orientationRad, u, v, nx, ny);
        break;
    }
    case ZoneShape::SlantRange: {
        const double r = std::hypot(x, y), b = std::atan2(y, x);
        if (withinBearings(a, b)) { // (on its bearing: out to its least range, or in to its most)
            const double to = std::clamp(r, a.rangeMinM, a.rangeMaxM);
            nx = to * std::cos(b), ny = to * std::sin(b);
            break;
        }
        double best = std::numeric_limits<double>::infinity(); // (else its nearer edge along a bound bearing)
        for (const double edge : {a.azimuthMinRad + a.orientationRad, a.azimuthMaxRad + a.orientationRad}) {
            const double ex = std::cos(edge), ey = std::sin(edge), t = std::clamp(x * ex + y * ey, a.rangeMinM, a.rangeMaxM);
            const double d = std::hypot(x - t * ex, y - t * ey);
            if (d < best) best = d, nx = t * ex, ny = t * ey;
        }
        break;
    }
    default: break;
    }
    const double dx = nx - x, dy = ny - y, d = std::hypot(dx, dy);
    if (d > 1e-9) inX = dx / d, inY = dy / d;
    else { // (on its edge: toward its centre)
        double cx = 0.0, cy = 0.0;
        centre(a, cx, cy);
        const double c = std::hypot(cx - x, cy - y);
        inX = c > 1e-9 ? (cx - x) / c : 1.0, inY = c > 1e-9 ? (cy - y) / c : 0.0;
    }
}

void ray(const MustFlyArea& a, double bearing, double& nx, double& ny) noexcept {
    double cx = 0.0, cy = 0.0;
    centre(a, cx, cy);
    const double dx = std::cos(bearing), dy = std::sin(bearing);
    double t = 0.0;
    switch (a.shape) {
    case ZoneShape::Polygon:
        for (std::size_t i = 0; i < a.vertexCount; ++i) { // (the farthest crossing of its outline)
            const double* p = a.vertices[i];
            const double* q = a.vertices[(i + 1) % a.vertexCount];
            const double ex = q[0] - p[0], ey = q[1] - p[1], den = dx * ey - dy * ex;
            if (std::abs(den) < 1e-12) continue;
            const double s = ((p[0] - cx) * ey - (p[1] - cy) * ex) / den, u = ((p[0] - cx) * dy - (p[1] - cy) * dx) / den;
            if (s >= 0.0 && u >= 0.0 && u <= 1.0) t = std::max(t, s);
        }
        break;
    case ZoneShape::Ellipse: {
        double u = 0.0, v = 0.0;
        along(a.orientationRad, dx, dy, u, v);
        t = 1.0 / std::sqrt((u / a.semiMajorM) * (u / a.semiMajorM) + (v / a.semiMinorM) * (v / a.semiMinorM));
        break;
    }
    case ZoneShape::Rectangle: {
        double u = 0.0, v = 0.0;
        along(a.orientationRad, dx, dy, u, v);
        t = std::min(std::abs(u) > 1e-12 ? 0.5 * a.heightM / std::abs(u) : std::numeric_limits<double>::infinity(),
                     std::abs(v) > 1e-12 ? 0.5 * a.widthM / std::abs(v) : std::numeric_limits<double>::infinity());
        break;
    }
    case ZoneShape::SlantRange: { // (its outer arc, the bearing held within its extent)
        double b = bearing;
        if (!withinBearings(a, b)) {
            const double lo = a.azimuthMinRad + a.orientationRad, hi = a.azimuthMaxRad + a.orientationRad;
            b = std::abs(geo::wrapPi(b - lo)) <= std::abs(geo::wrapPi(b - hi)) ? lo : hi;
        }
        nx = a.rangeMaxM * std::cos(b), ny = a.rangeMaxM * std::sin(b);
        return;
    }
    default: break;
    }
    nx = cx + t * dx, ny = cy + t * dy;
}

double extent(const MustFlyArea& a) noexcept {
    switch (a.shape) {
    case ZoneShape::Polygon: {
        double cx = 0.0, cy = 0.0, far = 0.0;
        centre(a, cx, cy);
        for (std::size_t i = 0; i < a.vertexCount; ++i) far = std::max(far, std::hypot(a.vertices[i][0] - cx, a.vertices[i][1] - cy));
        return far;
    }
    case ZoneShape::Ellipse: return a.semiMajorM;
    case ZoneShape::Rectangle: return 0.5 * std::hypot(a.widthM, a.heightM);
    case ZoneShape::SlantRange: return a.rangeMaxM;
    default: return 0.0;
    }
}

} // namespace fsim::control::zones
