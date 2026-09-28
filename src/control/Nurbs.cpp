// A curve segment as A-GRA's schema gives it (MA_NURBS_PointType; docs/flight-autonomy.md, 4.26): a clamped rational
// B-spline, its point and first two derivatives, and whether its counts, knots and weights make one. Published
// mathematics: the B-spline basis by the Cox-de Boor recursion and its derivatives (Piegl and Tiller, The NURBS Book,
// 2nd ed., eqs. 2.5 and 2.9, the triangle of algorithm A2.3), a rational curve's derivatives by the quotient rule (its
// eq. 4.8). A Bezier's form is not flown here: Route.cpp's Bernstein basis flies it, as a BezierSegment.
#include "control/Route.h"

#include "control/CapabilityHost.h"
#include "control/ControlStack.h"
#include "control/Runtime.h"
#include "core/Geodesy.h"
#include "fsim/Altimeter.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fsim::control::route {

CurvePoint rational(const NurbsSegment& s, double t) noexcept {
    const int n = static_cast<int>(s.points) - 1, m = static_cast<int>(s.knots) - 1, p = m - n - 1;
    const double* U = s.knot;
    const double u0 = U[p], u1 = U[m - p], span = u1 - u0;
    const double u = u0 + std::clamp(t, 0.0, 1.0) * span;
    // the knot span U[k] <= u < U[k + 1]; at the end, the last that is not empty
    int k = n;
    if (u < U[n + 1]) {
        k = p;
        while (k < n && !(u < U[k + 1])) ++k;
    }
    // the basis functions N[k-p..k] at u, and their first two derivatives: the triangle of knot differences (all
    // above 0 on a span that is not empty) and the basis beside it
    constexpr int kMax = static_cast<int>(NurbsSegment::kPoints);
    double ndu[kMax][kMax], left[kMax], right[kMax];
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - U[k + 1 - j], right[j] = U[k + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            const double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    double ders[3][kMax];
    for (int j = 0; j <= p; ++j) ders[0][j] = ndu[j][p], ders[1][j] = ders[2][j] = 0.0;
    const int nd = std::min(2, p); // (a line's second derivative is none)
    for (int r = 0; r <= p; ++r) {
        double a[2][kMax];
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;
        for (int d = 1; d <= nd; ++d) {
            double v = 0.0;
            const int rk = r - d, pk = p - d;
            if (r >= d) a[s2][0] = a[s1][0] / ndu[pk + 1][rk], v = a[s2][0] * ndu[rk][pk];
            const int j1 = rk >= -1 ? 1 : -rk, j2 = r - 1 <= pk ? d - 1 : p - r;
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                v += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) a[s2][d] = -a[s1][d - 1] / ndu[pk + 1][r], v += a[s2][d] * ndu[r][pk];
            ders[d][r] = v;
            std::swap(s1, s2);
        }
    }
    double factor = p;
    for (int d = 1; d <= nd; ++d) {
        for (int j = 0; j <= p; ++j) ders[d][j] *= factor;
        factor *= p - d;
    }
    // the weighted sums - each axis's numerator, and the weights' - with their derivatives
    double A[3][3] = {}, W[3] = {};
    const double* axes[3] = {s.north, s.east, s.down};
    for (int j = 0; j <= p; ++j) {
        const int i = k - p + j;
        const double w = s.weight[i];
        for (int d = 0; d <= 2; ++d) {
            W[d] += ders[d][j] * w;
            for (int a = 0; a < 3; ++a) A[d][a] += ders[d][j] * w * axes[a][i];
        }
    }
    // C = A / W, C' = (A' - W' C) / W, C'' = (A'' - 2 W' C' - W'' C) / W: by u, then by t (u = u0 + t span)
    CurvePoint c;
    for (int a = 0; a < 3; ++a) {
        c.p[a] = A[0][a] / W[0];
        const double d1 = (A[1][a] - W[1] * c.p[a]) / W[0];
        const double d2 = (A[2][a] - 2.0 * W[1] * d1 - W[2] * c.p[a]) / W[0];
        c.d1[a] = d1 * span, c.d2[a] = d2 * span * span;
    }
    return c;
}

bool wellFormed(const NurbsSegment& s) noexcept {
    const std::uint32_t n = s.points, m = s.knots;
    if (n < 4 || n > NurbsSegment::kPoints || m < 4 || m > NurbsSegment::kKnots || m < n + 2) return false;
    const auto p = static_cast<std::uint32_t>(s.degree()); // (1 or more)
    for (std::uint32_t i = 0; i < n; ++i)
        if (!std::isfinite(s.north[i]) || !std::isfinite(s.east[i]) || !std::isfinite(s.down[i]) || !std::isfinite(s.weight[i]) || !(s.weight[i] > 0.0))
            return false;
    for (std::uint32_t i = 0; i < m; ++i)
        if (!std::isfinite(s.knot[i]) || s.knot[i] < 0.0 || (i > 0 && s.knot[i] < s.knot[i - 1])) return false;
    // clamped: its first knot degree + 1 times and no more, and its last
    for (std::uint32_t i = 1; i <= p; ++i)
        if (s.knot[i] != s.knot[0] || s.knot[m - 1 - i] != s.knot[m - 1]) return false;
    const double u0 = s.knot[p], u1 = s.knot[m - 1 - p];
    if (!(u1 > u0) || !(s.knot[p + 1] > u0) || !(s.knot[m - 2 - p] < u1)) return false;
    // within the domain, no knot more often than the degree: the curve would break there
    std::uint32_t run = 1;
    for (std::uint32_t i = p + 2; i < m - 1 - p; ++i) {
        run = s.knot[i] == s.knot[i - 1] ? run + 1 : 1;
        if (run > p) return false;
    }
    if (!isHold(s.curvature) && !(std::isfinite(s.curvature) && s.curvature > 0.0)) return false;
    if (!isHold(s.firstIndex) && s.firstIndex != 0.0) return false;                         // (a clamped curve starts at its first point)
    if (!isHold(s.lastIndex) && s.lastIndex != static_cast<double>(n - 1)) return false; // (and ends at its last)
    return true;
}

void Curve::normalize(std::uint32_t from, CurveZ z) noexcept {
    if (z == CurveZ::Down) return;
    for (std::uint32_t i = from; i < count; ++i)
        for (std::uint32_t j = 0; j < segments[i].points; ++j) // (an affine change of its points is its curve's)
            segments[i].down[j] = z == CurveZ::AltitudeOffset ? -segments[i].down[j] : alt0 - segments[i].down[j];
}

Attitude::Attitude(const FramePose& pose) noexcept {
    // the body's axes over north, east and down (its roll, pitch and yaw), as framePoint turns a point's offsets
    const double cf = std::cos(pose.rollRad), sf = std::sin(pose.rollRad), ct = std::cos(pose.pitchRad), st = std::sin(pose.pitchRad);
    const double cp = std::cos(pose.yawRad), sp = std::sin(pose.yawRad);
    m[0][0] = ct * cp, m[0][1] = sf * st * cp - cf * sp, m[0][2] = cf * st * cp + sf * sp;
    m[1][0] = ct * sp, m[1][1] = sf * st * sp + cf * cp, m[1][2] = cf * st * sp - sf * cp;
    m[2][0] = -st, m[2][1] = sf * ct, m[2][2] = cf * ct;
}

void Attitude::turn(double x, double y, double z, double& north, double& east, double& down) const noexcept {
    north = m[0][0] * x + m[0][1] * y + m[0][2] * z;
    east = m[1][0] * x + m[1][1] * y + m[1][2] * z;
    down = m[2][0] * x + m[2][1] * y + m[2][2] * z;
}

void Curve::orient(std::uint32_t from, const Attitude& attitude, CurveZ z) noexcept {
    const bool absolute = z == CurveZ::AbsoluteAltitude; // (its height no offset in the frame: only its x and y turned)
    for (std::uint32_t i = from; i < count; ++i) {
        NurbsSegment& s = segments[i];
        for (std::uint32_t j = 0; j < s.points; ++j) {
            double down;
            attitude.turn(s.north[j], s.east[j], absolute ? 0.0 : s.down[j], s.north[j], s.east[j], down);
            if (!absolute) s.down[j] = down;
        }
    }
}

namespace {

constexpr double kR = geo::kEarthRadiusM;
constexpr double kPi = 3.14159265358979323846;

/// A rhumb line's from (lat0, lat): its latitude difference over its Mercator one (the parallel's cosine where level).
double rhumbScale(double lat0, double lat) noexcept {
    const double dPsi = std::log(std::tan(0.25 * kPi + 0.5 * std::clamp(lat, -1.5, 1.5)) / std::tan(0.25 * kPi + 0.5 * std::clamp(lat0, -1.5, 1.5)));
    return std::abs(dPsi) > 1e-12 ? (lat - lat0) / dPsi : std::cos(lat0);
}

} // namespace

double frameTurn(const FramePose& pose, FrameRotation rotation) noexcept {
    switch (rotation) { // (as a frame's point is turned: Frames.cpp)
    case FrameRotation::Yaw: return pose.yawRad;
    case FrameRotation::Heading: return std::hypot(pose.northMs, pose.eastMs) > 0.1 ? std::atan2(pose.eastMs, pose.northMs) : pose.yawRad;
    default: return 0.0;
    }
}

void earthToPlane(double lat0, double lon0, FrameOffsets offsets, double psi, double lat, double lon, double& north, double& east) noexcept {
    double n, e;
    switch (offsets) {
    case FrameOffsets::GreatCircle: { // (A-GRA's azimuthal equidistant layout)
        const double d = geo::distanceM(lat0, lon0, lat, lon), az = geo::bearingRad(lat0, lon0, lat, lon);
        n = d * std::cos(az), e = d * std::sin(az);
        break;
    }
    case FrameOffsets::Rhumb: // north along the meridian, east the rhumb line's departure (as a frame's, Frames.cpp)
        n = (lat - lat0) * kR, e = geo::wrapPi(lon - lon0) * kR * rhumbScale(lat0, lat);
        break;
    default: geo::localNorthEastM(lat0, lon0, lat, lon, n, e); break;
    }
    const double cp = std::cos(psi), sp = std::sin(psi);
    north = n * cp + e * sp, east = -n * sp + e * cp;
}

void planeToEarth(double lat0, double lon0, FrameOffsets offsets, double psi, double north, double east, double& lat, double& lon) noexcept {
    const double cp = std::cos(psi), sp = std::sin(psi);
    const double n = north * cp - east * sp, e = north * sp + east * cp;
    switch (offsets) {
    case FrameOffsets::GreatCircle:
        if (const double d = std::hypot(n, e); d > 0.0) geo::destination(lat0, lon0, std::atan2(e, n), d, lat, lon);
        else lat = lat0, lon = lon0;
        break;
    case FrameOffsets::Rhumb:
        lat = std::clamp(lat0 + n / kR, -0.5 * kPi, 0.5 * kPi);
        lon = geo::wrapPi(lon0 + e / (kR * rhumbScale(lat0, lat)));
        break;
    default: geo::offsetLatLon(lat0, lon0, n, e, lat, lon); break;
    }
}

bool sharperThan(const NurbsSegment& s, double curvature, double& from, double& to) noexcept {
    constexpr int kSteps = 64;
    const double limit = 1.01 * curvature; // (what sampling it misses)
    bool found = false;
    for (int k = 0; k <= kSteps; ++k) {
        const double t = static_cast<double>(k) / kSteps;
        const bool over = std::abs(evaluate(s, t).curvature()) > limit;
        if (over && !found) found = true, from = t;
        if (over) to = t;
        if (!over && found) break; // the first section only
    }
    return found;
}

} // namespace fsim::control::route

namespace fsim::control {

namespace {

/// Bezier segments as NurbsSegments, at most `room` of them into `out` (a command gives ten at most: eleven are still
/// refused as too many).
Span<const NurbsSegment> asNurbs(Span<const BezierSegment> in, NurbsSegment* out, std::size_t room) noexcept {
    const std::size_t n = std::min(in.size(), room);
    for (std::size_t i = 0; i < n; ++i) out[i] = NurbsSegment::of(in[i]);
    return Span<const NurbsSegment>(out, n);
}

} // namespace

CommandResult CapabilityHost::submit(const CurveCommand& curve, Span<const BezierSegment> segments, const CommandOptions& options,
                                     const sim::VehicleState& state, double now, const CurveShape* shape) {
    NurbsSegment made[11];
    return submit(curve, asNurbs(segments, made, 11), options, state, now, shape);
}

CommandResult CapabilityHost::update(ActivityId activity, const CurveCommand& curve, Span<const BezierSegment> segments,
                                     const sim::VehicleState& state, Caller caller, const CurveShape* shape) noexcept {
    NurbsSegment made[11];
    return update(activity, curve, asNurbs(segments, made, 11), state, caller, shape);
}

void mergeCurveShape(CurveCommand& curve, CurveShape& shape, const CurveCommand& given, const CurveShape& givenShape) noexcept {
    double* to[CurveShape::kFields];
    double* from[CurveShape::kFields];
    CurveShape copy = givenShape;
    shape.fields(to), copy.fields(from);
    for (std::size_t i = 0; i < CurveShape::kFields; ++i)
        if (!isHold(*from[i])) *to[i] = *from[i];
    if (!isHold(given.latitudeRad) || !isHold(given.longitudeRad)) shape = CurveShape{}; // (a point replaces a frame)
    if (!isHold(givenShape.frame)) curve.latitudeRad = curve.longitudeRad = kHold;         // (and a frame a point: placed again from it)
}

int curveWhereField(const CurveCommand& given, const CurveShape* shape) noexcept {
    const double where[] = {given.latitudeRad, given.longitudeRad, given.altitudeM, given.altitudeReference, given.altitudeMinM, given.altitudeMaxM,
                            given.pointRotation, given.pointOffsets, given.pointZ};
    const int field[] = {0, 1, 2, 8, 9, 10, 11, 12, 13};
    for (int i = 0; i < 9; ++i)
        if (!isHold(where[i])) return field[i];
    if (shape) {
        CurveShape copy = *shape;
        double* f[CurveShape::kFields];
        copy.fields(f);
        for (std::size_t i = 0; i < CurveShape::kFields; ++i)
            if (!isHold(*f[i])) return 14 + static_cast<int>(i);
    }
    return -1;
}

double CapabilityHost::curveTurn(const CurveCommand& c, const CurveShape& shape) const noexcept {
    if (isHold(shape.frame) || isHold(c.pointRotation)) return 0.0;
    FrameSpec spec;
    FramePose now;
    if (!sessionView_ || !sessionView_->frame(static_cast<FrameId>(shape.frame), spec, now)) return 0.0;
    return route::frameTurn(now, static_cast<FrameRotation>(static_cast<int>(c.pointRotation)));
}

bool CapabilityHost::curveAttitude(const CurveCommand& c, const CurveShape& shape, FramePose& pose) const noexcept {
    if (isHold(shape.frame) || c.pointRotation != static_cast<double>(FrameRotation::Attitude)) return false;
    FrameSpec spec;
    return sessionView_ && sessionView_->frame(static_cast<FrameId>(shape.frame), spec, pose);
}

Reason CapabilityHost::placeCurve(CurveCommand& c, const sim::VehicleState& state, CommandResult& detail) noexcept {
    const CurveShape& shape = curveShape_;
    auto bad = [&detail](std::int16_t field, Reason why = Reason::InvalidParameter) {
        detail.index = field;
        return why;
    };
    auto code = [](double v, double count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
    auto finite = [](double v) { return isHold(v) || std::isfinite(v); };
    // its frame's fields, whole and in range; its offsets only with a frame
    if (!isHold(shape.frame) && !(shape.frame == std::floor(shape.frame) && shape.frame >= 1.0 && shape.frame <= 9007199254740992.0)) return bad(14);
    if (!code(shape.frameRotation, static_cast<double>(FrameRotation::Count))) return bad(15);
    if (!code(shape.frameOffsets, static_cast<double>(FrameOffsets::Count))) return bad(16);
    if (!finite(shape.frameXM)) return bad(17);
    if (!finite(shape.frameYM)) return bad(18);
    if (!finite(shape.frameZM)) return bad(19);
    if (isHold(shape.frame)) {
        const double offsets[] = {shape.frameRotation, shape.frameOffsets, shape.frameXM, shape.frameYM, shape.frameZM};
        for (int i = 0; i < 5; ++i)
            if (!isHold(offsets[i])) return bad(static_cast<std::int16_t>(15 + i));
    }
    // its points turned as its frame is: with one
    const auto rotation = isHold(c.pointRotation) ? FrameRotation::Unrotated : static_cast<FrameRotation>(static_cast<int>(c.pointRotation));
    if (rotation != FrameRotation::Unrotated && isHold(shape.frame)) return bad(11);
    // in its frame: where the frame is now (its height, given a z), its axes turned with it (in three dimensions, by its pose)
    curveTurn_ = 0.0;
    if (!isHold(shape.frame)) {
        FramePose now;
        if (!sessionView_ || !sessionView_->frame(static_cast<FrameId>(shape.frame), curveFrame_, now)) return bad(14);
        const GeoPoint at = framePoint(now, shape.frameOffset());
        c.latitudeRad = at.latitudeRad, c.longitudeRad = geo::wrapPi(at.longitudeRad);
        if (!isHold(shape.frameZM)) c.altitudeM = at.altitudeMslM, c.altitudeReference = static_cast<double>(AltitudeReference::Msl);
        curveTurn_ = route::frameTurn(now, rotation);
        curvePose_ = now;
    }
    // left out: where the aircraft is, at its altitude in the curve's reference, held within its range
    if (isHold(c.latitudeRad)) c.latitudeRad = state.latitudeRad, c.longitudeRad = state.longitudeRad;
    const auto reference = isHold(c.altitudeReference) ? AltitudeReference::Msl : static_cast<AltitudeReference>(static_cast<int>(c.altitudeReference));
    if (isHold(c.altitudeM)) {
        c.altitudeM = reference == AltitudeReference::Msl ? state.altitudeMslM : altitudeNow(reference, state, &config_->altimeter);
        if (!isHold(c.altitudeMinM)) c.altitudeM = std::max(c.altitudeM, c.altitudeMinM);
        if (!isHold(c.altitudeMaxM)) c.altitudeM = std::min(c.altitudeM, c.altitudeMaxM);
    } else if ((!isHold(c.altitudeMinM) && c.altitudeM < c.altitudeMinM) || (!isHold(c.altitudeMaxM) && c.altitudeM > c.altitudeMaxM)) {
        return bad(2);
    }
    return Reason::None;
}

void ControlStack::command(const CurveCommand& curve, Span<const NurbsSegment> segments) {
    // into the stack's own path store, as a World's host writes it
    if (!config_->path) config_->path = std::make_unique<PathStore>();
    PathStore& path = *config_->path;
    path.segmentCount = static_cast<std::uint32_t>(std::min(segments.size(), PathStore::kSegments));
    std::copy_n(segments.data(), path.segmentCount, path.segments);
    ++path.curve, ++path.revision;
    command(Command(curve));
}

} // namespace fsim::control
