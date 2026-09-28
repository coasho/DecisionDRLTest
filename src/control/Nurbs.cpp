// A curve segment as A-GRA's schema gives it (MA_NURBS_PointType; docs/flight-autonomy.md, 4.26): a clamped rational
// B-spline, its point and first two derivatives, and whether its counts, knots and weights make one. Published
// mathematics: the B-spline basis by the Cox-de Boor recursion and its derivatives (Piegl and Tiller, The NURBS Book,
// 2nd ed., eqs. 2.5 and 2.9, the triangle of algorithm A2.3), a rational curve's derivatives by the quotient rule (its
// eq. 4.8). A Bezier's form is not flown here: Route.cpp's Bernstein basis flies it, as a BezierSegment.
#include "control/Route.h"

#include "control/CapabilityHost.h"
#include "control/ControlStack.h"
#include "control/Runtime.h"

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
                                     const sim::VehicleState& state, double now) {
    NurbsSegment made[11];
    return submit(curve, asNurbs(segments, made, 11), options, state, now);
}

CommandResult CapabilityHost::update(ActivityId activity, const CurveCommand& curve, Span<const BezierSegment> segments,
                                     const sim::VehicleState& state, Caller caller) noexcept {
    NurbsSegment made[11];
    return update(activity, curve, asNurbs(segments, made, 11), state, caller);
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
