// A curve's segments as A-GRA's schema gives them (docs/flight-autonomy.md, 4.26; ADR-29 FA-5d): clamped rational
// B-splines, measured against the Cox-de Boor recursion evaluated here, apart from the platform's triangle of knot
// differences - a Bezier's form as the Bernstein basis flies it, an exact circle as a rational quadratic, the
// derivatives against finite differences; what is not a clamped curve refused, naming the segment, and a curve that
// turns tighter than the curvature it declares, naming the section; flown by a wing and a multirotor.
#include "control/Route.h"
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

/// The B-spline basis N(i, p) at u by Cox and de Boor's recursion: each span half open, the last one closed.
double basis(int i, int p, double u, const double* U, int m) {
    if (p == 0) {
        if (U[i] <= u && u < U[i + 1]) return 1.0;
        return u == U[m] && U[i] < U[i + 1] && U[i + 1] == U[m] ? 1.0 : 0.0;
    }
    double a = 0.0, b = 0.0;
    if (U[i + p] > U[i]) a = (u - U[i]) / (U[i + p] - U[i]) * basis(i, p - 1, u, U, m);
    if (U[i + p + 1] > U[i + 1]) b = (U[i + p + 1] - u) / (U[i + p + 1] - U[i + 1]) * basis(i + 1, p - 1, u, U, m);
    return a + b;
}

/// A segment's point at t (0 to 1 over its knots' domain): the weighted sum of its points over its weights'.
std::array<double, 3> reference(const NurbsSegment& s, double t) {
    const int n = static_cast<int>(s.points) - 1, m = static_cast<int>(s.knots) - 1, p = m - n - 1;
    const double u = s.knot[p] + std::clamp(t, 0.0, 1.0) * (s.knot[m - p] - s.knot[p]);
    std::array<double, 3> num{};
    double den = 0.0;
    for (int i = 0; i <= n; ++i) {
        const double b = basis(i, p, u, s.knot, m) * s.weight[i];
        num[0] += b * s.north[i], num[1] += b * s.east[i], num[2] += b * s.down[i];
        den += b;
    }
    for (double& v : num) v /= den;
    return num;
}

/// A clamped segment of the given degree through `points` (north, east, down), with `weights` and its interior knots.
NurbsSegment clamped(int degree, const std::vector<std::array<double, 3>>& points, const std::vector<double>& weights, const std::vector<double>& interior) {
    NurbsSegment s;
    s.points = static_cast<std::uint32_t>(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) s.north[i] = points[i][0], s.east[i] = points[i][1], s.down[i] = points[i][2], s.weight[i] = weights[i];
    std::vector<double> k(static_cast<std::size_t>(degree + 1), 0.0);
    k.insert(k.end(), interior.begin(), interior.end());
    k.insert(k.end(), static_cast<std::size_t>(degree + 1), 1.0);
    s.knots = static_cast<std::uint32_t>(k.size());
    std::copy(k.begin(), k.end(), s.knot);
    REQUIRE(s.knots == s.points + static_cast<std::uint32_t>(degree) + 1);
    return s;
}

/// A semicircle of radius r about (cn, ce) as a rational quadratic (NURBS's exact circle): from its west round its
/// north to its east, right turns; two quarters meeting at a double knot.
NurbsSegment semicircle(double cn, double ce, double r, double down) {
    const double w = std::sqrt(0.5);
    return clamped(2, {{cn, ce - r, down}, {cn + r, ce - r, down}, {cn + r, ce, down}, {cn + r, ce + r, down}, {cn, ce + r, down}}, {1.0, w, 1.0, w, 1.0},
                   {0.5, 0.5});
}

/// A rational cubic S along north: `length` long, swinging `swing` east and back, its points weighted unevenly.
NurbsSegment cubicS(double n0, double e0, double length, double swing, double climb) {
    const double L = length;
    return clamped(3,
                   {{n0, e0, 0.0},
                    {n0 + 0.15 * L, e0, 0.0},
                    {n0 + 0.3 * L, e0 + swing, -0.2 * climb},
                    {n0 + 0.5 * L, e0 + swing, -0.5 * climb},
                    {n0 + 0.7 * L, e0 - swing, -0.8 * climb},
                    {n0 + 0.85 * L, e0, -climb},
                    {n0 + L, e0, -climb}},
                   {1.0, 1.4, 0.8, 1.0, 1.25, 0.9, 1.0}, {0.2, 0.45, 0.7});
}

/// Where the aircraft is from (lat0, lon0): north and east, m.
std::array<double, 2> from(const sim::VehicleState& s, double lat0, double lon0) {
    return {(s.latitudeRad - lat0) * kR, (s.longitudeRad - lon0) * kR * std::cos(0.5 * (s.latitudeRad + lat0))};
}

/// The curve as a dense polyline by the reference evaluation: the nearest distance to it over the ground.
struct Line {
    std::vector<std::array<double, 3>> points;
    explicit Line(const std::vector<NurbsSegment>& segments) {
        for (const auto& s : segments)
            for (int k = points.empty() ? 0 : 1; k <= 2000; ++k) points.push_back(reference(s, k / 2000.0));
    }
    double off(double north, double east, std::size_t& hint) const {
        const std::size_t lo = hint > 300 ? hint - 300 : 0, hi = std::min(points.size() - 1, hint + 1500);
        double best = 1e18;
        std::size_t at = hint;
        for (std::size_t j = lo; j < hi; ++j) {
            const auto& a = points[j];
            const auto& b = points[j + 1];
            const double bn = b[0] - a[0], be = b[1] - a[1], span = bn * bn + be * be;
            const double u = span > 0.0 ? std::clamp(((north - a[0]) * bn + (east - a[1]) * be) / span, 0.0, 1.0) : 0.0;
            if (const double d = std::hypot(a[0] + u * bn - north, a[1] + u * be - east); d < best) best = d, at = j;
        }
        hint = at;
        return best;
    }
};

} // namespace

TEST_CASE("nurbs: a clamped rational B-spline evaluated as the Cox-de Boor recursion has it; a circle exact; a Bezier's form as its basis", "[modes]") {
    // an uneven rational cubic, and a quartic with a double interior knot: each point, and the derivatives by differences
    const NurbsSegment s = cubicS(100.0, -50.0, 3000.0, 450.0, 120.0);
    NurbsSegment q = clamped(4, {{0.0, 0.0, 0.0}, {300.0, 50.0, -10.0}, {500.0, 400.0, -30.0}, {800.0, 450.0, 0.0}, {1000.0, 100.0, 20.0}, {1200.0, 0.0, 0.0},
                                 {1500.0, 50.0, 0.0}},
                             {1.0, 2.0, 0.5, 1.0, 1.5, 1.0, 1.0}, {0.4, 0.4});
    REQUIRE(route::wellFormed(s));
    REQUIRE(route::wellFormed(q));
    double worst = 0.0, worstD1 = 0.0, worstD2 = 0.0;
    const NurbsSegment* both[2] = {&s, &q};
    for (const NurbsSegment* seg : both) {
        for (int k = 0; k <= 200; ++k) {
            const double t = k / 200.0;
            const route::CurvePoint c = route::rational(*seg, t);
            const auto r = reference(*seg, t);
            for (int a = 0; a < 3; ++a) worst = std::max(worst, std::abs(c.p[a] - r[static_cast<std::size_t>(a)]));
            // (central differences of the reference, Richardson's of steps h and h/2: their error of order h^4 - away from its
            // ends and from the knots where a higher derivative breaks)
            const double h = 5e-4;
            const bool near = t < 2 * h || t > 1.0 - 2 * h || std::abs(t - 0.4) < 3 * h || std::abs(t - 0.2) < 3 * h || std::abs(t - 0.45) < 3 * h ||
                              std::abs(t - 0.7) < 3 * h;
            if (near) continue;
            const auto a0 = reference(*seg, t - h), a1 = reference(*seg, t + h), b0 = reference(*seg, t - 0.5 * h), b1 = reference(*seg, t + 0.5 * h);
            double e1 = 0.0, e2 = 0.0, m1 = 0.0, m2 = 0.0; // (each derivative's error against its size, as vectors)
            for (int a = 0; a < 3; ++a) {
                const auto i = static_cast<std::size_t>(a);
                const double wide1 = (a1[i] - a0[i]) / (2 * h), narrow1 = (b1[i] - b0[i]) / h;
                const double wide2 = (a1[i] - 2 * r[i] + a0[i]) / (h * h), narrow2 = (b1[i] - 2 * r[i] + b0[i]) / (0.25 * h * h);
                const double d1 = (4.0 * narrow1 - wide1) / 3.0, d2 = (4.0 * narrow2 - wide2) / 3.0;
                e1 += (c.d1[a] - d1) * (c.d1[a] - d1), m1 += d1 * d1;
                e2 += (c.d2[a] - d2) * (c.d2[a] - d2), m2 += d2 * d2;
            }
            worstD1 = std::max(worstD1, std::sqrt(e1 / m1));
            worstD2 = std::max(worstD2, std::sqrt(e2 / m2));
        }
    }
    // the ends: its first point and its last
    const route::CurvePoint start = route::rational(s, 0.0), end = route::rational(s, 1.0);
    // a circle: its points 700 m from the centre, its curvature 1/700
    const NurbsSegment arc = semicircle(200.0, 300.0, 700.0, -40.0);
    REQUIRE(route::wellFormed(arc));
    double radiusOff = 0.0, curvatureOff = 0.0;
    for (int k = 0; k <= 400; ++k) {
        const route::CurvePoint c = route::rational(arc, k / 400.0);
        radiusOff = std::max(radiusOff, std::abs(std::hypot(c.p[0] - 200.0, c.p[1] - 300.0) - 700.0));
        curvatureOff = std::max(curvatureOff, std::abs(c.curvature() - 1.0 / 700.0) * 700.0);
    }
    // a Bezier's form: the rational basis against Bernstein's, and flown by Bernstein's
    BezierSegment b;
    for (int i = 0; i < 6; ++i) b.north[i] = 400.0 * i + 30.0 * i * i, b.east[i] = 250.0 * std::sin(i), b.down[i] = -12.0 * i;
    const NurbsSegment asNurbs = NurbsSegment::of(b);
    REQUIRE(asNurbs.bezier());
    REQUIRE(route::wellFormed(asNurbs));
    double bezierOff = 0.0, same = 0.0;
    for (int k = 0; k <= 100; ++k) {
        const double t = k / 100.0;
        const route::CurvePoint x = route::evaluate(b, t), y = route::rational(asNurbs, t), z = route::evaluate(asNurbs, t);
        for (int a = 0; a < 3; ++a) {
            bezierOff = std::max({bezierOff, std::abs(x.p[a] - y.p[a]), std::abs(x.d1[a] - y.d1[a]) * 1e-3, std::abs(x.d2[a] - y.d2[a]) * 1e-6});
            same = std::max({same, std::abs(x.p[a] - z.p[a]), std::abs(x.d1[a] - z.d1[a]), std::abs(x.d2[a] - z.d2[a])});
        }
    }
    std::printf("nurbs: points %.2e m off the recursion, derivatives %.2e and %.2e off its differences; a circle %.2e m off its radius, its curvature "
                "%.2e off; a Bezier's form %.2e off Bernstein's (flown by it: %.0e)\n",
                worst, worstD1, worstD2, radiusOff, curvatureOff, bezierOff, same);
    CHECK(worst < 1e-9);
    CHECK(worstD1 < 1e-8); // (8.3e-10: a sixteenth of step 1e-3's - the extrapolation's own error, of order h^4)
    CHECK(worstD2 < 1e-7); // (1.5e-8: rounding's, as at step 1e-3)
    CHECK(std::abs(start.p[0] - 100.0) < 1e-9);
    CHECK(std::abs(start.p[1] + 50.0) < 1e-9);
    CHECK(std::abs(end.p[0] - 3100.0) < 1e-9);
    CHECK(std::abs(end.p[2] + 120.0) < 1e-9);
    CHECK(radiusOff < 1e-9);
    CHECK(curvatureOff < 1e-9);
    CHECK(bezierOff < 1e-9);
    CHECK(same == 0.0);
}

TEST_CASE("nurbs: what is not a clamped curve is refused, naming the segment; a curve turning tighter than it says, naming the section", "[modes]") {
    const NurbsSegment good = cubicS(0.0, 0.0, 8000.0, 400.0, 0.0); // (gentle enough for a C172x's full bank at 55 m/s)
    REQUIRE(route::wellFormed(good));
    struct Case {
        const char* what;
        NurbsSegment s;
    };
    std::vector<Case> cases;
    auto with = [&](const char* what, auto change) {
        NurbsSegment s = good;
        change(s);
        cases.push_back({what, s});
    };
    with("three points", [](NurbsSegment& s) { s.points = 3, s.knots = 7; });
    with("eleven points", [](NurbsSegment& s) { s.points = 11, s.knots = 14; });
    with("fifteen knots", [](NurbsSegment& s) { s.knots = 15; });
    with("no degree", [](NurbsSegment& s) { s.knots = s.points + 1; });
    with("a point not finite", [](NurbsSegment& s) { s.east[3] = std::nan(""); });
    with("a weight of 0", [](NurbsSegment& s) { s.weight[2] = 0.0; });
    with("a negative knot", [](NurbsSegment& s) { s.knot[0] = s.knot[1] = s.knot[2] = s.knot[3] = -1.0; });
    with("knots falling", [](NurbsSegment& s) { s.knot[5] = 0.1; });
    with("not clamped at its start", [](NurbsSegment& s) { s.knot[3] = 0.1; });
    with("not clamped at its end", [](NurbsSegment& s) { s.knot[7] = 0.9; });
    with("its first knot once too often", [](NurbsSegment& s) { s.knot[4] = 0.0; });
    { // a break: a quadratic's interior knot three times
        const NurbsSegment broken = clamped(2, {{0.0, 0.0, 0.0}, {500.0, 0.0, 0.0}, {1000.0, 100.0, 0.0}, {1500.0, 0.0, 0.0}, {2000.0, -100.0, 0.0},
                                                {2500.0, 0.0, 0.0}, {3000.0, 0.0, 0.0}},
                                            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.2, 0.5, 0.5, 0.5});
        cases.push_back({"a break within it", broken});
    }
    with("a curvature of 0", [](NurbsSegment& s) { s.curvature = 0.0; });
    with("its first index 1", [](NurbsSegment& s) { s.firstIndex = 1.0; });
    with("its last index not its last point", [](NurbsSegment& s) { s.lastIndex = 5.0; });
    for (const Case& c : cases) {
        INFO(c.what);
        CHECK_FALSE(route::wellFormed(c.s));
    }
    NurbsSegment said = good;
    said.firstIndex = 0.0, said.lastIndex = 6.0, said.curvature = 1.0;
    CHECK(route::wellFormed(said));

    // flown: a malformed segment refused naming it; a curvature said too small refused naming the section
    session::World w(options("nurbs-refusals"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    w.step(stepsFor(w, 2.0));
    NurbsSegment second = cubicS(8000.0, 0.0, 8000.0, 400.0, 0.0);
    second.knot[3] = 0.1; // (not clamped)
    const NurbsSegment pair[2] = {good, second};
    CommandResult r = w.submit(v, CurveCommand{}, Span<const NurbsSegment>(pair, 2));
    CHECK(r.reason == Reason::InvalidCurve);
    CHECK(r.index == 1);
    double most = 0.0; // the most it turns
    for (int k = 0; k <= 400; ++k) most = std::max(most, std::abs(route::rational(good, k / 400.0).curvature()));
    NurbsSegment tight = good;
    tight.curvature = 0.8 * most;
    r = w.submit(v, CurveCommand{}, Span<const NurbsSegment>(&tight, 1));
    std::printf("nurbs refusals: a curvature said %.2e where it turns %.2e: %s at %d, from %.3f to %.3f\n", tight.curvature, most, reasonName(r.reason), r.index,
                static_cast<double>(r.from), static_cast<double>(r.to));
    CHECK(r.reason == Reason::InvalidCurve);
    CHECK(r.index == 0);
    CHECK(r.from > 0.0f);
    CHECK(r.to > r.from);
    for (int k = 0; k <= 20; ++k) { // (the section it names turns tighter than it said)
        const double t = static_cast<double>(r.from) + static_cast<double>(r.to - r.from) * k / 20.0;
        CHECK(std::abs(route::rational(good, t).curvature()) > tight.curvature);
    }
    tight.curvature = 1.02 * most; // (said as it is: accepted)
    CHECK(w.submit(v, CurveCommand{}, Span<const NurbsSegment>(&tight, 1)).accepted());
}

TEST_CASE("nurbs: a wing and a multirotor fly a rational cubic S and a circle as rational quadratics, read back as given", "[modes]") {
    struct Aircraft {
        const char* type;
        bool rotor;
        double scale; ///< the curves' size, m: a wing's its turn radius
        double limit; ///< off the curve at most, m
    };
    for (const Aircraft& a : {Aircraft{"c172x", false, 0.0, 12.0}, Aircraft{"iris", true, 20.0, 0.5}}) {
        INFO(a.type);
        session::World w(options("nurbs-flights"));
        const auto v = a.rotor ? rotor(w, a.type, 20.0) : wing(w, a.type, 1500.0, 55.0);
        if (!a.rotor) {
            HsaCommand north;
            north.headingRad = 0.0, north.speed = 55.0, north.speedReference = 0.0, north.altitudeM = 1500.0;
            REQUIRE(w.submit(v, north).accepted());
            w.step(stepsFor(w, 15.0));
        }
        const sim::VehicleState s0 = *w.vehicleState(v);
        const double R = a.rotor ? a.scale : w.performance(v)->turnRadiusM(55.0);
        // along its heading (north): a lead, the S, a semicircle round to the right, and straight on south
        const double psi = s0.eulerRad[2];
        std::vector<NurbsSegment> segments = {cubicS(0.0, 0.0, 12.0 * R, 0.8 * R, a.rotor ? 3.0 : 60.0)}; // (its tightest about 2 R)
        const NurbsSegment& first = segments[0];
        const double n1 = first.north[first.points - 1], e1 = first.east[first.points - 1], d1 = first.down[first.points - 1];
        segments.push_back(semicircle(n1, e1 + 1.6 * R, 1.6 * R, d1));
        const NurbsSegment& arc = segments[1];
        const double n2 = arc.north[arc.points - 1], e2 = arc.east[arc.points - 1];
        segments.push_back(clamped(3, {{n2, e2, d1}, {n2 - R, e2, d1}, {n2 - 2 * R, e2, d1}, {n2 - 3 * R, e2, d1}}, {1.0, 1.0, 1.0, 1.0}, {}));
        for (auto& seg : segments) // (turned to its heading)
            for (std::uint32_t i = 0; i < seg.points; ++i) {
                const double n = seg.north[i], e = seg.east[i];
                seg.north[i] = n * std::cos(psi) - e * std::sin(psi), seg.east[i] = n * std::sin(psi) + e * std::cos(psi);
            }
        CurveCommand c;
        if (a.rotor) c.speedMinMs = c.speedMaxMs = 4.0;
        const CommandResult r = w.submit(v, c, Span<const NurbsSegment>(segments));
        INFO(reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        Setpoint flown;
        REQUIRE(w.activitySetpoint(r.activity, flown));
        CHECK(flown.nurbs.size() == 3);
        CHECK(flown.segments.empty()); // (not Bezier segments)
        CHECK(flown.nurbs[1].points == 5);
        CHECK(flown.nurbs[1].weight[1] == std::sqrt(0.5));
        const Line line(segments);
        std::size_t hint = 0;
        double off = 0.0, arcOff = 0.0;
        bool completed = false;
        std::uint32_t lastSegment = 0;
        const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad;
        for (unsigned k = 0; k < stepsFor(w, (12.0 + kPi * 1.6 + 3.0) * R / (a.rotor ? 4.0 : 55.0) * 1.5 + 60.0); ++k) {
            w.step();
            const auto p = from(*w.vehicleState(v), lat0, lon0);
            const ActivityRecord& rec = *w.activity(r.activity);
            if (!rec.live()) {
                completed = rec.state == ActivityState::Completed;
                break;
            }
            lastSegment = rec.progress.segment;
            const double d = line.off(p[0], p[1], hint);
            if (hint > 400) off = std::max(off, d); // (past the S's first fifth: joined)
            if (rec.progress.segment == 1) { // round the semicircle: off its radius
                const double cn = n1 * std::cos(psi) - (e1 + 1.6 * R) * std::sin(psi), ce = n1 * std::sin(psi) + (e1 + 1.6 * R) * std::cos(psi);
                arcOff = std::max(arcOff, std::abs(std::hypot(p[0] - cn, p[1] - ce) - 1.6 * R));
            }
        }
        std::printf("nurbs flight %-6s: R %.0f m; off the curve %.2f m, off the semicircle's radius %.2f m; completed %d (last segment %u)\n", a.type, R, off,
                    arcOff, completed ? 1 : 0, lastSegment);
        CHECK(completed);
        CHECK(off < a.limit);
        CHECK(arcOff < a.limit);
    }
}
