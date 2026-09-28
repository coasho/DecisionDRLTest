// The Vehicle Interface's curve following, flown (docs/vehicle-interface.md,
// 4.7): an S of quintic Beziers, climbing, on a stock wing, a direct and a
// fly-by-wire design, a helicopter and a multirotor, calm and in a
// crosswind - measured against the curve evaluated here by de Casteljau's
// construction, apart from the platform's Bernstein sums and Newton steps -
// and what a mission autonomy relies on: a duration met, a speed range,
// segments appended while flying, a new curve, the ends, progress, and the
// section a rejection names.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

/// Where the aircraft is from (lat0, lon0): north and east, m (the plane there, the curve's frame).
struct Local {
    double north, east;
};
Local from(const sim::VehicleState& s, double lat0, double lon0) {
    return {(s.latitudeRad - lat0) * kR, (s.longitudeRad - lon0) * kR * std::cos(0.5 * (s.latitudeRad + lat0))};
}

/// A segment's point by de Casteljau's construction: north, east, down.
std::array<double, 3> casteljau(const BezierSegment& b, double t) {
    std::array<double, 3> out{};
    const double* axes[3] = {b.north, b.east, b.down};
    for (int a = 0; a < 3; ++a) {
        double q[6];
        std::copy_n(axes[a], 6, q);
        for (int r = 5; r > 0; --r)
            for (int i = 0; i < r; ++i) q[i] = (1.0 - t) * q[i] + t * q[i + 1];
        out[static_cast<std::size_t>(a)] = q[0];
    }
    return out;
}

/// A quintic segment from a curve's position and its first and second
/// derivatives by the parameter at its two ends (Hermite's).
BezierSegment hermite(const double p0[3], const double d0[3], const double a0[3], const double p1[3], const double d1[3], const double a1[3]) {
    BezierSegment b;
    double* axes[3] = {b.north, b.east, b.down};
    for (int k = 0; k < 3; ++k) {
        double* q = axes[k];
        q[0] = p0[k];
        q[1] = p0[k] + d0[k] / 5.0;
        q[2] = p0[k] + 2.0 * d0[k] / 5.0 + a0[k] / 20.0;
        q[3] = p1[k] - 2.0 * d1[k] / 5.0 + a1[k] / 20.0;
        q[4] = p1[k] - d1[k] / 5.0;
        q[5] = p1[k];
    }
    return b;
}

/// The S every class flies, from where the aircraft is along `course0`:
/// `lead` metres straight, then a period of `period` metres over which the
/// curvature runs kappa sin(2 pi u) - a turn right and one left, each turning
/// the course kappa period / pi - climbing `climb` metres as (1 - cos pi u) / 2,
/// and `tail` metres straight on. Its segments are the lead, the period's four
/// quarters and the tail.
struct Design {
    double lead, period, tail, kappa, climb;
    double course0 = 0.5 * kPi;

    double length() const { return lead + period + tail; }
    double course(double s) const {
        if (s <= lead) return course0;
        const double u = std::min((s - lead) / period, 1.0);
        return course0 + kappa * period / (2.0 * kPi) * (1.0 - std::cos(2.0 * kPi * u));
    }
    double curvature(double s) const { return s <= lead || s >= lead + period ? 0.0 : kappa * std::sin(2.0 * kPi * (s - lead) / period); }
    /// Down at s, and its first and second derivatives by s.
    void down(double s, double& d, double& d1, double& d2) const {
        if (s <= lead) {
            d = d1 = d2 = 0.0;
        } else if (s >= lead + period) {
            d = -climb, d1 = d2 = 0.0;
        } else {
            const double u = (s - lead) / period;
            d = -0.5 * climb * (1.0 - std::cos(kPi * u));
            d1 = -0.5 * climb * kPi / period * std::sin(kPi * u);
            d2 = -0.5 * climb * kPi * kPi / (period * period) * std::cos(kPi * u);
        }
    }
    /// North and east at s: the course integrated by Simpson's rule.
    void position(double s, double& north, double& east) const {
        constexpr int n = 4000;
        const double h = s / n;
        north = east = 0.0;
        for (int i = 0; i <= n; ++i) {
            const double w = (i == 0 || i == n) ? 1.0 : (i % 2 ? 4.0 : 2.0);
            const double c = course(i * h);
            north += w * std::cos(c), east += w * std::sin(c);
        }
        north *= h / 3.0, east *= h / 3.0;
    }
    BezierSegment piece(double s0, double s1) const {
        const double l = s1 - s0;
        double p[2][3], d[2][3], a[2][3];
        for (int e = 0; e < 2; ++e) {
            const double s = e == 0 ? s0 : s1, c = course(s), k = curvature(s);
            double dn, d1, d2;
            down(s, dn, d1, d2);
            position(s, p[e][0], p[e][1]);
            p[e][2] = dn;
            d[e][0] = l * std::cos(c), d[e][1] = l * std::sin(c), d[e][2] = l * d1;
            a[e][0] = -l * l * k * std::sin(c), a[e][1] = l * l * k * std::cos(c), a[e][2] = l * l * d2;
        }
        return hermite(p[0], d[0], a[0], p[1], d[1], a[1]);
    }
    std::vector<BezierSegment> segments() const {
        const double knots[] = {0.0, lead, lead + 0.25 * period, lead + 0.5 * period, lead + 0.75 * period, lead + period, length()};
        std::vector<BezierSegment> out;
        for (std::size_t i = 0; i + 1 < std::size(knots); ++i) out.push_back(piece(knots[i], knots[i + 1]));
        return out;
    }
};

/// An S about `radius`: its tightest a quarter wider, each turn 100 degrees.
Design sAbout(double radius, double climb) {
    const double kappa = 1.0 / (1.25 * radius);
    return Design{2.0 * radius, 100.0 * kDeg * kPi / kappa, 1.5 * radius, kappa, climb};
}

/// A curve as a dense polyline, to measure against.
class Polyline {
public:
    explicit Polyline(const std::vector<BezierSegment>& segments, int perSegment = 400) {
        for (const auto& s : segments)
            for (int k = points_.empty() ? 0 : 1; k <= perSegment; ++k) points_.push_back(casteljau(s, static_cast<double>(k) / perSegment));
        along_.assign(points_.size(), 0.0);
        for (std::size_t i = 1; i < points_.size(); ++i)
            along_[i] = along_[i - 1] + std::hypot(points_[i][0] - points_[i - 1][0], points_[i][1] - points_[i - 1][1]);
    }
    double lengthM() const { return along_.empty() ? 0.0 : along_[along_.size() - 1]; }

    struct Near {
        double distanceM = 1e18, down = 0.0, alongM = 0.0;
    };
    /// The nearest point over the ground, searched about the last (`hint`, moved on).
    Near nearest(Local p, std::size_t& hint) const {
        const std::size_t lo = hint > 200 ? hint - 200 : 0, hi = std::min(points_.size() - 1, hint + 800);
        Near best;
        std::size_t at = hint;
        for (std::size_t j = lo; j < hi; ++j) {
            const auto& a = points_[j];
            const auto& b = points_[j + 1];
            const double bn = b[0] - a[0], be = b[1] - a[1], span = bn * bn + be * be;
            const double u = span > 0.0 ? std::clamp(((p.north - a[0]) * bn + (p.east - a[1]) * be) / span, 0.0, 1.0) : 0.0;
            const double d = std::hypot(a[0] + u * bn - p.north, a[1] + u * be - p.east);
            if (d < best.distanceM) best = {d, a[2] + u * (b[2] - a[2]), along_[j] + u * (along_[j + 1] - along_[j])}, at = j;
        }
        hint = at;
        return best;
    }

private:
    std::vector<std::array<double, 3>> points_;
    std::vector<double> along_;
};

/// What a flight along a curve did.
struct Flight {
    ActivityId activity = 0;
    double seconds = -1.0;                ///< to completion (-1: it did not complete)
    double off = 0.0, offHeight = 0.0;    ///< the most off the curve over the ground and in height, past `settledM` along it
    double ownCross = 0.0;                ///< the most cross-track the curve reported there
    double slowest = 1e9, fastest = 0.0;  ///< the ground speed there
    std::vector<std::uint32_t> segments;  ///< the segments progress went through, in order
    bool monotonic = true;                ///< its percent never went back
};

/// Fly the curve from the vehicle's position until it completes or `limitS` passes.
Flight fly(session::World& w, std::uint32_t v, const CurveCommand& c, const std::vector<BezierSegment>& segments, double settledM, double limitS) {
    const sim::VehicleState s0 = *w.vehicleState(v);
    const double lat0 = isHold(c.latitudeRad) ? s0.latitudeRad : c.latitudeRad, lon0 = isHold(c.longitudeRad) ? s0.longitudeRad : c.longitudeRad;
    const double alt0 = isHold(c.altitudeM) ? s0.altitudeMslM : c.altitudeM;
    const CommandResult r = w.submit(v, c, segments);
    INFO("the curve: " << reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    const Polyline line(segments);
    Flight f;
    f.activity = r.activity;
    std::size_t hint = 0;
    double percent = 0.0;
    const double t0 = w.simTime();
    for (unsigned k = 0; k < stepsFor(w, limitS); ++k) {
        w.step();
        const auto& s = *w.vehicleState(v);
        const ActivityRecord& a = *w.activity(r.activity);
        const Polyline::Near n = line.nearest(from(s, lat0, lon0), hint);
        if (a.live() && n.alongM > settledM) {
            f.off = std::max(f.off, n.distanceM);
            f.offHeight = std::max(f.offHeight, std::abs(s.altitudeMslM - (alt0 - n.down)));
            f.ownCross = std::max(f.ownCross, std::abs(a.progress.crossTrackM));
            f.slowest = std::min(f.slowest, groundSpeed(s)), f.fastest = std::max(f.fastest, groundSpeed(s));
        }
        if (f.segments.empty() || f.segments.back() != a.progress.segment) f.segments.push_back(a.progress.segment);
        if (a.progress.percent < percent - 0.2) f.monotonic = false;
        percent = std::max(percent, a.progress.percent);
        if (a.state == ActivityState::Completed) {
            f.seconds = w.simTime() - t0;
            break;
        }
    }
    return f;
}

/// Wings flying on east at their speed and height for `seconds` (an hsa each): their guidance has seen the wind.
void settle(session::World& w, std::initializer_list<std::uint32_t> wings, double seconds) {
    for (const auto v : wings) {
        const auto& s = *w.vehicleState(v);
        HsaCommand east;
        east.headingRad = 0.5 * kPi, east.speed = s.airspeedTrueMs, east.speedReference = 0.0, east.altitudeM = s.altitudeMslM;
        REQUIRE(w.submit(v, east).accepted());
    }
    w.step(stepsFor(w, seconds));
}

/// A straight segment from (n0, e0, d0) to (n1, e1, d1): its control points evenly along it.
BezierSegment straight(double n0, double e0, double d0, double n1, double e1, double d1) {
    BezierSegment b;
    for (int k = 0; k < 6; ++k) {
        const double u = k / 5.0;
        b.north[k] = n0 + u * (n1 - n0), b.east[k] = e0 + u * (e1 - e0), b.down[k] = d0 + u * (d1 - d0);
    }
    return b;
}

double code(EndBehavior e) { return static_cast<double>(e); }

} // namespace

TEST_CASE("curve: every class flies an S of quintic Beziers, climbing, calm and in a crosswind", "[modes]") {
    struct Aircraft {
        const char* type;
        bool rotor;
        double altitudeM, speedMs, windMs, radiusM, climbM; ///< (a wing's radius its own, at its speed and the wind)
        double calm, windy;                                 ///< off the curve along the S, calm and in the wind, m
    };
    // a 12 m/s crosswind at its start; the multirotor's 5 m/s (it cruises at 4)
    for (const Aircraft& a : {Aircraft{"c172x", false, 1500.0, 55.0, 12.0, 0.0, 100.0, 8.0, 12.0}, Aircraft{"b52h", false, 3000.0, 180.0, 12.0, 0.0, 100.0, 8.0, 8.0},
                             Aircraft{"f16c", false, 3000.0, 160.0, 12.0, 0.0, 100.0, 9.0, 16.0}, Aircraft{"uh60", true, 100.0, 20.0, 12.0, 150.0, 20.0, 12.0, 18.0},
                             Aircraft{"iris", true, 100.0, 4.0, 5.0, 20.0, 3.0, 0.3, 0.5}}) {
        for (const double wind : {0.0, a.windMs}) {
            INFO(a.type << ", wind " << wind);
            session::World w(options("curves-classes"));
            if (wind > 0.0) setWind(w, 0.0, wind); // from the north: across its start
            const auto v = a.rotor ? rotor(w, a.type, 20.0) : wing(w, a.type, a.altitudeM, a.speedMs);
            if (!a.rotor) settle(w, {v}, 5.0);
            const double radius = a.rotor ? a.radiusM : w.performance(v)->turnRadiusM(a.speedMs + a.windMs);
            const Design s = sAbout(radius, a.climbM);
            CurveCommand c;
            if (a.rotor) c.speedMinMs = c.speedMaxMs = a.speedMs; // (a wing: its airspeed as now)
            const Flight f = fly(w, v, c, s.segments(), s.lead, 3.0 * s.length() / a.speedMs + 120.0); // (measured along the S, past its lead)
            std::printf("curve %-6s wind %4.1f: %6.0f s (%6.0f at its speed), off %6.1f m (R %6.0f), height %5.1f m, own %6.1f, speed %5.1f to %5.1f\n", a.type,
                        wind, f.seconds, s.length() / a.speedMs, f.off, radius, f.offHeight, f.ownCross, f.slowest, f.fastest);
            CHECK(f.seconds > 0.0); // completed at its end
            CHECK(f.segments == std::vector<std::uint32_t>{0, 1, 2, 3, 4, 5});
            CHECK(f.monotonic);
            CHECK(f.off < (wind > 0.0 ? a.windy : a.calm));
            CHECK(std::abs(f.ownCross - f.off) < 0.2 * f.off + 2.0); // what it reports is what it flies
            CHECK(f.offHeight < (a.rotor ? 3.0 : 20.0));
            if (a.rotor) CHECK(f.slowest > 0.9 * a.speedMs); // (the S no tighter than its speed allows)
            const ActivityRecord& done = *w.activity(f.activity);
            CHECK(done.reason == Reason::GoalReached);
            CHECK(done.progress.percent == 100.0);
            CHECK(done.progress.distanceToGoM == 0.0);
        }
    }
}

TEST_CASE("curve: a duration is met; a range holds a wing's airspeed and a rotorcraft's ground speed", "[modes]") {
    session::World w(options("curves-pace"));
    setWind(w, 0.0, 10.0);
    const auto timed = wing(w, "c172x", 1500.0, 55.0, 0), ranged = wing(w, "c172x", 1500.0, 55.0, 3), slowed = wing(w, "c172x", 1500.0, 55.0, 6);
    settle(w, {timed, ranged, slowed}, 5.0);
    const double radius = w.performance(timed)->turnRadiusM(65.0);
    const Design s = sAbout(radius, 0.0);
    const auto segments = s.segments();
    const double length = Polyline(segments).lengthM();
    // the whole S in the time a 50 m/s ground speed takes, in a 10 m/s wind
    CurveCommand c;
    c.durationS = length / 50.0;
    const CommandResult a = w.submit(timed, c, segments);
    REQUIRE(a.accepted());
    // a range the ground speed it makes holding its airspeed stays within; and one it must slow for
    CurveCommand wide;
    wide.speedMinMs = 40.0, wide.speedMaxMs = 70.0;
    const CommandResult b = w.submit(ranged, wide, segments);
    REQUIRE(b.accepted());
    CurveCommand slow;
    slow.speedMaxMs = 45.0;
    const CommandResult d = w.submit(slowed, slow, segments);
    REQUIRE(d.accepted());
    const double tas0 = w.vehicleState(ranged)->airspeedTrueMs;
    double tasLow = 1e9, tasHigh = 0.0, slowestGs = 1e9, fastestGs = 0.0;
    double doneA = -1.0;
    const double t0 = w.simTime();
    for (unsigned k = 0; k < stepsFor(w, 3.0 * c.durationS); ++k) {
        w.step();
        if (doneA < 0.0 && !w.activity(a.activity)->live()) doneA = w.simTime() - t0;
        if (w.simTime() - t0 > 30.0 && w.activity(b.activity)->live()) {
            const auto& r = *w.vehicleState(ranged);
            tasLow = std::min(tasLow, r.airspeedTrueMs), tasHigh = std::max(tasHigh, r.airspeedTrueMs);
        }
        if (w.simTime() - t0 > 60.0 && w.activity(d.activity)->live()) {
            const double g = groundSpeed(*w.vehicleState(slowed));
            slowestGs = std::min(slowestGs, g), fastestGs = std::max(fastestGs, g);
        }
    }
    std::printf("curve pace: %.0f s for %.0f (%.1f %%), airspeed %.1f to %.1f (from %.1f), slowed %.1f to %.1f over the ground\n", doneA, c.durationS,
                100.0 * (doneA - c.durationS) / c.durationS, tasLow, tasHigh, tas0, slowestGs, fastestGs);
    CHECK(w.activity(a.activity)->state == ActivityState::Completed);
    CHECK(std::abs(doneA - c.durationS) < 0.05 * c.durationS);
    CHECK(tasHigh - tasLow < 3.0); // its airspeed held while its ground speed swings with the wind
    CHECK(std::abs(0.5 * (tasLow + tasHigh) - tas0) < 2.0);
    CHECK(fastestGs < 47.0); // never faster over the ground than its range
    CHECK(slowestGs > 35.0);
    CHECK(w.activity(d.activity)->progress.speedReference == static_cast<double>(SpeedReference::TrueAirspeed));

    // a multirotor: a duration over its short curve, stopping at its end, and a ground speed within its range
    session::World calm(options("curves-pace-rotor"));
    const auto quad = rotor(calm, "iris", 15.0, 0), other = rotor(calm, "iris", 15.0, 3);
    const Design q = sAbout(10.0, 2.0);
    const auto qs = q.segments();
    CurveCommand qc;
    qc.durationS = Polyline(qs).lengthM() / 3.0;
    qc.end = code(EndBehavior::Loiter); // (braking to its end on time, not slowing with what is left to nothing short of it)
    const CommandResult e = calm.submit(quad, qc, qs);
    REQUIRE(e.accepted());
    CurveCommand qr;
    qr.speedMinMs = 2.0, qr.speedMaxMs = 2.5;
    const CommandResult g = calm.submit(other, qr, qs);
    REQUIRE(g.accepted());
    double doneE = -1.0, fastest = 0.0;
    const double t1 = calm.simTime();
    for (unsigned k = 0; k < stepsFor(calm, 3.0 * qc.durationS) && (calm.activity(e.activity)->live() || calm.activity(g.activity)->live()); ++k) {
        calm.step();
        if (doneE < 0.0 && !calm.activity(e.activity)->live()) doneE = calm.simTime() - t1;
        if (calm.simTime() - t1 > 5.0 && calm.activity(g.activity)->live()) fastest = std::max(fastest, groundSpeed(*calm.vehicleState(other)));
    }
    std::printf("curve pace, multirotor: %.1f s for %.1f (%.1f %%), ranged at most %.2f m/s\n", doneE, qc.durationS, 100.0 * (doneE - qc.durationS) / qc.durationS,
                fastest);
    CHECK(std::abs(doneE - qc.durationS) < 0.05 * qc.durationS);
    CHECK(fastest < 2.6);
    CHECK(fastest > 2.3);
    CHECK(calm.activity(g.activity)->progress.speedReference == static_cast<double>(SpeedReference::GroundSpeed));
}

TEST_CASE("curve: segments appended while flying are flown on to; a new curve is flown afresh", "[modes]") {
    session::World w(options("curves-append"));
    const auto v = wing(w, "c172x", 1500.0, 55.0);
    settle(w, {v}, 5.0);
    const double radius = w.performance(v)->turnRadiusM(55.0);
    const Design s = sAbout(radius, 60.0);
    const auto all = s.segments();
    // the lead and the first quarter, then the rest appended while it flies them
    const std::vector<BezierSegment> first(all.begin(), all.begin() + 2), rest(all.begin() + 2, all.end());
    const auto& s0 = *w.vehicleState(v);
    const double lat0 = s0.latitudeRad, lon0 = s0.longitudeRad, alt0 = s0.altitudeMslM;
    const CommandResult r = w.submit(v, CurveCommand{}, first);
    REQUIRE(r.accepted());
    w.step(stepsFor(w, 20.0));
    CHECK(w.activity(r.activity)->progress.segments == 2);
    const double percentBefore = w.activity(r.activity)->progress.percent;
    const std::uint32_t segmentBefore = w.activity(r.activity)->progress.segment;
    // a gap where it would join is refused, naming the segment; so is appending nothing
    CurveCommand append;
    append.append = 1.0;
    std::vector<BezierSegment> gapped = rest;
    gapped.at(0).north[0] += 5.0;
    CommandResult refused = w.update(r.activity, append, gapped);
    CHECK(refused.reason == Reason::InvalidCurve);
    CHECK(refused.index == 0);
    refused = w.update(r.activity, append, Span<const BezierSegment>{});
    CHECK(refused.reason == Reason::InvalidCurve);
    CHECK(refused.index == 0);
    const CommandResult appended = w.update(r.activity, append, rest);
    REQUIRE(appended.accepted());
    CHECK(appended.activity == r.activity);
    w.step();
    const ActivityProgress& p = w.activity(r.activity)->progress;
    CHECK(p.segments == 6);
    CHECK(p.segment == segmentBefore); // on where it was
    CHECK(p.percent < percentBefore); // of a longer curve
    // flown on to its new end, on the curve all the way
    const Polyline line(all);
    std::size_t hint = 0;
    double off = 0.0, offHeight = 0.0;
    std::uint32_t segment = 0;
    bool inOrder = true;
    for (unsigned k = 0; k < stepsFor(w, 3.0 * s.length() / 55.0) && w.activity(r.activity)->live(); ++k) {
        w.step();
        const auto& st = *w.vehicleState(v);
        const Polyline::Near n = line.nearest(from(st, lat0, lon0), hint);
        off = std::max(off, n.distanceM);
        offHeight = std::max(offHeight, std::abs(st.altitudeMslM - (alt0 - n.down)));
        if (w.activity(r.activity)->progress.segment < segment) inOrder = false;
        segment = w.activity(r.activity)->progress.segment;
    }
    std::printf("curve append: off %.1f m, height %.1f m, to segment %u\n", off, offHeight, segment);
    CHECK(w.activity(r.activity)->state == ActivityState::Completed);
    CHECK(inOrder);
    CHECK(segment == 5);
    CHECK(off < 20.0); // (from its start)
    CHECK(offHeight < 10.0);

    // an UPDATE with segments and no append: a new curve, from the same reference, flown afresh
    const auto& s1 = *w.vehicleState(v);
    const Local here = from(s1, lat0, lon0);
    (void)here;
    const std::vector<BezierSegment> east = {straight(0.0, 0.0, 0.0, 0.0, 3000.0, 0.0)};
    const CommandResult again = w.submit(v, CurveCommand{}, east); // (a NEW: its reference the aircraft)
    REQUIRE(again.accepted());
    w.step(stepsFor(w, 10.0));
    const std::vector<BezierSegment> north = {straight(0.0, 0.0, 0.0, 1500.0, 1500.0, 0.0), straight(1500.0, 1500.0, 0.0, 4000.0, 1500.0, 0.0)};
    const CommandResult replaced = w.update(again.activity, CurveCommand{}, north);
    REQUIRE(replaced.accepted());
    w.step();
    CHECK(w.activity(again.activity)->progress.segments == 2);
    CHECK(w.activity(again.activity)->progress.segment == 0);
    // its options alone: how it is flown, its segments kept
    CurveCommand faster;
    faster.speedMinMs = 58.0;
    REQUIRE(w.update(again.activity, faster).accepted());
    w.step(stepsFor(w, 20.0));
    CHECK(w.activity(again.activity)->progress.segments == 2);
    CHECK(w.activity(again.activity)->progress.speedMs > 57.0);
}

TEST_CASE("curve: at its end a curve continues, orbits its end or hovers there", "[modes]") {
    session::World w(options("curves-ends"));
    const auto goer = wing(w, "c172x", 1500.0, 55.0, 0), orbiter = wing(w, "c172x", 1500.0, 55.0, 3);
    settle(w, {goer, orbiter}, 5.0);
    const double radius = w.performance(goer)->turnRadiusM(55.0);
    const auto wingS = sAbout(radius, 0.0).segments();
    const CommandResult go = w.submit(goer, CurveCommand{}, wingS);
    CurveCommand loiter;
    loiter.end = code(EndBehavior::Loiter);
    const auto& o = *w.vehicleState(orbiter);
    const double oLat = o.latitudeRad, oLon = o.longitudeRad;
    const CommandResult orbit = w.submit(orbiter, loiter, wingS);
    const auto hoverer = rotor(w, "iris", 15.0, 6), dasher = rotor(w, "uh60", 15.0, 9);
    const auto& h = *w.vehicleState(hoverer);
    const double hLat = h.latitudeRad, hLon = h.longitudeRad;
    CurveCommand stop = loiter;
    stop.speedMinMs = stop.speedMaxMs = 4.0;
    const auto rotorS = sAbout(10.0, 0.0).segments();
    const CommandResult hover = w.submit(hoverer, stop, rotorS);
    CurveCommand dash;
    dash.speedMinMs = dash.speedMaxMs = 15.0;
    const CommandResult on = w.submit(dasher, dash, sAbout(150.0, 0.0).segments());
    REQUIRE((go.accepted() && orbit.accepted() && hover.accepted() && on.accepted()));

    const auto endOf = [](const std::vector<BezierSegment>& s) { return casteljau(s.back(), 1.0); };
    const auto wingEnd = endOf(wingS), rotorEnd = endOf(rotorS);
    double orbitMin = 1e9, orbitMax = 0.0, done = -1.0, courseOff = 0.0;
    for (unsigned k = 0; k < stepsFor(w, 700.0); ++k) {
        w.step();
        if (!w.activity(orbit.activity)->live() && done < 0.0) done = w.simTime();
        if (done > 0.0 && w.simTime() > done + 120.0) { // settled in its orbit
            const Local p = from(*w.vehicleState(orbiter), oLat, oLon);
            const double d = std::hypot(p.north - wingEnd[0], p.east - wingEnd[1]);
            orbitMin = std::min(orbitMin, d), orbitMax = std::max(orbitMax, d);
        }
        if (!w.activity(go.activity)->live()) courseOff = std::max(courseOff, std::abs(degreesApart(track(*w.vehicleState(goer)), 0.5 * kPi)));
    }
    const Local hp = from(*w.vehicleState(hoverer), hLat, hLon);
    const double hoverOff = std::hypot(hp.north - rotorEnd[0], hp.east - rotorEnd[1]);
    const double orbitR = w.performance(orbiter)->turnRadiusM(55.0);
    std::printf("curve ends: on at most %.1f deg off its last course; orbit %.0f to %.0f m (R %.0f); iris %.2f m from its end at %.2f m/s; uh60 on at %.1f deg, "
                "%.1f m/s\n",
                courseOff, orbitMin, orbitMax, orbitR, hoverOff, groundSpeed(*w.vehicleState(hoverer)), track(*w.vehicleState(dasher)) / kDeg,
                groundSpeed(*w.vehicleState(dasher)));
    for (const ActivityId id : {go.activity, orbit.activity, hover.activity, on.activity}) {
        CHECK(w.activity(id)->state == ActivityState::Completed);
        CHECK(w.activity(id)->reason == Reason::GoalReached);
    }
    CHECK(courseOff < 3.0); // on along its last course, east
    CHECK(std::abs(w.vehicleState(goer)->airspeedTrueMs - 55.0) < 2.0);
    CHECK(orbitMin > 0.8 * orbitR);
    CHECK(orbitMax < 1.25 * orbitR);
    CHECK(std::abs(w.activity(orbit.activity)->progress.crossTrackM) < 20.0); // at its end, off the curve: not the orbit's radius
    CHECK(hoverOff < 1.0);
    CHECK(groundSpeed(*w.vehicleState(hoverer)) < 0.3);
    CHECK(std::abs(degreesApart(track(*w.vehicleState(dasher)), 0.5 * kPi)) < 3.0);
    CHECK(std::abs(groundSpeed(*w.vehicleState(dasher)) - 15.0) < 1.0); // at the speed it flew the curve
}

TEST_CASE("curve: what cannot be flown is refused, naming the segment and the section; what can be made to fit is clamped", "[modes]") {
    session::World w(options("curves-checks"));
    const auto c172 = wing(w, "c172x", 1500.0, 55.0, 0), f16 = wing(w, "f16c", 3000.0, 160.0, 2), b52 = wing(w, "b52h", 3000.0, 180.0, 5);
    const auto iris = rotor(w, "iris", 0.0, 4);
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    auto refuse = [&](std::uint32_t v, const std::vector<BezierSegment>& segments, const CurveCommand& c, Reason reason, int index,
                      const CommandOptions& o = {}) {
        const CommandResult r = w.submit(v, c, segments, o);
        INFO(reasonName(r.reason) << " at " << r.index << ", " << constraintName(r.constraint));
        CHECK(r.reason == reason);
        CHECK(r.index == index);
        return r;
    };
    const double radius = w.performance(c172)->turnRadiusM(55.0);
    const auto good = sAbout(radius, 0.0).segments();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // the segments themselves: 1 to 10, finite, each starting where the one before ends
    refuse(c172, {}, CurveCommand{}, Reason::InvalidCurve, 0);
    CHECK(w.submit(c172, Command(CurveCommand{})).reason == Reason::InvalidCurve); // a curve's command alone has none
    refuse(c172, std::vector<BezierSegment>(11, good[0]), CurveCommand{}, Reason::InvalidCurve, 10);
    auto with = [&](std::size_t i, auto change) {
        std::vector<BezierSegment> s = good;
        change(s.at(i));
        return s;
    };
    refuse(c172, with(3, [&](BezierSegment& b) { b.east[2] = nan; }), CurveCommand{}, Reason::InvalidCurve, 3);
    refuse(c172, with(2, [](BezierSegment& b) { b.down[0] -= 2.0; }), CurveCommand{}, Reason::InvalidCurve, 2); // 2 m from where 1 ends
    CHECK(w.submit(c172, CurveCommand{}, with(2, [](BezierSegment& b) { b.down[0] -= 0.5; })).accepted());      // (within a metre)
    // its options
    CurveCommand c;
    c.latitudeRad = 0.5; // a reference needs both
    refuse(c172, good, c, Reason::InvalidParameter, 1);
    c = CurveCommand{}, c.speedMinMs = 50.0, c.speedMaxMs = 40.0;
    refuse(c172, good, c, Reason::InvalidParameter, 4);
    c = CurveCommand{}, c.durationS = 0.0;
    refuse(c172, good, c, Reason::InvalidParameter, 5);
    c = CurveCommand{}, c.end = 2.0;
    refuse(c172, good, c, Reason::InvalidParameter, 6);
    c = CurveCommand{}, c.append = 1.0; // a NEW has nothing to append to
    refuse(c172, good, c, Reason::InvalidParameter, 7);
    // a section tighter than the wing turns at its full bank at its speed: the segment and where on it, whatever the policy
    const Design tight = sAbout(0.3 * radius, 0.0);
    const auto tooTight = tight.segments();
    for (const CommandOptions& o : {CommandOptions{}, reject}) {
        const CommandResult r = refuse(c172, tooTight, CurveCommand{}, Reason::InvalidCurve, 1, o);
        CHECK(r.constraint == Constraint::MaxTurnRate);
        // the curvature rises as sin(pi t / 2) over the first quarter's segment: over the limit from where that passes it, to its end
        const Performance& f = *w.performance(c172);
        const double fastest = w.vehicleState(c172)->airspeedTrueMs;
        const double limit = 9.80665 * std::tan(f.maxBankRad) / (fastest * fastest);
        const double predicted = 2.0 / kPi * std::asin(limit / tight.kappa);
        std::printf("curve too tight: section %.3f to %.3f of segment %d (predicted from %.3f)\n", static_cast<double>(r.from), static_cast<double>(r.to), r.index,
                    predicted);
        CHECK(std::abs(static_cast<double>(r.from) - predicted) < 0.05);
        CHECK(r.to == 1.0f);
    }
    CHECK(w.submit(iris, CurveCommand{}, sAbout(3.0, 0.0).segments(), reject).accepted()); // a rotorcraft slows for it
    // a climb steeper than it climbs at its speed: flown at its rate (clamped, and said), or refused
    const auto steep = sAbout(radius, 1500.0).segments(); // (steep already in the first quarter)
    const CommandResult r = refuse(c172, steep, CurveCommand{}, Reason::PerformanceLimit, 1, reject);
    CHECK(r.constraint == Constraint::MaxClimbRate);
    const CommandResult clamped = w.submit(c172, CurveCommand{}, steep);
    REQUIRE(clamped.accepted());
    CHECK((clamped.flags & kClamped) != 0);
    CHECK(clamped.index == 1);
    // a range it cannot fly within, a duration it cannot keep: past the F-16C's envelope, and under
    // the B-52H's stall speed (the c172x's profile knows neither end); a range it can fly within is no fault
    const std::vector<BezierSegment> dash = {straight(0, 0, 0, 0, 20000, 0)};
    c = CurveCommand{}, c.speedMinMs = 900.0, c.speedMaxMs = 1000.0;
    CHECK(refuse(f16, dash, c, Reason::PerformanceLimit, 3, reject).constraint == Constraint::MaxAirspeed);
    const CommandResult fastest = w.submit(f16, c, dash); // (or its least clamped to its fastest)
    CHECK(fastest.accepted());
    CHECK((fastest.flags & kClamped) != 0);
    c = CurveCommand{}, c.durationS = 10.0;
    CHECK(refuse(f16, dash, c, Reason::PerformanceLimit, 5, reject).constraint == Constraint::MaxAirspeed);
    c = CurveCommand{}, c.speedMaxMs = 30.0;
    CHECK(refuse(b52, dash, c, Reason::PerformanceLimit, 4, reject).constraint == Constraint::MinAirspeed);
    c = CurveCommand{}, c.speedMinMs = 5.0, c.speedMaxMs = 1000.0;
    CHECK(w.submit(b52, c, dash, reject).accepted());
    // a segment with no length over the ground (straight up) is nothing to follow
    refuse(c172, {straight(0, 0, 0, 0, 0, -50)}, CurveCommand{}, Reason::InvalidCurve, 0);
    // an UPDATE: past the room the store has, appended segments are refused
    auto run = [](int from, int count) { // `count` segments of 500 m on east from `from`'s end
        std::vector<BezierSegment> out;
        for (int k = from; k < from + count; ++k) out.push_back(straight(0, 500.0 * k, 0, 0, 500.0 * (k + 1), 0));
        return out;
    };
    CurveCommand append;
    append.append = 1.0;
    const CommandResult live = w.submit(c172, CurveCommand{}, run(0, 10));
    REQUIRE(live.accepted());
    REQUIRE(w.update(live.activity, append, run(10, 10)).accepted());
    REQUIRE(w.update(live.activity, append, run(20, 10)).accepted());
    const CommandResult full = w.update(live.activity, append, run(30, 10));
    CHECK(full.reason == Reason::InvalidCurve);
    CHECK(full.index == static_cast<int>(PathStore::kSegments) - 30);
    CHECK(w.update(live.activity, append, run(30, 2)).accepted()); // (as many as there is room for)
}

TEST_CASE("curve: every aircraft offers A-GRA's curve following, which takes UPDATE", "[modes]") {
    session::World w(options("curves-discovery"));
    const std::uint32_t aircraft[] = {wing(w, "c172x", 1500.0, 55.0, 0), wing(w, "b52h", 3000.0, 180.0, 2), wing(w, "f16c", 3000.0, 160.0, 4),
                                      rotor(w, "uh60", 0.0, 6), rotor(w, "iris", 0.0, 8)};
    for (const auto v : aircraft) {
        const CapabilityDescriptor* d = nullptr;
        for (const auto& c : w.capabilities(v))
            if (c.id == "fsim.guidance.curve") d = &c;
        REQUIRE(d != nullptr);
        CHECK(d->mode == FlightMode::CurveFollowing);
        CHECK(d->setpoint == SetpointKind::Curve);
        CHECK(d->persistence == Persistence::Terminating);
        CHECK((d->interactions & kUpdate) != 0);
        REQUIRE(d->parameters.size() == 20);
        CHECK(d->parameters[7].name == "append");
    }
}
