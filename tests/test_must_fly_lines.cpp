// A must fly through a corridor (docs/flight-autonomy.md, 4.44; ADR-29 FA-8b2): A-GRA's LineTarget and OpLine - vertices on
// the Earth or in a frame, moving, each with its altitude, band and widths - flown through: onto its first segment from
// behind its first vertex, every vertex flown by, the last flown over; each turn kept within its widths, or refused.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)

void fly(session::World& w, double seconds) { w.step(stepsFor(w, seconds)); }

/// A corridor north and east of where an aircraft was: its vertices in metres, and on the Earth.
struct Corridor {
    double lat0 = 0.0, lon0 = 0.0;
    std::vector<std::pair<double, double>> points; ///< north, east
    OpLine line;

    Corridor(const sim::VehicleState& s, std::vector<std::pair<double, double>> at, double widthM) : lat0(s.latitudeRad), lon0(s.longitudeRad), points(at) {
        for (const auto& [n, e] : points) {
            LineVertex v;
            v.latitudeRad = lat0 + n / kR, v.longitudeRad = lon0 + e / (kR * std::cos(lat0));
            line.vertices.push_back(v);
        }
        if (widthM > 0.0) line.leftWidthM = line.rightWidthM = widthM;
    }
    /// The aircraft north and east of the reference.
    void where(const sim::VehicleState& s, double& n, double& e) const {
        n = (s.latitudeRad - lat0) * kR;
        e = (s.longitudeRad - lon0) * kR * std::cos(lat0);
    }
    /// How far the aircraft is from the corridor's line, and whether it is past its first vertex (along its first segment).
    double off(const sim::VehicleState& s, bool& past) const {
        double n = 0.0, e = 0.0, best = 1e300;
        where(s, n, e);
        for (std::size_t i = 0; i + 1 < points.size(); ++i) {
            const double an = points[i].first, ae = points[i].second, dn = points[i + 1].first - an, de = points[i + 1].second - ae;
            const double raw = ((n - an) * dn + (e - ae) * de) / (dn * dn + de * de), t = std::min(std::max(raw, 0.0), 1.0);
            if (i == 0 && raw > 0.0) past = true;
            best = std::min(best, std::hypot(n - an - t * dn, e - ae - t * de));
        }
        return best;
    }
    /// How far the aircraft is from its last vertex.
    double fromEnd(const sim::VehicleState& s) const {
        double n = 0.0, e = 0.0;
        where(s, n, e);
        return std::hypot(n - points.back().first, e - points.back().second);
    }
};

/// Each must fly flown until it ended: the most each aircraft was off its corridor's line once past its first vertex, and
/// how far from its last vertex, and at what altitude, it ended.
struct Flown {
    ActivityState state = ActivityState::Active;
    double worstOffM = 0.0, fromEndM = 0.0, altitudeM = 0.0, timeS = 0.0;
};

std::vector<Flown> flyThrough(session::World& w, const std::vector<std::uint32_t>& ids, const std::vector<ActivityId>& acts,
                              const std::vector<const Corridor*>& corridors, double seconds) {
    std::vector<Flown> out(ids.size());
    std::vector<bool> done(ids.size(), false), past(ids.size(), false);
    for (int i = 0; i < static_cast<int>(seconds * 10.0); ++i) {
        fly(w, 0.1);
        bool live = false;
        for (std::size_t k = 0; k < ids.size(); ++k) {
            if (done[k]) continue;
            const sim::VehicleState& s = *w.vehicleState(ids[k]);
            bool now = past[k];
            const double off = corridors[k]->off(s, now);
            past[k] = now;
            if (past[k]) out[k].worstOffM = std::max(out[k].worstOffM, off);
            const ActivityRecord* r = w.activity(acts[k]);
            if (r->live()) {
                live = true;
                continue;
            }
            out[k].state = r->state, out[k].fromEndM = corridors[k]->fromEnd(s), out[k].altitudeM = s.altitudeMslM, out[k].timeS = w.simTime();
            done[k] = true;
        }
        if (!live) break;
    }
    return out;
}

} // namespace

TEST_CASE("must fly: a corridor flown through, each turn within its widths, per class (MFY-05)", "[modes][must_fly]") {
    session::World w(options("must-fly-lines"));
    const auto iris = rotor(w, "iris", 15.0, 6), uh60 = rotor(w, "uh60", 15.0, 9);
    const auto c172 = wing(w, "c172x", 1500.0, 55.0), f16 = wing(w, "f16c", 3000.0, 160.0, 3);
    // an L ahead of the C172 (a right angle left), a Z ahead of the F-16C (two 45-degree turns), an S ahead of the UH-60A
    // and the IRIS (two 45-degree turns) - each wide enough for its turns
    const Corridor l(*w.vehicleState(c172), {{0.0, 3000.0}, {0.0, 7000.0}, {4000.0, 7000.0}}, 500.0);
    const Corridor z(*w.vehicleState(f16), {{0.0, 8000.0}, {0.0, 20000.0}, {8000.0, 28000.0}, {8000.0, 40000.0}}, 3000.0);
    const Corridor s(*w.vehicleState(uh60), {{300.0, 0.0}, {900.0, 0.0}, {1300.0, 400.0}, {1900.0, 400.0}}, 150.0);
    const Corridor i(*w.vehicleState(iris), {{30.0, 0.0}, {90.0, 0.0}, {130.0, 40.0}, {190.0, 40.0}}, 25.0);
    MustFlyCommand through;
    through.location = static_cast<double>(MustFlyLocation::Line);
    const std::vector<std::uint32_t> ids = {c172, f16, uh60, iris};
    const std::vector<const Corridor*> corridors = {&l, &z, &s, &i};
    std::vector<ActivityId> acts;
    for (std::size_t k = 0; k < ids.size(); ++k) {
        const CommandResult r = w.submit(ids[k], through, corridors[k]->line);
        INFO(k << ": " << reasonName(r.reason) << " at " << r.index);
        REQUIRE(r.accepted());
        acts.push_back(r.activity);
    }
    const std::vector<Flown> flown = flyThrough(w, ids, acts, corridors, 400.0);
    const double widths[] = {500.0, 3000.0, 150.0, 25.0};
    for (std::size_t k = 0; k < flown.size(); ++k) {
        INFO(k << ": off by " << flown[k].worstOffM << " m, ended " << flown[k].fromEndM << " m from its end");
        CHECK(flown[k].state == ActivityState::Completed);
        CHECK(flown[k].worstOffM <= widths[k]); // (within its widths once in it: the C172 122 m, the F-16C 170, the UH-60A 15, the IRIS 0.9)
        CHECK(flown[k].fromEndM <= 20.0);       // (done as its last vertex is passed: 0.4 to 6.5 m from it)
    }
    CHECK(w.supportTable(c172)->find("fsim.guidance.must_fly")->support == Support::Supported);
}

TEST_CASE("must fly: a corridor's band and altitudes flown, its window of bearings kept; a turn too tight refused", "[modes][must_fly]") {
    session::World w(options("must-fly-line-band"));
    const auto banded = wing(w, "c172x", 1500.0, 55.0), heights = wing(w, "c172x", 1500.0, 55.0, 3), windowed = wing(w, "c172x", 1500.0, 55.0, 6);
    // a band above the aircraft, 1,700 to 1,800 m; two vertices at 1,600 and 1,650 m
    Corridor up(*w.vehicleState(banded), {{0.0, 3000.0}, {0.0, 9000.0}}, 0.0);
    up.line.altitudeMinM = 1700.0, up.line.altitudeMaxM = 1800.0;
    Corridor own(*w.vehicleState(heights), {{0.0, 3000.0}, {0.0, 9000.0}}, 0.0);
    own.line.vertices[0].altitudeM = 1600.0, own.line.vertices[1].altitudeM = 1650.0;
    // a corridor running north from 5 km east, entered from the west (bearings 260 to 280 degrees from its first vertex)
    const Corridor north(*w.vehicleState(windowed), {{0.0, 5000.0}, {5000.0, 5000.0}}, 1000.0);
    MustFlyCommand through;
    through.location = static_cast<double>(MustFlyLocation::Line);
    const CommandResult r1 = w.submit(banded, through, up.line), r2 = w.submit(heights, through, own.line);
    MustFlyCommand fromWest = through;
    fromWest.ingressMinRad = -100.0 * kDeg, fromWest.ingressMaxRad = -80.0 * kDeg;
    const CommandResult r3 = w.submit(windowed, fromWest, north.line);
    INFO(reasonName(r1.reason) << " " << reasonName(r2.reason) << " " << reasonName(r3.reason) << " at " << r3.index);
    REQUIRE((r1.accepted() && r2.accepted() && r3.accepted()));
    // a right angle in a corridor 20 m wide: the C172's turn cuts inside it by more - refused, naming the turn's point
    const sim::VehicleState a = *w.vehicleState(banded);
    const Corridor tight(a, {{0.0, 3000.0}, {0.0, 6000.0}, {3000.0, 6000.0}}, 20.0);
    const CommandResult refused = w.submit(banded, through, tight.line);
    CHECK(refused.reason == Reason::PerformanceLimit);
    CHECK(refused.constraint == Constraint::MaxTurnRate);
    CHECK(refused.index == 1); // (its second vertex: straight in to its first, so the route's point 1)
    double cameFrom = std::numeric_limits<double>::quiet_NaN();
    std::vector<Flown> flown(3);
    std::vector<bool> done(3, false);
    const ActivityId acts[] = {r1.activity, r2.activity, r3.activity};
    const std::uint32_t ids[] = {banded, heights, windowed};
    const Corridor* corridors[] = {&up, &own, &north};
    for (int k = 0; k < 3000 && !(done[0] && done[1] && done[2]); ++k) {
        fly(w, 0.1);
        for (std::size_t j = 0; j < 3; ++j) {
            const sim::VehicleState& s = *w.vehicleState(ids[j]);
            if (j == 2 && std::isnan(cameFrom)) { // (its track as it comes within 300 m of the first vertex)
                double n = 0.0, e = 0.0;
                north.where(s, n, e);
                if (std::hypot(n, e - 5000.0) < 300.0) cameFrom = track(s);
            }
            const ActivityRecord* r = w.activity(acts[j]);
            if (done[j] || r->live()) continue;
            flown[j].state = r->state, flown[j].altitudeM = s.altitudeMslM, flown[j].fromEndM = corridors[j]->fromEnd(s);
            done[j] = true;
        }
    }
    for (const Flown& f : flown) CHECK(f.state == ActivityState::Completed);
    CHECK((flown[0].altitudeM >= 1700.0 && flown[0].altitudeM <= 1800.0)); // (in its band)
    CHECK(std::abs(flown[1].altitudeM - 1650.0) <= 15.0);                    // (its last vertex's own)
    INFO("came in on " << cameFrom / kDeg);
    CHECK(std::abs(degreesApart(cameFrom, 90.0 * kDeg)) <= 20.0); // (heading east: from the west)
}

TEST_CASE("must fly: an operational line kept by its id, one in a frame and one moving; refusals naming its fields (MFY-03, ENV-06)",
          "[modes][must_fly]") {
    session::World w(options("must-fly-op-lines"));
    const auto v = wing(w, "c172x", 1500.0, 55.0), framed = wing(w, "c172x", 1500.0, 55.0, 3), moving = wing(w, "c172x", 1500.0, 55.0, 6);
    Corridor kept(*w.vehicleState(v), {{0.0, 3000.0}, {0.0, 6000.0}}, 300.0);
    kept.line.id = 31;
    CHECK(w.setOpLine(kept.line) == Reason::None);
    CHECK(w.opLine(31)->revision == 1);
    CHECK(w.setOpLine(kept.line) == Reason::None);
    CHECK(w.opLine(31)->revision == 2);
    CHECK(w.opLines() == std::vector<OpLineId>{31});
    CHECK(w.opLine(31)->vertices.size() == 2);
    MustFlyCommand byId;
    byId.location = static_cast<double>(MustFlyLocation::OpLine), byId.target = 31.0;
    const CommandResult r = w.submit(v, byId);
    INFO(reasonName(r.reason) << " at " << r.index);
    REQUIRE(r.accepted());
    // a line in a fixed frame turned 90 degrees (its x east), from 3 to 6 km along x from the second aircraft; one moving north
    // at 10 m/s, from 1.5 km south of the third
    const sim::VehicleState f = *w.vehicleState(framed), m = *w.vehicleState(moving);
    FrameSpec turned;
    turned.latitudeRad = f.latitudeRad, turned.longitudeRad = f.longitudeRad, turned.altitudeMslM = 1500.0, turned.yawRad = 90.0 * kDeg;
    const FrameId frame = w.createFrame(turned);
    REQUIRE(frame != 0);
    OpLine inFrame;
    inFrame.frame = static_cast<double>(frame), inFrame.frameRotation = static_cast<double>(FrameRotation::Yaw);
    for (const double x : {3000.0, 6000.0}) {
        LineVertex q;
        q.xM = x, q.yM = 0.0;
        inFrame.vertices.push_back(q);
    }
    Corridor drifting(m, {{-1500.0, 3000.0}, {-1500.0, 7000.0}}, 0.0);
    drifting.line.northMs = 10.0, drifting.line.eastMs = 0.0;
    MustFlyCommand through;
    through.location = static_cast<double>(MustFlyLocation::Line);
    const CommandResult rf = w.submit(framed, through, inFrame), rm = w.submit(moving, through, drifting.line);
    INFO(reasonName(rf.reason) << " " << reasonName(rm.reason));
    REQUIRE((rf.accepted() && rm.accepted()));
    const double t0 = w.simTime();
    std::vector<Flown> ended(3);
    std::vector<bool> done(3, false);
    const ActivityId acts[] = {r.activity, rf.activity, rm.activity};
    const std::uint32_t ids[] = {v, framed, moving};
    for (int k = 0; k < 3000 && !(done[0] && done[1] && done[2]); ++k) {
        fly(w, 0.1);
        for (std::size_t j = 0; j < 3; ++j) {
            const ActivityRecord* a = w.activity(acts[j]);
            if (done[j] || a->live()) continue;
            const sim::VehicleState& s = *w.vehicleState(ids[j]);
            ended[j].state = a->state, ended[j].timeS = w.simTime();
            double n = 0.0, e = 0.0;
            if (j == 0) ended[j].fromEndM = kept.fromEnd(s);
            if (j == 1) offset(s, f.latitudeRad, f.longitudeRad, n, e), ended[j].fromEndM = std::hypot(n, e - 6000.0);
            if (j == 2) { // (its last vertex where the line was then)
                drifting.where(s, n, e);
                ended[j].fromEndM = std::hypot(n - (-1500.0 + 10.0 * (w.simTime() - t0)), e - 7000.0);
            }
            done[j] = true;
        }
    }
    for (const Flown& e : ended) {
        INFO("ended " << e.fromEndM << " m from its last vertex");
        CHECK(e.state == ActivityState::Completed);
        CHECK(e.fromEndM <= 40.0);
    }
    // one the world does not keep, or removed; and lines A-GRA's schema would not take, named from its field 10
    byId.target = 32.0;
    CHECK(w.submit(v, byId).reason == Reason::UnknownGeometry);
    CHECK(w.removeOpLine(31));
    CHECK_FALSE(w.removeOpLine(31));
    byId.target = 31.0;
    CHECK(w.submit(v, byId).reason == Reason::UnknownGeometry);
    auto refusal = [&](void (*spoil)(OpLine&)) {
        OpLine l = kept.line;
        l.id = 0;
        spoil(l);
        const CommandResult bad = w.submit(v, through, l);
        return bad.reason == Reason::InvalidParameter ? static_cast<int>(bad.index) : -100;
    };
    CHECK(refusal([](OpLine& l) { l.vertices.resize(1); }) == 10);
    CHECK(refusal([](OpLine& l) { l.vertices[1] = l.vertices[0]; }) == 10);                   // (two at one place)
    CHECK(refusal([](OpLine& l) { l.vertices[0].altitudeMinM = 2000.0, l.vertices[0].altitudeMaxM = 1000.0; }) == 10); // (its range)
    CHECK(refusal([](OpLine& l) { l.projection = 5.0; }) == 11);
    CHECK(refusal([](OpLine& l) { l.leftWidthM = -1.0; }) == 12);
    CHECK(refusal([](OpLine& l) { l.altitudeMinM = 2000.0, l.altitudeMaxM = 1000.0; }) == 13);
    CHECK(refusal([](OpLine& l) { l.frame = 99.0; }) == 14);
    CHECK(refusal([](OpLine& l) { l.northMs = 5.0; }) == 15);                                    // (a velocity one way)
    CHECK(w.submit(v, through).reason == Reason::InvalidParameter);                               // (a line's, given none)
    // an altitude given outside its band, or in another reference: refused as a zone's
    OpLine banded = kept.line;
    banded.id = 0, banded.altitudeMinM = 1000.0, banded.altitudeMaxM = 2000.0;
    MustFlyCommand at = through;
    at.altitudeM = 2500.0;
    CommandResult outside = w.submit(v, at, banded);
    CHECK((outside.reason == Reason::InvalidParameter && outside.index == 3));
    at.altitudeM = 1500.0, at.altitudeReference = static_cast<double>(AltitudeReference::AboveGround);
    outside = w.submit(v, at, banded);
    CHECK((outside.reason == Reason::InvalidParameter && outside.index == 4));
    OpLine none = kept.line;
    none.id = 0;
    CHECK(w.setOpLine(none) == Reason::InvalidParameter);
    CHECK(w.supportTable(v)->find("fsim.geometry")->support == Support::Supported);
}

TEST_CASE("must fly: a corridor's UPDATE flies the new one, or keeps its own; queued with its line, it starts later", "[modes][must_fly]") {
    session::World w(options("must-fly-line-update"));
    const auto v = wing(w, "c172x", 1500.0, 55.0), kept = wing(w, "c172x", 1500.0, 55.0, 3), later = wing(w, "c172x", 1500.0, 55.0, 6);
    const Corridor first(*w.vehicleState(v), {{0.0, 3000.0}, {0.0, 8000.0}}, 300.0);
    const Corridor moved(*w.vehicleState(v), {{2000.0, 3000.0}, {2000.0, 8000.0}}, 300.0);
    const Corridor own(*w.vehicleState(kept), {{0.0, 3000.0}, {0.0, 8000.0}}, 300.0);
    const Corridor queued(*w.vehicleState(later), {{0.0, 4000.0}, {0.0, 8000.0}}, 300.0);
    MustFlyCommand through;
    through.location = static_cast<double>(MustFlyLocation::Line);
    const CommandResult r = w.submit(v, through, first.line), rk = w.submit(kept, through, own.line);
    REQUIRE((r.accepted() && rk.accepted()));
    fly(w, 10.0);
    CHECK(w.update(r.activity, MustFlyCommand{}, moved.line).accepted());
    MustFlyCommand faster; // (its speed alone: its line kept)
    faster.speed = 60.0;
    const CommandResult u = w.update(rk.activity, Command(faster));
    INFO(reasonName(u.reason) << " at " << u.index);
    CHECK(u.accepted());
    CommandOptions wait;
    wait.window.startNotBefore = w.simTime() + 20.0;
    const CommandResult q = w.submit(later, through, queued.line, wait);
    REQUIRE(q.accepted());
    CHECK((q.flags & kDeferred) != 0);
    const std::vector<Flown> flown = flyThrough(w, {v, kept, later}, {r.activity, rk.activity, q.activity}, {&moved, &own, &queued}, 300.0);
    for (const Flown& f : flown) {
        INFO("ended " << f.fromEndM << " m from its last vertex");
        CHECK(f.state == ActivityState::Completed);
        CHECK(f.fromEndM <= 40.0);
    }
}
