// The command envelope (docs/flight-autonomy.md, 4.8; ADR-29 FA-2): a
// command's id and the requirements it comes from go with its activity and
// come back in its answers; a validation is answered as a NEW would be and
// nothing flies; every finding is named, the answer's reason the first, and
// every value flown other than asked; several NEWs at once are each answered
// on their own.
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string_view>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

double code(SpeedReference r) { return static_cast<double>(r); }

/// A waypoint `north` and `east` metres from where the vehicle is, at `altitudeM`.
Waypoint at(const sim::VehicleState& s, double north, double east, double altitudeM) {
    Waypoint w;
    w.latitudeRad = s.latitudeRad + north / kEarthM;
    w.longitudeRad = s.longitudeRad + east / (kEarthM * std::cos(s.latitudeRad));
    w.altitudeM = altitudeM;
    return w;
}

bool same(const Finding& f, Reason reason, int index, Constraint constraint) {
    return f.reason == reason && f.index == index && f.constraint == constraint;
}

} // namespace

TEST_CASE("envelope: a command's id and requirements go with its activity and come back in its answers", "[envelope]") {
    session::World w(options("envelope-ids"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    CommandOptions o;
    o.commandId = 42;
    o.trace[0] = {RequirementKind::Task, 7};
    o.trace[1] = {RequirementKind::Command, 9};
    o.interactive = false;
    HsaCommand h;
    h.headingRad = 0.5;
    const CommandResult r = w.submit(f16, h, o);
    REQUIRE(r.accepted());
    CHECK(r.newActivity);
    CHECK(r.commandId == 42);
    const ActivityRecord& a = *w.activity(r.activity);
    CHECK(a.commandId == 42);
    CHECK((a.trace[0].kind == RequirementKind::Task && a.trace[0].id == 7));
    CHECK((a.trace[1].kind == RequirementKind::Command && a.trace[1].id == 9));
    CHECK(a.trace[2].kind == RequirementKind::None);
    CHECK_FALSE(a.interactive);
    CHECK(std::string_view(requirementKindName(RequirementKind::Effect)) == "effect");

    // an UPDATE echoes the id and makes nothing new; a refused one too
    HsaCommand turn;
    turn.headingRad = 1.0;
    CommandResult u = w.update(r.activity, Command(turn));
    REQUIRE(u.accepted());
    CHECK_FALSE(u.newActivity);
    CHECK(u.commandId == 42);
    u = w.update(r.activity, Command(VelocityCommand{}));
    CHECK(u.reason == Reason::WrongCommandType);
    CHECK(u.commandId == 42);
    // and a CANCEL
    const CommandResult c = w.cancel(r.activity);
    CHECK(c.status == CommandStatus::Canceled);
    CHECK(c.commandId == 42);

    // the existing entry points: the first command makes an activity, the next the live one takes
    CHECK(w.commandResult(f16, VelocityCommand{160.0, 0.0, 0.5}).newActivity);
    CHECK_FALSE(w.commandResult(f16, VelocityCommand{160.0, 0.0, 0.6}).newActivity);

    // what a refusal is about: the activity holding the authority
    CommandOptions autopilot;
    autopilot.source = Source::Autopilot;
    const CommandResult held = w.submit(f16, VelocityCommand{160.0, 0.0, 0.5}, autopilot);
    REQUIRE(held.accepted());
    const CommandResult refused = w.submit(f16, AttitudeCommand{});
    CHECK(refused.reason == Reason::AuthorityHeld);
    CHECK(refused.other == held.activity);
}

TEST_CASE("envelope: a validation is answered as a NEW would be, and nothing flies", "[envelope]") {
    session::World w(options("envelope-validate"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    const std::size_t before = w.activities(f16).size();
    CommandOptions v;
    v.validateOnly = true;
    v.commandId = 5;
    HsaCommand h;
    h.headingRad = 1.0;
    CommandResult r = w.submit(f16, h, v);
    CHECK(r.status == CommandStatus::Valid);
    CHECK_FALSE(r.accepted());
    CHECK(r.activity == 0);
    CHECK_FALSE(r.newActivity);
    CHECK(r.commandId == 5);
    // what Clamp would fly: valid, with the value it would hold
    HsaCommand fast;
    fast.speed = 600.0, fast.speedReference = code(SpeedReference::TrueAirspeed);
    r = w.submit(f16, fast, v);
    CHECK(r.status == CommandStatus::Valid);
    CHECK((r.flags & kClamped) != 0);
    const CommandDetails& d = *w.commandDetails(f16);
    REQUIRE(d.adjustmentCount == 1);
    CHECK(d.adjustments[0].index == 2);
    CHECK(d.adjustments[0].constraint == Constraint::MaxAirspeed);
    CHECK(d.adjustments[0].requested == 600.0);
    CHECK(d.adjustments[0].adjusted < 450.0);
    // with Reject, refused as a NEW would be
    v.range = RangePolicy::Reject;
    r = w.submit(f16, fast, v);
    CHECK(r.status == CommandStatus::Rejected);
    CHECK(r.reason == Reason::PerformanceLimit);
    // a support effector's, and an authority's refusal, the same
    CHECK(w.submit(f16, GearCommand{1.0}, v).status == CommandStatus::Valid);
    CommandOptions override;
    override.source = Source::Override;
    REQUIRE(w.submit(f16, VelocityCommand{160.0, 0.0, 0.5}, override).accepted());
    const std::size_t flying = w.activities(f16).size();
    v.range = RangePolicy::Clamp;
    CHECK(w.submit(f16, h, v).reason == Reason::AuthorityHeld);
    // nothing flew for any of them
    CHECK(flying == before + 1);
    CHECK(w.activities(f16).size() == flying);
}

TEST_CASE("envelope: every finding is named, the answer's reason the first, and every value flown other than asked", "[envelope]") {
    session::World w(options("envelope-findings"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    CommandOptions reject;
    reject.range = RangePolicy::Reject;
    // faster than it flies and higher than its ceiling: the altitude is checked first, as it was
    HsaCommand both;
    both.speed = 600.0, both.speedReference = code(SpeedReference::TrueAirspeed);
    both.altitudeM = 25000.0;
    CommandResult r = w.submit(f16, both, reject);
    CHECK(r.reason == Reason::PerformanceLimit);
    CHECK(r.index == 4);
    CHECK(r.constraint == Constraint::MaxAltitude);
    CommandDetails d = *w.commandDetails(f16);
    REQUIRE(d.findingCount == 2);
    CHECK(same(d.findings[0], Reason::PerformanceLimit, 4, Constraint::MaxAltitude));
    CHECK(same(d.findings[1], Reason::PerformanceLimit, 2, Constraint::MaxAirspeed));
    CHECK(std::string_view(d.findings[1].description) == reasonDescription(Reason::PerformanceLimit));
    CHECK(d.adjustmentCount == 0);
    // clamped instead: flown with both held, each named with what it asked and what it flies
    r = w.submit(f16, both);
    REQUIRE(r.accepted());
    CHECK((r.flags & kClamped) != 0);
    CHECK(r.index == 4); // (the first clamp's, as it was)
    d = *w.commandDetails(f16);
    CHECK(d.findingCount == 0);
    REQUIRE(d.adjustmentCount == 2);
    CHECK((d.adjustments[0].index == 4 && d.adjustments[0].constraint == Constraint::MaxAltitude && d.adjustments[0].requested == 25000.0));
    CHECK(d.adjustments[0].adjusted < 25000.0);
    CHECK((d.adjustments[1].index == 2 && d.adjustments[1].constraint == Constraint::MaxAirspeed && d.adjustments[1].requested == 600.0));
    CHECK(d.adjustments[1].adjusted < 450.0);
    // an UPDATE's details are its own
    HsaCommand slower;
    slower.speed = 200.0, slower.speedReference = code(SpeedReference::TrueAirspeed);
    REQUIRE(w.update(r.activity, Command(slower)).accepted());
    CHECK((w.commandDetails(f16)->findingCount == 0 && w.commandDetails(f16)->adjustmentCount == 0));

    // a route: every point beyond the aircraft, each named with its field (A-GRA's invalid segments): the
    // first point too fast, the last too high - and so too steep a climb to it, even held to the ceiling
    const auto& s = *w.vehicleState(f16);
    std::vector<Waypoint> route = {at(s, 0.0, 6000.0, 3000.0), at(s, 0.0, 12000.0, 3000.0), at(s, 0.0, 18000.0, 3000.0)};
    for (Waypoint& p : route) p.speed = 200.0, p.speedReference = code(SpeedReference::TrueAirspeed);
    route[0].speed = 600.0;
    route[2].altitudeM = 25000.0;
    r = w.submit(f16, RouteCommand{}, route, reject);
    CHECK(r.reason == Reason::PerformanceLimit);
    CHECK(r.index == 0);
    d = *w.commandDetails(f16);
    REQUIRE(d.findingCount == 3);
    CHECK(same(d.findings[0], Reason::PerformanceLimit, 0, Constraint::MaxAirspeed));
    CHECK(same(d.findings[1], Reason::PerformanceLimit, 2, Constraint::MaxAltitude));
    CHECK(same(d.findings[2], Reason::PerformanceLimit, 2, Constraint::MaxClimbRate));
    r = w.submit(f16, RouteCommand{}, route);
    REQUIRE(r.accepted());
    d = *w.commandDetails(f16);
    REQUIRE(d.adjustmentCount == 3);
    CHECK((d.adjustments[0].index == 0 && d.adjustments[0].field == 4 && d.adjustments[0].constraint == Constraint::MaxAirspeed));
    CHECK((d.adjustments[1].index == 2 && d.adjustments[1].field == 2 && d.adjustments[1].constraint == Constraint::MaxAltitude));
    CHECK((d.adjustments[2].index == 2 && d.adjustments[2].field == 8 && d.adjustments[2].constraint == Constraint::MaxClimbRate));
    CHECK(d.adjustments[2].adjusted < d.adjustments[2].requested); // flown at the rate it climbs

    // a curve: every section too tight for the aircraft, whatever the policy, each with its section
    BezierSegment corner{{0.0, 150.0, 200.0, 200.0, 200.0, 200.0}, {0.0, 0.0, 0.0, 50.0, 150.0, 200.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    BezierSegment back{{200.0, 200.0, 200.0, 150.0, 50.0, 0.0}, {200.0, 350.0, 400.0, 400.0, 400.0, 400.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    const BezierSegment curve[] = {corner, back};
    r = w.submit(f16, CurveCommand{}, Span<const BezierSegment>(curve, 2));
    CHECK(r.reason == Reason::InvalidCurve);
    CHECK(r.index == 0);
    d = *w.commandDetails(f16);
    REQUIRE(d.findingCount == 2);
    for (std::size_t i = 0; i < 2; ++i) {
        INFO("segment " << i);
        const Finding& f = d.findings[i];
        CHECK(same(f, Reason::InvalidCurve, static_cast<int>(i), Constraint::MaxTurnRate));
        CHECK((f.from >= 0.0f && f.from <= f.to && f.to <= 1.0f));
    }
}

TEST_CASE("envelope: several NEWs at once are each answered on their own", "[envelope]") {
    session::World w(options("envelope-batch"));
    const auto f16 = wing(w, "f16c", 3000.0, 160.0);
    const auto& s = *w.vehicleState(f16);
    const std::vector<Waypoint> route = {at(s, 0.0, 6000.0, 3000.0), at(s, 0.0, 12000.0, 3000.0)};
    HsaCommand h;
    h.headingRad = 1.0;
    std::vector<BatchCommand> batch(3);
    batch[0].command = Command(h), batch[0].options.commandId = 1;
    batch[1].command = SupportCommand(GearCommand{0.5}), batch[1].options.commandId = 2; // (out of its range: refused with Reject)
    batch[1].options.range = RangePolicy::Reject;
    batch[1].command = SupportCommand(GearCommand{2.0});
    batch[2].command = Command(RouteCommand{}), batch[2].waypoints = route, batch[2].options.commandId = 3;
    std::vector<CommandDetails> details;
    const double now = w.simTime();
    const std::vector<CommandResult> results = w.submitBatch(f16, batch, &details);
    REQUIRE(results.size() == 3);
    REQUIRE(details.size() == 3);
    CHECK((results[0].accepted() && results[0].newActivity && results[0].commandId == 1));
    CHECK((results[1].reason == Reason::OutOfRange && results[1].commandId == 2));
    CHECK((details[1].findingCount == 1 && details[1].findings[0].reason == Reason::OutOfRange));
    CHECK((results[2].accepted() && results[2].commandId == 3));
    // made in order at one moment: the route took the hsa's axes
    const ActivityRecord& first = *w.activity(results[0].activity);
    CHECK(first.state == ActivityState::Canceled);
    CHECK(first.reason == Reason::Preempted);
    CHECK(first.by == results[2].activity);
    CHECK((first.startTime == now && w.activity(results[2].activity)->startTime == now));
}
