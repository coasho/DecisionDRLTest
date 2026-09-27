// The control architecture's conformance suite (docs/control-architecture.md,
// section 14, step 6). The command lifecycle (section 10) is one state
// machine whichever adapter flies an aircraft and whatever its catalog offers:
// - every capability of every aircraft the platform ships answers NEW, UPDATE
//   and CANCEL as its descriptor says;
// - one aircraft per adapter keeps the lifecycle's rules through seeded random
//   sequences of every operation - NEW, UPDATE, CANCEL, the existing entry
//   point, world steps, resets, the vehicle default, a target that goes - and
//   the same sequence gives the same answers.
#include "control/Adapter.h"
#include "control/Catalog.h"
#include "control/Runtime.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iterator>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

constexpr double kEarthRadiusM = 6371000.0;

/// One aircraft per adapter family, and where it flies.
struct Aircraft {
    const char* type;
    const char* family;
    double altitudeM, tasMs;
};
constexpr Aircraft kAdapters[] = {{"jsbsim:c172x", "jsbsim.stock", 2500.0, 55.0},
                                  {"jsbsim:b52h", "jsbsim.direct", 3000.0, 180.0},
                                  {"jsbsim:f16c", "jsbsim.fbw", 3000.0, 160.0},
                                  {"jsbsim:uh1h", "jsbsim.helicopter", 300.0, 0.0},
                                  {"jsbsim:iris", "jsbsim.multirotor", 100.0, 0.0}};

session::WorldOptions options(const std::string& name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 5;
    return o;
}

session::VehicleSpec spec(const std::string& name, const std::string& type, double altitudeM, double tasMs, double northDeg = 0.0) {
    session::VehicleSpec s;
    s.name = name;
    s.type = type;
    s.initial.latitudeDeg += northDeg;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = tasMs;
    return s;
}

std::string familyOf(const session::World& w, std::uint32_t v) { return adapterFor(w.profile(v)->identity.family).family(); }

/// Every aircraft the platform ships: the stock c172x and hangar's designs.
std::vector<std::string> shippedAircraft() {
    std::vector<std::string> out{"jsbsim:c172x"};
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(FSIM_TEST_AIRCRAFT_DIR, ec)) {
        const std::string name = entry.path().filename().string();
        if (entry.is_directory(ec) && std::filesystem::is_regular_file(entry.path() / (name + ".xml"), ec)) out.push_back("jsbsim:" + name);
    }
    std::sort(out.begin() + 1, out.end());
    return out;
}

bool sameRecord(const ActivityRecord& a, const ActivityRecord& b) {
    const bool ends = (std::isnan(a.endTime) && std::isnan(b.endTime)) || a.endTime == b.endTime;
    return a.id == b.id && a.vehicle == b.vehicle && a.capability == b.capability && a.source == b.source && a.axes == b.axes &&
           a.state == b.state && a.reason == b.reason && a.by == b.by && a.constraints == b.constraints &&
           a.constraintsSeen == b.constraintsSeen && a.startTime == b.startTime && ends && a.waiting == b.waiting && a.waitingFor == b.waitingFor &&
           a.rank == b.rank && a.precedence == b.precedence && a.interrupt == b.interrupt;
}

/// What the rules of docs/flight-autonomy.md, 4.9 let a contender do about a live
/// activity `h` on its axes - the model the walks hold the host to.
enum class Standing { Takes, Waits, Refused };
Standing standing(Source source, std::uint32_t precedence, Rank rank, bool interrupt, const ActivityRecord& h) {
    if (h.source > source) return interrupt ? Standing::Refused : Standing::Waits; // a higher source's
    const bool platform = source != Source::Policy;
    if (platform && interrupt) return Standing::Takes;   // the primary controller: any rank
    if (!platform && !interrupt) return Standing::Waits; // a policy's nice command
    if (precedence != h.precedence) return precedence < h.precedence ? Standing::Takes : Standing::Waits;
    return ranksAhead(h.rank, rank) ? Standing::Waits : Standing::Takes; // equal: the newest
}
Standing standing(const ActivityRecord& c, const ActivityRecord& h) { return standing(c.source, c.precedence, c.rank, c.interrupt, h); }

std::uint32_t serialOf(ActivityId id) { return static_cast<std::uint32_t>(id & 0xFFFFFFFFu); }

/// Makes commands for a vehicle's capabilities, each value from the
/// capability's descriptor: its default, or - wild - drawn inside its range
/// about the flight, now and then outside it or left out.
class Maker {
public:
    Maker(session::World& w, std::uint32_t vehicle, std::uint64_t seed) : w_(w), vehicle_(vehicle), rng_(seed) {}

    std::uint32_t target = 0; ///< the vehicle a guidance capability that needs one follows
    std::vector<Waypoint> waypoints; ///< the last route's, made beside its RouteCommand
    std::vector<BezierSegment> segments; ///< the last curve's, made beside its CurveCommand
    double curveEnd[3] = {0.0, 0.0, 0.0}; ///< where they end, from their reference: an append joins there

    double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng_); }
    bool chance(double p) { return uniform(0.0, 1.0) < p; }
    std::size_t pick(std::size_t n) { return std::uniform_int_distribution<std::size_t>(0, n - 1)(rng_); }

    /// A flight or guidance capability's command; false for the others.
    bool cascade(const CapabilityDescriptor& d, bool wild, Command& out) {
        if (d.kind == CapabilityKind::Guidance && d.setpoint == SetpointKind::Hsa) { // a mode: its fixed-size setpoint's fields
            out = HsaCommand{};
            double* fields[kMaxCommandFields];
            const std::size_t n = std::min(commandFields(out, fields), d.parameters.size());
            for (std::size_t i = 0; i < n; ++i) *fields[i] = value(d.parameters[i], wild);
            return true;
        }
        if (d.kind == CapabilityKind::Guidance && d.setpoint == SetpointKind::Pattern) { // left out (the defaults), or about the flight
            PatternCommand c;
            if (wild) {
                const auto& s = state();
                auto some = [&](double v) { return chance(0.5) ? kHold : v; };
                auto whole = [&](std::size_t n) { return chance(0.6) ? kHold : static_cast<double>(pick(n)); };
                c.pattern = whole(static_cast<std::size_t>(PatternKind::Count));
                if (chance(0.5)) c.latitudeRad = s.latitudeRad + uniform(-0.002, 0.002), c.longitudeRad = s.longitudeRad + uniform(-0.002, 0.002);
                c.altitudeM = some(s.altitudeMslM + uniform(-200.0, 200.0));
                c.radiusM = some(uniform(1.0, 3000.0));
                c.clockwise = whole(2);
                c.courseRad = some(uniform(-3.0, 3.0));
                c.legM = some(uniform(0.0, 4000.0));
                c.speed = some(s.airspeedTrueMs > 5.0 ? s.airspeedTrueMs * uniform(0.85, 1.15) : uniform(2.0, 8.0));
                c.durationS = chance(0.7) ? kHold : uniform(2.0, 60.0);
                if (chance(0.1)) { // now and then a field out of its range: refused, or clamped
                    out = c;
                    double* fields[kMaxCommandFields];
                    const std::size_t n = std::min(commandFields(out, fields), d.parameters.size());
                    const std::size_t i = pick(n);
                    *fields[i] = value(d.parameters[i], true);
                    return true;
                }
            }
            out = c;
            return true;
        }
        if (d.kind == CapabilityKind::Guidance && d.setpoint == SetpointKind::Curve) { // its options, its segments beside them
            CurveCommand c;
            const auto& s = state();
            const double ground = std::max(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]), 3.0);
            if (wild) { // left out mostly, else about the flight
                auto some = [&](double v) { return chance(0.6) ? kHold : v; };
                c.altitudeM = some(s.altitudeMslM + uniform(-100.0, 100.0));
                c.speedMinMs = some(ground * uniform(0.6, 1.0));
                c.speedMaxMs = some(ground * uniform(1.0, 1.4));
                c.durationS = chance(0.85) ? kHold : uniform(30.0, 300.0);
                c.end = chance(0.6) ? kHold : static_cast<double>(pick(2));
                c.append = chance(0.8) ? kHold : static_cast<double>(pick(2));
                if (chance(0.1)) { // now and then a field out of its range: refused, or clamped
                    out = c;
                    double* fields[kMaxCommandFields];
                    const std::size_t n = std::min(commandFields(out, fields), d.parameters.size());
                    const std::size_t i = pick(n);
                    *fields[i] = value(d.parameters[i], true);
                    c = std::get<CurveCommand>(out);
                }
            }
            out = c;
            // straight pieces of 40 s each, zigzagging ahead from its reference (the aircraft, at a NEW);
            // an append mostly from where the last curve made ended; now and then one it cannot fly
            const bool joins = wild && c.append == 1.0 && chance(0.7);
            const int n = wild ? 1 + static_cast<int>(pick(4)) : 2;
            const double leg = 40.0 * ground, psi = s.eulerRad[2];
            double n0 = joins ? curveEnd[0] : 0.0, e0 = joins ? curveEnd[1] : 0.0;
            const double d0 = joins ? curveEnd[2] : 0.0;
            segments.clear();
            for (int k = 0; k < n; ++k) {
                const double side = (k % 2 ? 0.2 : -0.2) * leg;
                const double n1 = n0 + leg * std::cos(psi) - side * std::sin(psi), e1 = e0 + leg * std::sin(psi) + side * std::cos(psi);
                BezierSegment b;
                for (int i = 0; i < 6; ++i) {
                    const double u = i / 5.0;
                    b.north[i] = n0 + u * (n1 - n0), b.east[i] = e0 + u * (e1 - e0), b.down[i] = d0;
                }
                if (wild && chance(0.03)) b.east[pick(6)] = std::numeric_limits<double>::quiet_NaN();
                if (wild && k > 0 && chance(0.03)) b.north[0] += 5.0; // apart from the one before
                segments.push_back(b);
                n0 = n1, e0 = e1;
            }
            curveEnd[0] = n0, curveEnd[1] = e0, curveEnd[2] = d0;
            return true;
        }
        if (d.kind == CapabilityKind::Guidance && d.setpoint == SetpointKind::Route) { // its options, its waypoints beside them
            const int points = wild ? 1 + static_cast<int>(pick(4)) : 2;
            RouteCommand r;
            if (wild) { // whole options mostly (a projection, a repeat, an end, a start it has), else anything its ranges allow
                double* fields[kMaxCommandFields];
                const std::size_t n = std::min(commandFields(out = r, fields), d.parameters.size());
                for (std::size_t i = 0; i < n; ++i) *fields[i] = chance(0.9) ? static_cast<double>(pick(i == 3 ? static_cast<std::size_t>(points) : 2)) : value(d.parameters[i], wild);
                r = std::get<RouteCommand>(out);
            }
            out = r;
            const auto& s = state();
            waypoints.clear();
            for (int k = 0; k < points; ++k) {
                const PositionCommand a = ahead(4000.0 * (k + 1));
                Waypoint p;
                p.latitudeRad = a.latitudeRad + (k % 2) * 2000.0 / kEarthRadiusM; // a turn at each point
                p.longitudeRad = a.longitudeRad;
                if (wild) {
                    if (chance(0.05)) p.latitudeRad = std::numeric_limits<double>::quiet_NaN(); // one it cannot fly
                    if (k > 0 && chance(0.05)) p = waypoints.back();                             // the same place twice
                    if (chance(0.5)) p.altitudeM = s.altitudeMslM + uniform(-200.0, 200.0);
                    if (chance(0.5)) p.speed = s.airspeedTrueMs > 5.0 ? s.airspeedTrueMs * uniform(0.85, 1.15) : uniform(2.0, 8.0);
                    if (chance(0.2)) p.turn = 1.0;
                }
                waypoints.push_back(p);
            }
            return true;
        }
        if (d.kind == CapabilityKind::Guidance) {
            BehaviorCommand b;
            b.id = wild && chance(0.5) ? d.id : d.behavior; // either id selects it
            if (d.needsTarget) b.target = wild && chance(0.1) ? 0 : target;
            for (const auto& p : d.parameters)
                if (const double v = value(p, wild); !std::isnan(v)) b.params[p.name] = v;
            if (d.behavior == "waypoints" && !(wild && chance(0.1))) {
                const int n = wild ? 1 + static_cast<int>(pick(3)) : 1;
                const double first = wild && chance(0.3) ? 100.0 : 8000.0; // inside its capture radius: a route done at once
                for (int i = 0; i < n; ++i) b.points.push_back(ahead(first + 4000.0 * i));
            }
            out = b;
            return true;
        }
        if (d.kind != CapabilityKind::Flight || isSupport(d)) return false;
        switch (d.level) {
        case Level::Attitude: out = AttitudeCommand{}; break;
        case Level::Acceleration: out = AccelerationCommand{}; break;
        case Level::Velocity: out = VelocityCommand{}; break;
        case Level::Position: out = PositionCommand{}; break;
        default: out = ActuatorCommand{}; break;
        }
        double* fields[kMaxCommandFields];
        const std::size_t n = std::min(commandFields(out, fields), d.parameters.size());
        for (std::size_t i = 0; i < n; ++i) *fields[i] = value(d.parameters[i], wild);
        return true;
    }

    /// A support effector's (or the engines') command; false for the others.
    bool support(const CapabilityDescriptor& d, bool wild, SupportCommand& out) {
        for (std::size_t k = 0; k < kSupportKinds; ++k) {
            if (d.id != supportCapability(k)) continue;
            switch (k) {
            case 0: out = GearCommand{}; break;
            case 1: out = FlapsCommand{}; break;
            case 2: out = WheelBrakesCommand{}; break;
            case 3: out = SpeedbrakeCommand{}; break;
            case 4: out = PitchTrimCommand{}; break;
            default: out = EnginesCommand{}; break;
            }
            double* fields[4];
            const std::size_t n = std::min(supportFields(out, fields), d.parameters.size());
            for (std::size_t i = 0; i < n; ++i) *fields[i] = value(d.parameters[i], wild);
            return true;
        }
        return false;
    }

    static bool isSupport(const CapabilityDescriptor& d) {
        for (std::size_t k = 0; k < kSupportKinds; ++k)
            if (d.id == supportCapability(k)) return true;
        return false;
    }

    /// A point `metres` ahead, at the vehicle's height.
    PositionCommand ahead(double metres) const {
        const auto& s = state();
        const double psi = s.eulerRad[2];
        PositionCommand p;
        p.latitudeRad = s.latitudeRad + metres * std::cos(psi) / kEarthRadiusM;
        p.longitudeRad = s.longitudeRad + metres * std::sin(psi) / (kEarthRadiusM * std::cos(s.latitudeRad));
        p.altitudeMslM = s.altitudeMslM;
        return p;
    }

private:
    const sim::VehicleState& state() const { return *w_.vehicleState(vehicle_); }

    double value(const ParameterInfo& p, bool wild) {
        const auto& s = state();
        double def = p.defaultValue;
        if (std::isnan(def) && !p.optional) { // a field it cannot do without: a value for this flight
            if (p.name == "latitude_rad") def = ahead(3000.0).latitudeRad;
            else if (p.name == "longitude_rad") def = ahead(3000.0).longitudeRad;
            else if (p.name == "altitude_msl_m") def = s.altitudeMslM;
            else def = std::isfinite(p.min) ? p.min : 0.0;
        }
        if (!wild) return def;
        if (chance(0.04)) return kHold; // left out: fine where it may be
        if (chance(0.08)) {             // outside its range: clamped, or rejected
            if (std::isfinite(p.max)) return p.max + 1.0 + std::abs(p.max);
            if (std::isfinite(p.min)) return p.min - 1.0 - std::abs(p.min);
        }
        if (std::isnan(def) && chance(0.4)) return kHold; // an optional field left to the loop
        const double centre = std::isnan(def) ? centreOf(p, s) : def, span = spanOf(p, s);
        const double lo = std::max(p.min, centre - span), hi = std::min(p.max, centre + span);
        return lo < hi ? uniform(lo, hi) : std::clamp(centre, p.min, p.max);
    }

    static double centreOf(const ParameterInfo& p, const sim::VehicleState& s) {
        const double deg = 180.0 / 3.14159265358979323846;
        if (p.name.find("airspeed") != std::string::npos) return s.airspeedTrueMs;
        if (p.name == "heading_rad") return s.eulerRad[2];
        if (p.name == "heading_deg") return s.eulerRad[2] * deg;
        if (p.name.find("altitude") != std::string::npos && p.name.find("delta") == std::string::npos) return s.altitudeMslM;
        if (p.name == "lat_deg") return s.latitudeRad * deg;
        if (p.name == "lon_deg") return s.longitudeRad * deg;
        return 0.0;
    }

    static double spanOf(const ParameterInfo& p, const sim::VehicleState& s) {
        if (p.name.find("airspeed") != std::string::npos) return 0.15 * s.airspeedTrueMs;
        if (p.name.find("altitude") != std::string::npos) return 200.0;
        if (p.unit == "rad") return 0.5;
        if (p.unit == "rad/s") return 0.3;
        if (p.unit == "m/s") return 5.0;
        if (p.unit == "m/s2") return 2.0;
        if (p.unit == "g") return 0.8;
        if (p.unit == "m") return 1000.0;
        if (p.unit == "deg") return p.name == "heading_deg" ? 90.0 : 0.05;
        if (p.unit == "s") return 3.0;
        if (p.unit == "1/s") return 0.2;
        return 0.5;
    }

    session::World& w_;
    std::uint32_t vehicle_;
    std::mt19937_64 rng_;
};

/// NEW of what the maker made: a route with the waypoints it made beside it, a curve with its segments.
CommandResult submitMade(session::World& w, std::uint32_t v, const Command& c, const Maker& make, const CommandOptions& options = {}) {
    if (const auto* route = std::get_if<RouteCommand>(&c)) return w.submit(v, *route, make.waypoints, options);
    if (const auto* curve = std::get_if<CurveCommand>(&c)) return w.submit(v, *curve, make.segments, options);
    return w.submit(v, c, options);
}

/// UPDATE with what the maker made: a route's or a curve's options, and now and then its waypoints or segments.
CommandResult updateMade(session::World& w, ActivityId activity, const Command& c, const Maker& make, bool waypoints, Source caller = Source::Policy) {
    if (const auto* route = std::get_if<RouteCommand>(&c); route && waypoints) return w.update(caller, activity, *route, make.waypoints);
    if (const auto* curve = std::get_if<CurveCommand>(&c); curve && waypoints) return w.update(caller, activity, *curve, make.segments);
    return w.update(caller, activity, c);
}

/// A command of another type than `c`: what an UPDATE of it must refuse.
Command otherType(const Command& c) {
    if (std::holds_alternative<VelocityCommand>(c)) return AttitudeCommand{};
    return VelocityCommand{};
}

std::map<ActivityId, ActivityRecord> snapshot(const session::World& w, std::uint32_t v) {
    std::map<ActivityId, ActivityRecord> out;
    for (const auto& r : w.activities(v)) out[r.id] = r;
    return out;
}

// --- every capability of every aircraft -----------------------------------------------------------

/// The descriptor is well formed, and a command built from it goes through
/// NEW, a world step, UPDATE and CANCEL as the descriptor says.
void lifecycle(session::World& w, std::uint32_t v, Maker& make) {
    const std::vector<CapabilityDescriptor> caps = w.capabilities(v); // a copy: discovery may add behaviours
    std::set<std::string> ids;
    std::uint32_t serial = 0;
    for (std::size_t i = 0; i < caps.size(); ++i) {
        const CapabilityDescriptor& d = caps[i];
        INFO(d.id);
        CHECK_FALSE(d.id.empty());
        CHECK(ids.insert(d.id).second);
        for (const auto& p : d.parameters) {
            INFO(p.name);
            CHECK(p.min <= p.max);
            if (!std::isnan(p.defaultValue)) CHECK((p.defaultValue >= p.min && p.defaultValue <= p.max));
        }
        const bool commandable = (d.interactions & kCommand) != 0;
        CHECK(commandable == (d.kind != CapabilityKind::Status));
        if (!commandable) {
            CHECK(d.axes == 0); // a status to read and a mode to set, nothing to fly
            continue;
        }
        CHECK((d.interactions & (kCancel | kStatus)) == (kCancel | kStatus));
        // a behaviour's parameters are heap data; a mode's setpoint is fixed-size and takes UPDATE (docs/vehicle-interface.md, 4.2)
        CHECK(((d.interactions & kUpdate) != 0) == (d.kind != CapabilityKind::Guidance || d.setpoint != SetpointKind::Behavior));
        const bool primary = (d.axes & kPrimaryAxes) != 0; // flown through the cascade, or the engines' thrust beside it
        CHECK(primary == (!Maker::isSupport(d) || d.id == "fsim.flight.engines"));
        CHECK(w.capabilityStatus(v, d.id).availability == Availability::Available);

        // NEW: pending, owning the capability's axes
        Command c;
        SupportCommand sc;
        const bool support = make.support(d, false, sc);
        CommandResult r;
        if (support) {
            r = w.submit(v, sc);
            if (r.reason == Reason::Unavailable && std::holds_alternative<GearCommand>(sc)) { // down above its placard speed: up, then
                sc = GearCommand{0.0};
                r = w.submit(v, sc);
            }
        } else {
            REQUIRE(make.cascade(d, false, c));
            r = submitMade(w, v, c, make);
        }
        REQUIRE(r.accepted());
        CHECK(r.activity == activityId(v, ++serial));
        const ActivityRecord* a = w.activity(r.activity);
        REQUIRE(a != nullptr);
        CHECK(a->state == ActivityState::Pending);
        CHECK(a->capability == i);
        CHECK(a->source == Source::Policy);
        CHECK((a->axes & d.axes) == d.axes);
        CHECK(a->startTime == w.simTime());
        CHECK(std::isnan(a->endTime));

        // a world step flies it: active, or done if it gets somewhere
        w.step();
        a = w.activity(r.activity);
        REQUIRE(a != nullptr);
        if (d.persistence == Persistence::Terminating) CHECK((a->state == ActivityState::Active || (a->state == ActivityState::Completed && a->reason == Reason::GoalReached)));
        else CHECK(a->state == ActivityState::Active);

        // UPDATE: the same command type where the capability takes updates, another type never
        const bool live = a->live();
        const Reason updated = support ? w.update(r.activity, sc).reason : w.update(r.activity, c).reason;
        const Reason other = support ? w.update(r.activity, Command(VelocityCommand{})).reason : w.update(r.activity, otherType(c)).reason;
        const bool updatable = (d.interactions & kUpdate) != 0;
        if (live) {
            CHECK(updated == (updatable ? Reason::None : Reason::NotUpdatable));
            CHECK(other == (updatable ? Reason::WrongCommandType : Reason::NotUpdatable));
        } else {
            CHECK(updated == Reason::ActivityEnded);
            CHECK(other == Reason::ActivityEnded);
        }

        // CANCEL: canceled on request, now; then it is over for good
        const CommandResult x = w.cancel(r.activity);
        if (live) {
            CHECK(x.status == CommandStatus::Canceled);
            a = w.activity(r.activity);
            REQUIRE(a != nullptr);
            CHECK(a->state == ActivityState::Canceled);
            CHECK(a->reason == Reason::Requested);
            CHECK(a->by == 0);
            CHECK(a->endTime == w.simTime());
        } else {
            CHECK(x.reason == Reason::ActivityEnded);
        }
        CHECK(w.cancel(r.activity).reason == Reason::ActivityEnded);
        CHECK((support ? w.update(r.activity, sc) : w.update(r.activity, c)).reason == Reason::ActivityEnded);
    }
    CHECK(w.cancel(activityId(v, serial + 1)).reason == Reason::UnknownActivity);
}

// --- random sequences -----------------------------------------------------------------------------

enum class Op { New, Update, Cancel, Legacy, Step, Reset, Default, Retarget, Authority, Precedence };

const char* opName(Op op) {
    switch (op) {
    case Op::New: return "new";
    case Op::Update: return "update";
    case Op::Cancel: return "cancel";
    case Op::Legacy: return "command";
    case Op::Step: return "step";
    case Op::Reset: return "reset";
    case Op::Default: return "default";
    case Op::Authority: return "authority";
    case Op::Precedence: return "precedence";
    default: return "retarget";
    }
}

/// What the rules say a vehicle's authority is (docs/vehicle-interface.md, 6
/// and 7.2), kept beside the World's and compared with it after each operation.
struct AuthorityModel {
    ControlMode mode = ControlMode::Open;
    std::vector<ControlStatus> control;       ///< per capability
    std::vector<CapabilityStatus> restricted; ///< per capability: the platform's
    /// Why the policy may not command capability `c` now, as the rules have it; None if it may.
    Reason refuses(std::size_t c) const {
        if (mode == ControlMode::Granted && !control[c].granted) return Reason::NotGranted;
        if (restricted[c].availability != Availability::Available) return restricted[c].reason;
        return Reason::None;
    }
};

bool among(Reason r, std::initializer_list<Reason> allowed) { return std::find(allowed.begin(), allowed.end(), r) != allowed.end(); }

/// What an operation did, for the rules it must have kept.
struct Done {
    Op op = Op::Step;
    CommandResult result;       ///< NEW, UPDATE, CANCEL
    CommandOptions options;     ///< NEW
    ActivityId addressed = 0;   ///< UPDATE, CANCEL
    Source caller = Source::Policy; ///< UPDATE, CANCEL: the source the call declares
    ControlMode mode = ControlMode::Open; ///< the vehicle's, as the operation was made
    bool accepted = false;      ///< the existing entry point's answer
    std::size_t capability = 0; ///< NEW, the existing entry point, an authority call: the capability it is about
    Reason refused = Reason::None; ///< NEW, the existing entry point: what the authority model refuses a policy
    std::string authority;      ///< an authority call: which ("mode", "request", ...)
    Reason answer = Reason::None; ///< a request's answer
    std::map<ActivityId, Reason> ends; ///< an authority call: the policy's activities it must end, and why
    double start = 0.0;         ///< the simulation time before it
    double now = 0.0;           ///< the simulation time after it
    double stepS = 0.0;         ///< a world step
    bool diverged = false;
};

/// The lifecycle's rules (docs/control-architecture.md, 10.1-10.6) between
/// the records before an operation and after it. Returns the highest serial seen.
std::uint32_t keepsTheRules(session::World& w, std::uint32_t v, const std::map<ActivityId, ActivityRecord>& before,
                            const std::map<ActivityId, ActivityRecord>& after, const Done& done, std::uint32_t lastSerial,
                            std::map<std::string, int>& seen) {
    const auto& caps = w.capabilities(v);
    INFO("after " << opName(done.op) << " at t=" << done.now);
    const bool step = done.op == Op::Step, reset = done.op == Op::Reset;
    const bool created = (done.op == Op::New && done.result.accepted()) || (done.op == Op::Legacy && done.accepted);

    // the answer: one of the reasons its operation may give, and true to the records it was given
    auto was = [&](ActivityId id) -> const ActivityRecord* {
        const auto it = before.find(id);
        return it == before.end() ? nullptr : &it->second;
    };
    if (done.op == Op::New || done.op == Op::Update || done.op == Op::Cancel) {
        const bool ok = done.result.accepted() || done.result.status == CommandStatus::Canceled;
        ++seen[std::string(opName(done.op)) + ":" + (ok ? "done" : reasonName(done.result.reason))];
        if (done.result.flags & kClamped) ++seen["clamped"];
    }
    if (done.op == Op::New) {
        if (done.result.accepted()) CHECK(done.result.reason == Reason::None);
        else CHECK(among(done.result.reason, {Reason::UnknownCapability, Reason::Unavailable, Reason::VersionUnsupported, Reason::InvalidParameter,
                                              Reason::OutOfRange, Reason::InvalidAxes, Reason::AuthorityHeld, Reason::ControllerNotAxisAware,
                                              Reason::PerformanceLimit, Reason::InvalidWaypoint, Reason::InvalidCurve, Reason::NotGranted,
                                              Reason::CollisionAvoidance, Reason::Restricted, Reason::Diverged,
                                              // what the vehicle cannot do at all, and the flight phase (docs/flight-autonomy.md, 4.3)
                                              Reason::NotSupported, Reason::NotImplemented, Reason::OnGround,
                                              // a precedence override from a policy, a window it cannot meet, no room to wait (4.9)
                                              Reason::NotAllowed, Reason::TimeConstraint, Reason::QueueFull}));
        // a policy's precedence override is refused; one that waits was accepted to (4.9)
        if (done.options.source == Source::Policy && done.options.precedenceOverride != kNoPrecedenceOverride) CHECK_FALSE(done.result.accepted());
        if (done.result.reason == Reason::NotAllowed) CHECK(done.options.precedenceOverride != kNoPrecedenceOverride);
        if (done.result.flags & kDeferred) {
            CHECK(done.result.accepted());
            ++seen["new:deferred"];
        }
        // a policy is refused what the rules refuse it, and nothing else is refused for its authority (6.1, 7.2)
        const Reason refused = done.options.source == Source::Policy ? done.refused : Reason::None;
        if (refused != Reason::None) CHECK(done.result.reason == refused);
        else CHECK_FALSE(among(done.result.reason, {Reason::NotGranted, Reason::CollisionAvoidance, Reason::Restricted}));
        if ((done.result.flags & kClamped) != 0) CHECK(done.options.range == RangePolicy::Clamp);
        if (done.result.reason == Reason::AuthorityHeld) {
            const ActivityRecord* holder = was(done.result.other);
            REQUIRE(holder != nullptr);
            CHECK(holder->live());
            CHECK(holder->source > done.options.source); // only a higher source holds its axes against a NEW...
            CHECK(done.options.interrupt);               // ...that would interrupt it (4.9: one that would not waits)
        }
    }
    if (done.op == Op::Update || done.op == Op::Cancel) {
        const ActivityRecord* target = was(done.addressed);
        const bool ok = done.op == Op::Update ? done.result.accepted() : done.result.status == CommandStatus::Canceled;
        if (!target) CHECK(done.result.reason == Reason::UnknownActivity);
        else if (!target->live()) CHECK(done.result.reason == Reason::ActivityEnded);
        else if (done.mode == ControlMode::Granted && done.caller < target->source) { // FA stays the primary controller (6.1)
            CHECK(done.result.reason == Reason::AuthorityHeld);
            CHECK(done.result.other == done.addressed);
        } else if (done.op == Op::Cancel) CHECK(ok);
        else if (!ok)
            CHECK(among(done.result.reason, {Reason::NotUpdatable, Reason::WrongCommandType, Reason::InvalidParameter, Reason::OutOfRange, Reason::PerformanceLimit,
                                             Reason::InvalidWaypoint, Reason::InvalidCurve,
                                             Reason::NotSupported})); // a field the aircraft has nothing for (docs/flight-autonomy.md, 4.3)
    }

    if (done.op == Op::Legacy && done.refused != Reason::None) CHECK_FALSE(done.accepted); // (the existing entry points gated the same way)
    if (done.op == Op::Authority) {
        ++seen[done.authority + (done.authority == "request" ? std::string(":") + reasonName(done.answer) : std::string())];
        for (const auto& [id, why] : done.ends) { // exactly the policy's activities the rules end, as they say
            const auto it = after.find(id);
            REQUIRE(it != after.end());
            CHECK(it->second.state == ActivityState::Canceled);
            CHECK(it->second.reason == why);
        }
    }

    // what waited and started in this operation (4.9): it may take axes from, and end, what flies
    bool startedNow = false;
    for (const auto& [id, r] : after) // (flying, or ended since: an activity that has started waits no more)
        if (const ActivityRecord* old = was(id); old && old->waiting != ActivityWait::None && r.waiting == ActivityWait::None) startedNow = true;
    for (const auto& [id, r] : after) // (a new one, taken at once by what started)
        if (!was(id) && r.state == ActivityState::Canceled && r.reason == Reason::Preempted) startedNow = true;

    // every record: its state and its reason agree, and so does its end
    std::uint32_t highest = lastSerial;
    std::uint32_t newRecords = 0;
    for (const auto& [id, r] : after) {
        REQUIRE(r.capability < caps.size());
        INFO("activity " << serialOf(id) << " " << caps[r.capability].id << " " << activityStateName(r.state) << " " << reasonName(r.reason));
        CHECK(activityVehicle(id) == v);
        CHECK(r.vehicle == v);
        CHECK(r.live() == std::isnan(r.endTime));
        // progress, where a behaviour gives it (docs/vehicle-interface.md, 5.3), within its bounds
        if (r.progress.segments) CHECK(r.progress.segment < r.progress.segments);
        if (!std::isnan(r.progress.percent)) CHECK((r.progress.percent >= 0.0 && r.progress.percent <= 100.0));
        if (!r.live()) CHECK(r.endTime >= r.startTime);
        const TimeWindow& window = r.window;
        switch (r.state) {
        case ActivityState::Pending:
        case ActivityState::Active: CHECK((r.reason == Reason::None && r.by == 0)); break;
        case ActivityState::Completed:
            CHECK((r.reason == Reason::GoalReached && r.by == 0));
            // a persistent activity is done only when its end window closes; none is done before a critical one opens (4.9)
            if (caps[r.capability].persistence == Persistence::Persistent) CHECK(r.endTime >= window.endNotAfter);
            if (window.endCritical()) CHECK_FALSE(r.endTime < window.endNotBefore);
            break;
        case ActivityState::Canceled:
            CHECK(among(r.reason, {Reason::Requested, Reason::Preempted, Reason::Released, Reason::Revoked, Reason::NotGranted, Reason::CollisionAvoidance,
                                   Reason::Restricted}));
            CHECK((r.by != 0) == (r.reason == Reason::Preempted));
            break;
        case ActivityState::Failed:
            CHECK(r.by == 0);
            if (r.reason == Reason::TargetLost) CHECK(caps[r.capability].needsTarget);
            if (r.reason == Reason::TimeConstraint) { // a window it had to meet, missed: waiting past it, or flying
                const bool waited = r.waiting != ActivityWait::None;
                if (waited) CHECK((r.endTime >= window.endNotAfter || (window.startCritical() && r.endTime > window.startNotAfter)));
                else CHECK((window.endCritical() && (r.endTime >= window.endNotAfter || r.endTime < window.endNotBefore)));
            } else if (!among(r.reason, {Reason::TargetLost, Reason::BehaviorFailed, Reason::CapabilityLost, Reason::Diverged})) {
                // else only one that waited, as it would start: what its NEW's checks say from where the aircraft is then
                CHECK(r.waiting != ActivityWait::None);
                CHECK(among(r.reason, {Reason::OnGround, Reason::InvalidParameter, Reason::OutOfRange, Reason::PerformanceLimit, Reason::InvalidWaypoint,
                                       Reason::InvalidCurve, Reason::NotSupported, Reason::ControllerNotAxisAware, Reason::Unavailable}));
            }
            break;
        }
        const ActivityRecord* previous = was(id);
        if (r.reason == Reason::Preempted && (previous == nullptr || previous->live())) { // (judged as it ends)
            const auto it = after.find(r.by);
            const ActivityRecord* waited = was(r.by);
            // by a newer activity, or one that waited and started now...
            CHECK((serialOf(r.by) > serialOf(id) || (waited && waited->waiting != ActivityWait::None)));
            if (it != after.end()) {
                CHECK(it->second.source >= r.source);                                 // ...of its own or a higher source...
                if (r.endTime == done.now) CHECK(standing(it->second, r) == Standing::Takes); // ...that the rules let take it (4.9)
            }
        }
        const ActivityRecord* old = was(id);
        if (!old) {
            // a new activity: only a NEW makes one, pending, with the next serial
            ++newRecords;
            CHECK(created);
            CHECK(serialOf(id) == lastSerial + 1);
            // pending - waiting to start if its NEW said so - unless what waited started at once and took it (4.9)
            CHECK((r.state == ActivityState::Pending || (r.state == ActivityState::Canceled && r.reason == Reason::Preempted)));
            CHECK(r.startTime == done.now);
            CHECK(r.source == (done.op == Op::New ? done.options.source : Source::Policy));
            if (done.op == Op::New) CHECK(id == done.result.activity);
            CHECK((r.waiting != ActivityWait::None) == (done.op == Op::New && (done.result.flags & kDeferred) != 0));
            if (done.op == Op::New) CHECK((r.rank == done.options.rank && r.interrupt == done.options.interrupt));
            highest = std::max(highest, serialOf(id));
            continue;
        }
        CHECK((old->capability == r.capability && old->source == r.source && old->startTime == r.startTime));
        if (!old->live()) {
            CHECK(sameRecord(*old, r)); // an ended activity never changes
            continue;
        }
        if (old->state != r.state) {
            ++seen[std::string(activityStateName(old->state)) + "->" + activityStateName(r.state)]; // the edge
            if (!r.live()) ++seen[std::string(activityStateName(r.state)) + ":" + reasonName(r.reason)]; // and why it ended
        }
        if (r.live()) {
            // live on: active once flown, pending again only after a reset
            if (old->state != r.state) {
                if (r.state == ActivityState::Active) CHECK(step);
                else CHECK(reset);
            }
            // waiting until it starts, never again after (4.9)
            if (old->waiting == ActivityWait::None) CHECK(r.waiting == ActivityWait::None);
            if (old->waiting != ActivityWait::None && r.waiting == ActivityWait::None) ++seen["started"];
            CHECK((r.axes & ~old->axes) == 0); // it never gains an axis...
            if (r.axes != old->axes) {
                CHECK((created || startedNow)); // ...and loses one only to a NEW (or what waited, starting), and then only a support axis
                CHECK((r.axes & kPrimaryAxes) == (old->axes & kPrimaryAxes));
            }
        } else if (r.state == ActivityState::Canceled && r.reason == Reason::Requested) {
            CHECK(done.op == Op::Cancel);
            CHECK(done.addressed == id);
            CHECK(r.endTime == done.now);
        } else if (r.state == ActivityState::Canceled && done.op == Op::Authority) {
            if (r.reason == Reason::Preempted) CHECK(startedNow); // (what waited, starting as what the rules end freed its axes)
            else CHECK(done.ends.count(id) == 1); // one the rules end: the policy's, of the capability (or with no grant)
            if (r.reason != Reason::Preempted) CHECK(r.source == Source::Policy);
            CHECK(r.endTime == done.now);
        } else if (r.state == ActivityState::Canceled) {
            // preempted by the activity just made, or by what waited and started now (in a step, at one of its steps' ends)
            CHECK((created || startedNow));
            if (!startedNow) CHECK(r.by == activityId(v, lastSerial + 1));
            if (step) {
                CHECK((r.endTime > done.start && r.endTime <= done.now));
                const double steps = (r.endTime - done.start) / done.stepS;
                CHECK(std::abs(steps - std::round(steps)) < 1e-6);
            } else {
                CHECK(r.endTime == done.now);
            }
        } else if (old->waiting != ActivityWait::None && r.state == ActivityState::Failed && r.endTime == done.now) {
            // one that waited: failed by a window it could no longer meet, or as it would start (the scheduler's, after any operation)
            ++seen["failed:waiting"];
        } else {
            // completed or failed: something a world step did, and it ended at that step's end
            CHECK(step);
            CHECK((r.endTime > done.start && r.endTime <= done.now));
            const double steps = (r.endTime - done.start) / done.stepS;
            CHECK(std::abs(steps - std::round(steps)) < 1e-6);
            if (r.reason == Reason::Diverged) CHECK(done.diverged);
            if (caps[r.capability].persistence == Persistence::Persistent && r.state == ActivityState::Completed) ++seen["completed:window"];
        }
        if (!step && !reset) CHECK((r.constraints == old->constraints && r.constraintsSeen == old->constraintsSeen));
    }
    CHECK(newRecords <= 1);
    if (done.op == Op::New && done.result.accepted()) CHECK(newRecords == 1);
    for (const auto& [id, r] : before)
        if (!after.count(id)) CHECK_FALSE(r.live()); // only the oldest ended ones leave the records

    // a step flies every live activity: none still pending but what waits to start
    if (step)
        for (const auto& [id, r] : after) {
            const ActivityRecord* old = was(id);
            CHECK((r.state != ActivityState::Pending || r.waiting != ActivityWait::None || (old && old->waiting != ActivityWait::None)));
        }

    // what waits (4.9): pending, within the windows it can still meet; scheduled until its start window opens;
    // queued behind something on its axes it may not take, which it names - nothing waits that could start
    auto flying = [](const ActivityRecord& r) { return r.live() && r.waiting == ActivityWait::None; };
    for (const auto& [id, r] : after) {
        if (!r.live()) continue;
        const TimeWindow& t = r.window;
        if (r.waiting == ActivityWait::None) {
            // flying past its end window: only a terminating activity whose end is not critical (late, it goes on)
            if (done.now >= t.endNotAfter) CHECK((caps[r.capability].persistence == Persistence::Terminating && !t.endCritical()));
            continue;
        }
        INFO("waiting " << serialOf(id) << " " << activityWaitName(r.waiting));
        CHECK(r.state == ActivityState::Pending);
        CHECK_FALSE(done.now >= t.endNotAfter);
        CHECK_FALSE((t.startCritical() && done.now > t.startNotAfter));
        if (r.waiting == ActivityWait::Scheduled) {
            CHECK(done.now < t.startNotBefore);
            CHECK(r.waitingFor == 0);
            ++seen["scheduled"];
            continue;
        }
        CHECK_FALSE(done.now < t.startNotBefore);
        bool blocked = false, named = false;
        for (const auto& [hid, h] : after)
            if (flying(h) && (h.axes & r.axes) && standing(r, h) != Standing::Takes) blocked = true, named = named || hid == r.waitingFor;
        CHECK(blocked);
        CHECK(named);
        ++seen["queued"];
    }

    // each axis has at most one live owner, and the runtime flies it from where the host put it
    const RuntimeConfig& config = w.controls(v)->config();
    AxisMask owned = 0;
    for (const auto& [id, r] : after) {
        if (!flying(r)) continue;
        CHECK((owned & r.axes) == 0);
        owned = static_cast<AxisMask>(owned | r.axes);
        const CapabilityDescriptor& d = caps[r.capability];
        for (std::size_t a = 0; a < kAxisCount; ++a) {
            if (!(r.axes & (1u << a))) continue;
            if (d.id == "fsim.flight.engines") CHECK(config.owner[a] == RuntimeConfig::kEngines);
            else if (Maker::isSupport(d)) CHECK(config.owner[a] == RuntimeConfig::kSupport);
            else CHECK(config.owner[a] < kSlotCount);
        }
    }
    return highest;
}

/// A seeded random sequence of operations on one aircraft, each checked
/// against the rules; returns what was answered and flown, to compare runs.
/// Now and then it plays the ends chance rarely reaches through the same
/// rules: a route done at once, and a pursuit whose target goes.
std::vector<double> randomSequence(const Aircraft& aircraft, std::uint64_t seed, int operations, std::map<std::string, int>& seen,
                                   int leastActivities = 51) {
    session::World w(options(std::string("conformance-") + aircraft.family));
    std::uint32_t target = w.createVehicle(spec("target", aircraft.type, aircraft.altitudeM, aircraft.tasMs, 0.05));
    const auto v = w.createVehicle(spec("subject", aircraft.type, aircraft.altitudeM, aircraft.tasMs));
    REQUIRE(v != 0);
    REQUIRE(familyOf(w, v) == aircraft.family);
    Maker make(w, v, seed);
    make.target = target;
    std::vector<double> transcript;
    std::vector<ActivityId> issued;
    std::uint32_t lastSerial = 0;
    auto before = snapshot(w, v);
    struct Planned {
        Op op;
        Command command;
        unsigned steps = 1;
    };
    std::deque<Planned> plan;
    AuthorityModel authority;
    std::mt19937_64 callers(seed ^ 0x9e3779b97f4a7c15ull); // who an UPDATE or a CANCEL says it is
    std::mt19937_64 validations(seed ^ 0x5851f42d4c957f2dull); // which NEWs are validated first (drawn apart, as the callers are)
    std::mt19937_64 schedules(seed ^ 0x2545f4914f6cdd1dull);   // ranks, windows and precedence (4.9; drawn apart likewise)
    auto chance = [&schedules](double p) { return std::uniform_real_distribution<double>(0.0, 1.0)(schedules) < p; };
    auto draw = [&schedules](int n) { return std::uniform_int_distribution<int>(0, n - 1)(schedules); };
    std::vector<std::uint32_t> precedences(w.capabilities(v).size(), 0); // the platform's, as the rules keep them
    authority.control.assign(w.capabilities(v).size(), ControlStatus{});
    authority.restricted.assign(w.capabilities(v).size(), CapabilityStatus{});

    for (int k = 0; k < operations; ++k) {
        if (k % 120 == 60) {
            BehaviorCommand route;
            route.id = "waypoints";
            route.points = {make.ahead(100.0)}; // inside its capture radius
            plan.push_back({Op::New, route});
            plan.push_back({Op::Step, {}, 1});
        } else if (k % 120 == 119) {
            BehaviorCommand pursue;
            pursue.id = "pursuit";
            pursue.target = target;
            plan.push_back({Op::New, pursue});
            plan.push_back({Op::Step, {}, 2});
            plan.push_back({Op::Retarget, {}});
            plan.push_back({Op::Step, {}, 1});
        }
        Done done;
        done.mode = authority.mode;
        done.start = w.simTime();
        done.stepS = w.dt() * w.frameSkip();
        const bool planned = !plan.empty();
        const Planned next = planned ? plan.front() : Planned{};
        if (planned) {
            plan.pop_front();
            done.op = next.op;
        } else {
            const double u = make.uniform(0.0, 1.0);
            done.op = u < 0.36 ? Op::New : u < 0.57 ? Op::Update : u < 0.65 ? Op::Cancel : u < 0.71 ? Op::Legacy
                    : u < 0.89 ? Op::Step : u < 0.91 ? Op::Reset : u < 0.94 ? Op::Default : u < 0.95 ? Op::Retarget : Op::Authority;
            if (chance(0.02)) done.op = Op::Precedence; // the platform sets a capability's precedence (4.9)
        }
        const auto& caps = w.capabilities(v);
        // a capability's command: flight, guidance or support
        auto commandFor = [&](const CapabilityDescriptor& d, Command& c, SupportCommand& sc) { return make.support(d, true, sc) ? 1 : (make.cascade(d, true, c) ? 0 : -1); };
        switch (done.op) {
        case Op::New: {
            if (planned) { // an operator's, so nothing holds it off
                done.options.source = Source::Override;
                done.result = w.submit(v, next.command, done.options);
                if (done.result.accepted()) issued.push_back(done.result.activity);
                break;
            }
            std::vector<std::size_t> commandable;
            for (std::size_t i = 0; i < caps.size(); ++i)
                if (caps[i].interactions & kCommand) commandable.push_back(i);
            done.capability = commandable[make.pick(commandable.size())];
            const CapabilityDescriptor d = caps[done.capability];
            done.refused = authority.refuses(done.capability);
            const double s = make.uniform(0.0, 1.0);
            done.options.source = s < 0.55 ? Source::Policy : s < 0.85 ? Source::Autopilot : Source::Override;
            const double r = make.uniform(0.0, 1.0);
            done.options.range = r < 0.55 ? RangePolicy::Clamp : r < 0.85 ? RangePolicy::Reject : RangePolicy::None;
            if (make.chance(0.4)) {
                auto both = [](AxisMask a, AxisMask b) { return static_cast<AxisMask>(a | b); };
                const AxisMask masks[] = {kLateral,
                                          axisBit(Axis::Pitch),
                                          axisBit(Axis::Thrust),
                                          both(axisBit(Axis::Pitch), axisBit(Axis::Thrust)),
                                          both(kLateral, axisBit(Axis::Thrust)),
                                          axisBit(Axis::Roll),
                                          axisBit(Axis::Yaw),
                                          kPrimaryAxes,
                                          both(kPrimaryAxes, axisBit(Axis::Flaps)),
                                          axisBit(Axis::Gear),
                                          both(axisBit(Axis::Roll), axisBit(Axis::Pitch))};
                done.options.axes = make.chance(0.9) ? masks[make.pick(std::size(masks))] : static_cast<AxisMask>(make.pick(1u << 10));
            }
            // ranked, not interrupting, overriding its precedence, windowed, now and then (4.9)
            if (chance(0.45)) {
                done.options.rank = {static_cast<std::uint16_t>(draw(4)), static_cast<std::uint16_t>(draw(4))};
                done.options.interrupt = !chance(0.4);
                if (chance(done.options.source == Source::Policy ? 0.05 : 0.2)) done.options.precedenceOverride = static_cast<std::uint32_t>(draw(3));
                if (chance(0.5)) {
                    TimeWindow& t = done.options.window;
                    const double now = w.simTime(), step = done.stepS;
                    if (chance(0.5)) t.startNotBefore = now + step * (1 + draw(8));
                    if (chance(0.3)) t.startNotAfter = (std::isnan(t.startNotBefore) ? now : t.startNotBefore) + step * draw(6);
                    if (chance(0.5)) t.endNotAfter = now + step * (2 + draw(40));
                    if (chance(0.2)) t.endNotBefore = now + step * draw(20);
                    t.criticality = static_cast<TimeCriticality>(draw(4));
                    if (chance(0.05)) std::swap(t.startNotBefore, t.endNotAfter); // (out of order...)
                    if (chance(0.04)) t.endNotAfter = now - step * draw(3);         // (...or already over)
                }
            }
            Command c;
            SupportCommand sc;
            const int kind = commandFor(d, c, sc);
            REQUIRE(kind >= 0);
            // now and then validated first (docs/flight-autonomy.md, 4.8): nothing changes, and the NEW is answered alike
            const bool validating = std::uniform_real_distribution<double>(0.0, 1.0)(validations) < 0.25;
            CommandResult checked;
            if (validating) {
                CommandOptions check = done.options;
                check.validateOnly = true;
                checked = kind == 1 ? w.submit(v, sc, check) : submitMade(w, v, c, make, check);
                const auto records = snapshot(w, v);
                CHECK(records.size() == before.size());
                for (const auto& [id, record] : records)
                    if (const auto it = before.find(id); it != before.end()) CHECK(sameRecord(it->second, record));
                CHECK(checked.activity == 0);
                CHECK_FALSE(checked.newActivity);
            }
            done.result = kind == 1 ? w.submit(v, sc, done.options) : submitMade(w, v, c, make, done.options);
            if (validating) {
                ++seen[checked.status == CommandStatus::Valid ? "validate:valid" : "validate:refused"];
                CHECK((checked.status == CommandStatus::Valid) == done.result.accepted());
                CHECK(checked.reason == done.result.reason);
                CHECK(checked.flags == done.result.flags);
            }
            if (done.result.accepted()) CHECK(done.result.newActivity);
            if (done.result.accepted()) issued.push_back(done.result.activity);
            if (done.result.accepted()) { // its capability's precedence: its override, else the platform's (4.9)
                const ActivityRecord* made = w.activity(done.result.activity);
                REQUIRE(made != nullptr);
                const std::uint32_t expected = done.options.precedenceOverride != kNoPrecedenceOverride ? done.options.precedenceOverride
                                                                                                         : precedences[done.capability];
                if (made->live()) CHECK(made->precedence == expected);
            }
            break;
        }
        case Op::Precedence: {
            std::vector<std::size_t> commandable;
            for (std::size_t i = 0; i < caps.size(); ++i)
                if (caps[i].interactions & kCommand) commandable.push_back(i);
            done.capability = commandable[static_cast<std::size_t>(draw(static_cast<int>(commandable.size())))];
            const auto p = static_cast<std::uint32_t>(draw(4));
            precedences[done.capability] = p;
            CHECK(w.setCapabilityPrecedence(v, caps[done.capability].id, p) == Reason::None);
            CHECK(w.capabilityPrecedence(v, caps[done.capability].id) == p);
            ++seen["precedence"];
            break;
        }
        case Op::Update:
        case Op::Cancel: {
            // a live activity mostly; else one that ended, one never made, or another vehicle's
            std::vector<ActivityId> live;
            for (const auto& [id, r] : before)
                if (r.live()) live.push_back(id);
            const double which = make.uniform(0.0, 1.0);
            done.addressed = which < 0.5 && !live.empty()   ? live[make.pick(live.size())]
                             : which < 0.8 && !issued.empty() ? issued[make.pick(issued.size())]
                             : which < 0.9                    ? activityId(v, lastSerial + 1 + static_cast<std::uint32_t>(make.pick(50)))
                                                              : activityId(v + 1000, 1);
            // the source the call declares, the policy's mostly (drawn apart: the sequence is as it was without it)
            const double from = std::uniform_real_distribution<double>(0.0, 1.0)(callers);
            done.caller = from < 0.7 ? Source::Policy : from < 0.85 ? Source::Autopilot : Source::Override;
            if (done.op == Op::Cancel) {
                done.result = w.cancel(done.caller, done.addressed);
                break;
            }
            // its own capability's command mostly, another now and then
            const ActivityRecord* r = w.activity(done.addressed);
            const CapabilityDescriptor& d = caps[r && make.chance(0.7) ? r->capability : make.pick(caps.size())];
            Command c;
            SupportCommand sc;
            const int kind = commandFor(d, c, sc);
            done.result = kind == 1   ? w.update(done.caller, done.addressed, sc)
                          : kind == 0 ? updateMade(w, done.addressed, c, make, make.chance(0.6), done.caller)
                                      : w.update(done.caller, done.addressed, Command(VelocityCommand{}));
            break;
        }
        case Op::Legacy: {
            std::vector<std::size_t> cascade;
            for (std::size_t i = 0; i < caps.size(); ++i)
                if ((caps[i].kind == CapabilityKind::Flight && !Maker::isSupport(caps[i])) || (caps[i].kind == CapabilityKind::Guidance && make.chance(0.15)))
                    cascade.push_back(i);
            Command c;
            done.capability = cascade[make.pick(cascade.size())];
            done.refused = authority.refuses(done.capability);
            REQUIRE(make.cascade(caps[done.capability], true, c));
            done.accepted = w.command(v, c);
            break;
        }
        case Op::Authority: {
            // one of the calls, about a capability it can command mostly; the rules' answer and ends worked out first
            std::vector<std::size_t> commandable;
            for (std::size_t i = 0; i < caps.size(); ++i)
                if (caps[i].interactions & kCommand) commandable.push_back(i);
            const double a = make.uniform(0.0, 1.0);
            // what a release, a revocation or a refusal ends: mostly a capability the policy flies now
            std::vector<std::size_t> flown;
            for (const auto& [id, r] : before)
                if (r.live() && r.source == Source::Policy) flown.push_back(r.capability);
            const bool ending = a >= 0.55 && a < 0.87 && !flown.empty() && make.chance(0.7);
            const std::size_t c = done.capability = ending ? flown[make.pick(flown.size())] : commandable[make.pick(commandable.size())];
            auto endingPolicy = [&](auto matches, Reason why) {
                for (const auto& [id, r] : before)
                    if (r.live() && r.source == Source::Policy && matches(r)) done.ends[id] = why;
            };
            const std::string& id = caps[c].id;
            if (a < 0.2) {
                done.authority = "mode";
                const ControlMode mode = make.chance(0.6) ? ControlMode::Granted : ControlMode::Open;
                if (mode == ControlMode::Granted && authority.mode != mode)
                    endingPolicy([&](const ActivityRecord& r) { return !authority.control[r.capability].granted; }, Reason::NotGranted);
                authority.mode = mode;
                CHECK(w.setControlMode(v, mode) == Reason::None);
            } else if (a < 0.55) {
                done.authority = "request";
                const CapabilityStatus own = w.vehicleState(v)->diverged ? CapabilityStatus{Availability::TemporarilyUnavailable, Reason::Diverged}
                                                                          : authority.restricted[c];
                const Reason expected = !authority.control[c].allowed ? Reason::NotAllowed
                                        : own.availability != Availability::Available ? own.reason
                                                                                      : Reason::None;
                if (expected == Reason::None) authority.control[c].granted = true;
                done.answer = w.requestControl(v, id);
                CHECK(done.answer == expected);
            } else if (a < 0.65) {
                done.authority = "release";
                endingPolicy([&](const ActivityRecord& r) { return r.capability == c; }, Reason::Released);
                authority.control[c].granted = false;
                CHECK(w.releaseControl(v, id) == Reason::None);
            } else if (a < 0.75) {
                done.authority = "revoke";
                const Reason why = make.chance(0.5) ? Reason::Revoked : Reason::CollisionAvoidance;
                endingPolicy([&](const ActivityRecord& r) { return r.capability == c; }, why);
                authority.control[c].granted = false;
                CHECK(w.revokeControl(v, id, why) == Reason::None);
            } else if (a < 0.87) {
                done.authority = "allow";
                const bool allowed = make.chance(0.6);
                if (!allowed && authority.control[c].granted) endingPolicy([&](const ActivityRecord& r) { return r.capability == c; }, Reason::Revoked);
                if (!allowed) authority.control[c].granted = false;
                authority.control[c].allowed = allowed;
                CHECK(w.setAllowed(v, id, allowed) == Reason::None);
            } else {
                done.authority = "restrict";
                const bool lift = make.chance(0.5);
                const Reason why = make.chance(0.5) ? Reason::Restricted : Reason::CollisionAvoidance;
                authority.restricted[c] = lift ? CapabilityStatus{} : CapabilityStatus{Availability::TemporarilyUnavailable, why};
                CHECK(w.setAvailability(v, id, lift ? Availability::Available : Availability::TemporarilyUnavailable, why) == Reason::None);
            }
            break;
        }
        case Op::Step: {
            w.step(planned ? next.steps : 1 + static_cast<unsigned>(make.pick(6)));
            const auto& s = *w.vehicleState(v);
            done.diverged = s.diverged != 0;
            const auto& in = *w.inputs(v);
            if (!done.diverged)
                CHECK((std::isfinite(s.altitudeMslM) && std::isfinite(in.aileron) && std::isfinite(in.elevator) && std::isfinite(in.throttle[0])));
            for (const double x : {s.latitudeRad, s.longitudeRad, s.altitudeMslM, in.elevator, in.throttle[0]})
                transcript.push_back(std::isnan(x) ? -1e300 : x); // a diverged flight compares too
            break;
        }
        case Op::Reset: REQUIRE(w.resetVehicle(v)); break;
        case Op::Default: {
            const Reason r = w.setVehicleDefault(v, make.chance(0.5) ? VehicleDefault::Hold : VehicleDefault::Neutral);
            CHECK(among(r, {Reason::None, Reason::ControllerNotAxisAware}));
            break;
        }
        case Op::Retarget: // the vehicle it follows goes, and another comes
            REQUIRE(w.removeVehicle(target));
            target = w.createVehicle(spec("target-" + std::to_string(k), aircraft.type, aircraft.altitudeM, aircraft.tasMs, 0.05));
            REQUIRE(target != 0);
            make.target = target;
            break;
        }
        done.now = w.simTime();
        const auto after = snapshot(w, v);
        lastSerial = keepsTheRules(w, v, before, after, done, lastSerial, seen);
        // the World's authority is what the rules say it is
        CHECK(w.controlMode(v) == authority.mode);
        const bool diverged = w.vehicleState(v)->diverged != 0;
        for (std::size_t i = 0; i < caps.size(); ++i) {
            if (!(caps[i].interactions & kCommand)) continue;
            INFO(caps[i].id);
            const ControlStatus st = w.controlStatus(v, caps[i].id);
            CHECK((st.allowed == authority.control[i].allowed && st.granted == authority.control[i].granted));
            if (!diverged) {
                const CapabilityStatus a = w.capabilityStatus(v, caps[i].id);
                CHECK((a.availability == authority.restricted[i].availability && a.reason == authority.restricted[i].reason));
            }
        }
        const bool changes = done.op == Op::Step || done.op == Op::Reset || (done.op == Op::New && done.result.accepted()) ||
                             (done.op == Op::Legacy && done.accepted) || (done.op == Op::Cancel && done.result.status == CommandStatus::Canceled) ||
                             (done.op == Op::Authority && !done.ends.empty()) || done.op == Op::Precedence;
        if (!changes) {
            // refused, an UPDATE, or nothing to do with the records: they are as they were
            CHECK(after.size() == before.size());
            for (const auto& [id, r] : after)
                if (const auto it = before.find(id); it != before.end()) CHECK(sameRecord(it->second, r));
        }
        before = after;
        transcript.insert(transcript.end(), {static_cast<double>(done.op), static_cast<double>(done.result.status), static_cast<double>(done.result.reason),
                                             static_cast<double>(serialOf(done.result.activity)), static_cast<double>(done.result.flags),
                                             static_cast<double>(done.accepted), static_cast<double>(done.answer),
                                             static_cast<double>(done.ends.size())});
    }
    INFO(aircraft.type << ": " << lastSerial << " activities, the last " << before.size() << " of them kept");
    CHECK(static_cast<int>(lastSerial) >= leastActivities);
    return transcript;
}

} // namespace

TEST_CASE("conformance: every capability of every aircraft the platform ships answers NEW, UPDATE and CANCEL as its descriptor says",
          "[conformance]") {
    const auto aircraft = shippedAircraft();
    REQUIRE(aircraft.size() > 10);
    std::set<std::string> families;
    for (const auto& type : aircraft) {
        INFO(type);
        session::World w(options("conformance-catalog"));
        // where the aircraft flies: its identified reference condition, or the c172x's
        double altitudeM = 2500.0, tasMs = 55.0;
        {
            const auto probe = w.createVehicle(spec("probe", type, 3000.0, 150.0));
            REQUIRE(probe != 0);
            const PlantSection& plant = w.profile(probe)->plant;
            if (plant.header.present() && std::isfinite(plant.tasMs)) altitudeM = plant.altitudeM, tasMs = plant.tasMs;
            families.insert(familyOf(w, probe));
            REQUIRE(w.removeVehicle(probe));
        }
        const auto target = w.createVehicle(spec("target", type, altitudeM, tasMs, 0.05));
        const auto v = w.createVehicle(spec("subject", type, altitudeM, tasMs));
        REQUIRE(v != 0);
        Maker make(w, v, 1);
        make.target = target;
        lifecycle(w, v, make);
    }
    CHECK(families == std::set<std::string>{"jsbsim.stock", "jsbsim.direct", "jsbsim.fbw", "jsbsim.helicopter", "jsbsim.multirotor"}); // every adapter
}

TEST_CASE("conformance: one aircraft per adapter keeps the lifecycle's rules through random sequences of every operation", "[conformance]") {
    std::map<std::string, int> all;
    for (const Aircraft& a : kAdapters) {
        INFO(a.type);
        std::map<std::string, int> seen, again;
        const auto first = randomSequence(a, 20260926, 600, seen);
        CHECK(first == randomSequence(a, 20260926, 600, again)); // the same calls, the same answers and the same flight
        // every edge of the state machine and every answer an operation can give, for every adapter: a walk
        // meets nearly all, and what one misses (any change to what a command draws moves it) another seeded
        // walk of the same adapter meets - the same every run, at most three more
        static const char* const kEvery[] = {"pending->active", "active->pending", "pending->canceled", "active->canceled", "canceled:preempted",
                                             "canceled:requested", "completed:goal_reached", "new:done", "new:authority_held",
                                             "new:invalid_axes", "new:out_of_range", "new:invalid_parameter", "update:done", "update:not_updatable",
                                             "update:wrong_command_type", "update:activity_ended", "update:unknown_activity", "cancel:done",
                                             "cancel:activity_ended", "cancel:unknown_activity", "clamped", "validate:valid", "validate:refused",
                                             "new:deferred", "scheduled", "queued", "started"};
        auto unmet = [&seen] { return std::any_of(std::begin(kEvery), std::end(kEvery), [&seen](const char* what) { return seen[what] == 0; }); };
        for (std::uint64_t seed = 20260926 + 100; unmet() && seed < 20260926 + 103; ++seed) randomSequence(a, seed, 600, seen, 0);
        for (const char* what : kEvery) {
            INFO(what);
            CHECK(seen[what] > 0);
        }
        for (const auto& [what, n] : seen) all[what] += n;
    }
    // The rarer answers, over every adapter's walks: a follower whose target goes, a route with a point it
    // cannot fly and a curve with a segment it cannot fly (docs/vehicle-interface.md, 5.1), an append where
    // the curve does not end, and every answer and end the grants give (6). A walk meets some of them once
    // or twice, and any change to what a command draws moves it: more seeded walks, the same every run,
    // until each has appeared (at most twelve more).
    static const char* const kRare[] = {"failed:target_lost", "new:invalid_waypoint", "new:invalid_curve", "update:invalid_curve",
                                        "new:not_granted", "request:none", "request:not_allowed", "canceled:released", "canceled:revoked",
                                        "canceled:not_granted", "canceled:collision_avoidance", "mode", "release", "revoke", "allow",
                                        "restrict", "update:authority_held", "cancel:authority_held",
                                        // ranks, queues and time windows (docs/flight-autonomy.md, 4.9)
                                        "precedence", "new:not_allowed", "new:time_constraint", "failed:time_constraint", "completed:window",
                                        "failed:waiting"};
    auto missing = [&all] { return std::any_of(std::begin(kRare), std::end(kRare), [&all](const char* what) { return all[what] == 0; }); };
    for (std::uint64_t seed = 20260927; missing() && seed < 20260927 + 12; ++seed)
        for (const Aircraft& a : kAdapters) {
            std::map<std::string, int> more;
            randomSequence(a, seed, 600, more, 0); // (only for the rare answers: no least number of activities)
            for (const auto& [what, n] : more) all[what] += n;
        }
    for (const char* what : kRare) {
        INFO(what);
        CHECK(all[what] > 0);
    }
    CHECK(all["new:restricted"] + all["new:collision_avoidance"] + all["request:restricted"] + all["request:collision_avoidance"] > 0);
    std::string summary;
    for (const auto& [what, n] : all) summary += what + " " + std::to_string(n) + "; ";
    INFO(summary);
    CHECK(all.size() > 20);
}
