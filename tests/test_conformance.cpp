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
                                  {"jsbsim:f16c", "jsbsim.fbw", 3000.0, 160.0}};

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
           a.constraintsSeen == b.constraintsSeen && a.startTime == b.startTime && ends;
}

std::uint32_t serialOf(ActivityId id) { return static_cast<std::uint32_t>(id & 0xFFFFFFFFu); }

/// Makes commands for a vehicle's capabilities, each value from the
/// capability's descriptor: its default, or - wild - drawn inside its range
/// about the flight, now and then outside it or left out.
class Maker {
public:
    Maker(session::World& w, std::uint32_t vehicle, std::uint64_t seed) : w_(w), vehicle_(vehicle), rng_(seed) {}

    std::uint32_t target = 0; ///< the vehicle a guidance capability that needs one follows

    double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng_); }
    bool chance(double p) { return uniform(0.0, 1.0) < p; }
    std::size_t pick(std::size_t n) { return std::uniform_int_distribution<std::size_t>(0, n - 1)(rng_); }

    /// A flight or guidance capability's command; false for the others.
    bool cascade(const CapabilityDescriptor& d, bool wild, Command& out) {
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
        double* fields[8];
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
        CHECK(((d.interactions & kUpdate) != 0) == (d.kind != CapabilityKind::Guidance)); // a behaviour's parameters are heap data
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
            r = w.submit(v, c);
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

enum class Op { New, Update, Cancel, Legacy, Step, Reset, Default, Retarget };

const char* opName(Op op) {
    switch (op) {
    case Op::New: return "new";
    case Op::Update: return "update";
    case Op::Cancel: return "cancel";
    case Op::Legacy: return "command";
    case Op::Step: return "step";
    case Op::Reset: return "reset";
    case Op::Default: return "default";
    default: return "retarget";
    }
}

bool among(Reason r, std::initializer_list<Reason> allowed) { return std::find(allowed.begin(), allowed.end(), r) != allowed.end(); }

/// What an operation did, for the rules it must have kept.
struct Done {
    Op op = Op::Step;
    CommandResult result;       ///< NEW, UPDATE, CANCEL
    CommandOptions options;     ///< NEW
    ActivityId addressed = 0;   ///< UPDATE, CANCEL
    bool accepted = false;      ///< the existing entry point's answer
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
                                              Reason::OutOfRange, Reason::InvalidAxes, Reason::AuthorityHeld, Reason::ControllerNotAxisAware}));
        if ((done.result.flags & kClamped) != 0) CHECK(done.options.range == RangePolicy::Clamp);
        if (done.result.reason == Reason::AuthorityHeld) {
            const ActivityRecord* holder = was(done.result.other);
            REQUIRE(holder != nullptr);
            CHECK(holder->live());
            CHECK(holder->source > done.options.source); // only a higher source holds its axes against a NEW
        }
    }
    if (done.op == Op::Update || done.op == Op::Cancel) {
        const ActivityRecord* target = was(done.addressed);
        const bool ok = done.op == Op::Update ? done.result.accepted() : done.result.status == CommandStatus::Canceled;
        if (!target) CHECK(done.result.reason == Reason::UnknownActivity);
        else if (!target->live()) CHECK(done.result.reason == Reason::ActivityEnded);
        else if (done.op == Op::Cancel) CHECK(ok);
        else if (!ok) CHECK(among(done.result.reason, {Reason::NotUpdatable, Reason::WrongCommandType, Reason::InvalidParameter, Reason::OutOfRange}));
    }

    // every record: its state and its reason agree, and so does its end
    std::uint32_t highest = lastSerial;
    std::uint32_t newRecords = 0;
    for (const auto& [id, r] : after) {
        REQUIRE(r.capability < caps.size());
        INFO("activity " << serialOf(id) << " " << caps[r.capability].id << " " << activityStateName(r.state) << " " << reasonName(r.reason));
        CHECK(activityVehicle(id) == v);
        CHECK(r.vehicle == v);
        CHECK(r.live() == std::isnan(r.endTime));
        if (!r.live()) CHECK(r.endTime >= r.startTime);
        switch (r.state) {
        case ActivityState::Pending:
        case ActivityState::Active: CHECK((r.reason == Reason::None && r.by == 0)); break;
        case ActivityState::Completed: CHECK((r.reason == Reason::GoalReached && r.by == 0)); break;
        case ActivityState::Canceled:
            CHECK(among(r.reason, {Reason::Requested, Reason::Preempted}));
            CHECK((r.by != 0) == (r.reason == Reason::Preempted));
            break;
        case ActivityState::Failed:
            CHECK(among(r.reason, {Reason::TargetLost, Reason::BehaviorFailed, Reason::CapabilityLost, Reason::Diverged}));
            CHECK(r.by == 0);
            if (r.reason == Reason::TargetLost) CHECK(caps[r.capability].needsTarget);
            break;
        }
        if (r.reason == Reason::Preempted) {
            CHECK(serialOf(r.by) > serialOf(id)); // by a newer activity...
            if (const auto it = after.find(r.by); it != after.end()) CHECK(it->second.source >= r.source); // ...of its own or a higher source
        }
        const ActivityRecord* old = was(id);
        if (!old) {
            // a new activity: only a NEW makes one, pending, with the next serial
            ++newRecords;
            CHECK(created);
            CHECK(serialOf(id) == lastSerial + 1);
            CHECK(r.state == ActivityState::Pending);
            CHECK(r.startTime == done.now);
            CHECK(r.source == (done.op == Op::New ? done.options.source : Source::Policy));
            if (done.op == Op::New) CHECK(id == done.result.activity);
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
            CHECK((r.axes & ~old->axes) == 0); // it never gains an axis...
            if (r.axes != old->axes) {
                CHECK(created); // ...and loses one only to a NEW, and then only a support axis
                CHECK((r.axes & kPrimaryAxes) == (old->axes & kPrimaryAxes));
            }
        } else if (r.state == ActivityState::Canceled && r.reason == Reason::Requested) {
            CHECK(done.op == Op::Cancel);
            CHECK(done.addressed == id);
            CHECK(r.endTime == done.now);
        } else if (r.state == ActivityState::Canceled) {
            CHECK(created); // preempted by the activity just made
            CHECK(r.by == activityId(v, lastSerial + 1));
            CHECK(r.endTime == done.now);
        } else {
            // completed or failed: something a world step did, and it ended at that step's end
            CHECK(step);
            CHECK((r.endTime > done.start && r.endTime <= done.now));
            const double steps = (r.endTime - done.start) / done.stepS;
            CHECK(std::abs(steps - std::round(steps)) < 1e-6);
            if (r.reason == Reason::Diverged) CHECK(done.diverged);
        }
        if (!step && !reset) CHECK((r.constraints == old->constraints && r.constraintsSeen == old->constraintsSeen));
    }
    CHECK(newRecords <= 1);
    if (done.op == Op::New && done.result.accepted()) CHECK(newRecords == 1);
    for (const auto& [id, r] : before)
        if (!after.count(id)) CHECK_FALSE(r.live()); // only the oldest ended ones leave the records

    // a step flies every live activity: none still pending
    if (step)
        for (const auto& [id, r] : after) CHECK(r.state != ActivityState::Pending);

    // each axis has at most one live owner, and the runtime flies it from where the host put it
    const RuntimeConfig& config = w.controls(v)->config();
    AxisMask owned = 0;
    for (const auto& [id, r] : after) {
        if (!r.live()) continue;
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
std::vector<double> randomSequence(const Aircraft& aircraft, std::uint64_t seed, int operations, std::map<std::string, int>& seen) {
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
        done.start = w.simTime();
        done.stepS = w.dt() * w.frameSkip();
        const bool planned = !plan.empty();
        const Planned next = planned ? plan.front() : Planned{};
        if (planned) {
            plan.pop_front();
            done.op = next.op;
        } else {
            const double u = make.uniform(0.0, 1.0);
            done.op = u < 0.38 ? Op::New : u < 0.60 ? Op::Update : u < 0.68 ? Op::Cancel : u < 0.74 ? Op::Legacy
                    : u < 0.93 ? Op::Step : u < 0.95 ? Op::Reset : u < 0.99 ? Op::Default : Op::Retarget;
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
            const CapabilityDescriptor d = caps[commandable[make.pick(commandable.size())]];
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
            Command c;
            SupportCommand sc;
            const int kind = commandFor(d, c, sc);
            REQUIRE(kind >= 0);
            done.result = kind == 1 ? w.submit(v, sc, done.options) : w.submit(v, c, done.options);
            if (done.result.accepted()) issued.push_back(done.result.activity);
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
            if (done.op == Op::Cancel) {
                done.result = w.cancel(done.addressed);
                break;
            }
            // its own capability's command mostly, another now and then
            const ActivityRecord* r = w.activity(done.addressed);
            const CapabilityDescriptor& d = caps[r && make.chance(0.7) ? r->capability : make.pick(caps.size())];
            Command c;
            SupportCommand sc;
            const int kind = commandFor(d, c, sc);
            done.result = kind == 1 ? w.update(done.addressed, sc) : kind == 0 ? w.update(done.addressed, c)
                                                                          : w.update(done.addressed, Command(VelocityCommand{}));
            break;
        }
        case Op::Legacy: {
            std::vector<std::size_t> cascade;
            for (std::size_t i = 0; i < caps.size(); ++i)
                if ((caps[i].kind == CapabilityKind::Flight && !Maker::isSupport(caps[i])) || (caps[i].kind == CapabilityKind::Guidance && make.chance(0.15)))
                    cascade.push_back(i);
            Command c;
            REQUIRE(make.cascade(caps[cascade[make.pick(cascade.size())]], true, c));
            done.accepted = w.command(v, c);
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
        const bool changes = done.op == Op::Step || done.op == Op::Reset || (done.op == Op::New && done.result.accepted()) ||
                             (done.op == Op::Legacy && done.accepted) || (done.op == Op::Cancel && done.result.status == CommandStatus::Canceled);
        if (!changes) {
            // refused, an UPDATE, or nothing to do with the records: they are as they were
            CHECK(after.size() == before.size());
            for (const auto& [id, r] : after)
                if (const auto it = before.find(id); it != before.end()) CHECK(sameRecord(it->second, r));
        }
        before = after;
        transcript.insert(transcript.end(), {static_cast<double>(done.op), static_cast<double>(done.result.status), static_cast<double>(done.result.reason),
                                             static_cast<double>(serialOf(done.result.activity)), static_cast<double>(done.result.flags),
                                             static_cast<double>(done.accepted)});
    }
    INFO(aircraft.type << ": " << lastSerial << " activities, the last " << before.size() << " of them kept");
    CHECK(lastSerial > 50);
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
    CHECK(families == std::set<std::string>{"jsbsim.stock", "jsbsim.direct", "jsbsim.fbw"}); // every adapter
}

TEST_CASE("conformance: one aircraft per adapter keeps the lifecycle's rules through random sequences of every operation", "[conformance]") {
    std::map<std::string, int> all;
    for (const Aircraft& a : kAdapters) {
        INFO(a.type);
        std::map<std::string, int> seen, again;
        const auto first = randomSequence(a, 20260926, 600, seen);
        CHECK(first == randomSequence(a, 20260926, 600, again)); // the same calls, the same answers and the same flight
        // every edge of the state machine and every answer an operation can give, for every adapter
        for (const char* what : {"pending->active", "active->pending", "pending->canceled", "active->canceled", "canceled:preempted",
                                 "canceled:requested", "completed:goal_reached", "new:done", "new:authority_held",
                                 "new:invalid_axes", "new:out_of_range", "new:invalid_parameter", "update:done", "update:not_updatable",
                                 "update:wrong_command_type", "update:activity_ended", "update:unknown_activity", "cancel:done",
                                 "cancel:activity_ended", "cancel:unknown_activity", "clamped"}) {
            INFO(what);
            CHECK(seen[what] > 0);
        }
        for (const auto& [what, n] : seen) all[what] += n;
    }
    CHECK(all["failed:target_lost"] > 0); // a follower whose target goes
    std::string summary;
    for (const auto& [what, n] : all) summary += what + " " + std::to_string(n) + "; ";
    INFO(summary);
    CHECK(all.size() > 20);
}
