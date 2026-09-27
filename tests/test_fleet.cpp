// The fleet acceptance test (docs/flight-autonomy.md, 9.3): on every aircraft
// hangar ships, every capability it advertises flies a case from a standard
// condition - a wing at 3,000 m at its reference airspeed, a rotorcraft
// hovering at 150 m, an aircraft parked for the ground cases - within the
// thresholds of its class; and discovery tells the truth: every advertised
// capability has a passing case, every physical exception its declaration,
// every refusal the reason section 4.3 gives. A capability that stops working
// on any aircraft fails this test, so it cannot stay advertised.
#include "mode_flights.h"

#include "control/Features.h"
#include "fsim/BuiltinControllers.h"
#include "sim/FlightModel.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
/// The air's density at 3,000 m over sea level's (ISA): a wing's true airspeed there is its calibrated over its root.
constexpr double kSigma3000 = 0.7422;

/// The classes a case's thresholds are set for.
enum class Class { Direct, FlyByWire, Helicopter, Multirotor };

const char* className(Class c) {
    switch (c) {
    case Class::Direct: return "fixed-wing direct";
    case Class::FlyByWire: return "fixed-wing fly-by-wire";
    case Class::Helicopter: return "helicopter";
    default: return "multirotor";
    }
}

/// hangar's designs, as the platform finds them.
std::vector<std::string> designs() {
    std::vector<std::string> out;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(FSIM_TEST_AIRCRAFT_DIR, ec)) {
        const std::string name = entry.path().filename().string();
        if (entry.is_directory(ec) && std::filesystem::is_regular_file(entry.path() / (name + ".xml"), ec)) out.push_back(name);
    }
    std::sort(out.begin(), out.end());
    return out;
}

/// One aircraft in a case: its type, class, standard condition and the vehicles flying it.
struct Plane {
    std::string type;
    Class cls = Class::Direct;
    bool rotor = false;
    double tasMs = 0.0;      ///< its reference airspeed (a wing's)
    double cruiseMs = 0.0;   ///< what a mode flies given none (a rotorcraft's half its fastest over the ground)
    double pitchDeg = 0.0, rollDeg = 0.0; ///< a rotorcraft's hover attitude
    double minCasMs = kHold, gearCasMs = kHold, flapCasMs = kHold; ///< a wing's least, and its gear's and flaps' most (NaN: none)
    double latitudeDeg = 0.0;
    std::uint32_t id = 0, helper = 0; ///< the vehicle, and the one it follows or flees (0: none)
    sim::VehicleState start{};        ///< as the case began
    std::set<std::string> offered;    ///< its capabilities' ids
    /// how far a case's geometry reaches: 20 s at its cruise (a wing's kilometres, a Crazyflie's metres)
    double scale() const { return 20.0 * cruiseMs; }
};

/// The fleet, one World for every case: vehicles are made afresh for each (the world keeps each
/// aircraft's model loaded, so a new vehicle of it is quick), all of them flying at once.
class Fleet {
public:
    Fleet() : w_(fleetOptions()) {
        const auto names = designs();
        REQUIRE(names.size() >= 35);
        for (std::size_t i = 0; i < names.size(); ++i) {
            Plane p;
            p.type = names[i];
            p.latitudeDeg = 40.0 + 0.4 * static_cast<double>(i); // 44 km apart
            session::VehicleSpec probe;
            probe.name = p.type + "-probe";
            probe.type = "jsbsim:" + p.type;
            probe.initial.latitudeDeg = p.latitudeDeg;
            probe.initial.altitudeMslM = 3000.0;
            const auto id = w_.createVehicle(probe);
            REQUIRE(id != 0);
            const VehicleProfile& profile = *w_.profile(id);
            const Performance& perf = *w_.performance(id);
            switch (profile.identity.family) {
            case ControlFamily::FlyByWire: p.cls = Class::FlyByWire; break;
            case ControlFamily::Helicopter: p.cls = Class::Helicopter; break;
            case ControlFamily::Multirotor: p.cls = Class::Multirotor; break;
            default: p.cls = Class::Direct; break;
            }
            p.rotor = isRotorcraft(profile.identity.family);
            p.tasMs = std::isfinite(profile.plant.tasMs) ? profile.plant.tasMs : (std::isfinite(perf.cruiseTasMs) ? perf.cruiseTasMs : 100.0);
            p.cruiseMs = std::isfinite(perf.cruiseTasMs) ? perf.cruiseTasMs : p.tasMs;
            p.minCasMs = perf.minCasMs;
            p.gearCasMs = profile.envelope.gearCasMaxMs;
            p.flapCasMs = profile.envelope.flaps.casMaxMs;
            if (p.rotor) {
                if (std::isfinite(profile.hover.pitchAttitudeRad)) p.pitchDeg = profile.hover.pitchAttitudeRad / kDeg;
                if (std::isfinite(profile.hover.rollAttitudeRad)) p.rollDeg = profile.hover.rollAttitudeRad / kDeg;
            }
            for (const auto& d : w_.capabilities(id)) p.offered.insert(d.id);
            REQUIRE(w_.removeVehicle(id));
            planes_.push_back(p);
        }
    }

    session::World& world() { return w_; }
    std::vector<Plane>& planes() { return planes_; }

    /// Every aircraft at its standard condition, settled on its velocity loop for 10 s, then let go: a
    /// wing at 3,000 m heading north at its reference airspeed (or `tasMs`'s), a rotorcraft hovering at
    /// 150 m. With `helperAheadS`, another of its type that many seconds of its cruise ahead, flying
    /// straight on (a rotorcraft's hovering).
    void standard(double helperAheadS = 0.0, const std::function<double(const Plane&)>& tasMs = {}) {
        clear();
        for (auto& p : planes_) {
            const double tas = p.rotor ? 0.0 : (tasMs ? tasMs(p) : p.tasMs);
            p.id = spawn(p, p.type + "-" + std::to_string(serial_), 0.0, tas, false);
            p.helper = helperAheadS > 0.0 ? spawn(p, p.type + "-helper-" + std::to_string(serial_), helperAheadS * p.cruiseMs, tas, false) : 0;
        }
        ++serial_;
        std::vector<ActivityId> held;
        for (auto& p : planes_)
            for (const auto v : {p.id, p.helper})
                if (v) {
                    const CommandResult r = w_.submit(v, still(p, *w_.vehicleState(v)));
                    REQUIRE(r.accepted());
                    if (v == p.id) held.push_back(r.activity);
                }
        w_.step(stepsFor(w_, 10.0));
        for (const ActivityId a : held) w_.cancel(a); // (the helpers fly on)
        for (auto& p : planes_) p.start = *w_.vehicleState(p.id);
    }

    /// Every aircraft parked on its wheels, skids or legs, settled for 5 s.
    void park() {
        clear();
        for (auto& p : planes_) p.id = spawn(p, p.type + "-parked-" + std::to_string(serial_), 0.0, 0.0, true);
        ++serial_;
        w_.step(stepsFor(w_, 5.0));
        for (auto& p : planes_) p.start = *w_.vehicleState(p.id);
    }

    /// Straight and level where it is: a wing's velocity loop at its speed and heading; a rotorcraft still over the ground.
    static VelocityCommand still(const Plane& p, const sim::VehicleState& s) {
        VelocityCommand c;
        c.verticalSpeedMs = 0.0;
        c.headingRad = s.eulerRad[2];
        if (p.rotor) c.northMs = c.eastMs = 0.0;
        else c.airspeedMs = s.airspeedTrueMs;
        return c;
    }

    /// Steps `seconds`, calling `each` after every world step.
    void fly(double seconds, const std::function<void()>& each = {}) {
        const unsigned n = stepsFor(w_, seconds);
        for (unsigned k = 0; k < n; ++k) {
            w_.step();
            if (each) each();
        }
    }

    /// Records that `capability` had its case on `p` (a failing check in it fails the test anyway).
    void covered(const Plane& p, const std::string& capability) { covered_[p.type].insert(capability); }
    const std::map<std::string, std::set<std::string>>& coverage() const { return covered_; }

private:
    static session::WorldOptions fleetOptions() {
        session::WorldOptions o;
        o.name = "fleet";
        o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
        o.publish = false;
        o.seed = 29;
        o.workers = 0; // all it can: the answers are the same with any number
        return o;
    }

    void clear() {
        for (auto& p : planes_) {
            if (p.id) REQUIRE(w_.removeVehicle(p.id));
            if (p.helper) REQUIRE(w_.removeVehicle(p.helper));
            p.id = p.helper = 0;
        }
    }

    std::uint32_t spawn(const Plane& p, const std::string& name, double aheadM, double tasMs, bool parked) {
        session::VehicleSpec s;
        s.name = name;
        s.type = "jsbsim:" + p.type;
        s.initial.latitudeDeg = p.latitudeDeg + aheadM / kEarthM / kDeg;
        s.initial.longitudeDeg = 0.0;
        s.initial.headingDeg = 0.0;
        if (parked) {
            s.initial.onGround = true;
            s.initial.airspeedTrueMs = 0.0;
        } else if (p.rotor) {
            s.initial.altitudeMslM = 150.0;
            s.initial.airspeedTrueMs = 0.0;
            s.initial.pitchDeg = p.pitchDeg;
            s.initial.rollDeg = p.rollDeg;
        } else {
            s.initial.altitudeMslM = 3000.0;
            s.initial.airspeedTrueMs = tasMs;
        }
        const auto id = w_.createVehicle(s);
        REQUIRE(id != 0);
        return id;
    }

    session::World w_;
    std::vector<Plane> planes_;
    std::map<std::string, std::set<std::string>> covered_;
    unsigned serial_ = 0;
};

double headingOffDeg(const sim::VehicleState& s, double headingRad) { return std::abs(std::remainder(s.eulerRad[2] - headingRad, 2.0 * kPi)) / kDeg; }

double groundDistance(const sim::VehicleState& a, const sim::VehicleState& b) {
    double north, east;
    offset(a, b.latitudeRad, b.longitudeRad, north, east);
    return std::hypot(north, east);
}

/// How far `s` is from a point over the ground.
double distanceTo(const sim::VehicleState& s, const PositionCommand& point) {
    double north, east;
    offset(s, point.latitudeRad, point.longitudeRad, north, east);
    return std::hypot(north, east);
}

PositionCommand pointFrom(const sim::VehicleState& s, double north, double east, double altitudeM, double captureM) {
    PositionCommand p;
    p.latitudeRad = s.latitudeRad + north / kEarthM;
    p.longitudeRad = s.longitudeRad + east / (kEarthM * std::cos(s.latitudeRad));
    p.altitudeMslM = altitudeM;
    p.captureRadiusM = captureM;
    return p;
}

BehaviorCommand behavior(const char* id, std::uint32_t target = 0) {
    BehaviorCommand b;
    b.id = id;
    b.target = target;
    return b;
}

/// The least a vehicle's state came to over a flight: height above the ground, calibrated airspeed; and whether it touched down.
struct Lows {
    double agl = kInf, cas = kInf;
    bool ground = false, diverged = false;
    void see(const sim::VehicleState& s) {
        agl = std::min(agl, s.altitudeAglM);
        cas = std::min(cas, s.airspeedCalibratedMs);
        ground = ground || s.onGround;
        diverged = diverged || s.diverged;
    }
};

/// No ground contact, no divergence and (`casFloor`) no calibrated airspeed below the envelope's least (a wing's).
void keptSafe(const Plane& p, const Lows& lows, bool casFloor = true) {
    CHECK_FALSE(lows.ground);
    CHECK_FALSE(lows.diverged);
    if (casFloor && !p.rotor && std::isfinite(p.minCasMs)) CHECK(lows.cas > p.minCasMs - 3.0);
}

/// A JSBSim property of the vehicle's model (0 where the model has no such property).
double property(session::World& w, std::uint32_t id, const char* path) { return w.model(id)->property(path).get(); }

} // namespace

TEST_CASE("fleet: discovery tells the truth on every aircraft", "[fleet]") {
    Fleet fleet;
    session::World& w = fleet.world();
    fleet.standard();
    for (auto& p : fleet.planes()) {
        INFO(p.type << " (" << className(p.cls) << ")");
        const SupportTable& table = *w.supportTable(p.id);
        for (const SupportInfo& row : table.rows()) {
            INFO(row.feature);
            const bool capability = std::string_view(row.feature) == row.capability;
            // an offered capability is one the table says works; what it does not offer, the table says why
            if (capability) CHECK(p.offered.count(row.feature) == ((row.support == Support::Supported || row.support == Support::Partial) ? 1u : 0u));
            if (row.support == Support::NotSupported) {
                CHECK(row.rules != 0);
                CHECK(*row.evidence != '\0'); // every physical exception its declaration and source (R12 and R13: the aircraft file)
            }
            if (row.support == Support::NotImplemented) CHECK(row.missing == std::string_view());
            if (row.support == Support::Partial) CHECK(*row.missing != '\0');
        }
        // every refusal the reason section 4.3 gives: for each capability it does not offer
        for (const SupportInfo& row : table.rows()) {
            const std::string_view id = row.feature;
            if (id != row.capability || p.offered.count(row.feature)) continue;
            const Reason expected = row.support == Support::NotSupported ? Reason::NotSupported : Reason::NotImplemented;
            INFO(id);
            if (id.substr(0, 14) == "fsim.guidance.") {
                BehaviorCommand b = behavior(row.feature + 14);
                CHECK(w.submit(p.id, b).reason == expected);
            } else if (id.substr(0, 13) == "fsim.support." || id == "fsim.flight.engines") {
                const std::string_view kind = id.substr(id.find_last_of('.') + 1);
                SupportCommand c = GearCommand{1.0};
                if (kind == "flaps") c = FlapsCommand{0.0};
                else if (kind == "wheel_brakes") c = WheelBrakesCommand{0.0, 0.0};
                else if (kind == "speedbrake") c = SpeedbrakeCommand{0.0};
                else if (kind == "pitch_trim") c = PitchTrimCommand{0.0};
                else if (kind == "engines") c = EnginesCommand{{0.5, kHold, kHold, kHold}};
                CHECK(w.submit(p.id, c).reason == expected);
            }
            const CapabilityStatus status = w.capabilityStatus(p.id, id);
            CHECK(status.availability == Availability::Unavailable);
            CHECK(status.reason == expected);
        }
        CHECK(w.submit(p.id, behavior("warp_drive")).reason == Reason::UnknownCapability);
    }
}

TEST_CASE("fleet: every advertised capability flies its case within its class's thresholds", "[fleet]") {
    Fleet fleet;
    session::World& w = fleet.world();
    auto& planes = fleet.planes();
    using Start = std::function<bool(const Plane&)>;
    using Seconds = std::function<double(const Plane&)>;
    using Watch = std::function<void(const Plane&)>;
    using Judge = std::function<void(const Plane&, const Lows&)>;
    // One case: every aircraft that offers `capability`, from its standard condition (with a helper
    // `helperAheadS` of its cruise ahead; 0: none), all flying at once. `start` commands it (false: it
    // was refused, and the case is not covered), `watch` sees every step, `judge` its end, `seconds` on.
    auto run = [&](const char* capability, double helperAheadS, const Start& start, const Seconds& seconds, const Watch& watch, const Judge& judge,
                   bool casFloor = true) {
        fleet.standard(helperAheadS);
        std::vector<const Plane*> cast;
        std::map<std::uint32_t, Lows> lows;
        std::map<std::uint32_t, double> ends;
        double longest = 0.0;
        for (const auto& p : planes) {
            if (!p.offered.count(capability)) continue;
            INFO(p.type << " (" << className(p.cls) << "): " << capability);
            if (!start(p)) continue;
            cast.push_back(&p);
            ends[p.id] = seconds(p);
            longest = std::max(longest, ends[p.id]);
        }
        const double t0 = w.simTime();
        const unsigned n = stepsFor(w, longest);
        std::set<std::uint32_t> judged;
        for (unsigned k = 0; k < n && judged.size() < cast.size(); ++k) {
            w.step();
            for (const Plane* p : cast) {
                if (judged.count(p->id)) continue;
                lows[p->id].see(*w.vehicleState(p->id));
                if (watch) watch(*p);
                if (w.simTime() - t0 < ends[p->id] - 1e-6 && k + 1 < n) continue;
                INFO(p->type << " (" << className(p->cls) << "): " << capability);
                keptSafe(*p, lows[p->id], casFloor);
                judge(*p, lows[p->id]);
                fleet.covered(*p, capability);
                judged.insert(p->id);
            }
        }
    };
    auto secs = [](double wing, double rotor) { return Seconds([wing, rotor](const Plane& p) { return p.rotor ? rotor : wing; }); };
    const Watch none;
    auto perf = [&](const Plane& p) -> const Performance& { return *w.performance(p.id); };
    // a wing's turn radius at its speed and full bank (the waypoints' admission's)
    auto fullBankRadius = [&](const Plane& p) {
        const double bank = std::isfinite(perf(p).maxBankRad) && perf(p).maxBankRad > 0.05 ? perf(p).maxBankRad : 0.52;
        return p.start.airspeedTrueMs * p.start.airspeedTrueMs / (9.80665 * std::tan(bank));
    };
    // what a case records about each vehicle as it starts and flies
    std::map<std::uint32_t, ActivityId> activity;
    std::map<std::uint32_t, double> least, gap;
    std::map<std::uint32_t, PositionCommand> target;

    // --- the flight levels ----------------------------------------------------------------------------
    run("fsim.flight.velocity", 0.0, [&](const Plane& p) { return w.submit(p.id, Fleet::still(p, p.start)).accepted(); }, secs(30.0, 30.0), none,
        [&](const Plane& p, const Lows&) {
            const auto& s = *w.vehicleState(p.id);
            // (the worst of the fleet: 0.05 deg; a wing 24 m high, a rotorcraft 0.13 m, 0.09 m off its point; 0.03 m/s)
            CHECK(headingOffDeg(s, p.start.eulerRad[2]) < 1.0);
            CHECK(std::abs(s.altitudeMslM - p.start.altitudeMslM) < (p.rotor ? 1.0 : 50.0));
            if (p.rotor) CHECK(groundDistance(s, p.start) < std::max(0.5, 0.1 * p.cruiseMs));
            else CHECK(std::abs(s.airspeedTrueMs - p.start.airspeedTrueMs) < 1.0);
        });
    // at the standard condition a support effector's status and its admission agree: refused with the
    // status's reason, or accepted where the range it gives allows the value
    for (const auto& p : planes) {
        INFO(p.type << " (" << className(p.cls) << ") at its standard condition");
        const std::pair<const char*, SupportCommand> asks[] = {{"fsim.support.gear", GearCommand{0.0}}, {"fsim.support.flaps", FlapsCommand{1.0}}};
        for (const auto& [capability, command] : asks) {
            if (!p.offered.count(capability)) continue;
            INFO(capability);
            const double value = std::holds_alternative<GearCommand>(command) ? 0.0 : 1.0;
            const CapabilityStatus status = w.capabilityStatus(p.id, capability);
            const bool allowed = status.availability == Availability::Available &&
                                 (status.rangeCount == 0 || (value >= status.ranges[0].min && value <= status.ranges[0].max));
            const CommandResult r = w.submit(p.id, command);
            CHECK(r.accepted() == allowed);
            if (status.availability != Availability::Available) CHECK(r.reason == status.reason);
            else if (!allowed) CHECK(r.reason == Reason::Unavailable);
            if (r.accepted()) w.cancel(r.activity);
        }
    }
    run("fsim.flight.attitude", 0.0,
        [&](const Plane& p) {
            AttitudeCommand a; // its hover's roll (a wing's level), its pitch, its heading (a wing's speed on the throttle)
            a.rollRad = p.rotor ? p.rollDeg * kDeg : 0.0;
            a.pitchRad = p.start.eulerRad[1];
            a.headingRad = p.start.eulerRad[2];
            if (!p.rotor) a.airspeedMs = p.start.airspeedTrueMs;
            return w.submit(p.id, a).accepted();
        },
        secs(20.0, 10.0), none,
        [&](const Plane& p, const Lows&) {
            const auto& s = *w.vehicleState(p.id);
            // (the worst: 0.13 deg of roll, 0.05 of heading; a wing 34 m off its height, a rotorcraft 0.17 m)
            CHECK(std::abs(degreesApart(s.eulerRad[0], p.rotor ? p.rollDeg * kDeg : 0.0)) < 1.0);
            CHECK(headingOffDeg(s, p.start.eulerRad[2]) < 1.0);
            CHECK(std::abs(s.altitudeMslM - p.start.altitudeMslM) < (p.rotor ? 2.0 : 75.0));
        });
    run("fsim.flight.acceleration", 0.0,
        [&](const Plane& p) {
            AccelerationCommand a; // 1 g and no roll rate (a rotorcraft: no body rates at all)
            if (p.rotor) a.pitchRateRadS = a.yawRateRadS = 0.0;
            return w.submit(p.id, a).accepted();
        },
        secs(20.0, 10.0), none,
        [&](const Plane& p, const Lows&) {
            const auto& s = *w.vehicleState(p.id);
            // (the worst: 0.19 deg of roll; 1 g held open loop, a wing 64 m off its height, a rotorcraft 1.6 m)
            CHECK(std::abs(degreesApart(s.eulerRad[0], p.start.eulerRad[0])) < 1.0);
            CHECK(std::abs(s.altitudeMslM - p.start.altitudeMslM) < (p.rotor ? 5.0 : 150.0));
        });
    run("fsim.flight.actuator", 0.0,
        [&](const Plane& p) {
            const sim::ControlInputs& in = *w.inputs(p.id); // what its velocity loop held it with, and some right aileron
            ActuatorCommand a;
            a.aileron = std::clamp(in.aileron + (p.rotor ? 0.05 : 0.2), -1.0, 1.0);
            a.elevator = in.elevator, a.rudder = in.rudder, a.throttle = in.throttle[0], a.flaps = in.flaps;
            return w.submit(p.id, a).accepted();
        },
        secs(2.0, 1.0), none,
        [&](const Plane& p, const Lows&) {
            const auto& s = *w.vehicleState(p.id);
            // it rolled right (the least: a wing 10.5 deg in 2 s, a rotorcraft 1.9 deg in 1 s), and stayed near its height (4 m)
            CHECK(degreesApart(s.eulerRad[0], p.start.eulerRad[0]) > (p.rotor ? 1.0 : 5.0));
            CHECK(std::abs(s.altitudeMslM - p.start.altitudeMslM) < (p.rotor ? 5.0 : 20.0));
        });
    run("fsim.flight.position", 0.0,
        [&](const Plane& p) { // 20 s of its cruise ahead
            const double d = p.scale(), psi = p.start.eulerRad[2];
            target[p.id] = pointFrom(p.start, d * std::cos(psi), d * std::sin(psi), p.start.altitudeMslM, p.rotor ? std::max(0.5, 0.02 * d) : 200.0);
            least[p.id] = kInf;
            return w.submit(p.id, target[p.id]).accepted();
        },
        secs(35.0, 60.0), [&](const Plane& p) { least[p.id] = std::min(least[p.id], distanceTo(*w.vehicleState(p.id), target[p.id])); },
        [&](const Plane& p, const Lows&) {
            // it got there (a wing passes within 3.4 m of it), and a rotorcraft stopped within its capture radius
            CHECK(least[p.id] < (p.rotor ? target[p.id].captureRadiusM : 20.0));
            if (p.rotor) CHECK(distanceTo(*w.vehicleState(p.id), target[p.id]) < target[p.id].captureRadiusM);
        });

    // each engine's throttle a little open (a multirotor's rotors: every axis theirs, so briefly): thrust is the
    // engines' now, and the velocity loop flies the rest
    std::map<std::uint32_t, EnginesCommand> engines;
    run("fsim.flight.engines", 0.0,
        [&](const Plane& p) {
            const sim::ControlInputs& in = *w.inputs(p.id);
            for (int i = 0; i < std::min(w.profile(p.id)->propulsion.engines, 4); ++i) engines[p.id].throttle[i] = std::clamp(in.throttle[i] + 0.05, 0.0, 1.0);
            return w.submit(p.id, Fleet::still(p, p.start)).accepted() && w.submit(p.id, engines[p.id]).accepted();
        },
        secs(10.0, 1.0), none,
        [&](const Plane& p, const Lows&) {
            const sim::ControlInputs& in = *w.inputs(p.id);
            for (int i = 0; i < 4; ++i)
                if (!isHold(engines[p.id].throttle[i])) CHECK(in.throttle[i] == engines[p.id].throttle[i]);
            if (p.rotor) CHECK(w.vehicleState(p.id)->altitudeMslM > p.start.altitudeMslM); // more thrust: it climbs
            else CHECK(headingOffDeg(*w.vehicleState(p.id), p.start.eulerRad[2]) < 1.0); // (the worst: 0.05 deg)
        });

    // --- the support effectors and envelope protection: set while it flies level, slow enough for its gear
    // and flaps - a wing at 90 % of the lower of their speeds; where it has neither (no hangar design records
    // them yet), at 1.4 times its least calibrated airspeed, a flap speed's usual place (about 1.7 times the
    // stall); with no least either, as it flies; never below 1.3 times its least
    {
        fleet.standard(0.0, [](const Plane& p) {
            double cas = p.tasMs * std::sqrt(kSigma3000);
            bool placard = false;
            for (const double limit : {p.gearCasMs, p.flapCasMs})
                if (std::isfinite(limit)) cas = std::min(cas, 0.9 * limit), placard = true;
            if (!placard && std::isfinite(p.minCasMs)) cas = std::min(cas, 1.4 * p.minCasMs);
            if (std::isfinite(p.minCasMs)) cas = std::max(cas, 1.3 * p.minCasMs);
            return cas / std::sqrt(kSigma3000);
        });
        for (const auto& p : planes) REQUIRE(w.submit(p.id, Fleet::still(p, p.start)).accepted());
        std::map<std::uint32_t, Lows> lows;
        auto seeAll = [&] {
            for (const auto& p : planes) lows[p.id].see(*w.vehicleState(p.id));
        };
        struct Moving {
            const Plane* plane;
            const char* capability;
            ActivityId activity;
        };
        std::vector<Moving> moving;
        auto set = [&](const Plane& p, const char* capability, const SupportCommand& c, bool completes) {
            if (!p.offered.count(capability)) return;
            INFO(p.type << ": " << capability);
            CHECK(w.capabilityStatus(p.id, capability).availability == Availability::Available);
            const CommandResult r = w.submit(p.id, c);
            CHECK(r.accepted());
            if (r.accepted() && completes) moving.push_back({&p, capability, r.activity});
        };
        auto arrived = [&] {
            for (const Moving& m : moving) {
                INFO(m.plane->type << ": " << m.capability << " " << activityStateName(w.activity(m.activity)->state) << ", the gear at "
                                   << property(w, m.plane->id, "gear/gear-pos-norm") << ", the flaps at " << property(w, m.plane->id, "fcs/flap-pos-deg")
                                   << " deg, " << w.vehicleState(m.plane->id)->airspeedCalibratedMs << " m/s CAS");
                CHECK(w.activity(m.activity)->state == ActivityState::Completed);
            }
            moving.clear();
        };
        // the gear up and the flaps half out, then the gear down and the flaps in: each where it was told
        for (const auto& p : planes) set(p, "fsim.support.gear", GearCommand{0.0}, true), set(p, "fsim.support.flaps", FlapsCommand{0.5}, true);
        fleet.fly(30.0, seeAll);
        arrived();
        for (const auto& p : planes) {
            INFO(p.type);
            if (p.offered.count("fsim.support.gear")) CHECK(property(w, p.id, "gear/gear-pos-norm") < 0.01);
            if (p.offered.count("fsim.support.flaps")) CHECK(property(w, p.id, "fcs/flap-pos-deg") > 1.0);
            set(p, "fsim.support.gear", GearCommand{1.0}, true), set(p, "fsim.support.flaps", FlapsCommand{0.0}, true);
        }
        fleet.fly(30.0, seeAll);
        arrived();
        for (const auto& p : planes) {
            INFO(p.type);
            if (p.offered.count("fsim.support.gear")) {
                CHECK(property(w, p.id, "gear/gear-pos-norm") > 0.99);
                fleet.covered(p, "fsim.support.gear");
            }
            if (p.offered.count("fsim.support.flaps")) {
                CHECK(property(w, p.id, "fcs/flap-pos-deg") < 0.1);
                fleet.covered(p, "fsim.support.flaps");
            }
        }
        // the pitch trim and the speedbrake set; envelope protection's mode changed and back
        for (const auto& p : planes) {
            set(p, "fsim.support.pitch_trim", PitchTrimCommand{0.05}, false);
            set(p, "fsim.support.speedbrake", SpeedbrakeCommand{0.5}, false);
            if (p.offered.count("fsim.envelope.protection")) {
                INFO(p.type << ": fsim.envelope.protection");
                const ProtectionMode was = w.protection(p.id);
                CHECK(w.setProtection(p.id, ProtectionMode::Report) == Reason::None);
                CHECK(w.protection(p.id) == ProtectionMode::Report);
                CHECK(w.setProtection(p.id, was) == Reason::None);
                CHECK(w.protection(p.id) == was);
                fleet.covered(p, "fsim.envelope.protection");
            }
        }
        fleet.fly(10.0, seeAll);
        for (const auto& p : planes) {
            INFO(p.type);
            keptSafe(p, lows[p.id]);
            if (p.offered.count("fsim.support.pitch_trim")) {
                CHECK(std::abs(property(w, p.id, "fcs/pitch-trim-cmd-norm") - 0.05) < 1e-9);
                fleet.covered(p, "fsim.support.pitch_trim");
            }
            if (p.offered.count("fsim.support.speedbrake")) {
                CHECK(std::abs(property(w, p.id, "fcs/speedbrake-cmd-norm") - 0.5) < 1e-9);
                fleet.covered(p, "fsim.support.speedbrake");
            }
        }
    }

    // --- parked: a wing idling stands; a policy's airborne guidance waits for the air, the flight levels and
    // the platform's own paths do not; the gear stays down; the flaps move; the wheel brakes hold it ---------
    {
        fleet.park();
        std::vector<std::pair<const Plane*, ActivityId>> flaps;
        for (const auto& p : planes) {
            INFO(p.type << " parked");
            REQUIRE(w.vehicleState(p.id)->onGround);
            // its engines' idle thrust under 8 % of its weight (the worst: the F-22A's 4.0 %, a jet's idle; hangar's
            // turboprops made 17 %, governed at flight idle on their low stops, until their ground range)
            if (!p.rotor) CHECK(property(w, p.id, "forces/fbx-prop-lbs") < 0.08 * property(w, p.id, "inertia/weight-lbs"));
            CHECK(w.capabilityStatus(p.id, "fsim.flight.velocity").availability == Availability::Available);
            if (p.offered.count("fsim.guidance.hsa")) {
                const CapabilityStatus hsa = w.capabilityStatus(p.id, "fsim.guidance.hsa");
                CHECK(hsa.availability == Availability::TemporarilyUnavailable);
                CHECK(hsa.reason == Reason::OnGround);
                HsaCommand h;
                h.headingRad = 1.0;
                CHECK(w.submit(p.id, h).reason == Reason::OnGround);
                CommandOptions autopilot;
                autopilot.source = Source::Autopilot; // the platform's own: a take-off's, a landing's
                const CommandResult r = w.submit(p.id, h, autopilot);
                CHECK(r.accepted());
                if (r.accepted()) w.cancel(Source::Autopilot, r.activity);
            }
            if (p.offered.count("fsim.support.gear")) {
                const CapabilityStatus gear = w.capabilityStatus(p.id, "fsim.support.gear");
                REQUIRE(gear.rangeCount == 1);
                CHECK(gear.ranges[0].min == 0.5);
                CHECK(w.submit(p.id, GearCommand{0.0}).reason == Reason::Unavailable);
            }
            if (p.offered.count("fsim.support.flaps")) {
                const CommandResult r = w.submit(p.id, FlapsCommand{0.5});
                CHECK(r.accepted());
                if (r.accepted()) flaps.emplace_back(&p, r.activity);
            }
            if (p.offered.count("fsim.support.wheel_brakes")) CHECK(w.submit(p.id, WheelBrakesCommand{1.0, 1.0}).accepted());
        }
        fleet.fly(10.0);
        std::map<std::uint32_t, sim::VehicleState> braked;
        for (const auto& p : planes) braked[p.id] = *w.vehicleState(p.id);
        fleet.fly(5.0);
        for (const auto& [p, a] : flaps) {
            INFO(p->type << " parked: flaps");
            CHECK(w.activity(a)->state == ActivityState::Completed);
            CHECK(property(w, p->id, "fcs/flap-pos-deg") > 1.0);
        }
        for (const auto& p : planes) {
            if (!p.offered.count("fsim.support.wheel_brakes")) continue;
            INFO(p.type << " parked: wheel brakes");
            CHECK(property(w, p.id, "fcs/left-brake-cmd-norm") == 1.0);
            CHECK(property(w, p.id, "fcs/right-brake-cmd-norm") == 1.0);
            const auto& s = *w.vehicleState(p.id); // stopped, and held
            CHECK(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]) < 0.05);
            CHECK(groundDistance(s, braked[p.id]) < 0.05);
            fleet.covered(p, "fsim.support.wheel_brakes");
        }
    }

    // --- guidance ------------------------------------------------------------------------------------
    // a new heading, a quarter turn right, and a new altitude: a wing 200 m lower (the climb case waits for
    // FA-3's energy management: asked for 200 m at 3,000 m, a C172 trades its speed down to its least and
    // creeps), a rotorcraft a tenth of its scale higher (2 to 20 m)
    auto climb = [](const Plane& p) { return p.rotor ? std::clamp(0.1 * p.scale(), 2.0, 20.0) : -200.0; };
    for (const char* mode : {"fsim.guidance.hold", "fsim.guidance.hsa"}) {
        const bool hold = std::string(mode) == "fsim.guidance.hold";
        run(mode, 0.0,
            [&](const Plane& p) {
                const double heading = std::remainder(p.start.eulerRad[2] + 0.5 * kPi, 2.0 * kPi);
                if (hold) {
                    BehaviorCommand b = behavior("hold");
                    b.params = {{"heading_deg", heading / kDeg}, {"altitude_m", p.start.altitudeMslM + climb(p)}};
                    return w.submit(p.id, b).accepted();
                }
                HsaCommand h;
                h.headingRad = heading, h.altitudeM = p.start.altitudeMslM + climb(p);
                return w.submit(p.id, h).accepted();
            },
            secs(120.0, 60.0), none,
            [&](const Plane& p, const Lows&) {
                const auto& s = *w.vehicleState(p.id);
                // (the worst: 0.05 deg; a wing still closing on its altitude, 8.4 m off - the B-52H - a rotorcraft on it)
                CHECK(headingOffDeg(s, p.start.eulerRad[2] + 0.5 * kPi) < 1.0);
                CHECK(std::abs(s.altitudeMslM - (p.start.altitudeMslM + climb(p))) < (p.rotor ? 0.5 : 20.0));
            });
    }
    // three sides of a square, turning right: a wing's legs four of its full-bank turns long (at least 2 km), a rotorcraft's its scale
    auto leg = [&](const Plane& p) { return p.rotor ? p.scale() : std::max(2000.0, 4.0 * fullBankRadius(p)); };
    auto square = [&](const Plane& p) {
        const double d = leg(p), capture = p.rotor ? std::max(0.5, 0.05 * d) : std::max(200.0, 0.1 * d), alt = p.start.altitudeMslM;
        return std::vector<PositionCommand>{pointFrom(p.start, d, 0.0, alt, capture), pointFrom(p.start, d, d, alt, capture), pointFrom(p.start, 0.0, d, alt, capture)};
    };
    const Seconds aroundSquare = [&](const Plane& p) { return 3.0 * leg(p) / std::max(p.cruiseMs, 0.1) * 1.5 + 60.0; };
    const Judge completed = [&](const Plane& p, const Lows&) {
        const ActivityRecord& r = *w.activity(activity[p.id]);
        INFO(activityStateName(r.state) << " " << reasonName(r.reason));
        CHECK(r.state == ActivityState::Completed);
    };
    run("fsim.guidance.waypoints", 0.0,
        [&](const Plane& p) {
            BehaviorCommand b = behavior("waypoints");
            b.points = square(p);
            const CommandResult r = w.submit(p.id, b);
            INFO("refused: " << reasonName(r.reason));
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        aroundSquare, none, completed);
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            std::vector<Waypoint> route;
            for (const PositionCommand& q : square(p)) {
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                route.push_back(wp);
            }
            const CommandResult r = w.submit(p.id, RouteCommand{}, Span<const Waypoint>(route.data(), route.size()));
            INFO("refused: " << reasonName(r.reason));
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        aroundSquare, none, completed);
    // round where it is: a loiter at its default radius (a rotorcraft's half its scale), a pattern's orbit at its own
    auto loiterRadius = [&](const Plane& p) {
        return p.rotor ? std::max(3.0, 0.5 * p.scale()) : std::max(1500.0, 1.25 * LoiterBehavior::orbitRadiusM(perf(p), p.start.airspeedTrueMs));
    };
    auto orbitRadius = [&](const Plane& p) { return perf(p).turnRadiusM(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs); };
    run("fsim.guidance.loiter", 0.0,
        [&](const Plane& p) {
            BehaviorCommand b = behavior("loiter");
            if (p.rotor) b.params = {{"radius_m", loiterRadius(p)}};
            return w.submit(p.id, b).accepted();
        },
        [&](const Plane& p) { return (1.0 + 2.5 * kPi) * loiterRadius(p) / std::max(p.cruiseMs, 0.1) + 60.0; }, none,
        [&](const Plane& p, const Lows&) {
            const double r = groundDistance(*w.vehicleState(p.id), p.start);
            CHECK(std::abs(r - loiterRadius(p)) < 0.02 * loiterRadius(p)); // (the worst: 0.1 %)
        });
    run("fsim.guidance.pattern", 0.0,
        [&](const Plane& p) {
            const CommandResult r = w.submit(p.id, PatternCommand{});
            activity[p.id] = r.activity;
            least[p.id] = 0.0; // (here: the worst cross-track after the first lap)
            return r.accepted();
        },
        [&](const Plane& p) { return (1.0 + 2.5 * kPi) * orbitRadius(p) / std::max(p.cruiseMs, 0.1) + 60.0; },
        [&](const Plane& p) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (r.progress.laps >= 1 && std::isfinite(r.progress.crossTrackM)) least[p.id] = std::max(least[p.id], std::abs(r.progress.crossTrackM));
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            CHECK(r.live());
            CHECK(r.progress.laps >= 1);
            // (the worst: a wing 3.6 % of its radius, 7.9 % the Skua's 132 m; a rotorcraft 7.3 %)
            CHECK(least[p.id] < (p.rotor ? std::max(0.5, 0.1 * orbitRadius(p)) : std::max(20.0, 0.05 * orbitRadius(p))));
        });
    // a gentle S: a wing's six of its full-bank turns long (at least its scale), a rotorcraft's its scale
    auto curveLength = [&](const Plane& p) { return p.rotor ? p.scale() : std::max(p.scale(), 6.0 * fullBankRadius(p)); };
    run("fsim.guidance.curve", 0.0,
        [&](const Plane& p) {
            const double L = curveLength(p), c = std::cos(p.start.eulerRad[2]), s = std::sin(p.start.eulerRad[2]);
            const double north[6] = {0.0, 0.2 * L, 0.4 * L, 0.6 * L, 0.8 * L, L}, east[6] = {0.0, 0.0, 0.1 * L, 0.1 * L, 0.2 * L, 0.2 * L};
            BezierSegment seg;
            for (int i = 0; i < 6; ++i) seg.north[i] = north[i] * c - east[i] * s, seg.east[i] = north[i] * s + east[i] * c;
            const CommandResult r = w.submit(p.id, CurveCommand{}, Span<const BezierSegment>(&seg, 1));
            INFO("refused: " << reasonName(r.reason));
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        [&](const Plane& p) { return 1.5 * curveLength(p) / std::max(p.cruiseMs, 0.1) + 30.0; }, none, completed);
    run("fsim.guidance.hover", 0.0, [&](const Plane& p) { return w.submit(p.id, behavior("hover")).accepted(); }, secs(30.0, 30.0), none,
        [&](const Plane& p, const Lows&) {
            const auto& s = *w.vehicleState(p.id);
            CHECK(groundDistance(s, p.start) < 0.1); // (the worst: 0.02 m)
            CHECK(std::abs(s.altitudeMslM - p.start.altitudeMslM) < 0.1);
        });
    // aerobatics: an aileron roll completes; a loop completes, or ends honestly - refused for its entry
    // speed, or given up when the aircraft leaves its envelope (below its least airspeed it may then go)
    for (const double manoeuvre : {0.0, 1.0}) {
        const bool roll = manoeuvre == 0.0;
        run("fsim.guidance.aerobatics", 0.0,
            [&](const Plane& p) {
                BehaviorCommand b = behavior("aerobatics");
                b.params = {{"manoeuvre", manoeuvre}};
                const CommandResult r = w.submit(p.id, b);
                INFO((roll ? "aileron roll" : "loop") << " refused: " << reasonName(r.reason));
                if (roll) CHECK(r.accepted());
                else CHECK((r.accepted() || r.reason == Reason::PerformanceLimit));
                activity[p.id] = r.activity;
                return r.accepted();
            },
            secs(90.0, 90.0), none,
            [&](const Plane& p, const Lows&) {
                const ActivityRecord& r = *w.activity(activity[p.id]);
                INFO((roll ? "aileron roll: " : "loop: ") << activityStateName(r.state) << " " << reasonName(r.reason));
                if (roll) {
                    CHECK(r.state == ActivityState::Completed);
                    CHECK(std::abs(w.vehicleState(p.id)->altitudeMslM - p.start.altitudeMslM) < 100.0); // (the worst: 34 m)
                } else {
                    CHECK((r.state == ActivityState::Completed || (r.state == ActivityState::Failed && r.reason == Reason::BehaviorFailed)));
                }
            },
            roll);
    }

    // --- another vehicle of its type: followed at a range, fled, flown beside ----------------------------------
    auto followRange = [](const Plane& p) { return p.rotor ? std::max(3.0, 0.25 * p.scale()) : 300.0; };
    auto apart = [&](const Plane& p) { return groundDistance(*w.vehicleState(p.id), *w.vehicleState(p.helper)); };
    run("fsim.guidance.pursuit", 10.0,
        [&](const Plane& p) {
            BehaviorCommand b = behavior("pursuit", p.helper);
            b.params = {{"range_m", followRange(p)}};
            least[p.id] = gap[p.id] = apart(p);
            return w.submit(p.id, b).accepted();
        },
        secs(150.0, 150.0), [&](const Plane& p) { least[p.id] = std::min(least[p.id], apart(p)); },
        [&](const Plane& p, const Lows&) {
            INFO("from " << gap[p.id] << " m: " << apart(p) << " m, the least " << least[p.id] << " m");
            // it closed to the range (the widest: 7 % over), and no closer than 0.6 of it (the C172 dips to 72 %)
            CHECK(std::abs(apart(p) - followRange(p)) < 0.15 * followRange(p));
            CHECK(least[p.id] > 0.6 * followRange(p));
        });
    run("fsim.guidance.evade", 10.0,
        [&](const Plane& p) {
            gap[p.id] = apart(p);
            return w.submit(p.id, behavior("evade", p.helper)).accepted();
        },
        secs(90.0, 90.0), none,
        [&](const Plane& p, const Lows& lows) {
            CHECK(apart(p) > gap[p.id] + 10.0 * p.cruiseMs);                    // away from it
            CHECK(lows.agl > std::min(150.0, p.start.altitudeAglM) - 15.0);     // and never below its floor
        });
    // a wing 100 m behind its leader and 60 m right, from 5 s of its cruise behind; a rotorcraft its cruise's
    // metres behind and right, from three times as far behind
    auto slotAhead = [](const Plane& p) { return p.rotor ? -p.cruiseMs : -100.0; };
    auto slotRight = [](const Plane& p) { return p.rotor ? p.cruiseMs : 60.0; };
    for (const bool rotorcraft : {false, true}) {
        run("fsim.guidance.formation", rotorcraft ? 3.0 : 5.0,
            [&](const Plane& p) {
                if (p.rotor != rotorcraft) return false;
                BehaviorCommand b = behavior("formation", p.helper);
                b.params = {{"ahead_m", slotAhead(p)}, {"right_m", slotRight(p)}};
                return w.submit(p.id, b).accepted();
            },
            secs(150.0, 150.0), none,
            [&](const Plane& p, const Lows&) {
                const auto& leader = *w.vehicleState(p.helper);
                const double psi = leader.eulerRad[2];
                double north, east;
                offset(*w.vehicleState(p.id), leader.latitudeRad, leader.longitudeRad, north, east);
                const double slotNorth = slotAhead(p) * std::cos(psi) - slotRight(p) * std::sin(psi);
                const double slotEast = slotAhead(p) * std::sin(psi) + slotRight(p) * std::cos(psi);
                // (the worst: a wing 9.3 m off its slot, a rotorcraft 0.01 m)
                CHECK(std::hypot(north - slotNorth, east - slotEast) < (p.rotor ? 0.5 : 20.0));
            });
    }

    // --- every capability each aircraft advertises had its case ----------------------------------------------
    for (const auto& p : planes) {
        INFO(p.type << " (" << className(p.cls) << ")");
        const auto it = fleet.coverage().find(p.type);
        for (const std::string& c : p.offered) {
            INFO(c);
            CHECK((it != fleet.coverage().end() && it->second.count(c) == 1));
        }
    }
}
