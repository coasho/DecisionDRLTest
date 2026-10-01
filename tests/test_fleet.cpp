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
#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/Altimeter.h"
#include "fsim/BuiltinControllers.h"
#include "fsim/GuidanceModes.h"
#include "fsim/VehicleProfile.h"
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

    /// Every aircraft as a recovery begins (ADR-29 FA-10a): a wing `southM` south of where its standard condition puts it, 300 m
    /// up, heading north at its reference airspeed; a rotorcraft hovering `rotorSouthM` times its scale south of it, 30 m up -
    /// each settled on its velocity loop for 10 s, then let go.
    void approaching(double southM, double rotorSouthScales) {
        clear();
        for (auto& p : planes_) {
            const double back = p.rotor ? rotorSouthScales * p.scale() : southM;
            p.id = spawn(p, p.type + "-approaching-" + std::to_string(serial_), -back, p.rotor ? 0.0 : p.tasMs, false, p.rotor ? 30.0 : 300.0);
        }
        ++serial_;
        std::vector<ActivityId> held;
        for (auto& p : planes_) {
            const CommandResult r = w_.submit(p.id, still(p, *w_.vehicleState(p.id)));
            REQUIRE(r.accepted());
            held.push_back(r.activity);
        }
        w_.step(stepsFor(w_, 10.0));
        for (const ActivityId a : held) w_.cancel(a);
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

    std::uint32_t spawn(const Plane& p, const std::string& name, double aheadM, double tasMs, bool parked, double altitudeMslM = kHold) {
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
            s.initial.altitudeMslM = std::isnan(altitudeMslM) ? 150.0 : altitudeMslM;
            s.initial.airspeedTrueMs = 0.0;
            s.initial.pitchDeg = p.pitchDeg;
            s.initial.rollDeg = p.rollDeg;
        } else {
            s.initial.altitudeMslM = std::isnan(altitudeMslM) ? 3000.0 : altitudeMslM;
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
/// (lat, lon) in the plane square to the vertical at (lat0, lon0), north and east metres - as a frame's Cartesian offsets
/// are laid out (Frames.cpp): the central projection, in which great circles are straight.
void tangentPlane(double lat0, double lon0, double lat, double lon, double& north, double& east) {
    const double r = 6371008.8;
    const double o[3] = {std::cos(lat0) * std::cos(lon0), std::cos(lat0) * std::sin(lon0), std::sin(lat0)};
    const double n[3] = {-std::sin(lat0) * std::cos(lon0), -std::sin(lat0) * std::sin(lon0), std::cos(lat0)};
    const double e[3] = {-std::sin(lon0), std::cos(lon0), 0.0};
    const double u[3] = {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    const double along = u[0] * o[0] + u[1] * o[1] + u[2] * o[2];
    north = r * (u[0] * n[0] + u[1] * n[1] + u[2] * n[2]) / along;
    east = r * (u[0] * e[0] + u[1] * e[1] + u[2] * e[2]) / along;
}

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
    std::map<std::uint32_t, double> least, gap, predicted, onBoard, atEnd;
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

    // --- a launch (ADR-29 FA-9a, LCH-01, LCH-04): parked, each on a runway of its own 3,500 m north - a wing takes off
    // within half its width of the centre line and climbs out; a rotorcraft lifts to its hover over where it stands ---------
    {
        fleet.park();
        std::map<std::uint32_t, ActivityId> launched;
        std::map<std::uint32_t, double> cross;
        std::map<std::uint32_t, sim::VehicleState> ended;
        for (const auto& p : planes) {
            if (!p.offered.count("fsim.guidance.launch")) continue;
            INFO(p.type << ": launch");
            const double ground = p.start.altitudeMslM - p.start.altitudeAglM;
            Runway r;
            r.id = 3;
            r.takeoff.start = RunwayPoint{p.start.latitudeRad, p.start.longitudeRad, ground};
            r.takeoff.limit = RunwayPoint{p.start.latitudeRad + 3500.0 / kEarthM, p.start.longitudeRad, ground};
            Airfield field;
            field.id = 7;
            field.runways = {r};
            REQUIRE(w.loadAirfield(p.id, field) == Reason::None);
            BehaviorCommand b = behavior("launch");
            b.params = {{"airfield", 7.0}, {"runway", 3.0}};
            const CommandResult c = w.submit(p.id, b);
            CHECK(c.accepted());
            if (c.accepted()) launched[p.id] = c.activity, cross[p.id] = 0.0;
        }
        for (double t = 0.0; t < 240.0 && ended.size() < launched.size(); t += 1.0)
            fleet.fly(1.0, [&] {
                for (const auto& [id, a] : launched) {
                    if (ended.count(id)) continue;
                    const auto& s = *w.vehicleState(id);
                    const auto& p = *std::find_if(planes.begin(), planes.end(), [id = id](const Plane& x) { return x.id == id; });
                    double north, east;
                    offset(s, p.start.latitudeRad, p.start.longitudeRad, north, east);
                    if (s.onGround) cross[id] = std::max(cross[id], std::abs(east));
                    if (!w.activity(a)->live()) ended[id] = s;
                }
            });
        for (const auto& p : planes) {
            if (!launched.count(p.id)) continue;
            INFO(p.type << " (" << className(p.cls) << "): launch " << activityStateName(w.activity(launched[p.id])->state));
            CHECK(w.activity(launched[p.id])->state == ActivityState::Completed);
            REQUIRE(ended.count(p.id));
            const sim::VehicleState& s = ended[p.id];
            if (p.rotor) {
                // its c.g. 10 m over the ground it stood on, over where it stood (the worst: 0.67 m)
                const double up = s.altitudeMslM - (p.start.altitudeMslM - p.start.altitudeAglM + 10.0);
                CHECK(std::hypot(groundDistance(s, p.start), up) < 1.0);
            } else {
                CHECK(cross[p.id] < 22.5);        // (the worst: the U-2S's 2.8 m, on its wingtip skid)
                CHECK(s.altitudeAglM > 400.0);    // climbed out to 450 m, cleaned up
                if (p.offered.count("fsim.support.gear")) CHECK(s.gearPosition < 0.1);
            }
            fleet.covered(p, "fsim.guidance.launch");
        }
    }
    // --- a rejected takeoff (ADR-29 FA-9b, LCH-02): each wing's launch canceled by its policy at half its rotation speed -
    // FA's own activity takes the rest and stops it on the runway ---------------------------------------------------------
    {
        fleet.park();
        std::map<std::uint32_t, ActivityId> launched, own;
        std::map<std::uint32_t, double> vr, cross;
        std::map<std::uint32_t, sim::VehicleState> ended;
        for (const auto& p : planes) {
            if (p.rotor || !p.offered.count("fsim.guidance.launch")) continue;
            const double ground = p.start.altitudeMslM - p.start.altitudeAglM;
            Runway r;
            r.id = 3;
            r.takeoff.start = RunwayPoint{p.start.latitudeRad, p.start.longitudeRad, ground};
            r.takeoff.limit = RunwayPoint{p.start.latitudeRad + 3500.0 / kEarthM, p.start.longitudeRad, ground};
            Airfield field;
            field.id = 7;
            field.runways = {r};
            REQUIRE(w.loadAirfield(p.id, field) == Reason::None);
            BehaviorCommand b = behavior("launch");
            b.params = {{"airfield", 7.0}, {"runway", 3.0}};
            const CommandResult c = w.submit(p.id, b);
            REQUIRE(c.accepted());
            launched[p.id] = c.activity, cross[p.id] = 0.0;
            const VehicleProfile& pr = *w.profile(p.id);
            const double stall = std::isfinite(pr.performance.stallFlapsCasMs) ? pr.performance.stallFlapsCasMs : pr.performance.stallCasMs;
            vr[p.id] = 1.1 * (std::isfinite(stall) ? stall : p.minCasMs);
        }
        for (double t = 0.0; t < 180.0 && ended.size() < launched.size(); t += 1.0)
            fleet.fly(1.0, [&] {
                for (const auto& [id, a] : launched) {
                    if (ended.count(id)) continue;
                    const auto& s = *w.vehicleState(id);
                    const auto& p = *std::find_if(planes.begin(), planes.end(), [id = id](const Plane& x) { return x.id == id; });
                    double north, east;
                    offset(s, p.start.latitudeRad, p.start.longitudeRad, north, east);
                    cross[id] = std::max(cross[id], std::abs(east));
                    if (!own.count(id) && s.airspeedCalibratedMs >= 0.5 * vr[id]) {
                        const CommandResult c = w.cancel(a);
                        CHECK(c.status == CommandStatus::Canceled);
                        own[id] = c.other;
                    }
                    if (own.count(id) && !(own[id] && w.activity(own[id])->live())) ended[id] = s;
                }
            });
        for (const auto& p : planes) {
            if (!launched.count(p.id)) continue;
            INFO(p.type << " (" << className(p.cls) << "): its launch canceled at half its rotation speed");
            REQUIRE(own.count(p.id));
            REQUIRE(own[p.id] != 0);
            CHECK(w.activity(own[p.id])->source == Source::Autopilot);
            CHECK(w.activity(own[p.id])->state == ActivityState::Completed);
            REQUIRE(ended.count(p.id));
            const sim::VehicleState& s = ended[p.id];
            double north, east;
            offset(s, p.start.latitudeRad, p.start.longitudeRad, north, east);
            CHECK(groundSpeed(s) < 0.5); // stopped on the runway (the most it rolled: the C-17A, 660 m; the most off its line: the U-2S, 1.97 m)
            CHECK((north > 0.0 && north < 3500.0));
            CHECK(cross[p.id] < 22.5);
        }
    }

    // --- a taxi (ADR-29 FA-9c, WPT-26): parked, each wing taxies north 150 m, east 200, north 200 and west 150, within 2 m of its
    // path - its legs, and each corner an arc a quarter wider than its tightest turn on its wheels - and stops at its end -----------
    {
        fleet.park();
        const std::vector<std::pair<double, double>> path = {{0.0, 0.0}, {150.0, 0.0}, {150.0, 200.0}, {350.0, 200.0}, {350.0, 50.0}};
        std::map<std::uint32_t, ActivityId> taxied;
        std::map<std::uint32_t, double> worst, radius;
        for (const auto& p : planes) {
            if (!p.offered.count("fsim.guidance.taxi")) continue;
            INFO(p.type << ": taxi");
            BehaviorCommand b = behavior("taxi");
            b.params = {{"speed_ms", 8.0}};
            for (std::size_t i = 1; i < path.size(); ++i) {
                PositionCommand at;
                at.latitudeRad = p.start.latitudeRad + path[i].first / kEarthM;
                at.longitudeRad = p.start.longitudeRad + path[i].second / (kEarthM * std::cos(p.start.latitudeRad));
                b.points.push_back(at);
            }
            const CommandResult c = w.submit(p.id, b);
            CHECK(c.accepted());
            if (!c.accepted()) continue;
            taxied[p.id] = c.activity, worst[p.id] = 0.0;
            const double tightest = w.profile(p.id)->envelope.groundTurnRadiusM;
            radius[p.id] = std::max(1.25 * (std::isfinite(tightest) ? tightest : 15.0), 8.0);
        }
        std::size_t live = taxied.size();
        for (double t = 0.0; t < 300.0 && live > 0; t += 1.0)
            fleet.fly(1.0, [&] {
                live = 0;
                for (const auto& [id, a] : taxied) {
                    if (!w.activity(a)->live()) continue;
                    ++live;
                    const auto& p = *std::find_if(planes.begin(), planes.end(), [id = id](const Plane& x) { return x.id == id; });
                    double north, east;
                    offset(*w.vehicleState(id), p.start.latitudeRad, p.start.longitudeRad, north, east);
                    if (north > 75.0 || east > 1.0) worst[id] = std::max(worst[id], taxiPathDistance(path, radius[id], north, east));
                }
            });
        for (const auto& p : planes) {
            if (!taxied.count(p.id)) continue;
            INFO(p.type << " (" << className(p.cls) << "): taxi " << activityStateName(w.activity(taxied[p.id])->state));
            CHECK(w.activity(taxied[p.id])->state == ActivityState::Completed);
            CHECK(worst[p.id] < 2.0); // (the worst: the U-2S, 1.62 m; the KC-46A 1.27, the B-52H 1.20)
            double north, east;
            offset(*w.vehicleState(p.id), p.start.latitudeRad, p.start.longitudeRad, north, east);
            CHECK(std::hypot(north - 350.0, east - 50.0) < 1.0);
            CHECK(w.vehicleState(p.id)->onGround);
            fleet.covered(p, "fsim.guidance.taxi");
        }
    }

    // --- a route that starts on the ground (ADR-29 FA-9d): parked, each wing's route taxies north 150 m and east 200 to its
    // runway's start, takes off along its runway points 3,000 m north, and flies two points 600 m up, a right turn between -----
    {
        fleet.park();
        constexpr double kStartN = 250.0, kStartE = 200.0, kUpM = 600.0;
        const std::vector<std::pair<double, double>> taxi = {{0.0, 0.0}, {150.0, 0.0}, {150.0, 200.0}, {kStartN, kStartE}};
        std::map<std::uint32_t, ActivityId> routed;
        std::map<std::uint32_t, double> worst, cross, radius, lastLegS;
        for (const auto& p : planes) {
            if (p.rotor || w.support(p.id, "fsim.guidance.route/waypoint_type/runway")->support != Support::Supported) continue;
            INFO(p.type << ": a route from the ground");
            const double ground = p.start.altitudeMslM - p.start.altitudeAglM;
            const auto at = [&](double north, double east, WaypointType type) {
                Waypoint x;
                x.latitudeRad = p.start.latitudeRad + north / kEarthM;
                x.longitudeRad = p.start.longitudeRad + east / (kEarthM * std::cos(p.start.latitudeRad));
                if (type == WaypointType::NavOnly) x.altitudeM = ground + kUpM, x.altitudeReference = static_cast<double>(AltitudeReference::Msl);
                else x.waypointType = static_cast<double>(type);
                return x;
            };
            const std::vector<Waypoint> route = {at(150.0, 0.0, WaypointType::Taxi), at(150.0, 200.0, WaypointType::Taxi),
                                                 at(kStartN, kStartE, WaypointType::RunwayStart), at(kStartN + 3000.0, kStartE, WaypointType::RunwayLimit),
                                                 at(5000.0, kStartE, WaypointType::NavOnly), at(5000.0, 9000.0, WaypointType::NavOnly)};
            const CommandResult c = w.submit(p.id, RouteCommand{}, Span<const Waypoint>(route.data(), route.size()));
            INFO(reasonName(c.reason) << " at " << c.index);
            CHECK(c.accepted());
            if (!c.accepted()) continue;
            routed[p.id] = c.activity, worst[p.id] = cross[p.id] = 0.0, lastLegS[p.id] = 0.0;
            const double tightest = w.profile(p.id)->envelope.groundTurnRadiusM;
            radius[p.id] = std::max(1.25 * (std::isfinite(tightest) ? tightest : 15.0), 8.0);
        }
        // flown until each has flown its last leg a minute (its first point passed, its turn made)
        const auto done = [&] {
            for (const auto& [id, a] : routed)
                if (w.activity(a)->live() && lastLegS[id] < 60.0) return false;
            return true;
        };
        for (double t = 0.0; t < 700.0 && !done(); t += 1.0)
            fleet.fly(1.0, [&] {
                for (const auto& [id, a] : routed) {
                    const auto& s = *w.vehicleState(id);
                    const auto& p = *std::find_if(planes.begin(), planes.end(), [id = id](const Plane& x) { return x.id == id; });
                    double north, east;
                    offset(s, p.start.latitudeRad, p.start.longitudeRad, north, east);
                    if (s.onGround && north < kStartN - 2.0 && (north > 75.0 || east > 1.0))
                        worst[id] = std::max(worst[id], taxiPathDistance(taxi, radius[id], north, east));
                    if (s.onGround && north > kStartN + 20.0) cross[id] = std::max(cross[id], std::abs(east - kStartE));
                    if (w.activity(a)->progress.segment == 5 && !s.onGround) lastLegS[id] += w.dt() * w.frameSkip();
                }
            });
        for (const auto& p : planes) {
            if (!routed.count(p.id)) continue;
            const ActivityRecord& a = *w.activity(routed[p.id]);
            INFO(p.type << " (" << className(p.cls) << "): a route from the ground " << activityStateName(a.state) << " " << reasonName(a.reason));
            CHECK((a.state == ActivityState::Active || a.state == ActivityState::Completed));
            CHECK(worst[p.id] < 2.0);   // (the worst: the U-2S, 1.62 m; the KC-46A 1.27, the B-52H 1.20)
            CHECK(cross[p.id] < 22.5);  // (the U-2S's 2.73 m; the rest within 0.31)
            const sim::VehicleState& s = *w.vehicleState(p.id);
            CHECK(!s.onGround);
            CHECK(std::abs(s.altitudeMslM - (p.start.altitudeMslM - p.start.altitudeAglM) - kUpM) < 60.0); // at its points' height (598.8 to 607.9 m)
            fleet.covered(p, "fsim.guidance.route");
        }
    }

    // --- a recovery (ADR-29 FA-10a, RCV-01, RCV-04): each to a runway of its own 3,000 m north, its threshold 20 km ahead of a
    // wing 300 m up (a rotorcraft's 20 s of its cruise ahead, hovering 30 m up) - a wing touches down in the zone and stops on
    // the runway, a rotorcraft lands on its spot 60 m beyond the threshold --------------------------------------------------
    {
        constexpr double kAheadM = 20000.0, kRotorScales = 1.0, kLengthM = 3000.0;
        fleet.approaching(kAheadM, kRotorScales);
        struct Landing {
            ActivityId activity = 0;
            double lat0 = 0.0, lon0 = 0.0, elevation = 0.0, sink = 0.0, worstCross = 0.0, hop = 0.0, aglAtTouch = 0.0;
            bool touched = false, ended = false;
            double endSpeed = 0.0; ///< its ground speed as its activity ended (then the vehicle default flies it)
            double touchAlong = 0.0, touchCross = 0.0, touchSink = 0.0;
        };
        std::map<std::uint32_t, Landing> landing;
        for (const auto& p : planes) {
            if (!p.offered.count("fsim.guidance.recovery")) continue;
            INFO(p.type << ": recovery");
            Landing l;
            l.elevation = p.start.altitudeMslM - p.start.altitudeAglM;
            const double ahead = p.rotor ? kRotorScales * p.scale() : kAheadM;
            l.lat0 = p.start.latitudeRad + ahead / kEarthM, l.lon0 = p.start.longitudeRad;
            Runway r;
            r.id = 3;
            r.landing.start = r.landing.threshold = RunwayPoint{l.lat0, l.lon0, l.elevation};
            r.landing.limit = RunwayPoint{l.lat0 + kLengthM / kEarthM, l.lon0, l.elevation};
            Airfield field;
            field.id = 7;
            field.runways = {r};
            REQUIRE(w.loadAirfield(p.id, field) == Reason::None);
            BehaviorCommand b = behavior("recovery");
            b.params = {{"airfield", 7.0}, {"runway", 3.0}};
            const CommandResult c = w.submit(p.id, b);
            INFO(reasonName(c.reason) << " at " << c.index);
            CHECK(c.accepted());
            if (c.accepted()) l.activity = c.activity, landing[p.id] = l;
        }
        const auto live = [&] {
            for (const auto& [id, l] : landing)
                if (w.activity(l.activity)->live()) return true;
            return false;
        };
        for (double t = 0.0; t < 3000.0 && live(); t += 1.0) // (two approaches and their missed approaches: 4.54)
            fleet.fly(1.0, [&] {
                for (auto& [id, l] : landing) {
                    const sim::VehicleState& s = *w.vehicleState(id);
                    double north, east;
                    offset(s, l.lat0, l.lon0, north, east);
                    if (!l.touched && !s.onGround) l.sink = s.velocityNedMs[2];
                    if (!l.touched && s.onGround)
                        l.touched = true, l.touchAlong = north, l.touchCross = east, l.touchSink = l.sink, l.aglAtTouch = s.altitudeAglM;
                    if (l.touched && !s.onGround) l.hop = std::max(l.hop, s.altitudeAglM - l.aglAtTouch);
                    if (l.touched) l.worstCross = std::max(l.worstCross, std::abs(east));
                    if (!l.ended && !w.activity(l.activity)->live()) l.ended = true, l.endSpeed = groundSpeed(s);
                }
            });
        for (const auto& p : planes) {
            if (!landing.count(p.id)) continue;
            const Landing& l = landing[p.id];
            const ActivityRecord& a = *w.activity(l.activity);
            INFO(p.type << " (" << className(p.cls) << "): recovery " << activityStateName(a.state) << " " << reasonName(a.reason) << "; touched down "
                        << l.touchAlong << " m along, " << l.touchCross << " m across, sinking " << l.touchSink << " m/s; bounced " << l.hop
                        << " m; worst " << l.worstCross << " m across");
            // the B-52H, no airbrakes yet, floats past its touchdown zone; the C-130J, its flight idle's thrust about its drag,
            // comes down its glide slope fast and bounces: each goes around from both its approaches and fails (4.54; the backlog's
            // B-1, FA-10c's drag devices)
            if (p.type == "b52h" || p.type == "c130j") {
                CHECK(a.state == ActivityState::Failed);
                CHECK(a.reason == Reason::LandingAbandoned);
                continue;
            }
            CHECK(a.state == ActivityState::Completed);
            CHECK(l.touched);
            CHECK(l.endSpeed < 0.5); // (stopped)
            if (p.rotor) {
                CHECK(std::hypot(l.touchAlong - 60.0, l.touchCross) < 3.0); // (on its spot: the worst, the UH-1H's 1.4 m)
                CHECK(l.touchSink < 1.0);
            } else {
                CHECK((l.touchAlong > 0.0 && l.touchAlong < 900.0)); // in the touchdown zone (221 to 677 m)
                CHECK(l.touchSink < 3.2);                             // (the worst: the Mirage 2000's 3.14 m/s)
                CHECK(l.worstCross < 22.5);                           // on the runway
                CHECK(l.hop < 3.0);                                   // (the RQ-4B's 2.3 m, a glider)
            }
            fleet.covered(p, "fsim.guidance.recovery");
        }
        for (const auto& p : planes)
            if (p.type == "b52h" || p.type == "c130j") fleet.covered(p, "fsim.guidance.recovery"); // (flown, gone around: above)
    }

    // --- guidance ------------------------------------------------------------------------------------
    // a new heading, a quarter turn right, and a new altitude: a wing 200 m higher - the fleet climb case (ADR-29
    // FA-3d): energy management keeps its calibrated airspeed at 1.1 times its least or more, where a C172 once
    // traded its speed down to its least and crept - a rotorcraft a tenth of its scale higher (2 to 20 m)
    auto climb = [](const Plane& p) { return p.rotor ? std::clamp(0.1 * p.scale(), 2.0, 20.0) : 200.0; };
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
            secs(200.0, 60.0), none,
            [&](const Plane& p, const Lows& lows) {
                const auto& s = *w.vehicleState(p.id);
                // (the worst: 0.14 deg held, 0.11 in an hsa, which trims out what its loops leave - the C172, still climbing; a
                // wing still closing on its altitude, 4.6 m off - the B-52H - a rotorcraft on it; the least airspeed 1.16 times
                // the least - the C172, holding the margin as it climbs - the next 1.56)
                CHECK(headingOffDeg(s, p.start.eulerRad[2] + 0.5 * kPi) < 1.0);
                CHECK(std::abs(s.altitudeMslM - (p.start.altitudeMslM + climb(p))) < (p.rotor ? 0.5 : 20.0));
                if (!p.rotor && std::isfinite(p.minCasMs)) CHECK(lows.cas >= 1.1 * p.minCasMs); // (no speed traded below the margin)
            });
    }
    // a speed optimisation (ADR-29 FA-3e, HSA-05): its heading and height held, flown at the performance tables' best
    // speed at the altitude and weight now - within 5 % of it (FA-3's acceptance)
    for (const SpeedOptimization o : {SpeedOptimization::MaxEndurance, SpeedOptimization::LongRangeCruise}) {
        const char* name = o == SpeedOptimization::MaxEndurance ? "max_endurance" : "long_range_cruise";
        run("fsim.guidance.hsa", 0.0,
            [&](const Plane& p) {
                HsaCommand h;
                h.headingRad = p.start.eulerRad[2], h.speedOptimization = static_cast<double>(o);
                return w.submit(p.id, h).accepted();
            },
            secs(240.0, 90.0), none,
            [&](const Plane& p, const Lows&) {
                const auto& s = *w.vehicleState(p.id);
                const double best = optimalTasMs(&w.profile(p.id)->tables, static_cast<double>(o), s.altitudeMslM, s.fuelKg);
                // (a rotorcraft's airspeed along its nose, as its loops fly it and its tables have it; a wing's true airspeed)
                const double flown = p.rotor ? s.airspeedTrueMs * std::cos(s.alphaRad) * std::cos(s.betaRad) : s.airspeedTrueMs;
                // (the worst: the Crazyflie's best range, 0.84 % short; every wing's within 0.01 %, its height within 1.1 m)
                INFO(name << ": " << flown << " m/s against " << best);
                REQUIRE(std::isfinite(best));
                CHECK(std::abs(flown - best) < std::max(0.05 * best, 0.5)); // (the Crazyflie's best endurance: its hover)
            });
    }
    // beyond its top level speed (docs/flight-autonomy.md, 4.48): a rotorcraft's airspeed is held to its tables' top at the
    // altitude and weight - refused under Reject, clamped to it otherwise - and it flies it there along its nose, its height
    // held (a wing's is held to its envelope and profile, as before)
    std::map<std::uint32_t, double> topMs;
    run("fsim.guidance.hsa", 0.0,
        [&](const Plane& p) {
            if (!p.rotor) return false;
            topMs[p.id] = topTasMs(&w.profile(p.id)->tables, p.start.altitudeMslM, w.vehicleState(p.id)->fuelKg);
            REQUIRE(std::isfinite(topMs[p.id]));
            HsaCommand h;
            h.headingRad = p.start.eulerRad[2], h.speed = 1.5 * topMs[p.id], h.speedReference = static_cast<double>(SpeedReference::TrueAirspeed);
            CommandOptions reject;
            reject.range = RangePolicy::Reject, reject.validateOnly = true;
            const CommandResult refused = w.submit(p.id, h, reject);
            CHECK((refused.reason == Reason::PerformanceLimit && refused.index == 2 && refused.constraint == Constraint::MaxAirspeed));
            const CommandResult r = w.submit(p.id, h);
            CHECK((r.accepted() && (r.flags & kClamped) != 0 && r.constraint == Constraint::MaxAirspeed));
            return r.accepted();
        },
        secs(0.0, 240.0), none,
        [&](const Plane& p, const Lows&) {
            const auto& s = *w.vehicleState(p.id);
            const double flown = s.airspeedTrueMs * std::cos(s.alphaRad) * std::cos(s.betaRad);
            INFO("flown " << flown << " m/s along the nose, its top " << topMs[p.id] << ", " << s.altitudeMslM - p.start.altitudeMslM << " m off its height");
            // (the worst: the IRIS+ and the UH-1H 0.011 % short of it, every height within 2 mm. On the way, the Crazyflie's
            // velocity loop takes over two minutes to it, its integral carrying the whole of its tilt there, and the UH-60A
            // sags 96 m at full collective, back on its height 70 s after the command)
            CHECK(std::abs(flown - topMs[p.id]) < 0.01 * topMs[p.id]);
            CHECK(std::abs(s.altitudeMslM - p.start.altitudeMslM) < 0.5);
        });
    // a barometric altitude (ADR-29 FA-4b, HSA-07): at the standard setting the altimeter reads the geopotential height;
    // set 20 hPa low, it reads 167 m less, and an hsa as high again as the climb case climbs on it, to its isobar
    std::map<std::uint32_t, double> onAltimeter;
    run("fsim.guidance.hsa", 0.0,
        [&](const Plane& p) {
            const double h = w.vehicleState(p.id)->altitudeMslM;
            const StateData d = w.stateData(p.id);
            CHECK(std::abs(d.indicatedAltitudeM - 6356766.0 * h / (6356766.0 + h)) < 0.05);
            // (the rest of its state data, ADR-29 FA-4c: the calm air it measures - the worst the Crazyflie's, 7.4e-5 m/s,
            // its air data at a few metres a second; every other within 1e-6 - and its orientation's rates)
            CHECK(std::sqrt(d.windNorthMs * d.windNorthMs + d.windEastMs * d.windEastMs + d.windDownMs * d.windDownMs) < 1e-3);
            CHECK((d.wanderAngleRad == 0.0 && std::isfinite(d.yawRateRadS) && std::isfinite(d.rollAccelerationRadS2)));
            REQUIRE(w.setQnh(p.id, 99325.0) == Reason::None);
            HsaCommand hsa;
            hsa.headingRad = p.start.eulerRad[2], hsa.altitudeReference = static_cast<double>(AltitudeReference::Barometric);
            hsa.altitudeM = onAltimeter[p.id] = w.stateData(p.id).indicatedAltitudeM + climb(p);
            return w.submit(p.id, hsa).accepted();
        },
        secs(200.0, 60.0), none,
        [&](const Plane& p, const Lows&) {
            const double reads = w.stateData(p.id).indicatedAltitudeM;
            const Altimeter altimeter{Air{}, 99325.0};
            INFO("it reads " << reads << " m, asked " << onAltimeter[p.id]);
            // (the worst: the Mirage 2000 3.3 m high, still closing on it, the Typhoon 1.2 m; a rotorcraft within a millimetre)
            CHECK(std::abs(reads - onAltimeter[p.id]) < (p.rotor ? 0.5 : 20.0));
            CHECK(std::abs(w.vehicleState(p.id)->altitudeMslM - barometricMslM(altimeter, reads)) < 1e-6);
            REQUIRE(w.setQnh(p.id, Altimeter::kStandardPa) == Reason::None); // (as it was, for the cases after)
        });
    // a magnetic heading (ADR-29 FA-4d, HSA-03): a quarter turn right from magnetic north, flown as the true heading the
    // declination where the aircraft is turns it to (the world's clock unset: the model's epoch, 2025.0)
    std::map<std::uint32_t, double> magneticAsked;
    run("fsim.guidance.hsa", 0.0,
        [&](const Plane& p) {
            const double declination = w.stateData(p.id).declinationRad;
            HsaCommand hsa;
            hsa.headingRad = magneticAsked[p.id] = std::remainder(p.start.eulerRad[2] + 0.5 * kPi - declination, 2.0 * kPi);
            hsa.directionReference = static_cast<double>(DirectionReference::MagneticNorth);
            return w.submit(p.id, hsa).accepted();
        },
        secs(200.0, 60.0), none,
        [&](const Plane& p, const Lows&) {
            const StateData d = w.stateData(p.id);
            const auto& s = *w.vehicleState(p.id);
            INFO("magnetic " << d.magneticHeadingRad / kDeg << " deg, asked " << magneticAsked[p.id] / kDeg);
            // (commanded turned by the declination where it is - 0.008 deg from the declination now at worst, moving between
            // the 10 s it is refreshed at - with a wing's heading trim, the mode's integral on what its loops leave, beside it:
            // the velocity level's heading the worst 0.057 deg from it, the F-35A's. Flown: the magnetic heading asked within
            // 0.009 deg, the true heading 0.14 deg from where it began - the Su-25's, its declination moved as it flew)
            CHECK(std::abs(std::remainder(w.commandState(p.id).headingRad - (magneticAsked[p.id] + d.declinationRad), 2.0 * kPi)) < 0.12 * kDeg);
            CHECK(headingOffDeg(s, p.start.eulerRad[2] + 0.5 * kPi) < 1.0);
            CHECK(std::abs(std::remainder(d.magneticHeadingRad - magneticAsked[p.id], 2.0 * kPi)) < 1.0 * kDeg);
        });
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
    // the same route as A-GRA's route plan (ADR-29 FA-7a, RPL-01, RPL-02, RPL-10): validated first, in calm air and modified
    // to validate - held to the aircraft's limits, as it will be flown - as MA would before sending it (FA-7c, RPL-06);
    // prepared for upload, published, uploaded, prepared for activation and activated - flown as the route is, its plan's
    // execution complete at its end
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            RoutePlan plan;
            plan.id = 1, plan.version = 1;
            for (const PositionCommand& q : square(p)) {
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                plan.waypoints.push_back(wp);
            }
            PlanValidation calm;
            calm.windNorthMs = calm.windEastMs = 0.0, calm.modifyToValidate = true;
            const PlanValidationResult checked = w.validatePlan(p.id, plan, calm);
            INFO("its validation: " << reasonName(checked.check.reason) << " at " << checked.check.index);
            CHECK((checked.valid && checked.check.status == CommandStatus::Valid));
            REQUIRE(w.planCommand(p.id, 1, PlanCommand::PrepareForUpload).completed);
            REQUIRE(w.publishPlan(p.id, plan) == Reason::None);
            REQUIRE(w.planCommand(p.id, 1, PlanCommand::Upload).completed);
            const PlanCommandResult ready = w.planCommand(p.id, 1, PlanCommand::PrepareForActivation);
            INFO("its preparation failed: " << reasonName(ready.reason));
            CHECK(ready.completed);
            const PlanCommandResult r = w.planCommand(p.id, 1, PlanCommand::Activate);
            INFO("its activation failed: " << reasonName(r.reason));
            CHECK(r.completed);
            activity[p.id] = r.check.activity;
            return r.completed;
        },
        aroundSquare, none,
        [&](const Plane& p, const Lows& lows) {
            completed(p, lows);
            const auto s = w.planStatus(p.id, 1);
            REQUIRE(s.has_value());
            CHECK((s->state == PlanState::Activated && s->execution == PlanExecution::Complete && s->activity == activity[p.id]));
        });
    // a must fly (ADR-29 FA-8a, MFY-01, MFY-07): a point a leg ahead and half a leg to the right, at its altitude,
    // approached from the east - the aircraft coming from the south-west, round through the points laid out to come
    // from within 80 to 100 degrees - flown over and passed
    std::map<std::uint32_t, double> closest, cameFrom;
    auto mustFlyPoint = [&](const Plane& p) { return pointFrom(p.start, leg(p), 0.5 * leg(p), p.start.altitudeMslM, 0.0); };
    run("fsim.guidance.must_fly", 0.0,
        [&](const Plane& p) {
            const PositionCommand q = mustFlyPoint(p);
            MustFlyCommand c;
            c.location = static_cast<double>(MustFlyLocation::Point);
            c.latitudeRad = q.latitudeRad, c.longitudeRad = q.longitudeRad, c.altitudeM = q.altitudeMslM;
            c.ingressMinRad = 80.0 * kDeg, c.ingressMaxRad = 100.0 * kDeg;
            const CommandResult r = w.submit(p.id, c);
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            closest[p.id] = kInf;
            return r.accepted();
        },
        [&](const Plane& p) { return 5.0 * leg(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) + 60.0; }, // (done by 0.71 of it at most: the UH-1H)
        [&](const Plane& p) {
            const ActivityRecord* r = w.activity(activity[p.id]);
            if (!r || !r->live()) return;
            const PositionCommand q = mustFlyPoint(p);
            const auto& s = *w.vehicleState(p.id);
            const double north = (s.latitudeRad - q.latitudeRad) * kEarthM, east = (s.longitudeRad - q.longitudeRad) * kEarthM * std::cos(q.latitudeRad);
            if (std::hypot(north, east) < closest[p.id])
                closest[p.id] = std::hypot(north, east), cameFrom[p.id] = std::remainder(std::atan2(-s.velocityNedMs[1], -s.velocityNedMs[0]), 2.0 * kPi);
        },
        [&](const Plane& p, const Lows& lows) {
            completed(p, lows);
            // (the worst of the fleet: flown over 34.5 m off, the Gripen's - the other fly-by-wire jets' 14 to 30 m, the direct
            // wings' at most 9.3 m, the UH-60A's 3.3 m, the multirotors' 0.17 m; come from 93.4 to 95.5 deg, aimed 95)
            const double off = p.cls == Class::FlyByWire ? 70.0 : p.cls == Class::Direct ? 20.0 : p.cls == Class::Helicopter ? 7.0 : 0.5;
            CHECK(closest[p.id] < off);
            CHECK(cameFrom[p.id] > 80.0 * kDeg);
            CHECK(cameFrom[p.id] < 100.0 * kDeg);
        });
    // a must fly into a zone (ADR-29 FA-8b1, MFY-04): a square half a leg across, its centre a leg ahead and half a leg to the
    // right, spanning a band from 100 m below the aircraft to 100 m above it (a rotorcraft's 20 m) - entered, and completed
    // once in it
    std::map<std::uint32_t, double> enteredAt, heightAt;
    auto zoneOf = [&](const Plane& p) {
        const double half = 0.25 * leg(p), band = p.rotor ? 20.0 : 100.0;
        OpZone z;
        z.shape = static_cast<double>(ZoneShape::Polygon);
        for (const auto& [n, e] : {std::pair{-half, -half}, {half, -half}, {half, half}, {-half, half}}) {
            const PositionCommand q = pointFrom(p.start, leg(p) + n, 0.5 * leg(p) + e, 0.0, 0.0);
            ZoneVertex v;
            v.latitudeRad = q.latitudeRad, v.longitudeRad = q.longitudeRad;
            z.vertices.push_back(v);
        }
        z.altitudeMinM = p.start.altitudeMslM - band, z.altitudeMaxM = p.start.altitudeMslM + band;
        return z;
    };
    run("fsim.guidance.must_fly", 0.0,
        [&](const Plane& p) {
            MustFlyCommand c;
            c.location = static_cast<double>(MustFlyLocation::Zone);
            const CommandResult r = w.submit(p.id, c, zoneOf(p));
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            enteredAt[p.id] = std::numeric_limits<double>::quiet_NaN();
            return r.accepted();
        },
        [&](const Plane& p) { return 4.0 * leg(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) + 60.0; },
        [&](const Plane& p) {
            const ActivityRecord* r = w.activity(activity[p.id]);
            if (!std::isnan(enteredAt[p.id]) || !r || r->live()) return;
            const auto& s = *w.vehicleState(p.id);
            const PositionCommand centre = pointFrom(p.start, leg(p), 0.5 * leg(p), 0.0, 0.0);
            const double north = (s.latitudeRad - centre.latitudeRad) * kEarthM, east = (s.longitudeRad - centre.longitudeRad) * kEarthM * std::cos(centre.latitudeRad);
            enteredAt[p.id] = std::max(std::abs(north), std::abs(east)) - 0.25 * leg(p); // (how far outside its edge it completed: <= 0)
            heightAt[p.id] = s.altitudeMslM - p.start.altitudeMslM;
        },
        [&](const Plane& p, const Lows& lows) {
            completed(p, lows);
            // in it as it completed (a C-17A 30 m inside its edge, a CF2 0.02 m; 0.5 % of the half for this flat measure)
            // and in its band (a Mirage 2000 19 m above where it began)
            CHECK(enteredAt[p.id] <= 0.005 * 0.25 * leg(p));
            CHECK(std::abs(heightAt[p.id]) <= (p.rotor ? 20.0 : 100.0));
        });
    // a must fly through a corridor (ADR-29 FA-8b2, MFY-05): from half a leg ahead, a leg on, then a leg turned 60 degrees
    // right, half a leg wide either side - flown through: within its widths once past its first vertex, and completed as its
    // last vertex is passed
    std::map<std::uint32_t, double> offLine, fromEnd;
    std::map<std::uint32_t, bool> pastFirst;
    auto cornersOf = [&](const Plane& p) {
        const double d = leg(p);
        return std::vector<std::pair<double, double>>{{0.5 * d, 0.0}, {1.5 * d, 0.0}, {1.5 * d + 0.5 * d, d * std::sin(60.0 * kDeg)}};
    };
    run("fsim.guidance.must_fly", 0.0,
        [&](const Plane& p) {
            OpLine line;
            for (const auto& [n, e] : cornersOf(p)) {
                const PositionCommand q = pointFrom(p.start, n, e, 0.0, 0.0);
                LineVertex v;
                v.latitudeRad = q.latitudeRad, v.longitudeRad = q.longitudeRad;
                line.vertices.push_back(v);
            }
            line.leftWidthM = line.rightWidthM = 0.5 * leg(p);
            MustFlyCommand c;
            c.location = static_cast<double>(MustFlyLocation::Line);
            const CommandResult r = w.submit(p.id, c, line);
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            offLine[p.id] = 0.0, fromEnd[p.id] = std::numeric_limits<double>::quiet_NaN(), pastFirst[p.id] = false;
            return r.accepted();
        },
        [&](const Plane& p) { return 4.5 * leg(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) + 60.0; },
        [&](const Plane& p) {
            const ActivityRecord* r = w.activity(activity[p.id]);
            if (!r || !std::isnan(fromEnd[p.id])) return;
            const auto& s = *w.vehicleState(p.id);
            const double n = (s.latitudeRad - p.start.latitudeRad) * kEarthM;
            const double e = (s.longitudeRad - p.start.longitudeRad) * kEarthM * std::cos(p.start.latitudeRad);
            const auto corners = cornersOf(p);
            double nearest = kInf;
            for (std::size_t i = 0; i + 1 < corners.size(); ++i) {
                const double an = corners[i].first, ae = corners[i].second, dn = corners[i + 1].first - an, de = corners[i + 1].second - ae;
                const double along = ((n - an) * dn + (e - ae) * de) / (dn * dn + de * de), t = std::clamp(along, 0.0, 1.0);
                if (i == 0 && along > 0.0) pastFirst[p.id] = true;
                nearest = std::min(nearest, std::hypot(n - an - t * dn, e - ae - t * de));
            }
            if (pastFirst[p.id]) offLine[p.id] = std::max(offLine[p.id], nearest);
            if (r->live()) return;
            fromEnd[p.id] = std::hypot(n - corners.back().first, e - corners.back().second);
        },
        [&](const Plane& p, const Lows& lows) {
            completed(p, lows);
            // within a tenth of a leg of its line once in it (a rotorcraft's quarter), its widths half a leg - the EA-18G 0.052 of
            // its leg off it at its turn, the UH-1H 0.125 - and done within 50 m of its last vertex (a rotorcraft's 10 m): the
            // KC-46A 22.8 m, the UH-1H 4.8 m
            CHECK(offLine[p.id] <= (p.rotor ? 0.25 : 0.1) * leg(p));
            CHECK(fromEnd[p.id] <= (p.rotor ? 10.0 : 50.0));
        });
    // a must fly into a volume (ADR-29 FA-8b3, MFY-06): a sphere a quarter of a leg across its radius, its centre a leg ahead,
    // half a leg to the right and 100 m above the aircraft (a rotorcraft's 20 m) - entered, and completed once in it
    std::map<std::uint32_t, double> fromCentre;
    auto ballOf = [&](const Plane& p) {
        const PositionCommand c = pointFrom(p.start, leg(p), 0.5 * leg(p), 0.0, 0.0);
        OpVolume v;
        v.shape = static_cast<double>(VolumeShape::Sphere);
        v.latitudeRad = c.latitudeRad, v.longitudeRad = c.longitudeRad, v.altitudeM = p.start.altitudeMslM + (p.rotor ? 20.0 : 100.0);
        v.radiusM = 0.25 * leg(p);
        return v;
    };
    run("fsim.guidance.must_fly", 0.0,
        [&](const Plane& p) {
            MustFlyCommand c;
            c.location = static_cast<double>(MustFlyLocation::Volume);
            const CommandResult r = w.submit(p.id, c, ballOf(p));
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            fromCentre[p.id] = std::numeric_limits<double>::quiet_NaN();
            return r.accepted();
        },
        [&](const Plane& p) { return 4.0 * leg(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) + 60.0; },
        [&](const Plane& p) {
            const ActivityRecord* r = w.activity(activity[p.id]);
            if (!std::isnan(fromCentre[p.id]) || !r || r->live()) return;
            const auto& s = *w.vehicleState(p.id);
            const OpVolume v = ballOf(p);
            const double north = (s.latitudeRad - v.latitudeRad) * kEarthM, east = (s.longitudeRad - v.longitudeRad) * kEarthM * std::cos(v.latitudeRad);
            fromCentre[p.id] = std::sqrt(north * north + east * east + (s.altitudeMslM - v.altitudeM) * (s.altitudeMslM - v.altitudeM)) - v.radiusM;
        },
        [&](const Plane& p, const Lows& lows) {
            completed(p, lows);
            // in it as it completed - a CF2 0.01 m inside its 5 m sphere after climbing 15 m, an E-3G 8 m inside its 5.8 km one; 0.5 %
            // of its radius for this flat measure
            CHECK(fromCentre[p.id] <= 0.005 * 0.25 * leg(p));
        });
    // its endurance (ADR-29 FA-3e, VAL-03): three times what it lasts, a timed pattern, is refused - a soft rejection,
    // which override_rejection overrides; five minutes straight ahead at its cruise, its prediction (read under a
    // reserve of 99.99 %, so the check reports what the flight needs) against the burn it flies, within 5 %
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const NavigationReport nav = w.navigationReport(p.id);
            REQUIRE(nav.energy != Energy::Unknown);
            CommandOptions validate;
            validate.validateOnly = true;
            PatternCommand beyond;
            beyond.durationS = 3.0 * nav.enduranceS;
            CHECK(w.submit(p.id, beyond, validate).reason == Reason::InsufficientEndurance);
            CHECK(w.commandDetails(p.id)->endurance.energy == static_cast<std::uint8_t>(nav.energy));
            validate.overrideRejection = true;
            const CommandResult anyway = w.submit(p.id, beyond, validate);
            CHECK((anyway.status == CommandStatus::Valid && (anyway.flags & kOverridden) != 0));
            const double speed = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, d = 300.0 * speed, psi = p.start.eulerRad[2];
            const PositionCommand q = pointFrom(p.start, d * std::cos(psi), d * std::sin(psi), p.start.altitudeMslM, 0.0);
            Waypoint ahead;
            ahead.latitudeRad = q.latitudeRad, ahead.longitudeRad = q.longitudeRad, ahead.altitudeM = q.altitudeMslM;
            ahead.speed = speed, ahead.speedReference = static_cast<double>(p.rotor ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
            // the terrain (ADR-29 FA-4a, VAL-06): a path under the ground - flat, at sea level - is refused, overridden or
            // not, the place it meets it named: an hsa's where it is, at once; a route's to a point under it, on its way
            // down (at its steepest descent, if the leg is steeper)
            HsaCommand under;
            under.altitudeM = -100.0;
            CHECK(w.submit(p.id, under, validate).reason == Reason::TerrainConflict);
            {
                const CommandDetails::Terrain& t = w.commandDetails(p.id)->terrain;
                CHECK((t.hit == 1 && t.index == -1 && t.altitudeMslM == -100.0 && t.groundM == 0.0 && std::abs(t.timeS) < 1e-9));
                PositionCommand there;
                there.latitudeRad = t.latitudeRad, there.longitudeRad = t.longitudeRad;
                CHECK(distanceTo(*w.vehicleState(p.id), there) < 1.0);
            }
            Waypoint down = ahead;
            down.altitudeM = -100.0;
            CHECK(w.submit(p.id, RouteCommand{}, Span<const Waypoint>(&down, 1), validate).reason == Reason::TerrainConflict);
            {
                const CommandDetails::Terrain& t = w.commandDetails(p.id)->terrain;
                CHECK((t.hit == 1 && t.index == 0 && t.altitudeMslM < 0.0 && t.altitudeMslM > -0.1 && t.timeS > 0.0));
            }
            NavigationSettings settings = w.navigation(p.id);
            const NavigationSettings kept = settings;
            settings.reserveFraction = 0.9999;
            REQUIRE(w.setNavigation(p.id, settings) == Reason::None);
            validate.overrideRejection = false;
            CHECK(w.submit(p.id, RouteCommand{}, Span<const Waypoint>(&ahead, 1), validate).reason == Reason::InsufficientEndurance);
            predicted[p.id] = w.commandDetails(p.id)->endurance.required;
            REQUIRE(w.setNavigation(p.id, kept) == Reason::None);
            onBoard[p.id] = nav.remaining, atEnd[p.id] = kInf;
            const CommandResult r = w.submit(p.id, RouteCommand{}, Span<const Waypoint>(&ahead, 1));
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        secs(330.0, 330.0),
        [&](const Plane& p) { // (what it has as the route ends: it flies on after)
            if (atEnd[p.id] == kInf && w.activity(activity[p.id])->state == ActivityState::Completed) atEnd[p.id] = w.navigationReport(p.id).remaining;
        },
        [&](const Plane& p, const Lows&) {
            const double burned = onBoard[p.id] - atEnd[p.id];
            INFO("predicted " << predicted[p.id] << ", burned " << burned);
            CHECK(w.activity(activity[p.id])->state == ActivityState::Completed);
            // no more than 5 % beyond it, the side that matters; it may be the more cautious (the worst: the Mirage
            // 2000, its tanks full and slow on the back of its power curve, 5.5 % short of it)
            CHECK(burned / predicted[p.id] - 1.0 < 0.05);
            CHECK(burned / predicted[p.id] - 1.0 > -0.08);
        });
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
            least[p.id] = 0.0; // (here: the worst cross-track after the first lap - a rotorcraft's second, below)
            return r.accepted();
        },
        [&](const Plane& p) { return (1.0 + 2.5 * kPi) * orbitRadius(p) / std::max(p.cruiseMs, 0.1) + 60.0; },
        [&](const Plane& p) {
            // a rotorcraft starts at its orbit's centre: its first lap of bearing round it is its way out to the circle
            // (the Crazyflie's crossed the lap's mark 1.3 m short of it), so it is judged from its second
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (r.progress.laps >= (p.rotor ? 2u : 1u) && std::isfinite(r.progress.crossTrackM))
                least[p.id] = std::max(least[p.id], std::abs(r.progress.crossTrackM));
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            CHECK(r.live());
            CHECK(r.progress.laps >= (p.rotor ? 2u : 1u));
            // (the worst: a wing 3.6 % of its radius, 7.9 % the Skua's 132 m; a rotorcraft 3.0 %, from its second lap)
            CHECK(least[p.id] < (p.rotor ? std::max(0.5, 0.1 * orbitRadius(p)) : std::max(20.0, 0.05 * orbitRadius(p))));
        });
    // an altitude stacked marshall (ADR-29 FA-8c, ASM-01): each aircraft alone in a stack round where it is - a wing orbiting, a
    // rotorcraft hovering - its least 100 m above it (a rotorcraft's 20 m): given that slot, climbed to and held
    std::map<std::uint32_t, double> slotOff, marshalledAt, marshallFor;
    run("fsim.guidance.marshall", 0.0,
        [&](const Plane& p) {
            MarshallCommand m;
            m.altitudeMinM = p.start.altitudeMslM + (p.rotor ? 20.0 : 100.0);
            if (p.rotor) m.pattern = static_cast<double>(PatternKind::Hover);
            const CommandResult r = w.submit(p.id, m, PatternShape{});
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            slotOff[p.id] = 0.0, marshalledAt[p.id] = w.simTime();
            marshallFor[p.id] = (1.0 + 2.5 * kPi) * orbitRadius(p) / std::max(p.cruiseMs, 0.1) + 120.0;
            Setpoint sp;
            if (r.accepted() && w.activitySetpoint(r.activity, sp)) CHECK((sp.marshall && sp.marshall->altitudeM == m.altitudeMinM));
            return r.accepted();
        },
        [&](const Plane& p) { return marshallFor[p.id]; },
        [&](const Plane& p) { // (its last 30 s: how far off its slot)
            if (w.simTime() - marshalledAt[p.id] < marshallFor[p.id] - 30.0) return;
            const double slot = p.start.altitudeMslM + (p.rotor ? 20.0 : 100.0);
            slotOff[p.id] = std::max(slotOff[p.id], std::abs(w.vehicleState(p.id)->altitudeMslM - slot));
        },
        [&](const Plane& p, const Lows&) {
            CHECK(w.activity(activity[p.id])->live());
            // (at worst a Gripen 9.1 m off its slot, a Mirage 2000 5.2 m; a rotorcraft under a centimetre)
            CHECK(slotOff[p.id] <= (p.rotor ? 0.5 : 20.0));
        });
    // a route intercept (ADR-29 FA-8d, RIC-01 to RIC-03): a plan of four points along the aircraft's heading, half a leg to its
    // right, from half a leg behind it - kept, then joined by the soonest way from where the aircraft is (a wing ahead of where
    // the perpendicular meets its leg, a rotorcraft there) and flown to its end: the plan activated by it, complete
    std::map<std::uint32_t, double> lastPass, joinAhead;
    std::map<std::uint32_t, std::int32_t> joinedAt;
    auto interceptPlan = [&](const Plane& p) {
        const double psi = p.start.eulerRad[2], d = leg(p);
        RoutePlan plan;
        plan.id = 40;
        for (const double along : {-0.5, 0.5, 1.5, 2.5}) {
            const PositionCommand q = pointFrom(p.start, along * d * std::cos(psi) - 0.5 * d * std::sin(psi), along * d * std::sin(psi) + 0.5 * d * std::cos(psi),
                                                p.start.altitudeMslM, 0.0);
            Waypoint wp;
            wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
            if (p.rotor) wp.speed = p.cruiseMs, wp.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            plan.waypoints.push_back(wp);
        }
        return plan;
    };
    run("fsim.guidance.intercept", 0.0,
        [&](const Plane& p) {
            const RoutePlan plan = interceptPlan(p);
            CHECK(w.planCommand(p.id, 40, PlanCommand::PrepareForUpload).completed);
            CHECK(w.publishPlan(p.id, plan) == Reason::None);
            CHECK(w.planCommand(p.id, 40, PlanCommand::Upload).completed);
            InterceptCommand c;
            c.plan = 40.0, c.method = static_cast<double>(InterceptMethod::Soonest);
            const CommandResult r = w.submit(p.id, c);
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            lastPass[p.id] = kInf;
            InterceptStatus st;
            if (r.accepted() && w.interceptStatus(r.activity, st)) {
                joinedAt[p.id] = st.laid >= 0 ? st.joined : -1;
                // how far ahead of where the perpendicular from the aircraft meets the plan's line it joins, m
                const double psi = p.start.eulerRad[2];
                const double north = (st.joinLatitudeRad - p.start.latitudeRad) * kEarthM;
                const double east = (st.joinLongitudeRad - p.start.longitudeRad) * kEarthM * std::cos(p.start.latitudeRad);
                joinAhead[p.id] = north * std::cos(psi) + east * std::sin(psi);
            }
            return r.accepted();
        },
        [&](const Plane& p) { return 6.0 * leg(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) + 60.0; },
        [&](const Plane& p) { // (how near it passes the plan's last point)
            const auto& s = *w.vehicleState(p.id);
            const Waypoint end = interceptPlan(p).waypoints.back();
            lastPass[p.id] = std::min(lastPass[p.id], geo::distanceM(s.latitudeRad, s.longitudeRad, end.latitudeRad, end.longitudeRad));
        },
        [&](const Plane& p, const Lows& lows) {
            completed(p, lows);
            CHECK(w.planStatus(p.id, 40)->execution == PlanExecution::Complete);
            // joined on a leg: a wing ahead of the perpendicular's foot (it turns toward the plan as it goes on: from the Skua's
            // 132 m to the C-17A's 9.4 km), a rotorcraft at it; the plan's last point passed at worst 8.1 m off (the Mirage 2000's),
            // a helicopter's 0.46 m, a multirotor's 0.05 m
            CHECK(joinedAt[p.id] >= 1);
            if (p.rotor) CHECK(std::abs(joinAhead[p.id]) < 1.0);
            else CHECK(joinAhead[p.id] > 0.0);
            CHECK(lastPass[p.id] < (p.cls == Class::FlyByWire ? 70.0 : p.cls == Class::Direct ? 20.0 : p.cls == Class::Helicopter ? 7.0 : 0.5));
        });
    // A-GRA's orbit as its schema gives it (ADR-29 FA-5a: LTR-03, LTR-06, LTR-07): a racetrack by two circles of its
    // orbit's radius, the first two radii ahead and the second three beyond, once round from where it joins it, and out
    // at the far end of the second - completed there, a lap counted
    std::map<std::uint32_t, PositionCommand> exitPoint;
    std::map<std::uint32_t, double> exitMiss;
    auto alongHeading = [&](const Plane& p, double m) {
        const double psi = p.start.eulerRad[2];
        return pointFrom(p.start, m * std::cos(psi), m * std::sin(psi), p.start.altitudeMslM, 0.0);
    };
    run("fsim.guidance.pattern", 0.0,
        [&](const Plane& p) {
            const double radius = orbitRadius(p);
            const PositionCommand a = alongHeading(p, 2.0 * radius), b = alongHeading(p, 5.0 * radius);
            exitPoint[p.id] = alongHeading(p, 6.0 * radius);
            exitMiss[p.id] = kInf;
            PatternCommand c;
            PatternShape shape;
            c.pattern = static_cast<double>(PatternKind::Racetrack), c.radiusM = radius, shape.orbits = 1.0;
            c.latitudeRad = a.latitudeRad, c.longitudeRad = a.longitudeRad, shape.latitude2Rad = b.latitudeRad, shape.longitude2Rad = b.longitudeRad;
            shape.exitLatitudeRad = exitPoint[p.id].latitudeRad, shape.exitLongitudeRad = exitPoint[p.id].longitudeRad;
            const CommandResult r = w.submit(p.id, c, shape);
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        [&](const Plane& p) { return 24.0 * orbitRadius(p) / std::max(p.cruiseMs, 0.1) + 60.0; },
        [&](const Plane& p) { // (where it is as it completes: it flies on after)
            if (exitMiss[p.id] == kInf && !w.activity(activity[p.id])->live()) exitMiss[p.id] = distanceTo(*w.vehicleState(p.id), exitPoint[p.id]);
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            INFO(activityStateName(r.state) << " " << reasonName(r.reason) << ", " << r.progress.laps << " laps, " << exitMiss[p.id] << " m from its exit point");
            CHECK(r.state == ActivityState::Completed);
            CHECK(r.progress.laps == 1);
            // (the worst: a wing 10.0 m, the Gripen's 0.56 % of its radius, 3.6 % the Skua's 132 m; a rotorcraft 0.46 m, the
            // IRIS's 6.1 % of its 7.5 m)
            CHECK(exitMiss[p.id] < (p.rotor ? std::max(0.5, 0.1 * orbitRadius(p)) : std::max(20.0, 0.05 * orbitRadius(p))));
        });
    // A-GRA's hold (ADR-29 FA-5b: LTR-10, LTR-13): on a fix two turns ahead, entered at it by ATC's entry for the side it
    // comes from - here the parallel's: out a leg along the inbound course, round toward the holding side, back to the fix
    // - its legs half a minute, once round
    struct HoldPlan {
        PositionCommand fix;
        double courseRad = 0.0, radiusM = 0.0, legM = 0.0, startS = 0.0, speedMs = 0.0, inRad = 0.0;
    };
    std::map<std::uint32_t, HoldPlan> holds;
    std::map<std::uint32_t, double> parallelMiss, parallelTrack;
    run("fsim.guidance.pattern", 0.0,
        [&](const Plane& p) {
            HoldPlan& h = holds[p.id];
            h.speedMs = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs;
            const double turn = std::max(h.speedMs / (3.0 * kDeg), h.speedMs * h.speedMs / (9.80665 * std::tan(25.0 * kDeg)));
            const double psi = h.inRad = p.start.eulerRad[2];
            h.fix = pointFrom(p.start, 2.0 * turn * std::cos(psi), 2.0 * turn * std::sin(psi), p.start.altitudeMslM, 0.0);
            PatternCommand c;
            c.pattern = static_cast<double>(PatternKind::Hold), c.latitudeRad = h.fix.latitudeRad, c.longitudeRad = h.fix.longitudeRad;
            c.courseRad = std::remainder(psi + 120.0 * kDeg, 2.0 * kPi); // (it comes 120 degrees left of the inbound course: the parallel's side)
            PatternShape shape;
            shape.holdEntry = static_cast<double>(HoldEntry::Anchor), shape.legS = 30.0, shape.orbits = 1.0;
            const CommandResult r = w.submit(p.id, c, shape);
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            // (the Crazyflie's battery does not last a hold at its cruise of a metre a second - flown anyway, it flies the entry
            // and the lap, then falls: its endurance check is right, and the multirotor is flown here by the IRIS)
            if (p.rotor && r.reason == Reason::InsufficientEndurance) return false;
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            parallelMiss[p.id] = kInf, parallelTrack[p.id] = kHold;
            if (!r.accepted()) return false;
            Setpoint flown;
            REQUIRE(w.activitySetpoint(r.activity, flown));
            const auto& planned = std::get<PatternCommand>(std::get<Command>(flown.command));
            h.courseRad = planned.courseRad, h.radiusM = planned.radiusM, h.legM = planned.legM, h.startS = w.simTime();
            return true;
        },
        [&](const Plane& p) { // to the fix, its parallel leg, turn and way back, and a lap: two legs and two half circles
            const HoldPlan& h = holds[p.id];
            return (2.0 * h.radiusM + 4.3 * h.legM + 3.0 * kPi * h.radiusM) / std::max(h.speedMs, 0.1) * 1.25 + 60.0;
        },
        [&](const Plane& p) { // its mark: half a leg out along the inbound course, flown away from the fix, begun where the turn over the
                              // fix onto it ended - while it enters
            const HoldPlan& h = holds[p.id];
            if (w.simTime() - h.startS > (2.0 * h.radiusM + 2.0 * h.legM) / std::max(h.speedMs, 0.1) * 1.2) return;
            const auto& s = *w.vehicleState(p.id);
            double north, east;
            offset(s, h.fix.latitudeRad, h.fix.longitudeRad, north, east); // (the aircraft from the fix)
            const double out = h.courseRad + kPi, way = std::remainder(out - h.inRad, 2.0 * kPi) >= 0.0 ? 1.0 : -1.0;
            const double markN = way * h.radiusM * (std::sin(out) - std::sin(h.inRad)) + 0.5 * h.legM * std::cos(out);
            const double markE = way * h.radiusM * (std::cos(h.inRad) - std::cos(out)) + 0.5 * h.legM * std::sin(out);
            const double d = std::hypot(north - markN, east - markE);
            if (d < parallelMiss[p.id]) parallelMiss[p.id] = d, parallelTrack[p.id] = std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]);
        },
        [&](const Plane& p, const Lows&) {
            const HoldPlan& h = holds[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const double apart = std::abs(std::remainder(parallelTrack[p.id] - (h.courseRad + kPi), 2.0 * kPi)) / kDeg;
            INFO(activityStateName(r.state) << ", " << r.progress.laps << " laps; R " << h.radiusM << ", legs " << h.legM << "; its parallel leg passed "
                                            << parallelMiss[p.id] << " m off, its track " << apart << " deg off");
            CHECK(r.state == ActivityState::Completed);
            CHECK(r.progress.laps == 1);
            // (the worst: a wing 29 m and 1.3 deg, the E-3G's 0.41 % of its radius, the B-52H's 21 m; a rotorcraft 5.1 m and 3.4 deg,
            // the UH-1H's 1.8 %)
            CHECK(parallelMiss[p.id] < (p.rotor ? std::max(0.5, 0.1 * h.radiusM) : std::max(20.0, 0.05 * h.radiusM)));
            CHECK(apart < 10.0);
        });
    // A-GRA's hover (ADR-29 FA-5c: LTR-15): a rotorcraft over a point ten seconds of its cruise ahead and 10 m up, for 20 s from
    // its arrival there; a wing's refused, as its support table says (R1: its design does not fly on rotors)
    std::map<std::uint32_t, PositionCommand> hoverPoint;
    std::map<std::uint32_t, double> hoverArrived, hoverAfter;
    run("fsim.guidance.pattern", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], d = 0.5 * p.scale();
            const PositionCommand q = pointFrom(p.start, d * std::cos(psi), d * std::sin(psi), p.start.altitudeMslM + 10.0, 0.0);
            PatternCommand c;
            c.pattern = static_cast<double>(PatternKind::Hover), c.latitudeRad = q.latitudeRad, c.longitudeRad = q.longitudeRad;
            c.altitudeM = q.altitudeMslM, c.altitudeReference = static_cast<double>(AltitudeReference::Msl), c.durationS = 20.0;
            const CommandResult r = w.submit(p.id, c);
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            const control::SupportInfo* row = w.supportTable(p.id)->find("fsim.guidance.pattern/hover");
            REQUIRE(row != nullptr);
            if (!p.rotor) { // (not supported: its design says so)
                CHECK(row->support == Support::NotSupported);
                CHECK(r.reason == Reason::NotSupported);
                CHECK(r.index == 0);
                return false;
            }
            CHECK(row->support == Support::Supported);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            hoverPoint[p.id] = q, hoverArrived[p.id] = kHold, hoverAfter[p.id] = 0.0;
            return r.accepted();
        },
        [&](const Plane& p) { // there, and the last metres at its loops' pace (the UH-1H's last 17 m took 65 s: rotorcraft.md, 7)
            const double bandwidth = perf(p).velocityBandwidthRadS;
            return 4.0 * 0.5 * p.scale() / std::max(p.cruiseMs, 0.1) + (std::isfinite(bandwidth) && bandwidth > 0.0 ? 20.0 / bandwidth : 150.0) + 40.0;
        },
        [&](const Plane& p) { // there within a metre and 2 m of its height; after, how far off
            const auto& s = *w.vehicleState(p.id);
            const double off = std::hypot(distanceTo(s, hoverPoint[p.id]), s.altitudeMslM - hoverPoint[p.id].altitudeMslM);
            if (isHold(hoverArrived[p.id]) && distanceTo(s, hoverPoint[p.id]) < 1.0 && std::abs(s.altitudeMslM - hoverPoint[p.id].altitudeMslM) < 2.0)
                hoverArrived[p.id] = w.simTime();
            if (!w.activity(activity[p.id])->live()) hoverAfter[p.id] = std::max(hoverAfter[p.id], off);
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            INFO(activityStateName(r.state) << "; arrived " << hoverArrived[p.id] << ", completed " << r.endTime << "; then within " << hoverAfter[p.id] << " m");
            CHECK(r.state == ActivityState::Completed);
            REQUIRE_FALSE(isHold(hoverArrived[p.id]));
            // (each completed 20.00 to 20.03 s after its arrival - the UH-1H's 85 s after its command, the UH-60A's 17 s - and hovered
            // on within 0.65 m, the UH-60A's)
            CHECK(std::abs(r.endTime - hoverArrived[p.id] - 20.0) < 0.5); // (its duration from its arrival)
            CHECK(hoverAfter[p.id] < 1.0);
        });
    // A-GRA's relative point (ADR-29 FA-5c: LTR-18): an orbit round the origin of a frame moving across its heading at a tenth of
    // its speed, from two radii ahead - flown over the frame, round the point as it moves
    std::map<std::uint32_t, FrameSpec> carrier;
    std::map<std::uint32_t, double> carriedMiss;
    run("fsim.guidance.pattern", 0.0,
        [&](const Plane& p) {
            const double radius = orbitRadius(p), psi = p.start.eulerRad[2], v = 0.1 * (p.rotor ? p.cruiseMs : p.start.airspeedTrueMs);
            const PositionCommand o = alongHeading(p, 2.0 * radius);
            FrameSpec f;
            f.origin = FrameOrigin::Moving;
            f.latitudeRad = o.latitudeRad, f.longitudeRad = o.longitudeRad, f.altitudeMslM = p.start.altitudeMslM, f.yawRad = psi;
            f.northMs = -v * std::sin(psi), f.eastMs = v * std::cos(psi), f.timeS = w.simTime(); // (to its right)
            const FrameId id = w.createFrame(f);
            REQUIRE(id != 0);
            carrier[p.id] = f, carriedMiss[p.id] = 0.0;
            PatternCommand c;
            PatternShape shape;
            c.radiusM = radius, shape.frame = static_cast<double>(id);
            const CommandResult r = w.submit(p.id, c, shape);
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        [&](const Plane& p) { return (2.0 + 3.5 * kPi) * orbitRadius(p) / std::max(0.9 * (p.rotor ? p.cruiseMs : p.start.airspeedTrueMs), 0.1) + 60.0; },
        [&](const Plane& p) { // off its circle round the point where the frame is now, from its first lap
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (r.progress.laps < 1) return;
            const GeoPoint centre = framePoint(framePose(carrier[p.id], w.simTime()), FrameOffset{});
            double north, east;
            offset(*w.vehicleState(p.id), centre.latitudeRad, centre.longitudeRad, north, east);
            carriedMiss[p.id] = std::max(carriedMiss[p.id], std::abs(std::hypot(north, east) - orbitRadius(p)));
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            INFO(activityStateName(r.state) << ", " << r.progress.laps << " laps; R " << orbitRadius(p) << ", off its moving circle " << carriedMiss[p.id] << " m");
            CHECK(r.live());
            CHECK(r.progress.laps >= 1);
            // (the worst: a wing 16.8 m, the C172's 4.7 % of its 357 m; a rotorcraft 4.3 m, the UH-1H's 2.1 %)
            CHECK(carriedMiss[p.id] < (p.rotor ? std::max(0.5, 0.1 * orbitRadius(p)) : std::max(20.0, 0.05 * orbitRadius(p))));
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
    // A-GRA's curve as its schema gives it (ADR-29 FA-5d1: CRV-03): clamped rational B-splines - an S as a rational
    // cubic (seven points unevenly weighted, three interior knots) twelve orbit radii long, a semicircle round to the
    // right as a rational quadratic (NURBS's exact circle), a straight cubic on - off the curve from a fifth along it
    struct NurbsPlan {
        std::vector<std::array<double, 3>> line; ///< the curve, dense, north and east of its start
        std::size_t hint = 0;
        double miss = 0.0;
    };
    std::map<std::uint32_t, NurbsPlan> nurbsPlans;
    auto knotted = [](int degree, const std::vector<std::array<double, 3>>& points, const std::vector<double>& weights, const std::vector<double>& interior) {
        NurbsSegment seg;
        seg.points = static_cast<std::uint32_t>(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) seg.north[i] = points[i][0], seg.east[i] = points[i][1], seg.down[i] = points[i][2], seg.weight[i] = weights[i];
        std::vector<double> k(static_cast<std::size_t>(degree + 1), 0.0);
        k.insert(k.end(), interior.begin(), interior.end());
        k.insert(k.end(), static_cast<std::size_t>(degree + 1), 1.0);
        seg.knots = static_cast<std::uint32_t>(k.size());
        std::copy(k.begin(), k.end(), seg.knot);
        return seg;
    };
    run("fsim.guidance.curve", 0.0,
        [&](const Plane& p) {
            const double R = orbitRadius(p), L = 12.0 * R, a = 0.8 * R, rc = 1.6 * R, half = std::sqrt(0.5);
            std::vector<NurbsSegment> segments = {
                knotted(3, {{0.0, 0.0, 0.0}, {0.15 * L, 0.0, 0.0}, {0.3 * L, a, 0.0}, {0.5 * L, a, 0.0}, {0.7 * L, -a, 0.0}, {0.85 * L, 0.0, 0.0}, {L, 0.0, 0.0}},
                        {1.0, 1.4, 0.8, 1.0, 1.25, 0.9, 1.0}, {0.2, 0.45, 0.7}),
                knotted(2, {{L, 0.0, 0.0}, {L + rc, 0.0, 0.0}, {L + rc, rc, 0.0}, {L + rc, 2 * rc, 0.0}, {L, 2 * rc, 0.0}}, {1.0, half, 1.0, half, 1.0},
                        {0.5, 0.5}),
                knotted(3, {{L, 2 * rc, 0.0}, {L - R, 2 * rc, 0.0}, {L - 2 * R, 2 * rc, 0.0}, {L - 3 * R, 2 * rc, 0.0}}, {1.0, 1.0, 1.0, 1.0}, {})};
            const double c = std::cos(p.start.eulerRad[2]), sn = std::sin(p.start.eulerRad[2]);
            for (auto& seg : segments) // (along its heading)
                for (std::uint32_t i = 0; i < seg.points; ++i) {
                    const double n = seg.north[i], e = seg.east[i];
                    seg.north[i] = n * c - e * sn, seg.east[i] = n * sn + e * c;
                }
            NurbsPlan& plan = nurbsPlans[p.id];
            plan = NurbsPlan{};
            for (const auto& seg : segments)
                for (int k = plan.line.empty() ? 0 : 1; k <= 1500; ++k) {
                    const route::CurvePoint q = route::rational(seg, k / 1500.0);
                    plan.line.push_back({q.p[0], q.p[1], q.p[2]});
                }
            const CommandResult r = w.submit(p.id, CurveCommand{}, Span<const NurbsSegment>(segments));
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        [&](const Plane& p) { return (12.0 + 1.6 * kPi + 3.0) * orbitRadius(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) * 1.5 + 60.0; },
        [&](const Plane& p) { // the nearest point of the curve, searched about the last
            NurbsPlan& plan = nurbsPlans[p.id];
            if (!w.activity(activity[p.id])->live()) return;
            double north, east;
            offset(*w.vehicleState(p.id), p.start.latitudeRad, p.start.longitudeRad, north, east);
            const std::size_t lo = plan.hint > 300 ? plan.hint - 300 : 0, hi = std::min(plan.line.size() - 1, plan.hint + 1500);
            double best = 1e18;
            std::size_t at = plan.hint;
            for (std::size_t j = lo; j < hi; ++j) {
                const auto& a0 = plan.line[j];
                const auto& b0 = plan.line[j + 1];
                const double bn = b0[0] - a0[0], be = b0[1] - a0[1], span = bn * bn + be * be;
                const double u = span > 0.0 ? std::clamp(((north - a0[0]) * bn + (east - a0[1]) * be) / span, 0.0, 1.0) : 0.0;
                if (const double d = std::hypot(a0[0] + u * bn - north, a0[1] + u * be - east); d < best) best = d, at = j;
            }
            plan.hint = at;
            if (at > 300) plan.miss = std::max(plan.miss, best); // (from a fifth along the S: joined)
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const double R = orbitRadius(p), miss = nurbsPlans[p.id].miss;
            INFO(activityStateName(r.state) << "; R " << R << ", off the curve " << miss << " m");
            CHECK(r.state == ActivityState::Completed);
            // (the worst: a wing 3.8 % of its radius, the Skua's 5.1 m - the KC-46A's 287 m, 3.2 % of its 8.9 km; a rotorcraft 5.6 %,
            // the Crazyflie's 0.38 m)
            CHECK(miss < (p.rotor ? std::max(0.5, 0.1 * R) : std::max(20.0, 0.05 * R)));
        });
    // A-GRA's curve reference (ADR-29 FA-5d2: CRV-04, CRV-05, CRV-06): a straight cubic twelve orbit radii long from the origin
    // of a frame a radius ahead, along the frame's heading 30 degrees right of the aircraft's - its points turned with the
    // frame, their third read as offsets up, climbing 1 % - the frame moving to its right at a tenth of the aircraft's speed:
    // flown over the frame, off its line where the frame is now and off its height, from a third along it
    struct TurnedPlan {
        FrameSpec frame;
        double base = 0.0, across = 0.0, up = 0.0;
    };
    std::map<std::uint32_t, TurnedPlan> turnedPlans;
    run("fsim.guidance.curve", 0.0,
        [&](const Plane& p) {
            const double R = orbitRadius(p), psi = p.start.eulerRad[2] + 30.0 * kDeg, v = 0.1 * (p.rotor ? p.cruiseMs : p.start.airspeedTrueMs);
            const PositionCommand o = alongHeading(p, R);
            TurnedPlan& plan = turnedPlans[p.id];
            plan = TurnedPlan{};
            FrameSpec& f = plan.frame;
            f.origin = FrameOrigin::Moving;
            f.latitudeRad = o.latitudeRad, f.longitudeRad = o.longitudeRad, f.altitudeMslM = p.start.altitudeMslM, f.yawRad = psi;
            f.northMs = -v * std::sin(psi), f.eastMs = v * std::cos(psi), f.timeS = w.simTime(); // (to its right)
            const FrameId id = w.createFrame(f);
            REQUIRE(id != 0);
            NurbsSegment seg;
            seg.points = 4, seg.knots = 8;
            for (std::uint32_t i = 0; i < 4; ++i) seg.north[i] = 4.0 * R * i, seg.down[i] = 0.04 * R * i; // (along its x; up, read so)
            for (std::uint32_t i = 4; i < 8; ++i) seg.knot[i] = 1.0;
            CurveCommand c;
            c.pointRotation = static_cast<double>(FrameRotation::Yaw), c.pointZ = static_cast<double>(CurveZ::AltitudeOffset);
            CurveShape shape;
            shape.frame = static_cast<double>(id);
            plan.base = w.vehicleState(p.id)->altitudeMslM; // (its reference's height: the aircraft's)
            const CommandResult r = w.submit(p.id, c, Span<const NurbsSegment>(&seg, 1), {}, &shape);
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        [&](const Plane& p) { return 14.0 * orbitRadius(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) * 1.5 + 60.0; },
        [&](const Plane& p) {
            TurnedPlan& plan = turnedPlans[p.id];
            if (!w.activity(activity[p.id])->live()) return;
            const FramePose pose = framePose(plan.frame, w.simTime());
            const auto& s = *w.vehicleState(p.id);
            double north, east;
            offset(s, pose.latitudeRad, pose.longitudeRad, north, east);
            const double along = north * std::cos(pose.yawRad) + east * std::sin(pose.yawRad);
            if (along < 4.0 * orbitRadius(p)) return; // (from a third along it: joined from a radius off, 30 degrees across)
            plan.across = std::max(plan.across, std::abs(-north * std::sin(pose.yawRad) + east * std::cos(pose.yawRad)));
            plan.up = std::max(plan.up, std::abs(s.altitudeMslM - (plan.base + 0.01 * along)));
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const TurnedPlan& plan = turnedPlans[p.id];
            const double R = orbitRadius(p);
            INFO(activityStateName(r.state) << "; R " << R << ", off its moving line " << plan.across << " m, off its climb " << plan.up << " m");
            CHECK(r.state == ActivityState::Completed);
            // (the worst: a wing 4.3 % of its radius, the KC-46A's 385 m of its 8.9 km - the Skua 10.1 m, 7.7 % of its 132 m; a
            // rotorcraft 2.3 %, the UH-1H's 4.7 m; off its climb a wing 8.3 m, the Mirage 2000's, a rotorcraft 0.01 m)
            CHECK(plan.across < (p.rotor ? std::max(0.5, 0.1 * R) : std::max(20.0, 0.05 * R)));
            CHECK(plan.up < (p.rotor ? 0.5 : 20.0));
        });
    // A-GRA's circular loiter at a curve's end (ADR-29 FA-5d3: CRV-11): a straight cubic four orbit radii along its heading,
    // then round its end - a rotorcraft as a wing - off the circle its pace gives: a rotorcraft from half a lap after it
    // completes, a wing (joining from its centre) from a lap and a half
    struct Round {
        PositionCommand end;
        double done = -1.0, near = 1e18, far = 0.0;
    };
    std::map<std::uint32_t, Round> rounds;
    run("fsim.guidance.curve", 0.0,
        [&](const Plane& p) {
            const double R = orbitRadius(p);
            NurbsSegment seg;
            seg.points = 4, seg.knots = 8;
            const double c = std::cos(p.start.eulerRad[2]), sn = std::sin(p.start.eulerRad[2]);
            for (std::uint32_t i = 0; i < 4; ++i) seg.north[i] = 4.0 / 3.0 * R * i * c, seg.east[i] = 4.0 / 3.0 * R * i * sn;
            for (std::uint32_t i = 4; i < 8; ++i) seg.knot[i] = 1.0;
            CurveCommand loiter;
            loiter.end = static_cast<double>(EndBehavior::Loiter);
            rounds[p.id] = Round{alongHeading(p, 4.0 * R)};
            const CommandResult r = w.submit(p.id, loiter, Span<const NurbsSegment>(&seg, 1));
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        [&](const Plane& p) { return (4.0 + 5.5 * kPi) * orbitRadius(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) * 1.1 + 60.0; },
        [&](const Plane& p) {
            Round& round = rounds[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (r.live()) return;
            if (round.done < 0.0) round.done = w.simTime();
            const double R = perf(p).turnRadiusM(r.progress.speedMs); // (its circle: at the pace it flew the curve, calm)
            if (w.simTime() - round.done < (p.rotor ? 1.0 : 3.0) * kPi * R / std::max(r.progress.speedMs, 0.1)) return;
            const double d = distanceTo(*w.vehicleState(p.id), round.end);
            round.near = std::min(round.near, d), round.far = std::max(round.far, d);
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const Round& round = rounds[p.id];
            const double R = perf(p).turnRadiusM(r.progress.speedMs);
            INFO(activityStateName(r.state) << "; round its end " << round.near << " to " << round.far << " m, R " << R);
            CHECK(r.state == ActivityState::Completed);
            // (the worst: a wing 0.982 to 1.010 of its radius, the Skua's 129 of 132 m and the C172's 361 of 357 m; a rotorcraft
            // 0.928 to 1.007, the Crazyflie's 6.39 of 6.89 m at a metre a second)
            CHECK(round.near > (p.rotor ? 0.85 : 0.95) * R);
            CHECK(round.far < (p.rotor ? 1.1 : 1.05) * R);
        });
    // A-GRA's relative points in a route (ADR-29 FA-6a: WPT-22): three points in a frame at the aircraft, turned with its
    // heading and moving to its right at a tenth of its speed - five orbit radii ahead, five to the right, five ahead
    // again - its two legs in the frame flown over it: off each where the frame is now, in its middle third
    struct FramedRoute {
        FrameSpec frame;
        double worst = 0.0;
        int samples = 0;
    };
    std::map<std::uint32_t, FramedRoute> framedRoutes;
    const double zig[3][2] = {{5.0, 0.0}, {5.0, 5.0}, {10.0, 5.0}}; // (radii along its heading, and to its right)
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double R = orbitRadius(p), psi = p.start.eulerRad[2], v = 0.1 * (p.rotor ? p.cruiseMs : p.start.airspeedTrueMs);
            FramedRoute& f = framedRoutes[p.id];
            f = FramedRoute{};
            const auto& s = *w.vehicleState(p.id);
            f.frame.origin = FrameOrigin::Moving;
            f.frame.latitudeRad = s.latitudeRad, f.frame.longitudeRad = s.longitudeRad, f.frame.yawRad = psi;
            f.frame.northMs = -v * std::sin(psi), f.frame.eastMs = v * std::cos(psi), f.frame.timeS = w.simTime(); // (to its right)
            const FrameId id = w.createFrame(f.frame);
            REQUIRE(id != 0);
            std::vector<Waypoint> points;
            for (const auto& z : zig) {
                Waypoint q;
                q.frame = static_cast<double>(id), q.frameRotation = static_cast<double>(FrameRotation::Yaw);
                q.frameXM = z[0] * R, q.frameYM = z[1] * R;
                points.push_back(q);
            }
            const CommandResult r = w.submit(p.id, RouteCommand{}, points);
            INFO("refused: " << reasonName(r.reason) << " at " << r.index);
            CHECK(r.accepted());
            activity[p.id] = r.activity;
            return r.accepted();
        },
        [&](const Plane& p) { return 16.0 * orbitRadius(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) * 1.5 + 60.0; },
        [&](const Plane& p) {
            FramedRoute& f = framedRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const ActivityProgress& g = r.progress;
            if (!r.live() || g.segment < 1 || g.segmentPercent < 33.0 || g.segmentPercent > 67.0) return;
            const FramePose pose = framePose(f.frame, w.simTime());
            double north, east; // (in the plane at its origin its points are laid out in: its great circles are straight there)
            const auto& s = *w.vehicleState(p.id);
            tangentPlane(pose.latitudeRad, pose.longitudeRad, s.latitudeRad, s.longitudeRad, north, east);
            const double c = std::cos(pose.yawRad), sn = std::sin(pose.yawRad), R = orbitRadius(p);
            const double x = north * c + east * sn, y = -north * sn + east * c; // (in its axes)
            const double x0 = zig[g.segment - 1][0] * R, y0 = zig[g.segment - 1][1] * R, x1 = zig[g.segment][0] * R, y1 = zig[g.segment][1] * R;
            const double length = std::hypot(x1 - x0, y1 - y0);
            f.worst = std::max(f.worst, std::abs(-(x - x0) * (y1 - y0) / length + (y - y0) * (x1 - x0) / length));
            ++f.samples;
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const FramedRoute& f = framedRoutes[p.id];
            const double R = orbitRadius(p);
            INFO(activityStateName(r.state) << "; R " << R << ", off its legs in the frame " << f.worst << " m (" << f.samples << " samples)");
            CHECK(r.state == ActivityState::Completed);
            CHECK(f.samples > 0);
            // (the worst: a wing 5.1 % of its radius, the Skua's 6.7 m - in metres the F-35A's 49 m, 1.6 %; a rotorcraft 8.3 %, the
            // Crazyflie's 0.57 m. Measured in the plane at the frame's origin its points are laid out in, where its great circles
            // are straight: along its parallel instead, the heavies' legs 90 km out read 530 m off)
            CHECK(f.worst < (p.rotor ? std::max(0.5, 0.1 * R) : std::max(20.0, 0.05 * R)));
        });
    // A-GRA's turn points (ADR-29 FA-6b: WPT-04): two orbit radii ahead, a quarter circle of twice its radius round to the
    // right from a start turn point to an end turn point - ARINC 424's radius to fix - and two radii on: off the arc's
    // circle in its middle
    struct ArcRoute {
        PositionCommand start;
        double centreNorthM = 0.0, centreEastM = 0.0, worst = 0.0;
        int samples = 0;
    };
    std::map<std::uint32_t, ArcRoute> arcRoutes;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double R = orbitRadius(p), psi = p.start.eulerRad[2], r = 2.0 * R;
            const double c = std::cos(psi), sn = std::sin(psi);
            auto point = [&](double ahead, double right) {
                const PositionCommand q = pointFrom(p.start, ahead * c - right * sn, ahead * sn + right * c, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            Waypoint a = point(2.0 * R, 0.0), b = point(2.0 * R + r, r);
            const Waypoint e = point(2.0 * R + r, r + 2.0 * R);
            a.turn = static_cast<double>(TurnType::StartTurn), b.turn = static_cast<double>(TurnType::EndTurn);
            ArcRoute& f = arcRoutes[p.id];
            f = ArcRoute{};
            f.start = pointFrom(p.start, 2.0 * R * c, 2.0 * R * sn, p.start.altitudeMslM, 0.0);
            f.centreNorthM = -r * sn, f.centreEastM = r * c; // (from the arc's start: to its right)
            const CommandResult res = w.submit(p.id, RouteCommand{}, std::vector<Waypoint>{a, b, e});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane& p) { return (4.0 + 3.2) * orbitRadius(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) * 1.5 + 60.0; },
        [&](const Plane& p) {
            ArcRoute& f = arcRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const ActivityProgress& g = r.progress;
            if (!r.live() || g.segment != 1 || g.segmentPercent < 15.0 || g.segmentPercent > 85.0) return;
            double north, east;
            offset(*w.vehicleState(p.id), f.start.latitudeRad, f.start.longitudeRad, north, east);
            f.worst = std::max(f.worst, std::abs(std::hypot(north - f.centreNorthM, east - f.centreEastM) - 2.0 * orbitRadius(p)));
            ++f.samples;
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const ArcRoute& f = arcRoutes[p.id];
            const double R = 2.0 * orbitRadius(p);
            INFO(activityStateName(r.state) << "; its arc " << R << " m, off it " << f.worst << " m (" << f.samples << " samples)");
            CHECK(r.state == ActivityState::Completed);
            CHECK(f.samples > 0);
            // (the worst: a wing 3.2 % of its arc's radius, the Skua's 8.4 m - in metres the F-16C's 25 m, 0.5 %; a rotorcraft 1.7 %,
            // the Crazyflie's 0.23 m - the UH-1H's 6.5 m)
            CHECK(f.worst < (p.rotor ? std::max(0.5, 0.1 * R) : std::max(20.0, 0.05 * R)));
        });
    // A-GRA's loiter points (ADR-29 FA-6b2: WPT-18): three orbit radii ahead, then a loiter point eight ahead - a wing's
    // orbit once round it, a rotorcraft's hover over it for 10 s - and on to a point two radii to its right: round its point
    // (a wing's, from half a lap in) or over it (a rotorcraft's, from its arrival) while it loiters, and the route completed
    struct LoiterRoute {
        PositionCommand point;
        double radiusM = 0.0, swept = 0.0, bearing = kHold, worst = 0.0, arrived = kHold, over = 0.0, loiterS = 0.0;
    };
    std::map<std::uint32_t, LoiterRoute> loiterRoutes;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double R = orbitRadius(p), psi = p.start.eulerRad[2];
            const double c = std::cos(psi), sn = std::sin(psi);
            auto point = [&](double ahead, double right) {
                const PositionCommand q = pointFrom(p.start, ahead * c - right * sn, ahead * sn + right * c, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            Waypoint at = point(8.0 * R, 0.0);
            at.kind = static_cast<double>(EndPointKind::LoiterPoint);
            RouteLoiter l;
            l.point = 1;
            if (p.rotor) l.pattern.pattern = static_cast<double>(PatternKind::Hover), l.pattern.durationS = 10.0;
            else l.shape.orbits = 1.0;
            LoiterRoute& f = loiterRoutes[p.id];
            f = LoiterRoute{};
            f.point = pointFrom(p.start, 8.0 * R * c, 8.0 * R * sn, p.start.altitudeMslM, 0.0);
            const CommandResult res =
                w.submit(p.id, RouteCommand{}, std::vector<Waypoint>{point(3.0 * R, 0.0), at, point(8.0 * R, 2.0 * R)}, {}, std::vector<RouteLoiter>{l});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            Setpoint sp;
            if (res.accepted() && w.activitySetpoint(res.activity, sp) && sp.loiters.size() == 1) f.radiusM = sp.loiters[0].pattern.radiusM;
            return res.accepted();
        },
        [&](const Plane& p) { return (13.0 + 2.0 * kPi) * orbitRadius(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) * 1.5 + 70.0; },
        [&](const Plane& p) {
            LoiterRoute& f = loiterRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const ActivityProgress& g = r.progress;
            if (!r.live() || g.segment != 1 || g.segmentPercent != 100.0) return; // (loitering)
            f.loiterS += w.dt() * w.frameSkip();
            double north, east;
            offset(*w.vehicleState(p.id), f.point.latitudeRad, f.point.longitudeRad, north, east);
            const double d = std::hypot(north, east);
            if (p.rotor) {
                if (isHold(f.arrived) && d < 1.0) f.arrived = f.loiterS;
                if (!isHold(f.arrived)) f.over = std::max(f.over, d);
                return;
            }
            const double bearing = std::atan2(east, north);
            if (!isHold(f.bearing)) f.swept += std::remainder(bearing - f.bearing, 2.0 * kPi);
            f.bearing = bearing;
            if (std::abs(f.swept) > kPi) f.worst = std::max(f.worst, std::abs(d - f.radiusM));
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const LoiterRoute& f = loiterRoutes[p.id];
            INFO(activityStateName(r.state) << "; loitered " << f.loiterS << " s: a wing's R " << f.radiusM << ", " << f.swept / (2.0 * kPi)
                                            << " laps, off its circle " << f.worst << " m; a rotorcraft's arrived " << f.arrived << " s in, over "
                                            << "its point within " << f.over << " m");
            CHECK(r.state == ActivityState::Completed);
            // (the worst: a wing 2.7 % of its radius, the C172's 9.5 m - the Skua's 12.8 m of 132, in metres the F-16C's 18.7 m,
            // 0.8 % - round 1.58 laps, once round and on to where it leaves for the next point; a rotorcraft within 1.0 m of its
            // point once there, the UH-1H there 95 s into its loiter, its position loop's approach from where it would stop)
            if (p.rotor) {
                REQUIRE(!isHold(f.arrived));
                CHECK(f.loiterS - f.arrived > 9.9); // (its time from its arrival)
                CHECK(f.over < std::max(1.5, 0.05 * p.scale()));
            } else {
                CHECK(std::abs(f.swept) > 0.95 * 2.0 * kPi);
                CHECK(f.worst < std::max(20.0, 0.05 * f.radiusM));
            }
        });
    // A route that ends in a loiter (EndBehavior::Loiter): three orbit radii ahead, then its last point eight ahead, where a
    // rotorcraft stops and hovers - its stop its position loop's from where it would stop from its speed, as a loiter point's
    // hover (4.31): no farther past its point than that hover, the route completed at its point, and over it after. (A wing
    // orbits its last point: test_routes)
    struct EndStop {
        PositionCommand point;
        double past = -1e9, completedOff = kHold, after = 0.0;
    };
    std::map<std::uint32_t, EndStop> endStops;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            if (!p.rotor) return false;
            const double R = orbitRadius(p), psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            auto point = [&](double ahead) {
                const PositionCommand q = pointFrom(p.start, ahead * c, ahead * sn, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            EndStop& f = endStops[p.id];
            f = EndStop{};
            f.point = pointFrom(p.start, 8.0 * R * c, 8.0 * R * sn, p.start.altitudeMslM, 0.0);
            RouteCommand stop;
            stop.end = static_cast<double>(EndBehavior::Loiter);
            const CommandResult res = w.submit(p.id, stop, std::vector<Waypoint>{point(3.0 * R), point(8.0 * R)});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane& p) { // there, and its position loop's approach from where it would stop (the UH-1H's 95 s)
            const double bandwidth = perf(p).velocityBandwidthRadS;
            return 8.0 * orbitRadius(p) / std::max(p.cruiseMs, 0.1) * 1.5 + (std::isfinite(bandwidth) && bandwidth > 0.0 ? 20.0 / bandwidth : 150.0) + 40.0;
        },
        [&](const Plane& p) {
            EndStop& f = endStops[p.id];
            const auto& s = *w.vehicleState(p.id);
            double north, east;
            offset(s, f.point.latitudeRad, f.point.longitudeRad, north, east);
            const double psi = p.start.eulerRad[2];
            f.past = std::max(f.past, north * std::cos(psi) + east * std::sin(psi)); // (along its way there)
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (r.state != ActivityState::Completed) return;
            if (isHold(f.completedOff)) f.completedOff = std::hypot(north, east);
            f.after = std::max(f.after, std::hypot(north, east));
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const EndStop& f = endStops[p.id];
            INFO(activityStateName(r.state) << "; " << f.past << " m past its last point; completed " << f.completedOff << " m from it, then within "
                                            << f.after << " m");
            CHECK(r.state == ActivityState::Completed);
            // (the worst: the Crazyflie 0.78 m past its point, the IRIS 0.62 m, the helicopters none - handed over within a metre
            // of it, the UH-1H swung 42 m past it and the UH-60A 12 m. Completed 0.98 to 1.01 m from it, within a metre along its
            // leg, then held within 1.01 m, the UH-60A's. Its position loop's approach completes the helicopters later than
            // their swing did: the UH-1H by 82 s, the UH-60A by 30 s)
            CHECK(f.past < 1.5);
            CHECK(f.completedOff < 1.5);
            CHECK(f.after < 2.0);
        });
    // A-GRA's segment performance (ADR-29 FA-6c1: WPT-06, WPT-10): segments at the tables' best range speed now, their
    // speed replaced - flown at it in the second's middle - and a speed change made at a segment's acceleration, either
    // held to what the aircraft can: a wing's from that speed to 90 % of it over 40 s through the air, once settled; a
    // rotorcraft's first, from its hover to its cruise over 20 s over the ground. The ramp's middle's rate against the one
    // the route reads back, and the route completed (judged soon after: a rotorcraft flies on at its best range speed,
    // and the Crazyflie's battery, which lasts the route, is spent 144 s after it)
    struct SegmentRoute {
        double from = 0.0, to = 0.0, asked = 0.0, best = 0.0, worst = 0.0;
        bool passed = false;
        std::vector<double> t, v;
        int samples = 0;
    };
    std::map<std::uint32_t, SegmentRoute> segmentRoutes;
    auto rampS = [&](const Plane& p) { // (a rotorcraft's ramp: 20 s, or longer at its most)
        const double most = perf(p).maxAccelerationMs2;
        return std::max(20.0, std::isfinite(most) && most > 0.0 ? p.cruiseMs / most : 20.0);
    };
    auto rampAhead = [&](const Plane& p) { return p.cruiseMs * (0.5 * rampS(p) + 20.0); }; // (its first segment: its ramp, and 20 s on)
    const double longRange = static_cast<double>(SpeedOptimization::LongRangeCruise);
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            double ahead = 0.0;
            auto point = [&](double on) {
                ahead += on;
                const PositionCommand q = pointFrom(p.start, ahead * c, ahead * sn, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            SegmentRoute& f = segmentRoutes[p.id];
            f = SegmentRoute{};
            f.best = optimalTasMs(&w.profile(p.id)->tables, longRange, p.start.altitudeMslM, p.start.fuelKg);
            REQUIRE(std::isfinite(f.best));
            std::vector<Waypoint> points;
            if (p.rotor) { // its ramp; to its best range speed, and on at it (its speed left out); on to its end, where it stops
                Waypoint a = point(rampAhead(p));
                a.speed = p.cruiseMs, a.speedReference = static_cast<double>(SpeedReference::GroundSpeed), a.accelerationMs2 = p.cruiseMs / 20.0;
                Waypoint b = point(60.0 * f.best);
                b.speedOptimization = longRange;
                points = {a, b, point(60.0 * f.best), point(40.0 * f.best)};
            } else { // to its best range speed, and on at it; then its ramp
                Waypoint a = point(60.0 * p.start.airspeedTrueMs), b = point(60.0 * f.best), e = point(60.0 * f.best);
                a.speedOptimization = longRange;
                e.speed = 0.9 * f.best, e.accelerationMs2 = 0.1 * f.best / 40.0;
                points = {a, b, e};
            }
            const CommandResult res = w.submit(p.id, RouteCommand{}, points);
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            Setpoint sp;
            if (res.accepted() && w.activitySetpoint(res.activity, sp)) {
                const Waypoint& ramp = sp.waypoints[p.rotor ? 0 : 2];
                f.from = p.rotor ? 0.0 : sp.waypoints[1].speed, f.to = ramp.speed, f.asked = ramp.accelerationMs2;
            }
            return res.accepted();
        },
        [&](const Plane& p) {
            const SegmentRoute& f = segmentRoutes[p.id];
            const double v = p.start.airspeedTrueMs;
            return p.rotor ? 1.1 * (rampS(p) + 200.0) + 10.0 : 1.5 * (60.0 * v / std::min(v, f.best) + 125.0) + 60.0;
        },
        [&](const Plane& p) {
            SegmentRoute& f = segmentRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const ActivityProgress& g = r.progress;
            if (!r.live()) return;
            const auto& s = *w.vehicleState(p.id);
            const double v = p.rotor ? std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]) : s.airspeedTrueMs; // (calm air: a rotorcraft's airspeed)
            const std::uint32_t ramp = p.rotor ? 0 : 2, best = p.rotor ? 2 : 1;
            const double lo = f.from + 0.2 * (f.to - f.from), hi = f.from + 0.8 * (f.to - f.from), way = f.to > f.from ? 1.0 : -1.0;
            if (g.segment == ramp && !f.passed) {
                if (way * (v - hi) > 0.0) f.passed = true;
                else if (way * (v - lo) >= 0.0) f.t.push_back(w.simTime()), f.v.push_back(v);
            }
            if (g.segment == best && g.segmentPercent > 50.0 && g.segmentPercent < 95.0) {
                const double now = optimalTasMs(&w.profile(p.id)->tables, longRange, s.altitudeMslM, s.fuelKg);
                f.worst = std::max(f.worst, std::abs(v - now) / now);
                ++f.samples;
            }
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const SegmentRoute& f = segmentRoutes[p.id];
            double rate = 0.0;
            if (f.t.size() > 2) {
                const double n = static_cast<double>(f.t.size());
                double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
                for (std::size_t i = 0; i < f.t.size(); ++i) sx += f.t[i], sy += f.v[i], sxx += f.t[i] * f.t[i], sxy += f.t[i] * f.v[i];
                rate = std::abs((n * sxy - sx * sy) / (n * sxx - sx * sx));
            }
            INFO(activityStateName(r.state) << "; from " << f.from << " to " << f.to << " m/s at " << f.asked << " m/s^2 (read back), flown at " << rate
                                            << " (" << f.t.size() << " samples); off its best range speed " << 100.0 * f.worst << " % ("
                                            << f.samples << " samples)");
            CHECK(r.state == ActivityState::Completed);
            // (the worst: in the ramp's middle a wing 13.2 % faster than asked, the RQ-4B's - every wing 9 to 13 %, its speed
            // loop catching up on the lag it began with - a rotorcraft 14.9 %, the UH-60's, the Crazyflie 7.0 % slower; off
            // the best range speed a wing 0.98 %, the C-130J's, a rotorcraft 0.09 %, the Crazyflie's)
            CHECK(f.t.size() > 5);
            CHECK(std::abs(rate - f.asked) < 0.2 * f.asked);
            CHECK(f.samples > 0);
            CHECK(f.worst < 0.02);
        });
    // A-GRA's climb optimisation (ADR-29 FA-6c2: WPT-08): a minute on (a rotorcraft's 20 s), then a best rate climb, an
    // efficient climb and a best rate descent through both, each change sized to take some 40 s at the rate its tables give
    // (a wing's 300 m at the least; a rotorcraft's 20 s, 10 to 40 m, within the Crazyflie's battery); then level. Through
    // each best rate change's middle half, its mean rate against the mean its tables give as it flies; the efficient climb
    // at its altitude at its point, as its tables timed it; the route completed
    struct ClimbRoute {
        double h0 = 0.0, dh = 0.0, arrived = kHold, promisedUp = 0.0, promisedDown = 0.0;
        double into[2] = {kHold, kHold}, outOf[2] = {kHold, kHold}, promised[2] = {0.0, 0.0}; ///< each change's middle half: when, what promised
        int samples[2] = {0, 0};
        std::uint32_t segment = 0;
    };
    std::map<std::uint32_t, ClimbRoute> climbRoutes;
    auto climbAt = [&](const Plane& p, bool up, double altitudeMslM, double tasMs, double fuelKg) {
        return route::climbRateMs(&w.profile(p.id)->tables, perf(p), p.rotor, up, altitudeMslM, tasMs, fuelKg);
    };
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            const double v = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, h0 = p.start.altitudeMslM;
            ClimbRoute& f = climbRoutes[p.id];
            f = ClimbRoute{};
            f.h0 = h0;
            const double up = climbAt(p, true, h0, v, p.start.fuelKg), down = climbAt(p, false, h0, v, p.start.fuelKg);
            f.dh = p.rotor ? std::clamp(20.0 * up, 10.0, 40.0) : std::clamp(40.0 * up, 300.0, 1500.0);
            double ahead = 0.0;
            auto point = [&](double seconds, double altitude) {
                ahead += v * seconds;
                const PositionCommand q = pointFrom(p.start, ahead * c, ahead * sn, altitude, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = altitude;
                return wp;
            };
            const double lead = p.rotor ? 20.0 : 60.0, spare = p.rotor ? 15.0 : 30.0;
            Waypoint a = point(lead, h0), b = point(2.0 * f.dh / up + spare, h0 + f.dh), e = point(2.0 * f.dh / up + spare, h0 + 2.0 * f.dh),
                     d = point(1.5 * 2.0 * f.dh / down + spare, h0), end = point(spare, h0);
            if (p.rotor) a.speed = v, a.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            b.climbOptimization = d.climbOptimization = static_cast<double>(ClimbOptimization::BestRate);
            e.climbOptimization = static_cast<double>(ClimbOptimization::ExtendedRange);
            const CommandResult res = w.submit(p.id, RouteCommand{}, std::vector<Waypoint>{a, b, e, d, end});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            f.promisedUp = up, f.promisedDown = down;
            return res.accepted();
        },
        [&](const Plane& p) {
            const ClimbRoute& f = climbRoutes[p.id];
            const double lead = p.rotor ? 20.0 : 60.0, spare = p.rotor ? 15.0 : 30.0;
            return 1.3 * (lead + 2.0 * (2.0 * f.dh / f.promisedUp + spare) + 1.5 * 2.0 * f.dh / f.promisedDown + 2.0 * spare) + 20.0;
        },
        [&](const Plane& p) {
            ClimbRoute& f = climbRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (!r.live()) return;
            const ActivityProgress& g = r.progress;
            const auto& s = *w.vehicleState(p.id);
            if (g.segment != f.segment) {
                if (f.segment == 2) f.arrived = s.altitudeMslM - (f.h0 + 2.0 * f.dh);
                f.segment = g.segment;
            }
            // (the climb from halfway up to nine tenths - its loops caught up on the lag it began with; the descent's middle
            // half, through both climbs)
            for (int j = 0; j < 2; ++j) {
                if (g.segment != (j == 0 ? 1u : 3u)) continue;
                const double lo = j == 0 ? f.h0 + 0.5 * f.dh : f.h0 + 0.5 * f.dh, hi = j == 0 ? f.h0 + 0.9 * f.dh : f.h0 + 1.5 * f.dh;
                const bool inside = s.altitudeMslM > lo && s.altitudeMslM < hi;
                if (inside && isHold(f.outOf[j])) {
                    if (isHold(f.into[j])) f.into[j] = w.simTime();
                    f.promised[j] += climbAt(p, j == 0, s.altitudeMslM, s.airspeedTrueMs, s.fuelKg), ++f.samples[j];
                }
                if (!inside && !isHold(f.into[j]) && isHold(f.outOf[j])) f.outOf[j] = w.simTime();
            }
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const ClimbRoute& f = climbRoutes[p.id];
            double off[2] = {kHold, kHold};
            for (int j = 0; j < 2; ++j)
                if (!isHold(f.outOf[j]) && f.samples[j] > 0) off[j] = ((j == 0 ? 0.4 : 1.0) * f.dh / (f.outOf[j] - f.into[j])) / (f.promised[j] / f.samples[j]) - 1.0;
            INFO(activityStateName(r.state) << "; " << f.dh << " m at a time: through its middle half, its best rate climb " << 100.0 * off[0]
                                            << " % off its tables' (" << f.samples[0] << " samples), its descent " << 100.0 * off[1] << " % ("
                                            << f.samples[1] << "); the efficient climb " << f.arrived << " m off at its point");
            CHECK(r.state == ActivityState::Completed);
            REQUIRE((!isHold(off[0]) && !isHold(off[1])));
            // (the worst: in the climb's second part the Mirage 2000 12.4 % and the RQ-4B 11.4 % faster than its tables' rate,
            // their vertical loops still catching up on the lag the climb began with - every other wing within 6.2 %, a
            // rotorcraft within 1.1 %; through the descent's middle half within 4.3 %, the Mirage's; the efficient climb within
            // 9.1 m of its point's altitude there, the J-20A's)
            CHECK(std::abs(off[0]) < 0.25);
            CHECK(std::abs(off[1]) < 0.1);
            REQUIRE(!isHold(f.arrived));
            CHECK(std::abs(f.arrived) < 0.05 * f.dh);
        });
    // A-GRA's required time of arrival (ADR-29 FA-6d1: WPT-11): a point three minutes on at its speed, given a window of 10 s
    // a tenth later - slowed - or, where the slowest it flies level cannot take that, a tenth sooner - sped up. It arrives
    // within 2 s of when it aimed (a quarter of the window inside it), its estimate halfway told within as much
    struct ArrivalRoute {
        double aimS = kHold, arrivedS = kHold, estimateS = kHold, t0 = 0.0;
        bool slowed = true;
    };
    std::map<std::uint32_t, ArrivalRoute> arrivalRoutes;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            const double v = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs;
            ArrivalRoute& f = arrivalRoutes[p.id];
            f = ArrivalRoute{};
            f.t0 = w.simTime();
            double slowest = kHold, fastest = kHold;
            route::levelSpeedsMs(&w.profile(p.id)->tables, perf(p), p.rotor, p.start.altitudeMslM, p.start.fuelKg, slowest, fastest);
            f.slowed = !(slowest > v / 1.1 - 1.0);
            const double later = f.slowed ? 1.1 : 0.9, planned = 180.0;
            auto point = [&](double ahead) {
                const PositionCommand q = pointFrom(p.start, ahead * c, ahead * sn, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            Waypoint a = point(60.0 * v), b = point(planned * v);
            a.speed = v;
            if (p.rotor) a.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            b.arrivalBeginS = f.t0 + later * planned - 5.0, b.arrivalEndS = f.t0 + later * planned + 5.0;
            f.aimS = f.slowed ? later * planned - 2.5 : later * planned + 2.5;
            const CommandResult res = w.submit(p.id, RouteCommand{}, std::vector<Waypoint>{a, b});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index << " (its level speeds " << slowest << " to " << fastest << ", at " << v << ")");
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane&) { return 1.1 * 180.0 * 1.2 + 40.0; },
        [&](const Plane& p) {
            ArrivalRoute& f = arrivalRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (!isHold(f.arrivedS)) return;
            if (!r.live()) {
                f.arrivedS = w.simTime() - f.t0;
                return;
            }
            ArrivalEstimate e;
            if (isHold(f.estimateS) && r.progress.percent >= 50.0 && w.activityArrival(activity[p.id], e)) f.estimateS = e.arrivalS - f.t0;
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const ArrivalRoute& f = arrivalRoutes[p.id];
            INFO(activityStateName(r.state) << "; " << (f.slowed ? "slowed" : "sped up") << ": aimed at " << f.aimS << " s, arrived at " << f.arrivedS
                                            << " s; its estimate halfway " << f.estimateS << " s");
            CHECK(r.state == ActivityState::Completed);
            REQUIRE((!isHold(f.arrivedS) && !isHold(f.estimateS)));
            // (the worst: every one 0.03 s after its aim, a control period - its schedule closing on its own error each update -
            // its estimate halfway its aim; 34 slowed, the Crazyflie, at a metre a second its slowest, sped up)
            CHECK(std::abs(f.arrivedS - f.aimS) < 2.0); // (FA-6's acceptance: within 2 s of a feasible time of arrival)
            CHECK(std::abs(f.estimateS - f.arrivedS) < 2.0);
        });
    // and through a loiter (ADR-29 FA-6g2): a loiter point a minute on at its speed - or, where its orbit is joined two radii
    // out, half a minute past that - an orbit of 1.25 times its turn's radius for 30 s, then a point two minutes on. First flown
    // as planned; once it tells when it would arrive, the route again from where it is, the point given a window of 10 s later
    // by 8 % of the time its legs take - to the join, from the orbit's tangent on - slowed, or, where its slowest cannot take a
    // tenth, as much sooner. It arrives within 2 s of when it aimed; its estimate as its orbit began, against when it arrived,
    // told
    struct LoiterArrival {
        double planned = kHold, aimS = kHold, begun = kHold, arrivedS = kHold, legsS = kHold;
        bool slowed = true;
        Waypoint a, b;
        RouteLoiter orbit;
    };
    std::map<std::uint32_t, LoiterArrival> loiterArrivals;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            const double v = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs;
            LoiterArrival& f = loiterArrivals[p.id];
            f = LoiterArrival{};
            double slowest = kHold, fastest = kHold;
            route::levelSpeedsMs(&w.profile(p.id)->tables, perf(p), p.rotor, p.start.altitudeMslM, p.start.fuelKg, slowest, fastest);
            f.slowed = !(slowest > v / 1.1 - 1.0);
            auto point = [&](double ahead) {
                const PositionCommand q = pointFrom(p.start, ahead * c, ahead * sn, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            const double radius = 1.25 * orbitRadius(p), toLoiter = std::max(60.0 * v, 2.0 * radius + 30.0 * v), after = 120.0 * v;
            f.a = point(toLoiter), f.b = point(toLoiter + after);
            f.legsS = (toLoiter - 2.0 * radius + std::sqrt(std::max(after * after - radius * radius, 0.0))) / std::max(v, 0.1);
            f.a.speed = v, f.a.kind = static_cast<double>(EndPointKind::LoiterPoint);
            if (p.rotor) f.a.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            f.orbit.point = 0, f.orbit.pattern.pattern = static_cast<double>(PatternKind::Orbit), f.orbit.pattern.durationS = 30.0;
            f.orbit.pattern.radiusM = radius;
            Waypoint open = f.b; // (as planned: a window it is inside)
            open.arrivalBeginS = w.simTime(), open.arrivalEndS = w.simTime() + 7200.0;
            const CommandResult res = w.submit(p.id, RouteCommand{}, std::vector<Waypoint>{f.a, open}, {}, std::vector<RouteLoiter>{f.orbit});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index << " (its level speeds " << slowest << " to " << fastest << ", at " << v << ")");
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane& p) {
            const double v = std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1), radius = 1.25 * orbitRadius(p);
            return 1.1 * (std::max(60.0, 2.0 * radius / v + 30.0) + 120.0) + 30.0 + (2.0 * kPi + 2.0) * radius / v + 60.0;
        },
        [&](const Plane& p) {
            LoiterArrival& f = loiterArrivals[p.id];
            if (!isHold(f.arrivedS)) return;
            ArrivalEstimate e;
            if (isHold(f.planned)) { // (as planned, when it would arrive: the route again from here, its window from that)
                if (!w.activityArrival(activity[p.id], e)) return;
                f.planned = e.arrivalS;
                const double shift = (f.slowed ? 0.08 : -0.08) * f.legsS;
                f.b.arrivalBeginS = f.planned + shift - 5.0, f.b.arrivalEndS = f.planned + shift + 5.0;
                f.aimS = f.slowed ? f.b.arrivalBeginS + 2.5 : f.b.arrivalEndS - 2.5;
                const CommandResult res = w.submit(p.id, RouteCommand{}, std::vector<Waypoint>{f.a, f.b}, {}, std::vector<RouteLoiter>{f.orbit});
                INFO(p.type << " refused: " << reasonName(res.reason) << " at " << res.index << " (" << constraintName(res.constraint) << "), its legs "
                            << f.legsS << " s, " << (f.slowed ? "slowed" : "sped up"));
                CHECK(res.accepted());
                if (res.accepted()) activity[p.id] = res.activity; // (else on as planned: the aim missed, told)
                return;
            }
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (!r.live()) {
                f.arrivedS = w.simTime();
                return;
            }
            if (isHold(f.begun) && r.progress.segment == 0 && r.progress.segmentPercent >= 99.9 && w.activityArrival(activity[p.id], e)) f.begun = e.arrivalS;
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const LoiterArrival& f = loiterArrivals[p.id];
            INFO(activityStateName(r.state) << "; " << (f.slowed ? "slowed" : "sped up") << ": aimed " << f.aimS - f.planned << " s from as planned, arrived "
                                            << f.arrivedS - f.aimS << " s from its aim; its estimate as its orbit began " << f.begun - f.arrivedS
                                            << " s from when it arrived");
            CHECK(r.state == ActivityState::Completed);
            REQUIRE((!isHold(f.arrivedS) && !isHold(f.begun)));
            // (the worst: every one within 0.06 s of its aim, a control period or two - 34 slowed, the Crazyflie sped up - its estimate
            // as its orbit began within as much of when it arrived)
            CHECK(std::abs(f.arrivedS - f.aimS) < 2.0); // (FA-6's acceptance: within 2 s of a feasible time of arrival)
        });
    // A-GRA's planned inertial states (ADR-29 FA-6d2: WPT-20): a point half a minute on at its speed, and another two and a half
    // minutes on; between them two states - up, a minute on from the first point, by what it climbs in 20 s at a third of its
    // most (60 m at the most), and back down a minute later - each timed a tenth later than its speed makes it (slowed), or
    // where its slowest cannot take that, a tenth sooner (sped up). It passes each within 2 s of its time, near its altitude
    struct StatesRoute {
        route::Leg line;
        double t0 = 0.0, dh = 0.0, along[2] = {0.0, 0.0}, due[2] = {kHold, kHold}, passed[2] = {kHold, kHold}, off[2] = {kHold, kHold};
        bool slowed = true;
    };
    std::map<std::uint32_t, StatesRoute> statesRoutes;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            const double v = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, h0 = p.start.altitudeMslM;
            StatesRoute& f = statesRoutes[p.id];
            f = StatesRoute{};
            f.t0 = w.simTime();
            double slowest = kHold, fastest = kHold;
            route::levelSpeedsMs(&w.profile(p.id)->tables, perf(p), p.rotor, h0, p.start.fuelKg, slowest, fastest);
            f.slowed = !(slowest > v / 1.1 - 1.0);
            const double later = f.slowed ? 1.1 : 0.9;
            f.dh = std::min(60.0, 20.0 * perf(p).maxClimbMs / 3.0);
            auto point = [&](double seconds, double altitude) {
                const PositionCommand q = pointFrom(p.start, seconds * v * c, seconds * v * sn, altitude, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = altitude;
                return wp;
            };
            Waypoint a = point(30.0, h0), b = point(180.0, h0);
            a.speed = v;
            if (p.rotor) a.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            f.line = route::makeLeg(p.start.latitudeRad, p.start.longitudeRad, b.latitudeRad, b.longitudeRad, false);
            std::vector<RouteState> states;
            const double at[2] = {90.0, 150.0}, high[2] = {h0 + f.dh, h0};
            for (int j = 0; j < 2; ++j) {
                const Waypoint q = point(at[j], high[j]);
                RouteState st;
                st.point = 1, st.latitudeRad = q.latitudeRad, st.longitudeRad = q.longitudeRad, st.altitudeM = high[j];
                st.timeS = f.t0 + later * at[j];
                f.due[j] = later * at[j], f.along[j] = route::onLeg(f.line, q.latitudeRad, q.longitudeRad).alongM;
                states.push_back(st);
            }
            const CommandResult res = w.submit(p.id, RouteCommand{}, std::vector<Waypoint>{a, b}, {}, {}, states);
            INFO("refused: " << reasonName(res.reason) << " at " << res.index << " (" << constraintName(res.constraint) << "; its level speeds " << slowest
                             << " to " << fastest << ", at " << v << ", " << f.dh << " m up)");
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane&) { return 1.1 * 180.0 * 1.2 + 40.0; },
        [&](const Plane& p) {
            StatesRoute& f = statesRoutes[p.id];
            const auto& s = *w.vehicleState(p.id);
            const double x = route::onLeg(f.line, s.latitudeRad, s.longitudeRad).alongM;
            for (int j = 0; j < 2; ++j)
                if (isHold(f.passed[j]) && x >= f.along[j])
                    f.passed[j] = w.simTime() - f.t0, f.off[j] = s.altitudeMslM - (p.start.altitudeMslM + (j == 0 ? f.dh : 0.0));
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const StatesRoute& f = statesRoutes[p.id];
            INFO(activityStateName(r.state) << "; " << (f.slowed ? "slowed" : "sped up") << ", " << f.dh << " m up: due at " << f.due[0] << " s, passed at "
                                            << f.passed[0] << " s, " << f.off[0] << " m off; due at " << f.due[1] << " s, passed at " << f.passed[1] << " s, "
                                            << f.off[1] << " m off");
            CHECK(r.state == ActivityState::Completed);
            // (the worst: every one past each 0.03 s after its time, a control period; the Mirage 2000 9.2 m over at the top,
            // where its profile turns from climb to descent, the RQ-4B 9.2 m under - every other within 4.2 m, a rotorcraft
            // within 0.01 m; 34 slowed, the Crazyflie sped up)
            for (int j = 0; j < 2; ++j) {
                REQUIRE(!isHold(f.passed[j]));
                CHECK(std::abs(f.passed[j] - f.due[j]) < 2.0); // (FA-6's acceptance: within 2 s of a feasible time)
                CHECK(std::abs(f.off[j]) < std::max(3.0, 0.25 * f.dh));
            }
        });
    // A-GRA's required navigation performance (ADR-29 FA-6d3: WPT-21): half a minute straight on at its speed, an RNP of a
    // fifth of its scale; a fly-over point there, a quarter turn right, and half a minute on at an RNP of 0.1 m. Straight on it
    // never says it is off; past the corner it does, at every step that ends farther off than 0.1 m and at none that ends
    // well within it
    struct RnpRoute {
        double farthest[2] = {0.0, 0.0};
        int flagged[2] = {0, 0}, missed = 0, spurious = 0;
        bool seen = false;
    };
    std::map<std::uint32_t, RnpRoute> rnpRoutes;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            const double v = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, ahead = 30.0 * v;
            rnpRoutes[p.id] = RnpRoute{};
            auto point = [&](double north, double east) {
                const PositionCommand q = pointFrom(p.start, north, east, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            Waypoint a = point(ahead * c, ahead * sn), b = point(ahead * c - ahead * sn, ahead * sn + ahead * c);
            a.speed = v, a.turn = static_cast<double>(TurnType::FlyOver), a.rnpM = 0.2 * p.scale(), b.rnpM = 0.1;
            if (p.rotor) a.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            const CommandResult res = w.submit(p.id, RouteCommand{}, std::vector<Waypoint>{a, b});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane&) { return 60.0 * 1.3 + 20.0; },
        [&](const Plane& p) {
            RnpRoute& f = rnpRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (!r.live()) return;
            const std::uint32_t k = std::min<std::uint32_t>(r.progress.segment, 1);
            const bool flagged = (r.constraints & kActivityNavigationPerformance) != 0;
            const double off = std::abs(r.progress.crossTrackM);
            f.farthest[k] = std::max(f.farthest[k], off);
            if (flagged) ++f.flagged[k];
            if (k == 1 && off > 0.1 && !flagged) ++f.missed;
            if (k == 1 && off < 0.09 && flagged) ++f.spurious;
            f.seen = (r.constraintsSeen & kActivityNavigationPerformance) != 0;
        },
        [&](const Plane& p, const Lows&) {
            const RnpRoute& f = rnpRoutes[p.id];
            INFO("straight on " << f.farthest[0] << " m off at most (RNP " << 0.2 * p.scale() << " m), said so " << f.flagged[0] << " times; past the corner "
                                << f.farthest[1] << " m off at most (RNP 0.1 m), said so " << f.flagged[1] << " times, missed " << f.missed
                                << ", well within " << f.spurious);
            // (the worst: straight on the UH-1H 5.86 m off, of 60; every wing within 0.95 m. Past the corner a wing 150 m (the
            // Skua) to 7.6 km (the C-17A) off, turning as it can after flying over it; a rotorcraft 1.4 m (the Crazyflie) to 60 m)
            CHECK(f.flagged[0] == 0);
            CHECK(f.farthest[1] > 0.1);
            CHECK(f.flagged[1] > 0);
            CHECK(f.seen);
            CHECK(f.missed == 0);
            CHECK(f.spurious == 0);
        });
    // A-GRA's paths and links (ADR-29 FA-6e1: WPT-13, WPT-14): a path of one point half a minute on, linked into a second -
    // a square of half-minute sides, round and round - and a third linked from nowhere, behind to the left. It flies the
    // first, then round the square, the points named as given, and never near the third
    struct PathsRoute {
        std::vector<std::uint32_t> order;
        double nearestLost = 1e30;
        PositionCommand lost{};
    };
    std::map<std::uint32_t, PathsRoute> pathsRoutes;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            const double v = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs;
            PathsRoute& f = pathsRoutes[p.id];
            f = PathsRoute{};
            auto point = [&](double ahead, double right) { // (seconds at its speed)
                const PositionCommand q = pointFrom(p.start, (ahead * c - right * sn) * v, (ahead * sn + right * c) * v, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            std::vector<Waypoint> points = {point(30, 0), point(60, 0), point(60, 30), point(90, 30), point(90, 0), point(0, -60)};
            points[0].speed = v;
            if (p.rotor) points[0].speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            points[0].next = 1.0, points[4].next = 1.0; // (on into the square; round it)
            f.lost = pointFrom(p.start, (0 * c + 60 * sn) * v, (0 * sn - 60 * c) * v, p.start.altitudeMslM, 0.0);
            const std::vector<RoutePath> paths = {RoutePath{1, static_cast<double>(PathType::Ingress), 0, 1},
                                                  RoutePath{2, static_cast<double>(PathType::Primary), 1, 4},
                                                  RoutePath{3, static_cast<double>(PathType::ReturnToBase), 5, 1}};
            const CommandResult res = w.submit(p.id, RouteCommand{}, points, {}, {}, {}, paths);
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane&) { return 330.0; },
        [&](const Plane& p) {
            PathsRoute& f = pathsRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (!r.live()) return;
            if (f.order.empty() || f.order.back() != r.progress.segment) f.order.push_back(r.progress.segment);
            const auto& s = *w.vehicleState(p.id);
            const double north = (f.lost.latitudeRad - s.latitudeRad) * kEarthM, east = (f.lost.longitudeRad - s.longitudeRad) * kEarthM * std::cos(s.latitudeRad);
            f.nearestLost = std::min(f.nearestLost, std::hypot(north, east));
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const PathsRoute& f = pathsRoutes[p.id];
            std::string flown;
            for (std::size_t i = 0; i < f.order.size() && i < 12; ++i) flown += (i ? " " : "") + std::to_string(f.order[i]);
            INFO(activityStateName(r.state) << "; flew " << flown << ", nearest the third path's point " << f.nearestLost << " m");
            CHECK(r.live()); // (round and round: no end)
            REQUIRE(f.order.size() >= 7);
            CHECK(flown.rfind("0 1 2 3 4 1 2", 0) == 0);
            CHECK(f.nearestLost > p.scale()); // (every one nearest at its start, a minute off: 2.99 scales at the worst, the EC-130H)
        });
    // A-GRA's conditional branches (ADR-29 FA-6e2: WPT-15): the same first two paths, the square's last linked round, and a
    // branch there out to a third - a point half a minute to its right - once it has come there twice. It flies round twice,
    // then out, and the route completes there
    std::map<std::uint32_t, std::vector<std::uint32_t>> branchedOrders;
    // (a helicopter's stop at a route's end: its position loop's approach from where it would stop, slow - as the end stop's
    // case gives it time: the UH-1H's 91 s, the UH-60A's 48 s. A multirotor's is quick, and the Crazyflie's battery short)
    auto approachS = [&](const Plane& p) {
        const double bandwidth = perf(p).velocityBandwidthRadS;
        return p.cls != Class::Helicopter ? 0.0 : std::isfinite(bandwidth) && bandwidth > 0.0 ? 20.0 / bandwidth : 150.0;
    };
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            const double v = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs;
            branchedOrders[p.id].clear();
            auto point = [&](double ahead, double right) { // (seconds at its speed)
                const PositionCommand q = pointFrom(p.start, (ahead * c - right * sn) * v, (ahead * sn + right * c) * v, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            std::vector<Waypoint> points = {point(30, 0), point(60, 0), point(60, 30), point(90, 30), point(90, 0), point(90, -30)};
            points[0].speed = v;
            if (p.rotor) points[0].speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            points[0].next = 1.0, points[4].next = 1.0;
            const std::vector<RoutePath> paths = {RoutePath{1, static_cast<double>(PathType::Ingress), 0, 1},
                                                  RoutePath{2, static_cast<double>(PathType::Primary), 1, 4},
                                                  RoutePath{3, static_cast<double>(PathType::Egress), 5, 1}};
            RouteBranch out;
            out.point = 4, out.next = 5.0, out.captures = 2.0, out.capturesComparison = static_cast<double>(Comparison::GreaterEqual);
            RouteCommand route; // (it loiters at its end: a rotorcraft hovers there, a wing orbits)
            route.end = static_cast<double>(EndBehavior::Loiter);
            const CommandResult res = w.submit(p.id, route, points, {}, {}, {}, paths, std::vector<RouteBranch>{out});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        // (a rotorcraft's done within five minutes, inside the Crazyflie's battery, a helicopter's stop at its end after; a
        // heavy's turns, wider than the square's sides, take it past eight)
        [&](const Plane& p) { return p.rotor ? 330.0 + approachS(p) : 600.0; },
        [&](const Plane& p) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            std::vector<std::uint32_t>& order = branchedOrders[p.id];
            if (r.live() && (order.empty() || order.back() != r.progress.segment)) order.push_back(r.progress.segment);
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const std::vector<std::uint32_t>& order = branchedOrders[p.id];
            std::string flown;
            for (std::size_t i = 0; i < order.size() && i < 14; ++i) flown += (i ? " " : "") + std::to_string(order[i]);
            INFO(activityStateName(r.state) << " " << reasonName(r.reason) << "; flew " << flown << "; ended " << r.endTime);
            CHECK(flown == "0 1 2 3 4 1 2 3 4 5"); // (round twice, then out)
            CHECK(r.state == ActivityState::Completed); // (all 35: 256 s after the NEW, the fighters, to 500 s, the C-17A)
        });
    // and on what it has left (FA-6e2b): the same route, out at the square's last once its fuel or charge is below what it had
    // at the NEW, as its navigation report says - the first time it comes there
    std::map<std::uint32_t, std::vector<std::uint32_t>> enduranceOrders;
    std::map<std::uint32_t, NavigationReport> enduranceAt;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            const double v = p.rotor ? p.cruiseMs : p.start.airspeedTrueMs;
            enduranceOrders[p.id].clear();
            auto point = [&](double ahead, double right) { // (seconds at its speed)
                const PositionCommand q = pointFrom(p.start, (ahead * c - right * sn) * v, (ahead * sn + right * c) * v, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            std::vector<Waypoint> points = {point(30, 0), point(60, 0), point(60, 30), point(90, 30), point(90, 0), point(90, -30)};
            points[0].speed = v;
            if (p.rotor) points[0].speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            points[0].next = 1.0, points[4].next = 1.0;
            const std::vector<RoutePath> paths = {RoutePath{1, kHold, 0, 1}, RoutePath{2, kHold, 1, 4}, RoutePath{3, kHold, 5, 1}};
            const NavigationReport n0 = w.navigationReport(p.id);
            enduranceAt[p.id] = n0;
            RouteBranch out;
            out.point = 4, out.next = 5.0, out.percent = n0.percent - 1e-6, out.enduranceComparison = static_cast<double>(Comparison::LessEqual);
            RouteCommand route;
            route.end = static_cast<double>(EndBehavior::Loiter);
            const CommandResult res = w.submit(p.id, route, points, {}, {}, {}, paths, std::vector<RouteBranch>{out});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index << "; " << energyName(n0.energy) << " " << n0.percent << " %");
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane& p) { return p.rotor ? 250.0 + approachS(p) : 420.0; },
        [&](const Plane& p) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            std::vector<std::uint32_t>& order = enduranceOrders[p.id];
            if (r.live() && (order.empty() || order.back() != r.progress.segment)) order.push_back(r.progress.segment);
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const std::vector<std::uint32_t>& order = enduranceOrders[p.id];
            std::string flown;
            for (std::size_t i = 0; i < order.size() && i < 14; ++i) flown += (i ? " " : "") + std::to_string(order[i]);
            const NavigationReport n = w.navigationReport(p.id);
            INFO(activityStateName(r.state) << "; flew " << flown << "; " << n.percent << " % left of " << enduranceAt[p.id].percent);
            CHECK(flown == "0 1 2 3 4 5"); // (out the first time)
            CHECK(r.state == ActivityState::Completed); // (all 35: 161 s after the NEW, the fighters, to 229 s, the B-52H and the E-3G)
            CHECK(n.percent < enduranceAt[p.id].percent);
        });
    // A-GRA's civil path terminators (ADR-29 FA-6f1: WPT-19): two orbit radii ahead, then a radius to fix's arc - a quarter
    // circle of three radii round to the right, near 20 degrees of bank, as an RNP procedure's are laid out; its centre
    // given - and two radii on: off the arc all along it, over the ground from its centre (FA-6's criterion: within 20 m)
    struct RadiusRoute {
        double centreLatRad = 0.0, centreLonRad = 0.0, radiusM = 0.0, worst = 0.0, cross = 0.0;
        int samples = 0;
    };
    std::map<std::uint32_t, RadiusRoute> rfRoutes;
    auto overGround = [](double lat0, double lon0, double lat, double lon) { // (the great circle's distance)
        const double h = std::pow(std::sin(0.5 * (lat - lat0)), 2.0) + std::cos(lat0) * std::cos(lat) * std::pow(std::sin(0.5 * (lon - lon0)), 2.0);
        return 2.0 * 6371008.8 * std::asin(std::min(1.0, std::sqrt(h)));
    };
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double R = orbitRadius(p), psi = p.start.eulerRad[2], r = 3.0 * R;
            const double c = std::cos(psi), sn = std::sin(psi);
            auto point = [&](double ahead, double right) {
                const PositionCommand q = pointFrom(p.start, ahead * c - right * sn, ahead * sn + right * c, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            std::vector<Waypoint> points = {point(2.0 * R, 0.0), point(2.0 * R + r, r), point(2.0 * R + r, r + 2.0 * R)};
            points[1].terminator = static_cast<double>(PathTerminator::RadiusToFix);
            const Waypoint centre = point(2.0 * R, r);
            RouteTerminator arc;
            arc.point = 1, arc.centerLatitudeRad = centre.latitudeRad, arc.centerLongitudeRad = centre.longitudeRad, arc.clockwise = 1.0;
            RadiusRoute& f = rfRoutes[p.id];
            f = RadiusRoute{};
            f.centreLatRad = centre.latitudeRad, f.centreLonRad = centre.longitudeRad;
            f.radiusM = overGround(centre.latitudeRad, centre.longitudeRad, points[0].latitudeRad, points[0].longitudeRad);
            const CommandResult res = w.submit(p.id, RouteCommand{}, points, {}, {}, {}, {}, {}, std::vector<RouteTerminator>{arc});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane& p) { return (4.0 + 4.8) * orbitRadius(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) * 1.5 + 60.0; },
        [&](const Plane& p) {
            RadiusRoute& f = rfRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const ActivityProgress& g = r.progress;
            if (!r.live() || g.segment != 1 || g.segmentPercent < 5.0 || g.segmentPercent > 95.0) return;
            const auto& s = *w.vehicleState(p.id);
            f.worst = std::max(f.worst, std::abs(overGround(f.centreLatRad, f.centreLonRad, s.latitudeRad, s.longitudeRad) - f.radiusM));
            f.cross = std::max(f.cross, std::abs(g.crossTrackM));
            ++f.samples;
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const RadiusRoute& f = rfRoutes[p.id];
            INFO(activityStateName(r.state) << "; its radius to fix's arc " << f.radiusM << " m, off it " << f.worst << " m (its own cross-track "
                                            << f.cross << " m; " << f.samples << " samples)");
            CHECK(r.state == ActivityState::Completed);
            CHECK(f.samples > 0);
            // (the worst: a wing's the E-3G's 18.3 m on its 22.4 km arc, the B-52H's 17.1 m - its route's own cross-track the same
            // to a centimetre; a rotorcraft's the UH-1H's 4.4 m, 0.7 % of its arc. Laid out in the plane at its start, as a
            // start turn point's is, and read in the flat plane this file lays points out in, the C-17A's 28 km arc read 72 m off)
            CHECK(f.worst < (p.rotor ? std::max(0.5, 0.05 * f.radiusM) : 20.0));
        });
    // and a course to altitude (FA-6f2a): two orbit radii ahead, then up on its course - 30 s of its best climb, 300 m at
    // most (a Crazyflie's battery takes it no higher) - its point four radii on, where its gradient climbs it, and direct
    // to a point two radii to the right of that: its course ended at its altitude (within 10 m), wherever it came to it,
    // the route completed
    struct AltitudeRoute {
        double to = 0.0, endedUp = kHold, endedAlongM = kHold, planM = 0.0;
        PositionCommand start;
    };
    std::map<std::uint32_t, AltitudeRoute> caRoutes;
    auto climbOf = [&](const Plane& p) { // (its most climb, as its performance says: 2 m/s where it says none)
        const Performance* most = w.performance(p.id);
        return most && std::isfinite(most->maxClimbMs) && most->maxClimbMs > 0.0 ? most->maxClimbMs : 2.0;
    };
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double R = orbitRadius(p), psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi), rise = std::min(300.0, 30.0 * climbOf(p));
            auto point = [&](double ahead, double right, double up) {
                const PositionCommand q = pointFrom(p.start, ahead * c - right * sn, ahead * sn + right * c, p.start.altitudeMslM + up, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            std::vector<Waypoint> points = {point(2.0 * R, 0.0, 0.0), point(6.0 * R, 0.0, rise), point(6.0 * R, 2.0 * R, rise)};
            points[1].terminator = static_cast<double>(PathTerminator::CourseToAltitude);
            points[2].terminator = static_cast<double>(PathTerminator::DirectToFix);
            AltitudeRoute& f = caRoutes[p.id];
            f = AltitudeRoute{};
            f.to = p.start.altitudeMslM + rise, f.planM = 4.0 * R;
            f.start = pointFrom(p.start, 2.0 * R * c, 2.0 * R * sn, p.start.altitudeMslM, 0.0);
            const CommandResult res = w.submit(p.id, RouteCommand{}, points);
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane& p) { return (10.0 * orbitRadius(p) / std::max(p.rotor ? p.cruiseMs : p.start.airspeedTrueMs, 0.1) + 30.0) * 1.5 + 120.0; },
        [&](const Plane& p) {
            AltitudeRoute& f = caRoutes[p.id];
            const ActivityRecord& r = *w.activity(activity[p.id]);
            if (!isHold(f.endedUp) || r.progress.segment != 2) return;
            const auto& s = *w.vehicleState(p.id);
            f.endedUp = s.altitudeMslM, f.endedAlongM = distanceTo(s, f.start);
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            const AltitudeRoute& f = caRoutes[p.id];
            INFO(activityStateName(r.state) << "; its course ended at " << f.endedUp << " m (" << f.to << "), " << f.endedAlongM << " m on (planned "
                                            << f.planM << ")");
            CHECK(r.state == ActivityState::Completed);
            CHECK(f.endedUp > f.to - 10.5);
        });
    // and a hold once round (FA-6f2b): a loiter point three orbit radii ahead with a hold, flown once round (its entry, a lap,
    // its fix), and on two radii to its right: held, then completed
    std::map<std::uint32_t, double> heldIn, heldOut;
    run("fsim.guidance.route", 0.0,
        [&](const Plane& p) {
            const double R = orbitRadius(p), psi = p.start.eulerRad[2], c = std::cos(psi), sn = std::sin(psi);
            auto point = [&](double ahead, double right) {
                const PositionCommand q = pointFrom(p.start, ahead * c - right * sn, ahead * sn + right * c, p.start.altitudeMslM, 0.0);
                Waypoint wp;
                wp.latitudeRad = q.latitudeRad, wp.longitudeRad = q.longitudeRad, wp.altitudeM = q.altitudeMslM;
                return wp;
            };
            std::vector<Waypoint> points = {point(3.0 * R, 0.0), point(3.0 * R, 2.0 * R)};
            points[0].kind = static_cast<double>(EndPointKind::LoiterPoint);
            points[0].terminator = static_cast<double>(PathTerminator::HoldingWithFixTermination);
            RouteLoiter hold;
            hold.point = 0, hold.pattern.pattern = static_cast<double>(PatternKind::Hold);
            heldIn[p.id] = heldOut[p.id] = kHold;
            const CommandResult res = w.submit(p.id, RouteCommand{}, points, {}, std::vector<RouteLoiter>{hold});
            INFO("refused: " << reasonName(res.reason) << " at " << res.index);
            CHECK(res.accepted());
            activity[p.id] = res.activity;
            return res.accepted();
        },
        [&](const Plane& p) { return p.rotor ? 330.0 : 10.0 * orbitRadius(p) / std::max(p.start.airspeedTrueMs, 0.1) * 1.5 + 420.0; },
        [&](const Plane& p) {
            const ActivityProgress& g = w.activity(activity[p.id])->progress;
            if (isHold(heldIn[p.id]) && g.segment == 0 && g.segmentPercent >= 99.9) heldIn[p.id] = w.simTime();
            if (isHold(heldOut[p.id]) && g.segment == 1) heldOut[p.id] = w.simTime();
        },
        [&](const Plane& p, const Lows&) {
            const ActivityRecord& r = *w.activity(activity[p.id]);
            INFO(activityStateName(r.state) << "; held " << heldOut[p.id] - heldIn[p.id] << " s");
            CHECK(r.state == ActivityState::Completed);
            CHECK(heldOut[p.id] - heldIn[p.id] > 30.0); // (the least: a rate-one lap's 240 s, the rotorcraft's, the Skua's, the C172's;
                                                        // the most the C-17A's 628 s, its turns at 25 degrees of bank)
        });
    run("fsim.guidance.hover", 0.0, [&](const Plane& p) { return w.submit(p.id, behavior("hover")).accepted(); }, secs(30.0, 30.0), none,
        [&](const Plane& p, const Lows&) {
            const auto& s = *w.vehicleState(p.id);
            CHECK(groundDistance(s, p.start) < 0.1); // (the worst: 0.02 m)
            CHECK(std::abs(s.altitudeMslM - p.start.altitudeMslM) < 0.1);
        });
    // aerobatics: an aileron roll completes; a loop completes, or ends honestly - refused for its entry
    // speed, or given up when the aircraft leaves its envelope (below its least airspeed it may then go).
    // From cruise 7 of the 17 complete; 9 give up, 7 of them fighters floating over the top below their
    // least airspeed; the Mirage 2000 is refused (ADR-29 FA-3d)
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


TEST_CASE("fleet: the command interface on every aircraft - validated and read back, queued and scheduled, disabled and enabled, "
          "a task, named controllers, a route's end points (FA-2), FA's own plan (FA-7b)",
          "[fleet]") {
    Fleet fleet;
    session::World& w = fleet.world();
    auto& planes = fleet.planes();
    fleet.standard();
    constexpr const char* kHsa = "fsim.guidance.hsa";
    std::map<std::uint32_t, Lows> lows;
    auto watch = [&] {
        for (const auto& p : planes) lows[p.id].see(*w.vehicleState(p.id));
    };
    // an hsa 30 degrees right of where each began, at its speed and height (a rotorcraft turns where it hovers)
    auto turn = [](const Plane& p) {
        HsaCommand h;
        h.headingRad = std::remainder(p.start.eulerRad[2] + 30.0 * kDeg, 2.0 * kPi);
        return Command(h);
    };
    std::map<std::uint32_t, ActivityId> flying, queued, scheduled;

    // validated, then flown and read back: its setpoint completed from where it flies, what it commands
    for (const auto& p : planes) {
        INFO(p.type << " (" << className(p.cls) << ")");
        REQUIRE(p.offered.count(kHsa) == 1);
        CommandOptions check;
        check.validateOnly = true;
        const std::size_t records = w.activities(p.id).size();
        CHECK(w.submit(p.id, turn(p), check).status == CommandStatus::Valid);
        CHECK(w.activities(p.id).size() == records); // (nothing flies)
        const CommandResult a = w.submit(p.id, turn(p));
        REQUIRE(a.accepted());
        flying[p.id] = a.activity;
        Setpoint read;
        REQUIRE(w.activitySetpoint(a.activity, read));
        const auto* h = std::get_if<HsaCommand>(&std::get<Command>(read.command));
        REQUIRE(h != nullptr);
        CHECK((!isHold(h->speed) && !isHold(h->altitudeM) && h->altitudeReference == static_cast<double>(AltitudeReference::Msl)));
        CHECK(w.endPoints(a.activity).empty());
    }
    fleet.fly(5.0, watch);
    for (const auto& p : planes) {
        INFO(p.type << " (" << className(p.cls) << ")");
        CHECK(w.activity(flying[p.id])->state == ActivityState::Active);
        Setpoint read;
        REQUIRE(w.activitySetpoint(flying[p.id], read));
        const VehicleCommandState c = w.commandState(p.id);
        CHECK((c.altitudeM == std::get<HsaCommand>(std::get<Command>(read.command)).altitudeM &&
               c.altitudeReference == static_cast<double>(AltitudeReference::Msl)));
        CHECK(std::isnan(c.northAccelerationMs2) == p.rotor); // (a wing's, from its loops' longitudinal acceleration and load factor)
    }

    // queued behind what flies, then scheduled: what waits starts when it may
    for (const auto& p : planes) {
        INFO(p.type);
        CommandOptions polite;
        polite.interrupt = false;
        const CommandResult q = w.submit(p.id, Command(Fleet::still(p, *w.vehicleState(p.id))), polite);
        REQUIRE(q.accepted());
        CHECK((q.flags & kDeferred) != 0);
        const ActivityRecord& r = *w.activity(q.activity);
        CHECK((r.waiting == ActivityWait::Queued && r.waitingFor == flying[p.id]));
        queued[p.id] = q.activity;
    }
    fleet.fly(1.0, watch);
    for (const auto& p : planes) {
        INFO(p.type);
        CHECK(w.activity(queued[p.id])->waiting == ActivityWait::Queued);
        CHECK(w.cancel(flying[p.id]).status == CommandStatus::Canceled);
        CHECK((w.activity(queued[p.id])->waiting == ActivityWait::None && w.activity(queued[p.id])->live())); // (its axes free: it flies)
        CommandOptions later;
        later.window.startNotBefore = w.simTime() + 2.0;
        const CommandResult s = w.submit(p.id, turn(p), later);
        REQUIRE(s.accepted());
        CHECK(w.activity(s.activity)->waiting == ActivityWait::Scheduled);
        scheduled[p.id] = s.activity;
    }
    fleet.fly(3.0, watch);
    for (const auto& p : planes) {
        INFO(p.type);
        CHECK(w.activity(scheduled[p.id])->state == ActivityState::Active);
        const ActivityRecord& q = *w.activity(queued[p.id]);
        CHECK((q.state == ActivityState::Canceled && q.reason == Reason::Preempted && q.by == scheduled[p.id]));
        // disabled: kept, not flying
        CHECK(w.activityCommand(Source::Policy, scheduled[p.id], ActivityCommand::Disable).accepted());
        CHECK(w.activity(scheduled[p.id])->state == ActivityState::Disabled);
    }
    fleet.fly(1.0, watch);
    for (const auto& p : planes) {
        INFO(p.type);
        CHECK(w.activity(scheduled[p.id])->state == ActivityState::Disabled);
        CHECK(w.activityCommand(Source::Policy, scheduled[p.id], ActivityCommand::Enable).accepted());
    }
    fleet.fly(1.0, watch);
    for (const auto& p : planes) {
        INFO(p.type);
        CHECK(w.activity(scheduled[p.id])->state == ActivityState::Active); // enabled: it flies again
        // a task: kept, flown on its task command, canceled
        REQUIRE(w.storeTask(p.id, 1, turn(p)) == Reason::None);
        REQUIRE(w.commandTask(p.id, 1).accepted());
    }
    fleet.fly(1.0, watch);
    for (const auto& p : planes) {
        INFO(p.type);
        CHECK(w.activity(scheduled[p.id])->reason == Reason::Preempted);
        CHECK(w.taskStatus(p.id, 1)->state == TaskState::Executing);
        CHECK(w.cancelTask(p.id, 1).status == CommandStatus::Canceled);
        CHECK(w.taskStatus(p.id, 1)->state == TaskState::Canceled);
        // named controllers: a grant is one controller's, and only it commands the capability
        REQUIRE(w.setControlMode(p.id, ControlMode::Granted) == Reason::None);
        CHECK(w.requestControl(p.id, kHsa, 1) == Reason::None);
        CHECK(w.requestControl(p.id, kHsa, 2) == Reason::AuthorityHeld);
        CommandOptions one, two;
        one.controller = 1, two.controller = 2;
        CHECK(w.submit(p.id, turn(p), two).reason == Reason::NotGranted);
        const CommandResult a = w.submit(p.id, turn(p), one);
        REQUIRE(a.accepted());
        flying[p.id] = a.activity;
        CHECK(w.cancel(Caller{Source::Policy, 2}, a.activity).reason == Reason::AuthorityHeld);
    }
    fleet.fly(2.0, watch);
    for (const auto& p : planes) {
        INFO(p.type);
        CHECK(w.activity(flying[p.id])->state == ActivityState::Active);
        CHECK(w.releaseControl(p.id, kHsa, 1) == Reason::None);
        CHECK(w.activity(flying[p.id])->reason == Reason::Released);
        REQUIRE(w.setControlMode(p.id, ControlMode::Open) == Reason::None);
        // a route: its points read back completed, its end points - turn points, then its last
        const auto& s = *w.vehicleState(p.id);
        const double d = p.scale(), psi = s.eulerRad[2];
        auto ahead = [&](double along, double right) {
            Waypoint wp;
            const double north = along * std::cos(psi) - right * std::sin(psi), east = along * std::sin(psi) + right * std::cos(psi);
            wp.latitudeRad = s.latitudeRad + north / kEarthM;
            wp.longitudeRad = s.longitudeRad + east / (kEarthM * std::cos(s.latitudeRad));
            return wp;
        };
        const std::vector<Waypoint> route = {ahead(d, 0.0), ahead(2.0 * d, d), ahead(3.0 * d, 0.0)};
        const CommandResult r = w.submit(p.id, RouteCommand{}, route);
        REQUIRE(r.accepted());
        flying[p.id] = r.activity;
        const std::vector<EndPoint> e = w.endPoints(r.activity);
        REQUIRE(e.size() == 3);
        CHECK((e[0].kind == EndPointKind::TurnPoint && e[1].kind == EndPointKind::TurnPoint && e[2].kind == EndPointKind::Waypoint));
        CHECK((e[2].latitudeRad == route[2].latitudeRad && e[2].index == 2));
    }
    fleet.fly(2.0, watch);
    for (const auto& p : planes) {
        INFO(p.type << " (" << className(p.cls) << ")");
        Setpoint read;
        REQUIRE(w.activitySetpoint(flying[p.id], read));
        REQUIRE(read.waypoints.size() == 3);
        const VehicleCommandState c = w.commandState(p.id);
        const std::uint32_t at = w.activity(flying[p.id])->progress.segment;
        CHECK((c.altitudeM == read.waypoints[std::min<std::uint32_t>(at, 2)].altitudeM &&
               c.altitudeReference == static_cast<double>(AltitudeReference::Msl))); // (the point flown to's, completed)
        keptSafe(p, lows[p.id]);
    }
    // FA's own (FA-7b): an airfield where each began, and a departure plan over the route it flies, loaded by the platform -
    // read only to MA, activated by it, and flown
    for (const auto& p : planes) {
        INFO(p.type);
        Airfield field;
        field.id = 1;
        Runway runway;
        runway.id = 1, runway.takeoff.start.latitudeRad = p.start.latitudeRad, runway.takeoff.start.longitudeRad = p.start.longitudeRad;
        runway.takeoff.start.altitudeM = 0.0;
        field.runways = {runway};
        REQUIRE(w.loadAirfield(p.id, field) == Reason::None);
        Setpoint read;
        REQUIRE(w.activitySetpoint(flying[p.id], read));
        RoutePlan departure;
        departure.id = 1, departure.version = 1, departure.waypoints = read.waypoints; // (the route flying, completed)
        departure.paths = {RoutePath{1, static_cast<double>(PathType::Airborne), 0, static_cast<std::uint32_t>(read.waypoints.size())}};
        REQUIRE(w.loadPlan(p.id, departure) == Reason::None);
        CHECK(w.planCommand(p.id, 1, PlanCommand::PrepareForUpload).reason == Reason::ReadOnlyPlan);
        REQUIRE(w.planCommand(p.id, 1, PlanCommand::PrepareForActivation).completed);
        const PlanCommandResult activated = w.planCommand(p.id, 1, PlanCommand::Activate);
        REQUIRE(activated.completed);
        flying[p.id] = activated.check.activity;
    }
    fleet.fly(2.0, watch);
    for (const auto& p : planes) {
        INFO(p.type << " (" << className(p.cls) << ")");
        const auto st = w.planStatus(p.id, 1);
        REQUIRE(st.has_value());
        CHECK((st->faOwned && st->execution == PlanExecution::Executing && st->activity == flying[p.id]));
        CHECK((w.airfields(p.id).size() == 1 && w.airfield(p.id, 1)->revision == 1));
        keptSafe(p, lows[p.id]);
    }
}
