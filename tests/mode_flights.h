#pragma once

// Flights for the Vehicle Interface's modes (docs/vehicle-interface.md): a
// wing at its speed, a rotorcraft settled in its hover, the wind, the track.

#include "fsim/Control.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace fsim::modes {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDeg = kPi / 180.0;
inline constexpr double kEarthM = 6371000.0;

inline session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 17;
    return o;
}

inline unsigned stepsFor(const session::World& w, double seconds) { return static_cast<unsigned>(std::lround(seconds / (w.dt() * w.frameSkip()))); }

/// A fixed wing flying east at its altitude and speed; `index` spaces vehicles 3 km apart north.
inline std::uint32_t wing(session::World& w, const std::string& type, double altitudeM, double tasMs, int index = 0) {
    session::VehicleSpec s;
    s.name = type + "-" + std::to_string(index);
    s.type = "jsbsim:" + type;
    s.initial.latitudeDeg += 0.03 * index;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = tasMs;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    return id;
}

/// A rotorcraft hovering at 100 m, spawned at its hover attitude and held still over the ground for `settleS`.
inline std::uint32_t rotor(session::World& w, const std::string& type, double settleS = 15.0, int index = 0) {
    double pitchDeg = 0.0, rollDeg = 0.0;
    {
        session::VehicleSpec probe;
        probe.name = type + "-probe";
        probe.type = "jsbsim:" + type;
        probe.initial.latitudeDeg += 1.0;
        probe.initial.altitudeMslM = 100.0;
        const auto id = w.createVehicle(probe);
        REQUIRE(id != 0);
        const auto& hover = w.profile(id)->hover;
        if (std::isfinite(hover.pitchAttitudeRad)) pitchDeg = hover.pitchAttitudeRad / kDeg;
        if (std::isfinite(hover.rollAttitudeRad)) rollDeg = hover.rollAttitudeRad / kDeg;
        REQUIRE(w.removeVehicle(id));
    }
    session::VehicleSpec s;
    s.name = type + "-" + std::to_string(index);
    s.type = "jsbsim:" + type;
    s.initial.latitudeDeg += 0.02 * index;
    s.initial.altitudeMslM = 100.0;
    s.initial.headingDeg = 0.0;
    s.initial.airspeedTrueMs = 0.0;
    s.initial.pitchDeg = pitchDeg;
    s.initial.rollDeg = rollDeg;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    control::VelocityCommand still;
    still.verticalSpeedMs = 0.0;
    still.northMs = still.eastMs = 0.0;
    REQUIRE(w.submit(id, still).accepted());
    if (settleS > 0.0) w.step(stepsFor(w, settleS));
    return id;
}

/// The wind, blowing from `fromDeg` true.
inline void setWind(session::World& w, double fromDeg, double speedMs) {
    sim::EnvironmentState e = w.environment();
    e.windDirectionDeg = fromDeg;
    e.windSpeedMs = speedMs;
    w.setEnvironment(e);
}

/// The track over the ground, rad; the ground speed, m/s.
inline double track(const sim::VehicleState& s) { return std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]); }
inline double groundSpeed(const sim::VehicleState& s) { return std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]); }
/// a - b, wrapped to +-180 deg, in degrees.
inline double degreesApart(double a, double b) { return std::remainder(a - b, 2.0 * kPi) / kDeg; }

/// North and east of (lat0, lon0), m.
inline void offset(const sim::VehicleState& s, double lat0, double lon0, double& north, double& east) {
    north = (s.latitudeRad - lat0) * kEarthM;
    east = (s.longitudeRad - lon0) * kEarthM * std::cos(lat0);
}

/// Ground that rises `gradient` metres per metre eastward from `longitudeRad` (flat to its west).
class RisingGround final : public sim::GroundProvider {
public:
    RisingGround(double longitudeRad, double latitudeRad, double gradient) noexcept
        : lon0_(longitudeRad), metresPerRad_(kEarthM * std::cos(latitudeRad)), gradient_(gradient) {}
    double heightAboveEllipsoidM(double, double longitudeRad) const override {
        const double east = (longitudeRad - lon0_) * metresPerRad_;
        return east > 0.0 ? gradient_ * east : 0.0;
    }

private:
    double lon0_, metresPerRad_, gradient_;
};

} // namespace fsim::modes
