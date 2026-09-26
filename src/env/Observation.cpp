#include "env/Observation.h"

#include "control/Catalog.h"
#include "core/Log.h"
#include "core/Units.h"
#include "env/Registry.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string_view>

namespace fsim::env {

namespace {

constexpr double kPi = 3.14159265358979323846;
double wrapPi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
float clampf(double v) { return static_cast<float>(std::clamp(v, -5.0, 5.0)); }

/// "state": 20 elements. Altitude in km, speeds in units of 100 m/s, angles in
/// radians, rates in rad/s, accelerations in g, plus the task's target errors.
class StateObservation final : public ObservationBuilder {
public:
    StateObservation()
        : names_{"alt_msl_km", "agl_km", "tas_100ms", "alpha", "beta", "roll", "pitch", "hdg_sin", "hdg_cos",
                 "p", "q", "r", "vz_down_100ms", "ax_g", "ay_g", "az_g",
                 "alt_err_km", "hdg_err_sin", "hdg_err_cos", "throttle"} {}

    std::size_t size() const noexcept override { return names_.size(); }
    const std::vector<std::string>& names() const noexcept override { return names_; }

    void build(const sim::VehicleState& s, const TaskState& t, float* o) const override {
        const double hdgErr = wrapPi(t.targetHeadingRad - s.eulerRad[2]);
        o[0] = clampf(s.altitudeMslM / 1000.0);
        o[1] = clampf(s.altitudeAglM / 1000.0);
        o[2] = clampf(s.airspeedTrueMs / 100.0);
        o[3] = clampf(s.alphaRad);
        o[4] = clampf(s.betaRad);
        o[5] = clampf(s.eulerRad[0]);
        o[6] = clampf(s.eulerRad[1]);
        o[7] = clampf(std::sin(s.eulerRad[2]));
        o[8] = clampf(std::cos(s.eulerRad[2]));
        o[9] = clampf(s.angularRateBodyRadS[0]);
        o[10] = clampf(s.angularRateBodyRadS[1]);
        o[11] = clampf(s.angularRateBodyRadS[2]);
        o[12] = clampf(s.velocityNedMs[2] / 100.0);
        o[13] = clampf(s.accelerationBodyMs2[0] / 9.80665);
        o[14] = clampf(s.accelerationBodyMs2[1] / 9.80665);
        o[15] = clampf(s.accelerationBodyMs2[2] / 9.80665);
        o[16] = clampf((t.targetAltitudeM - s.altitudeMslM) / 1000.0);
        o[17] = clampf(std::sin(hdgErr));
        o[18] = clampf(std::cos(hdgErr));
        o[19] = clampf(s.engineCount > 0 ? s.throttlePosition[0] : 0.0);
    }

private:
    std::vector<std::string> names_;
};

/// "surfaces": aileron, elevator, rudder in [-1, 1] and throttle in [-1, 1]
/// mapped to [0, 1]. Values outside the range are clamped.
double cl(float v) noexcept { return static_cast<double>(std::clamp(v, -1.0f, 1.0f)); }
double unit(float v) noexcept { return (cl(v) + 1.0) * 0.5; } // [-1,1] -> [0,1]

/// An action element's range: [-1, 1] onto [lo, hi].
struct Range {
    double lo, hi;
    double operator()(float v) const noexcept { return lo + (cl(v) + 1.0) * 0.5 * (hi - lo); }
};

/// A parameter's range for this aircraft where its profile narrowed it (the
/// platform's own catalog gives it wider): the fixed range otherwise.
Range aircraftRange(const std::vector<control::CapabilityDescriptor>& capabilities, std::string_view capability,
                    std::string_view parameter, Range fixed) {
    static const control::CapabilityCatalog platform; // the loops' own bounds
    auto find = [&](const std::vector<control::CapabilityDescriptor>& all) -> const control::ParameterInfo* {
        for (const auto& d : all)
            if (d.id == capability)
                for (const auto& p : d.parameters)
                    if (p.name == parameter) return &p;
        return nullptr;
    };
    const control::ParameterInfo* own = find(capabilities);
    const control::ParameterInfo* wide = find(platform.descriptors());
    if (!own || !wide || !std::isfinite(own->min) || !std::isfinite(own->max) || own->max <= own->min) return fixed;
    if (own->min == wide->min && own->max == wide->max) return fixed; // not the aircraft's: the platform's bounds
    return Range{own->min, own->max};
}

class SurfacesAction final : public ActionMapper {
public:
    SurfacesAction() : names_{"aileron", "elevator", "rudder", "throttle"} {}
    std::size_t size() const noexcept override { return 4; }
    const std::vector<std::string>& names() const noexcept override { return names_; }
    control::Level level() const noexcept override { return control::Level::Actuator; }
    control::Command map(const float* a) const override {
        control::ActuatorCommand c;
        c.aileron = cl(a[0]);
        c.elevator = cl(a[1]);
        c.rudder = cl(a[2]);
        c.throttle = unit(a[3]);
        c.gearDown = 0.0;
        return c;
    }

private:
    std::vector<std::string> names_;
};

/// roll (+-60 deg), pitch (+-25 deg), throttle: the attitude loop holds them.
/// With the aircraft's ranges, roll and pitch to its bank and pitch limits.
class AttitudeAction final : public ActionMapper {
public:
    AttitudeAction() : names_{"roll", "pitch", "throttle"} {}
    std::size_t size() const noexcept override { return 3; }
    const std::vector<std::string>& names() const noexcept override { return names_; }
    control::Level level() const noexcept override { return control::Level::Attitude; }
    control::Command map(const float* a) const override {
        control::AttitudeCommand c;
        c.rollRad = roll_(a[0]);
        c.pitchRad = pitch_(a[1]);
        c.throttle = unit(a[2]);
        return c;
    }
    void useRanges(const std::vector<control::CapabilityDescriptor>& capabilities) override {
        roll_ = aircraftRange(capabilities, "fsim.flight.attitude", "roll_rad", kRoll);
        pitch_ = aircraftRange(capabilities, "fsim.flight.attitude", "pitch_rad", kPitch);
    }

private:
    static constexpr Range kRoll{-60.0 * units::kDegreesToRadians, 60.0 * units::kDegreesToRadians};
    static constexpr Range kPitch{-25.0 * units::kDegreesToRadians, 25.0 * units::kDegreesToRadians};
    std::vector<std::string> names_;
    Range roll_ = kRoll, pitch_ = kPitch;
};

/// load factor (-1 .. 5 g), roll rate (+-3 rad/s), throttle. With the
/// aircraft's ranges, its n_min .. n_max and its roll rate limit.
class AccelerationAction final : public ActionMapper {
public:
    AccelerationAction() : names_{"load_factor", "roll_rate", "throttle"} {}
    std::size_t size() const noexcept override { return 3; }
    const std::vector<std::string>& names() const noexcept override { return names_; }
    control::Level level() const noexcept override { return control::Level::Acceleration; }
    control::Command map(const float* a) const override {
        control::AccelerationCommand c;
        c.loadFactorG = loadFactor_(a[0]);
        c.rollRateRadS = rollRate_(a[1]);
        c.throttle = unit(a[2]);
        return c;
    }
    void useRanges(const std::vector<control::CapabilityDescriptor>& capabilities) override {
        loadFactor_ = aircraftRange(capabilities, "fsim.flight.acceleration", "load_factor_g", kLoadFactor);
        rollRate_ = aircraftRange(capabilities, "fsim.flight.acceleration", "roll_rate_rad_s", kRollRate);
    }

private:
    static constexpr Range kLoadFactor{-1.0, 5.0}, kRollRate{-3.0, 3.0};
    std::vector<std::string> names_;
    Range loadFactor_ = kLoadFactor, rollRate_ = kRollRate;
};

/// airspeed (20 .. 120 m/s), vertical speed (+-10 m/s), turn rate (+-0.2 rad/s).
class VelocityAction final : public ActionMapper {
public:
    VelocityAction() : names_{"airspeed", "vertical_speed", "turn_rate"} {}
    std::size_t size() const noexcept override { return 3; }
    const std::vector<std::string>& names() const noexcept override { return names_; }
    control::Level level() const noexcept override { return control::Level::Velocity; }
    control::Command map(const float* a) const override {
        control::VelocityCommand c;
        c.airspeedMs = 20.0 + 100.0 * unit(a[0]);
        c.verticalSpeedMs = 10.0 * cl(a[1]);
        c.turnRateRadS = 0.2 * cl(a[2]);
        return c;
    }

private:
    std::vector<std::string> names_;
};

} // namespace

void registerBuiltinObservations(PluginRegistry& registry) {
    registry.addObservation("state", []() -> std::unique_ptr<ObservationBuilder> {
        return std::make_unique<StateObservation>();
    });
}

void registerBuiltinActions(PluginRegistry& registry) {
    registry.addAction("surfaces", []() -> std::unique_ptr<ActionMapper> { return std::make_unique<SurfacesAction>(); });
    registry.addAction("attitude", []() -> std::unique_ptr<ActionMapper> { return std::make_unique<AttitudeAction>(); });
    registry.addAction("acceleration",
                       []() -> std::unique_ptr<ActionMapper> { return std::make_unique<AccelerationAction>(); });
    registry.addAction("velocity", []() -> std::unique_ptr<ActionMapper> { return std::make_unique<VelocityAction>(); });
}

std::unique_ptr<ObservationBuilder> createObservationBuilder(const std::string& id) {
    auto builder = PluginRegistry::instance().createObservation(id);
    if (!builder) LOG_ERROR("env") << "unknown observation '" << id << "'";
    return builder;
}

std::unique_ptr<ActionMapper> createActionMapper(const std::string& id) {
    auto mapper = PluginRegistry::instance().createAction(id);
    if (!mapper) LOG_ERROR("env") << "unknown action mapper '" << id << "'";
    return mapper;
}

} // namespace fsim::env
