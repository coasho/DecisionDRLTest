#include "env/Observation.h"

#include "core/Log.h"

#include <algorithm>
#include <cmath>
#include <memory>

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
class SurfacesAction final : public ActionMapper {
public:
    SurfacesAction() : names_{"aileron", "elevator", "rudder", "throttle"} {}
    std::size_t size() const noexcept override { return 4; }
    const std::vector<std::string>& names() const noexcept override { return names_; }
    void map(const float* a, sim::ControlInputs& c) const override {
        auto cl = [](float v) { return static_cast<double>(std::clamp(v, -1.0f, 1.0f)); };
        c.aileron = cl(a[0]);
        c.elevator = cl(a[1]);
        c.rudder = cl(a[2]);
        c.setThrottleAll((cl(a[3]) + 1.0) * 0.5);
        c.gearDown = 0.0;
        c.flaps = 0.0;
        c.brakeLeft = c.brakeRight = 0.0;
    }

private:
    std::vector<std::string> names_;
};

} // namespace

std::unique_ptr<ObservationBuilder> createObservationBuilder(const std::string& id) {
    if (id == "state") return std::make_unique<StateObservation>();
    LOG_ERROR("env") << "unknown observation '" << id << "'";
    return nullptr;
}

std::unique_ptr<ActionMapper> createActionMapper(const std::string& id) {
    if (id == "surfaces") return std::make_unique<SurfacesAction>();
    LOG_ERROR("env") << "unknown action mapper '" << id << "'";
    return nullptr;
}

} // namespace fsim::env
