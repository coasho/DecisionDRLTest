#pragma once

// Abstract vehicle simulation effects (design 9.5, ADR-21): disturbances and
// fidelity models attached to a vehicle (or to every vehicle) and applied
// once per FDM step before the flight model steps.

#include "fsim/EnvironmentState.h"
#include "fsim/Export.h"
#include "fsim/Property.h"
#include "fsim/Rng.h"
#include "fsim/VehicleState.h"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace fsim::sim {
class FlightModel;
}

namespace fsim::effects {

/// What the vehicle's sensors report (design 9.5 "Sensors"): the truth state
/// as modified by effects, plus sensor health flags.
struct SensedState {
    sim::VehicleState state;
    bool gnssValid = true;
    bool airDataValid = true;
    double positionErrorM = 0.0; ///< magnitude of the injected position error, for diagnostics
};

/// Channels an effect may touch. Wind and force/moment accumulate across the
/// effects of one step and are pushed to the flight model afterwards.
class FSIM_API EffectContext {
public:
    EffectContext(std::uint32_t id, const sim::VehicleState& truth, SensedState& sensedState, sim::FlightModel& model,
                  const sim::EnvironmentState& env, Rng& random, double stepDt, double time) noexcept
        : vehicleId(id), state(truth), sensed(sensedState), environment(env), rng(random), dt(stepDt), simTime(time), model_(model) {}

    const std::uint32_t vehicleId;
    const sim::VehicleState& state;          ///< truth before this step (read-only)
    SensedState& sensed;                     ///< what sensors will report (start of step: truth)
    const sim::EnvironmentState& environment;
    Rng& rng;                                ///< this vehicle's deterministic stream
    const double dt;
    const double simTime;

    /// Add to the wind at this vehicle (NED m/s, on top of the global wind).
    void addWindNed(double north, double east, double down) noexcept {
        windNed_[0] += north; windNed_[1] += east; windNed_[2] += down; windTouched_ = true;
    }
    /// Replace the wind at this vehicle entirely.
    void setWindNed(double north, double east, double down) noexcept {
        windNed_[0] = north; windNed_[1] = east; windNed_[2] = down; windTouched_ = true; windOverride_ = true;
    }
    /// Add a body-frame force (N) and moment (N m) at the centre of gravity.
    void addForceBody(double x, double y, double z) noexcept { force_[0] += x; force_[1] += y; force_[2] += z; forceTouched_ = true; }
    void addMomentBody(double l, double m, double n) noexcept { moment_[0] += l; moment_[1] += m; moment_[2] += n; forceTouched_ = true; }
    /// Any flight-model property (mass, fuel, engine health, ...).
    sim::PropertyHandle property(std::string_view path);

    // Read back by the session after every effect ran.
    const double* windNed() const noexcept { return windNed_; }
    bool windTouched() const noexcept { return windTouched_; }
    bool windOverride() const noexcept { return windOverride_; }
    const double* force() const noexcept { return force_; }
    const double* moment() const noexcept { return moment_; }
    bool forceTouched() const noexcept { return forceTouched_; }

private:
    sim::FlightModel& model_;
    double windNed_[3] = {0, 0, 0};
    double force_[3] = {0, 0, 0};
    double moment_[3] = {0, 0, 0};
    bool windTouched_ = false, windOverride_ = false, forceTouched_ = false;
};

/// The extension point. Effects live on the vehicle and survive resets.
class Effect {
public:
    virtual ~Effect() = default;
    virtual const char* id() const noexcept = 0;
    virtual void apply(EffectContext& ctx) = 0;
    virtual void onReset() {}
    bool enabled = true;
};

} // namespace fsim::effects
