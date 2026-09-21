#include "env/Task.h"

#include "core/Log.h"
#include "core/Units.h"
#include "env/Registry.h"
#include "env/Scenario.h"

#include <algorithm>
#include <cmath>

namespace fsim::env {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapPi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

/// Hold a target altitude and heading. Dense shaped reward in [about -1, 1]:
/// Gaussian-like terms for altitude and heading error, small penalties for
/// bank, sideslip and pitch rate; terminate on crash-like states.
class AltitudeHeadingHold final : public Task {
public:
    explicit AltitudeHeadingHold(const TaskParams& p)
        : altDelta_(p.targetAltitudeDeltaM), hdgDelta_(units::degreesToRadians(p.targetHeadingDeltaDeg)) {}

    std::string_view name() const noexcept override { return "altitude_heading_hold"; }

    void reset(std::size_t, const sim::VehicleState& initial, Rng& rng, TaskState& t) override {
        t.targetAltitudeM = std::max(initial.altitudeMslM + rng.uniform(-altDelta_, altDelta_), initial.altitudeMslM - initial.altitudeAglM + 300.0);
        t.targetHeadingRad = wrapPi(initial.eulerRad[2] + rng.uniform(-hdgDelta_, hdgDelta_));
        t.targetAirspeedMs = initial.airspeedTrueMs;
        t.lastReward = 0.0;
    }

    void evaluate(std::size_t, const sim::VehicleState& s, TaskState& t, double& reward, bool& terminated) override {
        const double altErr = t.targetAltitudeM - s.altitudeMslM;
        const double hdgErr = wrapPi(t.targetHeadingRad - s.eulerRad[2]);
        const double roll = s.eulerRad[0], pitch = s.eulerRad[1];

        const double altTerm = std::exp(-std::abs(altErr) / 100.0);          // 1 at target, 0.37 at 100 m
        const double hdgTerm = std::exp(-std::abs(hdgErr) / (10.0 * units::kDegreesToRadians)); // 0.37 at 10 deg
        const double smooth = 0.05 * std::abs(roll) + 0.02 * std::abs(s.angularRateBodyRadS[1]) + 0.05 * std::abs(s.betaRad);
        reward = 0.6 * altTerm + 0.4 * hdgTerm - std::min(smooth, 0.5);

        terminated = s.diverged || s.altitudeAglM < 30.0 || std::abs(roll) > 80.0 * units::kDegreesToRadians ||
                     std::abs(pitch) > 60.0 * units::kDegreesToRadians || std::abs(altErr) > 1500.0 ||
                     s.airspeedTrueMs < 15.0;
        if (terminated && !s.diverged) reward -= 10.0; // crash / loss of control
        t.lastReward = reward;
    }

private:
    double altDelta_, hdgDelta_;
};

/// Reward only survival and level flight; useful as a sanity baseline.
class LevelFlight final : public Task {
public:
    std::string_view name() const noexcept override { return "level_flight"; }
    void reset(std::size_t, const sim::VehicleState& initial, Rng&, TaskState& t) override {
        t.targetAltitudeM = initial.altitudeMslM;
        t.targetHeadingRad = initial.eulerRad[2];
        t.targetAirspeedMs = initial.airspeedTrueMs;
    }
    void evaluate(std::size_t, const sim::VehicleState& s, TaskState& t, double& reward, bool& terminated) override {
        reward = 1.0 - 0.5 * std::abs(s.eulerRad[0]) - 0.5 * std::abs(s.eulerRad[1]) - 0.001 * std::abs(t.targetAltitudeM - s.altitudeMslM);
        terminated = s.diverged || s.altitudeAglM < 30.0 || std::abs(s.eulerRad[0]) > 80.0 * units::kDegreesToRadians;
        if (terminated && !s.diverged) reward -= 10.0;
        t.lastReward = reward;
    }
};

} // namespace

void registerBuiltinTasks(PluginRegistry& registry) {
    registry.addTask("altitude_heading_hold", [](const TaskParams& p) -> std::unique_ptr<Task> {
        return std::make_unique<AltitudeHeadingHold>(p);
    });
    registry.addTask("level_flight", [](const TaskParams&) -> std::unique_ptr<Task> {
        return std::make_unique<LevelFlight>();
    });
}

TaskParams taskParams(const Scenario& s) {
    TaskParams p;
    p.targetAltitudeDeltaM = s.targetAltitudeDeltaM;
    p.targetHeadingDeltaDeg = s.targetHeadingDeltaDeg;
    p.maxEpisodeSteps = s.maxEpisodeSteps;
    p.aircraft = s.aircraft;
    p.agentStepSeconds = s.dt * s.frameSkip;
    return p;
}

std::unique_ptr<Task> createTask(const std::string& id, const Scenario& scenario) {
    auto task = PluginRegistry::instance().createTask(id, taskParams(scenario));
    if (!task) LOG_ERROR("env") << "unknown task '" << id << "'";
    return task;
}

} // namespace fsim::env
