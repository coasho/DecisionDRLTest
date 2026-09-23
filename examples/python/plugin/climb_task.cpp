// A task the platform does not ship, written in C++ and trained from Python.
//
// Build it with the platform (it is built with the examples) or against the
// SDK package with the same toolchain; load it into Python and name it:
//
//   fsim.load_plugin("climb_task.dll")
//   env = fsim.VecEnv(64, task="climb", action="attitude")
//
// Loading the DLL runs its static initialisers, and the one at the bottom of
// this file registers the task with the libfsim.dll the Python SDK already
// has loaded - the same registry the built-in tasks are in. Observations,
// actions and controllers register the same way (VecEnvPlugins.h,
// ControllerRegistry.h).
//
// The task: climb to a target 0.5 to 2 times the scenario's
// target_altitude_delta_m above where the episode starts, and stay there.
// The reward is 1 at the target and falls to 0 at 300 m off; losing control
// ends the episode with -10, as it does in the built-in tasks. The "state"
// observation shows the agent the target (alt_err_km), so no observation
// builder is needed.

#include <fsim/VecEnvPlugins.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

class Climb final : public fsim::Task {
public:
    explicit Climb(const fsim::TaskParams& p) : delta_(p.targetAltitudeDeltaM) {}

    std::string_view name() const noexcept override { return "climb"; }

    void reset(std::size_t, const fsim::VehicleState& initial, fsim::Rng& rng, fsim::TaskState& t) override {
        t.targetAltitudeM = initial.altitudeMslM + rng.uniform(0.5 * delta_, 2.0 * delta_);
        t.targetHeadingRad = initial.eulerRad[2]; // not rewarded; keeps hdg_err_* meaningful
        t.targetAirspeedMs = initial.airspeedTrueMs;
    }

    void evaluate(std::size_t, const fsim::VehicleState& s, fsim::TaskState& t, double& reward, bool& terminated) override {
        reward = 1.0 - std::min(std::abs(t.targetAltitudeM - s.altitudeMslM) / 300.0, 1.0);
        terminated = s.diverged || s.altitudeAglM < 30.0 || std::abs(s.eulerRad[0]) > 80.0 * kDegToRad ||
                     std::abs(s.eulerRad[1]) > 60.0 * kDegToRad || s.airspeedTrueMs < 15.0;
        if (terminated && !s.diverged) reward -= 10.0; // crash / loss of control
        t.lastReward = reward;
    }

private:
    double delta_;
};

[[maybe_unused]] const bool registered = [] {
    fsim::registerTask("climb", [](const fsim::TaskParams& p) { return std::make_unique<Climb>(p); });
    return true;
}();

} // namespace
