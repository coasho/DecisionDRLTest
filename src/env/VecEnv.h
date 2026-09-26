#pragma once

#include "core/Rng.h"
#include "core/Span.h"
#include "env/Observation.h"
#include "env/Scenario.h"
#include "env/Task.h"
#include "session/World.h"
#include "sim/GroundProvider.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

namespace fsim::env {

/// Vectorised environment (design 4.4, 6.1, 9.1): M environments x K vehicles
/// stepped in lockstep on the caller's thread + the vehicle pool's workers.
/// All buffers are preallocated and owned here; callers receive spans.
///
/// Layout (vehicle-major): index = env * K + vehicle.
///   observations: [M*K][O] float32     rewards: [M*K]      terminated/truncated: [M*K] (uint8)
///   actions:      [M*K][A] float32 (caller supplies)
///
/// Auto-reset, in one of two conventions (AutoReset). Either way the final
/// observation of an episode that just ended is kept in `finalObservations()`
/// and its flags are set in the batch returned by the step that ended it.
class VecEnv {
public:
    /// When an environment whose episode ended starts the next one.
    enum class AutoReset {
        /// On the following step, whose action it ignores, returning the new
        /// episode's first observation with zero reward (Gymnasium's default).
        NextStep,
        /// At once: the step that ended the episode returns the next one's
        /// first observation (Stable-Baselines3, Gymnasium's SAME_STEP).
        SameStep,
    };

    struct Options {
        unsigned numEnvs = 1;
        std::uint64_t seed = 0;
        unsigned workers = 0;              ///< 0 = physical cores - 2
        std::shared_ptr<sim::GroundProvider> ground; ///< null = flat at 0 m
        bool publish = true;               ///< visible to flightsim-viewer.exe
        bool terrain = false;              ///< physics ground from public elevation tiles
        std::filesystem::path scenarioPath; ///< optional: its environment and world-wide effects are applied to the world
        AutoReset autoReset = AutoReset::NextStep;
    };

    struct StepResult {
        Span<const float> observations;    ///< M*K*O
        Span<const float> rewards;         ///< M*K
        Span<const std::uint8_t> terminated; ///< M*K
        Span<const std::uint8_t> truncated;  ///< M*K
        Span<const std::uint32_t> episodeSteps; ///< M (steps in the current episode)
    };

    VecEnv(const Scenario& scenario, const Options& options);
    ~VecEnv();
    VecEnv(const VecEnv&) = delete;
    VecEnv& operator=(const VecEnv&) = delete;

    /// Reset every environment (new seeds derived from `seed`, or the
    /// constructor seed when 0). Returns the first observations.
    StepResult reset(std::uint64_t seed = 0);

    /// Advance every vehicle by `frameSkip` FDM steps with `actions` (M*K*A).
    StepResult step(Span<const float> actions);

    // Sizes
    unsigned numEnvs() const noexcept { return numEnvs_; }
    unsigned vehiclesPerEnv() const noexcept { return vehiclesPerEnv_; }
    std::size_t numVehicles() const noexcept { return static_cast<std::size_t>(numEnvs_) * vehiclesPerEnv_; }
    std::size_t observationSize() const noexcept { return obsBuilder_->size(); }
    std::size_t actionSize() const noexcept { return actionMapper_->size(); }
    /// "fixed" (the action's own ranges) or "aircraft" (the aircraft's where
    /// its profile narrows them); false for another mode.
    bool setActionRanges(std::string_view mode);
    const std::vector<std::string>& observationNames() const noexcept { return obsBuilder_->names(); }
    const std::vector<std::string>& actionNames() const noexcept { return actionMapper_->names(); }
    double agentStepSeconds() const noexcept { return scenario_.dt * scenario_.frameSkip; }

    /// Last observation of environments that auto-reset in the previous step (M*K*O).
    Span<const float> finalObservations() const noexcept { return Span<const float>(finalObs_.data(), finalObs_.size()); }

    /// Raw vehicle state of vehicle `i` (env * K + vehicle) after the last step.
    const sim::VehicleState& state(std::size_t i) const noexcept { return *world_->vehicleState(ids_[i]); }
    /// The world behind the environments (vehicles can be inspected or commanded directly).
    session::World& world() noexcept { return *world_; }
    const TaskState& taskState(std::size_t vehicle) const noexcept { return taskStates_[vehicle]; }
    const Scenario& scenario() const noexcept { return scenario_; }

    /// Total FDM vehicle-steps executed so far.
    std::uint64_t vehicleSteps() const noexcept { return vehicleSteps_; }

    AutoReset autoReset() const noexcept { return autoReset_; }
    /// Takes effect from the next step.
    void setAutoReset(AutoReset mode) noexcept { autoReset_ = mode; }
    /// World vehicle id of batch index i (env * K + vehicle).
    std::uint32_t vehicleId(std::size_t i) const noexcept { return ids_[i]; }

private:
    void resetEnv(unsigned env, bool initialLoad);
    void buildObservations();
    StepResult result() const noexcept;

    Scenario scenario_;
    unsigned numEnvs_;
    unsigned vehiclesPerEnv_;
    std::uint64_t seed_;
    std::uint64_t episodeCounter_ = 0;

    std::unique_ptr<session::World> world_;
    std::vector<std::uint32_t> ids_;                          ///< vehicle id per index
    std::unique_ptr<Task> task_;
    std::unique_ptr<ObservationBuilder> obsBuilder_;
    std::unique_ptr<ActionMapper> actionMapper_;

    std::vector<TaskState> taskStates_;
    std::vector<sim::InitialConditions> initialConditions_; ///< per vehicle, current episode
    std::vector<float> observations_, finalObs_, rewards_;
    std::vector<std::uint8_t> terminated_, truncated_;
    std::vector<std::uint32_t> episodeSteps_;
    std::vector<std::uint8_t> needsReset_; ///< per env
    std::uint64_t vehicleSteps_ = 0;
    AutoReset autoReset_ = AutoReset::NextStep;
};

} // namespace fsim::env
