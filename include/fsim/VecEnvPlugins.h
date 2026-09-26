#pragma once

// fsim SDK: extending VecEnv from trainer code (design 10.1 "SDK
// registration"). A task (reward and termination), an observation builder
// (the vector the agent sees) and an action mapper (what an action vector
// commands) are small interfaces; implement one, register it under an id,
// and name that id in VecEnvOptions - no platform rebuild.
//
//   class MyTask : public fsim::Task { ... };
//   fsim::registerTask("my_task", [](const fsim::TaskParams& p) { return std::make_unique<MyTask>(p); });
//   opt.task = "my_task";
//
// The built-ins ("altitude_heading_hold", "level_flight"; "state"; "surfaces",
// "attitude", "acceleration", "velocity") are implemented against the same
// interfaces. Registration is process-wide and takes effect for environments
// created afterwards; registering an existing id replaces it.

#include "fsim/Capability.h"
#include "fsim/Control.h"
#include "fsim/Export.h"
#include "fsim/Rng.h"
#include "fsim/VehicleState.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace fsim {

/// Per-vehicle values a task keeps across an episode (targets etc.), visible
/// to observation builders so agents can see what they are asked to do. Reset
/// to this default before every Task::reset(), so episodes are independent.
struct TaskState {
    double targetAltitudeM = 0.0;
    double targetHeadingRad = 0.0;
    double targetAirspeedMs = 0.0;
    double lastReward = 0.0;
    double custom[8] = {0, 0, 0, 0, 0, 0, 0, 0}; ///< free for user tasks and observations
};

/// Reward and termination. One instance per environment; evaluated on the
/// caller's thread over that environment's vehicles.
class Task {
public:
    virtual ~Task() = default;
    virtual std::string_view name() const noexcept = 0;
    /// Start of an episode: choose targets from the vehicle's initial state.
    virtual void reset(std::size_t vehicle, const VehicleState& initial, Rng& rng, TaskState& task) = 0;
    /// After a step: reward and termination for one vehicle.
    virtual void evaluate(std::size_t vehicle, const VehicleState& state, TaskState& task, double& reward, bool& terminated) = 0;
};

/// What the environment hands a task factory (the scenario's task parameters).
struct TaskParams {
    double targetAltitudeDeltaM = 300.0;
    double targetHeadingDeltaDeg = 60.0;
    unsigned maxEpisodeSteps = 2000;
    std::string aircraft;
    double agentStepSeconds = 0.0;
};

/// The per-vehicle observation vector. Values should be roughly normalised
/// to [-1, 1] for training stability.
class ObservationBuilder {
public:
    virtual ~ObservationBuilder() = default;
    virtual std::size_t size() const noexcept = 0;
    /// Names of the elements, in order (VecEnv::observationNames()).
    virtual const std::vector<std::string>& names() const noexcept = 0;
    virtual void build(const VehicleState& state, const TaskState& task, float* out) const = 0;
};

/// Maps an action vector (each element in [-1, 1]) to a command at one level
/// of the control stack; the built-in loops fly anything above the actuators.
class ActionMapper {
public:
    virtual ~ActionMapper() = default;
    virtual std::size_t size() const noexcept = 0;
    virtual const std::vector<std::string>& names() const noexcept = 0;
    virtual control::Level level() const noexcept = 0;
    virtual control::Command map(const float* action) const = 0;
    /// The aircraft's capabilities, with their parameters' ranges for this
    /// aircraft (VecEnv action_ranges "aircraft"); empty to go back to the
    /// mapper's own ("fixed"). The built-in mappers then map onto a range the
    /// aircraft's profile narrowed - a fighter's load factor to its n_min ..
    /// n_max - and keep their own elsewhere. A mapper may ignore it.
    virtual void useRanges(const std::vector<control::CapabilityDescriptor>& capabilities) { (void)capabilities; }
};

using TaskFactory = std::function<std::unique_ptr<Task>(const TaskParams&)>;
using ObservationFactory = std::function<std::unique_ptr<ObservationBuilder>()>;
using ActionFactory = std::function<std::unique_ptr<ActionMapper>()>;

FSIM_API void registerTask(const std::string& id, TaskFactory factory);
FSIM_API void registerObservation(const std::string& id, ObservationFactory factory);
FSIM_API void registerAction(const std::string& id, ActionFactory factory);

/// Registered ids (built-ins included), for listings and checks.
FSIM_API std::vector<std::string> taskIds();
FSIM_API std::vector<std::string> observationIds();
FSIM_API std::vector<std::string> actionIds();

} // namespace fsim
