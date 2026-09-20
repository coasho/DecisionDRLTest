#pragma once

#include "core/Rng.h"
#include "sim/VehicleState.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace fsim::env {

struct Scenario;

/// Per-vehicle values a task keeps across an episode (targets etc.). Exposed
/// to observation builders so agents can see what they are asked to do.
struct TaskState {
    double targetAltitudeM = 0.0;
    double targetHeadingRad = 0.0;
    double targetAirspeedMs = 0.0;
    double lastReward = 0.0;
};

/// Reward / termination logic (design 9.1 "Tasks"). One instance per
/// environment, evaluated on the caller's thread over all its vehicles.
class Task {
public:
    virtual ~Task() = default;
    virtual std::string_view name() const noexcept = 0;

    /// Start of an episode: choose targets from the initial state.
    virtual void reset(std::size_t vehicle, const sim::VehicleState& initial, Rng& rng, TaskState& task) = 0;

    /// After a step: reward and termination for one vehicle.
    virtual void evaluate(std::size_t vehicle, const sim::VehicleState& state, TaskState& task, double& reward,
                          bool& terminated) = 0;
};

/// Built-in tasks, created by id (design 10.1 "Registries").
std::unique_ptr<Task> createTask(const std::string& id, const Scenario& scenario);

} // namespace fsim::env
