#pragma once

#include "env/Task.h"
#include "sim/ControlInputs.h"
#include "sim/VehicleState.h"

#include <cstddef>
#include <string>
#include <vector>

namespace fsim::env {

/// Builds the per-vehicle observation vector (design 9.1 "Observations").
/// Values are roughly normalised to [-1, 1] for training stability.
class ObservationBuilder {
public:
    virtual ~ObservationBuilder() = default;
    virtual std::size_t size() const noexcept = 0;
    /// Names of the observation elements, in order (for logging / debugging).
    virtual const std::vector<std::string>& names() const noexcept = 0;
    virtual void build(const sim::VehicleState& state, const TaskState& task, float* out) const = 0;
};

/// Maps a normalised action vector to JSBSim control commands (design 9.1
/// "Actions").
class ActionMapper {
public:
    virtual ~ActionMapper() = default;
    virtual std::size_t size() const noexcept = 0;
    virtual const std::vector<std::string>& names() const noexcept = 0;
    virtual void map(const float* action, sim::ControlInputs& out) const = 0;
};

std::unique_ptr<ObservationBuilder> createObservationBuilder(const std::string& id);
std::unique_ptr<ActionMapper> createActionMapper(const std::string& id);

} // namespace fsim::env
