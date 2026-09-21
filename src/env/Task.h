#pragma once

// The task interface is public SDK (design 10.1 "SDK registration"): trainer
// code implements fsim::Task and registers it by id. Internally the platform
// uses the same types, so a built-in task and a trainer's task are the same
// kind of object.

#include "core/Rng.h"
#include "fsim/VecEnvPlugins.h"
#include "sim/VehicleState.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace fsim::env {

struct Scenario;

using fsim::Task;
using fsim::TaskParams;
using fsim::TaskState;

/// Task parameters the scenario fixes for the whole batch.
TaskParams taskParams(const Scenario& scenario);

/// Create by id from the registry (built-ins and anything the trainer
/// registered); null and a logged error if the id is unknown.
std::unique_ptr<Task> createTask(const std::string& id, const Scenario& scenario);

} // namespace fsim::env
