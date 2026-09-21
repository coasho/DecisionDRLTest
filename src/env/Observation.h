#pragma once

// Observation builders and action mappers are public SDK (design 10.1 "SDK
// registration"), like tasks: the built-ins below are registered against the
// same interfaces trainer code implements.

#include "env/Task.h"
#include "fsim/Control.h"
#include "fsim/VecEnvPlugins.h"
#include "fsim/VehicleState.h"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace fsim::env {

using fsim::ActionMapper;
using fsim::ObservationBuilder;

/// Create by id from the registry (built-ins and anything the trainer
/// registered); null and a logged error if the id is unknown.
std::unique_ptr<ObservationBuilder> createObservationBuilder(const std::string& id);
std::unique_ptr<ActionMapper> createActionMapper(const std::string& id);

} // namespace fsim::env
