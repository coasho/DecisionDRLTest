#pragma once

// Scenario files on the session world (design 9.13). The parser, dumper and
// this apply are shared by the C++ SDK (fsim::applyScenario) and the C ABI.

#include "fsim/Scenario.h"

#include <cstdint>
#include <vector>

namespace fsim::session {

class World;

/// See fsim::applyScenario; returns the created vehicle ids.
std::vector<std::uint32_t> applyScenario(World& world, const Scenario& scenario);

/// Only the environment and the world-wide effects (what a VecEnv takes from a scenario).
void applyScenarioWorld(World& world, const Scenario& scenario);

} // namespace fsim::session
