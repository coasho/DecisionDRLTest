// fsim SDK: scenario application on the public World.

#include "fsim/Scenario.h"

#include "session/Scenario.h"

namespace fsim {

std::vector<Vehicle> applyScenario(World& world, const Scenario& scenario) {
    std::vector<Vehicle> out;
    for (auto id : session::applyScenario(world.impl(), scenario)) out.push_back(world.vehicle(id));
    return out;
}

} // namespace fsim
