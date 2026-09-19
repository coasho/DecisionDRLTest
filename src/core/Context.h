#pragma once

#include <cstdint>
#include <filesystem>

namespace fsim {

/// Services handed to every module's lifecycle hooks (design 10.2). Grows as
/// the platform does (event bus, property store, job system, viewer); today it
/// carries what the headless milestones need.
struct Context {
    std::filesystem::path jsbsimRoot; ///< resolved JSBSim data tree
    std::uint64_t seed = 0;           ///< run seed for deterministic streams
    bool headless = true;             ///< no render/world/ui modules present
};

} // namespace fsim
