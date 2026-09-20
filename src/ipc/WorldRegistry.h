#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace fsim::ipc {

/// A live world as seen in the discovery registry (design 9.7).
struct WorldInfo {
    std::string name;
    std::uint64_t pid = 0;
    std::int64_t createdWallNs = 0;
    bool alive = false; ///< publisher process still running
};

/// Registry of published worlds: a tiny shared mapping every publisher adds
/// itself to and removes itself from. Entries of dead processes are pruned by
/// whoever touches the registry next.
class WorldRegistry {
public:
    /// Add `name` for this process. Returns false if the registry is full or unavailable.
    static bool add(const std::string& name) noexcept;
    static void remove(const std::string& name) noexcept;

    /// Live worlds, newest first. Empty when nothing is published.
    static std::vector<WorldInfo> list() noexcept;
};

} // namespace fsim::ipc
