#pragma once

#include <filesystem>

namespace fsim::platform {

/// Directory containing the running executable.
std::filesystem::path executableDir();

/// Per-user configuration directory (%LOCALAPPDATA%\flightsim on Windows).
/// Created on first use.
std::filesystem::path configDir();

} // namespace fsim::platform
