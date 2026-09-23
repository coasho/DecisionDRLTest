#pragma once

#include <filesystem>

namespace fsim::platform {

/// Directory containing the running executable.
std::filesystem::path executableDir();

/// Directory containing the module this code is linked into: the executable
/// for the platform's own programs, but libfsim.dll's directory when the SDK
/// is loaded by a host that is not ours (Python, Rust, C#), whose
/// executableDir() says nothing about where the platform's files are.
std::filesystem::path moduleDir();

/// Per-user configuration directory (%LOCALAPPDATA%\flightsim on Windows).
/// Created on first use.
std::filesystem::path configDir();

} // namespace fsim::platform
