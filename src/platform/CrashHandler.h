#pragma once

// Crash handler (design 15, M5): on an unhandled Win32 exception write a
// minidump to <config>/crash/<exe>-<time>.dmp and a line to stderr, then
// let the process die. Executables install it; libraries never do (a trainer
// owns its own process).

#include <filesystem>

namespace fsim::platform {

/// Install the handler; `dumpDir` empty = <configDir>/crash. Idempotent.
void installCrashHandler(const std::filesystem::path& dumpDir = {});

/// Where the next dump would be written (for messages and tests).
std::filesystem::path crashDumpDir();

} // namespace fsim::platform
