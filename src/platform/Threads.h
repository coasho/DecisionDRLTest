#pragma once

#include <string_view>

namespace fsim::platform {

/// Number of physical cores (hyper-threads excluded) or, if that cannot be
/// determined, the number of logical processors. Never returns 0.
unsigned physicalCoreCount() noexcept;

/// Number of logical processors. Never returns 0.
unsigned logicalCoreCount() noexcept;

/// Name the calling thread for debuggers and profilers. Best effort.
void setCurrentThreadName(std::string_view name) noexcept;

/// Pin the calling thread to one logical processor. Best effort; returns
/// false if the OS refused. Used for simulation workers (design 12.2).
bool pinCurrentThreadToCore(unsigned logicalCore) noexcept;

} // namespace fsim::platform
