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

/// First logical processor of physical core `physicalCore` (so that threads
/// pinned to different physical cores never share SMT siblings). Falls back
/// to `physicalCore` itself when the topology is unknown.
unsigned logicalProcessorOfCore(unsigned physicalCore) noexcept;

/// Declare the process per-monitor-DPI aware so window client sizes are in
/// physical pixels and match the Vulkan swapchain. Must be called before any
/// window is created; best effort on older Windows. Returns false if refused.
bool enableHighDpiAwareness() noexcept;

/// Ask the OS for 1 ms scheduler/timer granularity for this process (Windows
/// timeBeginPeriod). Idempotent; released at process exit.
void requestHighResolutionTimer() noexcept;

/// Display scale factor of the primary monitor (1.0 = 96 dpi, 1.5 = 150 %).
double systemDpiScale() noexcept;

} // namespace fsim::platform
