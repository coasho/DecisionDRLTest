#pragma once

// The per-thread error string behind fsim_last_error(), shared by every C ABI
// translation unit.

#include <string>

namespace fsim::sdk {

std::string& lastError() noexcept;

} // namespace fsim::sdk
