#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace fsim::platform {

/// Blocking HTTP(S) GET of a small resource (tiles). Windows: WinHTTP (no
/// extra runtime). Returns false with `error` set on any failure, including
/// non-2xx status codes. `timeoutMs` bounds connect + receive.
bool httpGet(const std::string& url, std::vector<std::uint8_t>& body, std::string* error = nullptr, unsigned timeoutMs = 15000);

} // namespace fsim::platform
