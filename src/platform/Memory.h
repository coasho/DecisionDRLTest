#pragma once

#include <cstddef>

namespace fsim::platform {

/// Reserve `bytes` of address space without backing it with memory, so that
/// something can grow into it later without ever moving. Null on failure.
void* reserveAddressSpace(std::size_t bytes) noexcept;

/// Back [address, address + bytes) of a reservation with zeroed memory.
bool commitMemory(void* address, std::size_t bytes) noexcept;

/// Give back a whole reservation made by reserveAddressSpace.
void releaseAddressSpace(void* address, std::size_t bytes) noexcept;

/// Granularity of commitMemory, in bytes.
std::size_t pageSize() noexcept;

} // namespace fsim::platform
