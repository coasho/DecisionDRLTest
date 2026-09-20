#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace fsim::platform {

/// Named, page-file-backed shared memory (design 9.7). The mapping lives as
/// long as any process holds it open; a creator that exits leaves it to the
/// readers, and it vanishes with the last handle. Zero-filled on creation.
class SharedMemory {
public:
    SharedMemory() = default;
    ~SharedMemory();
    SharedMemory(SharedMemory&& other) noexcept;
    SharedMemory& operator=(SharedMemory&& other) noexcept;
    SharedMemory(const SharedMemory&) = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;

    /// Create (or open, if it already exists) `name` with at least `size` bytes.
    /// `created()` tells which happened. Returns false on failure.
    bool create(const std::string& name, std::size_t size) noexcept;

    /// Open an existing mapping read/write. Returns false if it does not exist.
    bool open(const std::string& name, std::size_t size) noexcept;

    void close() noexcept;

    bool valid() const noexcept { return data_ != nullptr; }
    bool created() const noexcept { return created_; }
    void* data() noexcept { return data_; }
    const void* data() const noexcept { return data_; }
    std::size_t size() const noexcept { return size_; }
    const std::string& name() const noexcept { return name_; }

private:
    void* handle_ = nullptr;
    void* data_ = nullptr;
    std::size_t size_ = 0;
    bool created_ = false;
    std::string name_;
};

/// This process's id.
std::uint64_t currentProcessId() noexcept;

/// True if a process with that id is still running (best effort).
bool processAlive(std::uint64_t pid) noexcept;

} // namespace fsim::platform
