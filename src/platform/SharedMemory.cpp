#include "platform/SharedMemory.h"

#include <utility>

#ifdef _WIN32
#    include <windows.h>
#else
#    include <fcntl.h>
#    include <sys/mman.h>
#    include <unistd.h>
#endif

namespace fsim::platform {

SharedMemory::~SharedMemory() { close(); }

SharedMemory::SharedMemory(SharedMemory&& other) noexcept
    : handle_(std::exchange(other.handle_, nullptr)), data_(std::exchange(other.data_, nullptr)),
      size_(std::exchange(other.size_, 0)), created_(std::exchange(other.created_, false)), name_(std::move(other.name_)) {}

SharedMemory& SharedMemory::operator=(SharedMemory&& other) noexcept {
    if (this != &other) {
        close();
        handle_ = std::exchange(other.handle_, nullptr);
        data_ = std::exchange(other.data_, nullptr);
        size_ = std::exchange(other.size_, 0);
        created_ = std::exchange(other.created_, false);
        name_ = std::move(other.name_);
    }
    return *this;
}

#ifdef _WIN32

namespace {
std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return w;
}
} // namespace

bool SharedMemory::create(const std::string& name, std::size_t size) noexcept {
    close();
    const std::wstring wname = widen("Local\\" + name);
    HANDLE h = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, static_cast<DWORD>(static_cast<std::uint64_t>(size) >> 32),
                                  static_cast<DWORD>(size & 0xFFFFFFFFu), wname.c_str());
    if (!h) return false;
    created_ = GetLastError() != ERROR_ALREADY_EXISTS;
    void* p = MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (!p) {
        CloseHandle(h);
        return false;
    }
    handle_ = h;
    data_ = p;
    size_ = size;
    name_ = name;
    return true;
}

bool SharedMemory::open(const std::string& name, std::size_t size) noexcept {
    close();
    const std::wstring wname = widen("Local\\" + name);
    HANDLE h = OpenFileMappingW(FILE_MAP_ALL_ACCESS, FALSE, wname.c_str());
    if (!h) return false;
    void* p = MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (!p) {
        CloseHandle(h);
        return false;
    }
    handle_ = h;
    data_ = p;
    size_ = size;
    created_ = false;
    name_ = name;
    return true;
}

void SharedMemory::close() noexcept {
    if (data_) UnmapViewOfFile(data_);
    if (handle_) CloseHandle(static_cast<HANDLE>(handle_));
    data_ = nullptr;
    handle_ = nullptr;
    size_ = 0;
    created_ = false;
}

std::uint64_t currentProcessId() noexcept { return GetCurrentProcessId(); }

bool processAlive(std::uint64_t pid) noexcept {
    if (pid == 0) return false;
    HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
    if (!h) return GetLastError() == ERROR_ACCESS_DENIED; // exists, but not ours to inspect
    DWORD code = 0;
    const bool alive = GetExitCodeProcess(h, &code) && code == STILL_ACTIVE;
    CloseHandle(h);
    return alive;
}

#else

bool SharedMemory::create(const std::string& name, std::size_t size) noexcept {
    close();
    const std::string path = "/" + name;
    int fd = shm_open(path.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
    created_ = fd >= 0;
    if (fd < 0) fd = shm_open(path.c_str(), O_RDWR, 0600);
    if (fd < 0) return false;
    if (created_ && ftruncate(fd, static_cast<off_t>(size)) != 0) { ::close(fd); return false; }
    void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) return false;
    data_ = p; size_ = size; name_ = name;
    return true;
}

bool SharedMemory::open(const std::string& name, std::size_t size) noexcept {
    close();
    const std::string path = "/" + name;
    int fd = shm_open(path.c_str(), O_RDWR, 0600);
    if (fd < 0) return false;
    void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) return false;
    data_ = p; size_ = size; name_ = name; created_ = false;
    return true;
}

void SharedMemory::close() noexcept {
    if (data_) munmap(data_, size_);
    data_ = nullptr; size_ = 0; created_ = false;
}

std::uint64_t currentProcessId() noexcept { return static_cast<std::uint64_t>(getpid()); }
bool processAlive(std::uint64_t pid) noexcept { return pid != 0 && kill(static_cast<pid_t>(pid), 0) == 0; }

#endif

} // namespace fsim::platform
