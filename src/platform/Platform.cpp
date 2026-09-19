#include "platform/Clock.h"
#include "platform/Paths.h"
#include "platform/Threads.h"

#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#    include <windows.h>
#    include <shlobj.h>
#endif

namespace fsim::platform {

unsigned logicalCoreCount() noexcept {
    const unsigned n = std::thread::hardware_concurrency();
    return n == 0 ? 1u : n;
}

unsigned physicalCoreCount() noexcept {
#ifdef _WIN32
    DWORD length = 0;
    GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &length);
    if (length == 0) return logicalCoreCount();

    std::vector<unsigned char> buffer(length);
    auto* info = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buffer.data());
    if (!GetLogicalProcessorInformationEx(RelationProcessorCore, info, &length)) return logicalCoreCount();

    unsigned cores = 0;
    for (DWORD offset = 0; offset < length;) {
        auto* entry = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buffer.data() + offset);
        if (entry->Relationship == RelationProcessorCore) ++cores;
        offset += entry->Size;
    }
    return cores == 0 ? logicalCoreCount() : cores;
#else
    return logicalCoreCount();
#endif
}

void setCurrentThreadName(std::string_view name) noexcept {
#ifdef _WIN32
    // SetThreadDescription is available from Windows 10 1607; load dynamically so
    // the binary still starts on older kernels.
    using SetThreadDescriptionFn = HRESULT(WINAPI*)(HANDLE, PCWSTR);
    static const auto fn = reinterpret_cast<SetThreadDescriptionFn>(
        reinterpret_cast<void*>(GetProcAddress(GetModuleHandleW(L"kernel32.dll"), "SetThreadDescription")));
    if (!fn) return;

    std::wstring wide(name.begin(), name.end());
    fn(GetCurrentThread(), wide.c_str());
#else
    (void)name;
#endif
}

bool pinCurrentThreadToCore(unsigned logicalCore) noexcept {
#ifdef _WIN32
    if (logicalCore >= 64) return false; // single processor group only
    const DWORD_PTR mask = DWORD_PTR{1} << logicalCore;
    return SetThreadAffinityMask(GetCurrentThread(), mask) != 0;
#else
    (void)logicalCore;
    return false;
#endif
}

std::filesystem::path executableDir() {
#ifdef _WIN32
    wchar_t buffer[MAX_PATH];
    const DWORD n = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    if (n == 0 || n == MAX_PATH) return std::filesystem::current_path();
    return std::filesystem::path(buffer).parent_path();
#else
    return std::filesystem::current_path();
#endif
}

std::filesystem::path configDir() {
    std::filesystem::path dir;
#ifdef _WIN32
    PWSTR local = nullptr;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, nullptr, &local))) {
        dir = std::filesystem::path(local) / "flightsim";
        CoTaskMemFree(local);
    }
#endif
    if (dir.empty()) {
        if (const char* home = std::getenv("HOME")) dir = std::filesystem::path(home) / ".flightsim";
        else dir = std::filesystem::current_path() / ".flightsim";
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return dir;
}

} // namespace fsim::platform
