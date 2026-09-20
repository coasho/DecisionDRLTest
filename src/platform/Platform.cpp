#include "platform/Clock.h"
#include "platform/Paths.h"
#include "platform/Threads.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#    include <windows.h>
#    include <shlobj.h>
#    include <timeapi.h>
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

void sleepUntil(Clock::time_point deadline) noexcept {
    using namespace std::chrono;
#ifdef _WIN32
    // One high-resolution waitable timer per thread (Windows 10 1803+; falls
    // back to an ordinary timer, then to a plain sleep, on older systems).
    thread_local HANDLE timer = [] {
        HANDLE h = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
        if (!h) h = CreateWaitableTimerExW(nullptr, nullptr, 0, TIMER_ALL_ACCESS);
        return h;
    }();
    constexpr auto spinMargin = microseconds(150);
    if (timer) {
        const auto remaining = deadline - Clock::now();
        if (remaining > spinMargin) {
            LARGE_INTEGER due;
            due.QuadPart = -static_cast<LONGLONG>(duration_cast<nanoseconds>(remaining - spinMargin).count() / 100); // relative, 100 ns units
            if (SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE)) WaitForSingleObject(timer, INFINITE);
        }
    } else {
        const auto remaining = deadline - Clock::now();
        if (remaining > milliseconds(2)) std::this_thread::sleep_for(remaining - milliseconds(2));
    }
#else
    const auto remaining = deadline - Clock::now();
    if (remaining > microseconds(200)) std::this_thread::sleep_for(remaining - microseconds(200));
#endif
    while (Clock::now() < deadline) std::this_thread::yield();
}

bool attachParentConsole() noexcept {
#ifdef _WIN32
    // Streams the parent redirected (pipes, files) already have valid handles
    // and must be left alone; only unconnected ones go to the console.
    auto valid = [](DWORD id) {
        HANDLE h = GetStdHandle(id);
        return h != nullptr && h != INVALID_HANDLE_VALUE;
    };
    const bool outRedirected = valid(STD_OUTPUT_HANDLE), errRedirected = valid(STD_ERROR_HANDLE), inRedirected = valid(STD_INPUT_HANDLE);
    if (!AttachConsole(ATTACH_PARENT_PROCESS)) return outRedirected || errRedirected;
    FILE* f = nullptr;
    if (!outRedirected) freopen_s(&f, "CONOUT$", "w", stdout);
    if (!errRedirected) freopen_s(&f, "CONOUT$", "w", stderr);
    if (!inRedirected) freopen_s(&f, "CONIN$", "r", stdin);
    std::ios::sync_with_stdio();
    return true;
#else
    return true;
#endif
}

double processCpuSeconds() noexcept {
#ifdef _WIN32
    FILETIME creation, exit, kernel, user;
    if (!GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user)) return 0.0;
    auto toSeconds = [](const FILETIME& ft) {
        ULARGE_INTEGER v;
        v.LowPart = ft.dwLowDateTime;
        v.HighPart = ft.dwHighDateTime;
        return static_cast<double>(v.QuadPart) * 1e-7;
    };
    return toSeconds(kernel) + toSeconds(user);
#else
    return static_cast<double>(std::clock()) / CLOCKS_PER_SEC;
#endif
}

unsigned logicalProcessorOfCore(unsigned physicalCore) noexcept {
#ifdef _WIN32
    DWORD length = 0;
    GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &length);
    if (length == 0) return physicalCore;
    std::vector<unsigned char> buffer(length);
    auto* info = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buffer.data());
    if (!GetLogicalProcessorInformationEx(RelationProcessorCore, info, &length)) return physicalCore;
    unsigned index = 0;
    for (DWORD offset = 0; offset < length;) {
        auto* entry = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buffer.data() + offset);
        if (entry->Relationship == RelationProcessorCore) {
            if (index == physicalCore && entry->Processor.GroupCount > 0) {
                const KAFFINITY mask = entry->Processor.GroupMask[0].Mask;
                for (unsigned bit = 0; bit < 64; ++bit)
                    if (mask & (KAFFINITY{1} << bit)) return bit; // group 0 only
            }
            ++index;
        }
        offset += entry->Size;
    }
    return physicalCore;
#else
    return physicalCore;
#endif
}

bool enableHighDpiAwareness() noexcept {
#ifdef _WIN32
    // Windows 10 1703+: per-monitor v2. Loaded dynamically so older systems fall
    // back to the system-DPI-aware call from Vista.
    using SetCtxFn = BOOL(WINAPI*)(DPI_AWARENESS_CONTEXT);
    if (const auto fn = reinterpret_cast<SetCtxFn>(
            reinterpret_cast<void*>(GetProcAddress(GetModuleHandleW(L"user32.dll"), "SetProcessDpiAwarenessContext")))) {
        if (fn(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2)) return true;
    }
    return SetProcessDPIAware() != 0;
#else
    return false;
#endif
}

void requestHighResolutionTimer() noexcept {
#ifdef _WIN32
    static bool done = false;
    if (done) return;
    done = true;
    timeBeginPeriod(1);
    std::atexit([] { timeEndPeriod(1); });
#endif
}

double systemDpiScale() noexcept {
#ifdef _WIN32
    using GetDpiFn = UINT(WINAPI*)();
    if (const auto fn = reinterpret_cast<GetDpiFn>(
            reinterpret_cast<void*>(GetProcAddress(GetModuleHandleW(L"user32.dll"), "GetDpiForSystem")))) {
        return static_cast<double>(fn()) / 96.0;
    }
    const HDC dc = GetDC(nullptr);
    const int dpi = GetDeviceCaps(dc, LOGPIXELSX);
    ReleaseDC(nullptr, dc);
    return dpi > 0 ? dpi / 96.0 : 1.0;
#else
    return 1.0;
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
