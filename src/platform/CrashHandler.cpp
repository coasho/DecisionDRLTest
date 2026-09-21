#include "platform/CrashHandler.h"

#include "platform/Paths.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <dbghelp.h>
#endif

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

namespace fsim::platform {

namespace {

std::filesystem::path g_dumpDir;
bool g_installed = false;

#ifdef _WIN32
// dbghelp is loaded lazily: the DLL ships with Windows but is not linked at build time.
using MiniDumpWriteDumpFn = BOOL(WINAPI*)(HANDLE, DWORD, HANDLE, MINIDUMP_TYPE, PMINIDUMP_EXCEPTION_INFORMATION, PMINIDUMP_USER_STREAM_INFORMATION,
                                          PMINIDUMP_CALLBACK_INFORMATION);

LONG WINAPI onUnhandledException(EXCEPTION_POINTERS* info) {
    char name[64];
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &now);
    std::strftime(name, sizeof name, "%Y%m%d-%H%M%S", &tm);
    const std::filesystem::path exe = executableDir();
    wchar_t module[MAX_PATH];
    GetModuleFileNameW(nullptr, module, MAX_PATH);
    const std::filesystem::path dump = g_dumpDir / (std::filesystem::path(module).stem().string() + "-" + name + ".dmp");

    const DWORD code = info && info->ExceptionRecord ? info->ExceptionRecord->ExceptionCode : 0;
    std::fprintf(stderr, "\n*** crash: exception 0x%08lx", static_cast<unsigned long>(code));
    if (HMODULE dbghelp = LoadLibraryW(L"dbghelp.dll")) {
        const FARPROC proc = GetProcAddress(dbghelp, "MiniDumpWriteDump");
        MiniDumpWriteDumpFn write = nullptr;
        static_assert(sizeof(proc) == sizeof(write), "function pointer sizes");
        std::memcpy(&write, &proc, sizeof(write)); // GetProcAddress returns a generic function pointer
        if (write) {
            std::error_code ec;
            std::filesystem::create_directories(g_dumpDir, ec);
            HANDLE file = CreateFileW(dump.wstring().c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
            if (file != INVALID_HANDLE_VALUE) {
                MINIDUMP_EXCEPTION_INFORMATION mei{};
                mei.ThreadId = GetCurrentThreadId();
                mei.ExceptionPointers = info;
                mei.ClientPointers = FALSE;
                const auto type = static_cast<MINIDUMP_TYPE>(MiniDumpWithIndirectlyReferencedMemory | MiniDumpWithDataSegs | MiniDumpWithThreadInfo);
                const BOOL ok = write(GetCurrentProcess(), GetCurrentProcessId(), file, type, info ? &mei : nullptr, nullptr, nullptr);
                CloseHandle(file);
                std::fprintf(stderr, "; minidump %s %s", ok ? "written to" : "FAILED at", dump.string().c_str());
            }
        }
    }
    std::fprintf(stderr, " ***\n");
    std::fflush(stderr);
    return EXCEPTION_CONTINUE_SEARCH; // let Windows finish (error reporting, debugger)
}
#endif

} // namespace

std::filesystem::path crashDumpDir() { return g_dumpDir.empty() ? configDir() / "crash" : g_dumpDir; }

void installCrashHandler(const std::filesystem::path& dumpDir) {
    g_dumpDir = dumpDir.empty() ? configDir() / "crash" : dumpDir;
    if (g_installed) return;
    g_installed = true;
#ifdef _WIN32
    SetUnhandledExceptionFilter(onUnhandledException);
#endif
}

} // namespace fsim::platform
