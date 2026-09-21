// Crash handler: install with a given dump directory, then crash on purpose.
#include "platform/CrashHandler.h"

#include <cstdint>
#include <cstdio>

int main(int argc, char** argv) {
    fsim::platform::installCrashHandler(argc > 1 ? argv[1] : "");
    std::printf("dump dir: %s\n", fsim::platform::crashDumpDir().string().c_str());
    std::fflush(stdout);
    // An address the compiler cannot see through: page 0 + argc (never mapped).
    volatile int* p = reinterpret_cast<volatile int*>(static_cast<std::uintptr_t>(argc) * 4);
    return *p; // access violation
}
