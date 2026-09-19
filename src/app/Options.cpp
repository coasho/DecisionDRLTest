#include "app/Options.h"

#include <cstdio>
#include <cstdlib>
#include <string_view>

namespace fsim::app {

namespace {

bool parseUnsigned(std::string_view s, unsigned& out) {
    char* end = nullptr;
    const unsigned long v = std::strtoul(std::string(s).c_str(), &end, 10);
    if (!end || *end != '\0') return false;
    out = static_cast<unsigned>(v);
    return true;
}

bool parseInt(std::string_view s, int& out) {
    char* end = nullptr;
    const long v = std::strtol(std::string(s).c_str(), &end, 10);
    if (!end || *end != '\0') return false;
    out = static_cast<int>(v);
    return true;
}

bool parseDouble(std::string_view s, double& out) {
    char* end = nullptr;
    const double v = std::strtod(std::string(s).c_str(), &end);
    if (!end || *end != '\0') return false;
    out = v;
    return true;
}

} // namespace

void printUsage(const char* program) {
    std::printf(
        "Usage: %s [options]\n"
        "\n"
        "Headless JSBSim run: loads N vehicles and steps them in lockstep.\n"
        "\n"
        "  --aircraft <name>      JSBSim aircraft (default c172x)\n"
        "  --jsbsim-root <dir>    directory with aircraft/, engine/, systems/ (auto-detected)\n"
        "  --vehicles <n>         number of vehicles (default 1)\n"
        "  --workers <n>          sim worker threads (default: physical cores - 2)\n"
        "  --steps <n>            agent steps to run (default 300)\n"
        "  --frame-skip <n>       FDM steps per agent step (default 4)\n"
        "  --dt <seconds>         FDM step (default 1/120)\n"
        "  --seed <n>             seed for initial-condition sampling (default 1)\n"
        "  --throttle <0..1>      constant throttle for the smoke run (default 0.65)\n"
        "  --pin-workers          pin worker threads to cores\n"
        "  --benchmark            print only the throughput summary\n"
        "  --print-every <n>      status line every n agent steps (default 30, 0 = never)\n"
        "  --log-level <lvl>      trace|debug|info|warn|error (default info)\n"
        "  -h, --help\n",
        program);
}

std::optional<Options> parseOptions(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        auto value = [&](std::string_view& out) {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %.*s\n", static_cast<int>(arg.size()), arg.data());
                return false;
            }
            out = argv[++i];
            return true;
        };
        std::string_view v;
        bool ok = true;

        if (arg == "-h" || arg == "--help") o.help = true;
        else if (arg == "--aircraft") ok = value(v) && (o.aircraft = std::string(v), true);
        else if (arg == "--jsbsim-root") ok = value(v) && (o.jsbsimRoot = std::filesystem::path(v), true);
        else if (arg == "--vehicles") ok = value(v) && parseUnsigned(v, o.vehicles);
        else if (arg == "--workers") ok = value(v) && parseUnsigned(v, o.workers);
        else if (arg == "--steps") ok = value(v) && parseUnsigned(v, o.steps);
        else if (arg == "--frame-skip") ok = value(v) && parseInt(v, o.frameSkip);
        else if (arg == "--dt") ok = value(v) && parseDouble(v, o.dt);
        else if (arg == "--seed") {
            unsigned s = 0;
            ok = value(v) && parseUnsigned(v, s);
            o.seed = s;
        } else if (arg == "--throttle") ok = value(v) && parseDouble(v, o.throttle);
        else if (arg == "--pin-workers") o.pinWorkers = true;
        else if (arg == "--benchmark") o.benchmark = true;
        else if (arg == "--print-every") ok = value(v) && parseUnsigned(v, o.printEvery);
        else if (arg == "--log-level") ok = value(v) && (o.logLevel = std::string(v), true);
        else {
            std::fprintf(stderr, "unknown option: %.*s\n", static_cast<int>(arg.size()), arg.data());
            return std::nullopt;
        }

        if (!ok) {
            std::fprintf(stderr, "invalid value for %.*s\n", static_cast<int>(arg.size()), arg.data());
            return std::nullopt;
        }
    }

    if (o.vehicles == 0 || o.frameSkip < 1 || o.dt <= 0.0) {
        std::fprintf(stderr, "vehicles must be >= 1, frame-skip >= 1, dt > 0\n");
        return std::nullopt;
    }
    return o;
}

} // namespace fsim::app
