// Run a scenario file (design 9.13): the whole experiment - world, environment,
// vehicles, commands, effects - is data. Start flightsim-viewer.exe at any
// time to watch it.
//
//   scenario_runner <file.json> [--seconds S] [--realtime] [--quiet] [--dump]

#include <fsim/Scenario.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using namespace fsim;

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
}

int main(int argc, char** argv) {
    std::string file;
    double seconds = 120.0;
    bool realtime = false, quiet = false, dump = false;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (k == "--seconds") seconds = std::atof(next());
        else if (k == "--realtime") realtime = true;
        else if (k == "--quiet") quiet = true;
        else if (k == "--dump") dump = true;
        else file = k;
    }
    if (file.empty()) {
        std::fprintf(stderr, "usage: scenario_runner <file.json> [--seconds S] [--realtime] [--quiet] [--dump]\n");
        return 2;
    }
    try {
        const Scenario scenario = loadScenario(file);
        if (dump) std::printf("%s\n", dumpScenario(scenario).c_str());
        World world(scenario.world);
        const auto vehicles = applyScenario(world, scenario);
        std::printf("scenario %s: world '%s', %zu vehicle(s)%s\n", scenario.source.c_str(), world.name().c_str(), vehicles.size(),
                    world.published() ? " (published for viewers)" : "");

        const double stepS = world.stepSeconds();
        const int steps = static_cast<int>(seconds / stepS);
        const auto t0 = std::chrono::steady_clock::now();
        for (int k = 1; k <= steps; ++k) {
            world.step();
            if (!quiet && k % static_cast<int>(10.0 / stepS) == 0) {
                std::printf("--- %.0f s\n", world.time());
                for (const auto& v : world.vehicles()) {
                    const auto& s = v.state();
                    std::printf("  %-12s %-12s alt %7.1f m  hdg %5.1f  tas %5.1f m/s%s\n", v.name().c_str(), control::levelName(v.activeLevel()),
                                s.altitudeMslM, s.eulerRad[2] / kDeg, s.airspeedTrueMs, s.diverged ? "  DIVERGED" : "");
                }
            }
            if (realtime) {
                const auto target = t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(k * stepS));
                std::this_thread::sleep_until(target);
            }
        }
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("%d world steps in %.2f s (%.0fx real time)\n", steps, wall, seconds / wall);
        return 0;
    } catch (const Error& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
