#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace fsim::app {

/// Command-line options for flightsim.exe (design 9.4). Every option maps to a
/// key of the configuration schema; vsg::CommandLine replaces this parser once
/// the viewer lands, without changing the option names.
struct Options {
    std::string aircraft = "c172x";
    std::filesystem::path jsbsimRoot;  ///< empty = auto-detect via AssetResolver
    unsigned vehicles = 1;
    unsigned workers = 0;              ///< 0 = physical cores - 2 (min 1)
    unsigned steps = 300;              ///< agent steps
    int frameSkip = 4;                 ///< FDM steps per agent step
    double dt = 1.0 / 120.0;           ///< FDM step, seconds
    std::uint64_t seed = 1;
    double throttle = 0.65;            ///< constant throttle command for the smoke run
    bool benchmark = false;            ///< suppress periodic prints, report throughput only
    bool pinWorkers = false;
    unsigned printEvery = 30;          ///< agent steps between status lines (0 = never)
    std::string logLevel = "info";
    bool help = false;
};

/// Parse argv. Returns nullopt and prints a message on error.
std::optional<Options> parseOptions(int argc, char** argv);

void printUsage(const char* program);

} // namespace fsim::app
