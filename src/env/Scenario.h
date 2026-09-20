#pragma once

#include "sim/FlightModel.h"

#include <cstdint>
#include <filesystem>
#include <string>

namespace fsim::env {

/// Uniform jitter around a centre for initial conditions (design 9.1
/// "Scenario"). Sampled per vehicle from its own RNG stream.
struct InitialConditionRange {
    sim::InitialConditions centre;
    double latitudeJitterDeg = 0.02;
    double longitudeJitterDeg = 0.02;
    double altitudeJitterM = 150.0;
    double headingJitterDeg = 180.0; ///< 180 = any heading
    double airspeedJitterMs = 5.0;
};

/// What an environment simulates and how it is scored. In-code struct for
/// now; the .vsgt/JSON loader (design 9.1) maps onto it.
struct Scenario {
    std::string aircraft = "c172x";
    std::filesystem::path jsbsimRoot;   ///< empty = auto-detect
    unsigned vehiclesPerEnv = 1;
    double dt = 1.0 / 120.0;            ///< FDM step
    int frameSkip = 4;                  ///< FDM steps per agent step (30 Hz agent rate)
    std::uint32_t maxEpisodeSteps = 2000; ///< truncation, agent steps
    InitialConditionRange initial;

    std::string task = "altitude_heading_hold";
    std::string observation = "state";
    std::string action = "surfaces";

    // Task parameters (altitude_heading_hold): targets are sampled per episode
    // relative to the initial state within these ranges.
    double targetAltitudeDeltaM = 300.0;   ///< |target - initial altitude| <= this
    double targetHeadingDeltaDeg = 60.0;   ///< |target - initial heading| <= this
};

} // namespace fsim::env
