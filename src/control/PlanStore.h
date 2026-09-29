// The route plan store's entries (docs/flight-autonomy.md, 4.39): Plans.cpp's, and the route intercept's (4.47; Intercept.cpp),
// which flies a kept plan - CapabilityHost's own, apart from its header so that growing either moves neither.
#pragma once

#include "control/CapabilityHost.h"

#include <vector>

namespace fsim::control {

struct CapabilityHost::PlanEntry {
    PlanId id = 0;
    RoutePlan kept;               ///< as uploaded last (revision above 0)
    RoutePlan received;           ///< published since it was prepared for upload
    bool hasReceived = false;
    std::uint32_t revision = 0;   ///< uploads kept
    PlanState state = PlanState::ReadyForUpload;
    Reason reason = Reason::None; ///< why its last command failed, or why its execution ended
    ActivityId activity = 0;      ///< its activity, since it was activated last
    std::uint64_t commandId = 0;  ///< its activation's
    PlanExecution ended = PlanExecution::None; ///< once its activity ended (or FA aborted it unflown)
    double percent = kUnknown, startTime = kUnknown, endTime = kUnknown;
    bool faOwned = false;         ///< FA's own: loaded by the platform, read only to MA (4.40)
};

struct CapabilityHost::PlanStore {
    std::vector<PlanEntry> plans;    ///< in the order they were first prepared for upload, or loaded
    std::vector<Airfield> airfields; ///< FA's (4.40), in the order they were first loaded
    /// While a validation runs (4.41): the wind it gives, which the route's checks turn in (checkWind).
    bool windGiven = false;
    double windNorthMs = 0.0, windEastMs = 0.0;
    /// A route intercept's scratch (4.47): its plan's route with its join laid in, as the last intercept prepared it - room
    /// made at its first NEW, so that one starting in a step allocates nothing.
    std::vector<Waypoint> laid;
    std::vector<RoutePath> laidPaths;
    InterceptJoin laidJoin;
};

} // namespace fsim::control
