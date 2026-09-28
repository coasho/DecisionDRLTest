#pragma once

// The C ABI's opaque handles (shared by the translation units of fsim.dll).

#include "fsim/PerformanceProfile.h"
#include "fsim/Capability.h"
#include "fsim/World.h"
#include "fsim/fsim_c.h"
#include "session/World.h"

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

struct fsim_world {
    std::shared_ptr<fsim::session::World> owned; ///< null when the world belongs to something else (a VecEnv)
    fsim::session::World& world;
    std::unique_ptr<fsim::World> object;          ///< the C++ view handed out by fsim_world_object(), made on demand
    std::unordered_set<std::string> strings;      ///< strings handed out (capability ids, parameter names): stable until the world goes
    fsim::control::CommandResult last;             ///< the last answer to a NEW, UPDATE or CANCEL (fsim_last_command_detail)
    uint32_t lastVehicle = 0;                      ///< the vehicle that gave it: its details (World::commandDetails) are the last command's
    std::vector<fsim::control::Waypoint> waypoints; ///< a route's, as given (reused: a route updated every step allocates nothing once it has room)
    std::vector<fsim::control::BezierSegment> segments; ///< a curve's, as given (reused likewise)
    std::vector<fsim::control::NurbsSegment> nurbs;     ///< a curve's as A-GRA's schema gives them (likewise)
    /// An activity's setpoint as last read back (fsim_activity_get_setpoint): what its arrays point into.
    struct Readback {
        fsim::control::Setpoint setpoint;
        std::vector<double> fields;
        std::vector<fsim_waypoint> waypoints;
        std::vector<fsim_bezier_segment> segments;
        std::vector<fsim_nurbs_segment> nurbs;
        fsim_behavior_command behavior{};
        std::vector<const char*> names;
        std::vector<double> values;
        std::vector<fsim_position_command> points;
    } readback;
    /// The performance profile last asked for (fsim_vehicle_performance_profile): what its arrays point into.
    fsim::control::PerformanceProfile profile;

    const char* intern(const std::string& s) { return strings.insert(s).first->c_str(); }

    explicit fsim_world(const fsim::session::WorldOptions& o) : owned(std::make_shared<fsim::session::World>(o)), world(*owned) {}
    explicit fsim_world(fsim::session::World& borrowed) : world(borrowed) {}
};
