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
    std::vector<fsim::control::RouteLoiter> loiters;    ///< a route's loiters (likewise)
    std::vector<fsim::control::RouteState> states;      ///< a route's planned states (likewise)
    std::vector<fsim::control::RoutePath> paths;        ///< a route's paths (likewise)
    std::vector<fsim::control::RouteBranch> branches;   ///< a route's conditional branches (likewise)
    std::vector<fsim::control::RouteTerminator> terminators; ///< a route's civil path terminators' data (likewise)
    /// An activity's setpoint as last read back (fsim_activity_get_setpoint): what its arrays point into.
    struct Readback {
        fsim::control::Setpoint setpoint;
        std::vector<double> fields;
        std::vector<fsim_waypoint> waypoints;
        std::vector<fsim_bezier_segment> segments;
        std::vector<fsim_nurbs_segment> nurbs;
        std::vector<fsim_route_loiter> loiters;
        std::vector<fsim_route_state> states;
        std::vector<fsim_route_path> paths;
        std::vector<fsim_route_branch> branches;
        std::vector<fsim_route_terminator> terminators;
        fsim_behavior_command behavior{};
        std::vector<const char*> names;
        std::vector<double> values;
        std::vector<fsim_position_command> points;
    } readback;
    /// A route plan as last read back (fsim_vehicle_get_plan): what its metadata's arrays and texts point into (its route's
    /// arrays: `readback`'s).
    struct PlanReadback {
        fsim::control::RoutePlan plan;
        std::vector<fsim_point_metadata> points;
        std::vector<fsim_path_metadata> paths;
    } planReadback;
    /// An airfield as last read back (fsim_vehicle_get_airfield, _at): what its runways and ICAO code point into.
    struct AirfieldReadback {
        fsim::control::Airfield airfield;
        std::vector<fsim_runway> runways;
    } airfieldReadback;
    /// An operational zone as last read back (fsim_world_get_op_zone, _at): what its vertices and holes point into.
    struct ZoneReadback {
        std::vector<fsim_zone_vertex> vertices;
        std::vector<std::vector<fsim_zone_vertex>> holes;
        std::vector<const fsim_zone_vertex*> holePointers;
        std::vector<uint32_t> holeSizes;
    } zoneReadback;
    /// The performance profile last asked for (fsim_vehicle_performance_profile): what its arrays point into.
    fsim::control::PerformanceProfile profile;

    const char* intern(const std::string& s) { return strings.insert(s).first->c_str(); }

    explicit fsim_world(const fsim::session::WorldOptions& o) : owned(std::make_shared<fsim::session::World>(o)), world(*owned) {}
    explicit fsim_world(fsim::session::World& borrowed) : world(borrowed) {}
};
