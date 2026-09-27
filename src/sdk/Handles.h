#pragma once

// The C ABI's opaque handles (shared by the translation units of fsim.dll).

#include "fsim/Capability.h"
#include "fsim/World.h"
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
    std::vector<fsim::control::Waypoint> waypoints; ///< a route's, as given (reused: a route updated every step allocates nothing once it has room)

    const char* intern(const std::string& s) { return strings.insert(s).first->c_str(); }

    explicit fsim_world(const fsim::session::WorldOptions& o) : owned(std::make_shared<fsim::session::World>(o)), world(*owned) {}
    explicit fsim_world(fsim::session::World& borrowed) : world(borrowed) {}
};
