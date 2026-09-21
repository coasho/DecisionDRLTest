#pragma once

// The C ABI's opaque handles (shared by the translation units of fsim.dll).

#include "fsim/World.h"
#include "session/World.h"

#include <memory>

struct fsim_world {
    std::shared_ptr<fsim::session::World> owned; ///< null when the world belongs to something else (a VecEnv)
    fsim::session::World& world;
    std::unique_ptr<fsim::World> object;          ///< the C++ view handed out by fsim_world_object(), made on demand

    explicit fsim_world(const fsim::session::WorldOptions& o) : owned(std::make_shared<fsim::session::World>(o)), world(*owned) {}
    explicit fsim_world(fsim::session::World& borrowed) : world(borrowed) {}
};
