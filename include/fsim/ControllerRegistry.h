#pragma once

#include "fsim/Control.h"
#include "fsim/Export.h"

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace fsim::control {

/// id -> factory for controllers and behaviours (design 9.3
/// "ControllerRegistry"). Built-ins register themselves on first use; trainer
/// code adds its own before creating vehicles. Thread-safe.
class FSIM_API ControllerRegistry {
public:
    using Factory = std::function<std::unique_ptr<Controller>()>;

    static ControllerRegistry& instance();

    /// Register (or replace) a controller factory for `level`.
    void add(std::string id, Level level, Factory factory);
    /// Register (or replace) a behaviour factory (level Behavior).
    void addBehavior(std::string id, std::function<std::unique_ptr<Behavior>()> factory);

    /// Create by id; null if unknown.
    std::unique_ptr<Controller> create(std::string_view id) const;
    /// Level of a registered id, or Level::Count if unknown.
    Level levelOf(std::string_view id) const;

    /// Ids registered for a level (sorted).
    std::vector<std::string> ids(Level level) const;

    /// Default controller id per level (the built-in PID loops).
    const char* defaultId(Level level) const noexcept;

private:
    ControllerRegistry();
    struct Entry {
        Level level;
        Factory factory;
    };
    mutable std::mutex mutex_;
    std::vector<std::pair<std::string, Entry>> entries_;
};

/// Registers the built-in loops and behaviours (idempotent).
void registerBuiltinControllers(ControllerRegistry& registry);

} // namespace fsim::control
