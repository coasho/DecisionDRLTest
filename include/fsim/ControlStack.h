#pragma once

#include "fsim/Control.h"
#include "fsim/ControlInputs.h"
#include "fsim/Export.h"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace fsim::control {

/// A parameter for the controller registered as `controller`: how an
/// aircraft carries its own gains for the built-in loops (design 9.3).
struct ControllerSetting {
    std::string controller; ///< registry id, e.g. "pid_attitude"
    std::string parameter;  ///< e.g. "pitch.kp"
    double value = 0.0;
};

/// Per-vehicle control stack (design 9.3): one controller per level and one
/// active command. `update()` runs the cascade from the active level down to
/// the actuators and fills the flight model's inputs. Runs on the worker that
/// steps the vehicle; all calls that change it come from the caller's thread
/// between steps.
class FSIM_API ControlStack {
public:
    ControlStack();
    ~ControlStack();
    ControlStack(ControlStack&&) noexcept;
    ControlStack& operator=(ControlStack&&) noexcept;

    /// Set the active command; its level becomes the active level.
    void command(const Command& command);
    Level activeLevel() const noexcept { return active_ ? levelOf(*active_) : Level::Actuator; }
    const Command* activeCommand() const noexcept { return active_ ? &*active_ : nullptr; }

    /// Replace the controller at a level (by registry id or instance). Returns
    /// false when the id is unknown or the controller's level does not match.
    /// A controller created by id gets the settings of setControllerSettings();
    /// an instance is used as it is.
    bool use(Level level, std::string_view controllerId);
    bool use(Level level, std::unique_ptr<Controller> controller);

    /// Parameters for controllers by registry id - the aircraft's own gains:
    /// set now on the controllers this stack created by id, and on every one
    /// use(level, id) creates later. Returns the settings none of them took
    /// (a parameter the controller does not have, or an id no level runs).
    std::vector<ControllerSetting> setControllerSettings(std::vector<ControllerSetting> settings);
    const std::vector<ControllerSetting>& controllerSettings() const noexcept { return settings_; }
    Controller* controller(Level level) noexcept { return controllers_[static_cast<std::size_t>(level)].get(); }
    const Controller* controller(Level level) const noexcept { return controllers_[static_cast<std::size_t>(level)].get(); }

    /// Run the cascade for one control period and write the actuator inputs.
    void update(const ControlContext& ctx, sim::ControlInputs& out);

    /// The command the cascade produced at `level` in the last update (the
    /// active command at the active level), or null if that level did not run.
    const Command* derived(Level level) const noexcept {
        const auto& d = derived_[static_cast<std::size_t>(level)];
        return d ? &*d : nullptr;
    }

    /// The running behaviour, if the active level is Behavior.
    const Behavior* behavior() const noexcept { return behavior_.get(); }
    bool behaviorFinished() const noexcept { return behavior_ && behavior_->finished(); }

    /// Reset every controller's internal state (vehicle reset).
    void reset();

    /// Inputs held for channels no command sets (throttle, gear) before the
    /// first command; also what reset() returns to.
    void setInitialInputs(const sim::ControlInputs& inputs) noexcept { initial_ = inputs; last_ = inputs; }

private:
    bool apply(Controller& controller) const;

    std::optional<Command> active_;
    std::array<std::unique_ptr<Controller>, static_cast<std::size_t>(Level::Count)> controllers_;
    std::array<bool, static_cast<std::size_t>(Level::Count)> byId_{}; ///< created from the registry (settings apply)
    std::vector<ControllerSetting> settings_;
    std::array<std::optional<Command>, static_cast<std::size_t>(Level::Count)> derived_;
    std::unique_ptr<Behavior> behavior_;
    bool behaviorStarted_ = false;
    sim::ControlInputs last_;
    sim::ControlInputs initial_;
};

} // namespace fsim::control
