#pragma once

#include "fsim/Capability.h"
#include "fsim/Control.h"
#include "fsim/ControlInputs.h"
#include "fsim/Export.h"

#include <array>
#include <memory>
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

// The boundary with the contract layer (docs/control-architecture.md, 6):
// defined in the platform's src/control/Runtime.h, not part of the SDK.
struct RuntimeConfig;
struct RuntimeReport;

/// Per-vehicle control stack (design 9.3): one controller per level and the
/// commands engaged. `update()` runs the cascade from the engaged level down
/// to the actuators and fills the flight model's inputs. It is the control
/// runtime of ADR-26: it flies what its RuntimeConfig holds and reports in its
/// RuntimeReport. Runs on the worker that steps the vehicle; all calls that
/// change it come from the caller's thread between steps.
class FSIM_API ControlStack {
public:
    ControlStack();
    ~ControlStack();
    ControlStack(ControlStack&&) noexcept;
    ControlStack& operator=(ControlStack&&) noexcept;

    /// Set the active command; its level becomes the active level. (A World's
    /// vehicles take their commands through the World, whose contract layer
    /// writes the stack's configuration; this is the stack on its own.)
    void command(const Command& command);
    /// The highest level a command enters at; Actuator before any command.
    Level activeLevel() const noexcept;
    /// The command at the active level; before any command, the neutral
    /// actuator command every vehicle starts with.
    const Command* activeCommand() const noexcept;

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
    /// engaged command at its own level), or null if that level did not run.
    const Command* derived(Level level) const noexcept { return derived_[static_cast<std::size_t>(level)]; }

    /// The running behaviour, if a behaviour is engaged.
    const Behavior* behavior() const noexcept;
    bool behaviorFinished() const noexcept;

    /// Reset every controller's internal state (vehicle reset); engaged
    /// commands stay, and behaviours start again.
    void reset();

    /// Inputs held for channels no command sets (throttle, gear) before the
    /// first command; also what reset() returns to.
    void setInitialInputs(const sim::ControlInputs& inputs) noexcept { initial_ = inputs; last_ = inputs; }

    // --- The contract layer's side of the boundary (platform-internal) -------
    /// What to fly: written only between steps.
    RuntimeConfig& config() noexcept;
    const RuntimeConfig& config() const noexcept;
    /// What was flown: read and cleared only between steps.
    RuntimeReport& report() noexcept;
    const RuntimeReport& report() const noexcept;
    /// Hand over the behaviour a slot at Level::Behavior flies (between steps);
    /// it starts at its next update. Null removes it.
    void install(std::size_t slot, std::unique_ptr<Behavior> behavior) noexcept;

private:
    static constexpr std::size_t kLevels = static_cast<std::size_t>(Level::Count);
    static constexpr std::size_t kNoSlot = kSlotCount;

    bool apply(Controller& controller) const;
    std::size_t engagedSlot() const noexcept;
    void actuate(const ActuatorCommand& a, sim::ControlInputs& out) noexcept;
    void fail(sim::ControlInputs& out) noexcept;

    std::unique_ptr<RuntimeConfig> config_;
    std::unique_ptr<RuntimeReport> report_;
    std::array<std::unique_ptr<Controller>, kLevels> controllers_;
    std::array<bool, kLevels> byId_{}; ///< created from the registry (settings apply)
    std::vector<ControllerSetting> settings_;
    std::array<std::unique_ptr<Behavior>, kSlotCount> behaviors_; ///< per slot, at Level::Behavior
    std::array<std::uint32_t, kSlotCount> started_{};              ///< the slot generation each behaviour was started for
    std::array<const Command*, kLevels> derived_{};                ///< the last update's command per level (null: did not run)
    std::array<Command, kLevels> produced_{};                      ///< the controllers' outputs, by level
    sim::ControlInputs last_;
    sim::ControlInputs initial_;
};

} // namespace fsim::control
