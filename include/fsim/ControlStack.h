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
class VehicleAdapter;

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
    /// What the last update asked of the effectors ControlInputs has no room for.
    const sim::EffectorInputs& effectors() const noexcept { return effectors_; }

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
    /// The adapter whose real-time face writes the inputs (a stock JSBSim one by default).
    void setAdapter(const VehicleAdapter& adapter) noexcept { adapter_ = &adapter; }
    const VehicleAdapter& adapter() const noexcept { return *adapter_; }

private:
    static constexpr std::size_t kLevels = static_cast<std::size_t>(Level::Count);
    static constexpr std::size_t kNoSlot = kSlotCount;

    bool apply(Controller& controller) const;
    /// The slot that owns every primary axis (the usual case), else kNoSlot.
    std::size_t wholeSlot() const noexcept;
    /// The slot at the highest level that owns a primary axis, else kNoSlot.
    std::size_t topSlot() const noexcept;
    /// The vehicle default is a hold and some primary axis has no owner.
    bool holdsDefault() const noexcept;
    /// One slot owns every primary axis (the usual case): its cascade, as it
    /// always ran - and, Protected, with the envelope protection stage.
    template <bool Protected>
    void cascade(const ControlContext& ctx, std::size_t slot, sim::ControlInputs& out);
    /// The axes owned apart (docs/control-architecture.md, 9.5): one pass down
    /// the levels, each merging the demands that reached it into one command.
    void flyMerged(const ControlContext& ctx, sim::ControlInputs& out);
    /// Nothing flies a primary axis through the cascade: the neutral actuator command.
    void flyNeutral(sim::ControlInputs& out);
    /// No slot owns every primary axis: the merged pass or the neutral
    /// command, then the state checked against the envelope.
    void flyGeneral(const ControlContext& ctx, sim::ControlInputs& out);
    /// The limits in force this update (protection on): active_; and, if the
    /// setpoints above the acceleration level are to be limited, what they
    /// need from the state.
    void selectLimits(const ControlContext& ctx, bool state) noexcept;
    /// What the protection stage limited, and the state against the limits in
    /// force, into the report (docs/control-architecture.md, 11.4).
    void watch(const ControlContext& ctx, std::uint16_t limited) noexcept;
    void actuate(const ActuatorCommand& a, sim::ControlInputs& out) noexcept;
    void fail(sim::ControlInputs& out) noexcept;

    std::unique_ptr<RuntimeConfig> config_;
    std::unique_ptr<RuntimeReport> report_;
    const VehicleAdapter* adapter_ = nullptr;
    std::array<std::unique_ptr<Controller>, kLevels> controllers_;
    std::array<bool, kLevels> byId_{}; ///< created from the registry (settings apply)
    std::vector<ControllerSetting> settings_;
    std::array<std::unique_ptr<Behavior>, kSlotCount> behaviors_; ///< per slot, at Level::Behavior
    std::array<std::uint32_t, kSlotCount> started_{};              ///< the slot generation each behaviour was started for
    std::array<const Command*, kLevels> derived_{};                ///< the last update's command per level (null: did not run)
    std::array<Command, kLevels> outputs_{};                       ///< each level's controller's output, by the level that produced it
    std::array<Command, kLevels> merged_{};                        ///< the commands merged from several demands, by level
    const EnvelopeLimits* active_ = nullptr;                       ///< the limits in force this update (protection on)
    double tasPerCas_ = 1.0, tasPerMach_ = 0.0, bankCos_ = 1.0;    ///< what they need from the state (Limit): a LimitState
    EnvelopeLimits geared_{};                                      ///< they, with the gear's speed, when it is down
    std::uint16_t limited_ = 0;                                    ///< what the merged pass's protection stage limited
    // The vehicle default's hold (VehicleDefault::Hold): what the axes nobody
    // owns fly, captured from the state as each was let go.
    Command hold_ = VelocityCommand{};
    std::array<std::uint32_t, kPrimaryAxisCount> captured_{}; ///< per axis: the RuntimeConfig::letGo its target was captured at
    AxisMask holdValid_ = 0;                                  ///< the axes whose captured target is valid
    double holdHeadingRad_ = 0.0, holdAirspeedMs_ = 0.0, holdAltitudeM_ = 0.0;
    sim::ControlInputs last_;
    sim::ControlInputs initial_;
    sim::EffectorInputs effectors_;
};

} // namespace fsim::control
