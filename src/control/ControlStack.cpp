#include "control/ControlStack.h"

#include "control/Registry.h"
#include "control/Runtime.h"
#include "core/Log.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

/// What every vehicle flies before its first command, and what flies an axis
/// nobody owns: surfaces centred, throttle 0, flaps up, brakes off, gear held.
const Command kNeutral = ActuatorCommand{};

} // namespace

const char* levelName(Level level) noexcept {
    switch (level) {
    case Level::Actuator: return "actuator";
    case Level::Attitude: return "attitude";
    case Level::Acceleration: return "acceleration";
    case Level::Velocity: return "velocity";
    case Level::Position: return "position";
    case Level::Behavior: return "behavior";
    default: return "?";
    }
}

ControlStack::ControlStack() : config_(std::make_unique<RuntimeConfig>()), report_(std::make_unique<RuntimeReport>()) {
    auto& registry = ControllerRegistry::instance();
    for (std::size_t l = 0; l < static_cast<std::size_t>(Level::Behavior); ++l) {
        const Level level = static_cast<Level>(l);
        if (const char* id = registry.defaultId(level)) {
            controllers_[l] = registry.create(id);
            byId_[l] = controllers_[l] != nullptr;
        }
    }
}

ControlStack::~ControlStack() = default;
ControlStack::ControlStack(ControlStack&&) noexcept = default;
ControlStack& ControlStack::operator=(ControlStack&&) noexcept = default;

RuntimeConfig& ControlStack::config() noexcept { return *config_; }
const RuntimeConfig& ControlStack::config() const noexcept { return *config_; }
RuntimeReport& ControlStack::report() noexcept { return *report_; }
const RuntimeReport& ControlStack::report() const noexcept { return *report_; }

void ControlStack::install(std::size_t slot, std::unique_ptr<Behavior> behavior) noexcept {
    if (slot >= kSlotCount) return;
    behaviors_[slot] = std::move(behavior);
    started_[slot] = 0;
}

void ControlStack::command(const Command& command) {
    // A stack on its own is its own host: the command takes slot 0 and every
    // axis a command struct can set, as the World's legacy commands do.
    SetpointSlot& slot = config_->slots[0];
    if (const auto* b = std::get_if<BehaviorCommand>(&command)) {
        auto created = ControllerRegistry::instance().create(b->id);
        auto* behavior = dynamic_cast<Behavior*>(created.get());
        if (!behavior) {
            LOG_ERROR("control") << "unknown behaviour '" << b->id << "'; holding the current command";
            return;
        }
        created.release();
        install(0, std::unique_ptr<Behavior>(behavior));
        ++slot.generation;
    } else if (behaviors_[0]) {
        install(0, nullptr);
    }
    slot.command = command;
    slot.level = levelOf(command);
    ++slot.revision;
    if (slot.axes != kLegacyAxes) { // the first command
        slot.axes = kLegacyAxes;
        for (std::size_t a = 0; a < kAxisCount; ++a)
            if (kLegacyAxes & (1u << a)) config_->owner[a] = 0;
        ++config_->revision;
    }
}

std::size_t ControlStack::engagedSlot() const noexcept {
    // Until axes can be owned apart (step 3), one slot owns every primary axis.
    const std::uint8_t owner = config_->owner[static_cast<std::size_t>(Axis::Pitch)];
    return owner < kSlotCount ? owner : kNoSlot;
}

Level ControlStack::activeLevel() const noexcept {
    const std::size_t s = engagedSlot();
    return s == kNoSlot ? Level::Actuator : config_->slots[s].level;
}

const Command* ControlStack::activeCommand() const noexcept {
    const std::size_t s = engagedSlot();
    return s == kNoSlot ? &kNeutral : &config_->slots[s].command;
}

const Behavior* ControlStack::behavior() const noexcept {
    const std::size_t s = engagedSlot();
    return s == kNoSlot || config_->slots[s].level != Level::Behavior ? nullptr : behaviors_[s].get();
}

bool ControlStack::behaviorFinished() const noexcept {
    const Behavior* b = behavior();
    return b && b->finished();
}

bool ControlStack::use(Level level, std::string_view controllerId) {
    auto& registry = ControllerRegistry::instance();
    auto c = registry.create(controllerId);
    if (!c) {
        LOG_ERROR("control") << "unknown controller '" << controllerId << "'";
        return false;
    }
    apply(*c);
    if (!use(level, std::move(c))) return false;
    byId_[static_cast<std::size_t>(level)] = true;
    return true;
}

bool ControlStack::use(Level level, std::unique_ptr<Controller> controller) {
    if (!controller || controller->level() != level || level == Level::Behavior) {
        LOG_ERROR("control") << "controller '" << (controller ? controller->id() : "null") << "' does not accept level " << levelName(level);
        return false;
    }
    controllers_[static_cast<std::size_t>(level)] = std::move(controller);
    byId_[static_cast<std::size_t>(level)] = false;
    return true;
}

bool ControlStack::apply(Controller& controller) const {
    bool any = false;
    for (const auto& s : settings_)
        if (s.controller == controller.id()) any = controller.setParameter(s.parameter, s.value) || any;
    return any;
}

std::vector<ControllerSetting> ControlStack::setControllerSettings(std::vector<ControllerSetting> settings) {
    settings_ = std::move(settings);
    std::vector<ControllerSetting> unused;
    for (const auto& s : settings_) {
        bool taken = false;
        for (std::size_t l = 0; l < controllers_.size(); ++l) {
            Controller* c = controllers_[l].get();
            if (c && byId_[l] && s.controller == c->id()) taken = c->setParameter(s.parameter, s.value) || taken;
        }
        if (!taken) unused.push_back(s);
    }
    return unused;
}

void ControlStack::fail(sim::ControlInputs& out) noexcept {
    ++report_->errors;
    out = last_;
}

void ControlStack::update(const ControlContext& ctx, sim::ControlInputs& out) {
    RuntimeReport& report = *report_;
    ++report.updates;
    derived_.fill(nullptr);

    const std::size_t s = engagedSlot();
    if (s == kNoSlot) {
        derived_[static_cast<std::size_t>(Level::Actuator)] = &kNeutral;
        actuate(std::get<ActuatorCommand>(kNeutral), out);
        return;
    }
    const SetpointSlot& slot = config_->slots[s];
    SlotReport& flown = report.slots[s];
    flown.generation = slot.generation;
    flown.revision = slot.revision;

    // The engaged command is read where the host keeps it, never copied.
    const Command* current = &slot.command;
    Level level = slot.level;
    derived_[static_cast<std::size_t>(level)] = current;

    // Behaviours own their lifecycle; the rest of the cascade is stateless per level.
    if (level == Level::Behavior) {
        Behavior* behavior = behaviors_[s].get();
        if (!behavior) return fail(out);
        if (started_[s] != slot.generation) {
            behavior->start(ctx, std::get<BehaviorCommand>(*current));
            started_[s] = slot.generation;
        }
        Command next = behavior->update(ctx, *current);
        if (behavior->finished()) flown.events |= kFinished;
        const Level nextLevel = levelOf(next);
        if (nextLevel >= level) {
            LOG_ERROR("control") << "behaviour '" << behavior->id() << "' returned a command at level " << levelName(nextLevel);
            return fail(out);
        }
        const auto n = static_cast<std::size_t>(nextLevel);
        produced_[n] = std::move(next);
        derived_[n] = &produced_[n];
        level = nextLevel;
        current = derived_[n];
    }

    while (level != Level::Actuator) {
        Controller* c = controllers_[static_cast<std::size_t>(level)].get();
        if (!c) {
            LOG_ERROR("control") << "no controller at level " << levelName(level);
            return fail(out);
        }
        Command next = c->update(ctx, *current);
        const Level nextLevel = levelOf(next);
        if (nextLevel >= level) {
            LOG_ERROR("control") << "controller '" << c->id() << "' returned a command at level " << levelName(nextLevel)
                                 << " (must be lower than " << levelName(level) << ")";
            return fail(out);
        }
        const auto n = static_cast<std::size_t>(nextLevel);
        produced_[n] = std::move(next);
        derived_[n] = &produced_[n];
        level = nextLevel;
        current = derived_[n];
    }
    actuate(std::get<ActuatorCommand>(*current), out);
}

void ControlStack::actuate(const ActuatorCommand& a, sim::ControlInputs& out) noexcept {
    auto clamp01 = [](double v) { return std::clamp(v, 0.0, 1.0); };
    auto clamp11 = [](double v) { return std::clamp(v, -1.0, 1.0); };
    out.aileron = clamp11(orHold(a.aileron, 0.0));
    out.elevator = clamp11(orHold(a.elevator, 0.0));
    out.rudder = clamp11(orHold(a.rudder, 0.0));
    out.setThrottleAll(clamp01(orHold(a.throttle, last_.throttle[0])));
    out.flaps = clamp01(orHold(a.flaps, 0.0));
    out.gearDown = isHold(a.gearDown) ? last_.gearDown : (a.gearDown >= 0.5 ? 1.0 : 0.0);
    out.brakeLeft = clamp01(orHold(a.brakeLeft, 0.0));
    out.brakeRight = clamp01(orHold(a.brakeRight, 0.0));
    last_ = out;

    // effectors at their travel limits
    auto& flags = report_->axisFlags;
    if (std::abs(out.aileron) >= 1.0) flags[static_cast<std::size_t>(Axis::Roll)] |= kSaturated;
    if (std::abs(out.elevator) >= 1.0) flags[static_cast<std::size_t>(Axis::Pitch)] |= kSaturated;
    if (std::abs(out.rudder) >= 1.0) flags[static_cast<std::size_t>(Axis::Yaw)] |= kSaturated;
    if (out.throttle[0] <= 0.0 || out.throttle[0] >= 1.0) flags[static_cast<std::size_t>(Axis::Thrust)] |= kSaturated;
}

void ControlStack::reset() {
    for (auto& c : controllers_)
        if (c) c->reset();
    for (std::size_t s = 0; s < kSlotCount; ++s) {
        if (behaviors_[s]) behaviors_[s]->reset();
        started_[s] = 0; // behaviours start again
    }
    derived_.fill(nullptr);
    last_ = initial_;
}

const char* reasonName(Reason reason) noexcept {
    switch (reason) {
    case Reason::None: return "none";
    case Reason::UnknownVehicle: return "unknown_vehicle";
    case Reason::UnknownCapability: return "unknown_capability";
    case Reason::UnknownActivity: return "unknown_activity";
    case Reason::Unavailable: return "unavailable";
    case Reason::VersionUnsupported: return "version_unsupported";
    case Reason::InvalidParameter: return "invalid_parameter";
    case Reason::OutOfRange: return "out_of_range";
    case Reason::InvalidAxes: return "invalid_axes";
    case Reason::AuthorityHeld: return "authority_held";
    case Reason::ControllerNotAxisAware: return "controller_not_axis_aware";
    case Reason::ActivityEnded: return "activity_ended";
    case Reason::NotUpdatable: return "not_updatable";
    case Reason::WrongCommandType: return "wrong_command_type";
    case Reason::GoalReached: return "goal_reached";
    case Reason::Requested: return "requested";
    case Reason::Preempted: return "preempted";
    case Reason::TargetLost: return "target_lost";
    case Reason::BehaviorFailed: return "behavior_failed";
    case Reason::CapabilityLost: return "capability_lost";
    case Reason::Diverged: return "diverged";
    default: return "?";
    }
}

} // namespace fsim::control
