#include "control/ControlStack.h"

#include "control/Registry.h"
#include "core/Log.h"

#include <algorithm>

namespace fsim::control {

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

ControlStack::ControlStack() {
    auto& registry = ControllerRegistry::instance();
    for (std::size_t l = 0; l < static_cast<std::size_t>(Level::Behavior); ++l) {
        const Level level = static_cast<Level>(l);
        if (const char* id = registry.defaultId(level)) controllers_[l] = registry.create(id);
    }
    active_ = ActuatorCommand{};
}

ControlStack::~ControlStack() = default;
ControlStack::ControlStack(ControlStack&&) noexcept = default;
ControlStack& ControlStack::operator=(ControlStack&&) noexcept = default;

void ControlStack::command(const Command& command) {
    if (const auto* b = std::get_if<BehaviorCommand>(&command)) {
        auto created = ControllerRegistry::instance().create(b->id);
        auto* behavior = dynamic_cast<Behavior*>(created.get());
        if (!behavior) {
            LOG_ERROR("control") << "unknown behaviour '" << b->id << "'; holding the current command";
            return;
        }
        created.release();
        behavior_.reset(behavior);
        behaviorStarted_ = false;
    } else {
        behavior_.reset();
    }
    active_ = command;
}

bool ControlStack::use(Level level, std::string_view controllerId) {
    auto& registry = ControllerRegistry::instance();
    auto c = registry.create(controllerId);
    if (!c) {
        LOG_ERROR("control") << "unknown controller '" << controllerId << "'";
        return false;
    }
    return use(level, std::move(c));
}

bool ControlStack::use(Level level, std::unique_ptr<Controller> controller) {
    if (!controller || controller->level() != level || level == Level::Behavior) {
        LOG_ERROR("control") << "controller '" << (controller ? controller->id() : "null") << "' does not accept level " << levelName(level);
        return false;
    }
    controllers_[static_cast<std::size_t>(level)] = std::move(controller);
    return true;
}

void ControlStack::update(const ControlContext& ctx, sim::ControlInputs& out) {
    for (auto& d : derived_) d.reset();
    if (!active_) {
        out = last_;
        return;
    }

    const Command* current = &*active_;
    Level level = levelOf(*current);
    derived_[static_cast<std::size_t>(level)] = *current;

    // Behaviours own their lifecycle; the rest of the cascade is stateless per level.
    if (level == Level::Behavior) {
        if (!behavior_) {
            out = last_;
            return;
        }
        if (!behaviorStarted_) {
            behavior_->start(ctx, std::get<BehaviorCommand>(*current));
            behaviorStarted_ = true;
        }
        Command next = behavior_->update(ctx, *current);
        const Level nextLevel = levelOf(next);
        if (nextLevel >= level) {
            LOG_ERROR("control") << "behaviour '" << behavior_->id() << "' returned a command at level " << levelName(nextLevel);
            out = last_;
            return;
        }
        derived_[static_cast<std::size_t>(nextLevel)] = std::move(next);
        level = nextLevel;
        current = &*derived_[static_cast<std::size_t>(level)];
    }

    while (level != Level::Actuator) {
        Controller* c = controllers_[static_cast<std::size_t>(level)].get();
        if (!c) {
            LOG_ERROR("control") << "no controller at level " << levelName(level);
            out = last_;
            return;
        }
        Command next = c->update(ctx, *current);
        const Level nextLevel = levelOf(next);
        if (nextLevel >= level) {
            LOG_ERROR("control") << "controller '" << c->id() << "' returned a command at level " << levelName(nextLevel)
                                 << " (must be lower than " << levelName(level) << ")";
            out = last_;
            return;
        }
        derived_[static_cast<std::size_t>(nextLevel)] = std::move(next);
        level = nextLevel;
        current = &*derived_[static_cast<std::size_t>(level)];
    }

    const auto& a = std::get<ActuatorCommand>(*current);
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
}

void ControlStack::reset() {
    for (auto& c : controllers_)
        if (c) c->reset();
    if (behavior_) {
        behavior_->reset();
        behaviorStarted_ = false;
    }
    for (auto& d : derived_) d.reset();
    last_ = initial_;
}

} // namespace fsim::control
