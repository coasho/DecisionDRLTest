#include "control/ControlStack.h"

#include "control/Adapter.h"
#include "control/Protection.h"
#include "control/Registry.h"
#include "control/Runtime.h"
#include "core/Log.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

#if defined(__GNUC__) || defined(__clang__)
#define FSIM_ALWAYS_INLINE inline __attribute__((always_inline))
#elif defined(_MSC_VER)
#define FSIM_ALWAYS_INLINE __forceinline
#else
#define FSIM_ALWAYS_INLINE inline
#endif

namespace {

/// The index of the lowest set bit of m (m != 0).
inline unsigned lowestBit(unsigned m) noexcept {
#if defined(__GNUC__) || defined(__clang__)
    return static_cast<unsigned>(__builtin_ctz(m));
#else
    unsigned i = 0;
    while (!(m & 1u)) m >>= 1, ++i;
    return i;
#endif
}

/// What every vehicle flies before its first command, and what flies an axis
/// nobody owns: surfaces centred, throttle 0, flaps up, brakes off, gear held.
const Command kNeutral = ActuatorCommand{};

/// A command at `level` with every field kHold: the start of a merge.
Command blankCommand(Level level) noexcept {
    switch (level) {
    case Level::Attitude: return AttitudeCommand{kHold, kHold, kHold, kHold, kHold, kHold};
    case Level::Acceleration: return AccelerationCommand{kHold, kHold, kHold, kHold};
    case Level::Velocity: return VelocityCommand{kHold, kHold, kHold, kHold};
    case Level::Position: return PositionCommand{kHold, kHold, kHold, kHold, kHold};
    default: return ActuatorCommand{};
    }
}

/// Copy the fields of `axis` (docs/control-architecture.md, 9.5) from `src`
/// into `dst`, two commands of the same level above Actuator. Above the
/// actuators yaw goes with roll, so its fields are roll's.
void copyAxisFields(Command& dst, const Command& src, Axis axis) noexcept {
    const bool lateral = axis == Axis::Roll, pitch = axis == Axis::Pitch, thrust = axis == Axis::Thrust;
    if (auto* d = std::get_if<AttitudeCommand>(&dst)) {
        const auto* s = std::get_if<AttitudeCommand>(&src);
        if (!s) return;
        if (lateral) d->rollRad = s->rollRad, d->headingRad = s->headingRad, d->maxBankRad = s->maxBankRad;
        if (pitch) d->pitchRad = s->pitchRad;
        if (thrust) d->throttle = s->throttle, d->airspeedMs = s->airspeedMs;
    } else if (auto* d2 = std::get_if<AccelerationCommand>(&dst)) {
        const auto* s = std::get_if<AccelerationCommand>(&src);
        if (!s) return;
        if (lateral) d2->rollRateRadS = s->rollRateRadS;
        if (pitch) d2->loadFactorG = s->loadFactorG;
        if (thrust) d2->longitudinalMs2 = s->longitudinalMs2, d2->throttle = s->throttle;
    } else if (auto* d3 = std::get_if<VelocityCommand>(&dst)) {
        const auto* s = std::get_if<VelocityCommand>(&src);
        if (!s) return;
        if (lateral) d3->headingRad = s->headingRad, d3->turnRateRadS = s->turnRateRadS;
        if (pitch) d3->verticalSpeedMs = s->verticalSpeedMs;
        if (thrust) d3->airspeedMs = s->airspeedMs;
    } else if (auto* d4 = std::get_if<PositionCommand>(&dst)) {
        const auto* s = std::get_if<PositionCommand>(&src);
        if (!s) return;
        if (lateral) d4->latitudeRad = s->latitudeRad, d4->longitudeRad = s->longitudeRad, d4->captureRadiusM = s->captureRadiusM;
        if (pitch) d4->altitudeMslM = s->altitudeMslM;
        if (thrust) d4->airspeedMs = s->airspeedMs;
    }
}

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

ControlStack::ControlStack()
    : config_(std::make_unique<RuntimeConfig>()), report_(std::make_unique<RuntimeReport>()), adapter_(&adapterFor(ControlFamily::Stock)) {
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

std::size_t ControlStack::wholeSlot() const noexcept {
    const auto& owner = config_->owner;
    const std::uint8_t o = owner[static_cast<std::size_t>(Axis::Roll)];
    if (o >= kSlotCount) return kNoSlot;
    for (Axis a : {Axis::Pitch, Axis::Yaw, Axis::Thrust})
        if (owner[static_cast<std::size_t>(a)] != o) return kNoSlot;
    return o;
}

std::size_t ControlStack::topSlot() const noexcept {
    std::size_t top = kNoSlot;
    for (std::size_t s = 0; s < kSlotCount; ++s)
        if ((config_->slots[s].axes & kPrimaryAxes) && (top == kNoSlot || levelRank(config_->slots[s].level) > levelRank(config_->slots[top].level)))
            top = s;
    return top;
}

bool ControlStack::holdsDefault() const noexcept {
    if (config_->vehicleDefault != VehicleDefault::Hold) return false;
    for (std::size_t a = 0; a < kPrimaryAxisCount; ++a)
        if (config_->owner[a] == RuntimeConfig::kNone) return true;
    return false;
}

Level ControlStack::activeLevel() const noexcept {
    const std::size_t s = topSlot();
    if (s != kNoSlot) return config_->slots[s].level;
    return holdsDefault() ? Level::Velocity : Level::Actuator;
}

const Command* ControlStack::activeCommand() const noexcept {
    const std::size_t s = topSlot();
    if (s != kNoSlot) return &config_->slots[s].command;
    return holdsDefault() ? &hold_ : &kNeutral;
}

const Behavior* ControlStack::behavior() const noexcept {
    const std::size_t s = topSlot();
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
    ++report_->updates;
    derived_.fill(nullptr);
    const std::size_t s = wholeSlot();
    if (s == kNoSlot) return flyGeneral(ctx, out);
    if (config_->protection.mode == ProtectionMode::Off) return cascade<false>(ctx, s, out);
    const Level entry = config_->slots[s].level; // the acceleration loop's setpoints need nothing from the state; the actuators' limiter reads it itself
    selectLimits(ctx, entry != Level::Acceleration && entry != Level::Actuator);
    cascade<true>(ctx, s, out);
}

template <bool Protected>
FSIM_ALWAYS_INLINE void ControlStack::cascade(const ControlContext& ctx, std::size_t s, sim::ControlInputs& out) {
    RuntimeReport& report = *report_;
    const SetpointSlot& slot = config_->slots[s];
    SlotReport& flown = report.slots[s];
    flown.generation = slot.generation;
    flown.revision = slot.revision;
    // Protection (docs/control-architecture.md, 11.3): each setpoint limited
    // before its level flies it, as the loops see the state.
    [[maybe_unused]] const Protection& protection = config_->protection;
    [[maybe_unused]] const bool limiting = Protected && protection.mode == ProtectionMode::Limit;
    [[maybe_unused]] LimitMask limited = 0;
    [[maybe_unused]] const LimitState here{&ctx.sensed, tasPerCas_, tasPerMach_, bankCos_};
    auto limit = [&](Command& setpoint) {
        if constexpr (Protected)
            if (limiting) limited = static_cast<LimitMask>(limited | limitSetpoint(setpoint, *active_, protection, here));
    };

    // The engaged command is read where the host keeps it, never copied -
    // unless protection limits it: then a copy, the host's own left as it was.
    const Command* current = &slot.command;
    Level level = slot.level;
    if constexpr (Protected) {
        if (limiting && level != Level::Behavior && level != Level::Actuator) {
            Command& entry = merged_[static_cast<std::size_t>(level)];
            if (entry.index() == current->index()) assignSetpoint(entry, *current); // a struct copy, no variant machinery
            else entry = *current;
            limit(entry);
            current = &entry;
        }
    }
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
        if (const Reason failure = behavior->failure(); failure != Reason::None) {
            flown.events |= kFailed;
            flown.failure = failure;
        }
        const Level nextLevel = levelOf(next);
        if (levelRank(nextLevel) >= levelRank(level)) {
            LOG_ERROR("control") << "behaviour '" << behavior->id() << "' returned a command at level " << levelName(nextLevel);
            return fail(out);
        }
        const auto n = static_cast<std::size_t>(nextLevel);
        auto& produced = outputs_[static_cast<std::size_t>(level)];
        produced = std::move(next);
        if (nextLevel != Level::Actuator) limit(produced);
        derived_[n] = &produced;
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
        if (levelRank(nextLevel) >= levelRank(level)) {
            LOG_ERROR("control") << "controller '" << c->id() << "' returned a command at level " << levelName(nextLevel)
                                 << " (must be lower than " << levelName(level) << ")";
            return fail(out);
        }
        const auto n = static_cast<std::size_t>(nextLevel);
        auto& produced = outputs_[static_cast<std::size_t>(level)];
        produced = std::move(next);
        if (nextLevel != Level::Actuator) limit(produced);
        derived_[n] = &produced;
        level = nextLevel;
        current = derived_[n];
    }
    if constexpr (Protected) {
        // An elevator commanded as such: the feedback limiter on a surface-controlled aircraft.
        if (limiting && slot.level == Level::Actuator) {
            Command& limitedCommand = merged_[static_cast<std::size_t>(Level::Actuator)];
            if (limitedCommand.index() == current->index()) assignSetpoint(limitedCommand, *current);
            else limitedCommand = *current;
            auto& a = std::get<ActuatorCommand>(limitedCommand);
            limited = static_cast<LimitMask>(limited | limitElevator(a.elevator, *active_, protection, ctx.sensed));
            derived_[static_cast<std::size_t>(Level::Actuator)] = &limitedCommand;
            actuate(a, out);
        } else {
            actuate(std::get<ActuatorCommand>(*current), out);
        }
        watch(ctx, limited);
    } else {
        actuate(std::get<ActuatorCommand>(*current), out);
    }
}

void ControlStack::flyNeutral(sim::ControlInputs& out) {
    derived_[static_cast<std::size_t>(Level::Actuator)] = &kNeutral;
    actuate(std::get<ActuatorCommand>(kNeutral), out);
}

void ControlStack::flyGeneral(const ControlContext& ctx, sim::ControlInputs& out) {
    const bool protecting = config_->protection.mode != ProtectionMode::Off;
    if (protecting) selectLimits(ctx, true);
    limited_ = 0;
    if (topSlot() != kNoSlot || holdsDefault()) flyMerged(ctx, out); // the protection stage inside: limited_
    else flyNeutral(out);                                            // the neutral command: nothing to limit
    if (protecting) watch(ctx, limited_);
}

void ControlStack::selectLimits(const ControlContext& ctx, bool state) noexcept {
    // the configuration the aircraft flew into this update with: the flaps it was given, its gear
    active_ = &activeLimits(config_->protection, last_.flaps, ctx.state.gearPosition, geared_);
    if (!state || config_->protection.mode != ProtectionMode::Limit) return;
    const LimitState here = limitState(ctx.sensed, *active_, config_->protection); // as the loops see it
    tasPerCas_ = here.tasPerCas, tasPerMach_ = here.tasPerMach, bankCos_ = here.bankCos;
}

void ControlStack::watch(const ControlContext& ctx, LimitMask limited) noexcept {
    RuntimeReport& report = *report_;
    for (unsigned m = limited; m; m &= m - 1) { // each set bit
        const auto l = static_cast<std::size_t>(lowestBit(m));
        ++report.limits[l].limitedUpdates;
        report.axisFlags[static_cast<std::size_t>(limitAxis(static_cast<Limit>(l)))] |= kDemandLimited;
    }
    std::array<double, kLimitCount> excess;
    const LimitMask beyond = exceeded(*active_, ctx.state, excess); // the aircraft's truth, not what its sensors say
    for (unsigned m = beyond; m; m &= m - 1) {
        const auto l = static_cast<std::size_t>(lowestBit(m));
        LimitReport& r = report.limits[l];
        ++r.exceededUpdates;
        r.worstExcess = std::max(r.worstExcess, static_cast<float>(excess[l]));
        report.axisFlags[static_cast<std::size_t>(limitAxis(static_cast<Limit>(l)))] |= kExceeded;
    }
}

void ControlStack::flyMerged(const ControlContext& ctx, sim::ControlInputs& out) {
    const RuntimeConfig& c = *config_;
    RuntimeReport& report = *report_;
    constexpr std::size_t kPrimary = kPrimaryAxisCount;
    // where each primary axis's demand has got to, and the command that carries it
    std::array<Level, kPrimary> at{};
    std::array<const Command*, kPrimary> from{};
    AxisMask unowned = 0;
    for (std::size_t a = 0; a < kPrimary; ++a) {
        const std::uint8_t o = c.owner[a];
        if (o < kSlotCount && (c.slots[o].axes & (1u << a))) {
            at[a] = c.slots[o].level;
            from[a] = &c.slots[o].command;
        } else if (o == RuntimeConfig::kNone) {
            unowned = static_cast<AxisMask>(unowned | (1u << a));
        }
    }
    for (std::size_t s = 0; s < kSlotCount; ++s)
        if (c.slots[s].axes & kPrimaryAxes) {
            report.slots[s].generation = c.slots[s].generation;
            report.slots[s].revision = c.slots[s].revision;
        }

    // Protection (docs/control-architecture.md, 11.3): each level's setpoint to
    // the envelope before its loop flies it, as the loops see the state.
    const Protection& protection = c.protection;
    const bool limiting = protection.mode == ProtectionMode::Limit;
    const LimitState here{&ctx.sensed, tasPerCas_, tasPerMach_, bankCos_};
    LimitMask limited = 0;

    // The vehicle default's hold (VehicleDefault::Hold) enters at the velocity
    // level: the heading, true airspeed and height each group had when it was
    // let go, the height through the vertical speed as fsim.guidance.hold flies it.
    if (c.vehicleDefault == VehicleDefault::Hold && unowned) {
        const auto& s = ctx.sensed;
        AxisMask fresh = 0; // let go since its target was captured
        for (std::size_t a = 0; a < kPrimary; ++a)
            if ((unowned & (1u << a)) && (!(holdValid_ & (1u << a)) || captured_[a] != c.letGo[a])) {
                fresh = static_cast<AxisMask>(fresh | (1u << a));
                captured_[a] = c.letGo[a];
            }
        holdValid_ = static_cast<AxisMask>(holdValid_ | unowned);
        if (fresh & axisBit(Axis::Roll)) holdHeadingRad_ = s.eulerRad[2];
        if (fresh & axisBit(Axis::Pitch)) holdAltitudeM_ = s.altitudeMslM;
        if (fresh & axisBit(Axis::Thrust)) holdAirspeedMs_ = s.airspeedTrueMs;
        auto& hold = std::get<VelocityCommand>(hold_);
        hold.headingRad = holdHeadingRad_;
        hold.airspeedMs = holdAirspeedMs_;
        hold.verticalSpeedMs = std::clamp(0.25 * (holdAltitudeM_ - s.altitudeMslM), -6.0, 6.0);
        hold.turnRateRadS = kHold;
        for (std::size_t a = 0; a < kPrimary; ++a)
            if (unowned & (1u << a)) {
                at[a] = Level::Velocity;
                from[a] = &hold_;
            }
    }

    // A behaviour enters above the levels. Guidance takes every primary axis,
    // so at most one flies: here, the residual of one whose other axes were taken.
    for (std::size_t s = 0; s < kSlotCount; ++s) {
        const SetpointSlot& slot = c.slots[s];
        if (slot.level != Level::Behavior || !(slot.axes & kPrimaryAxes)) continue;
        Behavior* behavior = behaviors_[s].get();
        if (!behavior) return fail(out);
        if (started_[s] != slot.generation) {
            behavior->start(ctx, std::get<BehaviorCommand>(slot.command));
            started_[s] = slot.generation;
        }
        Command next = behavior->update(ctx, slot.command);
        SlotReport& flown = report.slots[s];
        if (behavior->finished()) flown.events |= kFinished;
        if (const Reason failure = behavior->failure(); failure != Reason::None) {
            flown.events |= kFailed;
            flown.failure = failure;
        }
        const Level nextLevel = levelOf(next);
        if (levelRank(nextLevel) >= levelRank(Level::Behavior)) {
            LOG_ERROR("control") << "behaviour '" << behavior->id() << "' returned a command at level " << levelName(nextLevel);
            return fail(out);
        }
        auto& produced = outputs_[static_cast<std::size_t>(Level::Behavior)];
        produced = std::move(next);
        derived_[static_cast<std::size_t>(Level::Behavior)] = &slot.command;
        for (std::size_t a = 0; a < kPrimary; ++a)
            if (from[a] == &slot.command) {
                at[a] = nextLevel;
                from[a] = &produced;
            }
        break;
    }

    for (const Level level : kCascadeOrder) { // from the top of the cascade down: a demand reaches each level before it runs
        const auto l = static_cast<std::size_t>(level);
        AxisMask engaged = 0;
        for (std::size_t a = 0; a < kPrimary; ++a)
            if (from[a] && at[a] == level) engaged = static_cast<AxisMask>(engaged | (1u << a));
        if (!engaged) continue;
        // one command at this level: each engaged axis's fields from where its demand came
        Command& merged = merged_[static_cast<std::size_t>(l)];
        merged = blankCommand(level);
        for (std::size_t a = 0; a < kPrimary; ++a)
            if (engaged & (1u << a)) copyAxisFields(merged, *from[a], static_cast<Axis>(a));
        if (limiting) limited = static_cast<LimitMask>(limited | limitSetpoint(merged, *active_, protection, here));
        derived_[static_cast<std::size_t>(l)] = &merged;
        Controller* controller = controllers_[static_cast<std::size_t>(l)].get();
        if (!controller) {
            LOG_ERROR("control") << "no controller at level " << levelName(level);
            return fail(out);
        }
        const ControlContext levelCtx{ctx.vehicleId, ctx.state, ctx.sensed, ctx.dt, ctx.world, ctx.rng, engaged};
        Command next = controller->update(levelCtx, merged);
        const Level nextLevel = levelOf(next);
        if (levelRank(nextLevel) >= levelRank(level)) {
            LOG_ERROR("control") << "controller '" << controller->id() << "' returned a command at level " << levelName(nextLevel)
                                 << " (must be lower than " << levelName(level) << ")";
            return fail(out);
        }
        auto& produced = outputs_[static_cast<std::size_t>(l)];
        produced = std::move(next);
        for (std::size_t a = 0; a < kPrimary; ++a)
            if (engaged & (1u << a)) {
                at[a] = nextLevel;
                from[a] = &produced;
            }
    }
    // The actuators: each axis's field from its demand, an axis nobody flies
    // the neutral default; a support effector a cascade slot owns, from that
    // slot's own chain.
    ActuatorCommand final = std::get<ActuatorCommand>(kNeutral);
    auto actuator = [&](std::size_t a) -> const ActuatorCommand* { return from[a] ? std::get_if<ActuatorCommand>(from[a]) : nullptr; };
    if (const auto* x = actuator(static_cast<std::size_t>(Axis::Roll))) final.aileron = x->aileron;
    if (const auto* x = actuator(static_cast<std::size_t>(Axis::Pitch))) final.elevator = x->elevator;
    if (const auto* x = actuator(static_cast<std::size_t>(Axis::Yaw))) final.rudder = x->rudder;
    if (const auto* x = actuator(static_cast<std::size_t>(Axis::Thrust))) final.throttle = x->throttle;
    auto chainOf = [&](Axis support) -> const ActuatorCommand* {
        const std::uint8_t o = c.owner[static_cast<std::size_t>(support)];
        if (o >= kSlotCount) return nullptr;
        for (std::size_t a = 0; a < kPrimary; ++a)
            if (c.owner[a] == o && from[a]) return actuator(a);
        return nullptr;
    };
    if (const auto* x = chainOf(Axis::Flaps)) final.flaps = x->flaps;
    if (const auto* x = chainOf(Axis::Gear)) final.gearDown = x->gearDown;
    if (const auto* x = chainOf(Axis::Brakes)) final.brakeLeft = x->brakeLeft, final.brakeRight = x->brakeRight;
    // An elevator commanded as such: the feedback limiter on a surface-controlled
    // aircraft. (Through the loops, their setpoints were limited above: a second
    // limiter after them would only fight their integrators.)
    const std::uint8_t pitchOwner = c.owner[static_cast<std::size_t>(Axis::Pitch)];
    if (limiting && pitchOwner < kSlotCount && c.slots[pitchOwner].level == Level::Actuator)
        limited = static_cast<LimitMask>(limited | limitElevator(final.elevator, *active_, protection, ctx.sensed));
    Command& assembled = merged_[static_cast<std::size_t>(Level::Actuator)];
    assembled = final;
    derived_[static_cast<std::size_t>(Level::Actuator)] = &assembled;
    actuate(final, out);
    limited_ = limited; // flyGeneral reports it, with the state's exceedances
}

void ControlStack::actuate(const ActuatorCommand& a, sim::ControlInputs& out) noexcept {
    adapter_->apply(a, last_, out);
    // A support axis flies its owner's demand: a support activity's, the
    // engaged command's (as set above), or nobody's - the neutral default:
    // flaps up, brakes off, the gear, speedbrake and trim left as they are.
    const RuntimeConfig& c = *config_;
    auto owner = [&c](Axis axis) { return c.owner[static_cast<std::size_t>(axis)]; };
    auto demand = [&c](Axis axis) -> const SupportDemand& {
        return c.support[static_cast<std::size_t>(axis) - static_cast<std::size_t>(Axis::Flaps)];
    };
    auto clamp01 = [](double v) { return std::clamp(v, 0.0, 1.0); };
    if (owner(Axis::Flaps) == RuntimeConfig::kSupport) out.flaps = clamp01(orHold(demand(Axis::Flaps).value, last_.flaps));
    else if (owner(Axis::Flaps) == RuntimeConfig::kNone) out.flaps = 0.0;
    if (owner(Axis::Gear) == RuntimeConfig::kSupport) {
        const double down = demand(Axis::Gear).value;
        out.gearDown = isHold(down) ? last_.gearDown : (down >= 0.5 ? 1.0 : 0.0);
    } else if (owner(Axis::Gear) == RuntimeConfig::kNone) {
        out.gearDown = last_.gearDown;
    }
    if (owner(Axis::Brakes) == RuntimeConfig::kSupport) {
        out.brakeLeft = clamp01(orHold(demand(Axis::Brakes).value, last_.brakeLeft));
        out.brakeRight = clamp01(orHold(demand(Axis::Brakes).value2, last_.brakeRight));
    } else if (owner(Axis::Brakes) == RuntimeConfig::kNone) {
        out.brakeLeft = out.brakeRight = 0.0;
    }
    if (owner(Axis::Thrust) == RuntimeConfig::kEngines)
        for (std::size_t i = 0; i < c.engines.size(); ++i) out.throttle[i] = clamp01(orHold(c.engines[i], last_.throttle[i]));
    effectors_.speedbrake = owner(Axis::Speedbrake) == RuntimeConfig::kSupport ? demand(Axis::Speedbrake).value : kHold;
    effectors_.pitchTrim = owner(Axis::PitchTrim) == RuntimeConfig::kSupport ? demand(Axis::PitchTrim).value : kHold;
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
    holdValid_ = 0; // the default's hold captures afresh
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

const char* activityStateName(ActivityState state) noexcept {
    switch (state) {
    case ActivityState::Pending: return "pending";
    case ActivityState::Active: return "active";
    case ActivityState::Completed: return "completed";
    case ActivityState::Failed: return "failed";
    case ActivityState::Canceled: return "canceled";
    default: return "?";
    }
}

} // namespace fsim::control
