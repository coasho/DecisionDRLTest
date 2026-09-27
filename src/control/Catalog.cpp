#include "control/Catalog.h"

#include "control/Adapter.h"
#include "control/Registry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.14159265358979323846;

ParameterInfo parameter(const char* name, const char* unit, double def, double lo = -kInf, double hi = kInf, bool optional = true) {
    return ParameterInfo{name, unit, lo, hi, def, optional};
}

/// A parameter whose range is the aircraft's, and the limits a value beyond it breaks.
ParameterInfo limited(const char* name, const char* unit, double def, double lo, double hi, Constraint below, Constraint above, bool optional = true) {
    return ParameterInfo{name, unit, lo, hi, def, optional, below, above};
}

CapabilityDescriptor flight(const char* name, Level level, std::vector<ParameterInfo> parameters, std::vector<std::string> uses) {
    CapabilityDescriptor d;
    d.id = std::string("fsim.flight.") + name;
    d.kind = CapabilityKind::Flight;
    d.interactions = kCommand | kUpdate | kCancel | kStatus;
    d.level = level;
    d.axes = kPrimaryAxes;
    d.axisGroups = level == Level::Actuator ? kGroupEachAxis | kGroupLateral | kGroupPitch | kGroupThrust : kGroupLateral | kGroupPitch | kGroupThrust;
    d.persistence = Persistence::Persistent;
    d.parameters = std::move(parameters);
    d.uses = std::move(uses);
    return d;
}

/// The platform's own behaviours are fsim.guidance.<id>; others user.guidance.<id>
/// unless registered with a dotted id.
std::string guidanceId(const std::string& behavior) {
    static const char* const builtin[] = {"hold", "waypoints", "loiter", "pursuit", "evade", "formation", "aerobatics", "hover"};
    if (behavior.find('.') != std::string::npos) return behavior;
    for (const char* b : builtin)
        if (behavior == b) return "fsim.guidance." + behavior;
    return "user.guidance." + behavior;
}

/// A route's point (BehaviorCommand::points): somewhere on the Earth, at an
/// altitude, reached within a positive capture radius; any airspeed or
/// heading it gives a number.
bool validPoint(const PositionCommand& p) noexcept {
    auto optional = [](double v) { return isHold(v) || std::isfinite(v); };
    return std::isfinite(p.latitudeRad) && std::abs(p.latitudeRad) <= kPi / 2 && std::isfinite(p.longitudeRad) && std::isfinite(p.altitudeMslM) &&
           std::isfinite(p.captureRadiusM) && p.captureRadiusM > 0.0 && optional(p.airspeedMs) && !(p.airspeedMs < 0.0) && optional(p.headingRad);
}

/// Clamp or reject one value against its parameter; `detail` says which
/// field (`field`) and what limit its value broke, for a rejection or the
/// first clamp.
Reason checkValue(const ParameterInfo& p, double& v, RangePolicy range, std::uint16_t& flags, CommandResult& detail, std::size_t field) noexcept {
    auto about = [&](Reason r) {
        detail.index = static_cast<std::int16_t>(field);
        detail.constraint = v < p.min ? p.below : v > p.max ? p.above : Constraint::None;
        return r;
    };
    if (isHold(v)) return p.optional ? Reason::None : about(Reason::InvalidParameter);
    if (!std::isfinite(v)) return about(Reason::InvalidParameter);
    if (!p.supported) return v == p.defaultValue ? Reason::None : about(Reason::InvalidParameter); // nothing on this aircraft moves it
    if (v >= p.min && v <= p.max) return Reason::None;
    if (range == RangePolicy::Reject) return about(Reason::OutOfRange);
    if (!(flags & kClamped)) about(Reason::None); // the first value clamped
    v = std::clamp(v, p.min, p.max);
    flags |= kClamped;
    return Reason::None;
}

} // namespace

std::size_t commandFields(Command& c, double* f[8]) noexcept {
    if (auto* a = std::get_if<ActuatorCommand>(&c)) {
        f[0] = &a->aileron, f[1] = &a->elevator, f[2] = &a->rudder, f[3] = &a->throttle;
        f[4] = &a->flaps, f[5] = &a->gearDown, f[6] = &a->brakeLeft, f[7] = &a->brakeRight;
        return 8;
    }
    if (auto* a = std::get_if<AttitudeCommand>(&c)) {
        f[0] = &a->rollRad, f[1] = &a->pitchRad, f[2] = &a->headingRad, f[3] = &a->maxBankRad, f[4] = &a->throttle, f[5] = &a->airspeedMs;
        return 6;
    }
    if (auto* a = std::get_if<AccelerationCommand>(&c)) {
        f[0] = &a->loadFactorG, f[1] = &a->rollRateRadS, f[2] = &a->longitudinalMs2, f[3] = &a->throttle;
        f[4] = &a->pitchRateRadS, f[5] = &a->yawRateRadS;
        return 6;
    }
    if (auto* v = std::get_if<VelocityCommand>(&c)) {
        f[0] = &v->airspeedMs, f[1] = &v->verticalSpeedMs, f[2] = &v->headingRad, f[3] = &v->turnRateRadS;
        f[4] = &v->northMs, f[5] = &v->eastMs;
        return 6;
    }
    if (auto* p = std::get_if<PositionCommand>(&c)) {
        f[0] = &p->latitudeRad, f[1] = &p->longitudeRad, f[2] = &p->altitudeMslM, f[3] = &p->airspeedMs, f[4] = &p->captureRadiusM;
        f[5] = &p->headingRad;
        return 6;
    }
    return 0;
}

std::size_t legacyFieldCount(Level level) noexcept {
    switch (level) {
    case Level::Actuator: return 8;
    case Level::Attitude: return 6;
    case Level::Acceleration: return 4;
    case Level::Velocity: return 4;
    case Level::Position: return 5;
    default: return 0;
    }
}

const char* supportCapability(std::size_t alternative) noexcept {
    static const char* const ids[] = {"fsim.support.gear",       "fsim.support.flaps",      "fsim.support.wheel_brakes",
                                      "fsim.support.speedbrake", "fsim.support.pitch_trim", "fsim.flight.engines"};
    return alternative < kSupportKinds ? ids[alternative] : "";
}

Axis supportAxis(const SupportCommand& c) noexcept {
    static const Axis axes[] = {Axis::Gear, Axis::Flaps, Axis::Brakes, Axis::Speedbrake, Axis::PitchTrim, Axis::Thrust};
    return axes[c.index()];
}

void supportValues(const SupportCommand& c, double& value, double& value2) noexcept {
    value2 = kHold;
    if (const auto* g = std::get_if<GearCommand>(&c)) value = g->down;
    else if (const auto* f = std::get_if<FlapsCommand>(&c)) value = f->position;
    else if (const auto* b = std::get_if<WheelBrakesCommand>(&c)) value = b->left, value2 = b->right;
    else if (const auto* s = std::get_if<SpeedbrakeCommand>(&c)) value = s->position;
    else if (const auto* t = std::get_if<PitchTrimCommand>(&c)) value = t->position;
}

double supportGoal(const SupportCommand& c) noexcept {
    if (const auto* g = std::get_if<GearCommand>(&c)) return g->down >= 0.5 ? 1.0 : 0.0;
    if (const auto* f = std::get_if<FlapsCommand>(&c)) return f->position;
    return kHold;
}

std::size_t supportFields(SupportCommand& c, double* f[4]) noexcept {
    if (auto* e = std::get_if<EnginesCommand>(&c)) return f[0] = &e->throttle[0], f[1] = &e->throttle[1], f[2] = &e->throttle[2], f[3] = &e->throttle[3], 4;
    if (auto* g = std::get_if<GearCommand>(&c)) return f[0] = &g->down, 1;
    if (auto* p = std::get_if<FlapsCommand>(&c)) return f[0] = &p->position, 1;
    if (auto* b = std::get_if<WheelBrakesCommand>(&c)) return f[0] = &b->left, f[1] = &b->right, 2;
    if (auto* s = std::get_if<SpeedbrakeCommand>(&c)) return f[0] = &s->position, 1;
    if (auto* t = std::get_if<PitchTrimCommand>(&c)) return f[0] = &t->position, 1;
    return 0;
}

CapabilityCatalog::CapabilityCatalog() : CapabilityCatalog(~0u) {}

CapabilityCatalog::CapabilityCatalog(std::uint32_t features) : features_(features) {
    // Until a profile narrows them (step 2), the ranges are the loops' own
    // bounds: no command the existing entry points send changes.
    const double nan = kHold;
    descriptors_.push_back(flight("actuator", Level::Actuator,
                                  {parameter("aileron", "", 0.0, -1.0, 1.0), parameter("elevator", "", 0.0, -1.0, 1.0),
                                   parameter("rudder", "", 0.0, -1.0, 1.0), parameter("throttle", "", 0.0, 0.0, 1.0),
                                   parameter("flaps", "", 0.0, 0.0, 1.0), parameter("gear_down", "", nan, 0.0, 1.0),
                                   parameter("brake_left", "", 0.0, 0.0, 1.0), parameter("brake_right", "", 0.0, 0.0, 1.0)},
                                  {}));
    // (a range the aircraft's envelope narrows says what limit a value beyond it breaks: a rejection's detail)
    using C = Constraint;
    const C orientation = C::MaxOrientation, rate = C::MaxOrientationRate;
    descriptors_.push_back(flight("attitude", Level::Attitude,
                                  {limited("roll_rad", "rad", 0.0, -kPi, kPi, orientation, orientation),
                                   limited("pitch_rad", "rad", 0.0, -kPi / 2, kPi / 2, orientation, orientation), parameter("heading_rad", "rad", nan),
                                   limited("max_bank_rad", "rad", 0.785, 0.0, kPi / 2, C::None, orientation), parameter("throttle", "", nan, 0.0, 1.0),
                                   limited("airspeed_ms", "m/s", nan, 0.0, kInf, C::MinAirspeed, C::MaxAirspeed)},
                                  {"fsim.flight.actuator"}));
    descriptors_.push_back(flight("acceleration", Level::Acceleration,
                                  {limited("load_factor_g", "g", 1.0, -kInf, kInf, C::MinAcceleration, C::MaxAcceleration),
                                   limited("roll_rate_rad_s", "rad/s", 0.0, -kInf, kInf, rate, rate),
                                   limited("longitudinal_ms2", "m/s2", nan, -kInf, kInf, C::MinAcceleration, C::MaxAcceleration),
                                   parameter("throttle", "", nan, 0.0, 1.0), limited("pitch_rate_rad_s", "rad/s", nan, -kInf, kInf, rate, rate),
                                   limited("yaw_rate_rad_s", "rad/s", nan, -kInf, kInf, rate, rate)},
                                  {"fsim.flight.actuator"}));
    descriptors_.push_back(flight("velocity", Level::Velocity,
                                  {limited("airspeed_ms", "m/s", nan, 0.0, kInf, C::MinAirspeed, C::MaxAirspeed),
                                   limited("vertical_speed_ms", "m/s", 0.0, -kInf, kInf, C::MaxDescentRate, C::MaxClimbRate),
                                   parameter("heading_rad", "rad", nan), limited("turn_rate_rad_s", "rad/s", nan, -kInf, kInf, C::MaxTurnRate, C::MaxTurnRate),
                                   parameter("north_ms", "m/s", nan), parameter("east_ms", "m/s", nan)},
                                  {"fsim.flight.attitude"}));
    descriptors_.push_back(flight("position", Level::Position,
                                  {parameter("latitude_rad", "rad", nan, -kPi / 2, kPi / 2, false), parameter("longitude_rad", "rad", nan, -kInf, kInf, false),
                                   limited("altitude_msl_m", "m", nan, -kInf, kInf, C::MinAltitude, C::MaxAltitude, false),
                                   limited("airspeed_ms", "m/s", nan, 0.0, kInf, C::MinAirspeed, C::MaxAirspeed),
                                   parameter("capture_radius_m", "m", 200.0, 0.0), parameter("heading_rad", "rad", nan)},
                                  {"fsim.flight.velocity"}));
    for (std::size_t l = 0; l < byLevel_.size(); ++l) byLevel_[l] = static_cast<int>(l);
    addBehaviors();
}

CapabilityCatalog::CapabilityCatalog(const VehicleProfile& profile, const VehicleAdapter& adapter) : CapabilityCatalog(adapter.features()) {
    adapter.declare(profile, *this);
}

ParameterInfo* CapabilityCatalog::parameterOf(std::string_view capability, std::string_view parameter) noexcept {
    const int index = find(capability);
    if (index < 0) return nullptr;
    for (auto& p : descriptors_[static_cast<std::size_t>(index)].parameters)
        if (p.name == parameter) return &p;
    return nullptr;
}

void CapabilityCatalog::rename(std::string_view capability, std::string_view parameter, std::string name) {
    if (ParameterInfo* p = parameterOf(capability, parameter)) p->name = std::move(name);
}

void CapabilityCatalog::unsupport(std::string_view capability, std::string_view parameter) {
    if (ParameterInfo* p = parameterOf(capability, parameter)) p->supported = false;
}

void CapabilityCatalog::setAxisGroups(std::uint8_t groups) {
    for (std::size_t l = 0; l < byLevel_.size(); ++l) { // the five flight capabilities, by level
        CapabilityDescriptor& d = descriptors_[static_cast<std::size_t>(byLevel_[l])];
        d.axisGroups = d.level == Level::Actuator ? static_cast<std::uint8_t>(kGroupEachAxis | groups) : groups;
    }
}

void CapabilityCatalog::narrow(std::string_view capability, std::string_view parameter, double lo, double hi) {
    const int index = find(capability);
    if (index < 0) return;
    for (auto& p : descriptors_[static_cast<std::size_t>(index)].parameters)
        if (p.name == parameter && p.supported) { // (one the aircraft has nothing for keeps its default)
            if (!std::isnan(lo)) p.min = std::max(p.min, lo);
            if (!std::isnan(hi)) p.max = std::min(p.max, hi);
            if (!std::isnan(p.defaultValue)) p.defaultValue = std::clamp(p.defaultValue, p.min, std::max(p.min, p.max));
        }
}

void CapabilityCatalog::addSupport(std::size_t alternative, int engines, AxisMask owns) {
    if (alternative >= kSupportKinds || bySupport_[alternative] >= 0) return;
    static const Axis axes[] = {Axis::Gear, Axis::Flaps, Axis::Brakes, Axis::Speedbrake, Axis::PitchTrim, Axis::Thrust};
    CapabilityDescriptor d;
    d.id = supportCapability(alternative);
    d.kind = alternative == 5 ? CapabilityKind::Flight : CapabilityKind::Support; // the engines' throttles own thrust
    d.interactions = kCommand | kUpdate | kCancel | kStatus;
    d.level = Level::Actuator;
    d.axes = owns ? owns : axisBit(axes[alternative]);
    d.persistence = alternative <= 1 ? Persistence::Terminating : Persistence::Persistent; // gear and flaps get somewhere
    switch (alternative) {
    case 0: d.parameters = {parameter("down", "", 1.0, 0.0, 1.0, false)}; break;
    case 1: d.parameters = {parameter("position", "", 0.0, 0.0, 1.0, false)}; break;
    case 2: d.parameters = {parameter("left", "", 0.0, 0.0, 1.0, false), parameter("right", "", 0.0, 0.0, 1.0, false)}; break;
    case 3: d.parameters = {parameter("position", "", 0.0, 0.0, 1.0, false)}; break;
    case 4: d.parameters = {parameter("position", "", 0.0, -1.0, 1.0, false)}; break;
    default: {
        static const char* const names[] = {"throttle_1", "throttle_2", "throttle_3", "throttle_4"};
        for (int i = 0; i < std::clamp(engines, 1, 4); ++i) d.parameters.push_back(parameter(names[i], "", kHold, 0.0, 1.0));
        d.axisGroups = d.axes == axisBit(Axis::Thrust) ? kGroupThrust : 0; // engines that fly every axis: all of them or none
        break;
    }
    }
    bySupport_[alternative] = static_cast<int>(descriptors_.size());
    descriptors_.push_back(std::move(d));
}

void CapabilityCatalog::addProtection() {
    if (find("fsim.envelope.protection") >= 0) return;
    CapabilityDescriptor d;
    d.id = "fsim.envelope.protection";
    d.kind = CapabilityKind::Status;
    d.interactions = kSettings | kStatus; // a mode to set, a status to read; nothing to command
    d.level = Level::Actuator;
    d.axes = 0;                           // it owns no axis: it limits what the owners demand
    d.parameters = {parameter("mode", "", 2.0, 0.0, 2.0, false)}; // 0 off, 1 report, 2 limit
    descriptors_.push_back(std::move(d));
}

void CapabilityCatalog::addBehaviors() {
    auto& registry = ControllerRegistry::instance();
    registryRevision_ = registry.revision();
    for (auto& [behavior, traits] : registry.behaviors()) {
        const bool known = std::any_of(descriptors_.begin(), descriptors_.end(), [&, &b = behavior](const CapabilityDescriptor& d) { return d.behavior == b; });
        if (known || (traits.features & ~features_)) continue; // offered, or not for this aircraft (a hover for a wing)
        CapabilityDescriptor d;
        d.id = guidanceId(behavior);
        d.kind = CapabilityKind::Guidance;
        d.interactions = kCommand | kCancel | kStatus; // parameters are heap data: a new target is a NEW
        d.level = Level::Behavior;
        d.axes = kPrimaryAxes;
        d.persistence = traits.persistence;
        d.parameters = traits.parameters;
        d.uses = traits.uses;
        d.behavior = behavior;
        d.needsTarget = traits.needsTarget;
        d.mode = traits.mode;
        descriptors_.push_back(std::move(d));
    }
}

bool CapabilityCatalog::refresh() {
    if (ControllerRegistry::instance().revision() == registryRevision_) return false;
    const std::size_t before = descriptors_.size();
    addBehaviors();
    return descriptors_.size() != before;
}

int CapabilityCatalog::indexOf(const Command& command) const noexcept {
    if (const auto* b = std::get_if<BehaviorCommand>(&command)) return find(b->id);
    return byLevel_[command.index()];
}

int CapabilityCatalog::find(std::string_view id) const noexcept {
    for (std::size_t i = 0; i < descriptors_.size(); ++i) {
        const auto& d = descriptors_[i];
        if (d.id == id || (!d.behavior.empty() && d.behavior == id)) return static_cast<int>(i);
    }
    return -1;
}

AxisMask CapabilityCatalog::defaultAxes(std::size_t index, const Command& command) const noexcept {
    AxisMask axes = descriptors_[index].axes;
    if (const auto* a = std::get_if<ActuatorCommand>(&command)) {
        // an actuator command owns the support effectors it sets
        if (!isHold(a->flaps)) axes |= axisBit(Axis::Flaps);
        if (!isHold(a->gearDown)) axes |= axisBit(Axis::Gear);
        if (!isHold(a->brakeLeft) || !isHold(a->brakeRight)) axes |= axisBit(Axis::Brakes);
    }
    return axes;
}

Reason CapabilityCatalog::check(std::size_t index, SupportCommand& command, RangePolicy range, std::uint16_t& flags, CommandResult& detail) const noexcept {
    const CapabilityDescriptor& d = descriptors_[index];
    double* fields[4];
    const std::size_t n = std::min(supportFields(command, fields), d.parameters.size());
    for (std::size_t i = 0; i < n; ++i)
        if (const Reason r = checkValue(d.parameters[i], *fields[i], range, flags, detail, i); r != Reason::None) return r;
    return Reason::None;
}

Reason CapabilityCatalog::check(std::size_t index, Command& command, RangePolicy range, std::uint16_t& flags, CommandResult& detail) const noexcept {
    const CapabilityDescriptor& d = descriptors_[index];
    if (auto* b = std::get_if<BehaviorCommand>(&command)) {
        if (d.needsTarget && b->target == 0) return Reason::InvalidParameter;
        for (auto& [key, value] : b->params)
            for (std::size_t i = 0; i < d.parameters.size(); ++i)
                if (d.parameters[i].name == key)
                    if (const Reason r = checkValue(d.parameters[i], value, range, flags, detail, i); r != Reason::None) return r;
        for (std::size_t i = 0; i < b->points.size(); ++i)
            if (!validPoint(b->points[i])) {
                detail.index = static_cast<std::int16_t>(std::min<std::size_t>(i, 0x7FFF)); // the route's point
                return Reason::InvalidParameter;
            }
        return Reason::None;
    }
    double* fields[8];
    const std::size_t n = std::min(commandFields(command, fields), d.parameters.size());
    for (std::size_t i = 0; i < n; ++i)
        if (const Reason r = checkValue(d.parameters[i], *fields[i], range, flags, detail, i); r != Reason::None) return r;
    return Reason::None;
}

} // namespace fsim::control
