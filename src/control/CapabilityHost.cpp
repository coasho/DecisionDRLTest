#include "control/CapabilityHost.h"

#include "control/Atmosphere.h"
#include "control/Features.h"
#include "control/Protection.h"
#include "control/Registry.h"
#include "control/Route.h"
#include "core/Geodesy.h"
#include "core/Log.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

/// A level no higher than Position: a behaviour's output enters below it.
int cascadeTop(Level level) noexcept { return std::min(levelRank(level), levelRank(Level::Position)); }

/// What an activity takes from the runtime's flags on the axes it owns.
std::uint16_t flagsOn(const RuntimeReport& report, AxisMask axes) noexcept {
    std::uint16_t f = 0;
    for (std::size_t a = 0; a < kAxisCount; ++a) {
        if (!(axes & (1u << a))) continue;
        const std::uint16_t x = report.axisFlags[a];
        if (x & kSaturated) f |= kActivitySaturated;
        if (x & kDemandLimited) f |= kActivityDemandLimited;
        if (x & kExceeded) f |= kActivityExceeded;
    }
    return f;
}

/// Accepted; `made` for a NEW that made an activity (A-GRA's NewActivity).
CommandResult accepted(ActivityId activity, std::uint16_t flags = 0, bool made = false) noexcept {
    CommandResult r;
    r.status = CommandStatus::Accepted;
    r.activity = activity;
    r.flags = flags;
    r.newActivity = made;
    return r;
}

/// `r` with what the check said the answer is about (docs/vehicle-interface.md, 5.1).
CommandResult about(CommandResult r, const CommandResult& detail) noexcept {
    r.index = detail.index;
    r.constraint = detail.constraint;
    r.from = detail.from;
    r.to = detail.to;
    return r;
}

/// `value` against a limit of the aircraft's (above: a most, else a least):
/// nothing within it, or where either is not known; beyond it, held to it and
/// logged - clamped, or with Reject a PerformanceLimit finding - naming the
/// field or waypoint `index` (and a waypoint's `field`) and the limit.
void bound(double& value, double limit, bool above, std::int16_t index, Constraint constraint, CheckLog& log, std::int16_t field = -1) noexcept {
    if (std::isnan(limit) || isHold(value) || (above ? value <= limit : value >= limit)) return;
    log.limit(value, limit, index, field, constraint, Reason::PerformanceLimit);
}

bool aboveGround(double reference) noexcept { return reference == static_cast<double>(AltitudeReference::AboveGround); }

} // namespace

CapabilityHost::CapabilityHost() = default;
CapabilityHost::~CapabilityHost() = default;
CapabilityHost::CapabilityHost(CapabilityHost&&) noexcept = default;
CapabilityHost& CapabilityHost::operator=(CapabilityHost&&) noexcept = default;

void CapabilityHost::bind(std::uint32_t vehicle, ControlStack& runtime, const CapabilityCatalog& catalog, const VehicleAdapter& adapter,
                          const VehicleProfile& profile, double controlPeriodS) noexcept {
    vehicle_ = vehicle;
    runtime_ = &runtime;
    config_ = &runtime.config();
    catalog_ = &catalog;
    adapter_ = &adapter;
    profile_ = &profile;
    controlPeriodS_ = controlPeriodS;
    config_->protection = protectionFor(profile, adapter.features()); // Limit with an envelope section, else Off
    config_->tables = profile.tables.empty() ? nullptr : &profile.tables; // (what a speed optimisation flies)
    // what the aircraft can do, from its profile and the loops it flies with (docs/vehicle-interface.md, 7.1)
    performance_ = adapter.performance(profile, runtime);
    performance_.revision = config_->performance.revision + 1;
    config_->performance = performance_;
    loopsSeen_ = runtime.loopsRevision();
    ++config_->revision;
    if (authority_.size() < catalog.size()) authority_.resize(catalog.size());
}

namespace {

/// Every field the same (NaN the same as NaN), its revision aside.
bool samePerformance(const Performance& a, const Performance& b) noexcept {
    auto same = [](double x, double y) { return x == y || (std::isnan(x) && std::isnan(y)); };
    const double fa[] = {a.minCasMs, a.maxCasMs, a.maxMach, a.maxTasMs, a.cruiseTasMs, a.maxGroundSpeedMs, a.ceilingM, a.maxBankRad, a.minPitchRad,
                         a.maxPitchRad, a.maxRollRateRadS, a.minLoadFactor, a.maxLoadFactor, a.maxTiltRad, a.maxAccelerationMs2, a.maxDecelerationMs2,
                         a.maxClimbMs, a.maxDescentMs, a.altitudeGainPerS, a.headingGain, a.headingReferenceTasMs, a.bankRateRadS, a.velocityBandwidthRadS};
    const double fb[] = {b.minCasMs, b.maxCasMs, b.maxMach, b.maxTasMs, b.cruiseTasMs, b.maxGroundSpeedMs, b.ceilingM, b.maxBankRad, b.minPitchRad,
                         b.maxPitchRad, b.maxRollRateRadS, b.minLoadFactor, b.maxLoadFactor, b.maxTiltRad, b.maxAccelerationMs2, b.maxDecelerationMs2,
                         b.maxClimbMs, b.maxDescentMs, b.altitudeGainPerS, b.headingGain, b.headingReferenceTasMs, b.bankRateRadS, b.velocityBandwidthRadS};
    if (a.hovers != b.hovers) return false;
    for (std::size_t i = 0; i < std::size(fa); ++i)
        if (!same(fa[i], fb[i])) return false;
    return true;
}

} // namespace

void CapabilityHost::refreshPerformance() noexcept {
    if (!runtime_ || runtime_->loopsRevision() == loopsSeen_) return;
    loopsSeen_ = runtime_->loopsRevision();
    Performance next = adapter_->performance(*profile_, *runtime_);
    if (samePerformance(next, performance_)) return;
    next.revision = performance_.revision + 1;
    performance_ = next;
    config_->performance = performance_; // (guidance reads it there, from its next update)
    ++controlRevision_;
}

CapabilityHost::Authority& CapabilityHost::authorityOf(std::size_t capability) {
    if (authority_.size() < catalog_->size()) authority_.resize(catalog_->size());
    return authority_[capability];
}

Reason CapabilityHost::admits(std::size_t capability, Source source, ControllerId controller) const noexcept {
    if (source != Source::Policy) return Reason::None; // the platform's own: FA is always the primary controller
    const Authority* a = capability < authority_.size() ? &authority_[capability] : nullptr;
    if (controlMode_ == ControlMode::Granted && !(a && a->granted && a->holder == controller)) return Reason::NotGranted;
    if (a && a->restricted.availability != Availability::Available) return a->restricted.reason;
    return Reason::None;
}

Reason CapabilityHost::addresses(const ActivityRecord& record, Caller caller) const noexcept {
    if (controlMode_ != ControlMode::Granted) return Reason::None;
    if (caller.source < record.source) return Reason::AuthorityHeld;
    if (caller.source == Source::Policy && record.source == Source::Policy && caller.controller != record.controller) return Reason::AuthorityHeld;
    return Reason::None;
}

void CapabilityHost::endPolicy(std::size_t capability, Reason reason, const sim::VehicleState& state, double now, const ControllerId* only) noexcept {
    auto policy = [capability, only](const ActivityRecord& r) {
        return r.capability == capability && r.source == Source::Policy && (!only || r.controller == *only);
    };
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live && policy(records_[s])) {
            end(s, ActivityState::Canceled, reason, 0, now);
            release(s); // its axes: the vehicle default
        }
    if (waitingCount_)
        for (Waiting& w : *waiting_)
            if (w.used && !w.suggested && policy(w.record)) endWaiting(w, ActivityState::Canceled, reason, now);
    schedule(state, now); // (what waited for the axes may start)
}

void CapabilityHost::setControlMode(ControlMode mode, const sim::VehicleState& state, double now) noexcept {
    if (mode == controlMode_) return;
    controlMode_ = mode;
    ++controlRevision_;
    if (mode != ControlMode::Granted) return;
    auto ungranted = [this](const ActivityRecord& r) {
        return r.source == Source::Policy && admits(r.capability, Source::Policy, r.controller) == Reason::NotGranted;
    };
    for (std::size_t s = 0; s < kActivities; ++s) // what the policy flies without a grant ends, and what it waits to fly
        if (slots_[s].live && ungranted(records_[s])) {
            end(s, ActivityState::Canceled, Reason::NotGranted, 0, now);
            release(s);
        }
    if (waitingCount_)
        for (Waiting& w : *waiting_)
            if (w.used && !w.suggested && ungranted(w.record)) endWaiting(w, ActivityState::Canceled, Reason::NotGranted, now);
    schedule(state, now);
}

Reason CapabilityHost::requestControl(std::size_t capability, const sim::VehicleState& state, ControllerId controller) noexcept {
    if (capability >= catalog_->size() || !(catalog_->descriptor(capability).interactions & kCommand)) return Reason::UnknownCapability;
    Authority& a = authorityOf(capability);
    if (!a.allowed) return Reason::NotAllowed;
    if (const CapabilityStatus st = status(capability, state); st.availability != Availability::Available)
        return st.reason != Reason::None ? st.reason : Reason::Unavailable;
    if (a.granted && a.holder != controller) return Reason::AuthorityHeld; // (another controller's: it lets go first)
    if (!a.granted) a.granted = true, a.holder = controller, ++controlRevision_;
    return Reason::None;
}

Reason CapabilityHost::releaseControl(std::size_t capability, const sim::VehicleState& state, double now, ControllerId controller) noexcept {
    if (capability >= catalog_->size()) return Reason::UnknownCapability;
    Authority& a = authorityOf(capability);
    if (a.granted && a.holder != controller) return Reason::NotGranted; // (not its to let go)
    if (a.granted) a.granted = false, ++controlRevision_;
    endPolicy(capability, Reason::Released, state, now, &controller);
    return Reason::None;
}

Reason CapabilityHost::revokeControl(std::size_t capability, Reason reason, const sim::VehicleState& state, double now) noexcept {
    if (capability >= catalog_->size()) return Reason::UnknownCapability;
    if (reason == Reason::None) reason = Reason::Revoked;
    // why the platform takes it back: an activity ended so is Canceled, and says so
    if (reason != Reason::Revoked && reason != Reason::CollisionAvoidance && reason != Reason::Restricted) return Reason::InvalidParameter;
    Authority& a = authorityOf(capability);
    if (a.granted) a.granted = false, ++controlRevision_;
    endPolicy(capability, reason, state, now);
    return Reason::None;
}

Reason CapabilityHost::setAllowed(std::size_t capability, bool allowed, const sim::VehicleState& state, double now) noexcept {
    if (capability >= catalog_->size()) return Reason::UnknownCapability;
    Authority& a = authorityOf(capability);
    if (a.allowed == allowed) return Reason::None;
    a.allowed = allowed;
    ++controlRevision_;
    if (!allowed && a.granted) { // no longer allowed: its grant revoked
        a.granted = false;
        endPolicy(capability, Reason::Revoked, state, now);
    }
    return Reason::None;
}

std::uint32_t CapabilityHost::precedenceFor(std::size_t capability, std::uint32_t override) const noexcept {
    if (override != kNoPrecedenceOverride) return override;
    return capability < authority_.size() ? authority_[capability].precedence : 0;
}

std::uint32_t CapabilityHost::precedence(std::size_t capability) const noexcept { return precedenceFor(capability, kNoPrecedenceOverride); }

Reason CapabilityHost::setPrecedence(std::size_t capability, std::uint32_t precedence, const sim::VehicleState& state, double now) noexcept {
    if (capability >= catalog_->size() || !(catalog_->descriptor(capability).interactions & kCommand)) return Reason::UnknownCapability;
    Authority& a = authorityOf(capability);
    if (a.precedence == precedence) return Reason::None;
    a.precedence = precedence;
    ++controlRevision_;
    // its activities that do not override it are arbitrated by it from now
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live && records_[s].capability == capability && slots_[s].precedenceOverride == kNoPrecedenceOverride)
            records_[s].precedence = precedence;
    if (waitingCount_)
        for (Waiting& w : *waiting_)
            if (w.used && !w.suggested && w.record.capability == capability && w.options.precedenceOverride == kNoPrecedenceOverride)
                w.record.precedence = precedence;
    schedule(state, now);
    return Reason::None;
}

ControlStatus CapabilityHost::controlStatus(std::size_t capability) const noexcept {
    if (capability >= authority_.size()) return {};
    const Authority& a = authority_[capability];
    return {a.allowed, a.granted, a.granted ? a.holder : 0};
}

Reason CapabilityHost::setAvailability(std::size_t capability, Availability availability, Reason reason, std::uint64_t associated,
                                       double nextAvailableS) noexcept {
    if (capability >= catalog_->size()) return Reason::UnknownCapability;
    if (availability != Availability::Available) {
        if (reason == Reason::None) reason = Reason::Restricted;
        // why the platform restricts it: what a refused NEW and request then answer
        if (reason != Reason::Restricted && reason != Reason::CollisionAvoidance && reason != Reason::Unavailable) return Reason::InvalidParameter;
    }
    Authority& a = authorityOf(capability);
    CapabilityStatus next;
    if (availability != Availability::Available) {
        next.availability = availability;
        next.reason = reason;
        next.reasons = reasonBit(reason);
        next.description = reasonDescription(reason);
        next.associated = associated;
        next.nextAvailableS = nextAvailableS;
    }
    const CapabilityStatus& was = a.restricted;
    const bool sameTime = was.nextAvailableS == next.nextAvailableS || (std::isnan(was.nextAvailableS) && std::isnan(next.nextAvailableS));
    if (next.availability == was.availability && next.reason == was.reason && next.associated == was.associated && sameTime) return Reason::None;
    a.restricted = next;
    ++controlRevision_;
    return Reason::None;
}

const HsaCommand* CapabilityHost::liveHsa() const noexcept {
    for (std::size_t s = 0; s < kSlotCount; ++s)
        if (slots_[s].live)
            if (const auto* h = std::get_if<HsaCommand>(&config_->slots[s].command)) return h;
    return nullptr;
}

Reason CapabilityHost::resolveHsa(HsaCommand& c, const sim::VehicleState& state, CommandResult& detail) const noexcept {
    auto bad = [&detail](std::int16_t field) {
        detail.index = field;
        return Reason::InvalidParameter;
    };
    auto code = [](double v, int count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
    if (!code(c.speedReference, static_cast<int>(SpeedReference::Count))) return bad(3);
    if (!code(c.altitudeReference, static_cast<int>(AltitudeReference::Count))) return bad(5);
    if (!code(c.speedOptimization, static_cast<int>(SpeedOptimization::Count))) return bad(6);
    if (!code(c.directionReference, static_cast<int>(DirectionReference::Count))) return bad(7);
    if (!isHold(c.headingRad) && !isHold(c.courseRad)) return bad(1); // one direction, a heading or a course
    for (const auto& [v, field] : {std::pair<double, std::int16_t>{c.headingRad, 0}, {c.courseRad, 1}, {c.speed, 2}, {c.altitudeM, 4}})
        if (!isHold(v) && !std::isfinite(v)) return bad(field);
    if (const Reason why = optimisable(c.speedOptimization, 6, detail); why != Reason::None) return why;
    // what it continues: the live hsa's commands, else what the aircraft flies now - a
    // rotorcraft's speed over the ground (a hover stays put), a wing's through the air
    HsaCommand base;
    if (const HsaCommand* live = liveHsa()) {
        base = *live;
    } else {
        const auto reference = (adapter_->features() & kFeatureHover) ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed;
        base.headingRad = state.eulerRad[2];
        base.directionReference = static_cast<double>(DirectionReference::TrueNorth);
        base.speedReference = static_cast<double>(reference);
        base.speed = speedNow(reference, state);
        base.altitudeReference = static_cast<double>(AltitudeReference::Msl);
        base.altitudeM = state.altitudeMslM;
    }
    // a reference given alone: the aircraft's own value in it now (hold the Mach it flies)
    if (!isHold(c.speedReference) && isHold(c.speed)) c.speed = speedNow(static_cast<SpeedReference>(static_cast<int>(c.speedReference)), state);
    if (!isHold(c.altitudeReference) && isHold(c.altitudeM))
        c.altitudeM = altitudeNow(static_cast<AltitudeReference>(static_cast<int>(c.altitudeReference)), state, &config_->altimeter);
    if (!isHold(c.directionReference) && isHold(c.headingRad) && isHold(c.courseRad)) // (its heading now, from that north)
        c.headingRad = c.directionReference == static_cast<double>(DirectionReference::MagneticNorth) ? geo::wrapPi(state.eulerRad[2] - declinationNow(state))
                                                                                                        : state.eulerRad[2];
    mergeHsa(base, c); // a value given alone is in the reference it continues
    c = base;
    if (!isHold(c.headingRad)) c.headingRad = geo::wrapPi(c.headingRad);
    if (!isHold(c.courseRad)) c.courseRad = geo::wrapPi(c.courseRad);
    optimise(c.speed, c.speedReference, c.speedOptimization, c.altitudeM, c.altitudeReference, state);
    return Reason::None;
}

Reason CapabilityHost::optimisable(double optimization, std::int16_t field, CommandResult& detail) const noexcept {
    if (isHold(optimization) || config_->tables) return Reason::None;
    detail.index = field; // (no performance tables to fly it from: a stock aircraft's, docs/flight-autonomy.md, 4.17)
    return Reason::NotImplemented;
}

void CapabilityHost::optimise(double& speed, double& reference, double optimization, double altitudeM, double altitudeReference,
                              const sim::VehicleState& state) const noexcept {
    if (isHold(optimization)) return;
    const double h = isHold(altitudeM) ? state.altitudeMslM
                                       : altitudeMslOf(altitudeM, static_cast<AltitudeReference>(static_cast<int>(orHold(altitudeReference, 0.0))), state,
                                                       &config_->altimeter);
    const double best = optimalTasMs(config_->tables, optimization, h, state.fuelKg);
    reference = static_cast<double>(SpeedReference::TrueAirspeed);
    speed = std::isfinite(best) ? best : state.airspeedTrueMs; // (above the altitudes flown: as it flies)
}

void CapabilityHost::limitHsa(HsaCommand& c, CheckLog& log) const noexcept { limitFlight(c.speed, c.speedReference, c.altitudeM, c.altitudeReference, log, 2, 4); }

void CapabilityHost::limitFlight(double& speed, double speedReference, double& altitude, double altitudeReference, CheckLog& log, std::int16_t speedIndex,
                                 std::int16_t altitudeIndex, std::int16_t speedField, std::int16_t altitudeField) const noexcept {
    const Performance& f = performance_;
    // the altitude: under the ceiling (barometric: what the altimeter reads there); above ground, above it
    const bool barometric = altitudeReference == static_cast<double>(AltitudeReference::Barometric);
    if (aboveGround(altitudeReference)) bound(altitude, 0.0, false, altitudeIndex, Constraint::MinAltitude, log, altitudeField);
    else if (barometric) bound(altitude, indicatedAltitudeM(config_->altimeter, f.ceilingM), true, altitudeIndex, Constraint::MaxAltitude, log, altitudeField);
    else bound(altitude, f.ceilingM, true, altitudeIndex, Constraint::MaxAltitude, log, altitudeField);
    // the speed, within the envelope's calibrated speeds and Mach at the altitude it asks for (the
    // standard atmosphere's); a rotorcraft's ground speed within its fastest
    auto limit = [&](double most, bool above, Constraint constraint) { bound(speed, most, above, speedIndex, constraint, log, speedField); };
    const auto reference = static_cast<SpeedReference>(static_cast<int>(orHold(speedReference, 0.0)));
    const double h = isHold(altitude) || aboveGround(altitudeReference) ? 0.0 : barometric ? barometricMslM(config_->altimeter, altitude) : altitude;
    switch (reference) {
    case SpeedReference::GroundSpeed:
        if (f.hovers) limit(f.maxGroundSpeedMs, true, Constraint::MaxAirspeed);
        return;
    case SpeedReference::CalibratedAirspeed:
        limit(f.minCasMs, false, Constraint::MinAirspeed);
        limit(f.maxCasMs, true, Constraint::MaxAirspeed);
        limit(isa::calibratedFromTrue(f.maxTasMs, h), true, Constraint::MaxAirspeed);
        return;
    case SpeedReference::TrueAirspeed:
        limit(isa::trueFromCalibrated(f.minCasMs, h), false, Constraint::MinAirspeed);
        limit(isa::trueFromCalibrated(f.maxCasMs, h), true, Constraint::MaxAirspeed);
        limit(f.maxMach * isa::speedOfSound(h), true, Constraint::MaxAirspeed);
        limit(f.maxTasMs, true, Constraint::MaxAirspeed);
        return;
    case SpeedReference::Mach: {
        const double a = isa::speedOfSound(h);
        limit(isa::trueFromCalibrated(f.minCasMs, h) / a, false, Constraint::MinAirspeed);
        limit(isa::trueFromCalibrated(f.maxCasMs, h) / a, true, Constraint::MaxAirspeed);
        limit(f.maxMach, true, Constraint::MaxAirspeed);
        limit(f.maxTasMs / a, true, Constraint::MaxAirspeed);
        return;
    }
    default: return;
    }
}

Reason CapabilityHost::checkRoute(RouteCommand& c, Span<const Waypoint> waypoints, const sim::VehicleState& state, CheckLog& log,
                                  Span<const RouteLoiter> loiters) {
    CommandResult& detail = log.result;
    auto bad = [&detail](std::int16_t field) {
        detail.index = field;
        return Reason::InvalidParameter;
    };
    auto code = [](double v, double count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
    if (!code(c.projection, static_cast<double>(Projection::Count))) return bad(0);
    if (!code(c.repeat, 2.0)) return bad(1);
    if (!code(c.end, static_cast<double>(EndBehavior::Count))) return bad(2);
    if (!code(c.start, static_cast<double>(PathStore::kWaypoints))) return bad(3);
    c.projection = orHold(c.projection, 0.0), c.repeat = orHold(c.repeat, 0.0), c.end = orHold(c.end, 0.0), c.start = orHold(c.start, 0.0);
    const auto count = static_cast<std::uint32_t>(std::min<std::size_t>(waypoints.size(), 0xFFFF));
    if (c.repeat == 1.0 && count == 1) return bad(1); // round and round one point is a pattern, not a route
    if (count > 0 && c.start >= count) return bad(3);
    if (!routePlan_) routePlan_ = std::make_unique<route::Plan>();
    route::Plan& p = *routePlan_;
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    std::int16_t which = -1;
    auto point = [&detail](std::uint32_t i, Reason why) {
        detail.index = static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF));
        return why;
    };
    // its points in frames placed where the frames are now (4.29) - 16 frames at most, each one the session has
    const std::uint32_t n = std::min<std::uint32_t>(count, route::Plan::kMax);
    std::copy_n(waypoints.data(), n, p.points);
    p.frameCount = 0;
    for (std::uint32_t i = 0; i < n; ++i) {
        Waypoint& w = p.points[i];
        auto finite = [](double v) { return isHold(v) || std::isfinite(v); };
        if (isHold(w.frame) || !(w.frame == std::floor(w.frame) && w.frame >= 1.0 && w.frame <= 9007199254740992.0) ||
            !code(w.frameRotation, static_cast<double>(FrameRotation::Count)) || !code(w.frameOffsets, static_cast<double>(FrameOffsets::Count)) ||
            !finite(w.frameXM) || !finite(w.frameYM) || !finite(w.frameZM))
            continue; // (none, or not whole: complete() refuses it)
        const auto id = static_cast<FrameId>(w.frame);
        FrameSpec spec;
        FramePose now;
        if (!sessionView_ || !sessionView_->frame(id, spec, now)) return point(i, Reason::InvalidWaypoint);
        std::uint32_t k = 0;
        while (k < p.frameCount && p.frameIds[k] != id) ++k;
        if (k == p.frameCount) {
            if (k == PathStore::kRouteFrames) return point(i, Reason::InvalidWaypoint);
            p.frameIds[k] = id, p.frames[k] = spec, ++p.frameCount;
        }
        const GeoPoint at = framePoint(now, w.frameOffset());
        w.latitudeRad = at.latitudeRad, w.longitudeRad = geo::wrapPi(at.longitudeRad);
        if (!isHold(w.frameZM)) w.altitudeM = at.altitudeMslM, w.altitudeReference = static_cast<double>(AltitudeReference::Msl);
    }
    if (const Reason r = route::complete(p.points, p.points, count, c.repeat == 1.0, state, performance_, hovers, which, &config_->altimeter);
        r != Reason::None) {
        detail.index = which;
        return r;
    }
    // what is not built yet: the end of a path not the route's own end (FA-6e's paths), and the actions a point's type
    // asks - answered as its row in the support table says: not supported where the aircraft cannot (a taxi's, without
    // its rule), else not implemented (a taxi's, a runway's and a takeoff's points FA-9's, an approach's and a touchdown
    // FA-10's, a ditch FA-16's)
    for (std::uint32_t i = 0; i < count; ++i) {
        const Waypoint& w = p.points[i];
        if (isHold(w.waypointType)) continue;
        const auto type = static_cast<WaypointType>(static_cast<int>(w.waypointType));
        if (type == WaypointType::NavOnly || type == WaypointType::Passive || (type == WaypointType::EndOfPath && i + 1 == count)) continue;
        const char* id = nullptr;
        switch (type) {
        case WaypointType::Taxi: id = "fsim.guidance.route/waypoint_type/taxi"; break;
        case WaypointType::RunwayStart:
        case WaypointType::RunwayThreshold:
        case WaypointType::RunwayLimit: id = "fsim.guidance.route/waypoint_type/runway"; break;
        case WaypointType::Takeoff:
        case WaypointType::TakeoffInitialPoint:
        case WaypointType::TakeoffFinalPoint: id = "fsim.guidance.route/waypoint_type/takeoff"; break;
        case WaypointType::Approach:
        case WaypointType::ApproachInitialPoint:
        case WaypointType::ApproachFinalPoint:
        case WaypointType::Touchdown: id = "fsim.guidance.route/waypoint_type/landing"; break;
        case WaypointType::HardDitch: id = "fsim.guidance.route/waypoint_type/hard_ditch"; break;
        default: break; // (the end of a path before the route's end: FA-6e's)
        }
        const SupportInfo* row = id && support_ ? support_->find(id) : nullptr;
        return point(i, row && row->support == Support::NotSupported ? Reason::NotSupported : Reason::NotImplemented);
    }
    p.count = count;
    p.start = static_cast<std::uint32_t>(c.start);
    p.repeat = c.repeat == 1.0;
    p.rhumb = c.projection == static_cast<double>(Projection::Rhumb);
    p.end = static_cast<EndBehavior>(static_cast<int>(c.end));
    // its loiter points' loiters (4.31): 16 at most, each at its point, checked as a pattern NEW is
    if (loiters.size() > PathStore::kRouteLoiters) return point(loiters[PathStore::kRouteLoiters].point, Reason::InvalidWaypoint);
    p.loiterCount = static_cast<std::uint32_t>(loiters.size());
    std::copy_n(loiters.data(), p.loiterCount, p.loiters);
    if (const int at = route::loiterFault(p); at >= 0) return point(static_cast<std::uint32_t>(at), Reason::InvalidWaypoint);
    for (std::uint32_t k = 0; k < p.loiterCount; ++k)
        if (const Reason why = checkLoiter(p.loiters[k]); why != Reason::None) return point(p.loiters[k].point, why);
    if (log.range == RangePolicy::None) { // (what it flies, the behaviour plans from where it starts: its loiters complete)
        if (p.loiterCount) {
            WindEstimate wind;
            wind.update(state, 0.0);
            route::plan(p, state.latitudeRad, state.longitudeRad, state.altitudeMslM, std::hypot(wind.northMs, wind.eastMs), performance_, hovers);
            completeLoiters(p, state);
            for (std::uint32_t k = 0; k < p.loiterCount; ++k)
                if (route::shapeFault(p.loiters[k].pattern, p.loiters[k].shape) >= 0) return point(p.loiters[k].point, Reason::InvalidWaypoint);
        }
        return Reason::None;
    }
    const Performance& f = performance_;
    // the altitude a segment climbs from: the point before's, in the same reference; the start's, the aircraft's
    auto from = [&](std::uint32_t i, bool entry) {
        const Waypoint& w = p.points[i];
        if (entry) return altitudeNow(static_cast<AltitudeReference>(static_cast<int>(w.altitudeReference)), state, &config_->altimeter);
        const Waypoint& b = p.points[p.prev(i)];
        return b.altitudeReference == w.altitudeReference ? b.altitudeM : kHold;
    };
    // each point's fields (the Waypoint's order: altitude 2, speed 4, bank 7, climb rate 8), every one out of range logged
    for (std::uint32_t i = 0; i < count; ++i) {
        Waypoint& w = p.points[i];
        const auto index = static_cast<std::int16_t>(i);
        limitFlight(w.speed, w.speedReference, w.altitudeM, w.altitudeReference, log, index, index, 4, 2);
        // its turn's bank within the aircraft's (a rotorcraft's, its tilt); a fly-by's radius given, no tighter than that bank's
        bound(w.maxBankRad, hovers ? f.maxTiltRad : f.maxBankRad, true, index, Constraint::MaxOrientation, log, 7);
        if (!isHold(w.turnRadiusM) && w.turn == static_cast<double>(TurnType::FlyBy)) {
            const double v = route::plannedSpeed(w.speed, w.speedReference, w.altitudeM), bank = hovers ? f.maxTiltRad : f.maxBankRad;
            if (std::isfinite(bank) && bank > 0.0) bound(w.turnRadiusM, v * v / (9.80665 * std::tan(bank)), false, index, Constraint::MaxOrientation, log, 21);
        }
        // a climb or descent rate within the aircraft's
        if (!isHold(w.climbRateMs)) {
            const double h0 = from(i, i == p.start);
            const bool descends = !isHold(h0) && w.altitudeM < h0;
            bound(w.climbRateMs, descends ? f.maxDescentMs : f.maxClimbMs, true, index, descends ? Constraint::MaxDescentRate : Constraint::MaxClimbRate, log,
                  8);
        }
    }
    WindEstimate wind;
    wind.update(state, 0.0);
    route::plan(p, state.latitudeRad, state.longitudeRad, state.altitudeMslM, std::hypot(wind.northMs, wind.eastMs), f, hovers);
    // its loiters where they fly (4.31), each against the performance as a pattern is - named by its point, its fields
    // after the point's - and two circles fitting as limited
    if (p.loiterCount) {
        std::int16_t radiusFrom[PathStore::kRouteLoiters];
        for (std::uint32_t k = 0; k < p.loiterCount; ++k) radiusFrom[k] = radiusField(p.loiters[k].pattern, p.loiters[k].shape);
        completeLoiters(p, state);
        for (std::uint32_t k = 0; k < p.loiterCount; ++k) {
            RouteLoiter& l = p.loiters[k];
            limitPattern(l.pattern, l.shape, log, radiusFrom[k], static_cast<std::int16_t>(std::min<std::uint32_t>(l.point, 0x7FFF)));
            if (route::shapeFault(l.pattern, l.shape) >= 0) return point(l.point, Reason::InvalidWaypoint);
        }
    }
    auto flown = [&p](std::uint32_t i) { return p.repeat || i >= p.start; }; // (a route that does not repeat flies nothing before its start)
    // its turn points as laid out (4.30): a start's arc through the next point, within 170 degrees, its radius given the arc's
    // (within a metre, or half a percent), one the aircraft can turn at its speed - no clamp makes one flyable: each named;
    // an end's course the arc's there, a capture's the next leg's (within a degree)
    for (std::uint32_t i = 0; i < count; ++i) {
        const Waypoint& w = p.points[i];
        if (!flown(i)) continue;
        const std::uint32_t j = p.next(i);
        const auto turn = static_cast<TurnType>(static_cast<int>(w.turn));
        if (turn == TurnType::StartTurn && p.leaves(i)) {
            const route::Leg& arc = p.legs[j];
            if (arc.arcRadiusM > 0.0 && std::abs(arc.arcAngleRad) > 170.0 * 3.14159265358979323846 / 180.0) return point(j, Reason::InvalidWaypoint);
            if (!isHold(w.turnRadiusM) && !(arc.arcRadiusM > 0.0 && std::abs(arc.arcRadiusM - w.turnRadiusM) <= std::max(1.0, 0.005 * w.turnRadiusM)))
                return point(j, Reason::InvalidWaypoint);
            const Waypoint& b = p.points[j];
            const double v = route::plannedSpeed(b.speed, b.speedReference, b.altitudeM), bank = hovers ? f.maxTiltRad : f.maxBankRad;
            if (arc.arcRadiusM > 0.0 && std::isfinite(bank) && bank > 0.0 && arc.arcRadiusM < v * v / (9.80665 * std::tan(bank)))
                log.find(Reason::InvalidWaypoint, static_cast<std::int16_t>(j), Constraint::MaxTurnRate);
        }
        const double within = 3.14159265358979323846 / 180.0;
        if (turn == TurnType::EndTurn && !isHold(w.courseRad)) {
            const route::Leg& in = p.leg(i, i == p.start);
            if (std::abs(geo::wrapPi(in.courseInRad - w.courseRad)) > within) return point(i, Reason::InvalidWaypoint);
        }
        if (turn == TurnType::CaptureOutboundCourse && p.leaves(i) && std::abs(geo::wrapPi(p.legs[j].courseOutRad - w.courseRad)) > within)
            return point(i, Reason::InvalidWaypoint);
    }
    // every fly-by turn too big for its legs: flown smaller, or refused (a turn
    // error) - the start's from the entry, then each after it
    for (std::uint32_t k = 0; k <= count; ++k) {
        const std::uint32_t i = k == 0 ? p.start : k - 1;
        if (k > 0 && (!flown(i) || (i == p.start && !p.repeat))) continue;
        if (!(k == 0 ? p.entryTurn : p.turns[i]).shrunk) continue;
        log.reshape(static_cast<std::int16_t>(i), Constraint::None, Reason::InvalidWaypoint);
    }
    // every gradient steeper than the aircraft climbs or descends (point to point, above sea level): flown at its rate, or refused
    for (std::uint32_t i = 0; i < count; ++i) {
        Waypoint& w = p.points[i];
        if (!flown(i) || !isHold(w.climbRateMs) || aboveGround(w.altitudeReference)) continue;
        const double v = route::plannedSpeed(w.speed, w.speedReference, w.altitudeM);
        // the start's segment from the aircraft; any other, and the start's after a lap, from the point before
        for (const bool entry : {true, false}) {
            if (entry ? i != p.start : (i == p.start && !p.repeat)) continue;
            const route::Leg& leg = entry ? p.entry : p.legs[i];
            double h0 = from(i, entry), h1 = w.altitudeM;
            if (isHold(h0) || !(leg.lengthM > 1.0)) continue;
            if (w.altitudeReference == static_cast<double>(AltitudeReference::Barometric)) // (on their isobars: above sea level)
                h0 = barometricMslM(config_->altimeter, h0), h1 = barometricMslM(config_->altimeter, h1);
            const bool descends = h1 < h0;
            const double most = descends ? f.maxDescentMs : f.maxClimbMs;
            double rate = std::abs(h1 - h0) * v / leg.lengthM; // what the gradient asks
            if (std::isnan(most) || rate <= most) continue;
            log.limit(rate, most, static_cast<std::int16_t>(i), 8, descends ? Constraint::MaxDescentRate : Constraint::MaxClimbRate, Reason::PerformanceLimit);
            w.climbRateMs = most;
            break;
        }
    }
    return Reason::None;
}

Reason CapabilityHost::checkPattern(const PatternCommand& c, bool merge, CommandResult& detail) noexcept {
    auto bad = [&detail](std::int16_t field) {
        detail.index = field;
        return Reason::InvalidParameter;
    };
    auto code = [](double v, double count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
    auto given = [](double v, double lo, bool open) { return isHold(v) || (std::isfinite(v) && (open ? v > lo : v >= lo)); };
    constexpr double inf = std::numeric_limits<double>::infinity();
    if (!code(c.pattern, static_cast<double>(PatternKind::Count))) return bad(0);
    if (!isHold(c.latitudeRad) && !(std::isfinite(c.latitudeRad) && std::abs(c.latitudeRad) <= 0.5 * 3.14159265358979323846)) return bad(1);
    if (!given(c.longitudeRad, -inf, true) || isHold(c.latitudeRad) != isHold(c.longitudeRad)) return bad(2); // a point, or none
    if (!given(c.altitudeM, -inf, true)) return bad(3);
    if (!code(c.altitudeReference, static_cast<double>(AltitudeReference::Count))) return bad(4);
    if (!given(c.radiusM, 0.0, true)) return bad(5);
    if (!code(c.clockwise, 2.0)) return bad(6);
    if (!given(c.courseRad, -inf, true)) return bad(7);
    if (!given(c.legM, 0.0, false)) return bad(8);
    if (!given(c.speed, 0.0, true)) return bad(9);
    if (!code(c.speedReference, static_cast<double>(SpeedReference::Count))) return bad(10);
    if (!given(c.durationS, 0.0, true)) return bad(11);
    if (!code(c.speedOptimization, static_cast<double>(SpeedOptimization::Count))) return bad(12);
    if (merge && !isHold(c.altitudeReference) && isHold(c.altitudeM)) return bad(3); // an UPDATE has no state to take one from
    if (merge && !isHold(c.speedReference) && isHold(c.speed) && isHold(c.speedOptimization)) return bad(9);
    return Reason::None;
}

Reason CapabilityHost::checkShape(const PatternCommand& c, const PatternShape& s, bool merge, CommandResult& detail) noexcept {
    auto bad = [&detail](std::int16_t field) {
        detail.index = field;
        return Reason::InvalidParameter;
    };
    auto code = [](double v, double count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
    auto given = [](double v, double lo, bool open) { return isHold(v) || (std::isfinite(v) && (open ? v > lo : v >= lo)); };
    auto latitude = [](double v) { return isHold(v) || (std::isfinite(v) && std::abs(v) <= 0.5 * 3.14159265358979323846); };
    constexpr double inf = std::numeric_limits<double>::infinity();
    if (!code(s.directionReference, static_cast<double>(DirectionReference::Count))) return bad(13);
    if (!given(s.headingRad, -inf, true)) return bad(14);
    if (!given(s.legS, 0.0, false)) return bad(15);
    if (!isHold(s.bankRad) && !(std::isfinite(s.bankRad) && s.bankRad > 0.0 && s.bankRad < 0.5 * 3.14159265358979323846)) return bad(16);
    if (!isHold(s.orbits) && !(s.orbits == std::floor(s.orbits) && s.orbits >= 0.0 && s.orbits <= 4294967295.0)) return bad(17);
    if (!latitude(s.latitude2Rad)) return bad(18);
    if (!given(s.longitude2Rad, -inf, true) || isHold(s.latitude2Rad) != isHold(s.longitude2Rad)) return bad(19);
    if (!given(s.radius2M, 0.0, true)) return bad(20);
    if (!latitude(s.entryLatitudeRad)) return bad(21);
    if (!given(s.entryLongitudeRad, -inf, true) || isHold(s.entryLatitudeRad) != isHold(s.entryLongitudeRad)) return bad(22);
    if (!latitude(s.exitLatitudeRad)) return bad(23);
    if (!given(s.exitLongitudeRad, -inf, true) || isHold(s.exitLatitudeRad) != isHold(s.exitLongitudeRad)) return bad(24);
    if (!given(s.turnRateRadS, 0.0, true)) return bad(25);
    if (!code(s.turnType, static_cast<double>(HoldTurn::Count))) return bad(26);
    if (!code(s.holdEntry, static_cast<double>(HoldEntry::Count))) return bad(27);
    if (!code(s.holdContext, static_cast<double>(HoldContext::Count))) return bad(28);
    if (!isHold(s.frame) && !(s.frame == std::floor(s.frame) && s.frame >= 1.0 && s.frame <= 9007199254740992.0)) return bad(29);
    if (!code(s.frameRotation, static_cast<double>(FrameRotation::Count))) return bad(30);
    if (!code(s.frameOffsets, static_cast<double>(FrameOffsets::Count))) return bad(31);
    if (!given(s.frameXM, -inf, true)) return bad(32);
    if (!given(s.frameYM, -inf, true)) return bad(33);
    if (!given(s.frameZM, -inf, true)) return bad(34);
    if (merge && !isHold(s.directionReference) && isHold(c.courseRad) && isHold(s.headingRad)) return bad(13);
    return Reason::None;
}

void CapabilityHost::limitPattern(PatternCommand& c, PatternShape& shape, CheckLog& log, std::int16_t radiusFrom, std::int16_t point) const noexcept {
    // (a route's loiter's: named by its point, its field after the point's - docs/flight-autonomy.md, 4.31)
    auto at = [point](std::int16_t field) { return point < 0 ? field : point; };
    auto of = [point](std::int16_t field) { return point < 0 ? std::int16_t{-1} : static_cast<std::int16_t>(kLoiterField + field); };
    limitFlight(c.speed, c.speedReference, c.altitudeM, c.altitudeReference, log, at(9), at(3), of(9), of(3));
    // a radius the aircraft can fly at its speed: a wing's at its full bank, a rotorcraft's a metre
    const Performance& f = performance_;
    double least = 1.0, v = 0.0;
    const bool banks = !f.hovers && std::isfinite(f.maxBankRad) && f.maxBankRad > 0.0;
    if (banks) {
        const double h = aboveGround(c.altitudeReference) ? 0.0
                         : c.altitudeReference == static_cast<double>(AltitudeReference::Barometric) ? barometricMslM(config_->altimeter, c.altitudeM)
                                                                                                      : c.altitudeM;
        v = route::plannedSpeed(c.speed, c.speedReference, h);
        least = v * v / (9.80665 * std::tan(f.maxBankRad));
    }
    if (radiusFrom == 16 && banks) { // a bank it can fly, and the radius it gives at it (docs/flight-autonomy.md, 4.23)
        const double given = shape.bankRad;
        bound(shape.bankRad, f.maxBankRad, true, at(16), Constraint::MaxOrientation, log, of(16));
        if (shape.bankRad != given) c.radiusM *= std::tan(given) / std::tan(shape.bankRad);
    } else if (radiusFrom == 25 && banks) { // a turn rate its full bank flies at its speed, and the radius it gives (4.24)
        const double given = shape.turnRateRadS; // (at the least radius: v / least, g tan(bank) / v)
        bound(shape.turnRateRadS, v / least, true, at(25), Constraint::MaxOrientation, log, of(25));
        if (shape.turnRateRadS != given) c.radiusM *= given / shape.turnRateRadS;
    } else {
        bound(c.radiusM, least, false, at(radiusFrom), Constraint::MaxOrientation, log, of(radiusFrom));
    }
    bound(shape.radius2M, least, false, at(20), Constraint::MaxOrientation, log, of(20));
}

Reason CapabilityHost::checkCurveOptions(const CurveCommand& c, bool appending, CommandResult& detail) const noexcept {
    auto bad = [&detail](std::int16_t field) {
        detail.index = field;
        return Reason::InvalidParameter;
    };
    auto code = [](double v, double count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
    auto given = [](double v, double lo) { return isHold(v) || (std::isfinite(v) && v > lo); };
    constexpr double inf = std::numeric_limits<double>::infinity();
    if (!isHold(c.latitudeRad) && !(std::isfinite(c.latitudeRad) && std::abs(c.latitudeRad) <= 0.5 * 3.14159265358979323846)) return bad(0);
    if (!given(c.longitudeRad, -inf) || isHold(c.latitudeRad) != isHold(c.longitudeRad)) return bad(1);
    if (!given(c.altitudeM, -inf)) return bad(2);
    if (!isHold(c.speedMinMs) && !(std::isfinite(c.speedMinMs) && c.speedMinMs >= 0.0)) return bad(3);
    if (!given(c.speedMaxMs, 0.0) || (!isHold(c.speedMinMs) && !isHold(c.speedMaxMs) && c.speedMaxMs < c.speedMinMs)) return bad(4);
    if (!given(c.durationS, 0.0)) return bad(5);
    if (!code(c.end, static_cast<double>(EndBehavior::Count))) return bad(6);
    if (!code(c.append, 2.0) || (!appending && c.append == 1.0)) return bad(7); // (a NEW has nothing to append to)
    if (!code(c.altitudeReference, static_cast<double>(AltitudeReference::Count))) return bad(8); // (4.27)
    if (!given(c.altitudeMinM, -inf)) return bad(9);
    if (!given(c.altitudeMaxM, -inf) || (!isHold(c.altitudeMinM) && !isHold(c.altitudeMaxM) && c.altitudeMaxM < c.altitudeMinM)) return bad(10);
    if (!code(c.pointRotation, static_cast<double>(FrameRotation::Count))) return bad(11);
    if (!code(c.pointOffsets, static_cast<double>(FrameOffsets::Count))) return bad(12);
    if (!code(c.pointZ, static_cast<double>(CurveZ::Count))) return bad(13);
    return Reason::None;
}

double CapabilityHost::fastest(double altitudeM) const noexcept {
    const Performance& f = performance_;
    if (adapter_->features() & kFeatureHover) return f.maxGroundSpeedMs;
    // (fmin passes over the NaN of a limit not known)
    return std::fmin(std::fmin(isa::trueFromCalibrated(f.maxCasMs, altitudeM), f.maxMach * isa::speedOfSound(altitudeM)), f.maxTasMs);
}

void CapabilityHost::limitCurveSpeeds(CurveCommand& c, CheckLog& log) const noexcept {
    const Performance& f = performance_;
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    const double h = orHold(c.altitudeM, 0.0);
    const double least = hovers ? 0.0 : isa::trueFromCalibrated(f.minCasMs, h); // (NaN: no limit)
    const double most = fastest(h);
    // a range it can fly within: no faster at its least than it flies, no slower at its most
    // (a least below its slowest, a most above its fastest, leave it room)
    bound(c.speedMinMs, most, true, 3, Constraint::MaxAirspeed, log);
    bound(c.speedMaxMs, least, false, 4, Constraint::MinAirspeed, log);
}

Reason CapabilityHost::checkCurve(CurveCommand& c, Span<const NurbsSegment> segments, bool appending, const sim::VehicleState& state, CheckLog& log) {
    CommandResult& detail = log.result;
    if (const Reason r = checkCurveOptions(c, appending, detail); r != Reason::None) return r;
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    if (!appending) // what it leaves out: the reference where the aircraft is (4.27: in a frame, in its reference and range)
        if (const Reason why = placeCurve(c, state, detail); why != Reason::None) return why;
    // the segments: 1 to 10, finite, joined (within a metre), a metre long or more over the ground, and room for them
    auto invalid = [&detail](std::size_t segment) {
        detail.index = static_cast<std::int16_t>(std::min<std::size_t>(segment, 0x7FFF));
        return Reason::InvalidCurve;
    };
    const std::size_t n = segments.size();
    if (n == 0) return invalid(0);
    if (n > 10) return invalid(10);
    const PathStore* store = config_->path.get();
    const std::size_t held = appending && store ? store->segmentCount : 0;
    if (held + n > PathStore::kSegments) return invalid(PathStore::kSegments - held);
    auto gap = [](const NurbsSegment& a, const NurbsSegment& b) { // a's end to b's start (clamped: their last point and first)
        const std::uint32_t e = a.points - 1;
        return std::sqrt(std::pow(b.north[0] - a.north[e], 2) + std::pow(b.east[0] - a.east[e], 2) + std::pow(b.down[0] - a.down[e], 2));
    };
    for (std::size_t i = 0; i < n; ++i) {
        const NurbsSegment& s = segments[i];
        if (!route::wellFormed(s)) return invalid(i); // (its values finite, a clamped curve: 4.26)
        if (i > 0 && gap(segments[i - 1], s) > 1.0) return invalid(i);
        if (i == 0 && held > 0 && gap(store->segments[held - 1], s) > 1.0) return invalid(0); // where the curve ends
        double length = 0.0; // (its chords: no longer than it)
        route::CurvePoint last = route::evaluate(s, 0.0);
        for (int k = 1; k <= 8; ++k) {
            const route::CurvePoint p = route::evaluate(s, k / 8.0);
            length += std::hypot(p.p[0] - last.p[0], p.p[1] - last.p[1]);
            last = p;
        }
        if (length < 1.0) return invalid(i); // nothing to follow over the ground
        double from = 0.0, to = 0.0; // turning no tighter than its curvature, given (4.26)
        if (!isHold(s.curvature) && route::sharperThan(s, s.curvature, from, to)) {
            detail.from = static_cast<float>(from), detail.to = static_cast<float>(to);
            return invalid(i);
        }
    }
    if (log.range == RangePolicy::None) return Reason::None;
    // its length, and the speed it will fly: the rest of the duration's, else its range's fastest
    if (!curvePlan_) curvePlan_ = std::make_unique<route::Curve>();
    route::Curve& k = *curvePlan_;
    k.count = static_cast<std::uint32_t>(held + n);
    if (held) std::copy_n(store->segments, held, k.segments);
    std::copy_n(segments.data(), n, k.segments + held);
    k.lat0 = c.latitudeRad, k.lon0 = c.longitudeRad, k.alt0 = c.altitudeM; // (its plane: 4.27)
    k.offsets = isHold(c.pointOffsets) ? FrameOffsets::Cartesian : static_cast<FrameOffsets>(static_cast<int>(c.pointOffsets));
    k.psi = curveTurn_;
    const CurveZ z = isHold(c.pointZ) ? CurveZ::Down : static_cast<CurveZ>(static_cast<int>(c.pointZ));
    k.normalize(0, z);
    if (c.pointRotation == static_cast<double>(FrameRotation::Attitude)) k.orient(0, route::Attitude(curvePose_), z); // (in three dimensions)
    k.measure(0);
    const Performance& f = performance_;
    limitCurveSpeeds(c, log);
    const double least = hovers ? 0.0 : isa::trueFromCalibrated(f.minCasMs, c.altitudeM);
    const double most = fastest(c.altitudeM);
    // the fastest it flies over the ground: as now - a rotorcraft's ground speed (its
    // cruise, if still), a wing's airspeed with the wind behind it - or the speed that
    // takes its duration, within its range
    WindEstimate wind;
    wind.update(state, 0.0);
    const double ground = std::hypot(state.velocityNedMs[0], state.velocityNedMs[1]);
    double fast = hovers ? (ground < 1.0 ? orHold(f.cruiseTasMs, 5.0) : ground) : state.airspeedTrueMs + std::hypot(wind.northMs, wind.eastMs);
    if (!isHold(c.durationS)) { // the speed that takes it: within the aircraft's, or (clamped) flown at the most it can
        fast = k.lengthM() / c.durationS;
        double w = fast;
        bound(w, most, true, 5, Constraint::MaxAirspeed, log);
        bound(w, least, false, 5, Constraint::MinAirspeed, log);
    }
    if (!isHold(c.speedMaxMs)) fast = std::min(fast, c.speedMaxMs);
    if (!isHold(c.speedMinMs)) fast = std::max(fast, c.speedMinMs);
    // a wing: no section tighter than its full bank turns at that speed; no clamp makes one flyable - each named
    if (!hovers && std::isfinite(f.maxBankRad) && f.maxBankRad > 0.0 && fast > 1.0) {
        const double tightest = 9.80665 * std::tan(f.maxBankRad) / (fast * fast);
        for (std::size_t i = 0; i < n; ++i) {
            double from = 0.0, to = 0.0;
            if (route::tooTight(k, static_cast<std::uint32_t>(held + i), tightest, from, to))
                log.find(Reason::InvalidCurve, static_cast<std::int16_t>(i), Constraint::MaxTurnRate, static_cast<float>(from), static_cast<float>(to));
        }
    }
    // no steeper than it climbs or descends at that speed: flown at its rate (clamped), or refused - each segment's steepest
    for (std::size_t i = 0; i < n; ++i) {
        double at = 0.0;
        const double gradient = route::steepest(k, static_cast<std::uint32_t>(held + i), at);
        const route::CurvePoint p = k.point(static_cast<std::uint32_t>(held + i), at);
        const bool descends = p.gradient() < 0.0;
        const double rate = descends ? f.maxDescentMs : f.maxClimbMs;
        if (std::isnan(rate) || gradient * fast <= rate) continue;
        const auto index = static_cast<std::int16_t>(i);
        const Constraint constraint = descends ? Constraint::MaxDescentRate : Constraint::MaxClimbRate;
        if (log.range == RangePolicy::Reject) {
            log.held(Reason::PerformanceLimit, index, constraint, static_cast<float>(at), static_cast<float>(at)); // (Clamp flies it at its rate)
        } else {
            double asked = gradient * fast;
            if (!(log.result.flags & kClamped)) log.result.from = log.result.to = static_cast<float>(at); // (the first clamp's section)
            log.limit(asked, rate, index, -1, constraint, Reason::PerformanceLimit);
        }
    }
    return Reason::None;
}

void CapabilityHost::writeCurve(Span<const NurbsSegment> segments, bool appending) {
    if (!config_->path) config_->path = std::make_unique<PathStore>();
    PathStore& store = *config_->path;
    if (!appending) store.segmentCount = 0, ++store.curve, store.curveShape = curveShape_, store.curveFrame = curveFrame_; // a new curve: flown afresh
    const auto n = std::min<std::size_t>(segments.size(), PathStore::kSegments - store.segmentCount);
    std::copy_n(segments.data(), n, store.segments + store.segmentCount);
    store.segmentCount += static_cast<std::uint32_t>(n);
    ++store.revision;
}

void CapabilityHost::writeRoute() {
    if (!config_->path) config_->path = std::make_unique<PathStore>();
    PathStore& store = *config_->path;
    const route::Plan& p = *routePlan_;
    std::copy_n(p.points, p.count, store.waypoints);
    store.count = p.count;
    store.routeFrameCount = p.frameCount; // (its points' frames, as placed: 4.29)
    std::copy_n(p.frameIds, p.frameCount, store.routeFrameIds);
    std::copy_n(p.frames, p.frameCount, store.routeFrames);
    store.routeLoiterCount = p.loiterCount; // (its loiters complete, their place their points': 4.31)
    for (std::uint32_t k = 0; k < p.loiterCount; ++k) store.routeLoiters[k] = route::unplaced(p.loiters[k]);
    ++store.revision;
}

void CapabilityHost::setProtection(ProtectionMode mode) noexcept {
    config_->protection.mode = mode;
    ++config_->revision;
}

ProtectionMode CapabilityHost::protection() const noexcept { return config_->protection.mode; }

EnvelopeStatus CapabilityHost::envelope() noexcept {
    EnvelopeStatus status = envelope_;
    status.mode = config_->protection.mode;
    envelope_ = EnvelopeStatus{};
    return status;
}

CommandResult CapabilityHost::rejected(Reason reason, ActivityId activity, ActivityId other) const noexcept {
    CommandResult r;
    r.status = CommandStatus::Rejected;
    r.reason = reason;
    r.activity = activity;
    r.other = other;
    return r;
}

CommandResult CapabilityHost::valid(const CommandResult& detail) const noexcept {
    CommandResult r = about(accepted(0, detail.flags), detail);
    r.status = CommandStatus::Valid;
    return r;
}

int CapabilityHost::liveSlot(ActivityId activity) const noexcept {
    if (!activity) return -1;
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live && slots_[s].activity == activity) return static_cast<int>(s);
    return -1;
}

CapabilityStatus CapabilityHost::vehicleStatus(std::size_t capability, const sim::VehicleState& state) const noexcept {
    CapabilityStatus out;
    if (capability >= catalog_->size()) out.availability = Availability::Unavailable, out.reason = Reason::UnknownCapability;
    else if (state.diverged) out.availability = Availability::TemporarilyUnavailable, out.reason = Reason::Diverged;
    return out;
}

Reason CapabilityHost::phase(const CapabilityDescriptor& d, const sim::VehicleState& state) noexcept {
    // the platform's airborne guidance waits for the aircraft to fly; the flight
    // levels and the support effectors are offered in every phase
    static constexpr std::string_view kPlatform = "fsim.guidance.";
    if (d.kind == CapabilityKind::Guidance && state.onGround && d.id.compare(0, kPlatform.size(), kPlatform) == 0) return Reason::OnGround;
    return Reason::None;
}

Reason CapabilityHost::missing(std::string_view feature) const noexcept {
    return support_ ? support_->refusal(feature) : Reason::UnknownCapability;
}

CapabilityStatus CapabilityHost::status(std::size_t capability, const sim::VehicleState& state) const noexcept {
    CapabilityStatus out;
    // every reason that holds, the first of them by precedence: the vehicle's
    // own, the flight phase, the platform's restriction, a placard
    auto add = [&out](Availability availability, Reason reason, const char* description) {
        if (out.reasons == 0) out.availability = availability, out.reason = reason, out.description = description;
        out.reasons |= reasonBit(reason);
    };
    const CapabilityStatus own = vehicleStatus(capability, state);
    if (own.availability != Availability::Available) {
        add(own.availability, own.reason, reasonDescription(own.reason));
        if (capability >= catalog_->size()) return out;
    }
    const CapabilityDescriptor& d = catalog_->descriptor(capability);
    if (const Reason why = phase(d, state); why != Reason::None) add(Availability::TemporarilyUnavailable, why, reasonDescription(why));
    if (capability < authority_.size() && authority_[capability].restricted.availability != Availability::Available) {
        const CapabilityStatus& r = authority_[capability].restricted;
        if (out.reasons == 0) out.associated = r.associated, out.nextAvailableS = r.nextAvailableS;
        add(r.availability, r.reason, r.description);
    }
    const int kind = catalog_->supportKindOf(capability);
    if (kind >= 0 && !d.parameters.empty()) { // a support effector's placard: the same as its admission's
        const Placard p = adapter_->placard(static_cast<std::size_t>(kind), state, *profile_);
        if (p.reason != Reason::None) {
            add(Availability::TemporarilyUnavailable, p.reason, p.description);
        } else if (p.min > d.parameters[0].min || p.max < d.parameters[0].max) {
            out.ranges[out.rangeCount++] = {0, std::max(p.min, d.parameters[0].min), std::min(p.max, d.parameters[0].max)};
            if (out.reasons == 0) out.description = p.description;
        }
    }
    return out;
}

CapabilityHost::Standing CapabilityHost::standing(Source source, std::uint32_t precedence, Rank rank, bool interrupt, ActivityId id,
                                                  const ActivityRecord& h) noexcept {
    if (h.source > source) return interrupt ? Standing::Refused : Standing::Waits; // a higher source's (ADR-26): refused, or waited for
    const bool platform = source != Source::Policy;
    if (platform && interrupt) return Standing::Takes;   // the primary controller interrupts any rank
    if (!platform && !interrupt) return Standing::Waits; // a policy's "nice" command lets what flies finish
    // rank decides - a policy's interrupting command, the platform's deferring one: its
    // capability's precedence, then its rank; equal, the newer command (ADR-26's rule: a
    // NEW takes; what waited - unassigned, enabled, scheduled - does not take from a newer one)
    if (precedence != h.precedence) return precedence < h.precedence ? Standing::Takes : Standing::Waits;
    if (!(rank == h.rank)) return ranksAhead(rank, h.rank) ? Standing::Takes : Standing::Waits;
    return id > h.id ? Standing::Takes : Standing::Waits;
}

CapabilityHost::Standing CapabilityHost::arbitrate(AxisMask axes, Source source, std::uint32_t precedence, Rank rank, bool interrupt, ActivityId id,
                                                   ActivityId& blocker) const noexcept {
    Standing out = Standing::Takes;
    for (std::size_t s = 0; s < kActivities; ++s) {
        if (!slots_[s].live || !(records_[s].axes & axes)) continue;
        const Standing st = standing(source, precedence, rank, interrupt, id, records_[s]);
        if (st == Standing::Refused) {
            blocker = records_[s].id;
            return st;
        }
        if (st == Standing::Waits && out == Standing::Takes) out = st, blocker = records_[s].id;
    }
    return out;
}

Reason CapabilityHost::checkWindow(const TimeWindow& w, double now) noexcept {
    if (w.criticality == TimeCriticality::None && !w.any()) return Reason::None; // (none given: every command's without one)
    for (const double t : {w.startNotBefore, w.startNotAfter, w.endNotBefore, w.endNotAfter})
        if (!std::isnan(t) && !std::isfinite(t)) return Reason::InvalidParameter;
    if (static_cast<unsigned>(w.criticality) >= static_cast<unsigned>(TimeCriticality::Count)) return Reason::InvalidParameter;
    auto inOrder = [](double a, double b) { return std::isnan(a) || std::isnan(b) || a <= b; };
    if (!inOrder(w.startNotBefore, w.startNotAfter) || !inOrder(w.endNotBefore, w.endNotAfter) || !inOrder(w.startNotBefore, w.endNotAfter))
        return Reason::InvalidParameter;
    // one it can still meet: its end window open, a critical start window open
    if (w.endNotAfter <= now || (w.startCritical() && w.startNotAfter < now)) return Reason::TimeConstraint;
    return Reason::None;
}

std::size_t CapabilityHost::directSlot(const SupportCommand& command) noexcept {
    if (std::holds_alternative<EnginesCommand>(command)) return kEnginesSlot;
    return kSlotCount + static_cast<std::size_t>(supportAxis(command)) - static_cast<std::size_t>(Axis::Flaps);
}

void CapabilityHost::writeDirect(std::size_t s, const SupportCommand& command) noexcept {
    RuntimeConfig& config = *config_;
    if (const auto* e = std::get_if<EnginesCommand>(&command)) {
        for (std::size_t i = 0; i < config.engines.size(); ++i) config.engines[i] = e->throttle[i];
        return;
    }
    SupportDemand& demand = config.support[s - kSlotCount];
    supportValues(command, demand.value, demand.value2);
    ++demand.revision;
}

Reason CapabilityHost::checkAxes(const CapabilityDescriptor& d, AxisMask axes) noexcept {
    if (axes & ~kAllAxes) return Reason::InvalidAxes;
    const auto primary = static_cast<AxisMask>(axes & kPrimaryAxes);
    if (!primary) return Reason::InvalidAxes; // a slot flies through the cascade on at least one primary axis
    if (primary == kPrimaryAxes || (d.axisGroups & kGroupEachAxis)) return Reason::None;
    // owned apart: a union of whole groups the capability allows (guidance allows none)
    AxisMask covered = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
        const auto group = static_cast<AxisGroup>(1u << bit);
        const AxisMask in = groupAxes(group);
        if ((d.axisGroups & group) && in && (primary & in) == in) covered = static_cast<AxisMask>(covered | in);
    }
    return covered == primary ? Reason::None : Reason::InvalidAxes;
}

Reason CapabilityHost::awareUpTo(int top) const noexcept {
    for (const Level l : kCascadeOrder) // every level with a controller, from the acceleration level up to rank `top`
        if (levelRank(l) <= top)
            if (const Controller* c = runtime_->controller(l); c && !c->axisAware()) return Reason::ControllerNotAxisAware;
    return Reason::None;
}

Reason CapabilityHost::checkAwareness(AxisMask taken, Level level, bool cascade) const noexcept {
    const RuntimeConfig& c = *config_;
    const auto primary = static_cast<AxisMask>(taken & kPrimaryAxes);
    if (!primary || (cascade && primary == kPrimaryAxes)) return Reason::None; // no change, or one owner of every axis
    // The highest level a controller would see axes owned apart at, afterwards:
    // the new owner's, what the others keep, and the default's hold. (Where a
    // demand really goes depends on the controllers; every level up to it is checked.)
    int top = cascade ? cascadeTop(level) : -1;
    AxisMask unowned = 0;
    for (std::size_t s = 0; s < kSlotCount; ++s)
        if (c.slots[s].axes & kPrimaryAxes & ~primary) top = std::max(top, cascadeTop(c.slots[s].level));
    for (std::size_t a = 0; a < kPrimaryAxisCount; ++a)
        if (!(primary & (1u << a)) && c.owner[a] == RuntimeConfig::kNone) unowned = static_cast<AxisMask>(unowned | (1u << a));
    if (unowned && c.vehicleDefault == VehicleDefault::Hold) top = std::max(top, levelRank(Level::Velocity));
    return awareUpTo(top);
}

Reason CapabilityHost::setVehicleDefault(VehicleDefault mode) noexcept {
    RuntimeConfig& c = *config_;
    if (mode == c.vehicleDefault) return Reason::None;
    if (mode == VehicleDefault::Hold) {
        AxisMask unowned = 0;
        int top = levelRank(Level::Velocity);
        for (std::size_t a = 0; a < kPrimaryAxisCount; ++a)
            if (c.owner[a] == RuntimeConfig::kNone) unowned = static_cast<AxisMask>(unowned | (1u << a));
        for (std::size_t s = 0; s < kSlotCount; ++s)
            if (c.slots[s].axes & kPrimaryAxes) top = std::max(top, cascadeTop(c.slots[s].level));
        // the hold would fly beside what others own
        if (unowned && unowned != kPrimaryAxes)
            if (const Reason why = awareUpTo(top); why != Reason::None) return why;
        for (std::size_t a = 0; a < kPrimaryAxisCount; ++a)
            if (unowned & (1u << a)) ++c.letGo[a]; // held from where they are now
    }
    c.vehicleDefault = mode;
    ++c.revision;
    return Reason::None;
}

void CapabilityHost::takeOver(AxisMask axes, ActivityId id, double now) noexcept {
    for (std::size_t s = 0; s < kActivities; ++s) {
        if (!slots_[s].activity || !(records_[s].axes & axes)) continue;
        const auto lost = static_cast<AxisMask>(records_[s].axes & axes);
        const auto kept = static_cast<AxisMask>(records_[s].axes & ~axes);
        if (slots_[s].live) {
            if ((lost & kPrimaryAxes) || !kept) end(s, ActivityState::Canceled, Reason::Preempted, id, now);
            else slots_[s].flags |= kActivityAxesReduced; // only support axes it can do without
        }
        if (!isCascade(s) || !(kept & kPrimaryAxes)) {
            release(s); // nothing left to fly through the cascade; support axes it kept return to the default
            continue;
        }
        // What it keeps flies on: the live activity, or the ended one's residual hold (9.4).
        records_[s].axes = kept;
        config_->slots[s].axes = kept;
        ++config_->revision;
    }
}

void CapabilityHost::makeRecord(ActivityRecord& record, ActivityId id, std::size_t capability, const CommandOptions& options, AxisMask axes,
                                double now) const noexcept {
    // every field set here, in place (a slot's record is written once per NEW)
    record.state = ActivityState::Pending, record.reason = Reason::None, record.by = 0;
    record.constraints = record.constraintsSeen = 0;
    record.endTime = std::numeric_limits<double>::quiet_NaN();
    record.progress = ActivityProgress{};
    record.waiting = ActivityWait::None, record.waitingFor = 0;
    record.id = id;
    record.vehicle = vehicle_;
    record.capability = static_cast<std::uint16_t>(capability);
    record.source = options.source;
    record.controller = options.controller;
    record.commandId = options.commandId;
    record.trace = options.trace;
    record.interactive = options.interactive;
    record.interrupt = options.interrupt;
    record.rank = options.rank;
    record.precedence = precedenceFor(capability, options.precedenceOverride);
    record.window = options.window;
    record.axes = axes;
    record.startTime = now;
}

ActivityRecord& CapabilityHost::start(std::size_t s, const Launch& what, const CommandOptions& options, double now) noexcept {
    slots_[s] = Slot{what.id, true, options.range, static_cast<std::uint16_t>(what.flags & kClamped ? kActivityClamped : 0), kUnknown};
    slots_[s].precedenceOverride = options.precedenceOverride;
    slots_[s].firstStart = what.firstStart;
    ActivityRecord& record = records_[s];
    if (what.waited) record = *what.waited, record.waiting = ActivityWait::None, record.waitingFor = 0; // (it flies)
    else makeRecord(record, what.id, what.capability, options, what.axes, now);
    if (!std::isnan(record.window.endNotAfter)) ++windowed_;
    return record;
}

Reason CapabilityHost::axesOf(std::size_t index, const Command& command, const CommandOptions& options, AxisMask& axes) const noexcept {
    const CapabilityDescriptor& d = catalog_->descriptor(index);
    axes = options.axes ? options.axes : catalog_->defaultAxes(index, command);
    // above the actuators a command owns whole groups: a wing's loop that banks
    // also coordinates, a rotorcraft's cyclic tilts in roll and pitch at once
    if (d.level != Level::Actuator) axes = widenToGroups(axes, d.axisGroups);
    return checkAxes(d, axes);
}

Reason CapabilityHost::prepare(std::size_t index, Command& setpoint, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments,
                               const sim::VehicleState& state, CheckLog& log, const PatternShape* shape, const CurveShape* curveShape,
                               Span<const RouteLoiter> loiters) {
    const bool checked = log.range != RangePolicy::None;
    CommandResult& detail = log.result;
    auto* hsa = std::get_if<HsaCommand>(&setpoint);
    if (hsa) // complete it first: what it leaves out, whatever the range policy (the runtime flies a complete setpoint)
        if (const Reason why = resolveHsa(*hsa, state, detail); why != Reason::None) return why;
    if (auto* route = std::get_if<RouteCommand>(&setpoint)) // its waypoints completed, and checked as its range policy says
        if (const Reason why = checkRoute(*route, waypoints, state, log, loiters); why != Reason::None) return why;
    if (auto* curve = std::get_if<CurveCommand>(&setpoint)) { // its segments checked as its range policy says (its shape into the scratch)
        curveShape_ = curveShape ? *curveShape : CurveShape{};
        if (const Reason why = checkCurve(*curve, segments, false, state, log); why != Reason::None) return why;
    }
    auto* pattern = std::get_if<PatternCommand>(&setpoint);
    std::int16_t radiusFrom = 5;
    if (pattern) { // complete it first, as an hsa (the runtime flies a complete setpoint) - with its shape, into the scratch
        patternShape_ = shape ? *shape : PatternShape{};
        if (const Reason why = checkPattern(*pattern, false, detail); why != Reason::None) return why;
        if (const Reason why = checkShape(*pattern, patternShape_, false, detail); why != Reason::None) return why;
        if (const Reason why = placePattern(*pattern, patternShape_, detail); why != Reason::None) return why; // (a hover, a frame's point: 4.25)
        if (const Reason why = optimisable(pattern->speedOptimization, 12, detail); why != Reason::None) return why;
        optimise(pattern->speed, pattern->speedReference, pattern->speedOptimization, pattern->altitudeM, pattern->altitudeReference, state);
        radiusFrom = radiusField(*pattern, patternShape_);
        completePattern(*pattern, patternShape_, state);
    }
    if (checked)
        if (const Reason why = catalog_->check(index, setpoint, log); why != Reason::None) return why;
    if (const auto* b = std::get_if<BehaviorCommand>(&setpoint); b && checked) // what the behaviour needs from where the aircraft is
        if (const BehaviorTraits::Admission admit = catalog_->admission(index)) {
            CommandResult why;
            if (const Reason r = admit(*b, state, performance_, why); r != Reason::None) log.find(r, why.index, why.constraint, why.from, why.to);
        }
    if (hsa && checked) limitHsa(*hsa, log);
    if (pattern && checked) limitPattern(*pattern, patternShape_, log, radiusFrom);
    if (pattern) // two circles that fit, their radii as limited (docs/flight-autonomy.md, 4.23)
        if (const int field = route::shapeFault(*pattern, patternShape_); field >= 0) {
            detail.index = static_cast<std::int16_t>(field);
            return Reason::InvalidParameter;
        }
    checkTerrain(setpoint, state, log); // (docs/flight-autonomy.md, 4.19: where it is checked)
    return Reason::None;
}

void CapabilityHost::launch(const Launch& what, const CommandOptions& options, Command&& setpoint, std::unique_ptr<Behavior>&& behavior, bool route,
                            Span<const NurbsSegment> curve, double now) noexcept {
    takeOver(what.axes, what.id, now);
    // A free slot: every slot in use flies a primary axis the new activity did not take.
    std::size_t s = 0;
    while (s + 1 < kSlotCount && slots_[s].activity) ++s;
    RuntimeConfig& config = *config_;
    if (route) writeRoute(); // (a guidance activity owns every primary axis: one route flies at a time)
    if (auto* c = std::get_if<CurveCommand>(&setpoint)) writeCurve(curve, false), c->append = kHold;
    if (std::holds_alternative<PatternCommand>(setpoint)) writeShape();
    SetpointSlot& slot = config.slots[s];
    slot.command = std::move(setpoint);
    slot.level = catalog_->descriptor(what.capability).level;
    slot.axes = what.axes;
    ++slot.generation;
    ++slot.revision;
    for (std::size_t a = 0; a < kAxisCount; ++a)
        if (what.axes & (1u << a)) config.owner[a] = static_cast<std::uint8_t>(s);
    ++config.revision;
    runtime_->install(s, std::move(behavior));
    start(s, what, options, now);
}

void CapabilityHost::launchDirect(const Launch& what, const CommandOptions& options, const SupportCommand& setpoint, double now) noexcept {
    takeOver(what.axes, what.id, now);
    const std::size_t s = directSlot(setpoint);
    writeDirect(s, setpoint);
    if (s == kEnginesSlot) { // thrust, or every primary axis where the engines fly the aircraft (a multirotor's rotors)
        for (std::size_t a = 0; a < kAxisCount; ++a)
            if (what.axes & (1u << a)) config_->owner[a] = RuntimeConfig::kEngines;
    } else {
        config_->owner[static_cast<std::size_t>(supportAxis(setpoint))] = RuntimeConfig::kSupport;
    }
    ++config_->revision;
    start(s, what, options, now);
    if (catalog_->descriptor(what.capability).persistence == Persistence::Terminating) slots_[s].target = supportGoal(setpoint);
}

CapabilityHost::Waiting* CapabilityHost::freeWaiting() {
    if (waitingCount_ >= kWaiting) return nullptr;
    if (!waiting_) waiting_ = std::make_unique<std::array<Waiting, kWaiting>>();
    for (Waiting& w : *waiting_)
        if (!w.used) return &w;
    return nullptr;
}

CapabilityHost::Waiting* CapabilityHost::waitingEntry(ActivityId activity) noexcept {
    if (!waitingCount_ || !activity) return nullptr;
    for (Waiting& w : *waiting_)
        if (w.used && !w.suggested && w.record.id == activity) return &w;
    return nullptr;
}

const CapabilityHost::Waiting* CapabilityHost::waitingEntry(ActivityId activity) const noexcept {
    return const_cast<CapabilityHost*>(this)->waitingEntry(activity);
}

void CapabilityHost::endWaiting(Waiting& w, ActivityState state, Reason reason, double now) noexcept {
    ActivityRecord& record = w.record;
    record.state = state;
    record.reason = reason;
    record.by = 0;
    record.endTime = now;
    recent_[recentNext_] = record;
    recentNext_ = (recentNext_ + 1) % kRecent;
    recentCount_ = std::min(recentCount_ + 1, kRecent);
    noteEnd(record);
    w.used = false;
    w.behavior.reset(); // (what it kept: freed, never allocated)
    --waitingCount_;
}

bool CapabilityHost::startWaiting(Waiting& w, const sim::VehicleState& state, double now) noexcept {
    const ActivityRecord record = w.record;
    const CapabilityDescriptor& d = catalog_->descriptor(record.capability);
    CommandResult detail; // (no answer to give: what the checks find is the activity's end, if anything)
    CheckLog log{detail, w.options.range, nullptr};
    // prepared afresh from where the aircraft is now: the NEW's checks that depend on it
    Reason why = Reason::None;
    if (w.options.range != RangePolicy::None) {
        if (const CapabilityStatus own = vehicleStatus(record.capability, state); own.availability != Availability::Available) why = own.reason;
        else if (record.source == Source::Policy) why = phase(d, state);
    }
    if (w.support) {
        SupportCommand setpoint = w.supportCommand;
        if (why == Reason::None && w.options.range != RangePolicy::None) why = catalog_->check(record.capability, setpoint, log);
        if (why == Reason::None) why = log.refused;
        if (why == Reason::None) why = adapter_->admit(setpoint, state, *profile_);
        if (why == Reason::None) why = checkAwareness(record.axes, Level::Actuator, false);
        if (why != Reason::None) {
            endWaiting(w, ActivityState::Failed, why, now);
            return false;
        }
        w.used = false, --waitingCount_;
        launchDirect(Launch{&record, record.id, record.capability, record.axes, detail.flags}, w.options, setpoint, now);
        return true;
    }
    Command setpoint = std::move(w.command); // (it flies once: moved, never copied)
    const bool route = std::holds_alternative<RouteCommand>(setpoint);
    if (why == Reason::None) why = prepare(record.capability, setpoint, Span<const Waypoint>(w.waypoints.data(), w.waypoints.size()),
                                           Span<const NurbsSegment>(w.segments.data(), w.segments.size()), state, log, &w.shape, &w.curveShape,
                                           Span<const RouteLoiter>(w.loiters.data(), w.loiters.size()));
    const bool found = why == Reason::None && log.refused != Reason::None; // (refused by what the checks found, not malformed)
    if (why == Reason::None) why = log.refused;
    if (found && w.options.range == RangePolicy::Reject && log.clampable) {
        // what Clamp would fly, suggested in its place (4.11): kept in its entry until a call makes it a task
        w.command = std::move(setpoint);
        if (route && routePlan_) { // (within its room)
            w.waypoints.assign(routePlan_->points, routePlan_->points + routePlan_->count);
            w.loiters.clear();
            for (std::uint32_t k = 0; k < routePlan_->loiterCount; ++k) w.loiters.push_back(route::unplaced(routePlan_->loiters[k]));
        }
        if (std::holds_alternative<PatternCommand>(w.command)) w.shape = patternShape_;
        w.suggested = true;
        w.suggestion = kSuggestedTask | ++suggestionSerial_;
        ++pendingSuggestions_;
        ActivityRecord& failed = w.record;
        failed.state = ActivityState::Failed, failed.reason = why, failed.by = 0, failed.endTime = now, failed.suggestion = w.suggestion;
        recent_[recentNext_] = failed;
        recentNext_ = (recentNext_ + 1) % kRecent;
        recentCount_ = std::min(recentCount_ + 1, kRecent);
        noteEnd(failed);
        return false;
    }
    if (why == Reason::None) why = checkAwareness(record.axes, d.level, true);
    if (why != Reason::None) {
        endWaiting(w, ActivityState::Failed, why, now);
        return false;
    }
    w.used = false, --waitingCount_;
    const auto* flown = std::get_if<RouteCommand>(&setpoint);
    const double firstStart = !isHold(w.firstStart) ? w.firstStart : flown ? flown->start : kHold;
    launch(Launch{&record, record.id, record.capability, record.axes, detail.flags, firstStart}, w.options, std::move(setpoint), std::move(w.behavior),
           route, Span<const NurbsSegment>(w.segments.data(), w.segments.size()), now);
    return true;
}

bool CapabilityHost::scheduleWaiting(const sim::VehicleState& state, double now) noexcept {
    // source, the higher first; then precedence and rank, the lower first; then age
    std::array<Waiting*, kWaiting> order{};
    std::size_t n = 0;
    for (Waiting& w : *waiting_)
        if (w.used && !w.suggested) order[n++] = &w;
    std::sort(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(n), [](const Waiting* a, const Waiting* b) {
        const ActivityRecord &x = a->record, &y = b->record;
        if (x.source != y.source) return x.source > y.source;
        if (x.precedence != y.precedence) return x.precedence < y.precedence;
        if (!(x.rank == y.rank)) return ranksAhead(x.rank, y.rank);
        return a->queued < b->queued; // (in the order they began to wait)
    });
    bool started = false;
    // Each start can free axes one before it in the order waited for (it ends
    // what it takes, and what that held besides): the pass again until none starts.
    for (bool again = true; again;) {
        again = false;
        for (std::size_t i = 0; i < n; ++i) {
            Waiting& w = *order[i];
            if (!w.used) continue; // (started or ended in an earlier pass)
            ActivityRecord& r = w.record;
            const TimeWindow& t = r.window;
            const bool disabled = r.state == ActivityState::Disabled; // (it waits for ENABLE: only its end window can end it)
            const bool first = !w.resumed;                            // (its start window is its first start's)
            // a window it can no longer meet: its end's closed, or its critical start's
            if (now >= t.endNotAfter || (!disabled && first && t.startCritical() && now > t.startNotAfter)) {
                endWaiting(w, ActivityState::Failed, Reason::TimeConstraint, now);
                continue;
            }
            if (disabled) continue;
            if (first && now < t.startNotBefore) {
                r.waiting = ActivityWait::Scheduled, r.waitingFor = 0;
                continue;
            }
            ActivityId blocker = 0;
            if (arbitrate(r.axes, r.source, r.precedence, r.rank, r.interrupt, r.id, blocker) != Standing::Takes) {
                r.waiting = ActivityWait::Queued, r.waitingFor = blocker;
                continue;
            }
            if (startWaiting(w, state, now)) started = again = true;
        }
    }
    return started;
}

void CapabilityHost::keepWindows(double now) noexcept {
    if (!windowed_) return;
    for (std::size_t s = 0; s < kActivities; ++s) {
        if (!slots_[s].live || !(now >= records_[s].window.endNotAfter)) continue;
        const ActivityRecord& r = records_[s];
        if (catalog_->descriptor(r.capability).persistence == Persistence::Persistent) end(s, ActivityState::Completed, Reason::GoalReached, 0, now); // done: its window's end
        else if (r.window.endCritical()) end(s, ActivityState::Failed, Reason::TimeConstraint, 0, now); // not done in time
        // (late, its end not critical: it goes on)
    }
}

CommandResult CapabilityHost::submit(const Command& command, const CommandOptions& options, const sim::VehicleState& state, double now) {
    return submitWith(command, {}, {}, options, state, now);
}

CommandResult CapabilityHost::submit(const RouteCommand& route, Span<const Waypoint> waypoints, const CommandOptions& options, const sim::VehicleState& state,
                                     double now, Span<const RouteLoiter> loiters) {
    return submitWith(Command(route), waypoints, {}, options, state, now, true, nullptr, nullptr, loiters);
}

CommandResult CapabilityHost::submit(const CurveCommand& curve, Span<const NurbsSegment> segments, const CommandOptions& options,
                                     const sim::VehicleState& state, double now, const CurveShape* shape) {
    return submitWith(Command(curve), {}, segments, options, state, now, true, nullptr, shape);
}

CommandResult CapabilityHost::submitWith(const Command& command, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments,
                                         const CommandOptions& options, const sim::VehicleState& state, double now, bool mayWait,
                                         const PatternShape* shape, const CurveShape* curveShape, Span<const RouteLoiter> loiters) {
    if (pendingSuggestions_) materialize();
    details_.clear();
    const int found = catalog_->indexOf(command);
    if (found < 0) return rejected(missing(featureOf(command))); // not supported, not implemented, or unknown
    const auto index = static_cast<std::size_t>(found);
    const CapabilityDescriptor& d = catalog_->descriptor(index);
    if (!(d.interactions & kCommand)) return rejected(Reason::UnknownCapability);
    // a behaviour takes a BehaviorCommand, a mode its own setpoint (a BehaviorCommand naming "hsa" is not an hsa)
    if (d.kind == CapabilityKind::Guidance && (d.setpoint == SetpointKind::Behavior) != std::holds_alternative<BehaviorCommand>(command))
        return rejected(Reason::WrongCommandType);
    // a policy's authority (a grant, under Granted) and the platform's restrictions, whatever the range policy
    if (const Reason why = admits(index, options.source, options.controller); why != Reason::None) return rejected(why);
    // how it is arbitrated and scheduled (docs/flight-autonomy.md, 4.9): a capability's precedence is the platform's
    if (options.precedenceOverride != kNoPrecedenceOverride && options.source == Source::Policy) return rejected(Reason::NotAllowed);
    if (const Reason why = checkWindow(options.window, now); why != Reason::None) return rejected(why);
    refreshPerformance(); // (what the checks below plan with)
    const bool checked = options.range != RangePolicy::None;
    if (checked) { // (the vehicle's own availability: the platform's restrictions stop a policy only, above)
        if (const CapabilityStatus own = vehicleStatus(index, state); own.availability != Availability::Available) return rejected(own.reason);
        if (d.version < options.minVersion) return rejected(Reason::VersionUnsupported);
        // the flight phase, as the status tells a policy (docs/flight-autonomy.md, 4.5): never FA's own sources
        if (options.source == Source::Policy)
            if (const Reason why = phase(d, state); why != Reason::None) return rejected(why);
    }

    Command setpoint = command;
    CommandResult detail; // what the checks found: kClamped, the first finding's detail
    CheckLog log{detail, options.range, &details_};
    if (const Reason why = prepare(index, setpoint, waypoints, segments, state, log, shape, curveShape, loiters); why != Reason::None)
        return about(rejected(why), detail);
    // every finding named: refused with the first (docs/flight-autonomy.md, 4.8) - and, where Clamp
    // would fly what the checks left, a task with it suggested in its place (4.11)
    if (log.refused != Reason::None) {
        if (options.range == RangePolicy::Reject && log.clampable && !options.validateOnly && d.kind != CapabilityKind::Support)
            details_.suggestion = suggest(setpoint, waypoints, segments, &patternShape_, &curveShape_, loiters);
        return about(rejected(log.refused), detail);
    }
    // its endurance (4.18): a flight with an end needs no more than the vehicle has above its reserve - a soft
    // rejection, which overrideRejection overrides (the first there is). Only a route, pattern or curve can have one.
    if (checked && sessionView_ &&
        (std::holds_alternative<RouteCommand>(setpoint) || std::holds_alternative<PatternCommand>(setpoint) || std::holds_alternative<CurveCommand>(setpoint)))
        if (const CommandDetails::Endurance need = endurance(setpoint, state, now); need.energy && need.required > need.remaining) {
            details_.endurance = need;
            if (!options.overrideRejection) {
                log.find(Reason::InsufficientEndurance, -1, Constraint::None);
                return about(rejected(Reason::InsufficientEndurance), detail);
            }
            detail.flags = static_cast<std::uint16_t>(detail.flags | kOverridden);
        }
    const std::uint16_t flags = detail.flags;
    AxisMask axes = 0;
    if (const Reason why = axesOf(index, command, options, axes); why != Reason::None) return rejected(why);
    // it starts now, or waits: for its start window, or for what it may not interrupt (4.9)
    const bool scheduled = options.window.startNotBefore > now;
    ActivityId blocker = 0;
    Standing standing = Standing::Waits;
    if (!scheduled) {
        if (const Reason why = checkAwareness(axes, d.level, true); why != Reason::None) return rejected(why);
        standing = arbitrate(axes, options.source, precedenceFor(index, options.precedenceOverride), options.rank, options.interrupt, kNewest, blocker);
        if (standing == Standing::Refused) return rejected(Reason::AuthorityHeld, 0, blocker);
    }
    const bool waits = standing != Standing::Takes;
    // the existing entry points never wait: what they may not take - a capability that
    // comes later by the platform's precedence - holds its axes against them, as a higher source does
    if (waits && !mayWait) return rejected(Reason::AuthorityHeld, 0, blocker);
    if (waits && waitingCount_ >= kWaiting) return rejected(Reason::QueueFull);
    if (options.validateOnly) { // as a NEW would be answered; nothing flies
        CommandResult r = valid(detail);
        if (waits) r.flags = static_cast<std::uint16_t>(r.flags | kDeferred), r.other = blocker;
        return r;
    }

    std::unique_ptr<Behavior> behavior;
    if (d.kind == CapabilityKind::Guidance) {
        auto created = ControllerRegistry::instance().create(d.behavior);
        auto* b = dynamic_cast<Behavior*>(created.get());
        if (!b) return rejected(Reason::UnknownCapability);
        created.release();
        behavior.reset(b);
    }

    const ActivityId id = activityId(vehicle_, ++serial_);
    if (waits) {
        // kept as given, to be prepared afresh when it starts: what a start needs made now (a start allocates nothing)
        Waiting& w = *freeWaiting();
        w.used = true, w.support = false;
        makeRecord(w.record, id, index, options, axes, now);
        w.record.waiting = scheduled ? ActivityWait::Scheduled : ActivityWait::Queued;
        w.record.waitingFor = blocker;
        w.queued = ++queueSerial_;
        w.resumed = false;
        w.options = options;
        w.command = command;
        if (const auto* route = std::get_if<RouteCommand>(&command)) w.firstStart = route->start;
        w.behavior = std::move(behavior);
        w.waypoints.reserve(PathStore::kWaypoints), w.waypoints.assign(waypoints.begin(), waypoints.end());
        w.segments.reserve(PathStore::kSegments), w.segments.assign(segments.begin(), segments.end());
        w.shape = shape ? *shape : PatternShape{};
        w.curveShape = curveShape ? *curveShape : CurveShape{};
        w.loiters.reserve(PathStore::kRouteLoiters), w.loiters.assign(loiters.begin(), loiters.end());
        if (!config_->path) config_->path = std::make_unique<PathStore>();
        if (std::holds_alternative<RouteCommand>(command) && !routePlan_) routePlan_ = std::make_unique<route::Plan>();
        if (std::holds_alternative<CurveCommand>(command) && !curvePlan_) curvePlan_ = std::make_unique<route::Curve>();
        ++waitingCount_;
        CommandResult r = about(accepted(id, static_cast<std::uint16_t>(flags | kDeferred), true), detail);
        r.other = blocker;
        return r;
    }
    const auto* route = std::get_if<RouteCommand>(&setpoint);
    const double firstStart = route ? route->start : kHold;
    if ((route || std::holds_alternative<CurveCommand>(setpoint) || std::holds_alternative<PatternCommand>(setpoint)) && !config_->path)
        config_->path = std::make_unique<PathStore>();
    launch(Launch{nullptr, id, index, axes, flags, firstStart}, options, std::move(setpoint), std::move(behavior), route != nullptr, segments, now);
    schedule(state, now); // (what waits is arbitrated against it)
    return about(accepted(id, flags, true), detail);
}

CommandResult CapabilityHost::submit(const SupportCommand& command, const CommandOptions& options, const sim::VehicleState& state, double now) {
    if (pendingSuggestions_) materialize();
    details_.clear();
    const int found = catalog_->indexOf(command);
    if (found < 0) return rejected(missing(featureOf(command))); // the aircraft has no such effector: why
    const auto index = static_cast<std::size_t>(found);
    const CapabilityDescriptor& d = catalog_->descriptor(index);
    if (const Reason why = admits(index, options.source, options.controller); why != Reason::None) return rejected(why);
    if (options.precedenceOverride != kNoPrecedenceOverride && options.source == Source::Policy) return rejected(Reason::NotAllowed);
    if (const Reason why = checkWindow(options.window, now); why != Reason::None) return rejected(why);
    const bool checked = options.range != RangePolicy::None;
    if (checked) {
        if (const CapabilityStatus own = vehicleStatus(index, state); own.availability != Availability::Available) return rejected(own.reason);
        if (d.version < options.minVersion) return rejected(Reason::VersionUnsupported);
    }
    SupportCommand setpoint = command;
    CommandResult detail;
    CheckLog log{detail, options.range, &details_};
    if (checked)
        if (const Reason why = catalog_->check(index, setpoint, log); why != Reason::None) return about(rejected(why), detail);
    if (log.refused != Reason::None) return about(rejected(log.refused), detail);
    const std::uint16_t flags = detail.flags;
    const bool scheduled = options.window.startNotBefore > now;
    // the placards: no gear up on the ground, no gear or flaps out above their speeds (one that starts later: then)
    if (!scheduled)
        if (const Reason why = adapter_->admit(setpoint, state, *profile_); why != Reason::None) return rejected(why);
    const AxisMask axes = d.axes;
    if (options.axes && options.axes != axes) return rejected(Reason::InvalidAxes);
    ActivityId blocker = 0;
    Standing standing = Standing::Waits;
    if (!scheduled) {
        // the engines take thrust from the cascade: what flies the rest flies it apart
        if (const Reason why = checkAwareness(axes, Level::Actuator, false); why != Reason::None) return rejected(why);
        standing = arbitrate(axes, options.source, precedenceFor(index, options.precedenceOverride), options.rank, options.interrupt, kNewest, blocker);
        if (standing == Standing::Refused) return rejected(Reason::AuthorityHeld, 0, blocker);
    }
    const bool waits = standing != Standing::Takes;
    if (waits && waitingCount_ >= kWaiting) return rejected(Reason::QueueFull);
    if (options.validateOnly) {
        CommandResult r = valid(detail);
        if (waits) r.flags = static_cast<std::uint16_t>(r.flags | kDeferred), r.other = blocker;
        return r;
    }

    const ActivityId id = activityId(vehicle_, ++serial_);
    if (waits) {
        Waiting& w = *freeWaiting();
        w.used = true, w.support = true;
        makeRecord(w.record, id, index, options, axes, now);
        w.record.waiting = scheduled ? ActivityWait::Scheduled : ActivityWait::Queued;
        w.record.waitingFor = blocker;
        w.queued = ++queueSerial_;
        w.resumed = false;
        w.options = options;
        w.supportCommand = command;
        ++waitingCount_;
        CommandResult r = about(accepted(id, static_cast<std::uint16_t>(flags | kDeferred), true), detail);
        r.other = blocker;
        return r;
    }
    launchDirect(Launch{nullptr, id, index, axes, flags}, options, setpoint, now);
    schedule(state, now);
    return about(accepted(id, flags, true), detail);
}

CommandResult CapabilityHost::update(ActivityId activity, const RouteCommand& route, Span<const Waypoint> waypoints,
                                     const sim::VehicleState& state, Caller caller, Span<const RouteLoiter> loiters) noexcept {
    details_.clear();
    const int found = liveSlot(activity);
    if (found < 0) {
        if (Waiting* w = waitingEntry(activity)) return updateWaiting(*w, Command(route), waypoints, {}, state, caller, nullptr, nullptr, loiters);
        return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    }
    const auto s = static_cast<std::size_t>(found);
    if (const Reason why = addresses(records_[s], caller); why != Reason::None) return rejected(why, activity, activity);
    if (!isCascade(s)) return rejected(Reason::WrongCommandType, activity);
    if (!(catalog_->descriptor(records_[s].capability).interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity); // (as update() answers)
    auto* live = std::get_if<RouteCommand>(&config_->slots[s].command);
    if (!live || !routePlan_) return rejected(Reason::WrongCommandType, activity);
    // the options given replace the route's; its waypoints, new ones or those it has (read before they are rewritten)
    RouteCommand next = *live;
    if (!isHold(route.projection)) next.projection = route.projection;
    if (!isHold(route.repeat)) next.repeat = route.repeat;
    if (!isHold(route.end)) next.end = route.end;
    if (!isHold(route.start)) next.start = route.start;
    const PathStore* store = config_->path.get();
    const bool kept = waypoints.empty() && store; // (and its loiters: new waypoints come with theirs - 4.31)
    const Span<const Waypoint> points = kept ? Span<const Waypoint>(store->waypoints, store->count) : waypoints;
    const Span<const RouteLoiter> held = kept ? Span<const RouteLoiter>(store->routeLoiters, store->routeLoiterCount) : loiters;
    CommandResult result = accepted(activity);
    result.commandId = records_[s].commandId;
    CheckLog log{result, slots_[s].range, &details_};
    if (const Reason why = checkRoute(next, points, state, log, held); why != Reason::None) return about(rejected(why, activity), result);
    checkTerrain(Command(next), state, log);
    if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
    if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    writeRoute();
    if (!isHold(route.start)) slots_[s].firstStart = next.start;
    *live = next;
    ++config_->slots[s].revision;
    return result;
}

CommandResult CapabilityHost::update(ActivityId activity, const CurveCommand& curve, Span<const NurbsSegment> segments,
                                     const sim::VehicleState& state, Caller caller, const CurveShape* shape) noexcept {
    details_.clear();
    const int found = liveSlot(activity);
    if (found < 0) {
        if (Waiting* w = waitingEntry(activity)) return updateWaiting(*w, Command(curve), {}, segments, state, caller, nullptr, shape);
        return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    }
    const auto s = static_cast<std::size_t>(found);
    if (const Reason why = addresses(records_[s], caller); why != Reason::None) return rejected(why, activity, activity);
    if (!isCascade(s)) return rejected(Reason::WrongCommandType, activity);
    if (!(catalog_->descriptor(records_[s].capability).interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity); // (as update() answers)
    auto* live = std::get_if<CurveCommand>(&config_->slots[s].command);
    if (!live) return rejected(Reason::WrongCommandType, activity);
    // the options given replace the curve's; `append` is this UPDATE's own
    CurveCommand next = *live;
    const double* given[] = {&curve.latitudeRad,       &curve.longitudeRad,  &curve.altitudeM,    &curve.speedMinMs,    &curve.speedMaxMs,
                             &curve.durationS,         &curve.end,           &curve.altitudeReference, &curve.altitudeMinM, &curve.altitudeMaxM,
                             &curve.pointRotation,     &curve.pointOffsets,  &curve.pointZ};
    double* kept[] = {&next.latitudeRad, &next.longitudeRad, &next.altitudeM,    &next.speedMinMs,    &next.speedMaxMs,   &next.durationS, &next.end,
                      &next.altitudeReference, &next.altitudeMinM, &next.altitudeMaxM, &next.pointRotation, &next.pointOffsets, &next.pointZ};
    for (std::size_t i = 0; i < std::size(given); ++i)
        if (!isHold(*given[i])) *kept[i] = *given[i];
    next.append = curve.append;
    const bool appending = curve.append == 1.0;
    CommandResult result = accepted(activity);
    result.commandId = records_[s].commandId;
    CheckLog log{result, slots_[s].range, &details_};
    if (segments.empty()) { // its options alone: how it is flown, not where
        if (appending) {
            result.index = 0; // nothing to append
            return about(rejected(Reason::InvalidCurve, activity), result);
        }
        if (const int field = curveWhereField(curve, shape); field >= 0) { // (where it is changes with its segments: 4.27)
            result.index = static_cast<std::int16_t>(field);
            return about(rejected(Reason::InvalidParameter, activity), result);
        }
        if (const Reason why = checkCurveOptions(next, false, result); why != Reason::None) return about(rejected(why, activity), result);
        if (slots_[s].range != RangePolicy::None) {
            limitCurveSpeeds(next, log);
            if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
            if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
        }
    } else {
        const CurveShape held = config_->path ? config_->path->curveShape : CurveShape{};
        if (appending) { // (from its reference, its points read as its - A-GRA's append uses the preceding CenterReference: 4.27)
            next.latitudeRad = live->latitudeRad, next.longitudeRad = live->longitudeRad, next.altitudeM = live->altitudeM;
            next.altitudeReference = live->altitudeReference, next.altitudeMinM = live->altitudeMinM, next.altitudeMaxM = live->altitudeMaxM;
            next.pointRotation = live->pointRotation, next.pointOffsets = live->pointOffsets, next.pointZ = live->pointZ;
            curveShape_ = held, curveTurn_ = curveTurn(next, held);
            curveAttitude(next, held, curvePose_);
        } else { // a new curve: its shape merged, as a pattern's; a reference given alone has no value to read
            if (!isHold(curve.altitudeReference) && isHold(curve.altitudeM)) {
                result.index = 8;
                return about(rejected(Reason::InvalidParameter, activity), result);
            }
            curveShape_ = held;
            mergeCurveShape(next, curveShape_, curve, shape ? *shape : CurveShape{});
        }
        if (const Reason why = checkCurve(next, segments, appending, state, log); why != Reason::None) return about(rejected(why, activity), result);
        checkTerrain(Command(next), state, log);
        if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
        writeCurve(segments, appending);
    }
    next.append = kHold;
    *live = next;
    ++config_->slots[s].revision;
    return result;
}

CommandResult CapabilityHost::update(ActivityId activity, const Command& setpoint, const sim::VehicleState& state, Caller caller,
                                     const PatternShape* shape) noexcept {
    if (const auto* route = std::get_if<RouteCommand>(&setpoint)) return update(activity, *route, {}, state, caller);
    if (const auto* curve = std::get_if<CurveCommand>(&setpoint)) return update(activity, *curve, Span<const NurbsSegment>{}, state, caller);
    details_.clear();
    const int found = liveSlot(activity);
    if (found < 0) {
        if (Waiting* w = waitingEntry(activity)) return updateWaiting(*w, setpoint, {}, {}, state, caller, shape);
        return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    }
    const auto s = static_cast<std::size_t>(found);
    if (const Reason why = addresses(records_[s], caller); why != Reason::None) return rejected(why, activity, activity);
    if (!isCascade(s)) return rejected(Reason::WrongCommandType, activity);
    const ActivityRecord& record = records_[s];
    if (!(catalog_->descriptor(record.capability).interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity);
    SetpointSlot& slot = config_->slots[s];
    if (setpoint.index() != slot.command.index()) return rejected(Reason::WrongCommandType, activity);
    CommandResult result = accepted(activity);
    result.commandId = record.commandId;
    CheckLog log{result, slots_[s].range, &details_};
    if (const auto* next = std::get_if<HsaCommand>(&setpoint)) {
        // a partial hsa (docs/vehicle-interface.md, 4.4): the fields given replace the commanded
        // ones; a reference needs its value (an UPDATE has no state to take one from)
        auto bad = [&](std::int16_t field) {
            result.index = field;
            return about(rejected(Reason::InvalidParameter, activity), result);
        };
        auto code = [](double v, int count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
        if (!code(next->speedReference, static_cast<int>(SpeedReference::Count))) return bad(3);
        if (!code(next->altitudeReference, static_cast<int>(AltitudeReference::Count))) return bad(5);
        if (!code(next->speedOptimization, static_cast<int>(SpeedOptimization::Count))) return bad(6);
        if (!code(next->directionReference, static_cast<int>(DirectionReference::Count))) return bad(7);
        if (!isHold(next->headingRad) && !isHold(next->courseRad)) return bad(1);
        if (!isHold(next->directionReference) && isHold(next->headingRad) && isHold(next->courseRad)) return bad(0);
        if (!isHold(next->speedReference) && isHold(next->speed) && isHold(next->speedOptimization)) return bad(2);
        if (!isHold(next->altitudeReference) && isHold(next->altitudeM)) return bad(4);
        if (const Reason why = optimisable(next->speedOptimization, 6, result); why != Reason::None) return about(rejected(why, activity), result);
        HsaCommand merged = std::get<HsaCommand>(slot.command);
        mergeHsa(merged, *next);
        if (!isHold(merged.headingRad)) merged.headingRad = geo::wrapPi(merged.headingRad);
        if (!isHold(merged.courseRad)) merged.courseRad = geo::wrapPi(merged.courseRad);
        optimise(merged.speed, merged.speedReference, merged.speedOptimization, merged.altitudeM, merged.altitudeReference, state);
        if (slots_[s].range != RangePolicy::None) {
            Command checked = merged;
            if (const Reason why = catalog_->check(record.capability, checked, log); why != Reason::None) return about(rejected(why, activity), result);
            merged = std::get<HsaCommand>(checked);
            limitHsa(merged, log);
            checkTerrain(Command(merged), state, log);
            if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
            if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
        }
        std::get<HsaCommand>(slot.command) = merged;
        ++slot.revision;
        return result;
    }
    if (const auto* next = std::get_if<PatternCommand>(&setpoint)) return updatePattern(s, activity, *next, shape, state, result, log);
    if (slots_[s].range == RangePolicy::None) {
        assignSetpoint(slot.command, setpoint);
    } else {
        Command checked = setpoint; // no heap data: a behaviour takes no UPDATE
        if (const Reason why = catalog_->check(record.capability, checked, log); why != Reason::None) return about(rejected(why, activity), result);
        if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
        assignSetpoint(slot.command, checked);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    }
    ++slot.revision;
    return result;
}

CommandResult CapabilityHost::update(ActivityId activity, const SupportCommand& setpoint, Caller caller) noexcept {
    details_.clear();
    const int found = liveSlot(activity);
    if (found < 0) {
        if (Waiting* w = waitingEntry(activity)) return updateWaiting(*w, setpoint, caller);
        return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    }
    const auto s = static_cast<std::size_t>(found);
    if (const Reason why = addresses(records_[s], caller); why != Reason::None) return rejected(why, activity, activity);
    const ActivityRecord& record = records_[s];
    if (isCascade(s) || catalog_->indexOf(setpoint) != static_cast<int>(record.capability)) return rejected(Reason::WrongCommandType, activity);
    SupportCommand checked = setpoint;
    CommandResult result = accepted(activity);
    result.commandId = record.commandId;
    if (slots_[s].range != RangePolicy::None) {
        CheckLog log{result, slots_[s].range, &details_};
        if (const Reason why = catalog_->check(record.capability, checked, log); why != Reason::None) return about(rejected(why, activity), result);
        if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    }
    writeDirect(s, checked);
    if (!std::isnan(slots_[s].target)) slots_[s].target = supportGoal(checked);
    return result;
}

CommandResult CapabilityHost::cancel(ActivityId activity, const sim::VehicleState& state, double now, Caller caller) noexcept {
    CommandResult r; // (the pending suggestions stay until a call that may allocate: a NEW, an activity or a task command)
    r.status = CommandStatus::Canceled;
    r.activity = activity;
    const int found = liveSlot(activity);
    if (found < 0) {
        Waiting* w = waitingEntry(activity);
        if (!w) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
        if (const Reason why = addresses(w->record, caller); why != Reason::None) return rejected(why, activity, activity);
        r.commandId = w->record.commandId;
        endWaiting(*w, ActivityState::Canceled, Reason::Requested, now); // (it held nothing)
        return r;
    }
    const auto s = static_cast<std::size_t>(found);
    if (const Reason why = addresses(records_[s], caller); why != Reason::None) return rejected(why, activity, activity);
    r.commandId = records_[s].commandId;
    end(s, ActivityState::Canceled, Reason::Requested, 0, now);
    release(s); // its axes: the vehicle default...
    schedule(state, now); // ...or what waited for them
    return r;
}

CommandOptions CapabilityHost::optionsOf(std::size_t s) const noexcept {
    const ActivityRecord& r = records_[s];
    CommandOptions o;
    o.source = r.source;
    o.axes = r.axes;
    o.range = slots_[s].range;
    o.commandId = r.commandId;
    o.trace = r.trace;
    o.interactive = r.interactive;
    o.interrupt = r.interrupt;
    o.rank = r.rank;
    o.precedenceOverride = slots_[s].precedenceOverride;
    o.controller = r.controller;
    o.window = r.window;
    return o;
}

SupportCommand CapabilityHost::supportCommandOf(std::size_t s) const noexcept {
    const RuntimeConfig& c = *config_;
    if (s == kEnginesSlot) {
        EnginesCommand e;
        for (std::size_t i = 0; i < c.engines.size(); ++i) e.throttle[i] = c.engines[i];
        return e;
    }
    const SupportDemand& d = c.support[s - kSlotCount];
    switch (static_cast<Axis>(static_cast<std::size_t>(Axis::Flaps) + (s - kSlotCount))) {
    case Axis::Flaps: return FlapsCommand{d.value};
    case Axis::Gear: return GearCommand{d.value};
    case Axis::Brakes: return WheelBrakesCommand{d.value, d.value2};
    case Axis::Speedbrake: return SpeedbrakeCommand{d.value};
    default: return PitchTrimCommand{d.value};
    }
}

bool CapabilityHost::retire(std::size_t s, ActivityState state) {
    Waiting* w = freeWaiting();
    if (!w) return false;
    const ActivityRecord& r = records_[s];
    w->used = true;
    w->support = !isCascade(s);
    w->options = optionsOf(s);
    w->record = r;
    w->record.state = state;
    w->record.waiting = state == ActivityState::Pending ? ActivityWait::Queued : ActivityWait::None; // (the scheduler names what it waits for)
    w->record.waitingFor = 0;
    w->queued = ++queueSerial_; // behind what waits
    w->resumed = true;          // (its start window was its first start's)
    if (w->support) {
        w->supportCommand = supportCommandOf(s);
    } else {
        Command& flown = config_->slots[s].command;
        w->behavior = runtime_->uninstall(s); // kept: it starts afresh when it flies again
        w->waypoints.clear(), w->segments.clear(), w->loiters.clear();
        if (auto* route = std::get_if<RouteCommand>(&flown); route && config_->path) {
            const PathStore& store = *config_->path;
            w->waypoints.reserve(PathStore::kWaypoints), w->waypoints.assign(store.waypoints, store.waypoints + store.count);
            w->loiters.reserve(PathStore::kRouteLoiters), w->loiters.assign(store.routeLoiters, store.routeLoiters + store.routeLoiterCount);
            w->firstStart = isHold(slots_[s].firstStart) ? route->start : slots_[s].firstStart;
            // resumed at the point it flew to
            if (store.count && r.progress.segments) route->start = static_cast<double>(std::min<std::uint32_t>(r.progress.segment, store.count - 1));
        }
        if (std::holds_alternative<CurveCommand>(flown) && config_->path) {
            const PathStore& store = *config_->path;
            w->segments.reserve(PathStore::kSegments), w->segments.assign(store.segments, store.segments + store.segmentCount);
        }
        w->shape = std::holds_alternative<PatternCommand>(flown) && config_->path ? config_->path->pattern : PatternShape{};
        w->curveShape = std::holds_alternative<CurveCommand>(flown) && config_->path ? config_->path->curveShape : CurveShape{};
        w->command = std::move(flown);
    }
    if (!std::isnan(r.window.endNotAfter)) --windowed_; // (it no longer flies)
    ++waitingCount_;
    // its slot freed, its record kept: its axes to the vehicle default, or to what waits
    release(s);
    return true;
}

CommandResult CapabilityHost::activityCommand(ActivityId activity, ActivityCommand command, Rank rank, const sim::VehicleState& state, double now,
                                              Caller caller) {
    if (pendingSuggestions_) materialize();
    details_.clear();
    const int found = liveSlot(activity);
    Waiting* w = found < 0 ? waitingEntry(activity) : nullptr;
    if (found < 0 && !w) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found < 0 ? 0 : found);
    ActivityRecord& record = w ? w->record : records_[s];
    if (const Reason why = addresses(record, caller); why != Reason::None) return rejected(why, activity, activity);
    if (static_cast<unsigned>(command) >= static_cast<unsigned>(ActivityCommand::Count)) return rejected(Reason::InvalidParameter, activity);
    if (!record.interactive) return rejected(Reason::NotInteractive, activity);
    CommandResult result = accepted(activity);
    result.commandId = record.commandId;
    auto full = [&] {
        CommandResult r = rejected(Reason::QueueFull, activity);
        r.commandId = result.commandId;
        return r;
    };
    switch (command) {
    case ActivityCommand::Disable: // it flies nothing, kept until enabled
        if (!w) {
            if (!retire(s, ActivityState::Disabled)) return full();
        } else if (record.state != ActivityState::Disabled) {
            record.state = ActivityState::Disabled, record.waiting = ActivityWait::None, record.waitingFor = 0;
        }
        break;
    case ActivityCommand::Unassign: // its axes given up: it waits for them again, behind what waits
        if (!w && !retire(s, ActivityState::Pending)) return full();
        break; // (one that waits, or is disabled, holds nothing)
    case ActivityCommand::Enable: // a disabled one waits to start again; one live and enabled stays so
        if (w && record.state == ActivityState::Disabled) record.state = ActivityState::Pending, record.waiting = ActivityWait::Queued, w->queued = ++queueSerial_;
        break;
    case ActivityCommand::Reset: // over from its beginning
        if (w) {
            if (auto* route = std::get_if<RouteCommand>(&w->command); route && !w->support) route->start = w->firstStart;
        } else {
            record.state = ActivityState::Pending;
            record.progress = ActivityProgress{};
            if (record.runs) record.run = 1; // (a task's: its runs over from the first)
            slots_[s].outside = false, slots_[s].restartAt = kUnknown;
            if (auto* route = isCascade(s) ? std::get_if<RouteCommand>(&config_->slots[s].command) : nullptr; route && !isHold(slots_[s].firstStart))
                route->start = slots_[s].firstStart; // (from its first point, as commanded, not where it resumed)
            if (isCascade(s)) ++config_->slots[s].generation, ++config_->slots[s].revision; // its behaviour begins afresh
        }
        break;
    case ActivityCommand::Delete: // a sticky disable: it ends, and cannot be enabled
        if (w) {
            endWaiting(*w, ActivityState::Deleted, Reason::Requested, now);
        } else {
            end(s, ActivityState::Deleted, Reason::Requested, 0, now);
            release(s);
        }
        break;
    case ActivityCommand::ChangeRank: record.rank = rank; break; // what it contests, arbitrated afresh below
    default: break;
    }
    schedule(state, now);
    return result;
}

CommandResult CapabilityHost::updateWaiting(Waiting& w, const Command& setpoint, Span<const Waypoint> waypoints, Span<const NurbsSegment> segments,
                                            const sim::VehicleState& state, Caller caller, const PatternShape* shape,
                                            const CurveShape* curveShape, Span<const RouteLoiter> loiters) noexcept {
    const ActivityRecord& record = w.record;
    const ActivityId activity = record.id;
    if (const Reason why = addresses(record, caller); why != Reason::None) return rejected(why, activity, activity);
    if (w.support) return rejected(Reason::WrongCommandType, activity);
    if (!(catalog_->descriptor(record.capability).interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity);
    if (setpoint.index() != w.command.index()) return rejected(Reason::WrongCommandType, activity);
    CommandResult result = accepted(activity);
    result.commandId = record.commandId;
    // what it will fly: the fields given replace its command's (a mode's merged, a level's replaced, a route's and a
    // curve's options kept where left out), then checked as its NEW was, from where the aircraft is now
    Command next = w.command;
    PatternShape nextShape = w.shape;
    CurveShape nextCurveShape = w.curveShape;
    Span<const Waypoint> points(w.waypoints.data(), w.waypoints.size());
    Span<const RouteLoiter> held(w.loiters.data(), w.loiters.size());
    Span<const NurbsSegment> pieces(w.segments.data(), w.segments.size());
    std::array<NurbsSegment, PathStore::kSegments> joined; // (a curve appended to: the two together)
    if (const auto* route = std::get_if<RouteCommand>(&setpoint)) {
        auto& kept = std::get<RouteCommand>(next);
        for (const auto& [from, to] : {std::pair{route->projection, &kept.projection}, {route->repeat, &kept.repeat}, {route->end, &kept.end},
                                       {route->start, &kept.start}})
            if (!isHold(from)) *to = from;
        if (!waypoints.empty()) points = waypoints, held = loiters; // (new waypoints come with their loiters: 4.31)
    } else if (const auto* curve = std::get_if<CurveCommand>(&setpoint)) {
        auto& kept = std::get<CurveCommand>(next);
        const double* from[] = {&curve->latitudeRad,   &curve->longitudeRad, &curve->altitudeM,         &curve->speedMinMs,   &curve->speedMaxMs,
                                &curve->durationS,     &curve->end,          &curve->altitudeReference, &curve->altitudeMinM, &curve->altitudeMaxM,
                                &curve->pointRotation, &curve->pointOffsets, &curve->pointZ};
        double* to[] = {&kept.latitudeRad, &kept.longitudeRad, &kept.altitudeM,    &kept.speedMinMs,    &kept.speedMaxMs,   &kept.durationS, &kept.end,
                        &kept.altitudeReference, &kept.altitudeMinM, &kept.altitudeMaxM, &kept.pointRotation, &kept.pointOffsets, &kept.pointZ};
        if (!isHold(curve->altitudeReference) && isHold(curve->altitudeM)) { // (a reference given alone has no value to read: 4.27)
            result.index = 8;
            return about(rejected(Reason::InvalidParameter, activity), result);
        }
        mergeCurveShape(kept, nextCurveShape, *curve, curveShape ? *curveShape : CurveShape{});
        for (std::size_t i = 0; i < std::size(from); ++i)
            if (!isHold(*from[i])) *to[i] = *from[i];
        const bool appending = curve->append == 1.0;
        if (!isHold(curve->append) && !appending && curve->append != 0.0) { // (as a live curve's UPDATE)
            result.index = 7;
            return about(rejected(Reason::InvalidParameter, activity), result);
        }
        if (appending && segments.empty()) {
            result.index = 0; // nothing to append
            return about(rejected(Reason::InvalidCurve, activity), result);
        }
        if (appending) {
            if (w.segments.size() + segments.size() > joined.size()) {
                result.index = static_cast<std::int16_t>(joined.size() - w.segments.size());
                return about(rejected(Reason::InvalidCurve, activity), result);
            }
            std::copy(w.segments.begin(), w.segments.end(), joined.begin());
            std::copy(segments.begin(), segments.end(), joined.begin() + static_cast<std::ptrdiff_t>(w.segments.size()));
            pieces = Span<const NurbsSegment>(joined.data(), w.segments.size() + segments.size());
        } else if (!segments.empty()) {
            pieces = segments;
        }
        kept.append = kHold; // (it starts as one curve)
    } else if (const auto* hsa = std::get_if<HsaCommand>(&setpoint)) {
        // a partial hsa (docs/vehicle-interface.md, 4.4): the fields given replace the commanded ones
        auto bad = [&](std::int16_t field) {
            result.index = field;
            return about(rejected(Reason::InvalidParameter, activity), result);
        };
        auto code = [](double v, int count) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); };
        if (!code(hsa->speedReference, static_cast<int>(SpeedReference::Count))) return bad(3);
        if (!code(hsa->altitudeReference, static_cast<int>(AltitudeReference::Count))) return bad(5);
        if (!code(hsa->directionReference, static_cast<int>(DirectionReference::Count))) return bad(7);
        if (!isHold(hsa->headingRad) && !isHold(hsa->courseRad)) return bad(1);
        if (!isHold(hsa->directionReference) && isHold(hsa->headingRad) && isHold(hsa->courseRad)) return bad(0);
        mergeHsa(std::get<HsaCommand>(next), *hsa);
    } else if (const auto* pattern = std::get_if<PatternCommand>(&setpoint)) {
        const PatternShape given = shape ? *shape : PatternShape{};
        if (const Reason why = checkPattern(*pattern, true, result); why != Reason::None) return about(rejected(why, activity), result);
        if (const Reason why = checkShape(*pattern, given, true, result); why != Reason::None) return about(rejected(why, activity), result);
        mergePattern(std::get<PatternCommand>(next), nextShape, *pattern, given);
    } else {
        assignSetpoint(next, setpoint); // a level's: replaced
    }
    CheckLog log{result, w.options.range, &details_};
    Command probe = next; // (fixed-size: a behaviour takes no UPDATE)
    if (const Reason why = prepare(record.capability, probe, points, pieces, state, log, &nextShape, &nextCurveShape, held); why != Reason::None)
        return about(rejected(why, activity), result);
    if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
    // kept for its start (room reserved at its NEW)
    if (const auto* route = std::get_if<RouteCommand>(&setpoint); route && !isHold(route->start)) w.firstStart = route->start;
    if (points.data() != w.waypoints.data()) w.waypoints.assign(points.begin(), points.end());
    if (held.data() != w.loiters.data()) w.loiters.assign(held.begin(), held.end());
    if (pieces.data() != w.segments.data()) w.segments.assign(pieces.begin(), pieces.end());
    w.shape = nextShape, w.curveShape = nextCurveShape;
    assignSetpoint(w.command, next);
    return result;
}

CommandResult CapabilityHost::updateWaiting(Waiting& w, const SupportCommand& setpoint, Caller caller) noexcept {
    const ActivityRecord& record = w.record;
    const ActivityId activity = record.id;
    if (const Reason why = addresses(record, caller); why != Reason::None) return rejected(why, activity, activity);
    if (!w.support || catalog_->indexOf(setpoint) != static_cast<int>(record.capability)) return rejected(Reason::WrongCommandType, activity);
    CommandResult result = accepted(activity);
    result.commandId = record.commandId;
    if (w.options.range != RangePolicy::None) {
        SupportCommand checked = setpoint;
        CheckLog log{result, w.options.range, &details_};
        if (const Reason why = catalog_->check(record.capability, checked, log); why != Reason::None) return about(rejected(why, activity), result);
        if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
    }
    w.supportCommand = setpoint; // (as given: checked again when it starts)
    return result;
}

CommandResult CapabilityHost::command(const Command& command, const sim::VehicleState& state, double now) {
    // The fast path: the same capability as the live legacy activity (a
    // behaviour command re-creates its behaviour, as it always has).
    if (updateLegacy(command)) return accepted(legacy_);
    static constexpr CommandOptions kLegacy = [] {
        CommandOptions o;
        o.source = Source::Policy;
        o.axes = kLegacyAxes;
        o.range = RangePolicy::None;
        return o;
    }();
    const CommandResult r = submitWith(command, {}, {}, kLegacy, state, now, false);
    if (r.accepted()) {
        legacy_ = r.activity;
        legacySlot_ = liveSlot(legacy_);
    } else if (const auto* b = std::get_if<BehaviorCommand>(&command); b && r.reason == Reason::UnknownCapability) {
        LOG_ERROR("control") << "unknown behaviour '" << b->id << "'; holding the current command";
    }
    return r;
}

const ActivityRecord* CapabilityHost::activity(ActivityId activity) const noexcept {
    if (!activity) return nullptr;
    if (const int s = liveSlot(activity); s >= 0) return &records_[static_cast<std::size_t>(s)];
    if (const Waiting* w = waitingEntry(activity)) return &w->record;
    for (std::size_t i = 0; i < recentCount_; ++i) {
        const ActivityRecord& r = recent_[(recentNext_ + kRecent - 1 - i) % kRecent];
        if (r.id == activity) return &r;
    }
    return nullptr;
}

std::vector<ActivityRecord> CapabilityHost::activities() const {
    std::vector<ActivityRecord> out;
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live) out.push_back(records_[s]);
    if (waitingCount_) { // what waits to start, oldest first
        const std::size_t first = out.size();
        for (const Waiting& w : *waiting_)
            if (w.used && !w.suggested) out.push_back(w.record);
        std::sort(out.begin() + static_cast<std::ptrdiff_t>(first), out.end(), [](const ActivityRecord& a, const ActivityRecord& b) { return a.id < b.id; });
    }
    for (std::size_t i = 0; i < recentCount_; ++i) out.push_back(recent_[(recentNext_ + kRecent - 1 - i) % kRecent]);
    return out;
}

void CapabilityHost::end(std::size_t s, ActivityState state, Reason reason, ActivityId by, double now) noexcept {
    ActivityRecord& record = records_[s];
    record.state = state;
    record.reason = reason;
    record.by = by;
    record.endTime = now;
    if (!std::isnan(record.window.endNotAfter)) --windowed_;
    slots_[s].live = false; // the runtime flies its residual hold until another activity takes the axes
    if (slots_[s].activity == legacy_) legacySlot_ = -1;
    recent_[recentNext_] = record;
    recentNext_ = (recentNext_ + 1) % kRecent;
    recentCount_ = std::min(recentCount_ + 1, kRecent);
    noteEnd(record);
}

void CapabilityHost::release(std::size_t s) noexcept {
    RuntimeConfig& config = *config_;
    if (s == kEnginesSlot) {
        for (std::size_t a = 0; a < kPrimaryAxisCount; ++a)
            if (config.owner[a] == RuntimeConfig::kEngines) {
                config.owner[a] = RuntimeConfig::kNone;
                ++config.letGo[a];
            }
        config.engines.fill(kHold);
    } else if (isSupport(s)) {
        const auto axis = static_cast<std::size_t>(Axis::Flaps) + (s - kSlotCount);
        if (config.owner[axis] == RuntimeConfig::kSupport) config.owner[axis] = RuntimeConfig::kNone;
        SupportDemand& demand = config.support[s - kSlotCount];
        demand.value = demand.value2 = kHold;
        ++demand.revision;
    } else {
        for (std::size_t a = 0; a < kAxisCount; ++a)
            if (config.owner[a] == s) {
                config.owner[a] = RuntimeConfig::kNone;
                if (a < kPrimaryAxisCount) ++config.letGo[a]; // the default's hold captures it afresh
            }
        config.slots[s].axes = 0;
        runtime_->install(s, nullptr);
        runtime_->report().slots[s] = SlotReport{}; // nothing it reported carries over to the slot's next activity
    }
    ++config.revision;
    if (static_cast<int>(s) == legacySlot_) legacySlot_ = -1;
    slots_[s] = Slot{};
}

bool CapabilityHost::afterStep(const sim::VehicleState& state, const EffectorPositions& positions, double now) noexcept {
    refreshPerformance();
    if (state.diverged != divergedSeen_) divergedSeen_ = state.diverged, ++controlRevision_; // every capability's availability changed
    RuntimeReport& report = runtime_->report();
    const RuntimeConfig& config = *config_;
    // Whether the state went past the envelope this step by more than its limiters overshoot: what a
    // manoeuvre is judged by (docs/flight-autonomy.md, 6). Per limit: 0.5 g; 3 deg of angle of attack (a
    // fly-by-wire limiter holds within a degree or two); none for bank and pitch, which a loop and a roll
    // pass by design; 0.35 rad/s of roll rate; 3 m/s below the least airspeed, 5 m/s above the most; Mach 0.02.
    static constexpr float kTolerance[kLimitCount] = {0.5f, 0.5f, 0.0524f, 1e30f, 1e30f, 1e30f, 0.35f, 3.0f, 5.0f, 0.02f};
    bool outside = false;
    for (std::size_t l = 0; l < kLimitCount; ++l)
        if (report.limits[l].exceededUpdates && report.limits[l].worstExcess > kTolerance[l]) outside = true;
    for (std::size_t s = 0; s < kActivities; ++s) {
        Slot& slot = slots_[s];
        if (!slot.activity) continue; // nothing flies here (and nothing reported)
        if (slot.live) {
            ActivityRecord& record = records_[s];
            if (!std::isnan(slot.restartAt) && now >= slot.restartAt) restartRun(s); // a task's next run (4.11)
            const bool flown = isCascade(s) ? report.updates > 0 && report.slots[s].generation == config.slots[s].generation
                                            : report.updates > 0;
            if (record.state == ActivityState::Pending && flown) record.state = ActivityState::Active;
            const std::uint16_t behaviorFlags = isCascade(s) ? report.slots[s].flags : std::uint16_t{0}; // what its behaviour held back
            const std::uint16_t flags = static_cast<std::uint16_t>(slot.flags | flagsOn(report, record.axes) | behaviorFlags);
            record.constraints = flags;
            record.constraintsSeen = static_cast<std::uint16_t>(record.constraintsSeen | flags);
            slot.flags = 0;
            // how far a behaviour has got, before it may complete (docs/vehicle-interface.md, 5.3)
            if (isCascade(s) && config.slots[s].level == Level::Behavior) runtime_->progress(s, record.progress);
            if (outside) slot.outside = true;
            if (state.diverged) {
                end(s, ActivityState::Failed, Reason::Diverged, 0, now);
            } else if (record.state == ActivityState::Active && isCascade(s)) {
                const SlotReport& events = report.slots[s];
                if (events.events & kFailed) end(s, ActivityState::Failed, events.failure, 0, now);
                else if (events.events & kFinished) {
                    // a manoeuvre flown past the envelope is not one completed (docs/flight-autonomy.md, 6)
                    if (slot.outside && catalog_->withinEnvelope(record.capability))
                        end(s, ActivityState::Failed, Reason::BehaviorFailed, 0, now);
                    else if (record.run < record.runs) { // a task's run done, runs to come (4.11): the next, its interval later
                        if (std::isnan(slot.restartAt)) { // (a behaviour done says so each step until it begins again)
                            const double interval = repeatInterval(record.id);
                            slot.restartAt = now + (std::isnan(interval) ? 0.0 : interval);
                            if (!(slot.restartAt > now)) restartRun(s);
                        }
                    }
                    else if (record.window.endCritical() && now < record.window.endNotBefore) // done before its critical end window (4.9)
                        end(s, ActivityState::Failed, Reason::TimeConstraint, 0, now);
                    else end(s, ActivityState::Completed, Reason::GoalReached, 0, now);
                }
            } else if (record.state == ActivityState::Active && isSupport(s) && !std::isnan(slot.target)) {
                // gear or flaps: done when they are there (and held there, as a residual)
                const std::size_t axis = static_cast<std::size_t>(Axis::Flaps) + (s - kSlotCount);
                const double position = axis == static_cast<std::size_t>(Axis::Gear) ? positions.gear : positions.flaps;
                if (!std::isnan(position) && std::abs(position - slot.target) < 0.01) {
                    if (record.window.endCritical() && now < record.window.endNotBefore) end(s, ActivityState::Failed, Reason::TimeConstraint, 0, now);
                    else end(s, ActivityState::Completed, Reason::GoalReached, 0, now);
                }
            }
        }
        if (isCascade(s)) {
            report.slots[s].events = 0;
            report.slots[s].failure = Reason::None;
            report.slots[s].flags = 0;
        }
    }
    // the envelope: what was limited, how long and how far the state was beyond each limit
    for (std::size_t l = 0; l < kLimitCount; ++l) {
        const LimitReport& r = report.limits[l];
        if (!r.limitedUpdates && !r.exceededUpdates) continue;
        LimitStatus& status = envelope_.limits[l];
        status.limitedUpdates += r.limitedUpdates;
        status.exceededUpdates += r.exceededUpdates;
        status.exceededS += r.exceededUpdates * controlPeriodS_;
        status.worstExcess = std::max(status.worstExcess, static_cast<double>(r.worstExcess));
    }
    report.clearAccumulators();
    keepWindows(now);           // the time windows (docs/flight-autonomy.md, 4.9)...
    return schedule(state, now); // ...and what waits, started when it may
}

void CapabilityHost::onReset() noexcept {
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live) {
            records_[s].state = ActivityState::Pending; // the runtime starts it again
            slots_[s].flags = 0;
        }
}

} // namespace fsim::control
