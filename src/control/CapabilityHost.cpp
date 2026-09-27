#include "control/CapabilityHost.h"

#include "control/Atmosphere.h"
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

CommandResult accepted(ActivityId activity, std::uint16_t flags = 0) noexcept {
    CommandResult r;
    r.status = CommandStatus::Accepted;
    r.activity = activity;
    r.flags = flags;
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
/// None within it, or where either is not known; beyond it, clamped to it
/// (kClamped) or, with Reject, PerformanceLimit - `detail` naming the field
/// or waypoint `index` and the limit (the first clamp's, when clamped).
Reason bound(double& value, double limit, bool above, std::int16_t index, Constraint constraint, RangePolicy range, std::uint16_t& flags,
             CommandResult& detail) noexcept {
    if (std::isnan(limit) || isHold(value) || (above ? value <= limit : value >= limit)) return Reason::None;
    if (range == RangePolicy::Reject || !(flags & kClamped)) detail.index = index, detail.constraint = constraint;
    if (range == RangePolicy::Reject) return Reason::PerformanceLimit;
    value = limit;
    flags |= kClamped;
    return Reason::None;
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
    // what the aircraft can do, from its profile and the loops it flies with (docs/vehicle-interface.md, 7.1)
    performance_ = adapter.performance(profile, runtime);
    performance_.revision = config_->performance.revision + 1;
    config_->performance = performance_;
    ++config_->revision;
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
    if (!isHold(c.headingRad) && !isHold(c.courseRad)) return bad(1); // one direction, a heading or a course
    for (const auto& [v, field] : {std::pair<double, std::int16_t>{c.headingRad, 0}, {c.courseRad, 1}, {c.speed, 2}, {c.altitudeM, 4}})
        if (!isHold(v) && !std::isfinite(v)) return bad(field);
    // what it continues: the live hsa's commands, else what the aircraft flies now - a
    // rotorcraft's speed over the ground (a hover stays put), a wing's through the air
    HsaCommand base;
    if (const HsaCommand* live = liveHsa()) {
        base = *live;
    } else {
        const auto reference = (adapter_->features() & kFeatureHover) ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed;
        base.headingRad = state.eulerRad[2];
        base.speedReference = static_cast<double>(reference);
        base.speed = speedNow(reference, state);
        base.altitudeReference = static_cast<double>(AltitudeReference::Msl);
        base.altitudeM = state.altitudeMslM;
    }
    // a reference given alone: the aircraft's own value in it now (hold the Mach it flies)
    if (!isHold(c.speedReference) && isHold(c.speed)) c.speed = speedNow(static_cast<SpeedReference>(static_cast<int>(c.speedReference)), state);
    if (!isHold(c.altitudeReference) && isHold(c.altitudeM))
        c.altitudeM = altitudeNow(static_cast<AltitudeReference>(static_cast<int>(c.altitudeReference)), state);
    mergeHsa(base, c); // a value given alone is in the reference it continues
    c = base;
    if (!isHold(c.headingRad)) c.headingRad = geo::wrapPi(c.headingRad);
    if (!isHold(c.courseRad)) c.courseRad = geo::wrapPi(c.courseRad);
    return Reason::None;
}

Reason CapabilityHost::limitHsa(HsaCommand& c, RangePolicy range, std::uint16_t& flags, CommandResult& detail) const noexcept {
    return limitFlight(c.speed, c.speedReference, c.altitudeM, c.altitudeReference, range, flags, detail, 2, 4);
}

Reason CapabilityHost::limitFlight(double& speed, double speedReference, double& altitude, double altitudeReference, RangePolicy range,
                                   std::uint16_t& flags, CommandResult& detail, std::int16_t speedIndex, std::int16_t altitudeIndex) const noexcept {
    const Performance& f = performance_;
    auto limit = [&](double& value, double most, bool above, std::int16_t index, Constraint constraint) {
        return bound(value, most, above, index, constraint, range, flags, detail);
    };
    // the altitude: under the ceiling; above ground, above it
    if (aboveGround(altitudeReference)) {
        if (const Reason r = limit(altitude, 0.0, false, altitudeIndex, Constraint::MinAltitude); r != Reason::None) return r;
    } else if (const Reason r = limit(altitude, f.ceilingM, true, altitudeIndex, Constraint::MaxAltitude); r != Reason::None) {
        return r;
    }
    // the speed, within the envelope's calibrated speeds and Mach at the altitude it asks for (the
    // standard atmosphere's); a rotorcraft's ground speed within its fastest
    const auto reference = static_cast<SpeedReference>(static_cast<int>(orHold(speedReference, 0.0)));
    const double h = isHold(altitude) || aboveGround(altitudeReference) ? 0.0 : altitude;
    const std::int16_t i = speedIndex;
    switch (reference) {
    case SpeedReference::GroundSpeed:
        return f.hovers ? limit(speed, f.maxGroundSpeedMs, true, i, Constraint::MaxAirspeed) : Reason::None;
    case SpeedReference::CalibratedAirspeed:
        if (const Reason r = limit(speed, f.minCasMs, false, i, Constraint::MinAirspeed); r != Reason::None) return r;
        if (const Reason r = limit(speed, f.maxCasMs, true, i, Constraint::MaxAirspeed); r != Reason::None) return r;
        return limit(speed, isa::calibratedFromTrue(f.maxTasMs, h), true, i, Constraint::MaxAirspeed);
    case SpeedReference::TrueAirspeed:
        if (const Reason r = limit(speed, isa::trueFromCalibrated(f.minCasMs, h), false, i, Constraint::MinAirspeed); r != Reason::None) return r;
        if (const Reason r = limit(speed, isa::trueFromCalibrated(f.maxCasMs, h), true, i, Constraint::MaxAirspeed); r != Reason::None) return r;
        if (const Reason r = limit(speed, f.maxMach * isa::speedOfSound(h), true, i, Constraint::MaxAirspeed); r != Reason::None) return r;
        return limit(speed, f.maxTasMs, true, i, Constraint::MaxAirspeed);
    case SpeedReference::Mach: {
        const double a = isa::speedOfSound(h);
        if (const Reason r = limit(speed, isa::trueFromCalibrated(f.minCasMs, h) / a, false, i, Constraint::MinAirspeed); r != Reason::None) return r;
        if (const Reason r = limit(speed, isa::trueFromCalibrated(f.maxCasMs, h) / a, true, i, Constraint::MaxAirspeed); r != Reason::None) return r;
        if (const Reason r = limit(speed, f.maxMach, true, i, Constraint::MaxAirspeed); r != Reason::None) return r;
        return limit(speed, f.maxTasMs / a, true, i, Constraint::MaxAirspeed);
    }
    default: return Reason::None;
    }
}

Reason CapabilityHost::checkRoute(RouteCommand& c, Span<const Waypoint> waypoints, const sim::VehicleState& state, RangePolicy range,
                                  std::uint16_t& flags, CommandResult& detail) {
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
    if (const Reason r = route::complete(p.points, waypoints.data(), count, c.repeat == 1.0, state, performance_, hovers, which); r != Reason::None) {
        detail.index = which;
        return r;
    }
    p.count = count;
    p.start = static_cast<std::uint32_t>(c.start);
    p.repeat = c.repeat == 1.0;
    p.rhumb = c.projection == static_cast<double>(Projection::Rhumb);
    p.end = static_cast<EndBehavior>(static_cast<int>(c.end));
    if (range == RangePolicy::None) return Reason::None; // (what it flies, the behaviour plans from where it starts)
    const Performance& f = performance_;
    // the altitude a segment climbs from: the point before's, in the same reference; the start's, the aircraft's
    auto from = [&](std::uint32_t i, bool entry) {
        const Waypoint& w = p.points[i];
        if (entry) return altitudeNow(static_cast<AltitudeReference>(static_cast<int>(w.altitudeReference)), state);
        const Waypoint& b = p.points[p.prev(i)];
        return b.altitudeReference == w.altitudeReference ? b.altitudeM : kHold;
    };
    for (std::uint32_t i = 0; i < count; ++i) {
        Waypoint& w = p.points[i];
        const auto index = static_cast<std::int16_t>(i);
        if (const Reason r = limitFlight(w.speed, w.speedReference, w.altitudeM, w.altitudeReference, range, flags, detail, index, index); r != Reason::None)
            return r;
        // its turn's bank within the aircraft's (a rotorcraft's, its tilt)
        if (const Reason r = bound(w.maxBankRad, hovers ? f.maxTiltRad : f.maxBankRad, true, index, Constraint::MaxOrientation, range, flags, detail);
            r != Reason::None)
            return r;
        // a climb or descent rate within the aircraft's
        if (!isHold(w.climbRateMs)) {
            const double h0 = from(i, i == p.start);
            const bool descends = !isHold(h0) && w.altitudeM < h0;
            if (const Reason r = bound(w.climbRateMs, descends ? f.maxDescentMs : f.maxClimbMs, true, index,
                                       descends ? Constraint::MaxDescentRate : Constraint::MaxClimbRate, range, flags, detail);
                r != Reason::None)
                return r;
        }
    }
    WindEstimate wind;
    wind.update(state, 0.0);
    route::plan(p, state.latitudeRad, state.longitudeRad, state.altitudeMslM, std::hypot(wind.northMs, wind.eastMs), f, hovers);
    auto flown = [&p](std::uint32_t i) { return p.repeat || i >= p.start; }; // (a route that does not repeat flies nothing before its start)
    // a fly-by turn too big for its legs: flown smaller, or refused - the first
    // one flown (the start's from the entry, then each after it)
    for (std::uint32_t k = 0; k <= count; ++k) {
        const std::uint32_t i = k == 0 ? p.start : k - 1;
        if (k > 0 && (!flown(i) || (i == p.start && !p.repeat))) continue;
        if (!(k == 0 ? p.entryTurn : p.turns[i]).shrunk) continue;
        if (range == RangePolicy::Reject || !(flags & kClamped)) detail.index = static_cast<std::int16_t>(i), detail.constraint = Constraint::None;
        if (range == RangePolicy::Reject) return Reason::InvalidWaypoint;
        flags |= kClamped;
    }
    // a gradient steeper than the aircraft climbs or descends (point to point, above sea level): flown at its rate, or refused
    for (std::uint32_t i = 0; i < count; ++i) {
        Waypoint& w = p.points[i];
        if (!flown(i) || !isHold(w.climbRateMs) || aboveGround(w.altitudeReference)) continue;
        const double v = route::plannedSpeed(w.speed, w.speedReference, w.altitudeM);
        // the start's segment from the aircraft; any other, and the start's after a lap, from the point before
        for (const bool entry : {true, false}) {
            if (entry ? i != p.start : (i == p.start && !p.repeat)) continue;
            const route::Leg& leg = entry ? p.entry : p.legs[i];
            const double h0 = from(i, entry);
            if (isHold(h0) || !(leg.lengthM > 1.0)) continue;
            const bool descends = w.altitudeM < h0;
            const double most = descends ? f.maxDescentMs : f.maxClimbMs;
            if (std::isnan(most) || std::abs(w.altitudeM - h0) * v / leg.lengthM <= most) continue;
            if (range == RangePolicy::Reject || !(flags & kClamped))
                detail.index = static_cast<std::int16_t>(i), detail.constraint = descends ? Constraint::MaxDescentRate : Constraint::MaxClimbRate;
            if (range == RangePolicy::Reject) return Reason::PerformanceLimit;
            w.climbRateMs = most;
            flags |= kClamped;
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
    if (merge && !isHold(c.altitudeReference) && isHold(c.altitudeM)) return bad(3); // an UPDATE has no state to take one from
    if (merge && !isHold(c.speedReference) && isHold(c.speed)) return bad(9);
    return Reason::None;
}

Reason CapabilityHost::limitPattern(PatternCommand& c, RangePolicy range, std::uint16_t& flags, CommandResult& detail) const noexcept {
    if (const Reason r = limitFlight(c.speed, c.speedReference, c.altitudeM, c.altitudeReference, range, flags, detail, 9, 3); r != Reason::None) return r;
    // a radius the aircraft can fly at its speed: a wing's at its full bank, a rotorcraft's a metre
    const Performance& f = performance_;
    double least = 1.0;
    if (!f.hovers && std::isfinite(f.maxBankRad) && f.maxBankRad > 0.0) {
        const double h = c.altitudeReference == static_cast<double>(AltitudeReference::AboveGround) ? 0.0 : c.altitudeM;
        const double v = route::plannedSpeed(c.speed, c.speedReference, h);
        least = v * v / (9.80665 * std::tan(f.maxBankRad));
    }
    return bound(c.radiusM, least, false, 5, Constraint::MaxOrientation, range, flags, detail);
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
    return Reason::None;
}

double CapabilityHost::fastest(double altitudeM) const noexcept {
    const Performance& f = performance_;
    if (adapter_->features() & kFeatureHover) return f.maxGroundSpeedMs;
    // (fmin passes over the NaN of a limit not known)
    return std::fmin(std::fmin(isa::trueFromCalibrated(f.maxCasMs, altitudeM), f.maxMach * isa::speedOfSound(altitudeM)), f.maxTasMs);
}

Reason CapabilityHost::limitCurveSpeeds(CurveCommand& c, RangePolicy range, std::uint16_t& flags, CommandResult& detail) const noexcept {
    const Performance& f = performance_;
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    const double h = orHold(c.altitudeM, 0.0);
    const double least = hovers ? 0.0 : isa::trueFromCalibrated(f.minCasMs, h); // (NaN: no limit)
    const double most = fastest(h);
    // a range it can fly within: no faster at its least than it flies, no slower at its most
    // (a least below its slowest, a most above its fastest, leave it room)
    if (const Reason r = bound(c.speedMinMs, most, true, 3, Constraint::MaxAirspeed, range, flags, detail); r != Reason::None) return r;
    return bound(c.speedMaxMs, least, false, 4, Constraint::MinAirspeed, range, flags, detail);
}

Reason CapabilityHost::checkCurve(CurveCommand& c, Span<const BezierSegment> segments, bool appending, const sim::VehicleState& state,
                                  RangePolicy range, std::uint16_t& flags, CommandResult& detail) {
    if (const Reason r = checkCurveOptions(c, appending, detail); r != Reason::None) return r;
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    if (!appending) { // what it leaves out: the reference where the aircraft is
        if (isHold(c.latitudeRad)) c.latitudeRad = state.latitudeRad, c.longitudeRad = state.longitudeRad;
        if (isHold(c.altitudeM)) c.altitudeM = state.altitudeMslM;
    }
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
    auto gap = [](const BezierSegment& a, const BezierSegment& b) { // a's end to b's start
        return std::sqrt(std::pow(b.north[0] - a.north[5], 2) + std::pow(b.east[0] - a.east[5], 2) + std::pow(b.down[0] - a.down[5], 2));
    };
    for (std::size_t i = 0; i < n; ++i) {
        const BezierSegment& s = segments[i];
        for (int k = 0; k < 6; ++k)
            if (!std::isfinite(s.north[k]) || !std::isfinite(s.east[k]) || !std::isfinite(s.down[k])) return invalid(i);
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
    }
    if (range == RangePolicy::None) return Reason::None;
    // its length, and the speed it will fly: the rest of the duration's, else its range's fastest
    if (!curvePlan_) curvePlan_ = std::make_unique<route::Curve>();
    route::Curve& k = *curvePlan_;
    k.count = static_cast<std::uint32_t>(held + n);
    if (held) std::copy_n(store->segments, held, k.segments);
    std::copy_n(segments.data(), n, k.segments + held);
    k.measure(0);
    const Performance& f = performance_;
    if (const Reason r = limitCurveSpeeds(c, range, flags, detail); r != Reason::None) return r;
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
        if (const Reason r = bound(w, most, true, 5, Constraint::MaxAirspeed, range, flags, detail); r != Reason::None) return r;
        if (const Reason r = bound(w, least, false, 5, Constraint::MinAirspeed, range, flags, detail); r != Reason::None) return r;
    }
    if (!isHold(c.speedMaxMs)) fast = std::min(fast, c.speedMaxMs);
    if (!isHold(c.speedMinMs)) fast = std::max(fast, c.speedMinMs);
    // a wing: no section tighter than its full bank turns at that speed; no clamp makes one flyable
    if (!hovers && std::isfinite(f.maxBankRad) && f.maxBankRad > 0.0 && fast > 1.0) {
        const double tightest = 9.80665 * std::tan(f.maxBankRad) / (fast * fast);
        for (std::size_t i = 0; i < n; ++i) {
            double from = 0.0, to = 0.0;
            if (route::tooTight(segments[i], tightest, from, to)) {
                detail.index = static_cast<std::int16_t>(i);
                detail.from = static_cast<float>(from), detail.to = static_cast<float>(to);
                detail.constraint = Constraint::MaxTurnRate;
                return Reason::InvalidCurve;
            }
        }
    }
    // no steeper than it climbs or descends at that speed: flown at its rate (clamped), or refused
    for (std::size_t i = 0; i < n; ++i) {
        double at = 0.0;
        const double gradient = route::steepest(segments[i], at);
        const route::CurvePoint p = route::evaluate(segments[i], at);
        const bool descends = p.gradient() < 0.0;
        const double rate = descends ? f.maxDescentMs : f.maxClimbMs;
        if (std::isnan(rate) || gradient * fast <= rate) continue;
        if (range == RangePolicy::Reject || !(flags & kClamped)) {
            detail.index = static_cast<std::int16_t>(i), detail.from = detail.to = static_cast<float>(at);
            detail.constraint = descends ? Constraint::MaxDescentRate : Constraint::MaxClimbRate;
        }
        if (range == RangePolicy::Reject) return Reason::PerformanceLimit;
        flags |= kClamped;
        break;
    }
    return Reason::None;
}

void CapabilityHost::writeCurve(Span<const BezierSegment> segments, bool appending) {
    if (!config_->path) config_->path = std::make_unique<PathStore>();
    PathStore& store = *config_->path;
    if (!appending) store.segmentCount = 0, ++store.curve; // a new curve: flown afresh
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

int CapabilityHost::liveSlot(ActivityId activity) const noexcept {
    if (!activity) return -1;
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live && slots_[s].activity == activity) return static_cast<int>(s);
    return -1;
}

CapabilityStatus CapabilityHost::status(std::size_t capability, const sim::VehicleState& state) const noexcept {
    if (capability >= catalog_->size()) return {Availability::Disabled, Reason::UnknownCapability};
    if (state.diverged) return {Availability::TemporarilyUnavailable, Reason::Diverged};
    return {};
}

ActivityId CapabilityHost::holder(AxisMask axes, Source source) const noexcept {
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live && (records_[s].axes & axes) && records_[s].source > source) return records_[s].id;
    return 0;
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

ActivityRecord& CapabilityHost::start(std::size_t s, ActivityId id, std::size_t capability, const CommandOptions& options, AxisMask axes,
                                      std::uint16_t flags, double now) noexcept {
    slots_[s] = Slot{id, true, options.range, static_cast<std::uint16_t>(flags & kClamped ? kActivityClamped : 0), kUnknown};
    ActivityRecord& record = records_[s];
    record = ActivityRecord{};
    record.id = id;
    record.vehicle = vehicle_;
    record.capability = static_cast<std::uint16_t>(capability);
    record.source = options.source;
    record.axes = axes;
    record.state = ActivityState::Pending;
    record.startTime = now;
    return record;
}

CommandResult CapabilityHost::submit(const Command& command, const CommandOptions& options, const sim::VehicleState& state, double now) {
    return submitWith(command, {}, {}, options, state, now);
}

CommandResult CapabilityHost::submit(const RouteCommand& route, Span<const Waypoint> waypoints, const CommandOptions& options, const sim::VehicleState& state,
                                     double now) {
    return submitWith(Command(route), waypoints, {}, options, state, now);
}

CommandResult CapabilityHost::submit(const CurveCommand& curve, Span<const BezierSegment> segments, const CommandOptions& options,
                                     const sim::VehicleState& state, double now) {
    return submitWith(Command(curve), {}, segments, options, state, now);
}

CommandResult CapabilityHost::submitWith(const Command& command, Span<const Waypoint> waypoints, Span<const BezierSegment> segments,
                                         const CommandOptions& options, const sim::VehicleState& state, double now) {
    const int found = catalog_->indexOf(command);
    if (found < 0) return rejected(Reason::UnknownCapability);
    const auto index = static_cast<std::size_t>(found);
    const CapabilityDescriptor& d = catalog_->descriptor(index);
    if (!(d.interactions & kCommand)) return rejected(Reason::UnknownCapability);
    // a behaviour takes a BehaviorCommand, a mode its own setpoint (a BehaviorCommand naming "hsa" is not an hsa)
    if (d.kind == CapabilityKind::Guidance && (d.setpoint == SetpointKind::Behavior) != std::holds_alternative<BehaviorCommand>(command))
        return rejected(Reason::WrongCommandType);
    const bool checked = options.range != RangePolicy::None;
    if (checked) {
        if (status(index, state).availability != Availability::Available) return rejected(Reason::Unavailable);
        if (d.version < options.minVersion) return rejected(Reason::VersionUnsupported);
    }

    Command setpoint = command;
    std::uint16_t flags = 0;
    CommandResult detail;
    auto* hsa = std::get_if<HsaCommand>(&setpoint);
    if (hsa) // complete it first: what it leaves out, whatever the range policy (the runtime flies a complete setpoint)
        if (const Reason why = resolveHsa(*hsa, state, detail); why != Reason::None) return about(rejected(why), detail);
    auto* route = std::get_if<RouteCommand>(&setpoint);
    if (route) // its waypoints completed, and checked as its range policy says
        if (const Reason why = checkRoute(*route, waypoints, state, options.range, flags, detail); why != Reason::None) return about(rejected(why), detail);
    auto* curve = std::get_if<CurveCommand>(&setpoint);
    if (curve) // its segments checked as its range policy says
        if (const Reason why = checkCurve(*curve, segments, false, state, options.range, flags, detail); why != Reason::None) return about(rejected(why), detail);
    auto* pattern = std::get_if<PatternCommand>(&setpoint);
    if (pattern) { // complete it first, as an hsa (the runtime flies a complete setpoint)
        if (const Reason why = checkPattern(*pattern, false, detail); why != Reason::None) return about(rejected(why), detail);
        WindEstimate wind;
        wind.update(state, 0.0);
        route::completePattern(*pattern, state, performance_, (adapter_->features() & kFeatureHover) != 0, std::hypot(wind.northMs, wind.eastMs));
    }
    if (checked)
        if (const Reason why = catalog_->check(index, setpoint, options.range, flags, detail); why != Reason::None) return about(rejected(why), detail);
    if (hsa && checked)
        if (const Reason why = limitHsa(*hsa, options.range, flags, detail); why != Reason::None) return about(rejected(why), detail);
    if (pattern && checked)
        if (const Reason why = limitPattern(*pattern, options.range, flags, detail); why != Reason::None) return about(rejected(why), detail);
    AxisMask axes = options.axes ? options.axes : catalog_->defaultAxes(index, command);
    // above the actuators a command owns whole groups: a wing's loop that banks
    // also coordinates, a rotorcraft's cyclic tilts in roll and pitch at once
    if (d.level != Level::Actuator) axes = widenToGroups(axes, d.axisGroups);
    if (const Reason why = checkAxes(d, axes); why != Reason::None) return rejected(why);
    if (const Reason why = checkAwareness(axes, d.level, true); why != Reason::None) return rejected(why);
    if (const ActivityId other = holder(axes, options.source)) return rejected(Reason::AuthorityHeld, 0, other);

    std::unique_ptr<Behavior> behavior;
    if (d.kind == CapabilityKind::Guidance) {
        auto created = ControllerRegistry::instance().create(d.behavior);
        auto* b = dynamic_cast<Behavior*>(created.get());
        if (!b) return rejected(Reason::UnknownCapability);
        created.release();
        behavior.reset(b);
    }

    const ActivityId id = activityId(vehicle_, ++serial_);
    takeOver(axes, id, now);
    // A free slot: every slot in use flies a primary axis the new activity did not take.
    std::size_t s = 0;
    while (s + 1 < kSlotCount && slots_[s].activity) ++s;

    RuntimeConfig& config = *config_;
    if (route) writeRoute(); // (a guidance activity owns every primary axis: one route flies at a time)
    if (curve) writeCurve(segments, false), curve->append = kHold;
    SetpointSlot& slot = config.slots[s];
    slot.command = std::move(setpoint);
    slot.level = d.level;
    slot.axes = axes;
    ++slot.generation;
    ++slot.revision;
    for (std::size_t a = 0; a < kAxisCount; ++a)
        if (axes & (1u << a)) config.owner[a] = static_cast<std::uint8_t>(s);
    ++config.revision;
    runtime_->install(s, std::move(behavior));
    start(s, id, index, options, axes, flags, now);
    return about(accepted(id, flags), detail);
}

CommandResult CapabilityHost::submit(const SupportCommand& command, const CommandOptions& options, const sim::VehicleState& state, double now) {
    const int found = catalog_->indexOf(command);
    if (found < 0) return rejected(Reason::UnknownCapability); // the aircraft has no such effector
    const auto index = static_cast<std::size_t>(found);
    const CapabilityDescriptor& d = catalog_->descriptor(index);
    const bool checked = options.range != RangePolicy::None;
    if (checked) {
        if (status(index, state).availability != Availability::Available) return rejected(Reason::Unavailable);
        if (d.version < options.minVersion) return rejected(Reason::VersionUnsupported);
    }
    SupportCommand setpoint = command;
    std::uint16_t flags = 0;
    CommandResult detail;
    if (checked)
        if (const Reason why = catalog_->check(index, setpoint, options.range, flags, detail); why != Reason::None) return about(rejected(why), detail);
    // the placards: no gear up on the ground, no gear or flaps out above their speeds
    if (const Reason why = adapter_->admit(setpoint, state, *profile_); why != Reason::None) return rejected(why);
    const AxisMask axes = d.axes;
    if (options.axes && options.axes != axes) return rejected(Reason::InvalidAxes);
    // the engines take thrust from the cascade: what flies the rest flies it apart
    if (const Reason why = checkAwareness(axes, Level::Actuator, false); why != Reason::None) return rejected(why);
    if (const ActivityId other = holder(axes, options.source)) return rejected(Reason::AuthorityHeld, 0, other);

    const ActivityId id = activityId(vehicle_, ++serial_);
    takeOver(axes, id, now);
    const std::size_t s = directSlot(setpoint);
    writeDirect(s, setpoint);
    if (s == kEnginesSlot) { // thrust, or every primary axis where the engines fly the aircraft (a multirotor's rotors)
        for (std::size_t a = 0; a < kAxisCount; ++a)
            if (axes & (1u << a)) config_->owner[a] = RuntimeConfig::kEngines;
    } else {
        config_->owner[static_cast<std::size_t>(supportAxis(setpoint))] = RuntimeConfig::kSupport;
    }
    ++config_->revision;
    start(s, id, index, options, axes, flags, now);
    if (d.persistence == Persistence::Terminating) slots_[s].target = supportGoal(setpoint);
    return about(accepted(id, flags), detail);
}

CommandResult CapabilityHost::update(ActivityId activity, const RouteCommand& route, Span<const Waypoint> waypoints,
                                     const sim::VehicleState& state) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
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
    const Span<const Waypoint> points = waypoints.empty() && store ? Span<const Waypoint>(store->waypoints, store->count) : waypoints;
    CommandResult result = accepted(activity);
    if (const Reason why = checkRoute(next, points, state, slots_[s].range, result.flags, result); why != Reason::None)
        return about(rejected(why, activity), result);
    if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    writeRoute();
    *live = next;
    ++config_->slots[s].revision;
    return result;
}

CommandResult CapabilityHost::update(ActivityId activity, const CurveCommand& curve, Span<const BezierSegment> segments,
                                     const sim::VehicleState& state) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
    if (!isCascade(s)) return rejected(Reason::WrongCommandType, activity);
    if (!(catalog_->descriptor(records_[s].capability).interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity); // (as update() answers)
    auto* live = std::get_if<CurveCommand>(&config_->slots[s].command);
    if (!live) return rejected(Reason::WrongCommandType, activity);
    // the options given replace the curve's; `append` is this UPDATE's own
    CurveCommand next = *live;
    const double* given[] = {&curve.latitudeRad, &curve.longitudeRad, &curve.altitudeM, &curve.speedMinMs, &curve.speedMaxMs, &curve.durationS, &curve.end};
    double* kept[] = {&next.latitudeRad, &next.longitudeRad, &next.altitudeM, &next.speedMinMs, &next.speedMaxMs, &next.durationS, &next.end};
    for (std::size_t i = 0; i < std::size(given); ++i)
        if (!isHold(*given[i])) *kept[i] = *given[i];
    next.append = curve.append;
    const bool appending = curve.append == 1.0;
    CommandResult result = accepted(activity);
    if (segments.empty()) { // its options alone: how it is flown, not where
        if (appending) {
            result.index = 0; // nothing to append
            return about(rejected(Reason::InvalidCurve, activity), result);
        }
        if (const Reason why = checkCurveOptions(next, false, result); why != Reason::None) return about(rejected(why, activity), result);
        if (slots_[s].range != RangePolicy::None) {
            if (const Reason why = limitCurveSpeeds(next, slots_[s].range, result.flags, result); why != Reason::None) return about(rejected(why, activity), result);
            if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
        }
    } else {
        if (appending) next.latitudeRad = live->latitudeRad, next.longitudeRad = live->longitudeRad, next.altitudeM = live->altitudeM; // (its reference)
        if (const Reason why = checkCurve(next, segments, appending, state, slots_[s].range, result.flags, result); why != Reason::None)
            return about(rejected(why, activity), result);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
        writeCurve(segments, appending);
    }
    next.append = kHold;
    *live = next;
    ++config_->slots[s].revision;
    return result;
}

CommandResult CapabilityHost::update(ActivityId activity, const Command& setpoint, const sim::VehicleState& state) noexcept {
    if (const auto* route = std::get_if<RouteCommand>(&setpoint)) return update(activity, *route, {}, state);
    if (const auto* curve = std::get_if<CurveCommand>(&setpoint)) return update(activity, *curve, {}, state);
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
    if (!isCascade(s)) return rejected(Reason::WrongCommandType, activity);
    const ActivityRecord& record = records_[s];
    if (!(catalog_->descriptor(record.capability).interactions & kUpdate)) return rejected(Reason::NotUpdatable, activity);
    SetpointSlot& slot = config_->slots[s];
    if (setpoint.index() != slot.command.index()) return rejected(Reason::WrongCommandType, activity);
    CommandResult result = accepted(activity);
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
        if (!isHold(next->headingRad) && !isHold(next->courseRad)) return bad(1);
        if (!isHold(next->speedReference) && isHold(next->speed)) return bad(2);
        if (!isHold(next->altitudeReference) && isHold(next->altitudeM)) return bad(4);
        HsaCommand merged = std::get<HsaCommand>(slot.command);
        mergeHsa(merged, *next);
        if (!isHold(merged.headingRad)) merged.headingRad = geo::wrapPi(merged.headingRad);
        if (!isHold(merged.courseRad)) merged.courseRad = geo::wrapPi(merged.courseRad);
        if (slots_[s].range != RangePolicy::None) {
            Command checked = merged;
            if (const Reason why = catalog_->check(record.capability, checked, slots_[s].range, result.flags, result); why != Reason::None)
                return about(rejected(why, activity), result);
            merged = std::get<HsaCommand>(checked);
            if (const Reason why = limitHsa(merged, slots_[s].range, result.flags, result); why != Reason::None)
                return about(rejected(why, activity), result);
            if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
        }
        std::get<HsaCommand>(slot.command) = merged;
        ++slot.revision;
        return result;
    }
    if (const auto* next = std::get_if<PatternCommand>(&setpoint)) {
        // a partial pattern (docs/vehicle-interface.md, 4.6): the fields given replace the commanded ones
        if (const Reason why = checkPattern(*next, true, result); why != Reason::None) return about(rejected(why, activity), result);
        PatternCommand merged = std::get<PatternCommand>(slot.command);
        mergePattern(merged, *next);
        merged.courseRad = geo::wrapPi(merged.courseRad), merged.longitudeRad = geo::wrapPi(merged.longitudeRad);
        if (slots_[s].range != RangePolicy::None) {
            if (const Reason why = limitPattern(merged, slots_[s].range, result.flags, result); why != Reason::None)
                return about(rejected(why, activity), result);
            if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
        }
        std::get<PatternCommand>(slot.command) = merged;
        ++slot.revision;
        return result;
    }
    if (slots_[s].range == RangePolicy::None) {
        assignSetpoint(slot.command, setpoint);
    } else {
        Command checked = setpoint; // no heap data: a behaviour takes no UPDATE
        if (const Reason why = catalog_->check(record.capability, checked, slots_[s].range, result.flags, result); why != Reason::None)
            return about(rejected(why, activity), result);
        assignSetpoint(slot.command, checked);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    }
    ++slot.revision;
    return result;
}

CommandResult CapabilityHost::update(ActivityId activity, const SupportCommand& setpoint) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
    const ActivityRecord& record = records_[s];
    if (isCascade(s) || catalog_->indexOf(setpoint) != static_cast<int>(record.capability)) return rejected(Reason::WrongCommandType, activity);
    SupportCommand checked = setpoint;
    CommandResult result = accepted(activity);
    if (slots_[s].range != RangePolicy::None) {
        if (const Reason why = catalog_->check(record.capability, checked, slots_[s].range, result.flags, result); why != Reason::None)
            return about(rejected(why, activity), result);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    }
    writeDirect(s, checked);
    if (!std::isnan(slots_[s].target)) slots_[s].target = supportGoal(checked);
    return result;
}

CommandResult CapabilityHost::cancel(ActivityId activity, double now) noexcept {
    const int found = liveSlot(activity);
    if (found < 0) return rejected(this->activity(activity) ? Reason::ActivityEnded : Reason::UnknownActivity, activity);
    const auto s = static_cast<std::size_t>(found);
    end(s, ActivityState::Canceled, Reason::Requested, 0, now);
    release(s); // its axes: the vehicle default
    CommandResult r;
    r.status = CommandStatus::Canceled;
    r.activity = activity;
    return r;
}

CommandResult CapabilityHost::command(const Command& command, const sim::VehicleState& state, double now) {
    // The fast path: the same capability as the live legacy activity (a
    // behaviour command re-creates its behaviour, as it always has).
    if (updateLegacy(command)) return accepted(legacy_);
    CommandOptions legacy;
    legacy.source = Source::Policy;
    legacy.axes = kLegacyAxes;
    legacy.range = RangePolicy::None;
    const CommandResult r = submit(command, legacy, state, now);
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
    for (std::size_t i = 0; i < recentCount_; ++i) out.push_back(recent_[(recentNext_ + kRecent - 1 - i) % kRecent]);
    return out;
}

void CapabilityHost::end(std::size_t s, ActivityState state, Reason reason, ActivityId by, double now) noexcept {
    ActivityRecord& record = records_[s];
    record.state = state;
    record.reason = reason;
    record.by = by;
    record.endTime = now;
    slots_[s].live = false; // the runtime flies its residual hold until another activity takes the axes
    if (slots_[s].activity == legacy_) legacySlot_ = -1;
    recent_[recentNext_] = record;
    recentNext_ = (recentNext_ + 1) % kRecent;
    recentCount_ = std::min(recentCount_ + 1, kRecent);
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

void CapabilityHost::afterStep(const sim::VehicleState& state, const EffectorPositions& positions, double now) noexcept {
    RuntimeReport& report = runtime_->report();
    const RuntimeConfig& config = *config_;
    for (std::size_t s = 0; s < kActivities; ++s) {
        Slot& slot = slots_[s];
        if (!slot.activity) continue; // nothing flies here (and nothing reported)
        if (slot.live) {
            ActivityRecord& record = records_[s];
            const bool flown = isCascade(s) ? report.updates > 0 && report.slots[s].generation == config.slots[s].generation
                                            : report.updates > 0;
            if (record.state == ActivityState::Pending && flown) record.state = ActivityState::Active;
            const std::uint16_t flags = static_cast<std::uint16_t>(slot.flags | flagsOn(report, record.axes));
            record.constraints = flags;
            record.constraintsSeen = static_cast<std::uint16_t>(record.constraintsSeen | flags);
            slot.flags = 0;
            // how far a behaviour has got, before it may complete (docs/vehicle-interface.md, 5.3)
            if (isCascade(s) && config.slots[s].level == Level::Behavior) runtime_->progress(s, record.progress);
            if (state.diverged) {
                end(s, ActivityState::Failed, Reason::Diverged, 0, now);
            } else if (record.state == ActivityState::Active && isCascade(s)) {
                const SlotReport& events = report.slots[s];
                if (events.events & kFailed) end(s, ActivityState::Failed, events.failure, 0, now);
                else if (events.events & kFinished) end(s, ActivityState::Completed, Reason::GoalReached, 0, now);
            } else if (record.state == ActivityState::Active && isSupport(s) && !std::isnan(slot.target)) {
                // gear or flaps: done when they are there (and held there, as a residual)
                const std::size_t axis = static_cast<std::size_t>(Axis::Flaps) + (s - kSlotCount);
                const double position = axis == static_cast<std::size_t>(Axis::Gear) ? positions.gear : positions.flaps;
                if (!std::isnan(position) && std::abs(position - slot.target) < 0.01)
                    end(s, ActivityState::Completed, Reason::GoalReached, 0, now);
            }
        }
        if (isCascade(s)) {
            report.slots[s].events = 0;
            report.slots[s].failure = Reason::None;
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
}

void CapabilityHost::onReset() noexcept {
    for (std::size_t s = 0; s < kActivities; ++s)
        if (slots_[s].live) {
            records_[s].state = ActivityState::Pending; // the runtime starts it again
            slots_[s].flags = 0;
        }
}

} // namespace fsim::control
