// A must fly (docs/flight-autonomy.md, 4.42; A-GRA's MUST_FLY): its location laid out as a route from where the aircraft is
// - the points it approaches through where it has a window of bearings to come from, then the location, flown over - as it
// is commanded or updated, and flown by the route's follower. In a file of its own: the host's checks and the behaviour.
#include "control/CapabilityHost.h"

#include "control/Checks.h"
#include "control/Route.h"
#include "control/Runtime.h"
#include "core/Geodesy.h"
#include "fsim/ControllerRegistry.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
/// How far above a vehicle a must fly flies over it, given no altitude: 500 ft (4.42).
constexpr double kOverEntityM = 152.4;
/// A turn onto an approach's last leg sharper than this is split by a point abeam (4.42).
constexpr double kMostTurnRad = 120.0 * kDeg;

bool code(double v, double count) noexcept { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); }
bool finiteOr(double v) noexcept { return isHold(v) || std::isfinite(v); }
/// An id as a field carries it: whole, from 1, within the 2^53 a double holds exactly.
bool anId(double v) noexcept { return v == std::floor(v) && v >= 1.0 && v <= 9007199254740992.0; }

/// `r` with what the check said the answer is about, as CapabilityHost.cpp's.
CommandResult about(CommandResult r, const CommandResult& detail) noexcept {
    r.index = detail.index;
    r.constraint = detail.constraint;
    r.from = detail.from;
    r.to = detail.to;
    return r;
}

/// The approach to a location from within its window of bearings (A-GRA's IngressConstraint: the bearings from the
/// location to the aircraft, `minRad` clockwise to `maxRad`), the aircraft `northA`, `eastA` from it: none where it comes
/// from within the window now; else the points it flies through, north and east of the location - the last `lengthM` out
/// along a bearing within the window (its nearer edge, moved in by 5 degrees or half the window), so the leg from there to
/// the location is flown on that bearing's reciprocal, and a turn onto that leg of more than 120 degrees split by a point
/// `2.5 radiusM` to the aircraft's side of it. Their count into `count`.
void approach(double northA, double eastA, double minRad, double maxRad, double lengthM, double radiusM, double out[2][2], int& count) noexcept {
    count = 0;
    const double width = geo::wrapTwoPi(maxRad - minRad); // (0: one bearing)
    const double now = geo::wrapTwoPi(std::atan2(eastA, northA));
    if (geo::wrapTwoPi(now - minRad) <= width) return; // (within it: straight in)
    const double margin = std::min(5.0 * kDeg, 0.5 * width);
    const double bearing = geo::wrapTwoPi(minRad - now) <= geo::wrapTwoPi(now - maxRad) ? minRad + margin : maxRad - margin;
    const double un = std::cos(bearing), ue = std::sin(bearing);
    const double n2 = lengthM * un, e2 = lengthM * ue;
    // the turn onto the last leg there, from the leg the aircraft flies to it
    const double in = std::atan2(e2 - eastA, n2 - northA);
    if (std::abs(geo::wrapPi(bearing + kPi - in)) > kMostTurnRad) {
        const double side = un * eastA - ue * northA >= 0.0 ? 1.0 : -1.0; // (the aircraft right of the bearing out, or left)
        out[count][0] = n2 - side * 2.5 * radiusM * ue, out[count][1] = e2 + side * 2.5 * radiusM * un, ++count;
    }
    out[count][0] = n2, out[count][1] = e2, ++count;
}

} // namespace

// --- The host: laid out and checked ----------------------------------------------------------------

void mergeMustFly(MustFlyCommand& dst, const MustFlyCommand& src) noexcept {
    if (!isHold(src.location) && src.location != dst.location) dst.latitudeRad = dst.longitudeRad = dst.target = kHold;
    const double* from[] = {&src.location, &src.latitudeRad,   &src.longitudeRad,  &src.altitudeM, &src.altitudeReference,
                            &src.target,   &src.ingressMinRad, &src.ingressMaxRad, &src.speed,     &src.speedReference};
    double* to[] = {&dst.location, &dst.latitudeRad,   &dst.longitudeRad,  &dst.altitudeM, &dst.altitudeReference,
                    &dst.target,   &dst.ingressMinRad, &dst.ingressMaxRad, &dst.speed,     &dst.speedReference};
    for (std::size_t i = 0; i < std::size(from); ++i)
        if (!isHold(*from[i])) *to[i] = *from[i];
}

Reason CapabilityHost::prepareMustFly(MustFlyCommand& c, const sim::VehicleState& state, CheckLog& log) {
    CommandResult& detail = log.result;
    auto bad = [&detail](std::int16_t field) {
        detail.index = field;
        return Reason::InvalidParameter;
    };
    // its fields whole, finite and in range; a location's own given, and nothing another's is
    if (!code(c.location, static_cast<double>(MustFlyLocation::Count))) return bad(0);
    double* fields[] = {&c.latitudeRad, &c.longitudeRad, &c.altitudeM, &c.altitudeReference, &c.target, &c.ingressMinRad, &c.ingressMaxRad, &c.speed,
                        &c.speedReference};
    for (std::size_t i = 0; i < std::size(fields); ++i)
        if (!finiteOr(*fields[i])) return bad(static_cast<std::int16_t>(i + 1));
    if (!code(c.altitudeReference, static_cast<double>(AltitudeReference::Count))) return bad(4);
    if (!code(c.speedReference, static_cast<double>(SpeedReference::Count))) return bad(9);
    if (!isHold(c.speed) && c.speed < 0.0) return bad(8);
    if (isHold(c.ingressMinRad) != isHold(c.ingressMaxRad)) return bad(isHold(c.ingressMinRad) ? 6 : 7);
    if (!isHold(c.ingressMinRad) && std::abs(c.ingressMinRad) > kPi) return bad(6); // (A-GRA's AngleType)
    if (!isHold(c.ingressMaxRad) && std::abs(c.ingressMaxRad) > kPi) return bad(7);
    c.location = orHold(c.location, 0.0);
    const auto kind = static_cast<MustFlyLocation>(static_cast<int>(c.location));
    if (kind != MustFlyLocation::Point && (!isHold(c.latitudeRad) || !isHold(c.longitudeRad))) return bad(isHold(c.latitudeRad) ? 2 : 1);
    if (kind == MustFlyLocation::Point && !isHold(c.target)) return bad(5);
    if (kind != MustFlyLocation::Point && (isHold(c.target) || !anId(c.target))) return bad(5);

    // the location, a point flown over (EndPointKind::Waypoint), at its altitude and speed
    Waypoint at;
    at.kind = static_cast<double>(EndPointKind::Waypoint);
    at.altitudeM = c.altitudeM, at.altitudeReference = c.altitudeReference, at.speed = c.speed, at.speedReference = c.speedReference;
    double ingressMin = c.ingressMinRad, ingressMax = c.ingressMaxRad;
    FramePose pose;       // (a location in a frame: the frame now)
    bool framed = false;
    switch (kind) {
    case MustFlyLocation::Point:
        if (isHold(c.latitudeRad) || std::abs(c.latitudeRad) > 0.5 * kPi) return bad(1);
        if (isHold(c.longitudeRad)) return bad(2);
        at.latitudeRad = c.latitudeRad, at.longitudeRad = geo::wrapPi(c.longitudeRad);
        break;
    case MustFlyLocation::Entity: {
        // another vehicle, flown over in its own frame (kVehicleFrames): at the altitude given, else as far above it as the
        // aircraft is now - no less than 500 ft - followed as it climbs or descends
        const double id = c.target;
        FrameSpec spec;
        if (id == static_cast<double>(vehicle_) || !sessionView_ || !sessionView_->frame(kVehicleFrames + static_cast<FrameId>(id), spec, pose))
            return bad(5);
        if (!isHold(c.altitudeReference) && isHold(c.altitudeM)) return bad(4); // (a reference to read nothing in)
        at.frame = static_cast<double>(kVehicleFrames + static_cast<FrameId>(id));
        if (isHold(c.altitudeM)) at.frameZM = -std::max(state.altitudeMslM - pose.altitudeMslM, kOverEntityM);
        framed = true;
        break;
    }
    case MustFlyLocation::OpPoint: {
        OpPoint p;
        if (!sessionView_ || !sessionView_->opPoint(static_cast<OpPointId>(c.target), p)) {
            detail.index = 5;
            return Reason::UnknownGeometry;
        }
        at.latitudeRad = p.latitudeRad, at.longitudeRad = isHold(p.longitudeRad) ? kHold : geo::wrapPi(p.longitudeRad);
        at.frame = p.frame, at.frameRotation = p.frameRotation, at.frameOffsets = p.frameOffsets;
        at.frameXM = p.frameXM, at.frameYM = p.frameYM, at.frameZM = p.frameZM;
        if (isHold(c.altitudeM) && isHold(c.altitudeReference)) at.altitudeM = p.altitudeM, at.altitudeReference = p.altitudeReference;
        if (isHold(ingressMin)) ingressMin = p.ingressMinRad, ingressMax = p.ingressMaxRad;
        if (!isHold(p.frame)) {
            FrameSpec spec;
            if (!sessionView_->frame(static_cast<FrameId>(p.frame), spec, pose)) {
                detail.index = 5;
                return Reason::UnknownGeometry; // (its frame gone)
            }
            framed = true;
        }
        break;
    }
    default: return bad(0);
    }

    // its window of bearings to come from: the points it approaches through, laid out from where the location and the
    // aircraft are now - in the location's frame, where it is in one, north and east of the frame's origin
    Waypoint points[3];
    std::uint32_t n = 0;
    if (!isHold(ingressMin)) {
        double latL = at.latitudeRad, lonL = at.longitudeRad, northL = 0.0, eastL = 0.0; // (in a frame: the location from its origin)
        if (framed) {
            const GeoPoint g = framePoint(pose, at.frameOffset());
            latL = g.latitudeRad, lonL = g.longitudeRad;
            geo::localNorthEastM(pose.latitudeRad, pose.longitudeRad, latL, lonL, northL, eastL);
        }
        double northA = 0.0, eastA = 0.0;
        geo::localNorthEastM(latL, lonL, state.latitudeRad, state.longitudeRad, northA, eastA);
        // the turn onto its last leg at its speed plus the wind (a rotorcraft's cruise, given none), and ten seconds of flight,
        // at least: how far out that leg begins
        const bool hovers = (adapter_->features() & kFeatureHover) != 0;
        const double altitudeMsl = isHold(at.altitudeM) || !isHold(at.frameZM) ? state.altitudeMslM : at.altitudeM;
        const double own = hovers ? orHold(performance_.cruiseTasMs, 5.0) : state.airspeedTrueMs;
        const double v = isHold(c.speed) ? own : route::plannedSpeed(c.speed, orHold(c.speedReference, 0.0), altitudeMsl);
        const WindEstimate wind = checkWind(state);
        const double radius = performance_.turnRadiusM(v + std::hypot(wind.northMs, wind.eastMs));
        double out[2][2];
        int k = 0;
        approach(northA, eastA, ingressMin, ingressMax, std::max(3.0 * radius, 10.0 * v), radius, out, k);
        for (int j = 0; j < k; ++j) {
            Waypoint& w = points[n++];
            w.altitudeM = at.altitudeM, w.altitudeReference = at.altitudeReference, w.speed = at.speed, w.speedReference = at.speedReference;
            if (framed) { // (unturned, where the frame's origin was: the frame's moves carry them)
                w.frame = at.frame, w.frameRotation = static_cast<double>(FrameRotation::Unrotated);
                w.frameOffsets = static_cast<double>(FrameOffsets::Cartesian);
                w.frameXM = northL + out[j][0], w.frameYM = eastL + out[j][1], w.frameZM = at.frameZM;
            } else {
                geo::offsetLatLon(latL, lonL, out[j][0], out[j][1], w.latitudeRad, w.longitudeRad);
                w.longitudeRad = geo::wrapPi(w.longitudeRad);
            }
        }
    }
    points[n++] = at;
    RouteCommand laid; // (great circles, once, on along its course at its end)
    return checkRoute(laid, Span<const Waypoint>(points, n), state, log);
}

CommandResult CapabilityHost::updateMustFly(std::size_t s, ActivityId activity, const MustFlyCommand& next, const sim::VehicleState& state,
                                            CommandResult& result, CheckLog& log) noexcept {
    SetpointSlot& slot = config_->slots[s];
    MustFlyCommand merged = std::get<MustFlyCommand>(slot.command);
    mergeMustFly(merged, next);
    // a window given both ways, or neither
    if (isHold(next.ingressMinRad) != isHold(next.ingressMaxRad)) {
        result.index = isHold(next.ingressMinRad) ? 6 : 7;
        return about(rejected(Reason::InvalidParameter, activity), result);
    }
    if (const Reason why = prepareMustFly(merged, state, log); why != Reason::None) return about(rejected(why, activity), result);
    if (slots_[s].range != RangePolicy::None) {
        Command checked = merged;
        if (const Reason why = catalog_->check(records_[s].capability, checked, log); why != Reason::None) return about(rejected(why, activity), result);
        merged = std::get<MustFlyCommand>(checked);
        checkTerrain(checked, state, log);
        if (log.refused != Reason::None) return about(rejected(log.refused, activity), result);
        if (result.flags & kClamped) slots_[s].flags |= kActivityClamped;
    }
    writeRoute(), routePlan_->passed.clear(); // (the path store's route afresh: flown from where the aircraft is)
    std::get<MustFlyCommand>(slot.command) = merged;
    ++slot.revision;
    return result;
}

// --- The behaviour ------------------------------------------------------------------------------

/// The world as the route sees it while it flies to its first point over another vehicle: that vehicle where it will be
/// as the aircraft gets there, carried on at its velocity - so the leg to it is a collision course, which closes on the
/// vehicle itself (4.42). Every other answer the world's.
struct MustFlyBehavior::Lead final : WorldView {
    const WorldView* world = nullptr;
    std::uint32_t entity = 0;
    bool leading = false;
    sim::VehicleState ahead{};
    const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept override {
        if (leading && id == entity) return &ahead;
        return world->vehicleState(id);
    }
    double simTime() const noexcept override { return world->simTime(); }
    const sim::EnvironmentState& environment() const noexcept override { return world->environment(); }
    bool navigation(std::uint32_t id, NavigationReport& out) const noexcept override { return world->navigation(id, out); }
};

MustFlyBehavior::MustFlyBehavior() : route_(std::make_unique<RouteBehavior>()), lead_(std::make_unique<Lead>()), options_(RouteCommand{}) {}
MustFlyBehavior::~MustFlyBehavior() = default;

void MustFlyBehavior::begin(const ControlContext& ctx, const Command&) { route_->begin(ctx, options_); }

Command MustFlyBehavior::update(const ControlContext& ctx, const Command& in) {
    const auto* m = std::get_if<MustFlyCommand>(&in);
    ActivityProgress progress;
    const bool first = !route_->progress(progress) || progress.segment == 0;
    const sim::VehicleState* e = m && m->location == static_cast<double>(MustFlyLocation::Entity) && ctx.world && ctx.path && ctx.path->count
                                     ? ctx.world->vehicleState(static_cast<std::uint32_t>(m->target))
                                     : nullptr;
    if (!e || !first) return route_->update(ctx, options_);
    // the first point, where that vehicle's frame puts it (north and east of it, unturned: 4.42), met at the aircraft's
    // speed over the ground: when, found again from where the point will be by then (three times: it converges)
    const sim::VehicleState& s = ctx.sensed;
    const Waypoint& p = ctx.path->waypoints[0];
    const double pn = orHold(p.frameXM, 0.0), pe = orHold(p.frameYM, 0.0);
    double n0 = 0.0, e0 = 0.0;
    geo::localNorthEastM(s.latitudeRad, s.longitudeRad, e->latitudeRad, e->longitudeRad, n0, e0);
    // (a rotorcraft from a hover: at the speed it will fly - its cruise, given none or in another reference - not the speed it
    // has yet)
    const bool metres = !isHold(m->speed) && (isHold(m->speedReference) || m->speedReference == static_cast<double>(SpeedReference::TrueAirspeed) ||
                                              m->speedReference == static_cast<double>(SpeedReference::GroundSpeed));
    const double least =
        (ctx.features & kFeatureHover) && ctx.performance ? (metres ? m->speed : orHold(ctx.performance->cruiseTasMs, 1.0)) : 1.0;
    const double speed = std::max(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]), std::max(least, 1.0));
    double t = 0.0;
    for (int k = 0; k < 3; ++k)
        t = std::min(std::hypot(n0 + pn + e->velocityNedMs[0] * t, e0 + pe + e->velocityNedMs[1] * t) / speed, 120.0);
    Lead& lead = *lead_;
    lead.world = ctx.world, lead.entity = static_cast<std::uint32_t>(m->target), lead.leading = true;
    lead.ahead = *e;
    geo::offsetLatLon(e->latitudeRad, e->longitudeRad, e->velocityNedMs[0] * t, e->velocityNedMs[1] * t, lead.ahead.latitudeRad, lead.ahead.longitudeRad);
    lead.ahead.altitudeMslM = e->altitudeMslM - e->velocityNedMs[2] * t;
    ControlContext led = ctx;
    led.world = &lead;
    return route_->update(led, options_);
}

void MustFlyBehavior::reset() { route_->reset(); }

bool MustFlyBehavior::finished() const noexcept { return route_->finished(); }

Reason MustFlyBehavior::failure() const noexcept { return route_->failure(); }

std::uint16_t MustFlyBehavior::constraints() const noexcept { return route_->constraints(); }

bool MustFlyBehavior::progress(ActivityProgress& out) const noexcept { return route_->progress(out); }

std::uint32_t MustFlyBehavior::ahead(std::uint32_t* points, std::uint32_t max, bool& ends) const noexcept { return route_->ahead(points, max, ends); }

bool MustFlyBehavior::arrival(ArrivalEstimate& out) const noexcept { return static_cast<const Behavior&>(*route_).arrival(out); }

void registerMustFly(ControllerRegistry& r) {
    constexpr double now = kHold, inf = std::numeric_limits<double>::infinity();
    auto p = [](const char* name, const char* unit, double def, double lo, double hi, Constraint below = Constraint::None,
                Constraint above = Constraint::None) { return ParameterInfo{name, unit, lo, hi, def, true, below, above}; };
    // its setpoint's fields, in order (the C ABI's too); left out: as the host lays it out (4.42)
    BehaviorTraits mustFly;
    mustFly.persistence = Persistence::Terminating;
    mustFly.parameters = {p("location", "", now, 0.0, static_cast<double>(MustFlyLocation::Count) - 1.0),
                          p("latitude_rad", "rad", now, -0.5 * kPi, 0.5 * kPi),
                          p("longitude_rad", "rad", now, -inf, inf),
                          p("altitude_m", "m", now, -inf, inf),
                          p("altitude_reference", "", now, 0.0, static_cast<double>(AltitudeReference::Count) - 1.0),
                          p("target", "", now, 1.0, 9007199254740992.0),
                          p("ingress_min_rad", "rad", now, -kPi, kPi),
                          p("ingress_max_rad", "rad", now, -kPi, kPi),
                          p("speed", "m/s or Mach", now, 0.0, inf, Constraint::MinAirspeed, Constraint::MaxAirspeed),
                          p("speed_reference", "", now, 0.0, static_cast<double>(SpeedReference::Count) - 1.0)};
    mustFly.uses = {"fsim.flight.velocity", "fsim.flight.position"};
    mustFly.mode = FlightMode::MustFly;
    mustFly.setpoint = SetpointKind::MustFly;
    r.addBehavior("must_fly", [] { return std::make_unique<MustFlyBehavior>(); }, std::move(mustFly));
}

} // namespace fsim::control
