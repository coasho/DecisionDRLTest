// A-GRA's RECOVERY to an airfield's runway (docs/flight-autonomy.md, 4.53; ADR-29 FA-10a: RCV-01, RCV-04): the host resolves
// the runway's landing line at the NEW and lays out the approach as a route - a wing's onto the extended centre line and down
// its glide slope, a rotorcraft's to a hover over the runway - and the "recovery" behaviour flies it, then a wing's flare,
// touchdown and rollout, a rotorcraft's vertical descent to the ground.
#include "control/CapabilityHost.h"
#include "control/Registry.h"
#include "core/Geodesy.h"
#include "fsim/ControllerRegistry.h"
#include "fsim/BuiltinControllers.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kGlideRad = 3.0 * kDeg;      // the glide slope (ICAO's standard)
constexpr double kScreenM = 15.0;             // crossing the threshold 50 ft up: the glide slope's aim that far beyond it
constexpr double kFinalM = 5000.0;            // the final approach fix this far out at least (three turn radii at least)
constexpr double kIntermediateM = 3000.0;     // the intermediate fix this far before it at least (two turn radii at least, and a
constexpr double kLevelS = 60.0;              // minute level at its speed there: to settle at its approach speed and attitude)
constexpr double kLandingFlaps = 1.0, kLeastFlaps = 0.3; // the flaps for landing: a takeoff's (4.49), out to all of them
constexpr double kDescentGradient = 0.035;    // its descent to the intermediate fix no steeper (2 deg: what a heavy holds its speed on)
constexpr double kFixesOutM = 60000.0;        // the intermediate fix no further out than this
constexpr double kManoeuvreShare = 1.6 / 1.3; // of its approach speed: to the intermediate fix, where it turns (1.6 times its stall)
constexpr double kCaptureRad = 3.0 * kDeg;    // a heading this far off flown as a turn at kTurnBankRad
constexpr double kTurnBankRad = 35.0 * kDeg;  // its turns' bank no more than this
constexpr double kApproachBankRad = 25.0 * kDeg; // its approach's turns planned at this bank (its points' bank)
constexpr double kFlapsPerS = 0.05;           // its flaps put out so fast, level before the glide slope, for the attitude it will
                                              // come down it at - and held down it: changed on it, they hunted (the E-7A porpoised)
constexpr double kSettledShare = 0.05;        // ...once within this share of its approach speed
constexpr double kFastShare = 1.08;           // down the slope faster than this times its approach speed: its flaps out for their drag
constexpr double kFlareS = 4.0;               // the flare: a sink of the height over this, no less than kFlareSinkMs
constexpr double kFlareSinkMs = 0.6;
constexpr double kFlareKp = 1.5 * kDeg, kFlareKi = 0.6 * kDeg; // its pitch on the sink's error: per m/s, and per m/s a second
constexpr double kFlareMostRad = 8.0 * kDeg;  // its pitch raised no more than this, nor to 2 deg short of its tail's touching
constexpr double kTailMarginRad = 2.0 * kDeg;
constexpr double kFlareBankRad = 10.0 * kDeg; // its bank in the flare: its heading held, its wings near level
// the attitude it comes down the glide slope at: no higher than its tail's touching less kFlareRoomRad, room to flare - its
// flaps put out level before it, then its approach speed raised down it, at kAdjustMsPerDegS within kMostRaise of it, while it
// is (the F-16C came down 8.2 deg nose up, its tail touching at 10.3, and touched down at 5 m/s). Its flaps start at a takeoff's:
// all of them, at 1.3 times its clean stall, brought the B-52H down 11.5 deg nose down onto its front gear
constexpr double kFlareRoomRad = 6.0 * kDeg;
constexpr double kAdjustMsPerDegS = 0.5, kMostRaise = 0.3;
constexpr double kFlareSpeedShare = 0.9;      // of its approach speed: the throttle back in the flare
// a rotorcraft's approach point on the centre line, 60 s of its cruise out and 10 s up - 30 m to 1 km, 100 m at most - then its
// hover 2 s up (1.5 to 10 m) this far beyond the threshold: 1 km out and 100 m up, the Crazyflie fell
constexpr double kRotorApproachS = 60.0, kRotorApproachUpS = 10.0, kRotorHoverS = 2.0;
constexpr double kRotorApproachLeastM = 30.0, kRotorApproachMostM = 1000.0, kRotorApproachMostAglM = 100.0;
constexpr double kHoverLeastAglM = 1.5, kHoverMostAglM = 10.0;
constexpr double kHoverAlongM = 60.0;
constexpr double kDescentMs = 0.7;            // its vertical descent
constexpr double kSettledS = 1.0;             // on the ground so long: landed

/// Along and across (+ right) the line from (lat0, lon0) on `course`.
void onLine(double lat0, double lon0, double course, double lat, double lon, double& along, double& cross) noexcept {
    double north, east;
    geo::localNorthEastM(lat0, lon0, lat, lon, north, east);
    along = north * std::cos(course) + east * std::sin(course);
    cross = -north * std::sin(course) + east * std::cos(course);
}

Waypoint along(double lat0, double lon0, double course, double alongM, double altitudeMslM, double speed, double reference) {
    Waypoint w;
    geo::offsetLatLon(lat0, lon0, alongM * std::cos(course), alongM * std::sin(course), w.latitudeRad, w.longitudeRad);
    w.altitudeM = altitudeMslM, w.altitudeReference = static_cast<double>(AltitudeReference::Msl);
    w.speed = speed, w.speedReference = reference;
    return w;
}

} // namespace

Reason CapabilityHost::landingLine(double airfieldId, double runwayId, RouteGround& out, CommandResult& detail) const {
    auto at = [&detail](std::int16_t field, Reason why) {
        detail.index = field;
        return why;
    };
    // its airfield and runway, the vehicle's (4.40); the runway's landing line: its threshold (else its start), on to its limit
    // (else on its direction), as long as that (else its available length), at the threshold's elevation (else the ground's)
    Airfield a;
    if (!(airfieldId >= 1.0) || airfieldId != std::floor(airfieldId) || airfieldId > 4294967295.0 || !airfield(static_cast<AirfieldId>(airfieldId), a))
        return at(0, Reason::UnknownAirfield);
    const Runway* r = nullptr;
    for (const Runway& x : a.runways)
        if (static_cast<double>(x.id) == runwayId) r = &x;
    if (!r) return at(1, Reason::UnknownAirfield);
    const RunwayCoordinates& l = r->landing;
    const RunwayPoint& t = isHold(l.threshold.latitudeRad) ? l.start : l.threshold;
    if (isHold(t.latitudeRad)) return at(1, Reason::InvalidParameter); // (a runway for takeoffs only)
    double course = r->directionRad, length = r->availableLengthM;
    if (!isHold(l.limit.latitudeRad)) {
        course = geo::bearingRad(t.latitudeRad, t.longitudeRad, l.limit.latitudeRad, l.limit.longitudeRad);
        length = geo::distanceM(t.latitudeRad, t.longitudeRad, l.limit.latitudeRad, l.limit.longitudeRad);
    }
    if (isHold(course) || isHold(length) || !(length > 0.0)) return at(1, Reason::InvalidParameter);
    double elevation = t.altitudeM;
    const bool above = !isHold(t.altitudeReference) && t.altitudeReference == static_cast<double>(AltitudeReference::AboveGround);
    if (isHold(elevation) || above) {
        const double ground = sessionView_ ? sessionView_->groundM(t.latitudeRad, t.longitudeRad) : kHold;
        if (!std::isfinite(ground)) return at(1, Reason::InvalidParameter); // (no elevation, and no ground known there)
        elevation = ground + (above ? orHold(t.altitudeM, 0.0) : 0.0);
    }
    out.startLatitudeRad = t.latitudeRad, out.startLongitudeRad = t.longitudeRad;
    out.courseRad = geo::wrapTwoPi(course), out.lengthM = length, out.airfield = airfieldId, out.runway = runwayId;
    out.elevationM = elevation;
    return Reason::None;
}

Reason CapabilityHost::prepareRecovery(BehaviorCommand& b, const sim::VehicleState& state, CheckLog& log) {
    CommandResult& detail = log.result;
    // a policy's parameters are its own: what the host writes ("_...") it cannot give (4.50)
    if (policyNew_) b.params.erase(b.params.lower_bound("_"), b.params.lower_bound("`"));
    if (state.onGround) return detail.index = -1, Reason::OnGround; // (FA's own, on the ground: a landing from the air)
    RouteGround line;
    if (const Reason why = landingLine(b.param("airfield", 0.0), b.param("runway", 0.0), line, detail); why != Reason::None) return why;
    const double elevation = line.elevationM, course = line.courseRad;
    const double lat0 = line.startLatitudeRad, lon0 = line.startLongitudeRad;
    Waypoint points[4];
    RouteCommand laid;
    if (performance_.hovers) {
        // a rotorcraft: onto the centre line 1 km out, 100 m up, then to a hover 10 m over the runway, stopped there
        const double speed = orHold(performance_.cruiseTasMs, 10.0);
        const double out = std::clamp(kRotorApproachS * speed, kRotorApproachLeastM, kRotorApproachMostM);
        const double up = std::min(kRotorApproachUpS * speed, kRotorApproachMostAglM);
        const double hover = std::clamp(kRotorHoverS * speed, kHoverLeastAglM, kHoverMostAglM);
        points[0] = along(lat0, lon0, course, -out, elevation + std::max(up, hover), speed, static_cast<double>(SpeedReference::GroundSpeed));
        points[1] = along(lat0, lon0, course, kHoverAlongM, elevation + hover, kHold, kHold);
        laid.end = static_cast<double>(EndBehavior::Loiter);
        if (const Reason why = checkRoute(laid, Span<const Waypoint>(points, 2), state, log); why != Reason::None) return why;
        b.params["_vapp"] = 0.0, b.params["_aim"] = kHoverAlongM;
    } else {
        // a wing: at its approach speed, 1.3 times its stall (its least: a fly-by-wire fighter's), onto the extended centre line
        // at the intermediate fix, level to the final approach fix, then down the glide slope to its aim on the runway - the
        // threshold crossed 15 m up
        if (!takeoffSpeeds(line)) return detail.index = 0, Reason::NotImplemented; // (no speed to land at: a stock model's)
        const double vapp = line.climbCasMs;
        const WindEstimate wind = checkWind(state);
        const double vfix = kManoeuvreShare * vapp, ground = vfix + std::hypot(wind.northMs, wind.eastMs);
        const double radius = ground * ground / (9.80665 * std::tan(kApproachBankRad));
        const double finalM = std::max(kFinalM, 3.0 * radius);
        const double aim = std::min(kScreenM / std::tan(kGlideRad), line.lengthM / 3.0), height = (finalM + aim) * std::tan(kGlideRad);
        // onto the extended centre line at the intermediate fix by a base leg from its side - a turn there of a quarter: led
        // straight to a fix behind it, the F-16C turned about there, slowing to its approach speed, and spun in - then level to
        // the final approach fix, slowing and configured, then down the glide slope. The fixes out along the centre line until
        // its descent to the base leg is no steeper than kDescentGradient: from 1,000 m 16 km out the C-130J came down 8 % and
        // reached the glide slope at 107 m/s against its 71
        double north0, east0;
        geo::localNorthEastM(lat0, lon0, state.latitudeRad, state.longitudeRad, north0, east0);
        const double side = -north0 * std::sin(course) + east0 * std::cos(course) < 0.0 ? -1.0 : 1.0; // (its side: + right of the line)
        const double baseM = 2.5 * radius;
        double fixM = finalM + std::max({kIntermediateM, 2.0 * radius, kLevelS * vfix});
        const double above = state.altitudeMslM - (elevation + height);
        auto base = [&](double out, double& n, double& e) { // (the base leg's start from the threshold)
            n = -out * std::cos(course) - side * baseM * std::sin(course), e = -out * std::sin(course) + side * baseM * std::cos(course);
        };
        for (; fixM < kFixesOutM; fixM += 1000.0) {
            double n, e;
            base(fixM, n, e);
            if (above <= kDescentGradient * std::hypot(n - north0, e - east0)) break;
        }
        const auto cas = static_cast<double>(SpeedReference::CalibratedAirspeed);
        double bn, be;
        base(fixM, bn, be);
        points[0].latitudeRad = lat0, points[0].longitudeRad = lon0;
        geo::offsetLatLon(lat0, lon0, bn, be, points[0].latitudeRad, points[0].longitudeRad);
        points[0].altitudeM = elevation + height, points[0].altitudeReference = static_cast<double>(AltitudeReference::Msl);
        points[0].speed = vfix, points[0].speedReference = cas;
        points[1] = along(lat0, lon0, course, -fixM, elevation + height, vfix, cas);
        points[2] = along(lat0, lon0, course, -finalM, elevation + height, vapp, cas);
        points[3] = along(lat0, lon0, course, aim, elevation, vapp, cas);
        for (Waypoint& w : points) w.maxBankRad = kApproachBankRad;
        points[2].kind = points[3].kind = static_cast<double>(EndPointKind::Waypoint); // (flown over: onto the line, down it)
        if (const Reason why = checkRoute(laid, Span<const Waypoint>(points, 4), state, log); why != Reason::None) return why;
        b.params["_vapp"] = vapp, b.params["_aim"] = aim;
    }
    b.params["_thr_lat"] = lat0, b.params["_thr_lon"] = lon0, b.params["_course"] = course, b.params["_length"] = line.lengthM;
    b.params["_elev"] = elevation, b.params["_end"] = laid.end;
    if (profile_ && std::isfinite(profile_->envelope.groundPitchMaxRad)) b.params["_tail"] = profile_->envelope.groundPitchMaxRad;
    laidRoute_ = true; // (written to the path store as a route's, with its activity)
    return Reason::None;
}

// --- flown ----------------------------------------------------------------------------------------------------------------

RecoveryBehavior::RecoveryBehavior()
    : route_(std::make_unique<RouteBehavior>()), rollout_(std::make_unique<LaunchBehavior>()), options_(RouteCommand{}) {}
RecoveryBehavior::~RecoveryBehavior() = default;

void RecoveryBehavior::start(const ControlContext& ctx, const BehaviorCommand& b) {
    hovers_ = (ctx.features & kFeatureHover) != 0;
    thrLat_ = b.param("_thr_lat", ctx.sensed.latitudeRad), thrLon_ = b.param("_thr_lon", ctx.sensed.longitudeRad);
    courseRad_ = b.param("_course", ctx.sensed.eulerRad[2]), lengthM_ = b.param("_length", 2000.0);
    elevationM_ = b.param("_elev", ctx.sensed.altitudeMslM - ctx.sensed.altitudeAglM), vappMs_ = b.param("_vapp", 0.0), aimM_ = b.param("_aim", 0.0);
    tailRad_ = b.param("_tail", kHold);
    airfield_ = b.param("airfield", 0.0), runway_ = b.param("runway", 0.0);
    auto& o = std::get<RouteCommand>(options_);
    o = RouteCommand{};
    o.end = b.param("_end", 0.0);
    phase_ = Phase::Approach, configured_ = false, settledS_ = 0.0, lastS_ = -1.0, descentMslM_ = kHold, speedAddMs_ = 0.0;
    flaps_ = kLeastFlaps;
    route_->begin(ctx, options_);
}

Command RecoveryBehavior::update(const ControlContext& ctx, const Command& in) {
    const auto& s = ctx.sensed;
    const double dt = lastS_ < 0.0 ? 0.0 : std::max(s.simTime - lastS_, 0.0);
    lastS_ = s.simTime;
    return hovers_ ? vertical(ctx, dt) : runway(ctx, in, dt);
}

Command RecoveryBehavior::runway(const ControlContext& ctx, const Command& in, double dt) {
    const auto& s = ctx.sensed;
    if (phase_ == Phase::Rollout) return rollout_->update(ctx, in);
    if (s.onGround) { // touched down: the rollout - idle, its brakes, steered onto the centre line, its nose down, to a stop
        RouteGround line;
        line.startLatitudeRad = thrLat_, line.startLongitudeRad = thrLon_, line.courseRad = courseRad_, line.lengthM = lengthM_;
        line.airfield = airfield_, line.runway = runway_;
        rollout_->startRollout(ctx, line);
        phase_ = Phase::Rollout;
        return rollout_->update(ctx, in);
    }
    Command out = route_->update(ctx, options_);
    ActivityProgress p;
    const std::uint32_t flownTo = route_->progress(p) ? p.segment : 0;
    const double tasPerCas = s.airspeedTrueMs / std::max(s.airspeedCalibratedMs, 1.0);
    // its gear and flaps out from the intermediate fix on, on the centre line, where it slows to its approach speed - a heavy's
    // drag to slow on (the C-130J, configured at the final fix, came down the glide slope at 107 m/s against its 71)
    if (flownTo >= 2) configured_ = true;
    const double high = std::isfinite(tailRad_) ? tailRad_ - kFlareRoomRad : 1e9, attitude = s.eulerRad[1];
    // level to the final fix, at its approach speed: its flaps out, then its speed raised, until it would come down the slope
    // under that attitude - then held down the slope (changed on it, they hunted: the E-7A and the Mirage 2000 porpoised)
    if (flownTo == 2 && phase_ == Phase::Approach && std::abs(s.airspeedCalibratedMs - (vappMs_ + speedAddMs_)) < kSettledShare * vappMs_ &&
        attitude > high + kGlideRad) {
        if (flaps_ < kLandingFlaps) flaps_ = std::min(flaps_ + kFlapsPerS * dt, kLandingFlaps);
        else speedAddMs_ = std::min(speedAddMs_ + kAdjustMsPerDegS * ((attitude - high - kGlideRad) / kDeg) * dt, kMostRaise * vappMs_);
    }
    // down the slope its flaps put out, never back, while it runs fast - their drag (the C-130J, its flight idle's thrust more than
    // its drag, gained 13 m/s down the slope) - and its speed raised, never lowered, while it comes down over that attitude still
    // (straight in, the EA-18G never settled level, and touched down at 6 m/s)
    if (flownTo >= 3 && phase_ == Phase::Approach) {
        if (s.airspeedCalibratedMs > kFastShare * (vappMs_ + speedAddMs_)) flaps_ = std::min(flaps_ + kFlapsPerS * dt, kLandingFlaps);
        if (attitude > high) speedAddMs_ = std::min(speedAddMs_ + kAdjustMsPerDegS * ((attitude - high) / kDeg) * dt, kMostRaise * vappMs_);
    }
    if (auto* fly = std::get_if<VelocityCommand>(&out); fly && phase_ == Phase::Approach) {
        if (speedAddMs_ != 0.0 && !isHold(fly->airspeedMs)) fly->airspeedMs += speedAddMs_ * tasPerCas;
        // its turns no steeper than kTurnBankRad: the route's capture of its first leg - a turn rate on its course's error -
        // banked the EA-18G to 81 deg at 1.6 times its least speed, and it departed (its points' banks are their turns'). Above
        // the route's own turns' kApproachBankRad: held to those, the RC-135W in a crosswind came onto its final 268 m off, the
        // route's course trim wound up
        const double most = 9.80665 * std::tan(kTurnBankRad) / std::max(s.airspeedTrueMs, 1.0);
        if (!isHold(fly->turnRateRadS)) fly->turnRateRadS = std::clamp(fly->turnRateRadS, -most, most);
        const double off = isHold(fly->headingRad) ? 0.0 : geo::wrapPi(fly->headingRad - s.eulerRad[2]);
        if (std::abs(off) > kCaptureRad) fly->turnRateRadS = std::copysign(most, off), fly->headingRad = kHold;
    }
    // the flare: near the runway, on the glide slope's last segment, its sink eased to the height over kFlareS - the shallower of
    // that and the glide slope's - and its speed bled off
    double alongM, crossM;
    onLine(thrLat_, thrLon_, courseRad_, s.latitudeRad, s.longitudeRad, alongM, crossM);
    const auto* v = std::get_if<VelocityCommand>(&out);
    if (!v || flownTo < 3 || alongM < -1500.0) return out;
    const double h = std::max(s.altitudeMslM - elevationM_, 0.0);
    const double sink = -std::max(kFlareSinkMs, h / kFlareS); // (its vertical speed, + up)
    if (phase_ == Phase::Approach) {
        if (sink <= v->verticalSpeedMs) return out; // (the glide slope's still the shallower)
        phase_ = Phase::Flare, flarePitchRad_ = s.eulerRad[1], flareIntegral_ = 0.0;
    }
    // the flare at the attitude level, its pitch raised on the sink's error from where it was - a velocity loop answered too
    // late: the F-16C pitched 8 to 10 deg and touched down at 4.2 m/s - its heading the route's (its crab held), its speed bled
    const double error = sink - (-s.velocityNedMs[2]);
    flareIntegral_ = std::clamp(flareIntegral_ + error * dt, -20.0, 20.0);
    const double most = std::min(flarePitchRad_ + kFlareMostRad, std::isfinite(tailRad_) ? tailRad_ - kTailMarginRad : 1e9);
    const double pitch = std::min(flarePitchRad_ + kFlareKp * error + kFlareKi * flareIntegral_, most);
    return AttitudeCommand{0.0, pitch, v->headingRad, kFlareBankRad, kHold, kFlareSpeedShare * tasPerCas * (vappMs_ + speedAddMs_)};
}

Command RecoveryBehavior::vertical(const ControlContext& ctx, double dt) {
    const auto& s = ctx.sensed;
    configured_ = true; // (its gear down where it has any)
    if (phase_ == Phase::Landed || (phase_ == Phase::Descent && s.onGround && (settledS_ += dt) >= kSettledS)) {
        if (phase_ == Phase::Landed) settledS_ += dt;
        phase_ = Phase::Landed; // on the ground: its collective, its throttle, down
        ActuatorCommand down;
        down.throttle = 0.0, down.gearDown = 1.0;
        return down;
    }
    if (phase_ == Phase::Approach) {
        Command out = route_->update(ctx, options_);
        if (!route_->finished()) return out;
        phase_ = Phase::Descent, descentMslM_ = s.altitudeMslM;
    }
    // straight down over its hover point, at kDescentMs, to below the ground (the ground stops it)
    double lat = thrLat_, lon = thrLon_;
    geo::offsetLatLon(thrLat_, thrLon_, aimM_ * std::cos(courseRad_), aimM_ * std::sin(courseRad_), lat, lon);
    if (!s.onGround) settledS_ = 0.0;
    descentMslM_ = std::max(descentMslM_ - kDescentMs * dt, elevationM_ - 5.0);
    return PositionCommand{lat, lon, descentMslM_, 2.0, 1.0, courseRad_};
}

void RecoveryBehavior::reset() { route_->reset(); }

bool RecoveryBehavior::finished() const noexcept {
    return hovers_ ? phase_ == Phase::Landed && settledS_ >= 2.0 * kSettledS : phase_ == Phase::Rollout && rollout_->finished();
}

Reason RecoveryBehavior::failure() const noexcept { return phase_ == Phase::Rollout ? Reason::None : route_->failure(); }

std::uint16_t RecoveryBehavior::constraints() const noexcept { return phase_ == Phase::Approach ? route_->constraints() : 0; }

bool RecoveryBehavior::progress(ActivityProgress& out) const noexcept {
    if (phase_ == Phase::Rollout) return rollout_->progress(out);
    return route_->progress(out);
}

bool RecoveryBehavior::handOver(BehaviorCommand& out) const {
    return phase_ == Phase::Rollout && rollout_->handOver(out); // (on the runway, stopping: FA's own stop - 4.50)
}

void RecoveryBehavior::configure(ActuatorCommand& out) const noexcept {
    if (!configured_) return;
    out.gearDown = 1.0;
    // its flaps up on the runway, its lift dumped onto its wheels: with them out, at its touchdown speed, the C-130J and the
    // B-52H flew off again 35 m
    if (!hovers_) out.flaps = phase_ == Phase::Rollout ? 0.0 : flaps_;
}

void registerRecovery(ControllerRegistry& r) {
    const double now = kHold;
    auto p = [](const char* name, const char* unit, double def, double lo, double hi) { return ParameterInfo{name, unit, lo, hi, def, true}; };
    BehaviorTraits recovery;
    recovery.persistence = Persistence::Terminating;
    recovery.parameters = {p("airfield", "", now, 1.0, 4294967295.0), p("runway", "", now, 1.0, 4294967295.0)};
    recovery.uses = {"fsim.flight.actuator", "fsim.flight.velocity", "fsim.flight.position"};
    recovery.mode = FlightMode::Recovery;
    recovery.axes = axisBit(Axis::Gear) | axisBit(Axis::Flaps) | axisBit(Axis::Brakes);
    r.addBehavior("recovery", [] { return std::make_unique<RecoveryBehavior>(); }, std::move(recovery));
}

} // namespace fsim::control
