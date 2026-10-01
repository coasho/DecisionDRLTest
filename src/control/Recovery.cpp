// A-GRA's RECOVERY to an airfield's runway (docs/flight-autonomy.md, 4.53; ADR-29 FA-10a: RCV-01, RCV-04): the host resolves
// the runway's landing line at the NEW and lays out the approach as a route - a wing's onto the extended centre line and down
// its glide slope, a rotorcraft's to a hover over the runway - and the "recovery" behaviour flies it, then a wing's flare,
// touchdown and rollout, a rotorcraft's vertical descent to the ground.
#include "control/CapabilityHost.h"
#include "control/PlanStore.h"
#include "control/Registry.h"
#include "control/Route.h"
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
// its drag devices (4.55): opened on an approach where they are (its profile's: airbrakes, not spoilers that dump lift - the
// E-7A, its spoilers opened as it pushed over onto its glide slope, oscillated and went around), out on the glide slope while
// its throttle is at idle and it still runs fast, never back, to kSpeedbrakeAir at most, and all out from its touchdown on: a transport's ground spoilers, a
// fighter's airbrakes
constexpr double kSpeedbrakeAir = 1.0;
constexpr double kIdleThrottle = 0.02;       // ...its throttle this far back: at idle...
constexpr double kGainingMs2 = 0.05;          // ...gaining speed still, its calibrated airspeed's rate over kRateS, kSlopeS on the
constexpr double kRateS = 5.0;                // glide slope at least (as it pushes over onto it, it gains a little for a while; the
constexpr double kSlopeS = 10.0;              // B-52H gained 6 m/s over half a minute)
// a rotorcraft's approach point on the centre line, 60 s of its cruise out and 10 s up - 30 m to 1 km, 100 m at most - then its
// hover 2 s up (1.5 to 10 m) this far beyond the threshold: 1 km out and 100 m up, the Crazyflie fell
constexpr double kRotorApproachS = 60.0, kRotorApproachUpS = 10.0, kRotorHoverS = 2.0;
constexpr double kRotorApproachLeastM = 30.0, kRotorApproachMostM = 1000.0, kRotorApproachMostAglM = 100.0;
constexpr double kHoverLeastAglM = 1.5, kHoverMostAglM = 10.0;
constexpr double kHoverAlongM = 60.0;
constexpr double kDescentMs = 0.7;            // its vertical descent
constexpr double kSettledS = 1.0;             // on the ground so long: landed
// its go-arounds (4.54): an approach not stable below kGateAglM - off the centre line more than kLateralShare of its way to the
// aim (kLateralLeastM at least), off the glide slope more than kVerticalRad (one dot: kVerticalLeastM at least), slower than
// its approach speed by kSlowMs or faster by kFastMs, or sinking kSinkMarginMs faster than the slope asks (kSinkLeastMs at
// least) - for kUnstableS; a crosswind beyond its limit on the centre line for as long; a bounce of kBounceM; no touchdown
// by the touchdown zone's end, the farther of kZoneM beyond its aim and a third of the runway (a half at most). It
// approaches kApproaches times at most, then flies its missed approach and fails
constexpr double kGateAglM = 150.0;           // 500 ft: the stabilized approach's gate in sight of the runway (FSF ALAR 7.1)
constexpr double kLateralShare = 0.02, kLateralLeastM = 15.0;
constexpr double kVerticalRad = 0.35 * kDeg, kVerticalLeastM = 8.0;
constexpr double kSlowMs = 5.0 * 0.514444, kFastMs = 20.0 * 0.514444; // Vref -5 kt to +20 kt (FSF ALAR 7.1)
constexpr double kSinkLeastMs = 1000.0 * 0.3048 / 60.0, kSinkMarginMs = 2.0; // 1,000 ft/min, or the slope's sink and 2 m/s
constexpr double kUnstableS = 2.0;
constexpr double kBounceM = 3.0;
constexpr double kZoneM = 450.0;
constexpr std::uint32_t kApproaches = 2;
constexpr double kCircuitLeastAglM = 300.0;   // its go-around's climb to its intermediate fix's altitude, this high at least
constexpr double kClimbedM = 30.0;            // ...within this of it, past the runway's end: its missed approach
constexpr std::uint32_t kMissedMost = 16;     // FA's chained missed approach: so many of its points at most
constexpr double kGoAroundLookM = 2000.0;     // its go-around back onto the centre line, on a point this far ahead

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

bool CapabilityHost::beyondCrosswind(double courseRad, const sim::VehicleState& state) const noexcept {
    if (!profile_ || !std::isfinite(profile_->envelope.crosswindMaxMs)) return false;
    const WindEstimate wind = checkWind(state);
    const double across = isHold(courseRad) ? std::hypot(wind.northMs, wind.eastMs)
                                            : std::abs(-wind.northMs * std::sin(courseRad) + wind.eastMs * std::cos(courseRad));
    return wind.valid && across > profile_->envelope.crosswindMaxMs;
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
    Waypoint points[4 + kMissedMost];
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
        for (std::uint32_t k = 0; k < 4; ++k) points[k].maxBankRad = kApproachBankRad;
        points[2].kind = points[3].kind = static_cast<double>(EndPointKind::Waypoint); // (flown over: onto the line, down it)
        // its missed approach (RCV-03): FA's own landing path for this runway - a path of a plan FA keeps, its type Landing, its
        // airfield and runway these - and the path a branch of it leads to, from where it leads on to its end, laid after the
        // approach (VI 1.2.6.3: "additional defined Path for missed approach hold procedure, along with conditional chaining to
        // tie it to the main approach route"). Without one, the circuit back to the approach
        std::uint32_t count = 4;
        static const std::vector<PlanEntry> kNoPlans;
        for (const PlanEntry& e : plans_ ? plans_->plans : kNoPlans) {
            const RoutePlan& p = e.kept;
            for (std::size_t i = 0; e.faOwned && count == 4 && i < p.paths.size(); ++i) {
                const bool here = p.paths[i].type == static_cast<double>(PathType::Landing);
                bool named = false;
                for (const PathMetadata& m : p.pathMetadata)
                    named = named || (m.path == i && static_cast<double>(m.airfield) == line.airfield && static_cast<double>(m.runway) == line.runway);
                if (!here || !named) continue;
                const RoutePath& landing = p.paths[i];
                for (const RouteBranch& branch : p.branches) {
                    if (count > 4 || branch.point < landing.first || branch.point >= landing.first + landing.count || !(branch.next >= 0.0)) continue;
                    const auto next = static_cast<std::uint32_t>(branch.next);
                    for (const RoutePath& q : p.paths) {
                        if (&q == &landing || next < q.first || next >= q.first + q.count) continue;
                        for (std::uint32_t k = next; k < q.first + q.count && k < p.waypoints.size() && count < 4 + kMissedMost; ++k) {
                            points[count] = p.waypoints[k];
                            points[count].next = points[count].terminator = kHold; // (flown in order, its legs straight)
                            ++count;
                        }
                        break;
                    }
                }
            }
        }
        if (const Reason why = checkRoute(laid, Span<const Waypoint>(points, count), state, log); why != Reason::None) return why;
        b.params["_vapp"] = vapp, b.params["_aim"] = aim, b.params["_missed"] = count > 4 ? 4.0 : 0.0;
        b.params["_circuit"] = std::max(points[1].altitudeM, elevation + kCircuitLeastAglM);
        if (profile_ && std::isfinite(profile_->envelope.crosswindMaxMs)) b.params["_xwind"] = profile_->envelope.crosswindMaxMs;
        if (profile_ && std::isfinite(profile_->envelope.gearCasMaxMs)) b.params["_gear_max"] = profile_->envelope.gearCasMaxMs;
        if (profile_ && std::isfinite(profile_->envelope.flaps.casMaxMs))
            b.params["_flaps_max"] = profile_->envelope.flaps.casMaxMs, b.params["_flaps_above"] = profile_->envelope.flapsThreshold;
        if (profile_ && profile_->effectors.speedbrakeApproach) b.params["_sb_air"] = 1.0; // (4.55)
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
    flaps_ = kLeastFlaps, speedbrake_ = 0.0, casRateMs2_ = 0.0, lastCasMs_ = ctx.sensed.airspeedCalibratedMs, slopeS_ = 0.0;
    // its go-arounds (4.54)
    crosswindMaxMs_ = b.param("_xwind", kHold), circuitMslM_ = b.param("_circuit", elevationM_ + kCircuitLeastAglM);
    missedFrom_ = static_cast<std::uint32_t>(b.param("_missed", 0.0));
    airbrakes_ = b.param("_sb_air", 0.0) != 0.0;
    gearMaxMs_ = b.param("_gear_max", kHold), flapsMaxMs_ = b.param("_flaps_max", kHold), flapsAbove_ = b.param("_flaps_above", 0.0);
    casMs_ = ctx.sensed.airspeedCalibratedMs, gearOut_ = false;
    zoneEndM_ = std::min(std::max(aimM_ + kZoneM, lengthM_ / 3.0), lengthM_ / 2.0);
    wind_.reset();
    touchAglM_ = unstableS_ = crosswindS_ = 0.0;
    goArounds_ = 0, cause_ = GoAround::None, gearUp_ = lastApproach_ = false, failed_ = Reason::None;
    route_->begin(ctx, options_);
}

void RecoveryBehavior::goAroundFrom(GoAround cause) noexcept {
    // full power, climbing straight ahead at the speed it turned onto its approach at, its flaps back to a takeoff's; its gear
    // up once it climbs (ICAO Doc 8168's missed approach: the climb straight on, then the turn)
    phase_ = Phase::GoAround, cause_ = cause, ++goArounds_;
    lastApproach_ = goArounds_ >= kApproaches;
    flaps_ = kLeastFlaps, speedbrake_ = 0.0, speedAddMs_ = 0.0, unstableS_ = crosswindS_ = 0.0;
}

void RecoveryBehavior::approachAgain(const ControlContext& ctx, std::uint32_t from) {
    // its route again from `from`: the approach's base point (0), or its chained missed approach's first
    auto& o = std::get<RouteCommand>(options_);
    o.start = static_cast<double>(from); // (kept: the route plans afresh when its options change)
    route_->begin(ctx, options_);
    phase_ = from == 0 ? Phase::Approach : Phase::Missed;
    configured_ = gearUp_ = gearOut_ = false, flaps_ = kLeastFlaps, speedbrake_ = 0.0, speedAddMs_ = 0.0, slopeS_ = 0.0;
}

Command RecoveryBehavior::goAround(const ControlContext& ctx, double alongM, double crossM) {
    const auto& s = ctx.sensed;
    static const Performance kNone{};
    const Performance& perf = ctx.performance ? *ctx.performance : kNone;
    // its gear up once it climbs - or level at its altitude, waved off there - clear of the ground
    if (!gearUp_ && (-s.velocityNedMs[2] > 1.0 || s.altitudeMslM >= circuitMslM_ - kClimbedM) && s.altitudeAglM > 15.0) gearUp_ = true;
    if (s.altitudeMslM >= circuitMslM_ - kClimbedM && alongM >= lengthM_) { // climbed out: its missed approach
        if (lastApproach_ && missedFrom_ == 0) {
            failed_ = cause_ == GoAround::Crosswind ? Reason::CrosswindLimit : Reason::LandingAbandoned;
        } else {
            approachAgain(ctx, missedFrom_);
            return route_->update(ctx, options_);
        }
    }
    // along the extended centre line, its heading crabbed into the wind (on the runway's heading alone, the F-16C drifted 2.8 km
    // across 15 m/s before its missed approach), to the runway's end - its missed approach point - and its altitude
    const double tasPerCas = s.airspeedTrueMs / std::max(s.airspeedCalibratedMs, 1.0);
    const double across = -wind_.northMs * std::sin(courseRad_) + wind_.eastMs * std::cos(courseRad_); // (to the right of it)
    const double track = courseRad_ - std::clamp(std::atan(crossM / kGoAroundLookM), -0.5, 0.5);
    const double heading = geo::wrapPi(track - std::asin(std::clamp(across / std::max(s.airspeedTrueMs, 10.0), -0.5, 0.5)));
    return VelocityCommand{kManoeuvreShare * vappMs_ * tasPerCas, route::verticalSpeedTo(circuitMslM_, 0.0, s, perf, false), heading, kHold, kHold, kHold};
}

Command RecoveryBehavior::update(const ControlContext& ctx, const Command& in) {
    const auto& s = ctx.sensed;
    const double dt = lastS_ < 0.0 ? 0.0 : std::max(s.simTime - lastS_, 0.0);
    lastS_ = s.simTime;
    return hovers_ ? vertical(ctx, dt) : runway(ctx, in, dt);
}

Command RecoveryBehavior::runway(const ControlContext& ctx, const Command& in, double dt) {
    const auto& s = ctx.sensed;
    wind_.update(s, dt);
    // its placards (4.57): its gear lowered only below its speed (configure), and kept down
    casMs_ = s.airspeedCalibratedMs;
    if (configured_ && !gearOut_ && !(casMs_ > gearMaxMs_)) gearOut_ = true;
    double alongM, crossM;
    onLine(thrLat_, thrLon_, courseRad_, s.latitudeRad, s.longitudeRad, alongM, crossM);
    if (phase_ == Phase::Rollout) {
        // a bounce of more than kBounceM, still fast: a balked landing - it goes around (4.54)
        if (s.onGround || s.altitudeAglM - touchAglM_ <= kBounceM || s.airspeedCalibratedMs < 0.8 * vappMs_) return rollout_->update(ctx, in);
        goAroundFrom(GoAround::Bounce);
    }
    if (phase_ == Phase::GoAround) return goAround(ctx, alongM, crossM);
    if (phase_ == Phase::Missed) { // its chained missed approach flown: approaching again, or on its last, failed at its end
        Command out = route_->update(ctx, options_);
        if (!route_->finished()) return out;
        if (lastApproach_) failed_ = cause_ == GoAround::Crosswind ? Reason::CrosswindLimit : Reason::LandingAbandoned;
        else approachAgain(ctx, 0);
        return route_->update(ctx, options_);
    }
    if (s.onGround) { // touched down: the rollout - idle, its brakes, steered onto the centre line, its nose down, to a stop
        RouteGround line;
        line.startLatitudeRad = thrLat_, line.startLongitudeRad = thrLon_, line.courseRad = courseRad_, line.lengthM = lengthM_;
        line.airfield = airfield_, line.runway = runway_;
        rollout_->startRollout(ctx, line);
        phase_ = Phase::Rollout, touchAglM_ = s.altitudeAglM;
        return rollout_->update(ctx, in);
    }
    Command out = route_->update(ctx, options_);
    ActivityProgress p;
    const std::uint32_t flownTo = route_->progress(p) ? p.segment : 0;
    // on its final, not down by the touchdown zone's end: it goes around (the B-52H, with no airbrakes, floated 3.9 km and
    // touched down past the runway's end)
    if (alongM > zoneEndM_ && (phase_ == Phase::Flare || flownTo == 3)) {
        goAroundFrom(GoAround::Long);
        return goAround(ctx, alongM, crossM);
    }
    const double tasPerCas = s.airspeedTrueMs / std::max(s.airspeedCalibratedMs, 1.0);
    // on the centre line: a crosswind beyond its limit waves it off (the Skua, its approach speed 17 m/s, never reached its
    // glide slope across 10 m/s); below the gate, an approach not stable sends it around
    if ((flownTo == 2 || flownTo == 3) && phase_ == Phase::Approach) {
        const double across = std::abs(-wind_.northMs * std::sin(courseRad_) + wind_.eastMs * std::cos(courseRad_));
        crosswindS_ = std::isfinite(crosswindMaxMs_) && wind_.valid && across > crosswindMaxMs_ ? crosswindS_ + dt : 0.0;
        const double h = s.altitudeMslM - elevationM_, toAim = std::max(aimM_ - alongM, 0.0);
        const double sink = s.velocityNedMs[2], slopeSink = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]) * std::tan(kGlideRad);
        const double speed = s.airspeedCalibratedMs - (vappMs_ + speedAddMs_);
        const bool unstable = h < kGateAglM && (std::abs(crossM) > std::max(kLateralLeastM, kLateralShare * toAim) ||
                                                std::abs(h - toAim * std::tan(kGlideRad)) > std::max(kVerticalLeastM, toAim * std::tan(kVerticalRad)) ||
                                                speed < -kSlowMs || speed > kFastMs || sink > std::max(kSinkLeastMs, slopeSink + kSinkMarginMs));
        unstableS_ = unstable ? unstableS_ + dt : 0.0;
        if (crosswindS_ >= kUnstableS || unstableS_ >= kUnstableS) {
            goAroundFrom(crosswindS_ >= kUnstableS ? GoAround::Crosswind : GoAround::Unstable);
            return goAround(ctx, alongM, crossM);
        }
    }
    // its gear and flaps out from the intermediate fix on, on the centre line, where it slows to its approach speed - a heavy's
    // drag to slow on (the C-130J, configured at the final fix, came down the glide slope at 107 m/s against its 71)
    if (flownTo >= 2 && flownTo <= 3) configured_ = true;
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
    if (dt > 0.0) casRateMs2_ += std::min(dt / kRateS, 1.0) * ((s.airspeedCalibratedMs - lastCasMs_) / dt - casRateMs2_);
    lastCasMs_ = s.airspeedCalibratedMs;
    if (flownTo == 3 && phase_ == Phase::Approach) {
        if (s.airspeedCalibratedMs > kFastShare * (vappMs_ + speedAddMs_)) flaps_ = std::min(flaps_ + kFlapsPerS * dt, kLandingFlaps);
        // its drag devices once its throttle is at idle and it still runs fast: the B-52H, nose low, its flaps at a takeoff's,
        // gained 6 m/s down the slope at idle, was above it by the gate and went around
        slopeS_ += dt;
        if (airbrakes_ && s.airspeedCalibratedMs > vappMs_ + speedAddMs_ && s.throttlePosition[0] <= kIdleThrottle &&
            casRateMs2_ > kGainingMs2 && slopeS_ >= kSlopeS)
            speedbrake_ = std::min(speedbrake_ + kFlapsPerS * dt, kSpeedbrakeAir);
        if (attitude > high) speedAddMs_ = std::min(speedAddMs_ + kAdjustMsPerDegS * ((attitude - high) / kDeg) * dt, kMostRaise * vappMs_);
    }
    if (auto* fly = std::get_if<VelocityCommand>(&out); fly && phase_ == Phase::Approach) {
        if (speedAddMs_ != 0.0 && !isHold(fly->airspeedMs)) fly->airspeedMs += speedAddMs_ * tasPerCas;
        // its route's turns are bounded by their planned bank (4.54); a heading far off is turned to at kTurnBankRad
        const double off = isHold(fly->headingRad) ? 0.0 : geo::wrapPi(fly->headingRad - s.eulerRad[2]);
        if (std::abs(off) > kCaptureRad)
            fly->turnRateRadS = std::copysign(9.80665 * std::tan(kTurnBankRad) / std::max(s.airspeedTrueMs, 1.0), off), fly->headingRad = kHold;
    }
    // the flare: near the runway, on the glide slope's last segment, its sink eased to the height over kFlareS - the shallower of
    // that and the glide slope's - and its speed bled off
    const auto* v = std::get_if<VelocityCommand>(&out);
    if (!v || flownTo != 3 || alongM < -1500.0) return out;
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

Reason RecoveryBehavior::failure() const noexcept {
    if (failed_ != Reason::None) return failed_;
    return phase_ == Phase::Rollout || phase_ == Phase::GoAround ? Reason::None : route_->failure();
}

std::uint16_t RecoveryBehavior::constraints() const noexcept {
    return phase_ == Phase::Approach || phase_ == Phase::Missed ? route_->constraints() : 0;
}

bool RecoveryBehavior::progress(ActivityProgress& out) const noexcept {
    if (phase_ == Phase::Rollout) return rollout_->progress(out);
    return route_->progress(out);
}

bool RecoveryBehavior::handOver(BehaviorCommand& out) const {
    return phase_ == Phase::Rollout && rollout_->handOver(out); // (on the runway, stopping: FA's own stop - 4.50)
}

void RecoveryBehavior::configure(ActuatorCommand& out) const noexcept {
    if (phase_ == Phase::GoAround) { // (its flaps a takeoff's, its gear up once it climbs, its brakes off)
        out.flaps = placard(flaps_), out.gearDown = gearUp_ ? 0.0 : 1.0, out.brakeLeft = out.brakeRight = 0.0;
        return;
    }
    if (phase_ == Phase::Missed) { // (its chained missed approach flown clean)
        out.flaps = 0.0, out.gearDown = 0.0;
        return;
    }
    if (!configured_) return;
    out.gearDown = gearOut_ || !(casMs_ > gearMaxMs_) || phase_ == Phase::Rollout ? 1.0 : 0.0;
    // its flaps up on the runway, its lift dumped onto its wheels: with them out, at its touchdown speed, the C-130J and the
    // B-52H flew off again 35 m
    if (!hovers_) out.flaps = phase_ == Phase::Rollout ? 0.0 : placard(flaps_);
}

double RecoveryBehavior::placard(double flaps) const noexcept {
    // its flaps out past the setting before its landing flaps only below their placard (4.57) - back to it above, as a 767's
    // flaps 25 and 30 do themselves; its protection holds it below the placard once they are out
    return casMs_ > flapsMaxMs_ ? std::min(flaps, flapsAbove_) : flaps;
}

double RecoveryBehavior::speedbrake() const noexcept {
    if (hovers_) return kHold;
    if (phase_ == Phase::Rollout) return 1.0; // (its ground spoilers, its airbrakes: its lift dumped onto its wheels)
    return phase_ == Phase::Approach || phase_ == Phase::Flare ? speedbrake_ : 0.0;
}

void registerRecovery(ControllerRegistry& r) {
    const double now = kHold;
    auto p = [](const char* name, const char* unit, double def, double lo, double hi) { return ParameterInfo{name, unit, lo, hi, def, true}; };
    BehaviorTraits recovery;
    recovery.persistence = Persistence::Terminating;
    recovery.parameters = {p("airfield", "", now, 1.0, 4294967295.0), p("runway", "", now, 1.0, 4294967295.0)};
    recovery.uses = {"fsim.flight.actuator", "fsim.flight.velocity", "fsim.flight.position"};
    recovery.mode = FlightMode::Recovery;
    recovery.axes = axisBit(Axis::Gear) | axisBit(Axis::Flaps) | axisBit(Axis::Brakes) | axisBit(Axis::Speedbrake);
    r.addBehavior("recovery", [] { return std::make_unique<RecoveryBehavior>(); }, std::move(recovery));
}

} // namespace fsim::control
