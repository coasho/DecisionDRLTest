// Where a commanded path meets the terrain (docs/flight-autonomy.md, 4.19; A-GRA's VIOLATION_TERRAIN and its
// TerrainConstraint): the first place along a route, a pattern, a curve or an hsa's line ahead that is below the
// ground. Checked at a NEW or an UPDATE, never stepped: in a file of its own.
#include "control/CapabilityHost.h"

#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kR = geo::kEarthRadiusM;
constexpr double kPi = 3.14159265358979323846;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
/// The most samples a piece of a path is checked at: past them, the spacing grows (a leg across a continent).
constexpr double kMaxSamples = 100000.0;
/// How long a path with no end is checked ahead, s: an hsa's line, a route's or a curve's course on past its end.
constexpr double kAheadS = 60.0;

bool aboveGround(double reference) noexcept { return reference == static_cast<double>(AltitudeReference::AboveGround); }

/// A leg's point `alongM` from its start, on past its end too (Route.h's Leg: a great circle, or a rhumb line).
void onLegAt(const route::Leg& leg, double alongM, double& lat, double& lon) noexcept {
    if (leg.rhumb) { // its latitude changes with the distance, its longitude with the stretched latitude
        const double f = leg.lengthM > 0.0 ? alongM / leg.lengthM : 0.0;
        lat = std::clamp(leg.latA + f * (leg.latB - leg.latA), -0.5 * kPi, 0.5 * kPi);
        const double psi = std::log(std::tan(0.25 * kPi + 0.5 * std::clamp(lat, -1.5, 1.5)));
        lon = geo::wrapPi(leg.lonA + (std::abs(leg.dPsi) > 1e-12 ? (psi - leg.psiA) / leg.dPsi : f) * leg.dLon);
        return;
    }
    // round its great circle from a, the way it runs there (n x a)
    const double t[3] = {leg.n[1] * leg.a[2] - leg.n[2] * leg.a[1], leg.n[2] * leg.a[0] - leg.n[0] * leg.a[2], leg.n[0] * leg.a[1] - leg.n[1] * leg.a[0]};
    const double c = std::cos(alongM / kR), s = std::sin(alongM / kR);
    const double p[3] = {leg.a[0] * c + t[0] * s, leg.a[1] * c + t[1] * s, leg.a[2] * c + t[2] * s};
    lat = std::asin(std::clamp(p[2], -1.0, 1.0));
    lon = std::atan2(p[1], p[0]);
}

/// The point `distanceM` from (lat, lon) along the great circle leaving it on `courseRad`.
void ahead(double lat, double lon, double courseRad, double distanceM, double& latOut, double& lonOut) noexcept {
    const double d = distanceM / kR;
    latOut = std::asin(std::clamp(std::sin(lat) * std::cos(d) + std::cos(lat) * std::sin(d) * std::cos(courseRad), -1.0, 1.0));
    lonOut = geo::wrapPi(lon + std::atan2(std::sin(courseRad) * std::sin(d) * std::cos(lat), std::cos(d) - std::sin(lat) * std::sin(latOut)));
}

/// A route leg's altitude in its reference, `x` metres from its start, as the route flies it: straight to its
/// point, or at its climb rate and then level; on past its end, on to the point's altitude at the rate.
struct Profile {
    double from = 0.0, to = 0.0;
    double rateMs = kNaN; ///< its climb rate; NaN: along its gradient
    double speedMs = 0.0, lengthM = 0.0;
    double at(double x) const noexcept {
        if (rateMs > 0.0 && speedMs > 0.0) return from + std::copysign(std::min(rateMs * x / speedMs, std::abs(to - from)), to - from);
        return lengthM > 1.0 ? from + (to - from) * std::clamp(x / lengthM, 0.0, 1.0) : to;
    }
    /// How far past its point it is still climbing or descending to it.
    double pastM() const noexcept { return rateMs > 0.0 && speedMs > 0.0 ? std::max(std::abs(to - from) / rateMs * speedMs - lengthM, 0.0) : 0.0; }
};

/// The path walked piece by piece, each sampled at the spacing (its ends too), and the time along it at its
/// speeds: where a sample is below the ground, the place it first goes below, found between it and the one before.
struct Walk {
    const SessionView& view;
    double spacingM = 30.0;
    double timeS = 0.0; ///< to the start of the piece being walked
    CommandDetails::Terrain hit{}; ///< (its index: the route point flown to, or the curve segment)

    /// `at(x)` gives the place and the altitude `x` metres into a piece `lengthM` long, flown at `speedMs`;
    /// the altitude above the ground where `above`. At least `samples` samples (a curve's height is not straight).
    /// True once it meets the terrain.
    template <typename At>
    bool piece(double lengthM, double speedMs, bool above, std::int16_t i, At&& at, int samples = 1) {
        const double length = std::isfinite(lengthM) ? std::max(lengthM, 0.0) : 0.0;
        const int n = static_cast<int>(std::clamp(std::ceil(length / spacingM), static_cast<double>(samples), kMaxSamples));
        CommandDetails::Terrain here{};
        auto below = [&](double x) { // (a NaN altitude judges nothing)
            double lat = 0.0, lon = 0.0, altitude = 0.0;
            at(x, lat, lon, altitude);
            const double ground = view.groundM(lat, lon);
            const double msl = above ? ground + altitude : altitude;
            here.latitudeRad = lat, here.longitudeRad = lon, here.altitudeMslM = msl, here.groundM = ground;
            return msl < ground;
        };
        for (int k = 0; k <= n; ++k) {
            double x = length * k / n;
            if (!below(x)) continue;
            // between the sample before, clear, and this one: where it goes below, to a tenth of a metre
            double clear = k > 0 ? length * (k - 1) / n : x;
            CommandDetails::Terrain first = here;
            for (int step = 0; step < 60 && x - clear > 0.1; ++step) {
                const double middle = 0.5 * (clear + x);
                if (below(middle)) x = middle, first = here;
                else clear = middle;
            }
            hit = first;
            hit.hit = 1;
            hit.timeS = timeS + (speedMs > 0.0 ? x / speedMs : 0.0);
            hit.index = i;
            return true;
        }
        if (speedMs > 0.0) timeS += length / speedMs;
        return false;
    }
    /// An arc `lengthM` long round a centre `centreNorthM`, `centreEastM` from (lat0, lon0), from `startBearingRad`
    /// the way `way` gives (+ clockwise; round and round, if longer than the circle), at `height(x)`.
    template <typename Height>
    bool arc(double lat0, double lon0, double centreNorthM, double centreEastM, double radiusM, double startBearingRad, double way,
             double lengthM, double speedMs, bool above, std::int16_t i, Height&& height) {
        const double side = way >= 0.0 ? 1.0 : -1.0;
        return piece(lengthM, speedMs, above, i, [&](double x, double& lat, double& lon, double& h) {
            const double bearing = startBearingRad + side * (radiusM > 0.0 ? x / radiusM : 0.0);
            geo::offsetLatLon(lat0, lon0, centreNorthM + radiusM * std::cos(bearing), centreEastM + radiusM * std::sin(bearing), lat, lon);
            h = height(x);
        });
    }
};

} // namespace

CommandDetails::Terrain CapabilityHost::terrain(const Command& setpoint, const sim::VehicleState& state) const noexcept {
    if (!sessionView_) return {};
    Walk walk{*sessionView_};
    const double resolution = sessionView_->groundResolutionM();
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    auto spacing = [&](double lengthM) { walk.spacingM = std::max(resolution, std::isfinite(lengthM) ? lengthM / kMaxSamples : 0.0); };
    // a place's altitude in a reference, above sea level: above ground, the ground's there
    auto msl = [&](double altitude, bool above, double lat, double lon) { return above ? altitude + sessionView_->groundM(lat, lon) : altitude; };

    if (std::get_if<RouteCommand>(&setpoint) && routePlan_ && routePlan_->count > 0) {
        // each leg from where the one before left the aircraft, its altitude in its own reference, and its fly-by turn
        // at its point; round again once where it repeats; else what it flies after its last point, until it has
        // settled at its altitude, and a minute on (a lap, round its point)
        const route::Plan& p = *routePlan_;
        spacing(p.lapM(true) * (p.repeat ? 2.0 : 1.0));
        double fromMsl = state.altitudeMslM;
        Profile last;
        auto leg = [&](std::uint32_t i, bool firstLap) {
            const Waypoint& w = p.points[i];
            const route::Leg& l = p.leg(i, firstLap);
            const bool above = aboveGround(w.altitudeReference);
            Profile f;
            f.from = firstLap && i == p.start ? (above ? state.altitudeAglM : state.altitudeMslM)
                     : above                  ? fromMsl - sessionView_->groundM(l.latA, l.lonA)
                                              : fromMsl;
            f.to = isHold(w.altitudeM) ? f.from : w.altitudeM;
            f.rateMs = isHold(w.climbRateMs) ? kNaN : w.climbRateMs;
            f.speedMs = route::plannedSpeed(w.speed, w.speedReference, msl(f.to, above, w.latitudeRad, w.longitudeRad));
            f.lengthM = l.lengthM;
            const route::Turn* before = p.turnBefore(i, firstLap);
            const route::Turn& turn = p.turn(i, firstLap);
            const bool turns = (i + 1 < p.count || p.repeat) && turn.radiusM > 0.0;
            const double start = before ? before->leadM : 0.0, end = l.lengthM - (turns ? turn.leadM : 0.0);
            const double arcM = turns ? turn.radiusM * std::abs(turn.angleRad) : 0.0;
            const auto index = static_cast<std::int16_t>(i);
            if (walk.piece(end - start, f.speedMs, above, index, [&](double x, double& lat, double& lon, double& h) {
                    onLegAt(l, start + x, lat, lon);
                    h = f.at(start + x);
                }))
                return true;
            if (turns && walk.arc(w.latitudeRad, w.longitudeRad, turn.centreNorthM, turn.centreEastM, turn.radiusM, turn.entryBearingRad, turn.angleRad,
                                  arcM, f.speedMs, above, index, [&](double x) { return f.at(end + x); }))
                return true;
            fromMsl = msl(f.at(end + arcM), above, w.latitudeRad, w.longitudeRad);
            last = f;
            return false;
        };
        for (std::uint32_t i = p.start; i < p.count; ++i)
            if (leg(i, true)) return walk.hit;
        if (p.repeat) {
            for (std::uint32_t i = 0; i < p.count; ++i)
                if (leg(i, false)) return walk.hit;
            return walk.hit;
        }
        const Waypoint& w = p.points[p.last()];
        const bool above = aboveGround(w.altitudeReference);
        const auto index = static_cast<std::int16_t>(p.last());
        auto height = [&](double x) { return last.at(last.lengthM + x); };
        if (p.end == EndBehavior::Loiter && hovers) { // a rotorcraft stops over the point, and settles at its altitude
            walk.piece(0.0, 0.0, above, index, [&](double, double& lat, double& lon, double& h) { lat = w.latitudeRad, lon = w.longitudeRad, h = last.to; });
        } else if (p.end == EndBehavior::Loiter) { // a wing orbits the point
            const double radius = p.orbit.radiusM > 0.0 ? p.orbit.radiusM : performance_.turnRadiusM(last.speedMs);
            walk.arc(w.latitudeRad, w.longitudeRad, 0.0, 0.0, radius, 0.0, 1.0, last.pastM() + 2.0 * kPi * radius, last.speedMs, above, index, height);
        } else { // on along its last leg
            const route::Leg& l = p.leg(p.last(), p.last() == p.start);
            walk.piece(last.pastM() + std::max(last.speedMs, 0.0) * kAheadS, last.speedMs, above, index, [&](double x, double& lat, double& lon, double& h) {
                onLegAt(l, l.lengthM + x, lat, lon);
                h = height(x);
            });
        }
        return walk.hit;
    }

    if (const auto* c = std::get_if<PatternCommand>(&setpoint)) { // its lap, and a racetrack's or a hold's entry to its fix
        route::Pattern p;
        route::planPattern(p, *c, state.latitudeRad, state.longitudeRad);
        const bool above = aboveGround(c->altitudeReference);
        const double speed = route::plannedSpeed(c->speed, c->speedReference, msl(c->altitudeM, above, c->latitudeRad, c->longitudeRad));
        auto level = [&](double) { return c->altitudeM; };
        spacing(p.entry.lengthM + p.lapM());
        auto line = [&](const route::Line& l) {
            return walk.piece(l.lengthM, speed, above, -1, [&](double x, double& lat, double& lon, double& h) {
                geo::offsetLatLon(p.lat0, p.lon0, l.northM + x * std::cos(l.courseRad), l.eastM + x * std::sin(l.courseRad), lat, lon);
                h = c->altitudeM;
            });
        };
        if (p.entry.lengthM > 0.0 && line(p.entry)) return walk.hit;
        for (std::uint32_t k = 0, i = p.first; k < p.count; ++k, i = p.next(i)) {
            const route::Pattern::Piece& piece = p.pieces[i];
            if (piece.arc ? walk.arc(p.lat0, p.lon0, piece.turn.centreNorthM, piece.turn.centreEastM, piece.turn.radiusM, piece.turn.entryBearingRad,
                                     piece.turn.angleRad, piece.turn.radiusM * piece.sweepRad, speed, above, -1, level)
                          : line(piece.line))
                return walk.hit;
        }
        return walk.hit;
    }

    if (const auto* c = std::get_if<CurveCommand>(&setpoint); c && curvePlan_ && curvePlan_->count > 0) {
        // along its segments at the speed it flies them, then on along its last course or round its end - from the
        // command's reference (the check filled it in: the scratch plan has its segments alone)
        const route::Curve& k = *curvePlan_;
        const double lat0 = isHold(c->latitudeRad) ? state.latitudeRad : c->latitudeRad;
        const double lon0 = isHold(c->longitudeRad) ? state.longitudeRad : c->longitudeRad;
        const double alt0 = isHold(c->altitudeM) ? state.altitudeMslM : c->altitudeM;
        const double length = k.lengthM();
        double speed = hovers ? std::hypot(state.velocityNedMs[0], state.velocityNedMs[1]) : state.airspeedTrueMs;
        if (!isHold(c->speedMaxMs)) speed = std::min(speed, c->speedMaxMs);
        if (!isHold(c->speedMinMs)) speed = std::max(speed, c->speedMinMs);
        if (!isHold(c->durationS) && c->durationS > 0.0) speed = length / c->durationS;
        spacing(length);
        for (std::uint32_t i = 0; i < k.count; ++i) // (each segment sampled 32 times at least: its height is not straight)
            if (walk.piece(k.startM[i + 1] - k.startM[i], speed, false, static_cast<std::int16_t>(i),
                           [&](double x, double& lat, double& lon, double& h) {
                               std::uint32_t j = 0;
                               double t = 0.0;
                               k.find(k.startM[i] + x, j, t);
                               const route::CurvePoint cp = route::evaluate(k.segments[j], t);
                               geo::offsetLatLon(lat0, lon0, cp.p[0], cp.p[1], lat, lon);
                               h = alt0 - cp.p[2];
                           },
                           route::Curve::kSamples))
                return walk.hit;
        const route::CurvePoint end = route::evaluate(k.segments[k.count - 1], 1.0);
        const double altitude = alt0 - end.p[2];
        const auto index = static_cast<std::int16_t>(k.count - 1);
        if (!isHold(c->end) && c->end == static_cast<double>(EndBehavior::Loiter)) { // a wing orbits its end; a rotorcraft stops over it
            const double radius = hovers ? 0.0 : performance_.turnRadiusM(speed);
            walk.arc(lat0, lon0, end.p[0], end.p[1], radius, 0.0, 1.0, 2.0 * kPi * radius, speed, false, index, [&](double) { return altitude; });
        } else {
            const double course = end.courseRad();
            walk.piece(std::max(speed, 0.0) * kAheadS, speed, false, index, [&](double x, double& lat, double& lon, double& h) {
                geo::offsetLatLon(lat0, lon0, end.p[0] + x * std::cos(course), end.p[1] + x * std::sin(course), lat, lon);
                h = altitude;
            });
        }
        return walk.hit;
    }

    if (const auto* c = std::get_if<HsaCommand>(&setpoint)) { // no end: its line ahead, level at its altitude, for a minute
        const bool above = aboveGround(c->altitudeReference);
        const double altitude = isHold(c->altitudeM) ? (above ? state.altitudeAglM : state.altitudeMslM) : c->altitudeM;
        const double course = !isHold(c->courseRad)    ? c->courseRad
                              : !isHold(c->headingRad) ? c->headingRad
                                                       : std::atan2(state.velocityNedMs[1], state.velocityNedMs[0]);
        double speed = isHold(c->speed) ? std::hypot(state.velocityNedMs[0], state.velocityNedMs[1])
                                        : route::plannedSpeed(c->speed, c->speedReference, msl(altitude, above, state.latitudeRad, state.longitudeRad));
        if (!(speed > 0.0)) speed = 0.0;
        spacing(speed * kAheadS);
        walk.piece(speed * kAheadS, speed, above, -1, [&](double x, double& lat, double& lon, double& h) {
            ahead(state.latitudeRad, state.longitudeRad, course, x, lat, lon);
            h = altitude;
        });
        return walk.hit;
    }
    return {};
}

void CapabilityHost::checkTerrain(const Command& setpoint, const sim::VehicleState& state, CheckLog& log) const noexcept {
    if (!sessionView_ || log.range == RangePolicy::None) return;
    if (!std::holds_alternative<RouteCommand>(setpoint) && !std::holds_alternative<PatternCommand>(setpoint) &&
        !std::holds_alternative<CurveCommand>(setpoint) && !std::holds_alternative<HsaCommand>(setpoint))
        return;
    const CommandDetails::Terrain hit = terrain(setpoint, state);
    if (!hit.hit) return;
    if (log.details) log.details->terrain = hit;
    log.find(Reason::TerrainConflict, hit.index, Constraint::None);
}

} // namespace fsim::control
