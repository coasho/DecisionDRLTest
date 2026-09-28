// Where a commanded path meets the terrain (docs/flight-autonomy.md, 4.19; A-GRA's VIOLATION_TERRAIN and its
// TerrainConstraint): the first place along a route, a pattern, a curve or an hsa's line ahead that is below the
// ground. Checked at a NEW or an UPDATE, never stepped: in a file of its own.
#include "control/CapabilityHost.h"

#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/Altimeter.h"
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
bool barometric(double reference) noexcept { return reference == static_cast<double>(AltitudeReference::Barometric); }

/// A leg's point `alongM` from its start, on past its end too (Route.h's Leg: a great circle, a rhumb line, or an arc).
void onLegAt(const route::Leg& leg, double alongM, double& lat, double& lon) noexcept {
    if (leg.arcRadiusM > 0.0) { // round its centre from a (4.30)
        const double bearing = leg.arcEntryBearingRad + (leg.arcAngleRad >= 0.0 ? 1.0 : -1.0) * alongM / leg.arcRadiusM;
        geo::offsetLatLon(leg.latA, leg.lonA, leg.arcCentreNorthM + leg.arcRadiusM * std::cos(bearing), leg.arcCentreEastM + leg.arcRadiusM * std::sin(bearing),
                          lat, lon);
        return;
    }
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

/// A route leg's altitude in its reference, `x` metres from its start, as the route flies it: straight to its
/// point, or at its climb rate and then level; on past its end, on to the point's altitude at the rate.
struct Profile {
    double from = 0.0, to = 0.0;
    double rateMs = kNaN; ///< its climb rate; NaN: along its gradient
    double speedMs = 0.0, lengthM = 0.0;
    bool late = false;    ///< its change at its rate made to end at its point (an efficient climb's, as late as it goes: 4.32)
    /// Its states' altitudes, where its first lap flies through them (4.34): the plan's for point `point`, as `height` reads each.
    const route::Plan* states = nullptr;
    std::uint32_t point = 0;
    double (*height)(double altitudeM, const Altimeter& altimeter) = nullptr;
    const Altimeter* altimeter = nullptr;
    double at(double x) const noexcept {
        if (states) return through(x);
        if (rateMs > 0.0 && speedMs > 0.0) {
            if (late) x -= std::max(lengthM - std::abs(to - from) / rateMs * speedMs, 0.0);
            if (late && x < 0.0) return from;
            return from + std::copysign(std::min(rateMs * x / speedMs, std::abs(to - from)), to - from);
        }
        return lengthM > 1.0 ? from + (to - from) * std::clamp(x / lengthM, 0.0, 1.0) : to;
    }
    /// How far past its point it is still climbing or descending to it.
    double pastM() const noexcept { return rateMs > 0.0 && speedMs > 0.0 && !states ? std::max(std::abs(to - from) / rateMs * speedMs - lengthM, 0.0) : 0.0; }
    /// Straight from its start through each state's altitude, at its place along the leg, to its point.
    double through(double x) const noexcept {
        double x0 = 0.0, h0 = from, x1 = lengthM, h1 = to;
        for (std::uint32_t j = 0; j < states->stateCount; ++j) {
            const RouteState& s = states->states[j];
            if (s.point != point || isHold(s.altitudeM)) continue;
            const double along = states->stateAlongM[j], h = height(s.altitudeM, *altimeter);
            if (along <= x && along >= x0) x0 = along, h0 = h;
            else if (along > x && along < x1) x1 = along, h1 = h;
        }
        if (x >= lengthM) return to;
        return x1 - x0 > 1e-6 ? h0 + (h1 - h0) * std::clamp((x - x0) / (x1 - x0), 0.0, 1.0) : h1;
    }
};

double asGiven(double altitudeM, const Altimeter&) { return altitudeM; }
double onIsobar(double altitudeM, const Altimeter& altimeter) { return barometricMslM(altimeter, altitudeM); }

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
    /// A pattern's way in and its lap from where it is joined, level at `altitude`, then its way out `awayM` long.
    bool pattern(const route::Pattern& p, double altitude, double speedMs, bool above, std::int16_t i, double awayM) {
        auto level = [altitude](double) { return altitude; };
        auto line = [&](const route::Line& l) {
            return piece(l.lengthM, speedMs, above, i, [&](double x, double& lat, double& lon, double& h) {
                geo::offsetLatLon(p.lat0, p.lon0, l.northM + x * std::cos(l.courseRad), l.eastM + x * std::sin(l.courseRad), lat, lon);
                h = altitude;
            });
        };
        auto fly = [&](const route::Pattern::Piece& q) {
            return q.arc ? arc(p.lat0, p.lon0, q.turn.centreNorthM, q.turn.centreEastM, q.turn.radiusM, q.turn.entryBearingRad, q.turn.angleRad,
                               q.turn.radiusM * q.sweepRad, speedMs, above, i, level)
                         : line(q.line);
        };
        for (std::uint32_t k = 0; k < p.entryCount; ++k)
            if (fly(p.entry[k])) return true;
        for (std::uint32_t k = 0, j = p.first; k < p.count; ++k, j = p.next(j))
            if (fly(p.pieces[j])) return true;
        if (!(awayM > 0.0)) return false;
        route::Line away = p.away;
        away.lengthM = awayM;
        return line(away);
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
    // a climb optimisation's change (4.32), walked as the lowest it could fly: a climb from the start at the least rate the
    // tables give through it, an efficient climb at the end at the most (its latest start); a best rate's descent from the
    // start at the most (0.1 m/s at the least: a change it can barely make). An efficient descent is along its gradient.
    auto climbProfile = [&](Profile& f, bool efficient, double fromMsl, double toMsl, double fuelKg) {
        const bool up = toMsl > fromMsl;
        double least = route::climbRateMs(config_->tables, performance_, hovers, up, fromMsl, f.speedMs, fuelKg), most = least;
        auto see = [&](double h) {
            const double r = route::climbRateMs(config_->tables, performance_, hovers, up, h, f.speedMs, fuelKg);
            least = std::min(least, r), most = std::max(most, r);
        };
        see(toMsl);
        if (config_->tables)
            for (const double h : config_->tables->altitudeM)
                if (h > std::min(fromMsl, toMsl) && h < std::max(fromMsl, toMsl)) see(h);
        f.rateMs = std::max(up && !efficient ? least : most, 0.1);
        f.late = up && efficient;
    };

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
            f.to = isHold(w.altitudeM)                 ? f.from
                   : barometric(w.altitudeReference) ? barometricMslM(config_->altimeter, w.altitudeM) // (on its isobar: 4.29)
                                                     : w.altitudeM;
            f.rateMs = isHold(w.climbRateMs) ? kNaN : w.climbRateMs;
            f.speedMs = route::plannedSpeed(w.speed, w.speedReference, msl(f.to, above, w.latitudeRad, w.longitudeRad));
            const bool efficient = w.climbOptimization == static_cast<double>(ClimbOptimization::ExtendedRange);
            if (!isHold(w.climbOptimization) && f.to != f.from && !(efficient && f.to < f.from)) { // (4.32: the lowest it could fly)
                const double toMsl = msl(f.to, above, w.latitudeRad, w.longitudeRad), fromHere = above ? msl(f.from, true, l.latA, l.lonA) : f.from;
                climbProfile(f, efficient, fromHere, toMsl, state.fuelKg);
            }
            if (firstLap && p.stateAltitudes(i)) { // (through its states: 4.34)
                f.states = &p, f.point = i, f.altimeter = &config_->altimeter;
                f.height = barometric(w.altitudeReference) ? onIsobar : asGiven;
            }
            // (a loiter point's leg, to where its loiter is joined, its altitude there: 4.31)
            const RouteLoiter* loiter = route::loiterPoint(w) ? p.loiterAt(i) : nullptr;
            const double joinM = loiter ? route::loiterJoinM(loiter->pattern, loiter->shape) : 0.0;
            f.lengthM = loiter ? std::max(l.lengthM - joinM, 0.0) : l.lengthM;
            const route::Turn* before = p.turnBefore(i, firstLap);
            const route::Turn& turn = p.turn(i, firstLap);
            const bool turns = (i + 1 < p.count || p.repeat) && turn.radiusM > 0.0;
            const double start = before ? before->leadM : 0.0;
            const double end = loiter ? std::max(start, l.lengthM - joinM) : l.lengthM - (turns ? turn.leadM : 0.0);
            const double arcM = turns ? turn.radiusM * std::abs(turn.angleRad) : 0.0;
            const auto index = static_cast<std::int16_t>(std::min<std::uint32_t>(p.named(i), 0x7FFF)); // (as given: 4.36)
            if (walk.piece(end - start, f.speedMs, above, index, [&](double x, double& lat, double& lon, double& h) {
                    onLegAt(l, start + x, lat, lon);
                    h = f.at(start + x);
                }))
                return true;
            if (loiter) { // its loiter from where it is joined: its way in and a lap, at its point's altitude
                double lat = 0.0, lon = 0.0;
                onLegAt(l, end, lat, lon);
                PatternShape shape = loiter->shape;
                route::loiterEntry(loiter->pattern, lat, lon, shape);
                route::Pattern pattern;
                route::planPattern(pattern, loiter->pattern, lat, lon, shape,
                                   loiter->shape.directionReference == static_cast<double>(DirectionReference::MagneticNorth) ? yearNow() : 2025.0,
                                   route::onLeg(l, lat, lon).courseRad);
                const double speed = route::plannedSpeed(loiter->pattern.speed, loiter->pattern.speedReference, msl(f.to, above, lat, lon));
                if (walk.pattern(pattern, f.to, speed, above, index, 0.0)) return true;
                fromMsl = msl(f.to, above, w.latitudeRad, w.longitudeRad);
                last = f;
                return false;
            }
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
            for (std::uint32_t i = p.loop; i < p.count; ++i) // (a lap on, from where it repeats from: 4.36)
                if (leg(i, false)) return walk.hit;
            return walk.hit;
        }
        const Waypoint& w = p.points[p.last()];
        if (route::loiterPoint(w)) return walk.hit; // (it ends in its loiter, walked)
        const bool above = aboveGround(w.altitudeReference);
        const auto index = static_cast<std::int16_t>(std::min<std::uint32_t>(p.named(p.last()), 0x7FFF));
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

    if (const auto* c = std::get_if<PatternCommand>(&setpoint)) {
        // its way in, its lap from where it is joined; its way out, where it leaves, for a minute
        route::Pattern p;
        const PatternShape& shape = patternShape_; // (as prepare() completed it)
        route::planPattern(p, *c, state.latitudeRad, state.longitudeRad, shape,
                           shape.directionReference == static_cast<double>(DirectionReference::MagneticNorth) ? yearNow() : 2025.0, route::trackOf(state));
        const bool above = aboveGround(c->altitudeReference);
        const double altitude = barometric(c->altitudeReference) ? barometricMslM(config_->altimeter, c->altitudeM) : c->altitudeM; // (its isobar)
        const double speed = route::plannedSpeed(c->speed, c->speedReference, msl(altitude, above, c->latitudeRad, c->longitudeRad));
        const bool leaves = p.exit >= 0 && (!isHold(c->durationS) || !isHold(shape.orbits));
        const double awayM = leaves ? std::max(speed, 0.0) * kAheadS : 0.0;
        spacing(p.entryM() + p.lapM() + awayM);
        walk.pattern(p, altitude, speed, above, -1, awayM);
        return walk.hit;
    }

    if (const auto* c = std::get_if<CurveCommand>(&setpoint); c && curvePlan_ && curvePlan_->count > 0) {
        // along its segments at the speed it flies them, then on along its last course or round its end - from the
        // command's reference (the check filled it in: the scratch plan has its segments alone)
        const route::Curve& k = *curvePlan_;
        const double lat0 = isHold(c->latitudeRad) ? state.latitudeRad : c->latitudeRad;
        const double lon0 = isHold(c->longitudeRad) ? state.longitudeRad : c->longitudeRad;
        const double alt0 = isHold(c->altitudeM) ? state.altitudeMslM : c->altitudeM;
        // its altitude as its reference reads it (4.27): over the ground under it, or on its isobar; absolute ones made
        // down from its reference as placed (the plan's)
        const bool above = aboveGround(c->altitudeReference);
        auto height = [&](double down) {
            const double h = alt0 - down;
            return barometric(c->altitudeReference) ? barometricMslM(config_->altimeter, h) : h;
        };
        const double length = k.lengthM();
        double speed = hovers ? std::hypot(state.velocityNedMs[0], state.velocityNedMs[1]) : state.airspeedTrueMs;
        if (!isHold(c->speedMaxMs)) speed = std::min(speed, c->speedMaxMs);
        if (!isHold(c->speedMinMs)) speed = std::max(speed, c->speedMinMs);
        if (!isHold(c->durationS) && c->durationS > 0.0) speed = length / c->durationS;
        spacing(length);
        for (std::uint32_t i = 0; i < k.count; ++i) // (each segment sampled 32 times at least: its height is not straight)
            if (walk.piece(k.startM[i + 1] - k.startM[i], speed, above, static_cast<std::int16_t>(i),
                           [&](double x, double& lat, double& lon, double& h) {
                               std::uint32_t j = 0;
                               double t = 0.0;
                               k.find(k.startM[i] + x, j, t);
                               const route::CurvePoint cp = k.point(j, t);
                               if (k.plain()) geo::offsetLatLon(lat0, lon0, cp.p[0], cp.p[1], lat, lon);
                               else k.fromPlane(cp.p[0], cp.p[1], lat, lon);
                               h = height(cp.p[2]);
                           },
                           route::Curve::kSamples))
                return walk.hit;
        const route::CurvePoint end = k.point(k.count - 1, 1.0);
        const double altitude = height(end.p[2]);
        const auto index = static_cast<std::int16_t>(k.count - 1);
        // (past its end, laid out from its end on the Earth where its plane is not plain: 4.27)
        double endLat = lat0, endLon = lon0, endNorth = end.p[0], endEast = end.p[1];
        if (!k.plain()) k.fromPlane(end.p[0], end.p[1], endLat, endLon), endNorth = endEast = 0.0;
        if (!isHold(c->end) && c->end == static_cast<double>(EndBehavior::Loiter)) { // each circles its end (4.28)
            const double radius = performance_.turnRadiusM(speed);
            walk.arc(endLat, endLon, endNorth, endEast, radius, 0.0, 1.0, 2.0 * kPi * radius, speed, above, index, [&](double) { return altitude; });
        } else {
            const double course = k.plain() ? end.courseRad() : geo::wrapPi(end.courseRad() + k.psi);
            walk.piece(std::max(speed, 0.0) * kAheadS, speed, above, index, [&](double x, double& lat, double& lon, double& h) {
                geo::offsetLatLon(endLat, endLon, endNorth + x * std::cos(course), endEast + x * std::sin(course), lat, lon);
                h = altitude;
            });
        }
        return walk.hit;
    }

    if (const auto* c = std::get_if<HsaCommand>(&setpoint)) { // no end: its line ahead, level at its altitude, for a minute
        const bool above = aboveGround(c->altitudeReference);
        const double altitude = isHold(c->altitudeM)               ? (above ? state.altitudeAglM : state.altitudeMslM)
                                : barometric(c->altitudeReference) ? barometricMslM(config_->altimeter, c->altitudeM) // (its isobar)
                                                                   : c->altitudeM;
        // (a magnetic direction turned true by the declination where it is: 4.22)
        const double turned = c->directionReference == static_cast<double>(DirectionReference::MagneticNorth) ? declinationNow(state) : 0.0;
        const double course = !isHold(c->courseRad)    ? c->courseRad + turned
                              : !isHold(c->headingRad) ? c->headingRad + turned
                                                       : std::atan2(state.velocityNedMs[1], state.velocityNedMs[0]);
        double speed = isHold(c->speed) ? std::hypot(state.velocityNedMs[0], state.velocityNedMs[1])
                                        : route::plannedSpeed(c->speed, c->speedReference, msl(altitude, above, state.latitudeRad, state.longitudeRad));
        if (!(speed > 0.0)) speed = 0.0;
        spacing(speed * kAheadS);
        walk.piece(speed * kAheadS, speed, above, -1, [&](double x, double& lat, double& lon, double& h) {
            geo::destination(state.latitudeRad, state.longitudeRad, course, x, lat, lon);
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
