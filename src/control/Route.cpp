#include "control/Route.h"

#include "control/Atmosphere.h"
#include "core/Geodesy.h"
#include "fsim/GuidanceModes.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control::route {

namespace {

constexpr double kR = geo::kEarthRadiusM;
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;
constexpr double kDeg = kPi / 180.0;

void unit(double lat, double lon, double v[3]) noexcept {
    const double c = std::cos(lat);
    v[0] = c * std::cos(lon), v[1] = c * std::sin(lon), v[2] = std::sin(lat);
}
double dot(const double a[3], const double b[3]) noexcept { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
void cross(const double a[3], const double b[3], double c[3]) noexcept {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}
/// The course of `t`, a direction tangent to the sphere at the point whose latitude and longitude these are.
double courseOf(const double t[3], double sinLat, double cosLat, double sinLon, double cosLon) noexcept {
    const double east = -sinLon * t[0] + cosLon * t[1];
    const double north = -sinLat * cosLon * t[0] - sinLat * sinLon * t[1] + cosLat * t[2];
    return std::atan2(east, north);
}
double courseAt(const double n[3], double lat, double lon) noexcept {
    double p[3], t[3];
    unit(lat, lon, p);
    cross(n, p, t); // the way the circle runs there
    return courseOf(t, std::sin(lat), std::cos(lat), std::sin(lon), std::cos(lon));
}
/// The Mercator projection's stretched latitude, and back.
double stretched(double lat) noexcept { return std::log(std::tan(0.25 * kPi + 0.5 * lat)); }
double unstretched(double psi) noexcept { return 2.0 * std::atan(std::exp(psi)) - 0.5 * kPi; }
/// A rhumb line's length, from latitude a to b, spanning dPsi stretched and dLon in longitude.
double rhumbM(double latA, double latB, double dPsi, double dLon) noexcept {
    const double dLat = latB - latA;
    const double q = std::abs(dPsi) > 1e-12 ? dLat / dPsi : std::cos(latA); // east-west: the parallel's scale
    return kR * std::sqrt(dLat * dLat + q * q * dLon * dLon);
}

/// The arc's centre and start, from its radius, lead and the leg in's course.
void place(Turn& t) noexcept {
    const double side = t.angleRad >= 0.0 ? 1.0 : -1.0; // the centre to the right of a right turn
    const double en = -t.leadM * std::cos(t.inRad), ee = -t.leadM * std::sin(t.inRad); // back along the leg in
    t.centreNorthM = en - side * t.radiusM * std::sin(t.inRad);
    t.centreEastM = ee + side * t.radiusM * std::cos(t.inRad);
    t.entryBearingRad = std::atan2(ee - t.centreEastM, en - t.centreNorthM);
}

/// A turn made smaller to fit: its lead cut to `leadM`, its radius with it.
void shrink(Turn& t, double leadM, bool flag) noexcept {
    t.leadM = leadM;
    t.radiusM = leadM / std::tan(0.5 * std::abs(t.angleRad));
    place(t);
    t.shrunk = t.shrunk || flag;
}

bool aboveGround(const Waypoint& w) noexcept { return w.altitudeReference == static_cast<double>(AltitudeReference::AboveGround); }
bool overGround(const Waypoint& w) noexcept { return w.speedReference == static_cast<double>(SpeedReference::GroundSpeed); }

} // namespace

Leg makeLeg(double latA, double lonA, double latB, double lonB, bool rhumb) noexcept {
    Leg leg;
    leg.latA = latA, leg.lonA = lonA, leg.latB = latB, leg.lonB = lonB;
    leg.rhumb = rhumb;
    if (rhumb) {
        leg.psiA = stretched(latA);
        leg.dPsi = stretched(latB) - leg.psiA;
        leg.dLon = geo::wrapPi(lonB - lonA);
        leg.lengthM = rhumbM(latA, latB, leg.dPsi, leg.dLon);
        leg.courseOutRad = leg.courseInRad = std::atan2(leg.dLon, leg.dPsi);
        return leg;
    }
    double b[3], c[3];
    unit(latA, lonA, leg.a);
    unit(latB, lonB, b);
    cross(leg.a, b, c);
    const double s = std::sqrt(dot(c, c));
    leg.lengthM = kR * std::atan2(s, dot(leg.a, b));
    if (s < 1e-12) {
        // one place (within millimetres): a leg north, which is passed at once
        const double north[3] = {-std::sin(latA) * std::cos(lonA), -std::sin(latA) * std::sin(lonA), std::cos(latA)};
        cross(leg.a, north, leg.n);
        return leg;
    }
    for (int k = 0; k < 3; ++k) leg.n[k] = c[k] / s;
    leg.courseOutRad = courseAt(leg.n, latA, lonA);
    leg.courseInRad = courseAt(leg.n, latB, lonB);
    return leg;
}

Fix onLeg(const Leg& leg, double lat, double lon) noexcept {
    Fix f;
    const double sinLat = std::sin(lat), cosLat = std::cos(lat);
    if (leg.rhumb) {
        // straight in the Mercator projection, whose scale here is the parallel's
        const double x = geo::wrapPi(lon - leg.lonA), y = stretched(lat) - leg.psiA; // east, north (stretched)
        const double m = std::hypot(leg.dLon, leg.dPsi);
        f.courseRad = leg.courseOutRad;
        if (m < 1e-15) return f;
        const double ue = leg.dLon / m, un = leg.dPsi / m;
        f.crossTrackM = (x * un - y * ue) * kR * cosLat;
        const double fraction = (x * ue + y * un) / m;
        const double latF = unstretched(leg.psiA + fraction * leg.dPsi); // the foot of the perpendicular
        f.alongM = std::copysign(rhumbM(leg.latA, latF, fraction * leg.dPsi, fraction * leg.dLon), fraction);
        return f;
    }
    const double sinLon = std::sin(lon), cosLon = std::cos(lon);
    const double p[3] = {cosLat * cosLon, cosLat * sinLon, sinLat};
    const double side = dot(p, leg.n); // + left of the circle
    f.crossTrackM = -kR * std::asin(std::clamp(side, -1.0, 1.0));
    const double q[3] = {p[0] - side * leg.n[0], p[1] - side * leg.n[1], p[2] - side * leg.n[2]}; // onto its plane
    double c[3], t[3];
    cross(leg.a, q, c);
    f.alongM = kR * std::atan2(dot(c, leg.n), dot(leg.a, q));
    cross(leg.n, q, t);
    f.courseRad = courseOf(t, sinLat, cosLat, sinLon, cosLon);
    return f;
}

Turn makeTurn(double inRad, double outRad, double radiusM) noexcept {
    Turn t;
    t.angleRad = geo::wrapPi(outRad - inRad);
    t.inRad = inRad;
    const double a = std::abs(t.angleRad);
    if (!(radiusM > 0.0) || a < kDeg || a > 150.0 * kDeg) return t; // straight on, or too sharp: flown over
    t.radiusM = radiusM;
    t.leadM = radiusM * std::tan(0.5 * a);
    place(t);
    return t;
}

Fix onArc(const Turn& turn, double lat0, double lon0, double lat, double lon) noexcept {
    Fix f;
    double north, east;
    geo::localNorthEastM(lat0, lon0, lat, lon, north, east);
    const double dn = north - turn.centreNorthM, de = east - turn.centreEastM;
    const double bearing = std::atan2(de, dn); // from the centre
    const double side = turn.angleRad >= 0.0 ? 1.0 : -1.0;
    f.courseRad = geo::wrapPi(bearing + side * 0.5 * kPi);        // clockwise round the centre for a right turn
    f.crossTrackM = side * (turn.radiusM - std::hypot(dn, de));  // inside a right turn is right of it
    f.alongM = side * geo::wrapPi(bearing - turn.entryBearingRad) * turn.radiusM;
    f.curvature = turn.radiusM > 0.0 ? side / turn.radiusM : 0.0;
    return f;
}

double plannedSpeed(double speed, double reference, double altitudeMslM) noexcept {
    if (isHold(speed)) return speed;
    switch (static_cast<SpeedReference>(static_cast<int>(orHold(reference, 0.0)))) {
    case SpeedReference::CalibratedAirspeed: return isa::trueFromCalibrated(speed, altitudeMslM);
    case SpeedReference::Mach: return speed * isa::speedOfSound(altitudeMslM);
    default: return speed;
    }
}

namespace {

double known(double v, double fallback) noexcept { return std::isnan(v) ? fallback : v; }

double lateralAcceleration(const Performance& f) noexcept { return 0.8 * known(f.maxAccelerationMs2, kG * std::tan(0.35)); }
double braking(const Performance& f) noexcept { return 0.8 * known(f.maxDecelerationMs2, known(f.maxAccelerationMs2, kG * std::tan(0.35))); }

} // namespace

double lateralLimit(const Performance& performance, double radiusM) noexcept { return std::sqrt(lateralAcceleration(performance) * radiusM); }

double brakingLimit(const Performance& performance, double speedAfterMs, double toGoM) noexcept {
    return std::sqrt(speedAfterMs * speedAfterMs + 2.0 * braking(performance) * std::max(toGoM, 0.0));
}

double verticalSpeedTo(double altitudeMslM, double feedforward, const sim::VehicleState& s, const Performance& f, bool hovers) noexcept {
    const double gain = known(f.altitudeGainPerS, hovers ? 0.5 : 0.25);
    const double climb = known(f.maxClimbMs, hovers ? 3.0 : 6.0), descent = known(f.maxDescentMs, climb);
    return std::clamp(feedforward + gain * (altitudeMslM - s.altitudeMslM), -descent, climb);
}

VelocityCommand follow(const ControlContext& ctx, const Performance& perf, const WindEstimate& wind, bool hovers, const Fix& fix, const Ahead& ahead,
                       const Steer& steer, Trims& trims, double& course, double& heading) noexcept {
    const auto& s = ctx.sensed;
    const double e = fix.crossTrackM;
    const double tn = std::cos(fix.courseRad), te = std::sin(fix.courseRad);
    const double windAlong = wind.northMs * tn + wind.eastMs * te, windAcross = -wind.northMs * te + wind.eastMs * tn;
    auto curvatureAhead = [&](double preview) {
        if (ahead.curvatureAt) return ahead.curvatureAt(ahead.path, preview);
        return ahead.toChangeM < preview ? ahead.curvature : fix.curvature;
    };

    if (hovers) {
        // the path speed over the ground: the segment's, or what its airspeed makes along the path in the wind
        double v = steer.speed;
        if (steer.reference != SpeedReference::GroundSpeed) {
            const double tas = trueAirspeedOf(steer.speed, steer.reference, s);
            const double crab = std::asin(std::clamp(-windAcross / std::max(tas, 0.5), -0.9, 0.9));
            v = std::max(tas * std::cos(crab) + windAlong, 0.0);
        }
        v = std::min(v, steer.speedLimitMs);
        // Line of sight to the path a few velocity-loop time constants ahead,
        // turned ahead of a curve (the curve there) by the loop's lag: the
        // velocity error its proportional term needs for the acceleration the
        // curve asks, less what its own integral has taken up. That integral
        // is modelled as the loop runs it (Rotor.cpp: a quarter of the
        // bandwidth squared, its gain falling off beyond the error a quarter
        // of the tilt answers), so round an orbit the lead fades, and as a
        // turn ends it reverses. An integral on the course error (its zero at
        // a quarter of the bandwidth) takes out the rest.
        const double bandwidth = perf.courseBandwidthRadS(0.0);
        const double lookahead = std::max(3.0 * v / bandwidth, 3.0);
        const double wanted = fix.courseRad - std::atan(e / lookahead); // the track that closes on the path
        const double most = known(perf.maxAccelerationMs2, kG * std::tan(0.35));
        const double band = 0.25 * most / bandwidth, lag = (v * v * curvatureAhead(v / bandwidth) - trims.taken) / bandwidth;
        trims.taken = std::clamp(trims.taken + 0.25 * bandwidth * bandwidth * band * band / (band * band + lag * lag) * lag * ctx.dt, -0.5 * most, 0.5 * most);
        const double ground = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
        if (ground > 1.0 && v > 1.0) {
            const double error = geo::wrapPi(wanted - std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]));
            if (std::abs(error) < 0.3) trims.courseRadS = std::clamp(trims.courseRadS + 0.25 * bandwidth * bandwidth * error * ctx.dt, -0.3, 0.3);
        }
        course = geo::wrapPi(wanted + std::clamp(lag / std::max(v, 0.5), -0.8, 0.8) + trims.courseRadS);
        heading = course;
        return VelocityCommand{kHold, steer.verticalSpeedMs, course, kHold, v * std::cos(course), v * std::sin(course)};
    }

    // a wing: the airspeed, and the turn rate that holds the path
    const double tasNow = std::max(s.airspeedTrueMs, 10.0);
    double tas;
    if (steer.reference == SpeedReference::GroundSpeed) {
        // the airspeed that makes the ground speed along the path, and a slow trim on what the wind estimate misses
        const double crab = std::asin(std::clamp(-windAcross / tasNow, -0.8, 0.8));
        tas = (steer.speed - windAlong) / std::max(std::cos(crab), 0.3);
        const double error = steer.speed - (s.velocityNedMs[0] * tn + s.velocityNedMs[1] * te);
        if (std::abs(error) < 5.0) trims.speedMs = std::clamp(trims.speedMs + 0.05 * error * ctx.dt, -5.0, 5.0);
        tas += trims.speedMs;
    } else {
        trims.speedMs = 0.0;
        tas = trueAirspeedOf(steer.speed, steer.reference, s);
    }
    const double vg = std::max(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]), 5.0);
    const double bandwidth = perf.courseBandwidthRadS(tasNow);
    const double lookahead = std::max(3.0 * vg / bandwidth, 50.0);
    // the turn anticipated: its curvature fed forward as long before it as the
    // roll into it lags - half the bank at the loop's rate, and the roll's own
    // time constant (a fifth of the heading loop's, or faster)
    double curvature = fix.curvature;
    if (ahead.turnRadiusM > 0.0) {
        const double bank = std::atan(vg * vg / (kG * ahead.turnRadiusM));
        curvature = curvatureAhead(vg * (0.5 * bank / std::max(known(perf.bankRateRadS, 0.35), 0.05) + std::clamp(0.2 / bandwidth, 0.3, 1.5)));
    }
    // the course to the path, and the turn rate that flies it: proportional, and an
    // integral (its zero at a quarter of the bandwidth) on what the turn-rate loop
    // leaves - an aircraft that needs a little bank to fly straight - once near it
    course = geo::wrapPi(fix.courseRad - std::atan(e / lookahead));
    heading = kHold;
    const double error = geo::wrapPi(course - std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]));
    if (std::abs(error) < 0.15) trims.courseRadS = std::clamp(trims.courseRadS + 0.25 * bandwidth * bandwidth * error * ctx.dt, -0.05, 0.05);
    // that is the track's rate; the velocity loop banks for the heading's, and in the wind
    // the track turns at the heading's rate times Va cos(crab) / Vg: turned back into it
    const double gn = s.velocityNedMs[0], ge = s.velocityNedMs[1], g2 = gn * gn + ge * ge;
    const double dot = gn * (gn - wind.northMs) + ge * (ge - wind.eastMs); // (ground . air velocity)
    const double toHeading = g2 > 25.0 && dot > 0.25 * g2 ? std::clamp(g2 / dot, 0.5, 2.0) : 1.0;
    return VelocityCommand{std::max(tas, 0.0), steer.verticalSpeedMs, kHold, toHeading * (curvature * vg + bandwidth * error) + trims.courseRadS, kHold, kHold};
}

Fix onLine(const Line& line, double lat0, double lon0, double lat, double lon) noexcept {
    Fix f;
    double north, east;
    geo::localNorthEastM(lat0, lon0, lat, lon, north, east);
    const double dn = north - line.northM, de = east - line.eastM;
    const double c = std::cos(line.courseRad), s = std::sin(line.courseRad);
    f.alongM = dn * c + de * s;
    f.crossTrackM = -dn * s + de * c; // + right: the course's right normal is (-sin, cos)
    f.courseRad = line.courseRad;
    return f;
}

double Pattern::lapM() const noexcept {
    double m = 0.0;
    for (std::uint32_t i = 0; i < count; ++i) m += pieceM(i);
    return m;
}

void planPattern(Pattern& p, const PatternCommand& c, double lat, double lon) noexcept {
    p.kind = static_cast<PatternKind>(static_cast<int>(orHold(c.pattern, 0.0)));
    p.lat0 = c.latitudeRad, p.lon0 = c.longitudeRad;
    p.radiusM = c.radiusM;
    p.entry = Line{};
    p.first = 0;
    const double side = orHold(c.clockwise, 1.0) == 0.0 ? -1.0 : 1.0; // +1: right turns, clockwise seen from above
    const double chi = orHold(c.courseRad, 0.0), r = c.radiusM, leg = orHold(c.legM, 0.0);
    const double un = std::cos(chi), ue = std::sin(chi); // along the course
    const double rn = -ue, re = un;                      // its right
    double north, east;
    geo::localNorthEastM(p.lat0, p.lon0, lat, lon, north, east); // where the aircraft is
    auto arc = [&](Pattern::Piece& piece, double cn, double ce, double way, double startBearing, double sweep) {
        piece = Pattern::Piece{};
        piece.arc = true;
        piece.turn.radiusM = r;
        piece.turn.angleRad = way; // (its sign: the way round)
        piece.turn.centreNorthM = cn, piece.turn.centreEastM = ce;
        piece.turn.entryBearingRad = startBearing;
        piece.sweepRad = sweep;
    };
    auto line = [](Pattern::Piece& piece, double n, double e, double course, double length) {
        piece = Pattern::Piece{};
        piece.line = Line{n, e, course, length};
    };
    switch (p.kind) {
    case PatternKind::FigureEight:
        // two circles meeting at the centre, their centres along the axis: the one
        // ahead flown the pattern's way round from the centre, then the other the other way
        p.count = 2;
        arc(p.pieces[0], r * un, r * ue, side, geo::wrapPi(chi + kPi), 2.0 * kPi);
        arc(p.pieces[1], -r * un, -r * ue, -side, chi, 2.0 * kPi);
        break;
    case PatternKind::Racetrack:
    case PatternKind::Hold: {
        // the inbound leg ends at the fix; half a circle to the outbound leg, and back
        p.count = 4;
        p.first = 1;
        line(p.pieces[0], -leg * un, -leg * ue, chi, leg);
        arc(p.pieces[1], side * r * rn, side * r * re, side, geo::wrapPi(chi - side * 0.5 * kPi), kPi);
        line(p.pieces[2], 2.0 * side * r * rn, 2.0 * side * r * re, geo::wrapPi(chi + kPi), leg);
        arc(p.pieces[3], side * r * rn - leg * un, side * r * re - leg * ue, side, geo::wrapPi(chi + side * 0.5 * kPi), kPi);
        // entered direct to the fix
        const double d = std::hypot(north, east);
        if (d > 1.0) p.entry = Line{north, east, std::atan2(-east, -north), d};
        break;
    }
    default: // an orbit: round the centre, its laps from where the aircraft is
        p.count = 1;
        arc(p.pieces[0], 0.0, 0.0, side, std::atan2(east, north), 2.0 * kPi);
        break;
    }
}

void completePattern(PatternCommand& c, const sim::VehicleState& s, const Performance& f, bool hovers, double windMs) noexcept {
    const double groundSpeed = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    const double track = groundSpeed > 1.0 ? std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]) : s.eulerRad[2];
    if (isHold(c.pattern)) c.pattern = static_cast<double>(PatternKind::Orbit);
    if (isHold(c.latitudeRad) || isHold(c.longitudeRad)) c.latitudeRad = s.latitudeRad, c.longitudeRad = s.longitudeRad;
    c.longitudeRad = geo::wrapPi(c.longitudeRad);
    if (isHold(c.altitudeReference)) c.altitudeReference = static_cast<double>(AltitudeReference::Msl);
    if (isHold(c.altitudeM)) c.altitudeM = altitudeNow(static_cast<AltitudeReference>(static_cast<int>(c.altitudeReference)), s);
    if (hovers && isHold(c.speed)) { // a rotorcraft's own speed (a hover's none) is no speed to fly a pattern at
        c.speed = std::isfinite(f.cruiseTasMs) && f.cruiseTasMs > 0.0 ? f.cruiseTasMs : 5.0;
        c.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
    } else {
        if (isHold(c.speedReference)) c.speedReference = static_cast<double>(hovers ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
        if (isHold(c.speed)) c.speed = speedNow(static_cast<SpeedReference>(static_cast<int>(c.speedReference)), s);
    }
    if (isHold(c.clockwise)) c.clockwise = 1.0;
    const auto kind = static_cast<PatternKind>(static_cast<int>(c.pattern));
    if (isHold(c.courseRad)) {
        const double away = geo::distanceM(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad);
        c.courseRad = kind == PatternKind::Hold && away > 100.0 ? geo::bearingRad(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad) : track;
    }
    c.courseRad = geo::wrapPi(c.courseRad);
    // the radius and the legs at the speed planned there (above ground: as high as the aircraft is)
    const double h = c.altitudeReference == static_cast<double>(AltitudeReference::AboveGround) ? s.altitudeMslM : c.altitudeM;
    const double v = std::max(plannedSpeed(c.speed, c.speedReference, h), 0.1);
    const double gusted = v + (hovers && c.speedReference == static_cast<double>(SpeedReference::GroundSpeed) ? 0.0 : windMs);
    if (isHold(c.radiusM))
        c.radiusM = kind == PatternKind::Hold ? std::max(gusted / (3.0 * kDeg), gusted * gusted / (kG * std::tan(25.0 * kDeg))) : f.turnRadiusM(gusted);
    if (isHold(c.legM)) c.legM = kind == PatternKind::Hold ? v * (h <= 4267.2 ? 60.0 : 90.0) : kind == PatternKind::Racetrack ? 2.0 * c.radiusM : 0.0;
}

CurvePoint evaluate(const BezierSegment& s, double t) noexcept {
    // Bernstein's basis of degree 5, and of 4 and 3 for the derivatives
    const double u = 1.0 - t;
    const double u2 = u * u, t2 = t * t;
    const double b5[6] = {u2 * u2 * u, 5.0 * u2 * u2 * t, 10.0 * u2 * u * t2, 10.0 * u2 * t2 * t, 5.0 * u * t2 * t2, t2 * t2 * t};
    const double b4[5] = {u2 * u2, 4.0 * u2 * u * t, 6.0 * u2 * t2, 4.0 * u * t2 * t, t2 * t2};
    const double b3[4] = {u2 * u, 3.0 * u2 * t, 3.0 * u * t2, t2 * t};
    CurvePoint c;
    const double* axes[3] = {s.north, s.east, s.down};
    for (int a = 0; a < 3; ++a) {
        const double* q = axes[a];
        for (int i = 0; i < 6; ++i) c.p[a] += b5[i] * q[i];
        for (int i = 0; i < 5; ++i) c.d1[a] += 5.0 * b4[i] * (q[i + 1] - q[i]);
        for (int i = 0; i < 4; ++i) c.d2[a] += 20.0 * b3[i] * (q[i + 2] - 2.0 * q[i + 1] + q[i]);
    }
    return c;
}

double CurvePoint::courseRad() const noexcept { return std::atan2(d1[1], d1[0]); }

double CurvePoint::curvature() const noexcept {
    const double v2 = d1[0] * d1[0] + d1[1] * d1[1];
    return v2 > 1e-12 ? (d1[0] * d2[1] - d1[1] * d2[0]) / (v2 * std::sqrt(v2)) : 0.0;
}

double CurvePoint::gradient() const noexcept {
    const double ground = std::hypot(d1[0], d1[1]);
    return ground > 1e-9 ? -d1[2] / ground : 0.0; // (down is positive)
}

double Curve::at(std::uint32_t i, double t) const noexcept {
    const double x = std::clamp(t, 0.0, 1.0) * kSamples;
    const int k = std::min(static_cast<int>(x), kSamples - 1);
    return startM[i] + table[i][k] + (x - k) * (table[i][k + 1] - table[i][k]);
}

void Curve::find(double s, std::uint32_t& i, double& t) const noexcept {
    s = std::clamp(s, 0.0, lengthM());
    i = 0;
    while (i + 1 < count && s >= startM[i + 1]) ++i;
    const double local = s - startM[i];
    int k = 0;
    while (k + 1 < kSamples && table[i][k + 1] < local) ++k;
    const double span = table[i][k + 1] - table[i][k];
    t = (k + (span > 1e-12 ? std::clamp((local - table[i][k]) / span, 0.0, 1.0) : 0.0)) / kSamples;
}

void Curve::measure(std::uint32_t from) noexcept {
    if (from == 0) startM[0] = 0.0;
    for (std::uint32_t i = from; i < count; ++i) {
        CurvePoint last = evaluate(segments[i], 0.0);
        table[i][0] = 0.0;
        for (int k = 1; k <= kSamples; ++k) {
            const CurvePoint c = evaluate(segments[i], static_cast<double>(k) / kSamples);
            table[i][k] = table[i][k - 1] + std::hypot(c.p[0] - last.p[0], c.p[1] - last.p[1]);
            last = c;
        }
        startM[i + 1] = startM[i] + table[i][kSamples];
    }
}

double curvatureAhead(const void* curve, double aheadM) noexcept {
    const auto& c = *static_cast<const Curve*>(curve);
    std::uint32_t i;
    double t;
    c.find(c.fromM + aheadM, i, t);
    return evaluate(c.segments[i], t).curvature();
}

double speedLimitAhead(const Performance& performance, const Curve& c, double speedMs, bool stops) noexcept {
    // sampled as far as it takes to stop, and a second more
    const double reach = speedMs * speedMs / (2.0 * braking(performance)) + speedMs;
    const double rest = c.lengthM() - c.fromM;
    constexpr int kSteps = 24;
    double limit = std::numeric_limits<double>::infinity();
    for (int k = 0; k <= kSteps; ++k) {
        const double d = reach * k / kSteps;
        if (d > rest) break;
        std::uint32_t i;
        double t;
        c.find(c.fromM + d, i, t);
        if (const double kappa = std::abs(evaluate(c.segments[i], t).curvature()); kappa > 1e-9)
            limit = std::min(limit, brakingLimit(performance, lateralLimit(performance, 1.0 / kappa), d));
    }
    if (stops) limit = std::min(limit, std::max(brakingLimit(performance, 0.0, rest), 0.5));
    return limit;
}

Fix onCurve(const Curve& c, std::uint32_t& segment, double& t, double lat, double lon) noexcept {
    double north, east;
    geo::localNorthEastM(c.lat0, c.lon0, lat, lon, north, east);
    // (B - P) . B' = 0 over the ground, from where it was: it has moved little since
    for (int k = 0; k < 8; ++k) {
        const CurvePoint p = evaluate(c.segments[segment], t);
        const double dn = p.p[0] - north, de = p.p[1] - east;
        const double g = dn * p.d1[0] + de * p.d1[1];
        const double slope = p.d1[0] * p.d1[0] + p.d1[1] * p.d1[1];
        const double gp = slope + dn * p.d2[0] + de * p.d2[1];
        const double step = std::clamp(g / (gp > 0.1 * slope && gp > 1e-9 ? gp : std::max(slope, 1e-9)), -0.2, 0.2);
        t -= step;
        if (t > 1.0) {
            if (segment + 1 < c.count) {
                ++segment, t = 0.0; // on into the next
                continue;
            }
            t = 1.0;
        } else if (t < 0.0) {
            t = 0.0;
        }
        if (std::abs(step) < 1e-7) break;
    }
    const CurvePoint p = evaluate(c.segments[segment], t);
    Fix f;
    f.courseRad = p.courseRad();
    f.crossTrackM = -(north - p.p[0]) * std::sin(f.courseRad) + (east - p.p[1]) * std::cos(f.courseRad);
    f.curvature = p.curvature();
    f.alongM = c.at(segment, t);
    return f;
}

bool tooTight(const BezierSegment& s, double limit, double& from, double& to) noexcept {
    constexpr int kSteps = 64;
    bool found = false;
    for (int k = 0; k <= kSteps; ++k) {
        const double t = static_cast<double>(k) / kSteps;
        const bool over = std::abs(evaluate(s, t).curvature()) > limit;
        if (over && !found) found = true, from = t;
        if (over) to = t;
        if (!over && found) break; // the first section only
    }
    return found;
}

double steepest(const BezierSegment& s, double& at) noexcept {
    constexpr int kSteps = 64;
    double most = 0.0;
    at = 0.0;
    for (int k = 0; k <= kSteps; ++k) {
        const double t = static_cast<double>(k) / kSteps;
        if (const double g = std::abs(evaluate(s, t).gradient()); g > most) most = g, at = t;
    }
    return most;
}

double Plan::pieceM(std::uint32_t i, bool firstLap) const noexcept {
    const Turn* before = turnBefore(i, firstLap);
    return std::max(0.0, leg(i, firstLap).lengthM - (before ? before->leadM : 0.0) - turn(i, firstLap).leadM);
}

double Plan::lapM(bool firstLap) const noexcept {
    double m = 0.0;
    for (std::uint32_t i = firstLap ? start : 0; i < count; ++i) {
        const Turn& t = turn(i, firstLap);
        m += pieceM(i, firstLap) + t.radiusM * std::abs(t.angleRad);
    }
    return m;
}

Reason complete(Waypoint* out, const Waypoint* in, std::uint32_t count, bool repeat, const sim::VehicleState& state, const Performance& performance,
                bool hovers, std::int16_t& bad) noexcept {
    auto invalid = [&bad](std::uint32_t i) {
        bad = static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF));
        return Reason::InvalidWaypoint;
    };
    if (count == 0) return invalid(0);
    if (count > Plan::kMax) return invalid(Plan::kMax);
    constexpr double inf = std::numeric_limits<double>::infinity();
    auto code = [](double v, auto kinds) { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < static_cast<double>(kinds)); };
    auto within = [](double v, double lo, double hi) { return isHold(v) || (std::isfinite(v) && v > lo && v < hi); };
    for (std::uint32_t i = 0; i < count; ++i) {
        Waypoint w = in[i]; // (in and out may be one array)
        if (!std::isfinite(w.latitudeRad) || std::abs(w.latitudeRad) > 0.5 * kPi || !std::isfinite(w.longitudeRad)) return invalid(i);
        if (isHold(w.turn) || !code(w.turn, TurnType::Count)) return invalid(i);
        if (!code(w.altitudeReference, AltitudeReference::Count) || !code(w.speedReference, SpeedReference::Count)) return invalid(i);
        if (!isHold(w.altitudeM) && !std::isfinite(w.altitudeM)) return invalid(i);
        if (!within(w.speed, 0.0, inf) || !within(w.maxBankRad, 0.0, 0.5 * kPi) || !within(w.climbRateMs, 0.0, inf)) return invalid(i);
        w.longitudeRad = geo::wrapPi(w.longitudeRad);
        if (i == 0) {
            // the aircraft's own now; a reference given alone, its value in that reference
            if (isHold(w.altitudeReference)) w.altitudeReference = static_cast<double>(AltitudeReference::Msl);
            if (isHold(w.altitudeM)) w.altitudeM = altitudeNow(static_cast<AltitudeReference>(static_cast<int>(w.altitudeReference)), state);
            if (hovers && isHold(w.speed)) { // a rotorcraft's own speed (a hover's none) is no speed to fly a route at
                w.speed = std::isfinite(performance.cruiseTasMs) && performance.cruiseTasMs > 0.0 ? performance.cruiseTasMs : 5.0;
                w.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
            } else {
                if (isHold(w.speedReference))
                    w.speedReference = static_cast<double>(hovers ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
                if (isHold(w.speed)) w.speed = speedNow(static_cast<SpeedReference>(static_cast<int>(w.speedReference)), state);
            }
        } else {
            // the previous point's; a reference alone has no value of the previous point's to take
            const Waypoint& p = out[i - 1];
            if (isHold(w.altitudeM)) {
                if (!isHold(w.altitudeReference) && w.altitudeReference != p.altitudeReference) return invalid(i);
                w.altitudeM = p.altitudeM, w.altitudeReference = p.altitudeReference;
            } else if (isHold(w.altitudeReference)) {
                w.altitudeReference = p.altitudeReference;
            }
            if (isHold(w.speed)) {
                if (!isHold(w.speedReference) && w.speedReference != p.speedReference) return invalid(i);
                w.speed = p.speed, w.speedReference = p.speedReference;
            } else if (isHold(w.speedReference)) {
                w.speedReference = p.speedReference;
            }
            if (geo::distanceM(p.latitudeRad, p.longitudeRad, w.latitudeRad, w.longitudeRad) < 1.0) return invalid(i); // the same place
        }
        if (!(w.speed > 0.0) || !std::isfinite(w.speed)) return invalid(i); // (the aircraft's own, stopped)
        out[i] = w;
    }
    if (repeat && count > 1 && geo::distanceM(out[count - 1].latitudeRad, out[count - 1].longitudeRad, out[0].latitudeRad, out[0].longitudeRad) < 1.0)
        return invalid(0); // the leg back to the first point would be none
    return Reason::None;
}

void plan(Plan& p, double lat, double lon, double altitudeMslM, double windMs, const Performance& performance, bool hovers) noexcept {
    const std::uint32_t n = p.count;
    auto point = [&p](std::uint32_t i) -> const Waypoint& { return p.points[i]; };
    for (std::uint32_t i = 0; i < n; ++i) {
        p.turns[i] = Turn{};
        p.legs[i] = i > 0 ? makeLeg(point(i - 1).latitudeRad, point(i - 1).longitudeRad, point(i).latitudeRad, point(i).longitudeRad, p.rhumb) : Leg{};
    }
    if (p.repeat && n > 1) p.legs[0] = makeLeg(point(n - 1).latitudeRad, point(n - 1).longitudeRad, point(0).latitudeRad, point(0).longitudeRad, p.rhumb);

    // A turn at point i is planned at the faster of the segments either side
    // of it, plus the wind (what the ground speed reaches downwind; a
    // rotorcraft flying over the ground has no such gust), with its bank.
    auto radius = [&](std::uint32_t i) {
        const Waypoint& a = point(i);
        const Waypoint& b = point(p.next(i));
        auto speedOf = [&](const Waypoint& w) { return plannedSpeed(w.speed, w.speedReference, aboveGround(w) ? altitudeMslM : w.altitudeM); };
        const double v = std::max(speedOf(a), speedOf(b)) + (hovers && overGround(a) && overGround(b) ? 0.0 : windMs);
        return isHold(a.maxBankRad) ? performance.turnRadiusM(v) : v * v / (kG * std::tan(a.maxBankRad));
    };
    auto flyBy = [&](std::uint32_t i) { return point(i).turn == static_cast<double>(TurnType::FlyBy); };
    for (std::uint32_t i = 0; i < n; ++i)
        if (p.leaves(i) && (i > 0 || p.repeat) && flyBy(i)) p.turns[i] = makeTurn(p.legs[i].courseInRad, p.legs[p.next(i)].courseOutRad, radius(i));

    // The entry, from where the aircraft is, and its turn at the first point
    // flown to - flown over if the aircraft is too near that point to turn
    // before it (where the aircraft is, is no fault of the route's).
    const Waypoint& first = point(p.start);
    const std::uint32_t after = p.next(p.start);
    p.entry = makeLeg(lat, lon, first.latitudeRad, first.longitudeRad, p.rhumb);
    p.entryTurn = Turn{};
    if (p.leaves(p.start) && flyBy(p.start)) {
        p.entryTurn = makeTurn(p.entry.courseInRad, p.legs[after].courseOutRad, radius(p.start));
        if (p.entryTurn.leadM > p.entry.lengthM) p.entryTurn = Turn{};
    }

    // Short legs: where the turns' leads at a leg's ends overlap, each is cut
    // to its share of it - in proportion to its lead - on both legs it
    // touches. The leg after the start is shared with the entry's turn too
    // (and, when the route repeats, with the start's own).
    auto leadBefore = [&](std::uint32_t i) {
        double lead = i > 0 || p.repeat ? p.turns[p.prev(i)].leadM : 0.0;
        if (i == after && p.leaves(p.start)) lead = p.repeat ? std::max(lead, p.entryTurn.leadM) : p.entryTurn.leadM;
        return lead;
    };
    double share[Plan::kMax];
    for (std::uint32_t i = 0; i < n; ++i) {
        share[i] = 1.0;
        if (i == 0 && !p.repeat) continue;
        const double leads = leadBefore(i) + p.turns[i].leadM;
        if (leads > p.legs[i].lengthM) share[i] = p.legs[i].lengthM / leads;
    }
    for (std::uint32_t i = 0; i < n; ++i) {
        Turn& t = p.turns[i];
        const double f = std::min(share[i], share[p.next(i)]);
        if (t.radiusM > 0.0 && f < 1.0) shrink(t, t.leadM * f, true);
    }
    if (p.entryTurn.radiusM > 0.0 && share[after] < 1.0) shrink(p.entryTurn, p.entryTurn.leadM * share[after], true);
}

} // namespace fsim::control::route
