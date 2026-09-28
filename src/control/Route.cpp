#include "control/Route.h"

#include "control/Atmosphere.h"
#include "core/Geodesy.h"
#include "fsim/GuidanceModes.h"
#include "fsim/Magnetic.h"

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

/// A turn at point i is planned at the faster of the segments either side of it, plus the wind (what the ground speed
/// reaches downwind; a rotorcraft flying over the ground has no such gust), with its bank.
double turnRadiusAt(const Plan& p, std::uint32_t i, double altitudeMslM, double windMs, const Performance& performance, bool hovers) noexcept {
    const Waypoint& a = p.points[i];
    const Waypoint& b = p.points[p.next(i)];
    auto speedOf = [&](const Waypoint& w) { return plannedSpeed(w.speed, w.speedReference, aboveGround(w) ? altitudeMslM : w.altitudeM); };
    if (!isHold(a.turnRadiusM)) return a.turnRadiusM; // (its TurnGeometry's: 4.30)
    const double v = std::max(speedOf(a), speedOf(b)) + (hovers && overGround(a) && overGround(b) ? 0.0 : windMs);
    return isHold(a.maxBankRad) ? performance.turnRadiusM(v) : v * v / (kG * std::tan(a.maxBankRad));
}

/// The leg to point i, the one before it `from`: straight, or the arc a start turn point begins - tangent there to its
/// course, left out the course the leg into it arrives on (`inRad`) - as plan() and replan() lay it (4.30).
Leg legTo(const Plan& p, std::uint32_t from, std::uint32_t i, double inRad) noexcept {
    const Waypoint& a = p.points[from];
    const Waypoint& b = p.points[i];
    if (a.turn != static_cast<double>(TurnType::StartTurn)) return makeLeg(a.latitudeRad, a.longitudeRad, b.latitudeRad, b.longitudeRad, p.rhumb);
    return makeArc(a.latitudeRad, a.longitudeRad, b.latitudeRad, b.longitudeRad, isHold(a.courseRad) ? inRad : a.courseRad);
}

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

Leg makeArc(double latA, double lonA, double latB, double lonB, double courseRad) noexcept {
    Leg leg = makeLeg(latA, lonA, latB, lonB, false);
    double n, e;
    geo::localNorthEastM(latA, lonA, latB, lonB, n, e);
    const double tn = std::cos(courseRad), te = std::sin(courseRad);
    const double alpha = std::atan2(tn * e - te * n, tn * n + te * e); // the chord from the course, + right
    const double chord = std::hypot(n, e);
    if (chord < 1e-9 || std::abs(alpha) < 1e-9) return leg; // (straight on)
    const double r = chord / (2.0 * std::abs(std::sin(alpha))), side = alpha >= 0.0 ? 1.0 : -1.0;
    leg.arcRadiusM = r;
    leg.arcAngleRad = 2.0 * alpha;
    leg.arcCentreNorthM = -side * r * te, leg.arcCentreEastM = side * r * tn; // (to the right of the course for a right turn)
    leg.arcEntryBearingRad = std::atan2(-leg.arcCentreEastM, -leg.arcCentreNorthM);
    leg.lengthM = r * std::abs(leg.arcAngleRad);
    leg.courseOutRad = geo::wrapPi(courseRad), leg.courseInRad = geo::wrapPi(courseRad + leg.arcAngleRad);
    return leg;
}

Fix onLeg(const Leg& leg, double lat, double lon) noexcept {
    if (leg.arcRadiusM > 0.0) { // (an arc from a turn point: round it, as a turn is)
        Turn arc;
        arc.radiusM = leg.arcRadiusM, arc.angleRad = leg.arcAngleRad;
        arc.centreNorthM = leg.arcCentreNorthM, arc.centreEastM = leg.arcCentreEastM, arc.entryBearingRad = leg.arcEntryBearingRad;
        return onArc(arc, leg.latA, leg.lonA, lat, lon);
    }
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

double lateralLimit(const Performance& performance, double radiusM) noexcept {
    // round the radius at 80 % of its acceleration, and at a turn rate its velocity loop follows (Performance::turnRadiusM)
    return std::min(std::sqrt(lateralAcceleration(performance) * radiusM), radiusM * performance.courseBandwidthRadS(0.0) / 3.0);
}

double brakingLimit(const Performance& performance, double speedAfterMs, double toGoM) noexcept {
    return std::sqrt(speedAfterMs * speedAfterMs + 2.0 * braking(performance) * std::max(toGoM, 0.0));
}

double verticalSpeedTo(double altitudeMslM, double feedforward, const sim::VehicleState& s, const Performance& f, bool hovers) noexcept {
    const double gain = known(f.altitudeGainPerS, hovers ? 0.5 : 0.25);
    const double climb = known(f.maxClimbMs, hovers ? 3.0 : 6.0), descent = known(f.maxDescentMs, climb);
    return std::clamp(feedforward + gain * (altitudeMslM - s.altitudeMslM), -descent, climb);
}

VelocityCommand follow(const ControlContext& ctx, const sim::VehicleState& s, const Performance& perf, const WindEstimate& wind, bool hovers,
                       const Fix& fix, const Ahead& ahead, const Steer& steer, Trims& trims, double& course, double& heading) noexcept {
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

double Pattern::entryM() const noexcept {
    double m = 0.0;
    for (std::uint32_t i = 0; i < entryCount; ++i) m += entry[i].lengthM();
    return m;
}

double Pattern::toExitM() const noexcept {
    if (exit < 0) return 0.0;
    double m = 0.0;
    for (std::uint32_t i = first; i != static_cast<std::uint32_t>(exit); i = next(i)) m += pieceM(i);
    return m;
}

void pointOn(const Pattern::Piece& piece, double m, double& north, double& east, double& courseRad) noexcept {
    if (piece.arc) {
        const Turn& t = piece.turn;
        const double way = t.angleRad >= 0.0 ? 1.0 : -1.0;
        const double bearing = t.entryBearingRad + way * m / t.radiusM;
        north = t.centreNorthM + t.radiusM * std::cos(bearing), east = t.centreEastM + t.radiusM * std::sin(bearing);
        courseRad = geo::wrapPi(bearing + way * 0.5 * kPi);
    } else {
        const Line& l = piece.line;
        north = l.northM + m * std::cos(l.courseRad), east = l.eastM + m * std::sin(l.courseRad);
        courseRad = l.courseRad;
    }
}

namespace {

/// The angle swept from bearing `from` to bearing `to` going round `way` (+1 clockwise): (0, 2 pi], a whole circle
/// where they meet.
double sweepOf(double from, double to, double way) noexcept {
    double a = way * (to - from);
    a -= 2.0 * kPi * std::floor(a / (2.0 * kPi));
    return a > 1e-9 ? a : a + 2.0 * kPi;
}

/// Where (north, east) is nearest the lap: the piece, and how far along it.
void nearestOnLap(const Pattern& p, double north, double east, std::uint32_t& piece, double& along) noexcept {
    double best = std::numeric_limits<double>::infinity();
    piece = 0, along = 0.0;
    for (std::uint32_t i = 0; i < p.count; ++i) {
        const Pattern::Piece& q = p.pieces[i];
        double m;
        if (q.arc) { // at its bearing from the centre, else the end nearer round
            const Turn& t = q.turn;
            const double way = t.angleRad >= 0.0 ? 1.0 : -1.0;
            double a = way * (std::atan2(east - t.centreEastM, north - t.centreNorthM) - t.entryBearingRad);
            a -= 2.0 * kPi * std::floor(a / (2.0 * kPi));
            if (a > q.sweepRad) a = a - q.sweepRad < 2.0 * kPi - a ? q.sweepRad : 0.0;
            m = a * t.radiusM;
        } else {
            const Line& l = q.line;
            m = std::clamp((north - l.northM) * std::cos(l.courseRad) + (east - l.eastM) * std::sin(l.courseRad), 0.0, l.lengthM);
        }
        double n, e, course;
        pointOn(q, m, n, e, course);
        if (const double d = std::hypot(north - n, east - e); d < best) best = d, piece = i, along = m;
    }
}

/// The lap's piece that begins `m` along piece i: i itself at its start, the next at its end, else the second part
/// of i cut there (the pieces after it, the first and the exit moved on by one).
std::uint32_t pieceFrom(Pattern& p, std::uint32_t i, double m) noexcept {
    if (m <= 1e-6) return i;
    if (m >= p.pieceM(i) - 1e-6 || p.count >= Pattern::kPieces) return p.next(i);
    for (std::uint32_t k = p.count; k > i + 1; --k) p.pieces[k] = p.pieces[k - 1];
    Pattern::Piece& a = p.pieces[i];
    Pattern::Piece& b = p.pieces[i + 1];
    b = a;
    if (a.arc) {
        const double swept = m / a.turn.radiusM;
        b.turn.entryBearingRad = geo::wrapPi(a.turn.entryBearingRad + (a.turn.angleRad >= 0.0 ? swept : -swept));
        b.sweepRad = a.sweepRad - swept;
        a.sweepRad = swept;
    } else {
        b.line.northM = a.line.northM + m * std::cos(a.line.courseRad), b.line.eastM = a.line.eastM + m * std::sin(a.line.courseRad);
        b.line.lengthM = a.line.lengthM - m;
        a.line.lengthM = m;
    }
    ++p.count;
    if (p.first > i) ++p.first;
    if (p.exit > static_cast<std::int32_t>(i)) ++p.exit;
    return i + 1;
}

/// A racetrack or a figure-eight by two circles (docs/flight-autonomy.md, 4.23): round the first, the leg out to the
/// second, round it, the leg back. A racetrack's circles are flown the same way and its legs touch them on the
/// outside; a figure-eight's second the other way, its legs crossing between them. Circles that overlap (only
/// without the host's checks) fly as if they touched.
void planCircles(Pattern& p, const PatternCommand& c, const PatternShape& shape, double side) noexcept {
    double n2, e2;
    geo::localNorthEastM(p.lat0, p.lon0, shape.latitude2Rad, shape.longitude2Rad, n2, e2);
    const double r1 = c.radiusM, r2 = orHold(shape.radius2M, r1);
    const double d = std::max(std::hypot(n2, e2), 1e-9);
    const double un = n2 / d, ue = e2 / d; // the axis, from the first centre to the second
    const double ln = ue, le = -un;        // its left
    const bool eight = p.kind == PatternKind::FigureEight;
    const double cosB = std::clamp((eight ? r1 + r2 : r1 - r2) / d, -1.0, 1.0), sinB = std::sqrt(1.0 - cosB * cosB);
    // from the first centre to where the leg out and the leg back touch it (a right-turning racetrack's leg out
    // passes left of the axis); the second circle's touch points along them (a figure-eight's: against them)
    const double outN = un * cosB + side * ln * sinB, outE = ue * cosB + side * le * sinB;
    const double backN = un * cosB - side * ln * sinB, backE = ue * cosB - side * le * sinB;
    const double k = eight ? -1.0 : 1.0, way2 = eight ? -side : side;
    auto arc = [](Pattern::Piece& piece, double cn, double ce, double radius, double way, double from, double to) {
        piece = Pattern::Piece{};
        piece.arc = true;
        piece.turn.radiusM = radius;
        piece.turn.angleRad = way;
        piece.turn.centreNorthM = cn, piece.turn.centreEastM = ce;
        piece.turn.entryBearingRad = from;
        piece.sweepRad = sweepOf(from, to, way);
    };
    auto line = [](Pattern::Piece& piece, double fromN, double fromE, double toN, double toE) {
        piece = Pattern::Piece{};
        piece.line = Line{fromN, fromE, std::atan2(toE - fromE, toN - fromN), std::hypot(toN - fromN, toE - fromE)};
    };
    arc(p.pieces[0], 0.0, 0.0, r1, side, std::atan2(backE, backN), std::atan2(outE, outN));
    line(p.pieces[1], r1 * outN, r1 * outE, n2 + k * r2 * outN, e2 + k * r2 * outE);
    arc(p.pieces[2], n2, e2, r2, way2, std::atan2(k * outE, k * outN), std::atan2(k * backE, k * backN));
    line(p.pieces[3], n2 + k * r2 * backN, e2 + k * r2 * backE, r1 * backN, r1 * backE);
    p.count = 4;
    p.first = 0;
    p.radiusM = std::min(r1, r2);
}

/// The course made good on `headingRad` at airspeed `v` in the wind (north, east); at a ground speed `v`
/// (`overGround`), the course whose air vector lies along the heading.
double madeGood(double headingRad, double v, bool overGround, double windNorthMs, double windEastMs) noexcept {
    const double hn = std::cos(headingRad), he = std::sin(headingRad);
    if (!overGround) return std::atan2(v * he + windEastMs, v * hn + windNorthMs);
    const double along = windNorthMs * hn + windEastMs * he;
    const double disc = along * along - (windNorthMs * windNorthMs + windEastMs * windEastMs) + v * v;
    if (disc < 0.0) return headingRad; // (a wind it cannot make that ground speed in)
    const double air = -along + std::sqrt(disc);
    return std::atan2(windEastMs + air * he, windNorthMs + air * hn);
}

/// The ground speed along `courseRad` at airspeed `v` in the wind, its crosswind held off (a tenth of `v` at least);
/// at a ground speed `v`, `v`.
double groundSpeedAlong(double courseRad, double v, bool overGround, double windNorthMs, double windEastMs) noexcept {
    if (overGround) return v;
    const double along = windNorthMs * std::cos(courseRad) + windEastMs * std::sin(courseRad);
    const double cross = -windNorthMs * std::sin(courseRad) + windEastMs * std::cos(courseRad);
    return std::max(std::sqrt(std::max(v * v - cross * cross, 0.0)) + along, 0.1 * v);
}

bool magneticOf(const PatternShape& shape) noexcept { return shape.directionReference == static_cast<double>(DirectionReference::MagneticNorth); }

/// A racetrack's or a hold's way in (docs/flight-autonomy.md, 4.24), from (north, east) where the aircraft is, on
/// `track` (kHold: toward where it goes first): its pieces into p.entry, and the piece of the lap it joins (p.first).
/// The lap as planPattern lays it: [0] the inbound leg to the fix, [1] the turn there, [2] the outbound leg, [3] the
/// turn back. Turns are of the pattern's radius, and tangent where the way in turns: a leg from the fix begins with a
/// turn over the fix, as ATC's entries do; the way onto a leg's start is the shortest of turn, straight, turn (Dubins's).
void enterHold(Pattern& p, HoldEntry entry, double north, double east, double track, double chi, double side, double r, double leg) noexcept {
    const double un = std::cos(chi), ue = std::sin(chi); // inbound
    const double hn = -side * ue, he = side * un;        // toward the holding side
    std::uint32_t n = 0;
    double atN = north, atE = east; // where the way in has come to
    auto lineTo = [&](double toN, double toE) {
        const double d = std::hypot(toN - atN, toE - atE);
        if (d > 1.0 && n < Pattern::kEntryPieces) {
            Pattern::Piece& piece = p.entry[n++];
            piece = Pattern::Piece{};
            piece.line = Line{atN, atE, std::atan2(toE - atE, toN - atN), d};
        }
        atN = toN, atE = toE;
    };
    auto arcRound = [&](double centreN, double centreE, double way, double sweep) { // from where it has come to
        if (n >= Pattern::kEntryPieces || sweep < 1e-6) return;
        Pattern::Piece& piece = p.entry[n++];
        piece = Pattern::Piece{};
        piece.arc = true;
        piece.turn.radiusM = std::hypot(atN - centreN, atE - centreE);
        piece.turn.angleRad = way;
        piece.turn.centreNorthM = centreN, piece.turn.centreEastM = centreE;
        piece.turn.entryBearingRad = std::atan2(atE - centreE, atN - centreN);
        piece.sweepRad = sweep;
        const double end = piece.turn.entryBearingRad + way * sweep;
        atN = centreN + piece.turn.radiusM * std::cos(end), atE = centreE + piece.turn.radiusM * std::sin(end);
    };
    // on `from`, a turn of radius r to `to`, the way it is shorter (none under a thousandth of a radian)
    auto turnTo = [&](double from, double to) {
        const double turn = geo::wrapPi(to - from), way = turn >= 0.0 ? 1.0 : -1.0;
        if (std::abs(turn) > 1e-3) arcRound(atN - way * r * std::sin(from), atE + way * r * std::cos(from), way, std::abs(turn));
    };
    // the shortest turn - straight - turn from where it is on `from` to (toN, toE) on `to`
    auto dubins = [&](double from, double toN, double toE, double to) {
        struct Way {
            double length = std::numeric_limits<double>::infinity(), way1 = 1.0, way2 = 1.0;
            double c1n = 0.0, c1e = 0.0, c2n = 0.0, c2e = 0.0, t1n = 0.0, t1e = 0.0, t2n = 0.0, t2e = 0.0;
        } best;
        auto sweep = [](double fromBearing, double toBearing, double way) { // [0, 2 pi)
            double a = way * (toBearing - fromBearing);
            return a - 2.0 * kPi * std::floor(a / (2.0 * kPi));
        };
        for (const double w1 : {1.0, -1.0})
            for (const double w2 : {1.0, -1.0}) {
                Way c;
                c.way1 = w1, c.way2 = w2;
                c.c1n = atN - w1 * r * std::sin(from), c.c1e = atE + w1 * r * std::cos(from); // (the turn's side of the way it goes)
                c.c2n = toN - w2 * r * std::sin(to), c.c2e = toE + w2 * r * std::cos(to);
                const double dn = c.c2n - c.c1n, de = c.c2e - c.c1e, d = std::hypot(dn, de);
                if (d < 1e-9) continue;
                const double vn = dn / d, ve = de / d;
                double nn, ne; // from the first centre to where the straight leaves it
                if (w1 == w2) {
                    nn = w1 * ve, ne = -w1 * vn; // (its outside: turned a quarter from the straight, against the way round)
                    c.t1n = c.c1n + r * nn, c.t1e = c.c1e + r * ne, c.t2n = c.c2n + r * nn, c.t2e = c.c2e + r * ne;
                } else {
                    if (d < 2.0 * r) continue;
                    const double cb = 2.0 * r / d, sb = std::sqrt(1.0 - cb * cb);
                    nn = vn * cb + w1 * ve * sb, ne = ve * cb - w1 * vn * sb; // (so that it leaves along the straight, the way round)
                    c.t1n = c.c1n + r * nn, c.t1e = c.c1e + r * ne, c.t2n = c.c2n - r * nn, c.t2e = c.c2e - r * ne;
                }
                const double straight = std::hypot(c.t2n - c.t1n, c.t2e - c.t1e);
                const double course = std::atan2(c.t2e - c.t1e, c.t2n - c.t1n);
                c.length = r * (sweep(from, course, w1) + sweep(course, to, w2)) + straight;
                if (c.length < best.length) best = c;
            }
        if (!std::isfinite(best.length)) return lineTo(toN, toE);
        const double course = std::atan2(best.t2e - best.t1e, best.t2n - best.t1n);
        arcRound(best.c1n, best.c1e, best.way1, sweep(from, course, best.way1));
        lineTo(best.t2n, best.t2e);
        arcRound(best.c2n, best.c2e, best.way2, sweep(course, to, best.way2));
        atN = toN, atE = toE; // (where it ends: the target, to rounding)
    };
    const double toFix = std::atan2(-east, -north); // its way to the fix
    const double on = isHold(track) ? toFix : track;
    bool overFix = false; // ATC's direct entry: to the fix, then round
    if (entry == HoldEntry::Anchor) { // ATC's entry for the side it comes from, by its way to the fix (left turns mirrored)
        const double a = side * geo::wrapPi(toFix - chi);
        if (a < -70.0 * kDeg) entry = HoldEntry::Parallel;
        else if (a > 110.0 * kDeg) entry = HoldEntry::Teardrop;
        else overFix = true;
    }
    if (overFix) {
        lineTo(0.0, 0.0);
        p.first = 1;
    } else {
        switch (entry) {
        case HoldEntry::Direct: { // where the lap is nearest, as an orbit is joined: no way in
            std::uint32_t i;
            double m;
            nearestOnLap(p, north, east, i, m);
            p.first = pieceFrom(p, i, m);
            break;
        }
        case HoldEntry::Inbound: // onto the inbound leg's start, on its course
            dubins(on, -leg * un, -leg * ue, chi);
            p.first = 0;
            break;
        case HoldEntry::Outbound: // onto the outbound leg's start, abeam the fix, on its course
            dubins(on, 2.0 * r * hn, 2.0 * r * he, chi + kPi);
            p.first = 2;
            break;
        case HoldEntry::Parallel: { // over the fix, turned out a leg along the inbound course, round toward the holding side, to the fix
            lineTo(0.0, 0.0);
            turnTo(toFix, chi + kPi);
            lineTo(atN - leg * un, atE - leg * ue);
            arcRound(atN + r * hn, atE + r * he, -side, kPi);
            lineTo(0.0, 0.0);
            p.first = 1;
            break;
        }
        default: { // teardrop: over the fix, turned out a leg 30 degrees into the holding side, round the pattern's way, to the fix
            lineTo(0.0, 0.0);
            const double t = chi + kPi - side * 30.0 * kDeg;
            turnTo(toFix, t);
            lineTo(atN + leg * std::cos(t), atE + leg * std::sin(t));
            arcRound(atN - side * r * std::sin(t), atE + side * r * std::cos(t), side, kPi + 30.0 * kDeg);
            lineTo(0.0, 0.0);
            p.first = 1;
            break;
        }
        }
    }
    p.entryCount = n;
}

/// The radius a hold's turn type gives at `gusted` (its speed plus the wind), 4.24.
double turnTypeRadiusM(HoldTurn type, double gusted, const Performance& f) noexcept {
    switch (type) {
    case HoldTurn::MilPower: return f.turnRadiusM(gusted);
    case HoldTurn::Relax: return std::max(gusted / (1.5 * kDeg), gusted * gusted / (kG * std::tan(15.0 * kDeg)));
    default: return std::max(gusted / (3.0 * kDeg), gusted * gusted / (kG * std::tan(25.0 * kDeg)));
    }
}

} // namespace

void planPattern(Pattern& p, const PatternCommand& c, double lat, double lon, const PatternShape& shape, double magneticYear, double trackRad) noexcept {
    p.kind = static_cast<PatternKind>(static_cast<int>(orHold(c.pattern, 0.0)));
    p.lat0 = c.latitudeRad, p.lon0 = c.longitudeRad;
    p.radiusM = c.radiusM;
    p.entryCount = 0;
    p.exit = -1;
    p.first = 0;
    const double side = orHold(c.clockwise, 1.0) == 0.0 ? -1.0 : 1.0; // +1: right turns, clockwise seen from above
    double chi = orHold(c.courseRad, 0.0);
    if (magneticOf(shape)) chi = geo::wrapPi(chi + declinationRad(c.latitudeRad, c.longitudeRad, 0.0, magneticYear));
    const double r = c.radiusM, leg = orHold(c.legM, 0.0);
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
    auto enter = [&](double toNorth, double toEast) { // a line from the aircraft
        const double d = std::hypot(toNorth - north, toEast - east);
        if (d > 1.0) line(p.entry[0], north, east, std::atan2(toEast - east, toNorth - north), d), p.entryCount = 1;
    };
    if (!isHold(shape.latitude2Rad) && !isHold(shape.longitude2Rad) && (p.kind == PatternKind::Racetrack || p.kind == PatternKind::FigureEight)) {
        planCircles(p, c, shape, side); // joined where it is nearest, as an orbit is
        std::uint32_t i;
        double m;
        nearestOnLap(p, north, east, i, m);
        p.first = pieceFrom(p, i, m);
    } else {
        switch (p.kind) {
        case PatternKind::FigureEight:
            // two circles meeting at the centre, their centres along the axis: the one
            // ahead flown the pattern's way round from the centre, then the other the other way
            p.count = 2;
            arc(p.pieces[0], r * un, r * ue, side, geo::wrapPi(chi + kPi), 2.0 * kPi);
            arc(p.pieces[1], -r * un, -r * ue, -side, chi, 2.0 * kPi);
            break;
        case PatternKind::Racetrack:
        case PatternKind::Hold:
            // the inbound leg ends at the fix; half a circle to the outbound leg, and back
            p.count = 4;
            p.first = 1;
            line(p.pieces[0], -leg * un, -leg * ue, chi, leg);
            arc(p.pieces[1], side * r * rn, side * r * re, side, geo::wrapPi(chi - side * 0.5 * kPi), kPi);
            line(p.pieces[2], 2.0 * side * r * rn, 2.0 * side * r * re, geo::wrapPi(chi + kPi), leg);
            arc(p.pieces[3], side * r * rn - leg * un, side * r * re - leg * ue, side, geo::wrapPi(chi + side * 0.5 * kPi), kPi);
            if (isHold(shape.holdEntry)) { // entered direct to the fix
                if (const double d = std::hypot(north, east); d > 1.0) line(p.entry[0], north, east, std::atan2(-east, -north), d), p.entryCount = 1;
            } else { // as its entry says (4.24)
                enterHold(p, static_cast<HoldEntry>(static_cast<int>(shape.holdEntry)), north, east, trackRad, chi, side, r, leg);
            }
            break;
        case PatternKind::Hover: // its point, and the way there
            p.count = 1;
            line(p.pieces[0], 0.0, 0.0, 0.0, 0.0);
            if (const double d = std::hypot(north, east); d > 1.0) line(p.entry[0], north, east, std::atan2(-east, -north), d), p.entryCount = 1;
            break;
        default: // an orbit: round the centre, its laps from where the aircraft is
            p.count = 1;
            arc(p.pieces[0], 0.0, 0.0, side, std::atan2(east, north), 2.0 * kPi);
            break;
        }
    }
    if (!isHold(shape.entryLatitudeRad) && !isHold(shape.entryLongitudeRad)) { // joined there, flown to directly
        double n, e, course;
        geo::localNorthEastM(p.lat0, p.lon0, shape.entryLatitudeRad, shape.entryLongitudeRad, n, e);
        std::uint32_t i;
        double m;
        nearestOnLap(p, n, e, i, m);
        p.first = pieceFrom(p, i, m);
        pointOn(p.pieces[p.first], 0.0, n, e, course);
        p.entryCount = 0;
        enter(n, e);
    }
    if (!isHold(shape.exitLatitudeRad) && !isHold(shape.exitLongitudeRad)) { // left there, on along its course
        double n, e, course;
        geo::localNorthEastM(p.lat0, p.lon0, shape.exitLatitudeRad, shape.exitLongitudeRad, n, e);
        std::uint32_t i;
        double m;
        nearestOnLap(p, n, e, i, m);
        p.exit = static_cast<std::int32_t>(pieceFrom(p, i, m));
        pointOn(p.pieces[p.exit], 0.0, n, e, course);
        p.away = Line{n, e, course, 1e7};
    }
}

void completePattern(PatternCommand& c, PatternShape& shape, const sim::VehicleState& s, const Performance& f, bool hovers, double windNorthMs,
                     double windEastMs, const Altimeter* altimeter, double magneticYear) noexcept {
    const double windMs = std::hypot(windNorthMs, windEastMs);
    const double groundSpeed = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    const double track = groundSpeed > 1.0 ? std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]) : s.eulerRad[2];
    if (isHold(c.pattern)) c.pattern = static_cast<double>(PatternKind::Orbit);
    if (isHold(c.latitudeRad) || isHold(c.longitudeRad)) c.latitudeRad = s.latitudeRad, c.longitudeRad = s.longitudeRad;
    c.longitudeRad = geo::wrapPi(c.longitudeRad);
    if (isHold(c.altitudeReference)) c.altitudeReference = static_cast<double>(AltitudeReference::Msl);
    if (isHold(c.altitudeM)) c.altitudeM = altitudeNow(static_cast<AltitudeReference>(static_cast<int>(c.altitudeReference)), s, altimeter);
    if (hovers && isHold(c.speed)) { // a rotorcraft's own speed (a hover's none) is no speed to fly a pattern at
        c.speed = std::isfinite(f.cruiseTasMs) && f.cruiseTasMs > 0.0 ? f.cruiseTasMs : 5.0;
        c.speedReference = static_cast<double>(SpeedReference::GroundSpeed);
    } else {
        if (isHold(c.speedReference)) c.speedReference = static_cast<double>(hovers ? SpeedReference::GroundSpeed : SpeedReference::TrueAirspeed);
        if (isHold(c.speed)) c.speed = speedNow(static_cast<SpeedReference>(static_cast<int>(c.speedReference)), s);
    }
    const auto kind = static_cast<PatternKind>(static_cast<int>(c.pattern));
    if (kind == PatternKind::Hover) return; // (its point, its altitude and its speed there: nothing round it - 4.25)
    if (isHold(c.clockwise)) c.clockwise = 1.0;
    // the radius and the legs at the speed planned there (above ground: as high as the aircraft is)
    const double h = c.altitudeReference == static_cast<double>(AltitudeReference::AboveGround) ? s.altitudeMslM
                                                                                                : altitudeMslOf(c.altitudeM, static_cast<AltitudeReference>(static_cast<int>(c.altitudeReference)), s, altimeter);
    const double v = std::max(plannedSpeed(c.speed, c.speedReference, h), 0.1);
    const bool overGround = c.speedReference == static_cast<double>(SpeedReference::GroundSpeed);
    const double gusted = v + (hovers && overGround ? 0.0 : windMs);
    const bool circles = !isHold(shape.latitude2Rad) && !isHold(shape.longitude2Rad); // (they give their course and legs)
    const bool magnetic = magneticOf(shape);
    const double declination = magnetic ? declinationRad(c.latitudeRad, c.longitudeRad, 0.0, magneticYear) : 0.0;
    if (!circles) {
        if (isHold(c.courseRad) && !isHold(shape.headingRad)) // the course it makes good on the heading, in its reference
            c.courseRad = madeGood(shape.headingRad + declination, v, overGround, windNorthMs, windEastMs) - declination;
        if (isHold(c.courseRad)) {
            const double away = geo::distanceM(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad);
            c.courseRad = kind == PatternKind::Hold && away > 100.0 ? geo::bearingRad(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad) : track;
            if (magnetic) c.courseRad -= declination;
        }
        c.courseRad = geo::wrapPi(c.courseRad);
    }
    if (isHold(c.radiusM) && !isHold(shape.bankRad)) c.radiusM = gusted * gusted / (kG * std::tan(shape.bankRad));
    if (isHold(c.radiusM) && !isHold(shape.turnRateRadS)) c.radiusM = gusted / shape.turnRateRadS;
    if (isHold(c.radiusM) && !isHold(shape.turnType)) c.radiusM = turnTypeRadiusM(static_cast<HoldTurn>(static_cast<int>(shape.turnType)), gusted, f);
    if (isHold(c.radiusM))
        c.radiusM = kind == PatternKind::Hold ? std::max(gusted / (3.0 * kDeg), gusted * gusted / (kG * std::tan(25.0 * kDeg))) : f.turnRadiusM(gusted);
    if (circles) {
        if (isHold(shape.radius2M)) shape.radius2M = c.radiusM;
    } else {
        if (isHold(c.legM) && !isHold(shape.legS)) c.legM = groundSpeedAlong(c.courseRad + declination, v, overGround, windNorthMs, windEastMs) * shape.legS;
        if (isHold(c.legM)) c.legM = kind == PatternKind::Hold ? v * (h <= 4267.2 ? 60.0 : 90.0) : kind == PatternKind::Racetrack ? 2.0 * c.radiusM : 0.0;
    }
}

int shapeFault(const PatternCommand& c, const PatternShape& shape) noexcept {
    const auto kind = static_cast<PatternKind>(static_cast<int>(orHold(c.pattern, 0.0)));
    const bool onFix = kind == PatternKind::Racetrack || kind == PatternKind::Hold;
    if (!isHold(shape.holdEntry) && (!onFix || shape.twoCircles() || !isHold(shape.entryLatitudeRad))) return 27;
    if (!isHold(shape.holdContext) && kind != PatternKind::Hold) return 28;
    if (isHold(shape.frame)) { // (a frame's offsets need their frame)
        const double offsets[] = {shape.frameRotation, shape.frameOffsets, shape.frameXM, shape.frameYM, shape.frameZM};
        for (int i = 0; i < 5; ++i)
            if (!isHold(offsets[i])) return 30 + i;
    }
    if (isHold(shape.latitude2Rad) && isHold(shape.longitude2Rad)) return -1;
    if (kind != PatternKind::Racetrack && kind != PatternKind::FigureEight) return 18;
    // (the circles give its course and legs: any there were given)
    if (!isHold(c.courseRad)) return 7;
    if (!isHold(c.legM)) return 8;
    if (!isHold(shape.headingRad)) return 14;
    if (!isHold(shape.legS)) return 15;
    const double r1 = c.radiusM, r2 = orHold(shape.radius2M, r1);
    if (!(r1 >= 1.0)) return 5;
    if (!(r2 >= 1.0)) return 20;
    double n2, e2;
    geo::localNorthEastM(c.latitudeRad, c.longitudeRad, shape.latitude2Rad, shape.longitude2Rad, n2, e2);
    const double d = std::hypot(n2, e2);
    if (kind == PatternKind::FigureEight ? !(d >= r1 + r2) : !(d > std::abs(r1 - r2) && d >= 1.0)) return 18;
    return -1;
}

int hoverFault(const PatternCommand& c, const PatternShape& shape) noexcept {
    const double fields[] = {c.radiusM, c.clockwise, c.courseRad, c.legM};
    for (int i = 0; i < 4; ++i)
        if (!isHold(fields[i])) return 5 + i;
    const double shaped[] = {shape.directionReference, shape.headingRad, shape.legS, shape.bankRad, shape.orbits, shape.latitude2Rad,
                             shape.longitude2Rad, shape.radius2M, shape.entryLatitudeRad, shape.entryLongitudeRad, shape.exitLatitudeRad,
                             shape.exitLongitudeRad, shape.turnRateRadS, shape.turnType, shape.holdEntry, shape.holdContext};
    for (int i = 0; i < 16; ++i)
        if (!isHold(shaped[i])) return 13 + i;
    return -1;
}

namespace {

/// A quintic Bezier's point by Bernstein's basis: its six control points on each axis.
inline CurvePoint bernstein(const double* north, const double* east, const double* down, double t) noexcept {
    // Bernstein's basis of degree 5, and of 4 and 3 for the derivatives
    const double u = 1.0 - t;
    const double u2 = u * u, t2 = t * t;
    const double b5[6] = {u2 * u2 * u, 5.0 * u2 * u2 * t, 10.0 * u2 * u * t2, 10.0 * u2 * t2 * t, 5.0 * u * t2 * t2, t2 * t2 * t};
    const double b4[5] = {u2 * u2, 4.0 * u2 * u * t, 6.0 * u2 * t2, 4.0 * u * t2 * t, t2 * t2};
    const double b3[4] = {u2 * u, 3.0 * u2 * t, 3.0 * u * t2, t2 * t};
    CurvePoint c;
    const double* axes[3] = {north, east, down};
    for (int a = 0; a < 3; ++a) {
        const double* q = axes[a];
        for (int i = 0; i < 6; ++i) c.p[a] += b5[i] * q[i];
        for (int i = 0; i < 5; ++i) c.d1[a] += 5.0 * b4[i] * (q[i + 1] - q[i]);
        for (int i = 0; i < 4; ++i) c.d2[a] += 20.0 * b3[i] * (q[i + 2] - 2.0 * q[i + 1] + q[i]);
    }
    return c;
}

} // namespace

CurvePoint evaluate(const BezierSegment& s, double t) noexcept { return bernstein(s.north, s.east, s.down, t); }

CurvePoint evaluate(const NurbsSegment& s, double t) noexcept { return s.bezier() ? bernstein(s.north, s.east, s.down, t) : rational(s, t); }

void Curve::toPlane(double lat, double lon, double& north, double& east) const noexcept {
    if (plain()) {
        geo::localNorthEastM(lat0, lon0, lat, lon, north, east);
        return;
    }
    earthToPlane(lat0, lon0, offsets, psi, lat, lon, north, east);
}

void Curve::fromPlane(double north, double east, double& lat, double& lon) const noexcept {
    if (plain()) {
        geo::offsetLatLon(lat0, lon0, north, east, lat, lon);
        return;
    }
    planeToEarth(lat0, lon0, offsets, psi, north, east, lat, lon);
}

CurvePoint Curve::point(std::uint32_t i, double t) const noexcept {
    const NurbsSegment& s = segments[i];
    return bezier[i] ? bernstein(s.north, s.east, s.down, t) : rational(s, t);
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
        bezier[i] = segments[i].bezier();
        CurvePoint last = point(i, 0.0);
        table[i][0] = 0.0;
        for (int k = 1; k <= kSamples; ++k) {
            const CurvePoint c = point(i, static_cast<double>(k) / kSamples);
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
    return c.point(i, t).curvature();
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
        if (const double kappa = std::abs(c.point(i, t).curvature()); kappa > 1e-9)
            limit = std::min(limit, brakingLimit(performance, lateralLimit(performance, 1.0 / kappa), d));
    }
    if (stops) limit = std::min(limit, std::max(brakingLimit(performance, 0.0, rest), 0.5));
    return limit;
}

Fix onCurve(const Curve& c, std::uint32_t& segment, double& t, double lat, double lon) noexcept {
    double north, east;
    c.toPlane(lat, lon, north, east);
    // (B - P) . B' = 0 over the ground, from where it was: it has moved little since
    for (int k = 0; k < 8; ++k) {
        const CurvePoint p = c.point(segment, t);
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
    const CurvePoint p = c.point(segment, t);
    Fix f;
    f.courseRad = p.courseRad();
    f.crossTrackM = -(north - p.p[0]) * std::sin(f.courseRad) + (east - p.p[1]) * std::cos(f.courseRad); // (in its plane, along its axes)
    if (c.psi != 0.0) f.courseRad = geo::wrapPi(f.courseRad + c.psi); // (its axes turned: from north)
    f.curvature = p.curvature();
    f.alongM = c.at(segment, t);
    return f;
}

bool tooTight(const Curve& c, std::uint32_t i, double limit, double& from, double& to) noexcept {
    constexpr int kSteps = 64;
    bool found = false;
    for (int k = 0; k <= kSteps; ++k) {
        const double t = static_cast<double>(k) / kSteps;
        const bool over = std::abs(c.point(i, t).curvature()) > limit;
        if (over && !found) found = true, from = t;
        if (over) to = t;
        if (!over && found) break; // the first section only
    }
    return found;
}

double steepest(const Curve& c, std::uint32_t i, double& at) noexcept {
    constexpr int kSteps = 64;
    double most = 0.0;
    at = 0.0;
    for (int k = 0; k <= kSteps; ++k) {
        const double t = static_cast<double>(k) / kSteps;
        if (const double g = std::abs(c.point(i, t).gradient()); g > most) most = g, at = t;
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
                bool hovers, std::int16_t& bad, const Altimeter* altimeter) noexcept {
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
        // its end point as A-GRA's schema gives it (4.29): a kind and a type (a waypoint's), a block, a frame's fields whole
        if (!code(w.kind, EndPointKind::Count) || !code(w.waypointType, WaypointType::Count)) return invalid(i);
        if (!isHold(w.waypointType)) {
            if (isHold(w.kind)) w.kind = static_cast<double>(EndPointKind::Waypoint);
            if (w.kind != static_cast<double>(EndPointKind::Waypoint)) return invalid(i);
        }
        if (!within(w.altitudeMinM, -inf, inf) || !within(w.altitudeMaxM, -inf, inf)) return invalid(i);
        if (!isHold(w.altitudeMinM) && !isHold(w.altitudeMaxM) && w.altitudeMaxM < w.altitudeMinM) return invalid(i);
        if (!isHold(w.frame) && !(w.frame == std::floor(w.frame) && w.frame >= 1.0 && w.frame <= 9007199254740992.0)) return invalid(i);
        if (!code(w.frameRotation, FrameRotation::Count) || !code(w.frameOffsets, FrameOffsets::Count)) return invalid(i);
        if (!within(w.frameXM, -inf, inf) || !within(w.frameYM, -inf, inf) || !within(w.frameZM, -inf, inf)) return invalid(i);
        if (isHold(w.frame) && !(isHold(w.frameRotation) && isHold(w.frameOffsets) && isHold(w.frameXM) && isHold(w.frameYM) && isHold(w.frameZM)))
            return invalid(i); // (offsets without their frame)
        // a turn point's course and radius where its type has them (4.30): a capture's course; a start's radius and course;
        // an end's course; a fly-by's radius - a start with a point after it, an end after a start; a waypoint turns none
        if (!within(w.courseRad, -inf, inf) || !within(w.turnRadiusM, 0.0, inf)) return invalid(i);
        const auto turn = static_cast<TurnType>(static_cast<int>(w.turn));
        if (noTurn(w) && turn != TurnType::FlyBy && turn != TurnType::FlyOver) return invalid(i);
        const bool coursed = turn == TurnType::CaptureOutboundCourse || turn == TurnType::StartTurn || turn == TurnType::EndTurn;
        if (!isHold(w.courseRad) && !coursed) return invalid(i);
        if (!isHold(w.turnRadiusM) && !(turn == TurnType::StartTurn || (turn == TurnType::FlyBy && !noTurn(w)))) return invalid(i);
        if (turn == TurnType::CaptureOutboundCourse && isHold(w.courseRad)) return invalid(i);
        if (turn == TurnType::StartTurn && !(i + 1 < count || repeat)) return invalid(i); // (an arc to no point)
        if (turn == TurnType::EndTurn) { // (a turn it ends: the point before begins it)
            const std::uint32_t before = i > 0 ? i - 1 : count - 1;
            if ((i == 0 && !repeat) || count < 2 || in[before].turn != static_cast<double>(TurnType::StartTurn)) return invalid(i);
        }
        const bool altitudeGiven = !isHold(w.altitudeM);
        w.longitudeRad = geo::wrapPi(w.longitudeRad);
        if (i == 0) {
            // the aircraft's own now; a reference given alone, its value in that reference
            if (isHold(w.altitudeReference)) w.altitudeReference = static_cast<double>(AltitudeReference::Msl);
            if (isHold(w.altitudeM)) w.altitudeM = altitudeNow(static_cast<AltitudeReference>(static_cast<int>(w.altitudeReference)), state, altimeter);
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
        // within its block: given, or refused; left out, held within it
        if (altitudeGiven && ((!isHold(w.altitudeMinM) && w.altitudeM < w.altitudeMinM) || (!isHold(w.altitudeMaxM) && w.altitudeM > w.altitudeMaxM)))
            return invalid(i);
        if (!isHold(w.altitudeMinM)) w.altitudeM = std::max(w.altitudeM, w.altitudeMinM);
        if (!isHold(w.altitudeMaxM)) w.altitudeM = std::min(w.altitudeM, w.altitudeMaxM);
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
    // the arcs start turn points begin (4.30), in order: each tangent to the course the leg into its start arrives on
    // (the entry's, for a route's own start point it does not come back to)
    p.arcs = false;
    for (std::uint32_t k = 0; k < n; ++k) {
        if (point(k).turn != static_cast<double>(TurnType::StartTurn) || !p.leaves(k)) continue;
        const bool first = k == p.start && !p.repeat;
        const double in = first ? makeLeg(lat, lon, point(k).latitudeRad, point(k).longitudeRad, p.rhumb).courseInRad : p.legs[k].courseInRad;
        p.legs[p.next(k)] = legTo(p, k, p.next(k), in);
        p.arcs = p.arcs || p.legs[p.next(k)].arcRadiusM > 0.0;
    }

    auto radius = [&](std::uint32_t i) { return turnRadiusAt(p, i, altitudeMslM, windMs, performance, hovers); };
    auto flyBy = [&](std::uint32_t i) { return point(i).turn == static_cast<double>(TurnType::FlyBy) && !noTurn(point(i)); }; // (a waypoint: flown over)
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

VelocityCommand hoverOver(const sim::VehicleState& s, const Performance& perf, double lat, double lon, double transit, double frameNorthMs,
                          double frameEastMs, double verticalSpeedMs) noexcept {
    double north, east; // the point, from here
    geo::localNorthEastM(s.latitudeRad, s.longitudeRad, lat, lon, north, east);
    const double distance = std::hypot(north, east);
    const double lag = std::isfinite(perf.velocityBandwidthRadS) && perf.velocityBandwidthRadS > 0.0 ? 1.0 / perf.velocityBandwidthRadS : 10.0;
    const double a = std::isfinite(perf.maxDecelerationMs2) && perf.maxDecelerationMs2 > 0.0 ? 0.5 * perf.maxDecelerationMs2 : 0.25;
    const double closing = std::fmin(transit, std::fmin(0.5 / lag * distance, a * (std::sqrt(lag * lag + 2.0 * distance / a) - lag)));
    const double k = distance > 1e-6 ? closing / distance : 0.0;
    return VelocityCommand{kHold, verticalSpeedMs, kHold, kHold, frameNorthMs + k * north, frameEastMs + k * east};
}

void replan(Plan& p, std::uint32_t i, bool firstLap, double altitudeMslM, double windMs, const Performance& performance, bool hovers) noexcept {
    const Waypoint& w = p.points[i];
    const bool entry = firstLap && i == p.start;
    Leg& in = entry ? p.entry : p.legs[i];
    if (entry) {
        in = makeLeg(in.latA, in.lonA, w.latitudeRad, w.longitudeRad, p.rhumb);
    } else if (i > 0 || p.repeat) {
        const std::uint32_t h = p.prev(i);
        in = legTo(p, h, i, h > 0 || p.repeat ? p.legs[h].courseInRad : in.courseOutRad); // (an arc's tangent: the leg before's, as planned)
    }
    Turn& t = entry ? p.entryTurn : p.turns[i];
    t = Turn{};
    if (!p.leaves(i)) return;
    const std::uint32_t j = p.next(i);
    Leg& out = p.legs[j];
    out = legTo(p, i, j, in.courseInRad);
    if (w.turn != static_cast<double>(TurnType::FlyBy) || noTurn(w) || (!entry && i == 0 && !p.repeat)) return;
    t = makeTurn(in.courseInRad, out.courseOutRad, turnRadiusAt(p, i, altitudeMslM, windMs, performance, hovers));
    if (entry && t.leadM > in.lengthM) { // (too near its point to turn before it: flown over)
        t = Turn{};
        return;
    }
    // its share of the legs either side, beside the turns at their other ends (plan()'s, here)
    const double before = entry ? 0.0 : firstLap && p.prev(i) == p.start ? p.entryTurn.leadM : p.turns[p.prev(i)].leadM;
    const double after = p.turns[j].leadM;
    double share = 1.0;
    if (before + t.leadM > in.lengthM) share = std::min(share, in.lengthM / (before + t.leadM));
    if (t.leadM + after > out.lengthM) share = std::min(share, out.lengthM / (t.leadM + after));
    if (t.radiusM > 0.0 && share < 1.0) shrink(t, t.leadM * share, true);
}

} // namespace fsim::control::route
