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
