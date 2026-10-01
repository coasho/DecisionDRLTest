// A-GRA's taxi route (docs/flight-autonomy.md, 4.51; ADR-29 FA-9c: WPT-26): the host checks the route at the NEW against the
// aircraft's tightest turn on its wheels and its taxi speed; the "taxi" behaviour steers its nose wheel along the route's legs
// and the arcs drawn at its corners, slows for them and for its end, and stops short of another vehicle in its way.
#include "control/CapabilityHost.h"
#include "core/Geodesy.h"
#include "fsim/BuiltinControllers.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kWiderThanTightest = 1.25, kLeastTurnM = 8.0; // a corner's arc: a quarter wider than its tightest turn, 8 m at least
constexpr double kTightestUnknownM = 15.0;                     // (no turn radius in its profile)
constexpr double kMostTurnRad = 170.0 * kDeg;                  // a corner sharper is no taxi's
constexpr double kSideMs2 = 1.5;                               // its sideways acceleration in a turn: the arcs' speed...
constexpr double kBuildOffM = 0.75;                            // ...and how far a turn building as its yaw allows may carry it off
constexpr double kYawAccelUnknown = 0.1;                       // (no yaw acceleration in its profile: a heavy's, rad/s^2)
constexpr double kSlowMs2 = 1.0;                               // its deceleration for an arc and for its end
constexpr double kClearM = 25.0;                               // another vehicle that near its path ahead stands in its way...
constexpr double kStopShortM = 30.0;                           // ...and it stops that much short of where it comes nearest
constexpr std::size_t kSeen = 128;                             // the vehicles it looks at
constexpr double kYawDamping = 3.0;                            // rudder per rad/s its yaw rate is off: a heavy's swing stopped
constexpr double kCloseM = 6.0, kCloseS = 1.0;                 // its course and offset closed over this, or a second's run
constexpr double kSettledRad = 10.0 * kDeg, kSettledM = 1.5;   // back on its path

double clamp1(double x) noexcept { return std::clamp(x, -1.0, 1.0); }

/// The path through `n` + 1 points (north, east; the first where it starts) with each corner drawn as an arc of `radius`
/// into `out`: its pieces' count, or 0 with `bad` the point (0 the first given) whose corner or leg it cannot draw.
std::size_t draw(const double* north, const double* east, std::size_t n, double radius, TaxiBehavior::Piece* out, int& bad) noexcept {
    std::array<double, TaxiBehavior::kMaxPoints + 1> course{}, length{}, tangent{}, turn{};
    for (std::size_t k = 0; k < n; ++k) {
        const double dn = north[k + 1] - north[k], de = east[k + 1] - east[k];
        length[k] = std::hypot(dn, de), course[k] = std::atan2(de, dn);
        if (!(length[k] > 0.5)) return bad = static_cast<int>(k), 0; // (a point on the last: no leg between them)
    }
    for (std::size_t j = 1; j < n; ++j) { // the corner at point j: its turn, and how far before and after it its arc runs
        turn[j] = geo::wrapPi(course[j] - course[j - 1]);
        if (std::abs(turn[j]) > kMostTurnRad) return bad = static_cast<int>(j - 1), 0;
        tangent[j] = radius * std::tan(0.5 * std::abs(turn[j]));
    }
    for (std::size_t k = 0; k < n; ++k)
        if (length[k] < tangent[k] + tangent[k + 1] - 1e-9) return bad = static_cast<int>(k), 0; // (its arcs overlap on the leg)
    std::size_t count = 0;
    double s = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        const double c = course[k], un = std::cos(c), ue = std::sin(c);
        TaxiBehavior::Piece leg;
        leg.course = c, leg.n0 = north[k] + tangent[k] * un, leg.e0 = east[k] + tangent[k] * ue;
        leg.length = length[k] - tangent[k] - tangent[k + 1], leg.s = s;
        out[count++] = leg, s += leg.length;
        if (k + 1 < n && std::abs(turn[k + 1]) > 1e-9) { // the arc at the next point, tangent to both legs
            TaxiBehavior::Piece arc;
            arc.arc = true, arc.right = turn[k + 1] > 0.0, arc.course = c, arc.radius = radius;
            arc.n0 = north[k + 1] - tangent[k + 1] * un, arc.e0 = east[k + 1] - tangent[k + 1] * ue;
            const double side = arc.right ? 1.0 : -1.0; // its centre to the right, or the left
            arc.cn = arc.n0 - side * radius * ue, arc.ce = arc.e0 + side * radius * un;
            arc.length = radius * std::abs(turn[k + 1]), arc.s = s;
            out[count++] = arc, s += arc.length;
        }
    }
    return count;
}

/// Where `t` along `p` is, and its course there.
void pointOn(const TaxiBehavior::Piece& p, double t, double& north, double& east, double& course) noexcept {
    t = std::clamp(t, 0.0, p.length);
    if (!p.arc) {
        course = p.course, north = p.n0 + t * std::cos(p.course), east = p.e0 + t * std::sin(p.course);
        return;
    }
    const double side = p.right ? 1.0 : -1.0;
    course = p.course + side * t / p.radius;
    north = p.cn + side * p.radius * std::sin(course), east = p.ce - side * p.radius * std::cos(course);
}

/// How far along `p` the point (north, east) is, clamped to it, and how far to its right.
double project(const TaxiBehavior::Piece& p, double north, double east, double& cross) noexcept {
    if (!p.arc) {
        const double dn = north - p.n0, de = east - p.e0, un = std::cos(p.course), ue = std::sin(p.course);
        cross = -dn * ue + de * un;
        return std::clamp(dn * un + de * ue, 0.0, p.length);
    }
    const double vn = north - p.cn, ve = east - p.ce, r = std::hypot(vn, ve);
    if (p.right) {
        cross = p.radius - r;
        return std::clamp(geo::wrapPi(std::atan2(vn, -ve) - p.course) * p.radius, 0.0, p.length);
    }
    cross = r - p.radius;
    return std::clamp(geo::wrapPi(p.course - std::atan2(-vn, ve)) * p.radius, 0.0, p.length);
}

} // namespace

Reason CapabilityHost::prepareTaxi(BehaviorCommand& b, const sim::VehicleState& state, CommandResult& detail) {
    auto at = [&detail](std::int16_t point, Reason why, Constraint c = Constraint::None) {
        detail.index = point, detail.constraint = c;
        return why;
    };
    if (performance_.hovers) return at(-1, Reason::NotImplemented); // (a rotorcraft's ground taxi is not built)
    const std::size_t n = b.points.size();
    if (n == 0 || n > TaxiBehavior::kMaxPoints) return at(static_cast<std::int16_t>(std::min<std::size_t>(n, 0x7FFF)), Reason::InvalidWaypoint);
    // its corners' arcs, a quarter wider than its tightest turn on its wheels: each leg must hold the arcs at its ends, each
    // corner be no sharper than kMostTurnRad - else the point it cannot turn at, with its turn rate the limit
    RouteGround handling;
    taxiHandling(b.param("speed_ms", 8.0), handling);
    const double radius = handling.taxiRadiusM;
    std::array<double, TaxiBehavior::kMaxPoints + 1> north{}, east{};
    for (std::size_t i = 0; i < n; ++i)
        geo::localNorthEastM(state.latitudeRad, state.longitudeRad, b.points[i].latitudeRad, b.points[i].longitudeRad, north[i + 1], east[i + 1]);
    std::array<TaxiBehavior::Piece, 2 * TaxiBehavior::kMaxPoints + 1> path;
    int bad = -1;
    if (draw(north.data(), east.data(), n, radius, path.data(), bad) == 0) return at(static_cast<std::int16_t>(bad), Reason::InvalidWaypoint, Constraint::MaxTurnRate);
    TaxiBehavior::write(handling, state.latitudeRad, state.longitudeRad, b);
    return Reason::None;
}

int CapabilityHost::taxiFault(double lat0, double lon0, const double* latitudes, const double* longitudes, std::size_t n, double radius) const noexcept {
    if (n == 0 || n > TaxiBehavior::kMaxPoints) return static_cast<int>(std::min<std::size_t>(n, TaxiBehavior::kMaxPoints));
    std::array<double, TaxiBehavior::kMaxPoints + 1> north{}, east{};
    for (std::size_t i = 0; i < n; ++i) geo::localNorthEastM(lat0, lon0, latitudes[i], longitudes[i], north[i + 1], east[i + 1]);
    std::array<TaxiBehavior::Piece, 2 * TaxiBehavior::kMaxPoints + 1> path;
    int bad = -1;
    return draw(north.data(), east.data(), n, radius, path.data(), bad) == 0 ? bad : -1;
}

void CapabilityHost::taxiHandling(double speed, RouteGround& out) const noexcept {
    const double tightest = profile_ && std::isfinite(profile_->envelope.groundTurnRadiusM) ? profile_->envelope.groundTurnRadiusM : kTightestUnknownM;
    const double radius = std::max(kWiderThanTightest * tightest, kLeastTurnM);
    const double yaw = profile_ && std::isfinite(profile_->envelope.groundYawAccelRadS2) && profile_->envelope.groundYawAccelRadS2 > 0.0
                           ? profile_->envelope.groundYawAccelRadS2 : kYawAccelUnknown;
    // its speed round an arc: its sideways acceleration kSideMs2, and slow enough that the turn's yaw rate v / R, building
    // at its yaw acceleration a in (v / R) / a, carries it no more than kBuildOffM off: (v t)^2 / 2R, so v^2 <= a R sqrt(2 R
    // kBuildOffM) (a heavy's nose tyre turns it slowly: the KC-135R swung 94 deg past a corner at 3.4 m/s)
    const double building = std::sqrt(yaw * radius * std::sqrt(2.0 * radius * kBuildOffM));
    out.taxiSpeedMs = speed, out.taxiRadiusM = radius, out.tightestM = tightest, out.yawAccel = yaw;
    out.taxiTurnMs = std::min({speed, std::sqrt(kSideMs2 * radius), building});
}

void TaxiBehavior::write(const RouteGround& g, double lat0, double lon0, BehaviorCommand& b) {
    b.params["speed_ms"] = g.taxiSpeedMs;
    b.params["_lat0"] = lat0, b.params["_lon0"] = lon0;
    b.params["_radius"] = g.taxiRadiusM, b.params["_tightest"] = g.tightestM, b.params["_yaw"] = g.yawAccel, b.params["_turn_ms"] = g.taxiTurnMs;
}

void TaxiBehavior::reset() {
    count_ = at_ = 0;
    alongM_ = crossM_ = throttleI_ = stoppedS_ = 0.0;
    lastS_ = -1.0;
    done_ = held_ = false;
}

void TaxiBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    reset();
    const auto& s = ctx.sensed;
    lat0_ = command.param("_lat0", s.latitudeRad), lon0_ = command.param("_lon0", s.longitudeRad);
    speedMs_ = command.param("speed_ms", 8.0), turnRadiusM_ = command.param("_radius", kLeastTurnM);
    tightestM_ = command.param("_tightest", kTightestUnknownM), turnMs_ = command.param("_turn_ms", std::min(speedMs_, 5.0));
    yawAccel_ = command.param("_yaw", kYawAccelUnknown);
    std::array<double, kMaxPoints + 1> north{}, east{};
    const std::size_t n = std::min(command.points.size(), kMaxPoints);
    for (std::size_t i = 0; i < n; ++i)
        geo::localNorthEastM(lat0_, lon0_, command.points[i].latitudeRad, command.points[i].longitudeRad, north[i + 1], east[i + 1]);
    int bad = -1;
    count_ = draw(north.data(), east.data(), n, turnRadiusM_, path_.data(), bad); // (0: nothing to taxi - it stops where it is)
}

void TaxiBehavior::startResolved(const ControlContext& ctx, const RouteGround& g, const double* latitudes, const double* longitudes, std::size_t n) {
    reset();
    const auto& s = ctx.sensed;
    lat0_ = s.latitudeRad, lon0_ = s.longitudeRad;
    speedMs_ = g.taxiSpeedMs, turnRadiusM_ = g.taxiRadiusM, tightestM_ = g.tightestM, turnMs_ = g.taxiTurnMs, yawAccel_ = g.yawAccel;
    std::array<double, kMaxPoints + 1> north{}, east{};
    n = std::min(n, kMaxPoints);
    for (std::size_t i = 0; i < n; ++i) geo::localNorthEastM(lat0_, lon0_, latitudes[i], longitudes[i], north[i + 1], east[i + 1]);
    int bad = -1;
    count_ = draw(north.data(), east.data(), n, turnRadiusM_, path_.data(), bad);
}

void TaxiBehavior::at(double s, double& north, double& east, double& course) const noexcept {
    if (count_ == 0) {
        north = east = course = 0.0;
        return;
    }
    std::size_t i = 0;
    while (i + 1 < count_ && path_[i + 1].s <= s) ++i;
    if (s > path_[i].s + path_[i].length) { // beyond its end: on along the last course
        pointOn(path_[i], path_[i].length, north, east, course);
        const double more = s - path_[i].s - path_[i].length;
        north += more * std::cos(course), east += more * std::sin(course);
        return;
    }
    pointOn(path_[i], s - path_[i].s, north, east, course);
}

double TaxiBehavior::curvature(double s) const noexcept {
    std::size_t i = 0;
    while (i + 1 < count_ && path_[i + 1].s <= s) ++i;
    if (count_ == 0 || !path_[i].arc || s > path_[i].s + path_[i].length) return 0.0;
    return (path_[i].right ? 1.0 : -1.0) / path_[i].radius;
}

double TaxiBehavior::nearest(double north, double east, double& cross) noexcept {
    // on from the piece it was on: to the next while it is at this one's end
    double t = project(path_[at_], north, east, cross);
    while (at_ + 1 < count_ && t >= path_[at_].length - 1e-6) {
        double c2;
        const double t2 = project(path_[at_ + 1], north, east, c2);
        if (t2 <= 1e-6 && std::abs(c2) > std::abs(cross)) break; // (still nearer this one's end)
        ++at_, t = t2, cross = c2;
    }
    return path_[at_].s + t;
}

double TaxiBehavior::obstructed(const ControlContext& ctx, double along) const noexcept {
    // another vehicle within kClearM of its path ahead, as far as it could stop from its taxi speed and kStopShortM more: how
    // far along it comes nearest (vehicles in the air, or behind it, are none)
    if (!ctx.world) return kInf;
    std::array<std::uint32_t, kSeen> ids{};
    const std::uint32_t seen = std::min<std::uint32_t>(ctx.world->vehicles(ids.data(), kSeen), kSeen);
    const double reach = speedMs_ * speedMs_ / (2.0 * kSlowMs2) + kStopShortM + kClearM;
    double first = kInf;
    for (std::uint32_t k = 0; k < seen; ++k) {
        if (ids[k] == ctx.vehicleId) continue;
        const sim::VehicleState* o = ctx.world->vehicleState(ids[k]);
        if (!o || !o->onGround) continue;
        double on, oe;
        geo::localNorthEastM(lat0_, lon0_, o->latitudeRad, o->longitudeRad, on, oe);
        for (double d = 0.0; d <= reach; d += 5.0) {
            double pn, pe, c;
            at(along + d, pn, pe, c);
            if (std::hypot(on - pn, oe - pe) < kClearM) {
                first = std::min(first, d);
                break;
            }
        }
    }
    return first;
}

Command TaxiBehavior::update(const ControlContext& ctx, const Command&) {
    const auto& s = ctx.sensed;
    const double dt = lastS_ < 0.0 ? 0.0 : std::max(s.simTime - lastS_, 0.0);
    lastS_ = s.simTime;
    const double psi = s.eulerRad[2], r = s.angularRateBodyRadS[2], gs = std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]);
    ActuatorCommand a;
    a.gearDown = 1.0;
    a.aileron = clamp1(-2.0 * s.eulerRad[0] - 0.5 * s.angularRateBodyRadS[0]);
    a.elevator = 0.0;
    double north, east;
    geo::localNorthEastM(lat0_, lon0_, s.latitudeRad, s.longitudeRad, north, east);
    const double total = length();
    alongM_ = count_ ? nearest(north, east, crossM_) : 0.0;
    const double left = total - alongM_;
    // its nose wheel on its path: the curvature of the path ahead by the time its yaw takes to build an arc's yaw rate (half
    // of it), its course's error and its offset closed over a length its speed sets (+ right); the curvature asked for as a
    // share of its tightest turn's, the rudder's (+ rudder: nose left), its yaw rate's error damped. (A point ahead pursued
    // instead cut an 8 m arc 2.4 m inside, looked for that far ahead.) No brake on the inside: the arcs are drawn wider than
    // its tightest turn, and a single main wheel both pedals brake (the U-2S's) stopped it on its nose, its tail wheel off the ground
    double pn, pe, pc;
    at(alongM_, pn, pe, pc);
    const double vTurn = std::min(gs, turnMs_); // (at the speed it takes the arc: at its taxi speed, an F/A-18C turned in 37 m early)
    const double preview = 0.5 * vTurn * (vTurn / turnRadiusM_) / yawAccel_;
    const double closing = std::max(kCloseM, kCloseS * gs);
    const double kappa = curvature(alongM_ + preview) + 2.0 * geo::wrapPi(pc - psi) / closing - crossM_ / (closing * closing); // (+ right)
    const double steer = tightestM_ * kappa;
    a.rudder = clamp1(-steer + kYawDamping * (r - gs * kappa));
    // its speed: its taxi speed, slower into and round its arcs and to a stop at its end - or short of another vehicle in its way
    double target = std::min(speedMs_, std::sqrt(2.0 * kSlowMs2 * std::max(left - 0.5, 0.0)));
    for (std::size_t i = at_; i < count_ && path_[i].s < alongM_ + speedMs_ * speedMs_ / (2.0 * kSlowMs2) + 1.0; ++i)
        if (path_[i].arc) target = std::min(target, std::sqrt(turnMs_ * turnMs_ + 2.0 * kSlowMs2 * std::max(path_[i].s - alongM_, 0.0)));
    // ...and at its arcs' speed until it is back on its path, its course within 10 deg and its offset 1.5 m (past an arc the
    // RQ-4B, still swinging, was given its taxi speed and full throttle, and never came back)
    if (std::abs(geo::wrapPi(pc - psi)) > kSettledRad || std::abs(crossM_) > kSettledM) target = std::min(target, turnMs_);
    const double blocked = obstructed(ctx, alongM_);
    held_ = std::isfinite(blocked);
    if (held_) target = std::min(target, std::sqrt(2.0 * kSlowMs2 * std::max(blocked - kStopShortM, 0.0)));
    // the throttle on its speed (an integral for the breakaway and the slope), the brakes where it is faster than it should be
    const double err = target - gs;
    throttleI_ = std::clamp(throttleI_ + 0.05 * err * dt, 0.0, 0.5);
    a.throttle = std::clamp(0.15 * err + throttleI_, 0.0, 0.8);
    double brake = err < -0.3 ? std::clamp(0.4 * (-err - 0.3), 0.0, 1.0) : 0.0;
    if (target < 0.05) a.throttle = 0.0, throttleI_ = 0.0, brake = 1.0; // (held: stopped, or stopping)
    a.brakeLeft = a.brakeRight = brake;
    // done: stopped at its end, for a second
    stoppedS_ = left < 2.0 && gs < 0.3 ? stoppedS_ + dt : 0.0;
    if (stoppedS_ >= 1.0 || count_ == 0) done_ = true;
    return a;
}

bool TaxiBehavior::progress(ActivityProgress& out) const noexcept {
    const double total = length();
    out.percent = total > 0.0 ? std::clamp(100.0 * alongM_ / total, 0.0, 100.0) : 100.0;
    out.crossTrackM = crossM_;
    double n, e, c;
    at(alongM_, n, e, c);
    out.courseRad = geo::wrapTwoPi(c);
    return true;
}

} // namespace fsim::control
