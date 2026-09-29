// A route's later laps where their legs differ from its first's (docs/flight-autonomy.md, 4.36; ADR-29 FA-6g1): the arcs from
// a start turn at the point its links loop back to, its course left out, which its laps begin on the course they come back
// on - checked by the host as its first lap's are. Its own translation unit, last in the library: what is added before the
// runtime's code moves it (the A/B's lesson).
#include "control/CapabilityHost.h"
#include "control/Checks.h"
#include "control/Route.h"
#include "core/Geodesy.h"
#include "fsim/Altimeter.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

Reason CapabilityHost::checkLaps(route::Plan& p, const sim::VehicleState& state, double windMs, CheckLog& log, CommandResult& detail) const noexcept {
    const std::uint32_t m = p.lapLegsTo;
    if (m == 0) return Reason::None;
    constexpr double kDegree = 3.14159265358979323846 / 180.0;
    const Performance& f = performance_;
    const bool hovers = (adapter_->features() & kFeatureHover) != 0;
    auto point = [&detail](std::uint32_t i, Reason why) {
        detail.index = static_cast<std::int16_t>(std::min<std::uint32_t>(i, 0x7FFF));
        return why;
    };
    // an arc to point j the aircraft cannot turn at that point's speed and its full bank (a rotorcraft's, its tilt)
    auto tight = [&](std::uint32_t j, const route::Leg& arc) {
        const Waypoint& b = p.points[j];
        const double v = route::plannedSpeed(b.speed, b.speedReference, b.altitudeM), bank = hovers ? f.maxTiltRad : f.maxBankRad;
        return arc.arcRadiusM > 0.0 && std::isfinite(bank) && bank > 0.0 && arc.arcRadiusM < v * v / (9.80665 * std::tan(bank));
    };
    // each arc a later lap flies from the point it loops back to, from the leg back there - each point before one a start
    // turn whose course is left out: within 170 degrees, its radius given the arc's; the gradient to its end, where it is the
    // shorter lap's
    route::Leg in = p.loopLeg;
    for (std::uint32_t j = p.loop + 1; j <= m; ++j) {
        in = route::lapLeg(p, j, in);
        const Waypoint& a = p.points[j - 1];
        if (in.arcRadiusM > 0.0 && std::abs(in.arcAngleRad) > 170.0 * kDegree) return point(j, Reason::InvalidWaypoint);
        if (!isHold(a.turnRadiusM) && !(in.arcRadiusM > 0.0 && std::abs(in.arcRadiusM - a.turnRadiusM) <= std::max(1.0, 0.005 * a.turnRadiusM)))
            return point(j, Reason::InvalidWaypoint);
        if (tight(j, in) && !tight(j, p.legs[j])) log.find(Reason::InvalidWaypoint, static_cast<std::int16_t>(j), Constraint::MaxTurnRate);
        Waypoint& w = p.points[j];
        const bool barometric = w.altitudeReference == static_cast<double>(AltitudeReference::Barometric);
        if (!isHold(w.climbRateMs) || !isHold(w.climbOptimization) || w.altitudeReference == static_cast<double>(AltitudeReference::AboveGround) ||
            a.altitudeReference != w.altitudeReference || !(in.lengthM > 1.0) || !(in.lengthM < p.legs[j].lengthM))
            continue;
        const double h0 = barometric ? barometricMslM(config_->altimeter, a.altitudeM) : a.altitudeM;
        const double h1 = barometric ? barometricMslM(config_->altimeter, w.altitudeM) : w.altitudeM;
        const bool descends = h1 < h0;
        const double most = descends ? f.maxDescentMs : f.maxClimbMs;
        double rate = std::abs(h1 - h0) * route::plannedSpeed(w.speed, w.speedReference, w.altitudeM) / in.lengthM; // (what the gradient asks)
        if (std::isnan(most) || rate <= most) continue;
        log.limit(rate, most, static_cast<std::int16_t>(j), 8, descends ? Constraint::MaxDescentRate : Constraint::MaxClimbRate, Reason::PerformanceLimit);
        w.climbRateMs = most;
    }
    // at the point after them: an end's course its arc's there, within a degree; a fly-by's turn, fitted to its legs
    const Waypoint& w = p.points[m];
    if (w.turn == static_cast<double>(TurnType::EndTurn) && !isHold(w.courseRad) && std::abs(geo::wrapPi(in.courseInRad - w.courseRad)) > kDegree)
        return point(m, Reason::InvalidWaypoint);
    if (route::lapTurn(p, in, state.altitudeMslM, windMs, f, hovers).shrunk && !p.turns[m].shrunk)
        log.reshape(static_cast<std::int16_t>(m), Constraint::None, Reason::InvalidWaypoint);
    return Reason::None;
}

} // namespace fsim::control
