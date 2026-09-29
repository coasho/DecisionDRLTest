// A flight's endurance (docs/flight-autonomy.md, 4.18; A-GRA's VIOLATION_ENDURANCE): what a flight with an end
// needs, against what the vehicle has above its reserve. Checked at a NEW, never stepped: in a file of its own.
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

/// An altitude in its reference as the height above sea level: the ground's under the aircraft now, the isobar
/// its altimeter reads it on.
double aboveSea(double altitudeM, double reference, const sim::VehicleState& s, const Altimeter& altimeter) noexcept {
    if (isHold(altitudeM)) return s.altitudeMslM;
    return altitudeMslOf(altitudeM, static_cast<AltitudeReference>(static_cast<int>(reference)), s, &altimeter);
}

} // namespace

CommandDetails::Endurance CapabilityHost::endurance(const Command& setpoint, const sim::VehicleState& state, double now) const noexcept {
    CommandDetails::Endurance out;
    if (!sessionView_) return out;
    // only a flight with an end: a route that does not repeat (a must fly's among them: 4.42), a pattern timed or of so
    // many laps, a curve
    const bool route = std::holds_alternative<RouteCommand>(setpoint) || std::holds_alternative<MustFlyCommand>(setpoint);
    const auto* pattern = std::get_if<PatternCommand>(&setpoint);
    const auto* curve = std::get_if<CurveCommand>(&setpoint);
    if (route && (!routePlan_ || routePlan_->repeat || routePlan_->count == 0)) return out;
    if (pattern && isHold(pattern->durationS) && isHold(patternShape_.orbits)) return out; // (its shape as prepare() completed it)
    if (curve && (!curvePlan_ || curvePlan_->count == 0)) return out;
    if (!route && !pattern && !curve) return out;
    const EnergyNow en = sessionView_->energyNow(vehicle_);
    if (en.energy == Energy::Unknown || !std::isfinite(en.remaining)) return out;

    // flown level, each leg at its speed and altitude, at the weight it has by then - the fuel burned before it gone,
    // a minute at a time: the tables' burn there, else what it consumes now
    const TablesSection* tables = profile_ && !profile_->tables.empty() ? &profile_->tables : nullptr;
    const bool fuel = en.energy == Energy::Fuel;
    double seconds = 0.0, needed = 0.0;
    bool flown = true;
    auto fly = [&](double timeS, double tasMs, double altitudeMslM) {
        if (!(timeS >= 0.0) || !std::isfinite(timeS)) {
            flown = false;
            return;
        }
        const int steps = static_cast<int>(std::clamp(std::ceil(timeS / 60.0), 1.0, 64.0));
        const double step = timeS / steps;
        for (int k = 0; k < steps; ++k) {
            double burn = kUnknown;
            if (tables) {
                const TablesAtSpeed at = tablesAt(*tables, altitudeMslM, en.massKg - (fuel ? needed : 0.0), tasMs);
                burn = fuel ? at.fuelKgS : at.powerW;
            }
            if (!(burn >= 0.0)) burn = en.consumption; // (outside the tables' speeds, or none)
            needed += step * (burn >= 0.0 ? burn : 0.0);
        }
        seconds += timeS;
    };
    if (route) { // to its last point, from where the aircraft is: each leg, and the turn at its end - and each loiter (4.31)
        const route::Plan& p = *routePlan_;
        for (std::uint32_t i = p.start; i < p.count; ++i) {
            const Waypoint& w = p.points[i];
            const double h = aboveSea(w.altitudeM, w.altitudeReference, state, config_->altimeter);
            const double tas = route::plannedSpeed(w.speed, w.speedReference, h);
            const route::Turn& t = p.turn(i, true);
            const double m = p.pieceM(i, true) + (i + 1 < p.count ? t.radiusM * std::abs(t.angleRad) : 0.0);
            fly(tas > 0.5 ? m / tas : kUnknown, tas, h);
            const RouteLoiter* l = route::loiterPoint(w) ? p.loiterAt(i) : nullptr;
            if (!l) continue;
            // its duration, its laps (its way in from its point, and on to its exit), or its end time from its arrival - the
            // first; none (the last point's): the route ends as it begins. A hover burns as the tables' hover does.
            const PatternCommand& c = l->pattern;
            const double speed = route::plannedSpeed(c.speed, c.speedReference, h);
            double timeS = isHold(c.durationS) ? kUnknown : c.durationS;
            if (!isHold(l->shape.orbits) && c.pattern != static_cast<double>(PatternKind::Hover)) { // (from where it begins: 4.31)
                const route::Leg& in = p.leg(i, true);
                double lat = w.latitudeRad, lon = w.longitudeRad, course = in.courseInRad;
                if (const double join = route::loiterJoinM(c, l->shape); join > 0.0 && in.lengthM > join)
                    geo::destination(w.latitudeRad, w.longitudeRad, geo::wrapPi(in.courseInRad + 3.14159265358979323846), join, lat, lon);
                else if (join > 0.0) // (a leg shorter than that: joined where it begins - 4.31, as the schedule measures it: 4.33)
                    lat = in.latA, lon = in.lonA, course = in.courseOutRad;
                PatternShape shape = l->shape;
                route::loiterEntry(c, lat, lon, shape);
                if (p.leaves(i)) route::loiterExit(c, p.points[p.next(i)].latitudeRad, p.points[p.next(i)].longitudeRad, shape);
                route::Pattern lap;
                route::planPattern(lap, c, lat, lon, shape,
                                   l->shape.directionReference == static_cast<double>(DirectionReference::MagneticNorth) ? yearNow() : 2025.0,
                                   course);
                const double laps = speed > 0.5 ? (lap.entryM() + l->shape.orbits * lap.lapM() + lap.toExitM()) / speed : kUnknown;
                timeS = std::fmin(timeS, laps);
            }
            if (!isHold(l->endTimeS) && std::isfinite(now)) timeS = std::fmin(timeS, std::max(l->endTimeS - (now + seconds), 0.0));
            if (std::isnan(timeS)) continue; // (no end: the route's)
            fly(timeS, c.pattern == static_cast<double>(PatternKind::Hover) ? 0.0 : speed, h);
        }
    } else if (pattern) { // its duration, at its speed - or its laps from its way in (and on to its exit point), the first
        const double h = aboveSea(pattern->altitudeM, pattern->altitudeReference, state, config_->altimeter);
        const double tas = route::plannedSpeed(pattern->speed, pattern->speedReference, h);
        double timeS = pattern->durationS, speed = tas;
        if (pattern->pattern == static_cast<double>(PatternKind::Hover)) { // its way there at its speed, then its duration over it (4.25)
            const double away = geo::distanceM(state.latitudeRad, state.longitudeRad, pattern->latitudeRad, pattern->longitudeRad);
            fly(tas > 0.5 ? away / tas : kUnknown, tas, h);
            speed = 0.0; // (the tables' burn in the hover, their least speed)
        } else if (!isHold(patternShape_.orbits)) {
            route::Pattern p;
            route::planPattern(p, *pattern, state.latitudeRad, state.longitudeRad, patternShape_,
                               patternShape_.directionReference == static_cast<double>(DirectionReference::MagneticNorth) ? yearNow() : 2025.0,
                               route::trackOf(state));
            const double laps = tas > 0.5 ? (p.entryM() + patternShape_.orbits * p.lapM() + p.toExitM()) / tas : kUnknown;
            timeS = isHold(timeS) ? laps : std::fmin(timeS, laps);
        }
        fly(timeS, speed, h);
    } else { // to its end: in its duration, else at the speed it flies within its range
        const double h = isHold(curve->altitudeM) ? state.altitudeMslM : aboveSea(curve->altitudeM, curve->altitudeReference, state, config_->altimeter);
        const double length = curvePlan_->lengthM();
        double speed = (adapter_->features() & kFeatureHover) ? std::hypot(state.velocityNedMs[0], state.velocityNedMs[1]) : state.airspeedTrueMs;
        if (!isHold(curve->speedMaxMs)) speed = std::min(speed, curve->speedMaxMs);
        if (!isHold(curve->speedMinMs)) speed = std::max(speed, curve->speedMinMs);
        const double timeS = !isHold(curve->durationS) ? curve->durationS : speed > 0.5 ? length / speed : kUnknown;
        fly(timeS, timeS > 0.0 ? length / timeS : speed, h);
    }
    if (!flown) return out; // (a leg at no speed: no time to judge it by)
    const double left = std::max(en.remaining - (std::isfinite(en.reserve) ? en.reserve : 0.0), 0.0);
    out.energy = static_cast<std::uint8_t>(en.energy);
    out.remaining = left, out.required = needed;
    // (as the navigation report's endurance: none left, none; nothing consumed now, for ever)
    out.remainingS = !(left > 0.0) ? 0.0 : en.consumption > 0.0 ? left / en.consumption : std::numeric_limits<double>::infinity();
    out.requiredS = seconds;
    return out;
}

} // namespace fsim::control
