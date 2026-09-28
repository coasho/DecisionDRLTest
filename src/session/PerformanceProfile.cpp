// A flight mode's performance profile (docs/flight-autonomy.md, 4.15; A-GRA's
// MA_FlightControlModesPerformanceProfileType): the guard rails a mission
// autonomy shapes its commands within, worked out at the vehicle's condition
// now from its performance tables, its envelope and its loops' Performance.
// Asked for, never stepped: in a file of its own, apart from the step.
#include "session/World.h"

#include "control/Atmosphere.h"
#include "fsim/PerformanceProfile.h"
#include "fsim/VehicleProfile.h"

#include <algorithm>
#include <cmath>

namespace fsim::session {

namespace {

using control::ProfileAcceleration;
using control::ProfilePoint;
constexpr double kNone = control::PerformanceProfile::kNone;
constexpr double kG = control::isa::kG0;
constexpr double kTop = 0.97; // the tables' level points run to this fraction of the top speed (4.13)

/// The least and the most of two, a NaN left out (NaN if both are).
double least(double a, double b) noexcept { return std::isnan(a) ? b : std::isnan(b) ? a : std::min(a, b); }
double most(double a, double b) noexcept { return std::isnan(a) ? b : std::isnan(b) ? a : std::max(a, b); }
/// `f` where it is given, else `c`: the flaps' limit, else the clean one's (as protection takes them).
double pick(double f, double c) noexcept { return std::isfinite(f) ? f : c; }

double tasOfCas(double casMs, double h) noexcept { return std::isfinite(casMs) ? control::isa::trueFromCalibrated(casMs, h) : kNone; }
double tasOfMach(double mach, double h) noexcept { return std::isfinite(mach) ? mach * control::isa::speedOfSound(h) : kNone; }

/// The capability that flies a mode A-GRA profiles; null for another.
const char* capabilityOf(control::FlightMode mode) noexcept {
    switch (mode) {
    case control::FlightMode::HsaCsa: return "fsim.guidance.hsa";
    case control::FlightMode::WaypointFollowing: return "fsim.guidance.route";
    case control::FlightMode::CurveFollowing: return "fsim.guidance.curve";
    default: return nullptr;
    }
}

} // namespace

control::Reason World::performanceProfile(std::uint32_t id, control::FlightMode mode, control::PerformanceProfile& out) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    const char* capability = capabilityOf(mode);
    if (!capability) return control::Reason::InvalidParameter; // (A-GRA profiles HSA/CSA, waypoint and curve following)
    if (e->catalog->find(capability) < 0) return e->support->refusal(capability);
    const control::Performance& perf = *performance(id);
    const sim::VehicleState& s = pool_->states()[e->slot];
    const sim::EnergyOnBoard en = pool_->vehicle(e->slot).energy();
    static const control::VehicleProfile kNoProfile;
    const control::VehicleProfile& prof = e->profile ? *e->profile : kNoProfile;
    const control::EnvelopeSection& env = prof.envelope;
    const control::TablesSection& t = prof.tables;
    const bool rotor = perf.hovers;

    // the condition now: its configuration as protection judges it (the flaps commanded beyond their threshold), the
    // retractable gear not fully up
    const sim::ControlInputs* in = inputs(id);
    // (its vectors kept, emptied: a profile asked again into the same one allocates nothing)
    for (auto* v : {&out.minAirspeed, &out.maxAirspeed, &out.bestEnduranceAirspeed, &out.bestRangeAirspeed, &out.maxDescentRate, &out.burn})
        v->clear();
    for (auto* v : {&out.minAcceleration, &out.maxAcceleration, &out.maxDeceleration}) v->clear();
    out.excessPower.clear(), out.maxOrientation.clear(), out.maxOrientationRate.clear();
    out.minAltitudeMslM = out.maxAltitudeMslM = out.maxTurnRateRadS = out.maxClimbRateMs = kNone;
    out.mode = mode;
    out.timeS = simTime_, out.altitudeMslM = s.altitudeMslM, out.weightKg = en.massKg, out.tasMs = s.airspeedTrueMs;
    out.flapsOut = in && in->flaps > env.flapsThreshold;
    out.gearDown = prof.effectors.retractableGear && s.gearPosition > 0.01;
    out.clean = !out.flapsOut && !out.gearDown;
    out.energy = en.fuelCapacityKg > 0.0 ? control::Energy::Fuel : en.chargeCapacityJ > 0.0 ? control::Energy::Battery : control::Energy::Unknown;
    control::EnvelopeLimits lim = env.clean;
    if (out.flapsOut) {
        const control::EnvelopeLimits& f = env.flaps;
        lim.loadFactorMin = pick(f.loadFactorMin, lim.loadFactorMin), lim.loadFactorMax = pick(f.loadFactorMax, lim.loadFactorMax);
        lim.bankMaxRad = pick(f.bankMaxRad, lim.bankMaxRad), lim.pitchMinRad = pick(f.pitchMinRad, lim.pitchMinRad);
        lim.pitchMaxRad = pick(f.pitchMaxRad, lim.pitchMaxRad), lim.rollRateMaxRadS = pick(f.rollRateMaxRadS, lim.rollRateMaxRadS);
        lim.casMinMs = f.casMinMs; // (the clean aircraft's least speed is not the flaps')
        lim.casMaxMs = pick(f.casMaxMs, lim.casMaxMs), lim.machMax = pick(f.machMax, lim.machMax);
    }
    const double w = out.weightKg, hNow = s.altitudeMslM, vNow = s.airspeedTrueMs;
    const bool tables = !t.empty() && out.clean; // (flown clean: with flaps or gear out, not what it flies)

    // the airspeeds against altitude, at the weight now: what it flies (the tables' least and top level speeds, a
    // rotorcraft's from the hover) within what it may (the envelope's calibrated and Mach limits, the gear's placard)
    auto airspeeds = [&](double h, double flownLo, double flownHi, double endurance, double range) {
        const double lo = out.flapsOut ? tasOfCas(lim.casMinMs, h) : most(flownLo, tasOfCas(lim.casMinMs, h));
        double hi = least(least(flownHi, tasOfCas(lim.casMaxMs, h)), tasOfMach(lim.machMax, h));
        if (out.gearDown) hi = least(hi, tasOfCas(env.gearCasMaxMs, h));
        if (std::isfinite(lo)) out.minAirspeed.push_back(ProfilePoint{lo, kNone, h, w});
        if (std::isfinite(hi)) out.maxAirspeed.push_back(ProfilePoint{hi, kNone, h, w});
        if (std::isfinite(endurance)) out.bestEnduranceAirspeed.push_back(ProfilePoint{endurance, kNone, h, w});
        if (std::isfinite(range)) out.bestRangeAirspeed.push_back(ProfilePoint{range, kNone, h, w});
    };
    if (tables) {
        for (const double h : t.altitudeM) {
            const control::TablesAt at = control::tablesAt(t, h, w);
            if (std::isfinite(at.maxTasMs)) airspeeds(h, at.minTasMs, at.maxTasMs, at.bestEnduranceTasMs, at.bestRangeTasMs);
        }
    } else {
        // none flown (a stock aircraft, or its flaps or gear out): its Performance's, at the altitude now
        const double flownHi = least(perf.maxTasMs, least(tasOfCas(perf.maxCasMs, hNow), tasOfMach(perf.maxMach, hNow)));
        airspeeds(hNow, out.clean ? tasOfCas(perf.minCasMs, hNow) : kNone, out.clean ? flownHi : kNone, kNone, kNone);
    }
    out.maxAltitudeMslM = !out.clean ? kNone : !t.empty() ? control::tablesCeilingM(t, w) : perf.ceilingM;

    // against airspeed and altitude, at the weight now: the tables' excess power at full power and at idle, their
    // burn; the load factors' accelerations with them (a rotorcraft's tilt and its loops' deceleration)
    const double zMax = std::isfinite(lim.loadFactorMin) ? -lim.loadFactorMin * kG : kNone; // (body z down: a push)
    const double zMin = std::isfinite(lim.loadFactorMax) ? -lim.loadFactorMax * kG : kNone; // (a pull)
    const double xyRotor = rotor ? perf.maxAccelerationMs2 : kNone, decelRotor = rotor ? perf.maxDecelerationMs2 : kNone;
    auto accelerations = [&](double v, double h, double psFull, double psIdle) {
        const double mach = v / control::isa::speedOfSound(h);
        const double full = !rotor && v > 0.0 && std::isfinite(psFull) ? psFull * kG / v : kNone;
        const double idle = !rotor && v > 0.0 && std::isfinite(psIdle) ? psIdle * kG / v : kNone;
        const ProfileAcceleration hi{rotor ? xyRotor : full, xyRotor, zMax, mach, v, h, w};
        const ProfileAcceleration lo{rotor ? -decelRotor : idle, rotor ? -xyRotor : kNone, zMin, mach, v, h, w};
        if (std::isfinite(hi.xMs2) || std::isfinite(hi.yMs2) || std::isfinite(hi.zMs2)) out.maxAcceleration.push_back(hi);
        if (std::isfinite(lo.xMs2) || std::isfinite(lo.yMs2) || std::isfinite(lo.zMs2)) out.minAcceleration.push_back(lo);
        const double decel = rotor ? -decelRotor : idle;
        if (std::isfinite(decel)) out.maxDeceleration.push_back(ProfileAcceleration{decel, kNone, kNone, mach, v, h, w});
        // the steepest it descends holding the speed (at idle), no faster than guidance asks
        const double descent = least(std::isfinite(psIdle) ? -psIdle : kNone, perf.maxDescentMs);
        if (std::isfinite(descent)) out.maxDescentRate.push_back(ProfilePoint{descent, v, h, w});
    };
    if (tables) {
        for (const double h : t.altitudeM) {
            const control::TablesAt at = control::tablesAt(t, h, w);
            if (!std::isfinite(at.maxTasMs)) continue;
            const double lo = at.minTasMs, hi = kTop * at.maxTasMs;
            for (const double f : t.speedFraction) {
                const double v = lo + f * (hi - lo);
                const control::TablesAtSpeed a = control::tablesAt(t, h, w, v);
                if (std::isfinite(a.psFullMs))
                    out.excessPower.push_back(control::ProfileExcessPower{a.psFullMs, !rotor && v > 0.0 ? a.psFullMs * kG / v : kNone, v, h, w});
                accelerations(v, h, a.psFullMs, a.psIdleMs);
                const double burn = out.energy == control::Energy::Fuel ? a.fuelKgS : out.energy == control::Energy::Battery ? a.powerW : kNone;
                if (std::isfinite(burn)) out.burn.push_back(ProfilePoint{burn, v, h, w});
            }
        }
    } else {
        accelerations(vNow, hNow, kNone, kNone); // (the limits alone, at the airspeed now)
    }

    // at the condition now: the attitudes and rates it may fly, the fastest turn and climb guidance flies
    out.maxOrientation.push_back(control::ProfileOrientation{kNone, pick(lim.pitchMaxRad, perf.maxPitchRad), pick(lim.pitchMinRad, perf.minPitchRad),
                                                             pick(lim.bankMaxRad, perf.maxBankRad), vNow, hNow, w});
    out.maxOrientationRate.push_back(control::ProfileRates{pick(lim.rollRateMaxRadS, perf.maxRollRateRadS), kNone, kNone, vNow});
    const double over = rotor ? std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]) : vNow; // (a rotorcraft turns over the ground)
    out.maxTurnRateRadS = over > 0.5 ? over / perf.turnRadiusM(over) : kNone;
    out.maxClimbRateMs = perf.maxClimbMs;
    return control::Reason::None;
}

} // namespace fsim::session
