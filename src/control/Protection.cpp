#include "control/Protection.h"

#include <algorithm>
#include <cmath>

namespace fsim::control {

namespace {

constexpr double kG = 9.80665;
/// The attitude loop's bank limit when a heading command gives none (AttitudeLoop).
constexpr double kLoopMaxBankRad = 0.785;
/// The elevator limiter begins this fraction of the way back from a limit (from
/// 1 g for the load factor, from zero lift for alpha)...
constexpr double kBand = 0.2;
/// ...and inside the band moves the elevator this many times what changes the
/// load factor by as much as the state is inside it, here.
constexpr double kStiffness = 2.0;

bool known(double v) noexcept { return !std::isnan(v); }
double orClean(double flaps, double clean) noexcept { return known(flaps) ? flaps : clean; }

/// cos(bank) for the angle-of-attack limits: a polynomial on |bank| up to 90
/// degrees (within 0.001 of the cosine; bit for bit the same on every
/// platform's maths library), floored at 0.1 - beyond 84 degrees of bank the
/// pitch attitude says little about the wing's angle.
double bankCos(double bankRad) noexcept {
    const double b = std::min(std::abs(bankRad), 1.5707963267948966);
    const double b2 = b * b;
    return std::max(1.0 - b2 * (0.5 - b2 * (1.0 / 24.0 - b2 * (1.0 / 720.0))), 0.1);
}

/// Clamps that remember which limits they applied. A comparison with NaN is
/// false, so a held field (kHold) or a limit the envelope does not give is
/// left alone without a test of its own - and without a branch.
struct Clamp {
    LimitMask hit = 0;
    void atMost(double& v, double hi, Limit l) noexcept {
        const bool over = v > hi;
        v = over ? hi : v;
        hit = static_cast<LimitMask>(hit | (static_cast<unsigned>(over) << static_cast<unsigned>(l)));
    }
    void atLeast(double& v, double lo, Limit l) noexcept {
        const bool under = v < lo;
        v = under ? lo : v;
        hit = static_cast<LimitMask>(hit | (static_cast<unsigned>(under) << static_cast<unsigned>(l)));
    }
    void within(double& v, double magnitude, Limit l) noexcept {
        atMost(v, magnitude, l);
        atLeast(v, -magnitude, l);
    }
};

/// A true airspeed setpoint within the calibrated limits and the Mach limit, converted at this condition.
void airspeed(Clamp& k, double& tas, const EnvelopeLimits& e, const LimitState& here) noexcept {
    k.atLeast(tas, e.casMinMs * here.tasPerCas, Limit::CasMin);
    k.atMost(tas, e.casMaxMs * here.tasPerCas, Limit::CasMax);
    if (here.tasPerMach > 0.0) k.atMost(tas, e.machMax * here.tasPerMach, Limit::Mach);
}

} // namespace

const char* limitName(Limit limit) noexcept {
    switch (limit) {
    case Limit::LoadFactorMax: return "load_factor_max";
    case Limit::LoadFactorMin: return "load_factor_min";
    case Limit::AlphaMax: return "alpha_max";
    case Limit::Bank: return "bank";
    case Limit::PitchMax: return "pitch_max";
    case Limit::PitchMin: return "pitch_min";
    case Limit::RollRate: return "roll_rate";
    case Limit::CasMin: return "cas_min";
    case Limit::CasMax: return "cas_max";
    case Limit::Mach: return "mach";
    default: return "?";
    }
}

LimitState limitState(const sim::VehicleState& s, const EnvelopeLimits& e, const Protection& p) noexcept {
    LimitState here;
    here.s = &s;
    if ((known(e.casMinMs) || known(e.casMaxMs)) && s.airspeedCalibratedMs > 1.0) here.tasPerCas = s.airspeedTrueMs / s.airspeedCalibratedMs;
    if (known(e.machMax) && s.mach > 0.05) here.tasPerMach = s.airspeedTrueMs / s.mach;
    const bool alpha = known(e.alphaMaxRad) && !(p.lawEnforces & limitBit(Limit::AlphaMax));
    if (alpha || known(e.pitchMinRad) || known(e.pitchMaxRad)) here.bankCos = bankCos(s.eulerRad[0]);
    return here;
}

Protection protectionFor(const VehicleProfile& profile) noexcept {
    Protection p;
    const EnvelopeSection& env = profile.envelope;
    if (!env.header.present()) return p; // nothing to protect: Off
    p.mode = ProtectionMode::Limit;
    p.clean = env.clean;
    const EnvelopeLimits &c = env.clean, &f = env.flaps;
    p.flaps.loadFactorMin = orClean(f.loadFactorMin, c.loadFactorMin);
    p.flaps.loadFactorMax = orClean(f.loadFactorMax, c.loadFactorMax);
    p.flaps.alphaMaxRad = orClean(f.alphaMaxRad, c.alphaMaxRad);
    p.flaps.bankMaxRad = orClean(f.bankMaxRad, c.bankMaxRad);
    p.flaps.pitchMinRad = orClean(f.pitchMinRad, c.pitchMinRad);
    p.flaps.pitchMaxRad = orClean(f.pitchMaxRad, c.pitchMaxRad);
    p.flaps.rollRateMaxRadS = orClean(f.rollRateMaxRadS, c.rollRateMaxRadS);
    p.flaps.casMinMs = orClean(f.casMinMs, c.casMinMs);
    p.flaps.casMaxMs = orClean(f.casMaxMs, c.casMaxMs);
    p.flaps.machMax = orClean(f.machMax, c.machMax);
    p.flapsThreshold = env.flapsThreshold;
    p.gearCasMaxMs = env.gearCasMaxMs;
    if (env.lawLoadFactor) p.lawEnforces |= limitBit(Limit::LoadFactorMax), p.lawEnforces |= limitBit(Limit::LoadFactorMin);
    if (env.lawAlpha) p.lawEnforces |= limitBit(Limit::AlphaMax);
    if (env.lawRollRate) p.lawEnforces |= limitBit(Limit::RollRate);
    p.alphaZeroLiftRad = known(profile.plant.alphaZeroLiftRad) ? profile.plant.alphaZeroLiftRad : 0.0;
    p.pitchSurface = profile.effectors.pitch == PitchControl::Surface;
    const double gain = std::abs(profile.plant.pitch.gain);
    if (known(gain) && gain > 0.1 && known(profile.plant.easMs) && profile.plant.easMs > 1.0) {
        p.elevatorGainG = gain;
        p.elevatorGainCasMs = profile.plant.easMs; // calibrated and equivalent airspeed are one below Mach 0.3 or so; a scale
    }
    return p;
}

const EnvelopeLimits& activeLimits(const Protection& p, double flapsCommand, double gearPosition, EnvelopeLimits& scratch) noexcept {
    const EnvelopeLimits& e = flapsCommand > p.flapsThreshold ? p.flaps : p.clean;
    if (!known(p.gearCasMaxMs) || gearPosition <= 0.01) return e;
    scratch = e;
    scratch.casMaxMs = known(e.casMaxMs) ? std::min(e.casMaxMs, p.gearCasMaxMs) : p.gearCasMaxMs;
    return scratch;
}

LimitMask limitSetpoint(Command& c, const EnvelopeLimits& e, const Protection& p, const LimitState& here) noexcept {
    const sim::VehicleState& s = *here.s;
    Clamp k;
    // alpha is limited through the pitch attitude and the load factor, unless the aircraft's law does it
    const bool alpha = known(e.alphaMaxRad) && !(p.lawEnforces & limitBit(Limit::AlphaMax));
    switch (c.index()) {
    case 1: {
        auto* a = std::get_if<AttitudeCommand>(&c);
        if (isHold(a->headingRad)) {
            k.within(a->rollRad, e.bankMaxRad, Limit::Bank);
        } else if (known(e.bankMaxRad)) { // the bank the heading is flown with
            if (isHold(a->maxBankRad) && e.bankMaxRad < kLoopMaxBankRad) a->maxBankRad = e.bankMaxRad, k.hit |= limitBit(Limit::Bank);
            else k.atMost(a->maxBankRad, e.bankMaxRad, Limit::Bank);
        }
        k.atMost(a->pitchRad, e.pitchMaxRad, Limit::PitchMax);
        k.atLeast(a->pitchRad, e.pitchMinRad, Limit::PitchMin);
        // the pitch attitude that puts the wing at alpha_max on the present flight path
        if (alpha) k.atMost(a->pitchRad, s.eulerRad[1] + (e.alphaMaxRad - s.alphaRad) * here.bankCos, Limit::AlphaMax);
        airspeed(k, a->airspeedMs, e, here);
        break;
    }
    case 2: {
        auto* n = std::get_if<AccelerationCommand>(&c);
        k.atMost(n->loadFactorG, e.loadFactorMax, Limit::LoadFactorMax);
        k.atLeast(n->loadFactorG, e.loadFactorMin, Limit::LoadFactorMin);
        // the load factor the wing gives at alpha_max here: lift grows with the angle from zero lift
        const double span = s.alphaRad - p.alphaZeroLiftRad;
        if (alpha && span > 0.035 && s.loadFactor > 0.3)
            k.atMost(n->loadFactorG, s.loadFactor * (e.alphaMaxRad - p.alphaZeroLiftRad) / span, Limit::AlphaMax);
        k.within(n->rollRateRadS, e.rollRateMaxRadS, Limit::RollRate);
        break;
    }
    case 3: {
        auto* v = std::get_if<VelocityCommand>(&c);
        airspeed(k, v->airspeedMs, e, here);
        const double tas = std::max(s.airspeedTrueMs, 10.0);
        if (known(e.bankMaxRad) && !isHold(v->turnRateRadS))
            k.within(v->turnRateRadS, kG * std::tan(std::min(e.bankMaxRad, 1.45)) / tas, Limit::Bank);
        // the vertical speed of the flight paths the pitch limits allow at this angle of attack
        if ((known(e.pitchMaxRad) || known(e.pitchMinRad)) && !isHold(v->verticalSpeedMs)) {
            const double onPath = s.alphaRad * here.bankCos;
            if (known(e.pitchMaxRad)) k.atMost(v->verticalSpeedMs, tas * std::sin(std::clamp(e.pitchMaxRad - onPath, -1.5, 1.5)), Limit::PitchMax);
            if (known(e.pitchMinRad)) k.atLeast(v->verticalSpeedMs, tas * std::sin(std::clamp(e.pitchMinRad - onPath, -1.5, 1.5)), Limit::PitchMin);
        }
        break;
    }
    case 4: airspeed(k, std::get_if<PositionCommand>(&c)->airspeedMs, e, here); break;
    default: break; // the actuators (limitElevator), a behaviour's parameters
    }
    return k.hit;
}

LimitMask limitElevator(double& elevator, const EnvelopeLimits& e, const Protection& p, const sim::VehicleState& s) noexcept {
    if (!p.pitchSurface || !known(p.elevatorGainG) || isHold(elevator) || s.onGround) return 0;
    auto ours = [&p](Limit l) { return !(p.lawEnforces & limitBit(l)); };
    const double n = s.loadFactor;
    double down = 0.0, up = 0.0; // how far inside the bands, in g
    LimitMask hit = 0;
    if (known(e.loadFactorMax) && ours(Limit::LoadFactorMax)) {
        const double on = e.loadFactorMax - kBand * std::max(e.loadFactorMax - 1.0, 0.5);
        if (n > on) down = n - on, hit |= limitBit(Limit::LoadFactorMax);
    }
    const double span = s.alphaRad - p.alphaZeroLiftRad;
    if (known(e.alphaMaxRad) && ours(Limit::AlphaMax) && span > 0.035 && n > 0.3) {
        const double on = e.alphaMaxRad - kBand * (e.alphaMaxRad - p.alphaZeroLiftRad);
        if (s.alphaRad > on) { // the load factor the wing makes from `on` up to here
            down = std::max(down, n * (s.alphaRad - on) / span);
            hit |= limitBit(Limit::AlphaMax);
        }
    }
    if (known(e.loadFactorMin) && ours(Limit::LoadFactorMin)) {
        const double on = e.loadFactorMin + kBand * std::max(1.0 - e.loadFactorMin, 0.5);
        if (n < on) up = on - n, hit |= limitBit(Limit::LoadFactorMin);
    }
    if (!hit) return 0;
    // the elevator that changes the load factor by a g here: the plant's, by the dynamic pressure
    const double q = s.airspeedCalibratedMs / p.elevatorGainCasMs;
    const double gPerUnit = std::max(p.elevatorGainG * q * q, 0.5);
    const double before = elevator;
    // +elevator is nose-down (JSBSim's sign); only ever away from the limit it nears
    if (down > 0.0) elevator = std::min(1.0, elevator + kStiffness * down / gPerUnit);
    else elevator = std::max(-1.0, elevator - kStiffness * up / gPerUnit);
    return elevator != before ? hit : 0;
}

LimitMask exceeded(const EnvelopeLimits& e, const sim::VehicleState& s, std::array<double, kLimitCount>& excess) noexcept {
    if (s.onGround) return 0;
    LimitMask m = 0;
    auto over = [&](Limit l, double by) {
        if (by > 0.0) { // false for a limit the envelope does not give (NaN)
            excess[static_cast<std::size_t>(l)] = by;
            m = static_cast<LimitMask>(m | limitBit(l));
        }
    };
    over(Limit::LoadFactorMax, s.loadFactor - e.loadFactorMax);
    over(Limit::LoadFactorMin, e.loadFactorMin - s.loadFactor);
    over(Limit::AlphaMax, s.alphaRad - e.alphaMaxRad);
    over(Limit::Bank, std::abs(s.eulerRad[0]) - e.bankMaxRad);
    over(Limit::PitchMax, s.eulerRad[1] - e.pitchMaxRad);
    over(Limit::PitchMin, e.pitchMinRad - s.eulerRad[1]);
    over(Limit::RollRate, std::abs(s.angularRateBodyRadS[0]) - e.rollRateMaxRadS);
    over(Limit::CasMin, e.casMinMs - s.airspeedCalibratedMs);
    over(Limit::CasMax, s.airspeedCalibratedMs - e.casMaxMs);
    over(Limit::Mach, s.mach - e.machMax);
    return m;
}

} // namespace fsim::control
