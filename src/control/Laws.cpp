#include "control/Laws.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fsim::control {

namespace {

constexpr double kG = 9.80665;
constexpr double kDeg = 3.14159265358979323846 / 180.0;

bool known(double v) noexcept { return !std::isnan(v); }
double orZero(double v) noexcept { return known(v) ? v : 0.0; }

/// PD on a plant gain / (s (tau s + 1)): (kp, kd) that put the closed loop's
/// poles at omega with damping zeta - or, where that would take negative rate
/// feedback (a plant already that damped), kd = 0 and the dominant real pole at omega.
std::pair<double, double> place(double gain, double tau, double omega, double zeta = 0.8) {
    const double kd = (2.0 * zeta * omega * tau - 1.0) / gain;
    if (kd >= 0.0) return {omega * omega * tau / gain, kd};
    if (omega * tau < 0.5) return {omega * (1.0 - tau * omega) / gain, 0.0};
    return {1.0 / (4.0 * tau * gain), 0.0};
}

/// The bank a heading change is flown with, by class.
double maxBankDeg(AircraftClass c) noexcept {
    switch (c) {
    case AircraftClass::Fighter: return 60.0;
    case AircraftClass::Attack: return 50.0;
    case AircraftClass::LightGa: return 45.0;
    case AircraftClass::Uav: return 40.0;
    default: return 30.0;
    }
}

} // namespace

bool canDesignLaws(const VehicleProfile& p) noexcept {
    const PlantSection& pl = p.plant;
    auto response = [](const FirstOrder& r) { return known(r.gain) && known(r.tauS) && std::abs(r.gain) > 1e-6 && r.tauS > 0.0; };
    return pl.header.present() && known(pl.tasMs) && pl.tasMs > 1.0 && known(pl.easMs) && pl.easMs > 1.0 && response(pl.roll) &&
           response(pl.pitch) && known(pl.speed.gain) && known(pl.speed.tauS);
}

std::vector<ControllerSetting> designLaws(const VehicleProfile& p) {
    if (!canDesignLaws(p)) return {};
    const PlantSection& pl = p.plant;
    const bool fbw = p.identity.family == ControlFamily::FlyByWire;
    const double v = pl.tasMs, eas = pl.easMs;
    // roll: bank on the roll rate the aileron gives
    const double gp = pl.roll.gain, tp = pl.roll.tauS;
    const double wPhi = std::clamp((fbw ? 0.7 : 0.6) / tp, 0.5, 2.5);
    auto [rollKp, rollKd] = place(gp, tp, wPhi);
    if (!fbw) rollKd = std::max(rollKd, 0.25 * rollKp); // the dutch roll a first-order fit cannot see
    // pitch: attitude on the pitch rate the load factor gives
    const double gn = pl.pitch.gain, tn = pl.pitch.tauS;
    const double wTheta = std::clamp((fbw ? 0.7 : 0.6) / tn, 0.5, 2.5);
    auto [pitchKp, pitchKd] = place(kG * gn / v, tn, wTheta);
    if (!fbw) pitchKd = std::max(pitchKd, 0.25 * pitchKp); // and the short period
    const double pitchKi = pitchKp * wTheta / (fbw ? 2.5 : 4.0);
    // a law coordinates its turns; surfaces get half the sideslip taken out by
    // the rudder, the sign from the rudder's own step
    const double gb = pl.yaw.gain;
    const double beta = fbw || !known(gb) || std::abs(gb) < 1e-3 ? 0.0 : std::clamp(-0.5 / gb, -3.0, 3.0);
    // the trim law and the zero-lift angle, as identified
    const double trim = orZero(pl.elevatorTrim), lift = orZero(pl.elevatorTrimLift);
    const double alpha0 = std::clamp(orZero(pl.alphaZeroLiftRad), -0.2, 0.2);
    // speed: thrust per unit throttle over mass, and the engine's lag
    const double gv = std::max(pl.speed.gain, 1e-3), te = std::max(pl.speed.tauS, 0.2);
    const double wV = std::min(0.25, 0.3 / te);
    const double throttle = known(pl.throttleTrim) ? pl.throttleTrim : 0.55; // the loops' own default without one
    // outer loops a fraction of the inner ones' speed
    const double wVz = std::min(0.35, wTheta / 4.0);
    const double wPsi = std::min(0.2, wPhi / 5.0);
    const double wN = std::clamp(0.3 / tn, 0.3, 2.0);
    const double wP = std::clamp(0.3 / tp, 0.3, 3.0);
    const double wA = std::clamp(0.5 / te, 0.1, 1.0);

    std::vector<ControllerSetting> out;
    auto set = [&out](const char* controller, const char* parameter, double value) { out.push_back({controller, parameter, value}); };
    const char* a = "pid_attitude";
    set(a, "schedule.tas_ms", v), set(a, "schedule.eas_ms", eas);
    set(a, "roll.kp", rollKp), set(a, "roll.kd", rollKd), set(a, "roll.ki", 0.0), set(a, "roll.max_rate", std::clamp(0.5 * gp, 0.05, 2.0));
    set(a, "pitch.kp", pitchKp), set(a, "pitch.kd", pitchKd), set(a, "pitch.ki", pitchKi), set(a, "pitch.integral_limit", fbw ? 0.3 : 0.5);
    set(a, "pitch.trim", trim), set(a, "pitch.trim_lift", lift);
    set(a, "heading.gain", wPsi * v / kG), set(a, "rudder.beta_gain", beta);
    set(a, "airspeed.kp", 1.8 * wV / gv), set(a, "airspeed.ki", wV * wV / gv), set(a, "throttle.feedforward", throttle);
    set(a, "roll.eas_exponent", fbw ? 0.0 : -2.0), set(a, "roll.tas_exponent", fbw ? 0.0 : 2.0);
    set(a, "pitch.eas_exponent", 0.0), set(a, "pitch.tas_exponent", fbw ? -1.0 : 0.0);
    const char* n = "pid_acceleration";
    set(n, "schedule.tas_ms", v), set(n, "schedule.eas_ms", eas);
    // the stick per g beyond what neutral stick gives: most of a law's; all of
    // the identified response for surfaces without a trim law, which feeds it otherwise
    set(n, "load_factor.feedforward", fbw ? 0.85 / gn : lift == 0.0 ? 1.0 / gn : 0.0), set(n, "load_factor.path_hold", fbw ? 1.0 : 0.0);
    set(n, "load_factor.kp", 0.1 / gn), set(n, "load_factor.ki", wN / gn), set(n, "load_factor.kd", fbw ? 0.0 : pitchKd);
    set(n, "load_factor.integral_limit", 0.3), set(n, "pitch.trim", trim), set(n, "pitch.trim_lift", lift);
    set(n, "roll_rate.feedforward", 0.8 / gp), set(n, "roll_rate.kp", 0.1 / gp), set(n, "roll_rate.ki", wP / gp);
    set(n, "rudder.beta_gain", beta), set(n, "longitudinal.kp", te * wA / gv), set(n, "longitudinal.ki", wA / gv);
    set(n, "throttle.feedforward", throttle);
    set(n, "load_factor.eas_exponent", 0.0), set(n, "load_factor.tas_exponent", fbw ? 0.0 : 1.0);
    set(n, "roll_rate.eas_exponent", fbw ? 0.0 : -2.0), set(n, "roll_rate.tas_exponent", fbw ? 0.0 : 2.0);
    const char* vel = "pid_velocity";
    set(vel, "schedule.tas_ms", v);
    set(vel, "vertical_speed.kp", wVz / v), set(vel, "vertical_speed.ki", wVz * wVz / (3.0 * v));
    set(vel, "vertical_speed.feedforward", 1.0), set(vel, "vertical_speed.command_lag", 1.0 / wTheta);
    set(vel, "vertical_speed.alpha_zero_lift", alpha0), set(vel, "max_bank", maxBankDeg(p.identity.aircraftClass) * kDeg);
    const char* pos = "pid_position";
    set(pos, "altitude.gain", wVz / 3.0), set(pos, "max_vertical_speed", std::clamp(0.1 * v, 3.0, 25.0));
    // The attitude loop over pseudo-controls (step 5b): the same poles, placed
    // on the allocation's lags - the roll rate's, the load factor's - in rates
    // rather than deflections (a unit gain), with the same damping floors.
    const char* pa = "pseudo_attitude";
    auto [kPhi, dPhi] = place(1.0, tp, wPhi);
    if (!fbw) dPhi = std::max(dPhi, 0.25 * kPhi);
    auto [kTheta, dTheta] = place(1.0, tn, wTheta);
    if (!fbw) dTheta = std::max(dTheta, 0.25 * kTheta);
    set(pa, "roll.gain", kPhi), set(pa, "roll.kd", dPhi), set(pa, "roll.max_rate", std::clamp(0.5 * gp, 0.05, 2.0));
    set(pa, "heading.gain", wPsi * v / kG), set(pa, "schedule.tas_ms", v);
    // the pitch integrates the rate it asks for, and the allocation trims the
    // load factor: the integrator only takes out what the path's feedforward misses
    set(pa, "pitch.kp", kTheta), set(pa, "pitch.kd", dTheta), set(pa, "pitch.ki", kTheta * wTheta / 8.0);
    set(pa, "pitch.integral_limit", 0.1);
    set(pa, "airspeed.kp", wV), set(pa, "airspeed.ki", wV * wV / 4.0), set(pa, "airspeed.integral_limit", 2.0);
    // the allocation's thrust: the throttle an acceleration needs, from the identified response
    set(n, "longitudinal.feedforward", 1.0 / gv);
    return out;
}

bool completeControl(VehicleProfile& p) {
    if (p.control.header.present()) return false;
    auto settings = designLaws(p);
    if (settings.empty()) return false;
    p.control.settings = std::move(settings);
    p.control.controllers = {{Level::Attitude, "pseudo_attitude"}}; // the loops above ask for pseudo-controls
    p.control.header = {ControlSection::kVersion, Provenance::Derived};
    return true;
}

} // namespace fsim::control
