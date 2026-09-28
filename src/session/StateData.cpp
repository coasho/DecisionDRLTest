// The barometric altimeter's setting and what it reads (docs/flight-autonomy.md, 4.20; A-GRA's QNH setting, VI 1.2.6.5,
// and MA_AirDataType, VI 1.2.6.8): set and asked for between steps, never stepped.
#include "session/World.h"

#include "fsim/Altimeter.h"

#include <cmath>
#include <limits>

namespace fsim::session {

namespace {

/// The settings an altimeter takes: 850 to 1,100 hPa, the sea-level pressures the Earth has seen.
constexpr double kLeastQnhPa = 85000.0, kMostQnhPa = 110000.0;

} // namespace

control::Reason World::setQnh(std::uint32_t id, double qnhPa) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    if (!(qnhPa >= kLeastQnhPa && qnhPa <= kMostQnhPa)) return control::Reason::OutOfRange;
    control::Altimeter a = e->host.altimeter();
    a.qnhPa = qnhPa;
    e->host.setAltimeter(a); // (a barometric altitude is flown on its new isobar from the next step)
    return control::Reason::None;
}

double World::qnh(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? e->host.altimeter().qnhPa : std::numeric_limits<double>::quiet_NaN();
}

control::StateData World::stateData(std::uint32_t id) const {
    control::StateData out;
    const Entry* e = entry(id);
    if (!e) return out;
    const sim::VehicleState& s = pool_->states()[e->slot];
    const control::Altimeter a = e->host.altimeter();
    out.indicatedAltitudeM = control::indicatedAltitudeM(a, s.altitudeMslM);
    out.indicatedAltitudeRateMs = control::indicatedRateMs(a, s.altitudeMslM, -s.velocityNedMs[2]);
    out.kollsmanHpa = a.qnhPa / 100.0;
    out.staticPressurePa = control::staticPressurePa(a.air, s.altitudeMslM);
    out.staticTemperatureK = control::staticTemperatureK(a.air, s.altitudeMslM);

    // its Euler angles' rates and accelerations (docs/flight-autonomy.md, 4.21), from its body's rates and their
    // accelerations through the kinematic equations: none where yaw and roll are one, pitched straight up or down
    const double phi = s.eulerRad[0], theta = s.eulerRad[1], psi = s.eulerRad[2];
    const double p = s.angularRateBodyRadS[0], q = s.angularRateBodyRadS[1], r = s.angularRateBodyRadS[2];
    const double sphi = std::sin(phi), cphi = std::cos(phi), stheta = std::sin(theta), ctheta = std::cos(theta);
    if (std::abs(ctheta) > 1e-3) {
        const double t = stheta / ctheta, across = q * sphi + r * cphi;
        const double rollRate = p + across * t, pitchRate = q * cphi - r * sphi;
        out.rollRateRadS = rollRate, out.pitchRateRadS = pitchRate, out.yawRateRadS = across / ctheta;
        const auto& dot = e->angularAcceleration;
        if (dot[0].valid() && dot[1].valid() && dot[2].valid()) {
            const double pd = dot[0].get(), qd = dot[1].get(), rd = dot[2].get();
            const double acrossDot = qd * sphi + rd * cphi + rollRate * pitchRate;
            out.pitchAccelerationRadS2 = qd * cphi - rd * sphi - rollRate * across;
            out.yawAccelerationRadS2 = (acrossDot + across * t * pitchRate) / ctheta;
            out.rollAccelerationRadS2 = pd + acrossDot * t + across * pitchRate / (ctheta * ctheta);
        }
    }
    out.wanderAngleRad = 0.0; // (its navigation frame is north's)

    // the wind where it is (4.21): its ground velocity less its velocity through the air, turned from its body's axes
    // to north, east and down
    const double v = s.airspeedTrueMs, ca = std::cos(s.alphaRad), sa = std::sin(s.alphaRad), cb = std::cos(s.betaRad), sb = std::sin(s.betaRad);
    const double u = v * ca * cb, side = v * sb, w = v * sa * cb;
    const double cpsi = std::cos(psi), spsi = std::sin(psi);
    const double north = ctheta * cpsi * u + (sphi * stheta * cpsi - cphi * spsi) * side + (cphi * stheta * cpsi + sphi * spsi) * w;
    const double east = ctheta * spsi * u + (sphi * stheta * spsi + cphi * cpsi) * side + (cphi * stheta * spsi - sphi * cpsi) * w;
    const double down = -stheta * u + sphi * ctheta * side + cphi * ctheta * w;
    out.windNorthMs = s.velocityNedMs[0] - north, out.windEastMs = s.velocityNedMs[1] - east, out.windDownMs = s.velocityNedMs[2] - down;
    return out;
}

} // namespace fsim::session
