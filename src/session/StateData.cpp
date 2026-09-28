// The barometric altimeter's setting and what it reads (docs/flight-autonomy.md, 4.20; A-GRA's QNH setting, VI 1.2.6.5,
// and MA_AirDataType, VI 1.2.6.8): set and asked for between steps, never stepped.
#include "session/World.h"

#include "fsim/Altimeter.h"

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
    return out;
}

} // namespace fsim::session
