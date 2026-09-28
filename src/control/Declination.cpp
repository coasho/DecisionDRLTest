// A magnetic direction turned true (docs/flight-autonomy.md, 4.22): the declination where the aircraft is, at the world's
// date. Asked when a command is resolved or checked, never stepped: in a file of its own.
#include "control/CapabilityHost.h"

#include "fsim/Magnetic.h"

namespace fsim::control {

double CapabilityHost::declinationNow(const sim::VehicleState& state) const noexcept {
    // (a host with no session reads the model's epoch)
    const double year = magneticYear(sessionView_ ? sessionView_->utcSeconds() : 0.0);
    return declinationRad(state.latitudeRad, state.longitudeRad, state.altitudeMslM, year);
}

} // namespace fsim::control
