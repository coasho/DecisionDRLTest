#pragma once

#include "fsim/ControlInputs.h"
#include "fsim/InitialConditions.h"
#include "fsim/Property.h"
#include "fsim/VehicleState.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

namespace fsim::sim {

class GroundProvider;

/// What to load (design 7.2).
struct AircraftSpec {
    std::string name;                 ///< JSBSim aircraft name, e.g. "c172x", "f16"
    std::filesystem::path jsbsimRoot; ///< directory containing aircraft/, engine/, systems/
    /// Directory holding <name>/<name>.xml when it is not <jsbsimRoot>/aircraft
    /// (aircraft of our own: io::AssetResolver::findAircraft). The aircraft's
    /// Engines/ folder is searched before <jsbsimRoot>/engine, as JSBSim does.
    std::filesystem::path aircraftDir = {};
};



/// Flight-dynamics interface hiding JSBSim (design 7.1, ADR-10).
class FlightModel {
public:
    virtual ~FlightModel() = default;

    /// Load an aircraft and apply initial conditions. Returns false (and logs)
    /// on failure; never throws for data errors.
    virtual bool load(const AircraftSpec& aircraft, const InitialConditions& ic) = 0;

    /// Re-apply initial conditions to the loaded aircraft. Cheap (~1 ms).
    virtual bool reset(const InitialConditions& ic) = 0;

    /// Advance exactly one fixed step `dt()` with the given commands.
    virtual void step(const ControlInputs& inputs) = 0;

    /// Fill `out` with the current state in SI / ECEF metres.
    virtual void state(VehicleState& out) const = 0;

    /// Resolve a property path once (e.g. "velocities/vc-kts"). Invalid paths
    /// return an invalid handle and log at warn level.
    virtual PropertyHandle property(std::string_view path) = 0;

    virtual double dt() const noexcept = 0;
    virtual bool loaded() const noexcept = 0;

    // --- Environment and disturbances (design 9.4, 9.5). Defaults are no-ops
    // so a model that cannot honour a channel simply ignores it.

    /// Steady wind at the vehicle, NED m/s (the direction the air moves).
    virtual void setWindNed(double north, double east, double down) { (void)north; (void)east; (void)down; }
    /// Turbulence intensity 0 (off) .. 1 (severe) with the reference wind speed at 20 ft AGL.
    virtual void setTurbulence(double intensity, double windSpeed20ftMs) { (void)intensity; (void)windSpeed20ftMs; }
    /// Sea-level temperature (K) and pressure (Pa) of the standard atmosphere.
    virtual void setAtmosphere(double temperatureSeaLevelK, double pressureSeaLevelPa) { (void)temperatureSeaLevelK; (void)pressureSeaLevelPa; }
    /// External force (N) and moment (N m) in the body frame, applied at the
    /// centre of gravity, held until changed.
    virtual void setExternalForceBody(const double forceN[3], const double momentNm[3]) { (void)forceN; (void)momentNm; }
    /// Seed the model's own random processes (turbulence, dispersions).
    virtual void seed(std::uint64_t value) { (void)value; }
};

} // namespace fsim::sim
