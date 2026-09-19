#pragma once

#include "sim/ControlInputs.h"
#include "sim/VehicleState.h"

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

class SGPropertyNode; // JSBSim property node; opaque outside JsbsimModel.cpp

namespace fsim::sim {

class GroundProvider;

/// What to load (design 7.2).
struct AircraftSpec {
    std::string name;                 ///< JSBSim aircraft name, e.g. "c172x", "f16"
    std::filesystem::path jsbsimRoot; ///< directory containing aircraft/, engine/, systems/
};

/// Where and how to start. Geodetic, SI. Sampled per vehicle by the
/// environment from the scenario's distributions.
struct InitialConditions {
    double latitudeDeg = 37.6188;   ///< default: KSFO area
    double longitudeDeg = -122.375;
    double altitudeMslM = 1500.0;
    double headingDeg = 0.0;
    double pitchDeg = 0.0;
    double rollDeg = 0.0;
    double airspeedTrueMs = 60.0;
    bool onGround = false;          ///< if true, altitude is taken from terrain
};

/// Cached, O(1) read/write access to one property after a one-time lookup
/// (design 7.1). Null-safe: get() on an invalid handle returns 0.
class PropertyHandle {
public:
    PropertyHandle() = default;
    explicit PropertyHandle(SGPropertyNode* node) noexcept : node_(node) {}

    bool valid() const noexcept { return node_ != nullptr; }
    double get() const noexcept;
    void set(double value) noexcept;

private:
    SGPropertyNode* node_ = nullptr;
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
};

} // namespace fsim::sim
