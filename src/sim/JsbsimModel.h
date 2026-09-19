#pragma once

#include "sim/FlightModel.h"

#include <memory>
#include <string>
#include <vector>

namespace JSBSim {
class FGFDMExec;
}

namespace fsim::sim {

class GroundProvider;

/// JSBSim adapter: one FGFDMExec per vehicle, JSBSim used as-is (design 7,
/// ADR-3). Construct and load on one thread; step() may run on any thread as
/// long as no two threads touch the same instance concurrently.
class JsbsimModel final : public FlightModel {
public:
    /// @param dt fixed step in seconds (design 6.4)
    /// @param ground terrain source for gear contact and AGL; shared by all vehicles
    JsbsimModel(double dt, std::shared_ptr<const GroundProvider> ground);
    ~JsbsimModel() override;

    JsbsimModel(const JsbsimModel&) = delete;
    JsbsimModel& operator=(const JsbsimModel&) = delete;

    bool load(const AircraftSpec& aircraft, const InitialConditions& ic) override;
    bool reset(const InitialConditions& ic) override;
    void step(const ControlInputs& inputs) override;
    void state(VehicleState& out) const override;
    PropertyHandle property(std::string_view path) override;

    double dt() const noexcept override { return dt_; }
    bool loaded() const noexcept override { return loaded_; }

    /// Direct access for tests and tooling only.
    JSBSim::FGFDMExec& exec() noexcept { return *fdm_; }

private:
    void applyInitialConditions(const InitialConditions& ic);
    void cacheCommandNodes();
    bool checkDivergence();

    double dt_;
    std::shared_ptr<const GroundProvider> ground_;
    std::unique_ptr<JSBSim::FGFDMExec> fdm_;
    bool loaded_ = false;
    bool diverged_ = false;
    std::uint32_t stepCount_ = 0;

    // Cached command nodes (design 12.2: no string lookups in step()).
    PropertyHandle aileronCmd_, elevatorCmd_, rudderCmd_, flapCmd_, gearCmd_, leftBrakeCmd_, rightBrakeCmd_;
    std::vector<PropertyHandle> throttleCmd_;
};

} // namespace fsim::sim
