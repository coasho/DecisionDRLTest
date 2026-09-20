#pragma once

// The simulation session behind fsim::World (design 9.2): vehicles by name
// and type, lockstep stepping through the vehicle pool, control cascades and
// effects on the worker that owns each vehicle, the environment applied per
// step, the message fabric, and the shared-memory publisher for viewers.

#include "comm/Comm.h"
#include "control/ControlStack.h"
#include "effects/Effect.h"
#include "fsim/EnvironmentState.h"
#include "fsim/InitialConditions.h"
#include "fsim/Rng.h"
#include "ipc/WorldPublisher.h"
#include "sim/GroundProvider.h"
#include "sim/VehiclePool.h"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace fsim::session {

struct WorldOptions {
    std::string name = "default";       ///< shared-memory world name (viewers attach by it)
    double dt = 1.0 / 120.0;            ///< FDM step
    int frameSkip = 4;                  ///< FDM steps per world step
    unsigned workers = 0;               ///< 0 = physical cores - 2
    bool pinWorkers = true;             ///< one worker per physical core (throughput stability)
    std::uint64_t seed = 0;
    std::uint32_t capacity = 256;       ///< published vehicle slots
    bool publish = true;                ///< false = invisible to viewers
    double publishIntervalSeconds = 1.0 / 60.0;
    std::filesystem::path jsbsimRoot;   ///< empty = auto-detect
    std::shared_ptr<sim::GroundProvider> ground; ///< null = flat at 0 m
};

struct VehicleSpec {
    std::string name;                   ///< unique in the world; empty = "<type>-<id>"
    std::string type = "jsbsim:c172x";  ///< "<flight model>:<aircraft>"; a bare aircraft name means JSBSim
    sim::InitialConditions initial;
    std::string model;                  ///< optional visual override for the viewer (glTF path)
    unsigned controlDivider = 1;        ///< run the control stack every N FDM steps
};

struct VehicleInfo {
    std::uint32_t id = 0;
    std::string name, type, aircraft, model;
    sim::InitialConditions initial;
    bool alive = false;
    std::uint64_t generation = 0;
};

class World final : public control::WorldView {
public:
    explicit World(const WorldOptions& options);
    ~World() override;
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    // --- Vehicles ------------------------------------------------------------
    /// Create (load) a vehicle. Returns its id, or 0 on failure (logged).
    std::uint32_t createVehicle(const VehicleSpec& spec);
    bool removeVehicle(std::uint32_t id);
    /// Re-apply initial conditions (`ic` null = the spec's) and reset controls/effects.
    bool resetVehicle(std::uint32_t id, const sim::InitialConditions* ic = nullptr);
    bool alive(std::uint32_t id) const noexcept;
    const VehicleInfo* info(std::uint32_t id) const noexcept;
    std::uint32_t find(const std::string& name) const noexcept;
    std::vector<std::uint32_t> vehicleIds() const;
    std::size_t vehicleCount() const noexcept { return liveCount_; }

    // --- State ---------------------------------------------------------------
    const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept override;
    const effects::SensedState* sensedState(std::uint32_t id) const noexcept;
    sim::FlightModel* model(std::uint32_t id) noexcept;
    const sim::ControlInputs* inputs(std::uint32_t id) const noexcept;

    // --- Control -------------------------------------------------------------
    bool command(std::uint32_t id, const control::Command& command);
    control::ControlStack* controls(std::uint32_t id) noexcept;
    const control::ControlStack* controls(std::uint32_t id) const noexcept;

    // --- Effects ---------------------------------------------------------------
    bool addEffect(std::uint32_t id, std::unique_ptr<effects::Effect> effect);
    /// Give every vehicle (present and future) its own instance from `factory`.
    using EffectFactory = std::function<std::unique_ptr<effects::Effect>()>;
    void addEffectToAll(EffectFactory factory);
    void clearEffects(std::uint32_t id);

    // --- Environment, time, comm ---------------------------------------------
    const sim::EnvironmentState& environment() const noexcept override { return environment_; }
    void setEnvironment(const sim::EnvironmentState& environment);
    double simTime() const noexcept override { return simTime_; }
    double dt() const noexcept { return options_.dt; }
    int frameSkip() const noexcept { return options_.frameSkip; }
    comm::Network& network() noexcept { return network_; }
    Rng& rng() noexcept { return rng_; }
    const std::string& name() const noexcept { return options_.name; }
    bool published() const noexcept { return publisher_ && publisher_->active(); }

    // --- Stepping ------------------------------------------------------------
    /// Advance every vehicle by `n` world steps (frameSkip FDM steps each).
    void step(unsigned n = 1);
    std::uint64_t vehicleSteps() const noexcept { return vehicleSteps_; }
    std::uint64_t worldSteps() const noexcept { return worldSteps_; }
    /// Push the current state to viewers now, regardless of the publish interval.
    void publishNow();

private:
    struct Entry {
        VehicleInfo info;
        std::size_t slot = 0;
        control::ControlStack stack;
        std::vector<std::unique_ptr<effects::Effect>> effects;
        effects::SensedState sensed;
        Rng rng;
        sim::ControlInputs inputs;         ///< last inputs the cascade produced
        sim::VehicleState working;         ///< per-sub-step state for control/effects
        unsigned controlDivider = 1;
        bool forceApplied = false;         ///< external force pushed last step (needs zeroing)
        bool windApplied = false;
    };

    Entry* entry(std::uint32_t id) noexcept;
    const Entry* entry(std::uint32_t id) const noexcept;
    void preStep(std::size_t slot, int subStep, sim::FlightModel& model, sim::ControlInputs& inputs);
    void applyEnvironment(sim::FlightModel& model) const;
    void publishVehicle(const Entry& e);
    static bool parseType(const std::string& type, std::string& family, std::string& aircraft);

    WorldOptions options_;
    std::filesystem::path jsbsimRoot_;
    std::shared_ptr<sim::GroundProvider> ground_;
    std::unique_ptr<sim::VehiclePool> pool_;
    std::vector<std::unique_ptr<Entry>> entries_;               ///< by slot (null when never used)
    std::unordered_map<std::uint32_t, std::size_t> idToSlot_;
    std::vector<std::size_t> freeSlots_;
    std::vector<std::string> slotAircraft_;                     ///< loaded aircraft per slot (for reuse)
    std::vector<sim::ControlInputs> poolInputs_;
    std::vector<EffectFactory> worldEffects_;
    sim::EnvironmentState environment_;
    std::uint64_t appliedEnvironment_ = ~0ull;
    comm::Network network_;
    std::unique_ptr<ipc::WorldPublisher> publisher_;
    Rng rng_;
    std::uint32_t nextId_ = 1;
    std::size_t liveCount_ = 0;
    double simTime_ = 0.0;
    std::uint64_t vehicleSteps_ = 0, worldSteps_ = 0;
};

} // namespace fsim::session
