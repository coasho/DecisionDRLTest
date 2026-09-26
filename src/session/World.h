#pragma once

// The simulation session behind fsim::World (design 9.2): vehicles by name
// and type, lockstep stepping through the vehicle pool, control cascades and
// effects on the worker that owns each vehicle, the environment applied per
// step, the message fabric, and the shared-memory publisher for viewers.

#include "comm/Comm.h"
#include "control/Adapter.h"
#include "control/CapabilityHost.h"
#include "control/Catalog.h"
#include "control/ControlStack.h"
#include "fsim/VehicleProfile.h"
#include "effects/Effect.h"
#include "fsim/EnvironmentState.h"
#include "fsim/InitialConditions.h"
#include "fsim/Rng.h"
#include "io/TerrainTiles.h"
#include "ipc/Recording.h"
#include "ipc/WorldPublisher.h"
#include "sim/GroundProvider.h"
#include "sim/VehiclePool.h"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
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
    std::shared_ptr<sim::GroundProvider> ground; ///< custom ground; null = `terrain` below or flat at 0 m
    bool terrain = false;               ///< physics ground from the public elevation tiles the viewer draws
    std::string terrainUrl;             ///< tile template; empty = AWS Terrarium
    unsigned terrainZoom = 12;          ///< ~38 m/px
    double terrainPrefetchRadiusM = 4000.0; ///< tiles loaded (blocking) around each new vehicle's position
    std::filesystem::path recordPath;   ///< non-empty: write every vehicle's state to this file for replay in the viewer
    double recordIntervalSeconds = 0.0; ///< simulation time between recorded frames; 0 = every world step
};

struct VehicleSpec {
    std::string name;                   ///< unique in the world; empty = "<type>-<id>"
    std::string type = "jsbsim:c172x";  ///< "<flight model>:<aircraft>"; a bare aircraft name means JSBSim
    sim::InitialConditions initial;
    std::string model;                  ///< optional visual override for the viewer (glTF path)
    unsigned controlDivider = 1;        ///< run the control stack every N FDM steps
    /// Sections of the aircraft's profile to replace for this vehicle (those
    /// present(); docs/control-architecture.md, 7.4); null = the aircraft's own.
    std::shared_ptr<const control::VehicleProfile> profile;
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

    // --- Control (design 9.3, docs/control-architecture.md) ---------------------
    /// What every existing entry point calls: an update of the vehicle's own
    /// activity at the same level, else a new one (10.7). False if the vehicle
    /// is unknown or the command rejected (a behaviour nobody registered, an
    /// axis an autopilot or override activity holds).
    bool command(std::uint32_t id, const control::Command& command);
    /// command() with its answer (the C ABI reports why a command was refused).
    control::CommandResult commandResult(std::uint32_t id, const control::Command& command);
    /// NEW: a command becomes an activity, or is rejected with a reason.
    control::CommandResult submit(std::uint32_t id, const control::Command& command, const control::CommandOptions& options = {});
    /// NEW for a support effector the vehicle has (gear, flaps, brakes, speedbrake, pitch trim).
    control::CommandResult submit(std::uint32_t id, const control::SupportCommand& command, const control::CommandOptions& options = {});
    /// UPDATE: a new setpoint for a live activity (the fast path).
    control::CommandResult update(control::ActivityId activity, const control::Command& setpoint);
    control::CommandResult update(control::ActivityId activity, const control::SupportCommand& setpoint);
    /// CANCEL: the activity ends; its axes fly the vehicle default.
    control::CommandResult cancel(control::ActivityId activity);
    /// A live or recently ended activity; null if unknown.
    const control::ActivityRecord* activity(control::ActivityId activity) const noexcept;
    /// A vehicle's live activities, then the ended ones it remembers, newest first.
    std::vector<control::ActivityRecord> activities(std::uint32_t id) const;
    /// What a vehicle offers (empty for an unknown vehicle).
    const std::vector<control::CapabilityDescriptor>& capabilities(std::uint32_t id);
    control::CapabilityStatus capabilityStatus(std::uint32_t id, std::string_view capability) const;
    /// What the vehicle flies with: its aircraft's profile, with the spec's sections over it.
    const control::VehicleProfile* profile(std::uint32_t id) const noexcept;
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
    const comm::Network& network() const noexcept { return network_; }
    Rng& rng() noexcept { return rng_; }
    const std::string& name() const noexcept { return options_.name; }
    bool published() const noexcept { return publisher_ && publisher_->active(); }
    /// The elevation tiles used for physics when `terrain` is on (null otherwise).
    io::TerrainTiles* terrain() noexcept { return terrain_.get(); }

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
        control::ControlStack stack;       ///< the control runtime
        control::CapabilityHost host;      ///< its contract layer (bound to `stack`)
        control::ActivityId commanded = 0; ///< the activity the last accepted command made or updated
        control::Level level = control::Level::Actuator; ///< as last published and recorded
        std::shared_ptr<const control::VehicleProfile> profile;
        std::shared_ptr<control::CapabilityCatalog> catalog; ///< its aircraft type's (or its own, with a profile of its own)
        sim::PropertyHandle flapsPosition;                   ///< for flap activities that complete in position
        sim::EffectorInputs effectors;                       ///< as last written to the flight model
        std::vector<std::unique_ptr<effects::Effect>> effects;
        effects::SensedState sensed;
        Rng rng;
        sim::ControlInputs inputs;         ///< last inputs the cascade produced
        sim::VehicleState working;         ///< per-sub-step state for control/effects
        unsigned controlDivider = 1;
        bool forceApplied = false;         ///< external force pushed last step (needs zeroing)
        bool windApplied = false;
    };

    /// What the control cascades see of other vehicles during a step: each
    /// one's state as the step began (Control.h, WorldView), whichever worker
    /// steps which vehicle and however far it has got.
    class StepView final : public control::WorldView {
    public:
        explicit StepView(const World& world) noexcept : world_(world) {}
        const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept override;
        double simTime() const noexcept override { return world_.simTime_; }
        const sim::EnvironmentState& environment() const noexcept override { return world_.environment_; }

    private:
        const World& world_;
    };

    Entry* entry(std::uint32_t id) noexcept;
    const Entry* entry(std::uint32_t id) const noexcept;
    void preStep(std::size_t slot, int subStep, sim::FlightModel& model, sim::ControlInputs& inputs);
    void applyEnvironment(sim::FlightModel& model) const;
    void levelChanged(Entry& e);
    void publishVehicle(const Entry& e);
    static bool parseType(const std::string& type, std::string& family, std::string& aircraft);

    WorldOptions options_;
    std::filesystem::path jsbsimRoot_;
    std::shared_ptr<sim::GroundProvider> ground_;
    std::shared_ptr<io::TerrainTiles> terrain_;              ///< set when options.terrain (ground_ aliases it)
    std::unique_ptr<sim::VehiclePool> pool_;
    std::vector<std::unique_ptr<Entry>> entries_;               ///< by slot (null when never used)
    std::unordered_map<std::uint32_t, std::size_t> idToSlot_;
    std::vector<std::size_t> freeSlots_;
    std::vector<std::string> slotAircraft_;                     ///< loaded aircraft per slot (for reuse)
    /// Each loaded aircraft's profile (JSBSim properties under fsim/), read on its first load.
    std::unordered_map<std::string, std::shared_ptr<const control::VehicleProfile>> profiles_;
    /// What each aircraft type offers, from its profile through its adapter.
    std::unordered_map<std::string, std::shared_ptr<control::CapabilityCatalog>> catalogs_;
    std::vector<sim::ControlInputs> poolInputs_;
    std::vector<sim::VehicleState> stepStates_;                 ///< by slot: every state as the current step began
    StepView stepView_{*this};
    std::vector<EffectFactory> worldEffects_;
    sim::EnvironmentState environment_;
    std::uint64_t appliedEnvironment_ = ~0ull;
    comm::Network network_;
    std::unique_ptr<ipc::WorldPublisher> publisher_;
    ipc::RecordWriter recorder_;
    double lastRecordedTime_ = -1.0;
    void recordVehicle(const Entry& e);
    void recordFrame();
    Rng rng_;
    std::uint32_t nextId_ = 1;
    std::size_t liveCount_ = 0;
    double simTime_ = 0.0;
    std::uint64_t vehicleSteps_ = 0, worldSteps_ = 0;
};

} // namespace fsim::session
