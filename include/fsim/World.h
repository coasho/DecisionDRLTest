#pragma once

// fsim SDK: the vehicle-instance API (design 9.2, ADR-18). A training
// application creates a World, creates vehicles by name/type/initial state,
// commands them at any control level and steps the world. Visualisation is
// transparent: every World publishes itself for flightsim-viewer.exe through
// shared memory (design 9.7) at no cost to stepping.

#include "fsim/Capability.h"
#include "fsim/Comm.h"
#include "fsim/Control.h"
#include "fsim/ControlInputs.h"
#include "fsim/ControlStack.h"
#include "fsim/Effects.h"
#include "fsim/EnvironmentState.h"
#include "fsim/Export.h"
#include "fsim/GroundProvider.h"
#include "fsim/InitialConditions.h"
#include "fsim/Property.h"
#include "fsim/VehicleProfile.h"
#include "fsim/VehicleState.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace fsim {

namespace session {
class World;
}

/// Thrown for load-time failures (unknown aircraft, bad type, duplicate name).
class Error : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct WorldOptions {
    std::string name = "default";     ///< viewers attach by this name
    double dt = 1.0 / 120.0;          ///< FDM step, seconds
    int frameSkip = 4;                ///< FDM steps per World::step()
    unsigned workers = 0;             ///< simulation threads; 0 = automatic (physical cores - 2)
    bool pinWorkers = true;           ///< one worker per physical core; disable when running several trainers on one machine
    std::uint64_t seed = 0;
    std::uint32_t capacity = 256;     ///< vehicle slots visible to viewers
    bool publish = true;              ///< false = never visible to viewers
    double publishIntervalSeconds = 1.0 / 60.0;
    std::string jsbsimRoot;           ///< empty = auto-detect
    bool terrain = false;             ///< physics ground from public elevation tiles (the ones the viewer draws); needs network or a warm cache
    std::string terrainUrl;           ///< XYZ template with {z}/{x}/{y}; empty = AWS Terrarium
    unsigned terrainZoom = 12;        ///< 12: ~38 m/px, 14: ~10 m/px (4x the tiles)
    std::shared_ptr<GroundProvider> ground; ///< your own ground model instead (overrides `terrain`)
    std::string recordPath;           ///< non-empty: record every vehicle's state for `flightsim-viewer --replay`
    double recordIntervalSeconds = 0.0; ///< simulation time between frames; 0 = every world step
};

struct VehicleSpec {
    std::string name;                 ///< unique; empty = "<aircraft>-<id>"
    std::string type = "jsbsim:c172x";///< "<flight model>:<aircraft>"
    InitialConditions initial;
    std::string model;                ///< optional glTF for the viewer
    unsigned controlDivider = 1;      ///< control stack every N FDM steps
    /// Profile sections to replace for this vehicle (those present(); see
    /// docs/control-architecture.md, 7.4): how a stock aircraft gets an
    /// envelope, say. Null = the aircraft's own.
    std::shared_ptr<const control::VehicleProfile> profile;
};

/// Calendar UTC for Environment::setTime.
struct Utc {
    int year = 2026, month = 1, day = 1, hour = 12, minute = 0;
    double second = 0.0;
};
FSIM_API double utcToUnixSeconds(const Utc& utc) noexcept;

struct Wind {
    double directionDeg = 0.0;  ///< blowing FROM, degrees true
    double speedMs = 0.0;
    double gustMs = 0.0;
    double turbulence = 0.0;    ///< 0..1
};
struct Atmosphere {
    double temperatureSeaLevelK = 288.15;
    double pressureSeaLevelPa = 101325.0;
    double humidity = 0.0;
};
struct Weather {
    double visibilityM = 50000.0;
    double cloudBaseM = 2000.0;
    double cloudCover = 0.0;
    double precipitation = 0.0;
};

class World;

/// Real-time environment control (design 9.4). Setters take effect at the next step.
class FSIM_API Environment {
public:
    void setTime(const Utc& utc);
    void setTime(double unixSeconds);
    void setTimeFactor(double factor);
    void setWind(const Wind& wind);
    void setAtmosphere(const Atmosphere& atmosphere);
    void setWeather(const Weather& weather);
    /// Everything at once (advanced: fields not covered by the setters).
    void set(const EnvironmentState& state);
    const EnvironmentState& state() const noexcept;
    /// Current UTC as Unix seconds (epoch + simulation time).
    double utcSeconds() const noexcept;

private:
    friend class World;
    explicit Environment(World& world) noexcept : world_(&world) {}
    World* world_;
};

/// Lightweight handle to a vehicle instance; copyable, valid until removed.
class FSIM_API Vehicle {
public:
    Vehicle() = default;
    bool valid() const noexcept;
    std::uint32_t id() const noexcept { return id_; }
    std::string name() const;
    std::string type() const;
    std::string model() const;  ///< the VehicleSpec::model override (empty = the type's model)

    // State (references are valid until the vehicle is removed)
    const VehicleState& state() const;            ///< truth
    const effects::SensedState& sensed() const;    ///< through sensor effects
    const ControlInputs& inputs() const;          ///< actuator inputs the cascade produced

    // Multi-level control (design 9.3): the per-step command. At the level
    // the vehicle's own activity flies it updates that activity, else it
    // starts a new one (docs/control-architecture.md, 10.7); false if the
    // command is rejected (a behaviour nobody registered, an axis an autopilot
    // or override activity holds).
    bool command(const control::Command& command);
    template <typename C>
    bool command(const C& c) { return command(control::Command(c)); }
    // Capability contracts (docs/sdk/control.md, "Capabilities and activities"):
    /// NEW: the command becomes an activity, or is rejected with a reason.
    control::CommandResult submit(const control::Command& command, const control::CommandOptions& options = {});
    /// NEW for a support effector (GearCommand, FlapsCommand, WheelBrakesCommand,
    /// SpeedbrakeCommand, PitchTrimCommand) or the engines' throttles (EnginesCommand).
    control::CommandResult submit(const control::SupportCommand& command, const control::CommandOptions& options = {});
    template <typename C>
    control::CommandResult submit(const C& c, const control::CommandOptions& options = {}) {
        if constexpr (std::is_constructible_v<control::SupportCommand, C> && !std::is_constructible_v<control::Command, C>)
            return submit(control::SupportCommand(c), options);
        else
            return submit(control::Command(c), options);
    }
    /// The live activities, then the ended ones the vehicle remembers, newest first.
    std::vector<control::ActivityRecord> activities() const;
    /// What the vehicle offers: its flight levels and behaviours.
    std::vector<control::CapabilityDescriptor> capabilities() const;
    control::CapabilityStatus capabilityStatus(std::string_view capability) const;
    /// What the vehicle knows about its aircraft: identity, effectors, envelope,
    /// propulsion, plant, performance, control (docs/control-architecture.md, 7).
    const control::VehicleProfile& profile() const;
    /// What flies the primary axes nobody owns (docs/sdk/control.md, "Owning
    /// axes apart"): Neutral, as always, or Hold - the heading, airspeed and
    /// height each had when it was let go. Reason::None if set.
    control::Reason setVehicleDefault(control::VehicleDefault mode);
    control::VehicleDefault vehicleDefault() const;
    /// Envelope protection (docs/sdk/control.md, "Envelope protection"): Limit
    /// by default for an aircraft with an envelope, else Off. It limits the
    /// demand; it does not keep the aircraft inside - what crosses a limit is
    /// reported in envelope(), which starts a new count at each call.
    control::Reason setProtection(control::ProtectionMode mode);
    control::ProtectionMode protection() const;
    control::EnvelopeStatus envelope();
    control::ControlStack& controls();
    control::Level activeLevel() const;
    bool use(control::Level level, std::string_view controllerId);

    // Lifecycle
    bool reset();
    bool reset(const InitialConditions& initial);
    bool remove();

    // Effects, properties, communication
    bool addEffect(std::unique_ptr<effects::Effect> effect);
    template <typename E, typename... Args>
    E* addEffect(Args&&... args) {
        auto e = std::make_unique<E>(std::forward<Args>(args)...);
        E* raw = e.get();
        return addEffect(std::move(e)) ? raw : nullptr;
    }
    PropertyHandle property(std::string_view path);
    comm::Node* node();

private:
    friend class World;
    Vehicle(World* world, std::uint32_t id) noexcept : world_(world), id_(id) {}
    World* world_ = nullptr;
    std::uint32_t id_ = 0;
};

class FSIM_API World {
public:
    explicit World(const WorldOptions& options = {});
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    /// Load a vehicle. Throws fsim::Error on failure.
    Vehicle createVehicle(const VehicleSpec& spec);
    Vehicle vehicle(std::uint32_t id) noexcept;
    Vehicle vehicle(std::string_view name) noexcept;
    std::vector<Vehicle> vehicles();
    std::size_t vehicleCount() const noexcept;

    /// UPDATE: a new setpoint for a live activity (the per-step path; allocates nothing).
    control::CommandResult update(control::ActivityId activity, const control::Command& setpoint);
    control::CommandResult update(control::ActivityId activity, const control::SupportCommand& setpoint);
    template <typename C>
    control::CommandResult update(control::ActivityId activity, const C& c) {
        if constexpr (std::is_constructible_v<control::SupportCommand, C> && !std::is_constructible_v<control::Command, C>)
            return update(activity, control::SupportCommand(c));
        else
            return update(activity, control::Command(c));
    }
    /// CANCEL: the activity ends; its axes fly the vehicle default.
    control::CommandResult cancel(control::ActivityId activity);
    /// A live or recently ended activity of any vehicle; empty if unknown.
    std::optional<control::ActivityRecord> activity(control::ActivityId activity) const;

    /// Advance every vehicle by n world steps (frameSkip FDM steps each).
    void step(unsigned n = 1);
    double time() const noexcept;            ///< simulation seconds since creation
    double stepSeconds() const noexcept;     ///< dt * frameSkip

    Environment& environment() noexcept { return environment_; }
    comm::Network& network();

    /// Give every vehicle (present and future) its own effect instance.
    void addEffectToAll(std::function<std::unique_ptr<effects::Effect>()> factory);
    template <typename E, typename... Args>
    void addEffectToAll(Args... args) {
        addEffectToAll([=] { return std::make_unique<E>(args...); });
    }

    const std::string& name() const noexcept;
    bool published() const noexcept;         ///< viewers can see this world
    std::uint64_t vehicleSteps() const noexcept;
    std::uint64_t worldSteps() const noexcept;
    void publishNow();

    /// Internal object (for the platform's own tools; not part of the stable API).
    session::World& impl() noexcept { return *impl_; }
    /// Internal: a view of a world owned elsewhere (VecEnv::world, the C ABI); not part of the stable API.
    static World* borrowed(session::World& w) { return new World(Borrow{}, w); }

private:
    friend class Vehicle;
    friend class Environment;
    struct Borrow {};
    World(Borrow, session::World& borrowed);
    std::shared_ptr<session::World> impl_;
    Environment environment_;
};

FSIM_API const char* version() noexcept;

} // namespace fsim
