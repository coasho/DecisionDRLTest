#pragma once

// fsim SDK: the vehicle-instance API (design 9.2, ADR-18). A training
// application creates a World, creates vehicles by name/type/initial state,
// commands them at any control level and steps the world. Visualisation is
// transparent: every World publishes itself for flightsim-viewer.exe through
// shared memory (design 9.7) at no cost to stepping.

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
#include "fsim/VehicleState.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
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

    // Multi-level control (design 9.3)
    bool command(const control::Command& command);
    template <typename C>
    bool command(const C& c) { return command(control::Command(c)); }
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
