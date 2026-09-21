// fsim SDK: World / Vehicle / Environment over session::World.

#include "fsim/World.h"

#include "core/Log.h"
#include "core/Time.h"
#include "session/World.h"

#include <chrono>
#include <ctime>

namespace fsim {

namespace {

const VehicleState& emptyState() {
    static const VehicleState s{};
    return s;
}
const effects::SensedState& emptySensed() {
    static const effects::SensedState s{};
    return s;
}
const ControlInputs& emptyInputs() {
    static const ControlInputs i{};
    return i;
}

} // namespace

double utcToUnixSeconds(const Utc& utc) noexcept {
    return core::unixSecondsFromCivil(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second);
}

// --- Environment --------------------------------------------------------------------

void Environment::setTime(const Utc& utc) { setTime(utcToUnixSeconds(utc)); }

void Environment::setTime(double unixSeconds) {
    auto s = world_->impl_->environment();
    s.epochUtcSeconds = unixSeconds - world_->impl_->simTime();
    world_->impl_->setEnvironment(s);
}

void Environment::setTimeFactor(double factor) {
    auto s = world_->impl_->environment();
    s.timeFactor = factor;
    world_->impl_->setEnvironment(s);
}

void Environment::setWind(const Wind& wind) {
    auto s = world_->impl_->environment();
    s.windDirectionDeg = wind.directionDeg;
    s.windSpeedMs = wind.speedMs;
    s.windGustMs = wind.gustMs;
    s.turbulence = wind.turbulence;
    world_->impl_->setEnvironment(s);
}

void Environment::setAtmosphere(const Atmosphere& atmosphere) {
    auto s = world_->impl_->environment();
    s.temperatureSeaLevelK = atmosphere.temperatureSeaLevelK;
    s.pressureSeaLevelPa = atmosphere.pressureSeaLevelPa;
    s.humidity = atmosphere.humidity;
    world_->impl_->setEnvironment(s);
}

void Environment::setWeather(const Weather& weather) {
    auto s = world_->impl_->environment();
    s.visibilityM = weather.visibilityM;
    s.cloudBaseM = weather.cloudBaseM;
    s.cloudCover = weather.cloudCover;
    s.precipitation = weather.precipitation;
    world_->impl_->setEnvironment(s);
}

void Environment::set(const EnvironmentState& state) { world_->impl_->setEnvironment(state); }
const EnvironmentState& Environment::state() const noexcept { return world_->impl_->environment(); }
double Environment::utcSeconds() const noexcept { return world_->impl_->environment().epochUtcSeconds + world_->impl_->simTime(); }

// --- Vehicle ------------------------------------------------------------------------

bool Vehicle::valid() const noexcept { return world_ && world_->impl_->alive(id_); }

std::string Vehicle::name() const {
    const auto* i = world_ ? world_->impl_->info(id_) : nullptr;
    return i ? i->name : std::string();
}

std::string Vehicle::type() const {
    const auto* i = world_ ? world_->impl_->info(id_) : nullptr;
    return i ? i->type : std::string();
}

std::string Vehicle::model() const {
    const auto* i = world_ ? world_->impl_->info(id_) : nullptr;
    return i ? i->model : std::string();
}

const VehicleState& Vehicle::state() const {
    const auto* s = world_ ? world_->impl_->vehicleState(id_) : nullptr;
    return s ? *s : emptyState();
}

const effects::SensedState& Vehicle::sensed() const {
    const auto* s = world_ ? world_->impl_->sensedState(id_) : nullptr;
    return s ? *s : emptySensed();
}

const ControlInputs& Vehicle::inputs() const {
    const auto* s = world_ ? world_->impl_->inputs(id_) : nullptr;
    return s ? *s : emptyInputs();
}

bool Vehicle::command(const control::Command& command) { return world_ && world_->impl_->command(id_, command); }

control::ControlStack& Vehicle::controls() {
    auto* c = world_ ? world_->impl_->controls(id_) : nullptr;
    if (!c) throw Error("Vehicle::controls: invalid vehicle handle");
    return *c;
}

control::Level Vehicle::activeLevel() const {
    const auto* c = world_ ? world_->impl_->controls(id_) : nullptr;
    return c ? c->activeLevel() : control::Level::Actuator;
}

bool Vehicle::use(control::Level level, std::string_view controllerId) {
    auto* c = world_ ? world_->impl_->controls(id_) : nullptr;
    return c && c->use(level, controllerId);
}

bool Vehicle::reset() { return world_ && world_->impl_->resetVehicle(id_); }
bool Vehicle::reset(const InitialConditions& initial) { return world_ && world_->impl_->resetVehicle(id_, &initial); }
bool Vehicle::remove() { return world_ && world_->impl_->removeVehicle(id_); }
bool Vehicle::addEffect(std::unique_ptr<effects::Effect> effect) { return world_ && world_->impl_->addEffect(id_, std::move(effect)); }

PropertyHandle Vehicle::property(std::string_view path) {
    auto* m = world_ ? world_->impl_->model(id_) : nullptr;
    return m ? m->property(path) : PropertyHandle();
}

comm::Node* Vehicle::node() { return world_ ? world_->impl_->network().node(id_) : nullptr; }

// --- World --------------------------------------------------------------------------

World::World(const WorldOptions& options) : environment_(*this) {
    session::WorldOptions o;
    o.name = options.name;
    o.dt = options.dt;
    o.frameSkip = options.frameSkip;
    o.workers = options.workers;
    o.pinWorkers = options.pinWorkers;
    o.seed = options.seed;
    o.capacity = options.capacity;
    o.publish = options.publish;
    o.publishIntervalSeconds = options.publishIntervalSeconds;
    o.jsbsimRoot = options.jsbsimRoot;
    o.terrain = options.terrain;
    o.terrainUrl = options.terrainUrl;
    o.terrainZoom = options.terrainZoom;
    o.ground = options.ground;
    o.recordPath = options.recordPath;
    o.recordIntervalSeconds = options.recordIntervalSeconds;
    impl_ = std::make_shared<session::World>(o);
    // Default epoch: now, so the viewer's sun matches the wall clock unless told otherwise.
    auto s = impl_->environment();
    s.epochUtcSeconds = static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    impl_->setEnvironment(s);
}

World::World(Borrow, session::World& borrowed) : impl_(&borrowed, [](session::World*) {}), environment_(*this) {}

World::~World() = default;

Vehicle World::createVehicle(const VehicleSpec& spec) {
    session::VehicleSpec s;
    s.name = spec.name;
    s.type = spec.type;
    s.initial = spec.initial;
    s.model = spec.model;
    s.controlDivider = spec.controlDivider;
    const std::uint32_t id = impl_->createVehicle(s);
    if (!id) throw Error("World::createVehicle: failed to create '" + spec.name + "' of type '" + spec.type + "' (see log)");
    return Vehicle(this, id);
}

Vehicle World::vehicle(std::uint32_t id) noexcept { return impl_->alive(id) ? Vehicle(this, id) : Vehicle(); }
Vehicle World::vehicle(std::string_view name) noexcept { return vehicle(impl_->find(std::string(name))); }

std::vector<Vehicle> World::vehicles() {
    std::vector<Vehicle> out;
    for (auto id : impl_->vehicleIds()) out.push_back(Vehicle(this, id));
    return out;
}

std::size_t World::vehicleCount() const noexcept { return impl_->vehicleCount(); }
void World::step(unsigned n) { impl_->step(n); }
double World::time() const noexcept { return impl_->simTime(); }
double World::stepSeconds() const noexcept { return impl_->dt() * impl_->frameSkip(); }
comm::Network& World::network() { return impl_->network(); }
void World::addEffectToAll(std::function<std::unique_ptr<effects::Effect>()> factory) { impl_->addEffectToAll(std::move(factory)); }
const std::string& World::name() const noexcept { return impl_->name(); }
bool World::published() const noexcept { return impl_->published(); }
std::uint64_t World::vehicleSteps() const noexcept { return impl_->vehicleSteps(); }
std::uint64_t World::worldSteps() const noexcept { return impl_->worldSteps(); }
void World::publishNow() { impl_->publishNow(); }

} // namespace fsim
