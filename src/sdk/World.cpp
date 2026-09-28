// fsim SDK: World / Vehicle / Environment over session::World.

#include "fsim/World.h"

#include "core/Log.h"
#include "core/Time.h"
#include "session/World.h"

#include <chrono>
#include <ctime>
#include <limits>

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

control::CommandResult Vehicle::submit(const control::Command& command, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, command, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::SupportCommand& command, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, command, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::RouteCommand& route, Span<const control::Waypoint> waypoints, const control::CommandOptions& options,
                                       Span<const control::RouteLoiter> loiters, Span<const control::RouteState> states) {
    if (world_) return world_->impl_->submit(id_, route, waypoints, options, loiters, states);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                       const control::CommandOptions& options, const control::CurveShape* shape) {
    if (world_) return world_->impl_->submit(id_, curve, segments, options, shape);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                       const control::CommandOptions& options, const control::CurveShape* shape) {
    if (world_) return world_->impl_->submit(id_, curve, segments, options, shape);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::PatternCommand& pattern, const control::PatternShape& shape, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, pattern, shape, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

std::vector<control::CommandResult> Vehicle::submitBatch(Span<const control::BatchCommand> batch, std::vector<control::CommandDetails>* details) {
    if (world_) return world_->impl_->submitBatch(id_, batch, details);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return std::vector<control::CommandResult>(batch.size(), r);
}

control::CommandDetails Vehicle::commandDetails() const {
    const control::CommandDetails* d = world_ ? world_->impl_->commandDetails(id_) : nullptr;
    return d ? *d : control::CommandDetails{};
}

std::vector<control::ActivityRecord> Vehicle::activities() const {
    return world_ ? world_->impl_->activities(id_) : std::vector<control::ActivityRecord>{};
}

control::VehicleCommandState Vehicle::commanded() const { return world_ ? world_->impl_->commandState(id_) : control::VehicleCommandState{}; }

control::NavigationReport Vehicle::navigationReport() const {
    return world_ ? world_->impl_->navigationReport(id_) : control::NavigationReport{};
}

control::Reason Vehicle::setNavigation(const control::NavigationSettings& settings) {
    return world_ ? world_->impl_->setNavigation(id_, settings) : control::Reason::UnknownVehicle;
}

control::NavigationSettings Vehicle::navigation() const {
    return world_ ? world_->impl_->navigation(id_) : control::NavigationSettings{};
}

control::Reason Vehicle::setQnh(double qnhPa) { return world_ ? world_->impl_->setQnh(id_, qnhPa) : control::Reason::UnknownVehicle; }

double Vehicle::qnh() const { return world_ ? world_->impl_->qnh(id_) : std::numeric_limits<double>::quiet_NaN(); }

control::StateData Vehicle::stateData() const { return world_ ? world_->impl_->stateData(id_) : control::StateData{}; }

control::Reason Vehicle::performanceProfile(control::FlightMode mode, control::PerformanceProfile& out) const {
    return world_ ? world_->impl_->performanceProfile(id_, mode, out) : control::Reason::UnknownVehicle;
}

std::vector<control::CapabilityDescriptor> Vehicle::capabilities() const {
    return world_ ? world_->impl_->capabilities(id_) : std::vector<control::CapabilityDescriptor>{};
}

const control::VehicleProfile& Vehicle::profile() const {
    static const control::VehicleProfile none;
    const auto* p = world_ ? world_->impl_->profile(id_) : nullptr;
    return p ? *p : none;
}

control::Reason Vehicle::setVehicleDefault(control::VehicleDefault mode) {
    return world_ ? world_->impl_->setVehicleDefault(id_, mode) : control::Reason::UnknownVehicle;
}

control::VehicleDefault Vehicle::vehicleDefault() const {
    return world_ ? world_->impl_->vehicleDefault(id_) : control::VehicleDefault::Neutral;
}

control::Reason Vehicle::setProtection(control::ProtectionMode mode) {
    return world_ ? world_->impl_->setProtection(id_, mode) : control::Reason::UnknownVehicle;
}

control::ProtectionMode Vehicle::protection() const {
    return world_ ? world_->impl_->protection(id_) : control::ProtectionMode::Off;
}

control::EnvelopeStatus Vehicle::envelope() { return world_ ? world_->impl_->envelope(id_) : control::EnvelopeStatus{}; }

control::CapabilityStatus Vehicle::capabilityStatus(std::string_view capability) const {
    if (world_) return world_->impl_->capabilityStatus(id_, capability);
    control::CapabilityStatus none;
    none.availability = control::Availability::Unavailable;
    none.reason = control::Reason::UnknownVehicle;
    none.reasons = control::reasonBit(none.reason);
    none.description = control::reasonDescription(none.reason);
    return none;
}

const control::SupportInfo* Vehicle::support(std::string_view feature) const {
    return world_ ? world_->impl_->support(id_, feature) : nullptr;
}

std::vector<control::SupportInfo> Vehicle::supportTable() const {
    const control::SupportTable* t = world_ ? world_->impl_->supportTable(id_) : nullptr;
    return t ? t->rows() : std::vector<control::SupportInfo>{};
}

control::Performance Vehicle::performance() const {
    const control::Performance* p = world_ ? world_->impl_->performance(id_) : nullptr;
    return p ? *p : control::Performance{};
}

std::uint32_t Vehicle::controlRevision() const { return world_ ? world_->impl_->controlRevision(id_) : 0; }

control::Reason Vehicle::setControlMode(control::ControlMode mode) {
    return world_ ? world_->impl_->setControlMode(id_, mode) : control::Reason::UnknownVehicle;
}

control::ControlMode Vehicle::controlMode() const { return world_ ? world_->impl_->controlMode(id_) : control::ControlMode::Open; }

control::Reason Vehicle::requestControl(std::string_view capability, control::ControllerId controller) {
    return world_ ? world_->impl_->requestControl(id_, capability, controller) : control::Reason::UnknownVehicle;
}

control::Reason Vehicle::releaseControl(std::string_view capability, control::ControllerId controller) {
    return world_ ? world_->impl_->releaseControl(id_, capability, controller) : control::Reason::UnknownVehicle;
}

control::Reason Vehicle::revokeControl(std::string_view capability, control::Reason reason) {
    return world_ ? world_->impl_->revokeControl(id_, capability, reason) : control::Reason::UnknownVehicle;
}

control::Reason Vehicle::setAllowed(std::string_view capability, bool allowed) {
    return world_ ? world_->impl_->setAllowed(id_, capability, allowed) : control::Reason::UnknownVehicle;
}

control::ControlStatus Vehicle::controlStatus(std::string_view capability) const {
    return world_ ? world_->impl_->controlStatus(id_, capability) : control::ControlStatus{false, false};
}

control::Reason Vehicle::storeTask(control::TaskId task, const control::Command& command, Span<const control::Waypoint> waypoints,
                                   Span<const control::BezierSegment> segments, control::TaskRepetition repetition, const control::PatternShape* shape) {
    return world_ ? world_->impl_->storeTask(id_, task, command, waypoints, segments, repetition, shape) : control::Reason::UnknownVehicle;
}

control::Reason Vehicle::storeTask(control::TaskId task, const control::BatchCommand& command, control::TaskRepetition repetition) {
    return world_ ? world_->impl_->storeTask(id_, task, command, repetition) : control::Reason::UnknownVehicle;
}

control::CommandResult Vehicle::commandTask(control::TaskId task, const control::CommandOptions& options) {
    if (!world_) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return world_->impl_->commandTask(id_, task, options);
}

control::CommandResult Vehicle::cancelTask(control::TaskId task, control::Caller caller) {
    if (!world_) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return world_->impl_->cancelTask(id_, task, caller);
}

control::Reason Vehicle::removeTask(control::TaskId task) { return world_ ? world_->impl_->removeTask(id_, task) : control::Reason::UnknownVehicle; }

std::optional<control::TaskStatus> Vehicle::taskStatus(control::TaskId task) const {
    return world_ ? world_->impl_->taskStatus(id_, task) : std::nullopt;
}

std::vector<control::TaskStatus> Vehicle::tasks() const { return world_ ? world_->impl_->tasks(id_) : std::vector<control::TaskStatus>{}; }

control::Reason Vehicle::setCapabilityPrecedence(std::string_view capability, std::uint32_t precedence) {
    return world_ ? world_->impl_->setCapabilityPrecedence(id_, capability, precedence) : control::Reason::UnknownVehicle;
}

std::uint32_t Vehicle::capabilityPrecedence(std::string_view capability) const {
    return world_ ? world_->impl_->capabilityPrecedence(id_, capability) : 0;
}

control::Reason Vehicle::setAvailability(std::string_view capability, control::Availability availability, control::Reason reason,
                                         std::uint64_t associated, double nextAvailableS) {
    return world_ ? world_->impl_->setAvailability(id_, capability, availability, reason, associated, nextAvailableS)
                  : control::Reason::UnknownVehicle;
}

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
    s.profile = spec.profile;
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
control::CommandResult World::update(control::ActivityId activity, const control::Command& setpoint) {
    return impl_->update(activity, setpoint);
}

control::CommandResult World::update(control::ActivityId activity, const control::SupportCommand& setpoint) {
    return impl_->update(activity, setpoint);
}

control::CommandResult World::update(control::ActivityId activity, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                     Span<const control::RouteLoiter> loiters, Span<const control::RouteState> states) {
    return impl_->update(activity, route, waypoints, loiters, states);
}

control::CommandResult World::update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                     const control::CurveShape* shape) {
    return impl_->update(activity, curve, segments, shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                     const control::CurveShape* shape) {
    return impl_->update(activity, curve, segments, shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::PatternCommand& pattern, const control::PatternShape& shape) {
    return impl_->update(activity, pattern, shape);
}

control::CommandResult World::cancel(control::ActivityId activity) { return impl_->cancel(activity); }

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::Command& setpoint) {
    return impl_->update(caller, activity, setpoint);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::SupportCommand& setpoint) {
    return impl_->update(caller, activity, setpoint);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::RouteCommand& route,
                                     Span<const control::Waypoint> waypoints, Span<const control::RouteLoiter> loiters,
                                     Span<const control::RouteState> states) {
    return impl_->update(caller, activity, route, waypoints, loiters, states);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                     Span<const control::BezierSegment> segments, const control::CurveShape* shape) {
    return impl_->update(caller, activity, curve, segments, shape);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                     Span<const control::NurbsSegment> segments, const control::CurveShape* shape) {
    return impl_->update(caller, activity, curve, segments, shape);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::PatternCommand& pattern,
                                     const control::PatternShape& shape) {
    return impl_->update(caller, activity, pattern, shape);
}

control::CommandResult World::cancel(control::Caller caller, control::ActivityId activity) { return impl_->cancel(caller, activity); }

control::CommandResult World::activityCommand(control::ActivityId activity, control::ActivityCommand command, control::Rank rank,
                                              control::Caller caller) {
    return impl_->activityCommand(caller, activity, command, rank);
}

std::optional<control::ActivityRecord> World::activity(control::ActivityId activity) const {
    if (const auto* a = impl_->activity(activity)) return *a;
    return std::nullopt;
}

std::optional<control::Setpoint> World::activitySetpoint(control::ActivityId activity) const {
    control::Setpoint s;
    if (!impl_->activitySetpoint(activity, s)) return std::nullopt;
    return s;
}

std::vector<control::EndPoint> World::endPoints(control::ActivityId activity, std::size_t max) const { return impl_->endPoints(activity, max); }

void World::step(unsigned n) { impl_->step(n); }
double World::time() const noexcept { return impl_->simTime(); }
double World::stepSeconds() const noexcept { return impl_->dt() * impl_->frameSkip(); }
comm::Network& World::network() { return impl_->network(); }

std::optional<double> World::terrainHeightM(double latitudeRad, double longitudeRad) const { return impl_->terrainHeightM(latitudeRad, longitudeRad); }

control::FrameId World::createFrame(const control::FrameSpec& spec) { return impl_->createFrame(spec); }

bool World::removeFrame(control::FrameId id) { return impl_->removeFrame(id); }

std::optional<control::FrameSpec> World::frame(control::FrameId id) const { return impl_->frame(id); }

std::optional<control::FramePose> World::framePose(control::FrameId id, double timeS) const { return impl_->framePose(id, timeS); }

std::optional<control::GeoPoint> World::framePoint(control::FrameId id, const control::FrameOffset& offset, double timeS) const {
    return impl_->framePoint(id, offset, timeS);
}
void World::addEffectToAll(std::function<std::unique_ptr<effects::Effect>()> factory) { impl_->addEffectToAll(std::move(factory)); }
const std::string& World::name() const noexcept { return impl_->name(); }
bool World::published() const noexcept { return impl_->published(); }
std::uint64_t World::vehicleSteps() const noexcept { return impl_->vehicleSteps(); }
std::uint64_t World::worldSteps() const noexcept { return impl_->worldSteps(); }
void World::publishNow() { impl_->publishNow(); }

} // namespace fsim
