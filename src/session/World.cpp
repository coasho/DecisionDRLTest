#include "session/World.h"

#include "core/Log.h"
#include "core/Units.h"
#include "io/AssetResolver.h"
#include "platform/Threads.h"
#include "sim/JsbsimModel.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace fsim::session {

namespace {

void copyName(char* dst, std::size_t capacity, const std::string& src) {
    std::memset(dst, 0, capacity);
    std::memcpy(dst, src.data(), std::min(src.size(), capacity - 1));
}

} // namespace

World::World(const WorldOptions& options) : options_(options), rng_(options.seed) {
    options_.frameSkip = std::max(1, options_.frameSkip);
    io::AssetResolver assets;
    if (auto root = assets.jsbsimRoot(options_.jsbsimRoot)) jsbsimRoot_ = *root;
    else LOG_WARN("session") << "JSBSim data root not found; vehicles of type jsbsim:* will fail to load";
    ground_ = options_.ground ? options_.ground : std::make_shared<sim::FlatGround>(0.0);

    const unsigned physical = platform::physicalCoreCount();
    const unsigned workers = options_.workers ? options_.workers : std::max(1u, physical > 2 ? physical - 2 : 1u);
    pool_ = std::make_unique<sim::VehiclePool>(workers, options_.pinWorkers && workers < physical);
    pool_->setPreStep([this](std::size_t slot, int subStep, sim::FlightModel& model, sim::ControlInputs& inputs) {
        preStep(slot, subStep, model, inputs);
    });

    if (options_.publish) {
        ipc::WorldPublisher::Options po;
        po.name = options_.name;
        po.capacity = options_.capacity;
        po.dt = options_.dt;
        po.frameSkip = options_.frameSkip;
        po.publishIntervalSeconds = options_.publishIntervalSeconds;
        publisher_ = std::make_unique<ipc::WorldPublisher>(po);
        publisher_->setEnvironment(environment_);
    }
    LOG_INFO("session") << "world '" << options_.name << "': dt " << options_.dt << " s, frame skip " << options_.frameSkip << ", "
                        << workers << " worker(s)" << (published() ? ", published" : "");
}

World::~World() = default;

bool World::parseType(const std::string& type, std::string& family, std::string& aircraft) {
    const auto colon = type.find(':');
    if (colon == std::string::npos) {
        family = "jsbsim";
        aircraft = type;
    } else {
        family = type.substr(0, colon);
        aircraft = type.substr(colon + 1);
    }
    return !aircraft.empty();
}

std::uint32_t World::createVehicle(const VehicleSpec& spec) {
    std::string family, aircraft;
    if (!parseType(spec.type, family, aircraft)) {
        LOG_ERROR("session") << "bad vehicle type '" << spec.type << "'";
        return 0;
    }
    if (family != "jsbsim") {
        LOG_ERROR("session") << "unknown flight model family '" << family << "' (only jsbsim in this build)";
        return 0;
    }
    if (!spec.name.empty() && find(spec.name) != 0) {
        LOG_ERROR("session") << "vehicle name '" << spec.name << "' already exists";
        return 0;
    }

    // Reuse a freed slot with the same aircraft loaded, else load a new model.
    std::size_t slot = static_cast<std::size_t>(-1);
    for (auto it = freeSlots_.begin(); it != freeSlots_.end(); ++it)
        if (slotAircraft_[*it] == aircraft) {
            slot = *it;
            freeSlots_.erase(it);
            break;
        }
    if (slot == static_cast<std::size_t>(-1)) {
        auto model = std::make_unique<sim::JsbsimModel>(options_.dt, ground_);
        if (!model->load(sim::AircraftSpec{aircraft, jsbsimRoot_}, spec.initial)) {
            LOG_ERROR("session") << "failed to load aircraft '" << aircraft << "'";
            return 0;
        }
        slot = pool_->add(std::move(model));
        entries_.emplace_back();
        slotAircraft_.push_back(aircraft);
        poolInputs_.emplace_back();
    } else {
        pool_->vehicle(slot).reset(spec.initial);
    }

    const std::uint32_t id = nextId_++;
    auto e = std::make_unique<Entry>();
    e->info.id = id;
    e->info.name = spec.name.empty() ? aircraft + "-" + std::to_string(id) : spec.name;
    e->info.type = family + ":" + aircraft;
    e->info.aircraft = aircraft;
    e->info.model = spec.model;
    e->info.initial = spec.initial;
    e->info.alive = true;
    e->info.generation = 1;
    e->slot = slot;
    e->rng = Rng::forVehicle(options_.seed, 0, id);
    e->controlDivider = std::max(1u, spec.controlDivider);
    // Neutral defaults until the first command: cruise power airborne, idle on the ground.
    e->inputs.setThrottleAll(spec.initial.onGround ? 0.0 : 0.65);
    e->inputs.gearDown = spec.initial.onGround ? 1.0 : 0.0;
    e->stack.setInitialInputs(e->inputs);
    for (auto& factory : worldEffects_) e->effects.push_back(factory());
    sim::FlightModel& model = pool_->vehicle(slot);
    model.seed(e->rng.next());
    applyEnvironment(model);
    model.state(e->working);
    e->sensed.state = e->working;
    pool_->refreshState(slot);
    pool_->setActive(slot, true);
    poolInputs_[slot] = e->inputs;

    network_.createNode(id, id);
    entries_[slot] = std::move(e);
    idToSlot_[id] = slot;
    ++liveCount_;
    publishVehicle(*entries_[slot]);
    if (publisher_ && slot >= publisher_->capacity())
        LOG_WARN("session") << "vehicle '" << entries_[slot]->info.name << "' is beyond the published capacity (" << publisher_->capacity()
                            << "); viewers will not show it";
    LOG_INFO("session") << "vehicle " << id << " '" << entries_[slot]->info.name << "' (" << entries_[slot]->info.type << ") created in slot " << slot;
    return id;
}

bool World::removeVehicle(std::uint32_t id) {
    Entry* e = entry(id);
    if (!e) return false;
    const std::size_t slot = e->slot;
    pool_->setActive(slot, false);
    network_.removeNode(id);
    if (publisher_) publisher_->clearVehicle(static_cast<std::uint32_t>(slot));
    idToSlot_.erase(id);
    entries_[slot].reset();
    freeSlots_.push_back(slot);
    --liveCount_;
    return true;
}

bool World::resetVehicle(std::uint32_t id, const sim::InitialConditions* ic) {
    Entry* e = entry(id);
    if (!e) return false;
    if (ic) e->info.initial = *ic;
    sim::FlightModel& model = pool_->vehicle(e->slot);
    if (!model.reset(e->info.initial)) return false;
    e->inputs = sim::ControlInputs{};
    e->inputs.setThrottleAll(e->info.initial.onGround ? 0.0 : 0.65);
    e->inputs.gearDown = e->info.initial.onGround ? 1.0 : 0.0;
    e->stack.setInitialInputs(e->inputs);
    e->stack.reset();
    for (auto& fx : e->effects) fx->onReset();
    poolInputs_[e->slot] = e->inputs;
    model.state(e->working);
    e->sensed.state = e->working;
    pool_->refreshState(e->slot);
    ++e->info.generation;
    publishVehicle(*e);
    return true;
}

bool World::alive(std::uint32_t id) const noexcept { return entry(id) != nullptr; }

const VehicleInfo* World::info(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->info : nullptr;
}

std::uint32_t World::find(const std::string& name) const noexcept {
    for (const auto& e : entries_)
        if (e && e->info.name == name) return e->info.id;
    return 0;
}

std::vector<std::uint32_t> World::vehicleIds() const {
    std::vector<std::uint32_t> ids;
    for (const auto& e : entries_)
        if (e) ids.push_back(e->info.id);
    std::sort(ids.begin(), ids.end());
    return ids;
}

World::Entry* World::entry(std::uint32_t id) noexcept {
    const auto it = idToSlot_.find(id);
    return it == idToSlot_.end() ? nullptr : entries_[it->second].get();
}

const World::Entry* World::entry(std::uint32_t id) const noexcept {
    const auto it = idToSlot_.find(id);
    return it == idToSlot_.end() ? nullptr : entries_[it->second].get();
}

const sim::VehicleState* World::vehicleState(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &pool_->states()[e->slot] : nullptr;
}

const effects::SensedState* World::sensedState(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->sensed : nullptr;
}

sim::FlightModel* World::model(std::uint32_t id) noexcept {
    Entry* e = entry(id);
    return e ? &pool_->vehicle(e->slot) : nullptr;
}

const sim::ControlInputs* World::inputs(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->inputs : nullptr;
}

bool World::command(std::uint32_t id, const control::Command& command) {
    Entry* e = entry(id);
    if (!e) return false;
    e->stack.command(command);
    if (publisher_) publisher_->setControlLevel(static_cast<std::uint32_t>(e->slot), static_cast<std::uint8_t>(e->stack.activeLevel()));
    return true;
}

control::ControlStack* World::controls(std::uint32_t id) noexcept {
    Entry* e = entry(id);
    return e ? &e->stack : nullptr;
}

const control::ControlStack* World::controls(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->stack : nullptr;
}

bool World::addEffect(std::uint32_t id, std::unique_ptr<effects::Effect> effect) {
    Entry* e = entry(id);
    if (!e || !effect) return false;
    e->effects.push_back(std::move(effect));
    return true;
}

void World::addEffectToAll(EffectFactory factory) {
    if (!factory) return;
    for (auto& e : entries_)
        if (e) e->effects.push_back(factory());
    worldEffects_.push_back(std::move(factory));
}

void World::clearEffects(std::uint32_t id) {
    if (Entry* e = entry(id)) e->effects.clear();
}

void World::setEnvironment(const sim::EnvironmentState& environment) {
    environment_ = environment;
    environment_.revision = appliedEnvironment_ == ~0ull ? 1 : appliedEnvironment_ + 1;
    if (publisher_) publisher_->setEnvironment(environment_);
}

void World::applyEnvironment(sim::FlightModel& model) const {
    const auto& env = environment_;
    // Meteorological convention: the direction the wind blows FROM.
    const double dir = units::degreesToRadians(env.windDirectionDeg);
    model.setWindNed(-env.windSpeedMs * std::cos(dir), -env.windSpeedMs * std::sin(dir), 0.0);
    model.setTurbulence(env.turbulence, std::max(env.windSpeedMs, 1.0));
    model.setAtmosphere(env.temperatureSeaLevelK, env.pressureSeaLevelPa);
}

void World::preStep(std::size_t slot, int subStep, sim::FlightModel& model, sim::ControlInputs& inputs) {
    Entry* e = entries_[slot].get();
    if (!e) return;
    const bool needState = !e->effects.empty() || e->stack.activeLevel() != control::Level::Actuator;
    if (needState) model.state(e->working);
    else e->working = pool_->states()[slot];
    const double t = simTime_ + subStep * options_.dt;

    // Effects: sensed state starts as truth, wind/force contributions accumulate.
    e->sensed.state = e->working;
    e->sensed.gnssValid = e->sensed.airDataValid = true;
    e->sensed.positionErrorM = 0.0;
    if (!e->effects.empty()) {
        effects::EffectContext ctx(e->info.id, e->working, e->sensed, model, environment_, e->rng, options_.dt, t);
        for (auto& fx : e->effects)
            if (fx->enabled) fx->apply(ctx);
        if (ctx.windTouched()) {
            const double dir = units::degreesToRadians(environment_.windDirectionDeg);
            const double* w = ctx.windNed();
            if (ctx.windOverride()) model.setWindNed(w[0], w[1], w[2]);
            else model.setWindNed(w[0] - environment_.windSpeedMs * std::cos(dir), w[1] - environment_.windSpeedMs * std::sin(dir), w[2]);
            e->windApplied = true;
        } else if (e->windApplied) {
            applyEnvironment(model);
            e->windApplied = false;
        }
        if (ctx.forceTouched()) {
            model.setExternalForceBody(ctx.force(), ctx.moment());
            e->forceApplied = true;
        } else if (e->forceApplied) {
            const double zero[3] = {0, 0, 0};
            model.setExternalForceBody(zero, zero);
            e->forceApplied = false;
        }
    }

    // Control cascade at the FDM rate (or every `controlDivider` steps).
    if (static_cast<unsigned>(subStep) % e->controlDivider == 0) {
        control::ControlContext ctx{e->info.id, e->working, e->sensed.state, options_.dt * e->controlDivider, this, &e->rng};
        e->stack.update(ctx, e->inputs);
    }
    inputs = e->inputs;
}

void World::step(unsigned n) {
    for (unsigned k = 0; k < n; ++k) {
        if (environment_.revision != appliedEnvironment_) {
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (entries_[s]) applyEnvironment(pool_->vehicle(s));
            appliedEnvironment_ = environment_.revision;
        }
        pool_->step(Span<const sim::ControlInputs>(poolInputs_), options_.frameSkip);
        simTime_ += options_.dt * options_.frameSkip;
        vehicleSteps_ += liveCount_ * static_cast<std::uint64_t>(options_.frameSkip);
        ++worldSteps_;
        network_.step(simTime_, options_.dt * options_.frameSkip, this, rng_);
        if (publisher_) {
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (entries_[s]) poolInputs_[s] = entries_[s]->inputs;
            publisher_->addSteps(liveCount_ * static_cast<std::uint64_t>(options_.frameSkip), 1);
            publisher_->publish(simTime_, pool_->states(), Span<const sim::ControlInputs>(poolInputs_));
        }
    }
}

void World::publishNow() {
    if (!publisher_) return;
    for (std::size_t s = 0; s < entries_.size(); ++s)
        if (entries_[s]) poolInputs_[s] = entries_[s]->inputs;
    publisher_->publish(simTime_, pool_->states(), Span<const sim::ControlInputs>(poolInputs_), true);
}

void World::publishVehicle(const Entry& e) {
    if (!publisher_) return;
    ipc::VehicleRecord r;
    r.id = e.info.id;
    r.generation = e.info.generation;
    r.alive = e.info.alive ? 1 : 0;
    r.controlLevel = static_cast<std::uint8_t>(e.stack.activeLevel());
    copyName(r.name, ipc::kNameLength, e.info.name);
    copyName(r.type, ipc::kTypeLength, e.info.type);
    copyName(r.model, ipc::kPathLength, e.info.model);
    r.initialLatitudeDeg = e.info.initial.latitudeDeg;
    r.initialLongitudeDeg = e.info.initial.longitudeDeg;
    r.initialAltitudeMslM = e.info.initial.altitudeMslM;
    r.initialHeadingDeg = e.info.initial.headingDeg;
    publisher_->setVehicle(static_cast<std::uint32_t>(e.slot), r);
    publishNow();
}

} // namespace fsim::session
