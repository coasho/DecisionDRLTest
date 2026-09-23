#include "env/VecEnv.h"

#include "core/Log.h"
#include "env/Registry.h"
#include "session/Scenario.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace fsim::env {

namespace {

/// "a, b, c" - the registered ids, for the "unknown id" message.
std::string joined(const std::vector<std::string>& ids) {
    std::string out;
    for (const auto& id : ids) {
        if (!out.empty()) out += ", ";
        out += id;
    }
    return out;
}

} // namespace

VecEnv::VecEnv(const Scenario& scenario, const Options& options)
    : scenario_(scenario), numEnvs_(std::max(1u, options.numEnvs)), vehiclesPerEnv_(std::max(1u, scenario.vehiclesPerEnv)),
      seed_(options.seed), autoReset_(options.autoReset) {
    task_ = createTask(scenario_.task, scenario_);
    obsBuilder_ = createObservationBuilder(scenario_.observation);
    actionMapper_ = createActionMapper(scenario_.action);
    const PluginRegistry& registry = PluginRegistry::instance();
    if (!task_)
        throw std::runtime_error("VecEnv: unknown task '" + scenario_.task + "'; registered: " + joined(registry.taskIds()));
    if (!obsBuilder_)
        throw std::runtime_error("VecEnv: unknown observation '" + scenario_.observation + "'; registered: " +
                                 joined(registry.observationIds()));
    if (!actionMapper_)
        throw std::runtime_error("VecEnv: unknown action '" + scenario_.action + "'; registered: " +
                                 joined(registry.actionIds()));

    session::WorldOptions wo;
    wo.name = scenario_.worldName;
    wo.dt = scenario_.dt;
    wo.frameSkip = scenario_.frameSkip;
    wo.workers = options.workers;
    wo.seed = options.seed;
    wo.publish = options.publish;
    wo.jsbsimRoot = scenario_.jsbsimRoot;
    wo.ground = options.ground;
    wo.terrain = options.terrain;
    wo.capacity = std::max<std::uint32_t>(16, static_cast<std::uint32_t>(numVehicles()));
    world_ = std::make_unique<session::World>(wo);
    if (!options.scenarioPath.empty()) session::applyScenarioWorld(*world_, loadScenario(options.scenarioPath));

    const std::size_t n = numVehicles();
    taskStates_.resize(n);
    initialConditions_.resize(n);
    observations_.assign(n * obsBuilder_->size(), 0.0f);
    finalObs_.assign(n * obsBuilder_->size(), 0.0f);
    rewards_.assign(n, 0.0f);
    terminated_.assign(n, 0);
    truncated_.assign(n, 0);
    episodeSteps_.assign(numEnvs_, 0);
    needsReset_.assign(numEnvs_, 0);

    ids_.reserve(n);
    for (unsigned e = 0; e < numEnvs_; ++e)
        for (unsigned v = 0; v < vehiclesPerEnv_; ++v) {
            session::VehicleSpec spec;
            spec.name = "env" + std::to_string(e) + "/" + std::to_string(v);
            spec.type = "jsbsim:" + scenario_.aircraft;
            spec.initial = scenario_.initial.centre; // provisional; reset() applies the episode's sampled conditions
            const std::uint32_t id = world_->createVehicle(spec);
            if (!id) throw std::runtime_error("VecEnv: failed to load aircraft " + scenario_.aircraft);
            ids_.push_back(id);
        }
    LOG_INFO("env") << numEnvs_ << " env(s) x " << vehiclesPerEnv_ << " " << scenario_.aircraft << "; task " << task_->name() << ", obs "
                    << obsBuilder_->size() << ", act " << actionMapper_->size() << " (" << control::levelName(actionMapper_->level()) << ")";
}

VecEnv::~VecEnv() = default;

void VecEnv::resetEnv(unsigned env, bool) {
    const std::uint64_t episode = episodeCounter_++;
    for (unsigned v = 0; v < vehiclesPerEnv_; ++v) {
        const std::size_t i = static_cast<std::size_t>(env) * vehiclesPerEnv_ + v;
        Rng rng = Rng::forVehicle(seed_ ^ (episode * 0x9E3779B97F4A7C15ull), env, v);
        const auto& r = scenario_.initial;
        sim::InitialConditions ic = r.centre;
        ic.latitudeDeg += rng.uniform(-r.latitudeJitterDeg, r.latitudeJitterDeg);
        ic.longitudeDeg += rng.uniform(-r.longitudeJitterDeg, r.longitudeJitterDeg);
        ic.altitudeMslM += rng.uniform(-r.altitudeJitterM, r.altitudeJitterM);
        ic.headingDeg += rng.uniform(-r.headingJitterDeg, r.headingJitterDeg);
        ic.airspeedTrueMs += rng.uniform(-r.airspeedJitterMs, r.airspeedJitterMs);
        initialConditions_[i] = ic;
        world_->resetVehicle(ids_[i], &ic);
        // Neutral command until the first action: cruise power, gear up.
        control::ActuatorCommand neutral;
        neutral.throttle = 0.6;
        neutral.gearDown = 0.0;
        world_->command(ids_[i], neutral);
        // Cleared first so episodes are independent even when a task keeps
        // its own values in TaskState::custom.
        taskStates_[i] = TaskState{};
        task_->reset(v, *world_->vehicleState(ids_[i]), rng, taskStates_[i]);
    }
    episodeSteps_[env] = 0;
    needsReset_[env] = 0;
}

void VecEnv::buildObservations() {
    const std::size_t o = obsBuilder_->size();
    for (std::size_t i = 0; i < ids_.size(); ++i) obsBuilder_->build(state(i), taskStates_[i], observations_.data() + i * o);
}

VecEnv::StepResult VecEnv::result() const noexcept {
    return StepResult{Span<const float>(observations_.data(), observations_.size()),
                      Span<const float>(rewards_.data(), rewards_.size()),
                      Span<const std::uint8_t>(terminated_.data(), terminated_.size()),
                      Span<const std::uint8_t>(truncated_.data(), truncated_.size()),
                      Span<const std::uint32_t>(episodeSteps_.data(), episodeSteps_.size())};
}

VecEnv::StepResult VecEnv::reset(std::uint64_t seed) {
    if (seed) seed_ = seed;
    episodeCounter_ = 0; // reset(seed) is reproducible: same seed, same episodes
    for (unsigned e = 0; e < numEnvs_; ++e) resetEnv(e, true);
    std::fill(rewards_.begin(), rewards_.end(), 0.0f);
    std::fill(terminated_.begin(), terminated_.end(), 0);
    std::fill(truncated_.begin(), truncated_.end(), 0);
    buildObservations();
    return result();
}

VecEnv::StepResult VecEnv::step(Span<const float> actions) {
    const std::size_t n = numVehicles(), a = actionMapper_->size();
    if (actions.size() < n * a) throw std::invalid_argument("VecEnv::step: action buffer too small");

    for (std::size_t i = 0; i < n; ++i) world_->command(ids_[i], actionMapper_->map(actions.data() + i * a));
    world_->step(1);
    vehicleSteps_ += n * static_cast<std::uint64_t>(scenario_.frameSkip);

    for (unsigned e = 0; e < numEnvs_; ++e) {
        if (needsReset_[e]) {
            // Gymnasium "next-step" auto-reset: the action after a terminal step
            // is ignored; the environment starts a new episode and returns its
            // first observation with zero reward and clear flags.
            resetEnv(e, false);
            for (unsigned v = 0; v < vehiclesPerEnv_; ++v) {
                const std::size_t i = static_cast<std::size_t>(e) * vehiclesPerEnv_ + v;
                rewards_[i] = 0.0f;
                terminated_[i] = truncated_[i] = 0;
            }
            continue;
        }
        ++episodeSteps_[e];
        bool envDone = false;
        for (unsigned v = 0; v < vehiclesPerEnv_; ++v) {
            const std::size_t i = static_cast<std::size_t>(e) * vehiclesPerEnv_ + v;
            double reward = 0.0;
            bool term = false;
            task_->evaluate(v, state(i), taskStates_[i], reward, term);
            rewards_[i] = static_cast<float>(reward);
            terminated_[i] = term ? 1 : 0;
            truncated_[i] = (!term && episodeSteps_[e] >= scenario_.maxEpisodeSteps) ? 1 : 0;
            envDone = envDone || term || truncated_[i];
        }
        if (envDone) needsReset_[e] = 1;
    }

    buildObservations();
    // Keep the terminal observations of environments that just finished.
    const std::size_t o = obsBuilder_->size();
    for (unsigned e = 0; e < numEnvs_; ++e) {
        if (!needsReset_[e]) continue;
        for (unsigned v = 0; v < vehiclesPerEnv_; ++v) {
            const std::size_t i = static_cast<std::size_t>(e) * vehiclesPerEnv_ + v;
            std::copy_n(observations_.data() + i * o, o, finalObs_.data() + i * o);
        }
        if (autoReset_ != AutoReset::SameStep) continue;
        // Start the next episode now and hand back its first observation; the
        // reward and flags stay those of the step that ended the last one.
        resetEnv(e, false);
        for (unsigned v = 0; v < vehiclesPerEnv_; ++v) {
            const std::size_t i = static_cast<std::size_t>(e) * vehiclesPerEnv_ + v;
            obsBuilder_->build(state(i), taskStates_[i], observations_.data() + i * o);
        }
    }
    return result();
}

} // namespace fsim::env
