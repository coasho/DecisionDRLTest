#include "env/VecEnv.h"

#include "core/Log.h"
#include "io/AssetResolver.h"
#include "platform/Threads.h"
#include "sim/JsbsimModel.h"

#include <algorithm>
#include <stdexcept>

namespace fsim::env {

VecEnv::VecEnv(const Scenario& scenario, const Options& options)
    : scenario_(scenario), numEnvs_(std::max(1u, options.numEnvs)), vehiclesPerEnv_(std::max(1u, scenario.vehiclesPerEnv)),
      seed_(options.seed) {
    io::AssetResolver assets;
    const auto root = assets.jsbsimRoot(scenario_.jsbsimRoot);
    if (!root) throw std::runtime_error("VecEnv: JSBSim data root not found");
    scenario_.jsbsimRoot = *root;

    task_ = createTask(scenario_.task, scenario_);
    obsBuilder_ = createObservationBuilder(scenario_.observation);
    actionMapper_ = createActionMapper(scenario_.action);
    if (!task_ || !obsBuilder_ || !actionMapper_) throw std::runtime_error("VecEnv: unknown task/observation/action id");

    ground_ = options.ground ? options.ground : std::make_shared<sim::FlatGround>(0.0);

    const std::size_t n = numVehicles();
    const unsigned physical = platform::physicalCoreCount();
    const unsigned workers = std::min<unsigned>(options.workers ? options.workers : std::max(1u, physical > 2 ? physical - 2 : 1u),
                                                static_cast<unsigned>(n));
    pool_ = std::make_unique<sim::VehiclePool>(workers);

    taskStates_.resize(n);
    initialConditions_.resize(n);
    inputs_.resize(n);
    observations_.assign(n * obsBuilder_->size(), 0.0f);
    finalObs_.assign(n * obsBuilder_->size(), 0.0f);
    rewards_.assign(n, 0.0f);
    terminated_.assign(n, 0);
    truncated_.assign(n, 0);
    episodeSteps_.assign(numEnvs_, 0);
    needsReset_.assign(numEnvs_, 0);

    const sim::AircraftSpec aircraft{scenario_.aircraft, scenario_.jsbsimRoot};
    for (std::size_t i = 0; i < n; ++i) {
        auto model = std::make_unique<sim::JsbsimModel>(scenario_.dt, ground_);
        // Provisional IC; reset() applies the episode's sampled conditions.
        if (!model->load(aircraft, scenario_.initial.centre)) throw std::runtime_error("VecEnv: failed to load aircraft " + scenario_.aircraft);
        pool_->add(std::move(model));
    }
    LOG_INFO("env") << numEnvs_ << " env(s) x " << vehiclesPerEnv_ << " " << scenario_.aircraft << " on " << workers
                    << " worker(s); task " << task_->name() << ", obs " << obsBuilder_->size() << ", act " << actionMapper_->size();
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
        pool_->vehicle(i).reset(ic);
        inputs_[i] = sim::ControlInputs{};
        inputs_[i].setThrottleAll(0.6);
        inputs_[i].gearDown = 0.0;

        sim::VehicleState initial;
        pool_->vehicle(i).state(initial);
        task_->reset(v, initial, rng, taskStates_[i]);
    }
    episodeSteps_[env] = 0;
    needsReset_[env] = 0;
}

void VecEnv::buildObservations() {
    const auto states = pool_->states();
    const std::size_t o = obsBuilder_->size();
    for (std::size_t i = 0; i < states.size(); ++i) obsBuilder_->build(states[i], taskStates_[i], observations_.data() + i * o);
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
    pool_->refreshStates();
    std::fill(rewards_.begin(), rewards_.end(), 0.0f);
    std::fill(terminated_.begin(), terminated_.end(), 0);
    std::fill(truncated_.begin(), truncated_.end(), 0);
    buildObservations();
    return result();
}

VecEnv::StepResult VecEnv::step(Span<const float> actions) {
    const std::size_t n = numVehicles(), a = actionMapper_->size();
    if (actions.size() < n * a) throw std::invalid_argument("VecEnv::step: action buffer too small");

    for (std::size_t i = 0; i < n; ++i) actionMapper_->map(actions.data() + i * a, inputs_[i]);
    pool_->step(Span<const sim::ControlInputs>(inputs_), scenario_.frameSkip);
    vehicleSteps_ += n * static_cast<std::uint64_t>(scenario_.frameSkip);

    for (unsigned e = 0; e < numEnvs_; ++e) {
        if (needsReset_[e]) {
            // Gymnasium "next-step" auto-reset: the action after a terminal step
            // is ignored; the environment starts a new episode and returns its
            // first observation with zero reward and clear flags.
            resetEnv(e, false);
            for (unsigned v = 0; v < vehiclesPerEnv_; ++v) {
                const std::size_t i = static_cast<std::size_t>(e) * vehiclesPerEnv_ + v;
                pool_->refreshState(i);
                rewards_[i] = 0.0f;
                terminated_[i] = truncated_[i] = 0;
            }
            continue;
        }
        ++episodeSteps_[e];
        const auto states = pool_->states();
        bool envDone = false;
        for (unsigned v = 0; v < vehiclesPerEnv_; ++v) {
            const std::size_t i = static_cast<std::size_t>(e) * vehiclesPerEnv_ + v;
            double reward = 0.0;
            bool term = false;
            task_->evaluate(v, states[i], taskStates_[i], reward, term);
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
    for (unsigned e = 0; e < numEnvs_; ++e)
        if (needsReset_[e])
            for (unsigned v = 0; v < vehiclesPerEnv_; ++v) {
                const std::size_t i = static_cast<std::size_t>(e) * vehiclesPerEnv_ + v;
                std::copy_n(observations_.data() + i * o, o, finalObs_.data() + i * o);
            }
    return result();
}

} // namespace fsim::env
