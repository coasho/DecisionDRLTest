// fsim.dll: C ABI + C++ SDK over env::VecEnv (design 4.4, 9.1, 9.2).

#include "fsim/VecEnv.h"
#include "fsim/World.h"
#include "fsim/fsim_c.h"

#include "core/Log.h"
#include "sdk/Handles.h"
#include "sdk/LastError.h"
#include "env/Registry.h"
#include "env/VecEnv.h"

#include <cstring>
#include <exception>
#include <functional>
#include <stdexcept>
#include <string>

#ifndef FSIM_VERSION_STRING
#    define FSIM_VERSION_STRING "0.0.0"
#endif

// ---------------------------------------------------------------------------
// Shared implementation: a C handle *is* the environment object.
// ---------------------------------------------------------------------------
struct fsim_vecenv {
    fsim::env::VecEnv env;
    fsim::env::VecEnv::StepResult last{};
    std::unique_ptr<fsim_world> world; ///< borrowed handle for fsim_vecenv_world(), made on demand
    explicit fsim_vecenv(const fsim::env::Scenario& s, const fsim::env::VecEnv::Options& o) : env(s, o) {}
};

namespace fsim::sdk {
std::string& lastError() noexcept {
    thread_local std::string t_lastError;
    return t_lastError;
}
} // namespace fsim::sdk

namespace {

void setError(const std::string& message) {
    fsim::sdk::lastError() = message;
    LOG_ERROR("sdk") << message;
}

int guard(const char* what, const std::function<void()>& fn) noexcept {
    try {
        fn();
        fsim::sdk::lastError().clear();
        return FSIM_OK;
    } catch (const std::invalid_argument& e) {
        setError(std::string(what) + ": " + e.what());
        return FSIM_INVALID_ARGUMENT;
    } catch (const std::exception& e) {
        setError(std::string(what) + ": " + e.what());
        return FSIM_ERROR;
    } catch (...) {
        setError(std::string(what) + ": unknown error");
        return FSIM_ERROR;
    }
}

fsim::env::Scenario toScenario(const fsim_options& o) {
    fsim::env::Scenario s;
    if (o.aircraft) s.aircraft = o.aircraft;
    if (o.jsbsim_root) s.jsbsimRoot = o.jsbsim_root;
    s.vehiclesPerEnv = o.vehicles_per_env;
    s.dt = o.dt;
    s.frameSkip = o.frame_skip;
    s.maxEpisodeSteps = o.max_episode_steps;
    if (o.task) s.task = o.task;
    if (o.observation) s.observation = o.observation;
    if (o.action) s.action = o.action;
    auto& ic = s.initial;
    ic.centre.latitudeDeg = o.latitude_deg;
    ic.centre.longitudeDeg = o.longitude_deg;
    ic.centre.altitudeMslM = o.altitude_m;
    ic.centre.headingDeg = o.heading_deg;
    ic.centre.airspeedTrueMs = o.airspeed_ms;
    ic.latitudeJitterDeg = o.latitude_jitter_deg;
    ic.longitudeJitterDeg = o.longitude_jitter_deg;
    ic.altitudeJitterM = o.altitude_jitter_m;
    ic.headingJitterDeg = o.heading_jitter_deg;
    ic.airspeedJitterMs = o.airspeed_jitter_ms;
    s.targetAltitudeDeltaM = o.target_altitude_delta_m;
    s.targetHeadingDeltaDeg = o.target_heading_delta_deg;
    if (o.world_name) s.worldName = o.world_name;
    return s;
}

fsim_options toOptions(const fsim::VecEnvOptions& o) {
    fsim_options c;
    fsim_options_init(&c);
    c.num_envs = o.numEnvs;
    c.vehicles_per_env = o.vehiclesPerEnv;
    c.workers = o.workers;
    c.seed = o.seed;
    c.aircraft = o.aircraft.c_str();
    c.jsbsim_root = o.jsbsimRoot.empty() ? nullptr : o.jsbsimRoot.c_str();
    c.task = o.task.c_str();
    c.observation = o.observation.c_str();
    c.action = o.action.c_str();
    c.dt = o.dt;
    c.frame_skip = o.frameSkip;
    c.max_episode_steps = o.maxEpisodeSteps;
    c.latitude_deg = o.latitudeDeg; c.longitude_deg = o.longitudeDeg; c.altitude_m = o.altitudeM;
    c.heading_deg = o.headingDeg; c.airspeed_ms = o.airspeedMs;
    c.latitude_jitter_deg = o.latitudeJitterDeg; c.longitude_jitter_deg = o.longitudeJitterDeg;
    c.altitude_jitter_m = o.altitudeJitterM; c.heading_jitter_deg = o.headingJitterDeg; c.airspeed_jitter_ms = o.airspeedJitterMs;
    c.target_altitude_delta_m = o.targetAltitudeDeltaM; c.target_heading_delta_deg = o.targetHeadingDeltaDeg;
    c.world_name = o.worldName.c_str();
    c.publish = o.publish ? 1 : 0;
    c.terrain = o.terrain ? 1 : 0;
    c.scenario_path = o.scenarioPath.empty() ? nullptr : o.scenarioPath.c_str();
    return c;
}

template <typename T>
fsim::ConstSpan<T> span(fsim::Span<const T> s) { return fsim::ConstSpan<T>{s.data(), s.size()}; }

fsim::StepResult toResult(const fsim_vecenv& h) {
    fsim::StepResult r;
    r.observations = span(h.last.observations);
    r.rewards = span(h.last.rewards);
    r.terminated = span(h.last.terminated);
    r.truncated = span(h.last.truncated);
    r.finalObservations = span(h.env.finalObservations());
    r.episodeSteps = span(h.last.episodeSteps);
    return r;
}

} // namespace

// ---------------------------------------------------------------------------
// C ABI
// ---------------------------------------------------------------------------
extern "C" {

FSIM_API uint32_t fsim_abi_version(void) { return FSIM_ABI_VERSION; }
FSIM_API const char* fsim_version(void) { return FSIM_VERSION_STRING; }
FSIM_API const char* fsim_last_error(void) { return fsim::sdk::lastError().c_str(); }

FSIM_API void fsim_set_log_level(int level) {
    if (level < FSIM_LOG_TRACE) level = FSIM_LOG_TRACE;
    if (level > FSIM_LOG_OFF) level = FSIM_LOG_OFF;
    fsim::log::setLevel(static_cast<fsim::log::Level>(level));
}

FSIM_API int fsim_log_level(void) { return static_cast<int>(fsim::log::level()); }

FSIM_API void fsim_options_init(fsim_options* o) {
    if (!o) return;
    std::memset(o, 0, sizeof(*o));
    o->struct_size = sizeof(fsim_options);
    o->num_envs = 1;
    o->vehicles_per_env = 1;
    o->aircraft = "c172x";
    o->task = "altitude_heading_hold";
    o->observation = "state";
    o->action = "surfaces";
    o->dt = 1.0 / 120.0;
    o->frame_skip = 4;
    o->max_episode_steps = 2000;
    o->latitude_deg = 37.6188;
    o->longitude_deg = -122.375;
    o->altitude_m = 1500.0;
    o->airspeed_ms = 60.0;
    o->latitude_jitter_deg = 0.02;
    o->longitude_jitter_deg = 0.02;
    o->altitude_jitter_m = 150.0;
    o->heading_jitter_deg = 180.0;
    o->airspeed_jitter_ms = 5.0;
    o->target_altitude_delta_m = 300.0;
    o->target_heading_delta_deg = 60.0;
    o->world_name = "vecenv";
    o->publish = 1;
}

FSIM_API int fsim_vecenv_create(const fsim_options* options, fsim_vecenv** out) {
    if (!options || !out || options->struct_size < sizeof(fsim_options)) {
        setError("fsim_vecenv_create: bad arguments (call fsim_options_init first)");
        return FSIM_INVALID_ARGUMENT;
    }
    *out = nullptr;
    return guard("fsim_vecenv_create", [&] {
        fsim::env::VecEnv::Options o;
        o.numEnvs = options->num_envs;
        o.seed = options->seed;
        o.workers = options->workers;
        o.publish = options->publish != 0;
        o.terrain = options->terrain != 0;
        if (options->scenario_path) o.scenarioPath = options->scenario_path;
        auto* h = new fsim_vecenv(toScenario(*options), o);
        h->last = h->env.reset();
        *out = h;
    });
}

FSIM_API void fsim_vecenv_destroy(fsim_vecenv* env) { delete env; }

FSIM_API int fsim_vecenv_reset(fsim_vecenv* env, uint64_t seed) {
    if (!env) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vecenv_reset", [&] { env->last = env->env.reset(seed); });
}

FSIM_API int fsim_vecenv_step(fsim_vecenv* env, const float* actions, size_t count) {
    if (!env || !actions) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vecenv_step", [&] { env->last = env->env.step(fsim::Span<const float>(actions, count)); });
}

FSIM_API int fsim_vecenv_buffers(const fsim_vecenv* env, fsim_buffers* out) {
    if (!env || !out || out->struct_size < sizeof(fsim_buffers)) return FSIM_INVALID_ARGUMENT;
    out->num_envs = env->env.numEnvs();
    out->vehicles_per_env = env->env.vehiclesPerEnv();
    out->observation_size = static_cast<uint32_t>(env->env.observationSize());
    out->action_size = static_cast<uint32_t>(env->env.actionSize());
    out->observations = env->last.observations.data();
    out->rewards = env->last.rewards.data();
    out->terminated = env->last.terminated.data();
    out->truncated = env->last.truncated.data();
    out->final_observations = env->env.finalObservations().data();
    out->episode_steps = env->last.episodeSteps.data();
    out->agent_step_seconds = env->env.agentStepSeconds();
    return FSIM_OK;
}

FSIM_API const char* fsim_vecenv_observation_name(const fsim_vecenv* env, uint32_t i) {
    if (!env || i >= env->env.observationNames().size()) return "";
    return env->env.observationNames()[i].c_str();
}

FSIM_API const char* fsim_vecenv_action_name(const fsim_vecenv* env, uint32_t i) {
    if (!env || i >= env->env.actionNames().size()) return "";
    return env->env.actionNames()[i].c_str();
}

FSIM_API uint64_t fsim_vecenv_vehicle_steps(const fsim_vecenv* env) { return env ? env->env.vehicleSteps() : 0; }

FSIM_API int fsim_vecenv_set_autoreset(fsim_vecenv* env, int mode) {
    if (!env || (mode != FSIM_AUTORESET_NEXT_STEP && mode != FSIM_AUTORESET_SAME_STEP)) {
        setError("fsim_vecenv_set_autoreset: mode must be FSIM_AUTORESET_NEXT_STEP or FSIM_AUTORESET_SAME_STEP");
        return FSIM_INVALID_ARGUMENT;
    }
    env->env.setAutoReset(mode == FSIM_AUTORESET_SAME_STEP ? fsim::env::VecEnv::AutoReset::SameStep
                                                           : fsim::env::VecEnv::AutoReset::NextStep);
    return FSIM_OK;
}

FSIM_API int fsim_vecenv_autoreset(const fsim_vecenv* env) {
    return env && env->env.autoReset() == fsim::env::VecEnv::AutoReset::SameStep ? FSIM_AUTORESET_SAME_STEP
                                                                                 : FSIM_AUTORESET_NEXT_STEP;
}

FSIM_API uint32_t fsim_vecenv_vehicle_ids(const fsim_vecenv* env, uint32_t* ids, uint32_t capacity) {
    if (!env) return 0;
    const std::size_t n = env->env.numVehicles();
    if (ids)
        for (std::size_t i = 0; i < n && i < capacity; ++i) ids[i] = env->env.vehicleId(i);
    return static_cast<uint32_t>(n);
}

FSIM_API const char* fsim_registered_id(int registry, uint32_t index) {
    // Registration is process-wide and may change between calls; the list is
    // taken fresh each time and kept per thread so the pointer outlives it.
    thread_local std::vector<std::string> ids;
    const auto& r = fsim::env::PluginRegistry::instance();
    switch (registry) {
    case FSIM_REGISTRY_TASK: ids = r.taskIds(); break;
    case FSIM_REGISTRY_OBSERVATION: ids = r.observationIds(); break;
    case FSIM_REGISTRY_ACTION: ids = r.actionIds(); break;
    default: return "";
    }
    return index < ids.size() ? ids[index].c_str() : "";
}

} // extern "C"

// ---------------------------------------------------------------------------
// C++ SDK
// ---------------------------------------------------------------------------
namespace fsim {

struct VecEnv::Impl {
    fsim_vecenv* handle = nullptr;
    ~Impl() { fsim_vecenv_destroy(handle); }
};

VecEnv::VecEnv(const VecEnvOptions& options) : impl_(std::make_unique<Impl>()) {
    const fsim_options c = toOptions(options);
    if (fsim_vecenv_create(&c, &impl_->handle) != FSIM_OK) throw std::runtime_error(fsim_last_error());
    fsim_vecenv_set_autoreset(impl_->handle, options.autoReset == AutoReset::SameStep ? FSIM_AUTORESET_SAME_STEP
                                                                                       : FSIM_AUTORESET_NEXT_STEP);
}

VecEnv::~VecEnv() = default;

StepResult VecEnv::reset(std::uint64_t seed) {
    if (fsim_vecenv_reset(impl_->handle, seed) != FSIM_OK) throw std::runtime_error(fsim_last_error());
    return toResult(*impl_->handle);
}

StepResult VecEnv::step(const float* actions, std::size_t count) {
    if (fsim_vecenv_step(impl_->handle, actions, count) != FSIM_OK) throw std::runtime_error(fsim_last_error());
    return toResult(*impl_->handle);
}

unsigned VecEnv::numEnvs() const noexcept { return impl_->handle->env.numEnvs(); }
unsigned VecEnv::vehiclesPerEnv() const noexcept { return impl_->handle->env.vehiclesPerEnv(); }
std::size_t VecEnv::numVehicles() const noexcept { return impl_->handle->env.numVehicles(); }
std::size_t VecEnv::observationSize() const noexcept { return impl_->handle->env.observationSize(); }
std::size_t VecEnv::actionSize() const noexcept { return impl_->handle->env.actionSize(); }
const std::vector<std::string>& VecEnv::observationNames() const noexcept { return impl_->handle->env.observationNames(); }
const std::vector<std::string>& VecEnv::actionNames() const noexcept { return impl_->handle->env.actionNames(); }
double VecEnv::agentStepSeconds() const noexcept { return impl_->handle->env.agentStepSeconds(); }
std::uint64_t VecEnv::vehicleSteps() const noexcept { return impl_->handle->env.vehicleSteps(); }
fsim_vecenv* VecEnv::handle() noexcept { return impl_->handle; }

World& VecEnv::world() { return *fsim_world_object(fsim_vecenv_world(impl_->handle)); }

extern "C" FSIM_API fsim_world* fsim_vecenv_world(fsim_vecenv* env) {
    if (!env) return nullptr;
    if (!env->world) env->world = std::make_unique<fsim_world>(env->env.world());
    return env->world.get();
}

const char* version() noexcept { return FSIM_VERSION_STRING; }

} // namespace fsim
