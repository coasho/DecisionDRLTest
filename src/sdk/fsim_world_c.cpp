// C ABI for the World / vehicle object model (design 9.8) over session::World.

#include "fsim/BuiltinEffects.h"
#include "fsim/Comm.h"
#include "fsim/fsim_c.h"

#include "core/Log.h"
#include "session/World.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

// The C state struct must mirror fsim::VehicleState exactly.
static_assert(sizeof(fsim_vehicle_state) == sizeof(fsim::sim::VehicleState), "fsim_vehicle_state layout differs from VehicleState");
static_assert(offsetof(fsim_vehicle_state, rotation_body_to_ecef) == offsetof(fsim::sim::VehicleState, rotationBodyToEcef), "layout");
static_assert(offsetof(fsim_vehicle_state, engine_count) == offsetof(fsim::sim::VehicleState, engineCount), "layout");
static_assert(offsetof(fsim_vehicle_state, on_ground) == offsetof(fsim::sim::VehicleState, onGround), "layout");
static_assert(sizeof(bool) == 1, "bool must be one byte for the C mirror of VehicleState");

struct fsim_world {
    fsim::session::World world;
    explicit fsim_world(const fsim::session::WorldOptions& o) : world(o) {}
};

namespace {

thread_local std::string t_error;

int fail(int code, const std::string& message) noexcept {
    t_error = message;
    LOG_ERROR("sdk") << message;
    return code;
}

int guard(const char* what, const std::function<int()>& fn) noexcept {
    try {
        const int r = fn();
        if (r == FSIM_OK) t_error.clear();
        return r;
    } catch (const std::invalid_argument& e) {
        return fail(FSIM_INVALID_ARGUMENT, std::string(what) + ": " + e.what());
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string(what) + ": " + e.what());
    } catch (...) {
        return fail(FSIM_ERROR, std::string(what) + ": unknown error");
    }
}

std::vector<std::pair<std::string, double>> params(const char* const* names, const double* values, uint32_t count) {
    std::vector<std::pair<std::string, double>> out;
    if (!names || !values) return out;
    for (uint32_t i = 0; i < count; ++i) out.emplace_back(names[i] ? names[i] : "", values[i]);
    return out;
}

double param(const std::vector<std::pair<std::string, double>>& p, const char* name, double fallback) {
    for (const auto& [k, v] : p)
        if (k == name) return v;
    return fallback;
}

fsim::sim::InitialConditions initialOf(const fsim_vehicle_spec& s) {
    fsim::sim::InitialConditions ic;
    ic.latitudeDeg = s.latitude_deg;
    ic.longitudeDeg = s.longitude_deg;
    ic.altitudeMslM = s.altitude_msl_m;
    ic.headingDeg = s.heading_deg;
    ic.pitchDeg = s.pitch_deg;
    ic.rollDeg = s.roll_deg;
    ic.airspeedTrueMs = s.airspeed_ms;
    ic.onGround = s.on_ground != 0;
    return ic;
}

const fsim_vehicle_state* asC(const fsim::sim::VehicleState* s) noexcept { return reinterpret_cast<const fsim_vehicle_state*>(s); }

int command(fsim_world* w, uint32_t id, const fsim::control::Command& c) {
    if (!w) return FSIM_INVALID_ARGUMENT;
    return w->world.command(id, c) ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "no vehicle with id " + std::to_string(id));
}

} // namespace

extern "C" {

FSIM_API double fsim_hold(void) { return std::numeric_limits<double>::quiet_NaN(); }

FSIM_API void fsim_world_options_init(fsim_world_options* o) {
    if (!o) return;
    std::memset(o, 0, sizeof(*o));
    o->struct_size = sizeof(*o);
    o->name = "default";
    o->dt = 1.0 / 120.0;
    o->frame_skip = 4;
    o->pin_workers = 1;
    o->capacity = 256;
    o->publish = 1;
    o->publish_interval_s = 1.0 / 60.0;
}

FSIM_API void fsim_vehicle_spec_init(fsim_vehicle_spec* s) {
    if (!s) return;
    std::memset(s, 0, sizeof(*s));
    s->struct_size = sizeof(*s);
    s->type = "jsbsim:c172x";
    const fsim::sim::InitialConditions ic;
    s->latitude_deg = ic.latitudeDeg;
    s->longitude_deg = ic.longitudeDeg;
    s->altitude_msl_m = ic.altitudeMslM;
    s->heading_deg = ic.headingDeg;
    s->airspeed_ms = ic.airspeedTrueMs;
    s->control_divider = 1;
}

FSIM_API void fsim_environment_init(fsim_environment* e) {
    if (!e) return;
    const fsim::sim::EnvironmentState d;
    e->struct_size = sizeof(*e);
    e->epoch_utc_seconds = d.epochUtcSeconds;
    e->time_factor = d.timeFactor;
    e->temperature_sl_k = d.temperatureSeaLevelK;
    e->pressure_sl_pa = d.pressureSeaLevelPa;
    e->humidity = d.humidity;
    e->wind_direction_deg = d.windDirectionDeg;
    e->wind_speed_ms = d.windSpeedMs;
    e->wind_gust_ms = d.windGustMs;
    e->turbulence = d.turbulence;
    e->visibility_m = d.visibilityM;
    e->cloud_base_m = d.cloudBaseM;
    e->cloud_cover = d.cloudCover;
    e->precipitation = d.precipitation;
}

FSIM_API int fsim_world_create(const fsim_world_options* options, fsim_world** out) {
    if (!options || !out || options->struct_size < sizeof(fsim_world_options)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_create: bad arguments");
    *out = nullptr;
    return guard("fsim_world_create", [&] {
        fsim::session::WorldOptions o;
        if (options->name) o.name = options->name;
        o.dt = options->dt;
        o.frameSkip = options->frame_skip;
        o.workers = options->workers;
        o.pinWorkers = options->pin_workers != 0;
        o.seed = options->seed;
        o.capacity = options->capacity;
        o.publish = options->publish != 0;
        o.publishIntervalSeconds = options->publish_interval_s;
        if (options->jsbsim_root) o.jsbsimRoot = options->jsbsim_root;
        *out = new fsim_world(o);
        return FSIM_OK;
    });
}

FSIM_API void fsim_world_destroy(fsim_world* world) { delete world; }

FSIM_API int fsim_world_step(fsim_world* world, uint32_t steps) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_world_step", [&] { world->world.step(steps); return FSIM_OK; });
}

FSIM_API double fsim_world_time(const fsim_world* world) { return world ? world->world.simTime() : 0.0; }
FSIM_API double fsim_world_step_seconds(const fsim_world* world) { return world ? world->world.dt() * world->world.frameSkip() : 0.0; }
FSIM_API uint64_t fsim_world_vehicle_steps(const fsim_world* world) { return world ? world->world.vehicleSteps() : 0; }
FSIM_API int fsim_world_published(const fsim_world* world) { return world && world->world.published() ? 1 : 0; }

FSIM_API int fsim_world_create_vehicle(fsim_world* world, const fsim_vehicle_spec* spec, uint32_t* id) {
    if (!world || !spec || !id || spec->struct_size < sizeof(fsim_vehicle_spec)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_create_vehicle: bad arguments");
    *id = 0;
    return guard("fsim_world_create_vehicle", [&] {
        fsim::session::VehicleSpec s;
        if (spec->name) s.name = spec->name;
        if (spec->type) s.type = spec->type;
        s.initial = initialOf(*spec);
        if (spec->model) s.model = spec->model;
        s.controlDivider = spec->control_divider;
        *id = world->world.createVehicle(s);
        return *id ? FSIM_OK : fail(FSIM_LOAD_FAILED, "fsim_world_create_vehicle: failed to create '" + s.name + "' (" + s.type + ")");
    });
}

FSIM_API int fsim_world_remove_vehicle(fsim_world* world, uint32_t id) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    return world->world.removeVehicle(id) ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "no vehicle with id " + std::to_string(id));
}

FSIM_API int fsim_world_reset_vehicle(fsim_world* world, uint32_t id, const fsim_vehicle_spec* spec) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_world_reset_vehicle", [&] {
        bool ok;
        if (spec) {
            const auto ic = initialOf(*spec);
            ok = world->world.resetVehicle(id, &ic);
        } else {
            ok = world->world.resetVehicle(id);
        }
        return ok ? FSIM_OK : fail(FSIM_ERROR, "fsim_world_reset_vehicle: reset failed for id " + std::to_string(id));
    });
}

FSIM_API uint32_t fsim_world_find_vehicle(const fsim_world* world, const char* name) {
    return world && name ? world->world.find(name) : 0;
}

FSIM_API uint32_t fsim_world_vehicle_count(const fsim_world* world) { return world ? static_cast<uint32_t>(world->world.vehicleCount()) : 0; }

FSIM_API uint32_t fsim_world_vehicle_ids(const fsim_world* world, uint32_t* ids, uint32_t capacity) {
    if (!world) return 0;
    const auto all = world->world.vehicleIds();
    if (ids)
        for (uint32_t i = 0; i < capacity && i < all.size(); ++i) ids[i] = all[i];
    return static_cast<uint32_t>(all.size());
}

FSIM_API const char* fsim_vehicle_name(const fsim_world* world, uint32_t id) {
    const auto* i = world ? world->world.info(id) : nullptr;
    return i ? i->name.c_str() : "";
}

FSIM_API const char* fsim_vehicle_type(const fsim_world* world, uint32_t id) {
    const auto* i = world ? world->world.info(id) : nullptr;
    return i ? i->type.c_str() : "";
}

FSIM_API const fsim_vehicle_state* fsim_vehicle_state_ptr(const fsim_world* world, uint32_t id) {
    return world ? asC(world->world.vehicleState(id)) : nullptr;
}

FSIM_API const fsim_vehicle_state* fsim_vehicle_sensed_ptr(const fsim_world* world, uint32_t id) {
    const auto* s = world ? world->world.sensedState(id) : nullptr;
    return s ? asC(&s->state) : nullptr;
}

FSIM_API int fsim_vehicle_get_property(fsim_world* world, uint32_t id, const char* path, double* value) {
    auto* m = world && path && value ? world->world.model(id) : nullptr;
    if (!m) return FSIM_INVALID_ARGUMENT;
    auto h = m->property(path);
    if (!h.valid()) return fail(FSIM_INVALID_ARGUMENT, std::string("unknown property ") + path);
    *value = h.get();
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_set_property(fsim_world* world, uint32_t id, const char* path, double value) {
    auto* m = world && path ? world->world.model(id) : nullptr;
    if (!m) return FSIM_INVALID_ARGUMENT;
    auto h = m->property(path);
    if (!h.valid()) return fail(FSIM_INVALID_ARGUMENT, std::string("unknown property ") + path);
    h.set(value);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_command_actuator(fsim_world* w, uint32_t id, const fsim_actuator_command* c) {
    if (!c) return FSIM_INVALID_ARGUMENT;
    fsim::control::ActuatorCommand a;
    a.aileron = c->aileron; a.elevator = c->elevator; a.rudder = c->rudder; a.throttle = c->throttle;
    a.flaps = c->flaps; a.gearDown = c->gear_down; a.brakeLeft = c->brake_left; a.brakeRight = c->brake_right;
    return command(w, id, a);
}

FSIM_API int fsim_vehicle_command_attitude(fsim_world* w, uint32_t id, const fsim_attitude_command* c) {
    if (!c) return FSIM_INVALID_ARGUMENT;
    fsim::control::AttitudeCommand a;
    a.rollRad = c->roll_rad; a.pitchRad = c->pitch_rad; a.headingRad = c->heading_rad; a.maxBankRad = c->max_bank_rad;
    a.throttle = c->throttle; a.airspeedMs = c->airspeed_ms;
    return command(w, id, a);
}

FSIM_API int fsim_vehicle_command_acceleration(fsim_world* w, uint32_t id, const fsim_acceleration_command* c) {
    if (!c) return FSIM_INVALID_ARGUMENT;
    fsim::control::AccelerationCommand a;
    a.loadFactorG = c->load_factor_g; a.rollRateRadS = c->roll_rate_rad_s; a.longitudinalMs2 = c->longitudinal_ms2; a.throttle = c->throttle;
    return command(w, id, a);
}

FSIM_API int fsim_vehicle_command_velocity(fsim_world* w, uint32_t id, const fsim_velocity_command* c) {
    if (!c) return FSIM_INVALID_ARGUMENT;
    fsim::control::VelocityCommand a;
    a.airspeedMs = c->airspeed_ms; a.verticalSpeedMs = c->vertical_speed_ms; a.headingRad = c->heading_rad; a.turnRateRadS = c->turn_rate_rad_s;
    return command(w, id, a);
}

FSIM_API int fsim_vehicle_command_position(fsim_world* w, uint32_t id, const fsim_position_command* c) {
    if (!c) return FSIM_INVALID_ARGUMENT;
    fsim::control::PositionCommand a;
    a.latitudeRad = c->latitude_rad; a.longitudeRad = c->longitude_rad; a.altitudeMslM = c->altitude_msl_m;
    a.airspeedMs = c->airspeed_ms; a.captureRadiusM = c->capture_radius_m;
    return command(w, id, a);
}

FSIM_API int fsim_vehicle_command_behavior(fsim_world* w, uint32_t id, const fsim_behavior_command* c) {
    if (!c || !c->id) return FSIM_INVALID_ARGUMENT;
    fsim::control::BehaviorCommand b;
    b.id = c->id;
    b.target = c->target;
    for (const auto& [k, v] : params(c->param_names, c->param_values, c->param_count)) b.params[k] = v;
    if (c->points)
        for (uint32_t i = 0; i < c->point_count; ++i) {
            fsim::control::PositionCommand p;
            p.latitudeRad = c->points[i].latitude_rad; p.longitudeRad = c->points[i].longitude_rad; p.altitudeMslM = c->points[i].altitude_msl_m;
            p.airspeedMs = c->points[i].airspeed_ms; p.captureRadiusM = c->points[i].capture_radius_m;
            b.points.push_back(p);
        }
    return command(w, id, b);
}

FSIM_API int fsim_vehicle_active_level(const fsim_world* world, uint32_t id) {
    const auto* c = world ? world->world.controls(id) : nullptr;
    return c ? static_cast<int>(c->activeLevel()) : -1;
}

FSIM_API int fsim_vehicle_behavior_finished(const fsim_world* world, uint32_t id) {
    const auto* c = world ? world->world.controls(id) : nullptr;
    return c && c->behaviorFinished() ? 1 : 0;
}

FSIM_API int fsim_vehicle_use_controller(fsim_world* world, uint32_t id, int level, const char* controllerId) {
    auto* c = world && controllerId ? world->world.controls(id) : nullptr;
    if (!c || level < 0 || level >= static_cast<int>(fsim::control::Level::Behavior)) return FSIM_INVALID_ARGUMENT;
    return c->use(static_cast<fsim::control::Level>(level), controllerId) ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, std::string("cannot use controller ") + controllerId);
}

FSIM_API int fsim_vehicle_set_controller_parameter(fsim_world* world, uint32_t id, int level, const char* name, double value) {
    auto* c = world && name ? world->world.controls(id) : nullptr;
    if (!c || level < 0 || level >= static_cast<int>(fsim::control::Level::Behavior)) return FSIM_INVALID_ARGUMENT;
    auto* ctl = c->controller(static_cast<fsim::control::Level>(level));
    if (!ctl || !ctl->setParameter(name, value)) return fail(FSIM_INVALID_ARGUMENT, std::string("unknown controller parameter ") + name);
    return FSIM_OK;
}

FSIM_API int fsim_world_get_environment(const fsim_world* world, fsim_environment* out) {
    if (!world || !out || out->struct_size < sizeof(fsim_environment)) return FSIM_INVALID_ARGUMENT;
    const auto& d = world->world.environment();
    out->epoch_utc_seconds = d.epochUtcSeconds; out->time_factor = d.timeFactor;
    out->temperature_sl_k = d.temperatureSeaLevelK; out->pressure_sl_pa = d.pressureSeaLevelPa; out->humidity = d.humidity;
    out->wind_direction_deg = d.windDirectionDeg; out->wind_speed_ms = d.windSpeedMs; out->wind_gust_ms = d.windGustMs; out->turbulence = d.turbulence;
    out->visibility_m = d.visibilityM; out->cloud_base_m = d.cloudBaseM; out->cloud_cover = d.cloudCover; out->precipitation = d.precipitation;
    return FSIM_OK;
}

FSIM_API int fsim_world_set_environment(fsim_world* world, const fsim_environment* e) {
    if (!world || !e || e->struct_size < sizeof(fsim_environment)) return FSIM_INVALID_ARGUMENT;
    auto d = world->world.environment();
    d.epochUtcSeconds = e->epoch_utc_seconds; d.timeFactor = e->time_factor;
    d.temperatureSeaLevelK = e->temperature_sl_k; d.pressureSeaLevelPa = e->pressure_sl_pa; d.humidity = e->humidity;
    d.windDirectionDeg = e->wind_direction_deg; d.windSpeedMs = e->wind_speed_ms; d.windGustMs = e->wind_gust_ms; d.turbulence = e->turbulence;
    d.visibilityM = e->visibility_m; d.cloudBaseM = e->cloud_base_m; d.cloudCover = e->cloud_cover; d.precipitation = e->precipitation;
    world->world.setEnvironment(d);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_add_effect(fsim_world* world, uint32_t id, const char* effectId, const char* const* names, const double* values, uint32_t count) {
    if (!world || !effectId) return FSIM_INVALID_ARGUMENT;
    const auto p = params(names, values, count);
    const std::string eid = effectId;
    if (!fsim::effects::createBuiltinEffect(eid, p)) return fail(FSIM_INVALID_ARGUMENT, "unknown effect " + eid);
    if (id == 0) {
        world->world.addEffectToAll([eid, p] { return fsim::effects::createBuiltinEffect(eid, p); });
        return FSIM_OK;
    }
    return world->world.addEffect(id, fsim::effects::createBuiltinEffect(eid, p)) ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "no vehicle with id " + std::to_string(id));
}

FSIM_API int fsim_vehicle_clear_effects(fsim_world* world, uint32_t id) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    world->world.clearEffects(id);
    return FSIM_OK;
}

FSIM_API int fsim_comm_create_node(fsim_world* world, uint32_t address) {
    if (!world || address == 0 || address == fsim::comm::kBroadcast) return FSIM_INVALID_ARGUMENT;
    world->world.network().createNode(address, 0);
    return FSIM_OK;
}

FSIM_API int fsim_comm_send(fsim_world* world, uint32_t from, uint32_t to, uint32_t channel, uint32_t format, const void* bytes, size_t length) {
    if (!world || !world->world.network().node(from)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::Message m;
    m.from = from;
    m.to = to;
    m.channel = channel;
    m.payload.format = format;
    if (bytes && length) m.payload.bytes.assign(static_cast<const uint8_t*>(bytes), static_cast<const uint8_t*>(bytes) + length);
    world->world.network().send(std::move(m));
    return FSIM_OK;
}

FSIM_API uint32_t fsim_comm_inbox_count(const fsim_world* world, uint32_t node) {
    const auto* n = world ? world->world.network().node(node) : nullptr;
    return n ? static_cast<uint32_t>(n->inbox().size()) : 0;
}

FSIM_API int fsim_comm_inbox_get(const fsim_world* world, uint32_t node, uint32_t index, fsim_message* out) {
    const auto* n = world && out ? world->world.network().node(node) : nullptr;
    if (!n || index >= n->inbox().size()) return FSIM_INVALID_ARGUMENT;
    const auto& m = n->inbox()[index];
    out->from = m.from; out->to = m.to; out->channel = m.channel; out->format = m.payload.format;
    out->time_sent = m.timeSent; out->time_delivered = m.timeDelivered;
    out->bytes = m.payload.bytes.data(); out->length = m.payload.bytes.size();
    return FSIM_OK;
}

FSIM_API int fsim_comm_set_medium(fsim_world* world, const char* mediumId, const char* const* names, const double* values, uint32_t count) {
    if (!world || !mediumId) return FSIM_INVALID_ARGUMENT;
    const std::string id = mediumId;
    const auto p = params(names, values, count);
    if (id == "ideal") {
        world->world.network().setMedium(std::make_unique<fsim::comm::IdealMedium>());
        return FSIM_OK;
    }
    if (id == "link") {
        auto link = std::make_unique<fsim::comm::LinkModel>();
        link->rangeM = param(p, "range_m", link->rangeM);
        link->latencyS = param(p, "latency_s", link->latencyS);
        link->jitterS = param(p, "jitter_s", link->jitterS);
        link->lossProbability = param(p, "loss_probability", link->lossProbability);
        world->world.network().setMedium(std::move(link));
        return FSIM_OK;
    }
    return fail(FSIM_INVALID_ARGUMENT, "unknown medium " + id);
}

FSIM_API int fsim_comm_attach_protocol(fsim_world* world, uint32_t node, const char* protocolId, const char* const* names, const double* values, uint32_t count) {
    if (!world || !protocolId || !world->world.network().node(node)) return FSIM_INVALID_ARGUMENT;
    const std::string id = protocolId;
    const auto p = params(names, values, count);
    if (id == "beacon") {
        world->world.network().attach(node, std::make_unique<fsim::comm::BeaconProtocol>(param(p, "period_s", 1.0), static_cast<uint32_t>(param(p, "channel", 1.0))));
        return FSIM_OK;
    }
    return fail(FSIM_INVALID_ARGUMENT, "unknown protocol " + id);
}

} // extern "C"
