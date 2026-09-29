// C ABI for the World / vehicle object model (design 9.8) over session::World.

#include "fsim/BuiltinEffects.h"
#include "fsim/Comm.h"
#include "fsim/Recording.h"
#include "fsim/Scenario.h"
#include "fsim/fsim_c.h"

#include "control/Catalog.h"
#include "fsim/Magnetic.h"
#include "core/Log.h"
#include "sdk/Handles.h"
#include "sdk/LastError.h"
#include "session/Scenario.h"
#include "session/World.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// The C state struct must mirror fsim::VehicleState exactly.
static_assert(sizeof(fsim_vehicle_state) == sizeof(fsim::sim::VehicleState), "fsim_vehicle_state layout differs from VehicleState");
static_assert(offsetof(fsim_vehicle_state, rotation_body_to_ecef) == offsetof(fsim::sim::VehicleState, rotationBodyToEcef), "layout");
static_assert(offsetof(fsim_vehicle_state, engine_count) == offsetof(fsim::sim::VehicleState, engineCount), "layout");
static_assert(offsetof(fsim_vehicle_state, on_ground) == offsetof(fsim::sim::VehicleState, onGround), "layout");
static_assert(offsetof(fsim_vehicle_state, engine_rpm) == offsetof(fsim::sim::VehicleState, engineRpm), "layout");
static_assert(offsetof(fsim_vehicle_state, nozzle_position) == offsetof(fsim::sim::VehicleState, nozzlePosition), "layout");
static_assert(offsetof(fsim_vehicle_state, leading_edge_flap_rad) == offsetof(fsim::sim::VehicleState, leadingEdgeFlapRad), "layout");
static_assert(offsetof(fsim_vehicle_state, wheel_count) == offsetof(fsim::sim::VehicleState, wheelCount), "layout");
static_assert(offsetof(fsim_vehicle_state, wheel_compression_m) == offsetof(fsim::sim::VehicleState, wheelCompressionM), "layout");
static_assert(offsetof(fsim_vehicle_state, wheel_speed_ms) == offsetof(fsim::sim::VehicleState, wheelSpeedMs), "layout");
static_assert(FSIM_MAX_WHEELS == fsim::sim::VehicleState::kMaxWheels, "layout");
static_assert(sizeof(bool) == 1, "bool must be one byte for the C mirror of VehicleState");
static_assert(sizeof(fsim_control_inputs) == sizeof(fsim::ControlInputs), "fsim_control_inputs layout differs from ControlInputs");
static_assert(offsetof(fsim_control_inputs, brake_right) == offsetof(fsim::ControlInputs, brakeRight), "layout");
// fsim_world_command_batch reads a row of doubles as the level's command struct.
static_assert(sizeof(fsim_actuator_command) == 8 * sizeof(double), "fsim_actuator_command must be 8 doubles");
static_assert(sizeof(fsim_attitude_command) == 6 * sizeof(double), "fsim_attitude_command must be 6 doubles");
static_assert(sizeof(fsim_acceleration_command) == 4 * sizeof(double), "fsim_acceleration_command must be 4 doubles");
static_assert(sizeof(fsim_velocity_command) == 4 * sizeof(double), "fsim_velocity_command must be 4 doubles");
static_assert(sizeof(fsim_position_command) == 5 * sizeof(double), "fsim_position_command must be 5 doubles");

namespace {

int fail(int code, const std::string& message) noexcept {
    fsim::sdk::lastError() = message;
    LOG_ERROR("sdk") << message;
    return code;
}

/// A query's answer that there is nothing to read (a task not kept, an activity not live): fsim_last_error says so, and nothing is logged.
int absent(int code, const std::string& message) noexcept {
    fsim::sdk::lastError() = message;
    return code;
}

int guard(const char* what, const std::function<int()>& fn) noexcept {
    try {
        const int r = fn();
        if (r == FSIM_OK) fsim::sdk::lastError().clear();
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

/// `local` into the caller's struct, up to the size the caller gives (its struct_size first); false if it gives none.
template <typename T>
bool copyOut(const T& local, T* out) noexcept {
    if (!out || out->struct_size < sizeof(uint32_t)) return false;
    const uint32_t size = out->struct_size;
    std::memcpy(reinterpret_cast<char*>(out) + sizeof(uint32_t), reinterpret_cast<const char*>(&local) + sizeof(uint32_t),
                std::min<std::size_t>(size, sizeof(T)) - sizeof(uint32_t));
    return true;
}

int command(fsim_world* w, uint32_t id, const fsim::control::Command& c) {
    if (!w) return FSIM_INVALID_ARGUMENT;
    const auto r = w->world.commandResult(id, c);
    if (r.accepted()) return FSIM_OK;
    if (r.reason == fsim::control::Reason::UnknownVehicle) return fail(FSIM_INVALID_ARGUMENT, "no vehicle with id " + std::to_string(id));
    return fail(FSIM_INVALID_ARGUMENT, std::string("vehicle ") + std::to_string(id) + ": command rejected (" + fsim::control::reasonName(r.reason) + ")");
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
    o->terrain_zoom = 12;
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
        o.terrain = options->terrain != 0;
        if (options->terrain_url) o.terrainUrl = options->terrain_url;
        o.terrainZoom = options->terrain_zoom ? options->terrain_zoom : 12u;
        if (options->record_path) o.recordPath = options->record_path;
        o.recordIntervalSeconds = options->record_interval_s;
        *out = new fsim_world(o);
        return FSIM_OK;
    });
}

FSIM_API void fsim_world_destroy(fsim_world* world) {
    if (world && !world->owned) return; // a borrowed handle (fsim_vecenv_world) belongs to its owner
    delete world;
}

FSIM_API void fsim_set_last_error(const char* message) { fsim::sdk::lastError() = message ? message : ""; }

} // extern "C"

fsim::World* fsim_world_object(fsim_world* world) {
    if (!world) return nullptr;
    if (!world->object) world->object.reset(fsim::World::borrowed(world->world));
    return world->object.get();
}

extern "C" {

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
    std::string error;
    if (!h.trySet(value, error)) return fail(FSIM_ERROR, std::string("set ") + path + ": " + error);
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
    // (through the stack: the vehicle's performance is computed afresh from it)
    if (!c->setParameter(static_cast<fsim::control::Level>(level), name, value)) return fail(FSIM_INVALID_ARGUMENT, std::string("unknown controller parameter ") + name);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_controller_parameter(const fsim_world* world, uint32_t id, int level, const char* name, double* value) {
    const auto* c = world && name && value ? world->world.controls(id) : nullptr;
    if (!c || level < 0 || level >= static_cast<int>(fsim::control::Level::Behavior)) return FSIM_INVALID_ARGUMENT;
    const auto* ctl = c->controller(static_cast<fsim::control::Level>(level));
    const auto v = ctl ? ctl->parameter(name) : std::nullopt;
    if (!v) return fail(FSIM_INVALID_ARGUMENT, std::string("unknown controller parameter ") + name);
    *value = *v;
    return FSIM_OK;
}

// --- Capabilities and activities (ABI 1.4) ------------------------------------------

namespace {

static_assert(sizeof(fsim_activity_id) == sizeof(fsim::control::ActivityId), "activity ids are 64 bits");

/// The world's last answer (fsim_last_command_detail), and the vehicle whose details go with it.
void remember(fsim_world* w, uint32_t vehicle, const fsim::control::CommandResult& r) noexcept {
    if (!w) return;
    w->last = r;
    w->lastVehicle = vehicle;
}

/// The answer into the C struct, and kept as the world's last (fsim_last_command_detail).
void toC(fsim_world* w, uint32_t vehicle, const fsim::control::CommandResult& r, fsim_command_result* out) noexcept {
    remember(w, vehicle, r);
    if (!out) return;
    out->status = static_cast<int32_t>(r.status);
    out->reason = static_cast<int32_t>(r.reason);
    out->activity = r.activity;
    out->other = r.other;
    out->flags = r.flags;
    out->reserved = r.index >= 0 ? static_cast<uint32_t>(r.index) + 1u : 0u;
}


void infoToC(const fsim::control::ActivityRecord& a, fsim_activity_info* out) noexcept {
    out->id = a.id;
    out->vehicle = a.vehicle;
    out->capability = a.capability;
    out->source = static_cast<int32_t>(a.source);
    out->axes = a.axes;
    out->state = static_cast<int32_t>(a.state);
    out->reason = static_cast<int32_t>(a.reason);
    out->by = a.by;
    out->constraints = a.constraints;
    out->constraints_seen = a.constraintsSeen;
    out->start_time = a.startTime;
    out->end_time = a.endTime;
}

/// Whether the caller's struct (its struct_size) reaches past `member`.
#define FSIM_HAS(o, type, member) ((o)->struct_size >= offsetof(type, member) + sizeof((o)->member))

fsim::control::CommandOptions fromC(const fsim_command_options* o) noexcept {
    fsim::control::CommandOptions out;
    if (!o) return out;
    out.source = static_cast<fsim::control::Source>(std::clamp(o->source, 0, 2));
    out.axes = static_cast<fsim::control::AxisMask>(o->axes & fsim::control::kAllAxes);
    out.range = static_cast<fsim::control::RangePolicy>(std::clamp(o->range, 0, 2));
    out.minVersion = static_cast<std::uint16_t>(std::min<uint32_t>(o->min_version, 0xFFFF));
    // ABI 1.8: the command envelope, where the caller's header has it
    if (FSIM_HAS(o, fsim_command_options, command_id)) out.commandId = o->command_id;
    if (FSIM_HAS(o, fsim_command_options, trace))
        for (std::size_t i = 0; i < fsim::control::kMaxRequirements; ++i) {
            const int kind = o->trace[i].kind;
            out.trace[i].kind = kind > 0 && kind < static_cast<int>(fsim::control::RequirementKind::Count) ? static_cast<fsim::control::RequirementKind>(kind)
                                                                                                          : fsim::control::RequirementKind::None;
            out.trace[i].id = out.trace[i].kind == fsim::control::RequirementKind::None ? 0 : o->trace[i].id;
        }
    if (FSIM_HAS(o, fsim_command_options, interactive)) out.interactive = o->interactive != 0;
    if (FSIM_HAS(o, fsim_command_options, validate_only)) out.validateOnly = o->validate_only != 0;
    // ABI 1.9: ranks, queues and time windows
    if (FSIM_HAS(o, fsim_command_options, interrupt)) out.interrupt = o->interrupt != 0;
    if (FSIM_HAS(o, fsim_command_options, override_rejection)) out.overrideRejection = o->override_rejection != 0;
    if (FSIM_HAS(o, fsim_command_options, rank_precedence)) out.rank = {o->rank_priority, o->rank_precedence};
    if (FSIM_HAS(o, fsim_command_options, precedence_override)) out.precedenceOverride = o->precedence_override;
    if (FSIM_HAS(o, fsim_command_options, end_not_after)) {
        out.window.startNotBefore = o->start_not_before, out.window.startNotAfter = o->start_not_after;
        out.window.endNotBefore = o->end_not_before, out.window.endNotAfter = o->end_not_after;
    }
    if (FSIM_HAS(o, fsim_command_options, criticality)) // (one beyond them: the window check refuses it, InvalidParameter)
        out.window.criticality = static_cast<fsim::control::TimeCriticality>(std::clamp(o->criticality, 0, 255));
    // ABI 1.12: a policy's controller (where 1.9's header had its reserved word, left 0 by fsim_command_options_init)
    if (FSIM_HAS(o, fsim_command_options, controller)) out.controller = o->controller;
    return out;
}

/// A level's command from its fields in the C structs' order - the struct's
/// count, or every field with the rotorcraft's (the rest kHold); false if the count is neither.
bool toCommand(int level, const double* fields, uint32_t count, fsim::control::Command& out) noexcept {
    switch (level) {
    case FSIM_LEVEL_ACTUATOR: out = fsim::control::ActuatorCommand{}; break;
    case FSIM_LEVEL_ATTITUDE: out = fsim::control::AttitudeCommand{}; break;
    case FSIM_LEVEL_ACCELERATION: out = fsim::control::AccelerationCommand{}; break;
    case FSIM_LEVEL_VELOCITY: out = fsim::control::VelocityCommand{}; break;
    case FSIM_LEVEL_POSITION: out = fsim::control::PositionCommand{}; break;
    default: return false;
    }
    double* slots[fsim::control::kMaxCommandFields];
    const std::size_t n = fsim::control::commandFields(out, slots);
    if (!fields || (count != n && count != fsim_command_field_count(level))) return false;
    for (std::size_t i = 0; i < count; ++i) *slots[i] = fields[i];
    return true;
}

/// "4 or 6": the counts a level's command is given in.
std::string fieldCounts(int level) {
    const uint32_t legacy = fsim_command_field_count(level), full = fsim_command_field_count_full(level);
    return legacy == full ? std::to_string(legacy) : std::to_string(legacy) + " or " + std::to_string(full);
}

fsim::control::BehaviorCommand toBehavior(const fsim_behavior_command* c) {
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
    return b;
}

const fsim::control::CapabilityDescriptor* capabilityAt(fsim_world* w, uint32_t id, uint32_t index) {
    if (!w) return nullptr;
    const auto& all = w->world.capabilities(id);
    return index < all.size() ? &all[index] : nullptr;
}

/// A support command of a kind from its fields; false if the kind or count is wrong.
bool toSupport(int kind, const double* f, uint32_t count, fsim::control::SupportCommand& out) noexcept {
    using namespace fsim::control;
    if (!f) return false;
    switch (kind) {
    case FSIM_SUPPORT_GEAR: return count == 1 && (out = GearCommand{f[0]}, true);
    case FSIM_SUPPORT_FLAPS: return count == 1 && (out = FlapsCommand{f[0]}, true);
    case FSIM_SUPPORT_WHEEL_BRAKES: return count == 2 && (out = WheelBrakesCommand{f[0], f[1]}, true);
    case FSIM_SUPPORT_SPEEDBRAKE: return count == 1 && (out = SpeedbrakeCommand{f[0]}, true);
    case FSIM_SUPPORT_PITCH_TRIM: return count == 1 && (out = PitchTrimCommand{f[0]}, true);
    case FSIM_SUPPORT_ENGINES: {
        if (count < 1 || count > 4) return false;
        EnginesCommand e;
        for (uint32_t i = 0; i < count; ++i) e.throttle[i] = f[i];
        out = e;
        return true;
    }
    default: return false;
    }
}

/// A mode's setpoint from its fields in order - a pattern's after its 13 into `shape` (its shape, ABI 1.21), a curve's
/// after its 14 into `curveShape` (its reference in a frame, ABI 1.25; the rest left out, and none given: the shape
/// untouched) - false if the mode or the count is wrong.
bool toMode(int mode, const double* fields, uint32_t count, fsim::control::Command& out, fsim::control::PatternShape* shape = nullptr,
            fsim::control::CurveShape* curveShape = nullptr) noexcept {
    if (!fields) return false;
    if (mode == FSIM_MODE_HSA) out = fsim::control::HsaCommand{};
    else if (mode == FSIM_MODE_ROUTE) out = fsim::control::RouteCommand{};
    else if (mode == FSIM_MODE_PATTERN) out = fsim::control::PatternCommand{};
    else if (mode == FSIM_MODE_CURVE) out = fsim::control::CurveCommand{};
    else if (mode == FSIM_MODE_MUST_FLY) out = fsim::control::MustFlyCommand{}; // (ABI 1.39)
    else return false;
    double* slots[fsim::control::kMaxCommandFields];
    std::size_t n = fsim::control::commandFields(out, slots);
    if (mode == FSIM_MODE_PATTERN && shape) { // (a pattern given without room for a shape takes its 13 alone)
        shape->fields(slots + n);
        n += fsim::control::PatternShape::kFields;
    }
    if (mode == FSIM_MODE_CURVE && curveShape) { // (likewise a curve's 14)
        curveShape->fields(slots + n);
        n += fsim::control::CurveShape::kFields;
    }
    // (ABI 1.14's hsa had 6 fields and its pattern 12, and before 1.25 a curve had 8: a caller built against them leaves
    // the fields added since out)
    const std::size_t least = mode == FSIM_MODE_HSA ? 6 : mode == FSIM_MODE_PATTERN ? 12 : mode == FSIM_MODE_CURVE ? 8 : n;
    if (count < least || count > n) return false;
    for (uint32_t i = 0; i < count; ++i) *slots[i] = fields[i];
    return true;
}

/// A pattern's fields past its 13: its shape given (ABI 1.21).
constexpr uint32_t kPatternFields = 13;

/// A marshall from its fields in order (ABI 1.43; docs/flight-autonomy.md, 4.46) - its 13, then its pattern's shape's into `shape`
/// (none given: the shape untouched) - false if the count is wrong. Not one of the command variant's: apart from toMode.
bool toMarshall(const double* fields, uint32_t count, fsim::control::MarshallCommand& out, fsim::control::PatternShape* shape) noexcept {
    if (!fields) return false;
    out = fsim::control::MarshallCommand{};
    double* slots[fsim::control::kMaxCommandFields];
    std::size_t n = fsim::control::marshallFields(out, slots);
    if (shape) shape->fields(slots + n), n += fsim::control::PatternShape::kFields;
    if (count < kPatternFields || count > n) return false;
    for (uint32_t i = 0; i < count; ++i) *slots[i] = fields[i];
    return true;
}
/// A curve's fields past its 14: its reference in a frame given (ABI 1.25).
constexpr uint32_t kCurveFields = 14;

/// A route's waypoints as the caller's header laid them out (`waypoints[0].struct_size`
/// apart), into the world's buffer; false if they cannot be read.
bool toWaypoints(fsim_world* w, const fsim_waypoint* waypoints, uint32_t count) {
    w->waypoints.clear();
    if (count == 0) return true;
    if (!waypoints) return false;
    const uint32_t stride = waypoints[0].struct_size;
    if (stride < offsetof(fsim_waypoint, altitude_m)) return false; // not even its position
    const auto* bytes = reinterpret_cast<const unsigned char*>(waypoints);
    w->waypoints.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_waypoint c;
        fsim_waypoint_init(&c);
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, std::min<std::size_t>(stride, sizeof c)); // what the caller's has
        fsim::control::Waypoint& p = w->waypoints[i];
        p.latitudeRad = c.latitude_rad, p.longitudeRad = c.longitude_rad;
        p.altitudeM = c.altitude_m, p.altitudeReference = c.altitude_reference;
        p.speed = c.speed, p.speedReference = c.speed_reference;
        p.turn = c.turn, p.maxBankRad = c.max_bank_rad, p.climbRateMs = c.climb_rate_ms;
        p.id = c.id;
        p.altitudeMinM = c.altitude_min_m, p.altitudeMaxM = c.altitude_max_m, p.kind = c.kind, p.waypointType = c.waypoint_type; // (ABI 1.26)
        p.frame = c.frame, p.frameRotation = c.frame_rotation, p.frameOffsets = c.frame_offsets;
        p.frameXM = c.frame_x_m, p.frameYM = c.frame_y_m, p.frameZM = c.frame_z_m;
        p.courseRad = c.course_rad, p.turnRadiusM = c.turn_radius_m; // (ABI 1.27)
        p.speedOptimization = c.speed_optimization, p.climbOptimization = c.climb_optimization, p.accelerationMs2 = c.acceleration_ms2; // (1.29)
        p.arrivalBeginS = c.arrival_begin_s, p.arrivalEndS = c.arrival_end_s; // (1.30)
        p.rnpM = c.rnp_m;                                                     // (1.32)
        p.next = c.next;                                                      // (1.33)
        p.terminator = c.terminator;                                          // (1.35)
    }
    return true;
}

/// A curve's segments as the caller's header laid them out (`segments[0].struct_size`
/// apart), into the world's buffer; false if they cannot be read.
bool toSegments(fsim_world* w, const fsim_bezier_segment* segments, uint32_t count) {
    w->segments.clear();
    if (count == 0) return true;
    if (!segments) return false;
    const uint32_t stride = segments[0].struct_size;
    if (stride < sizeof(fsim_bezier_segment)) return false; // (its first layout: every control point)
    const auto* bytes = reinterpret_cast<const unsigned char*>(segments);
    w->segments.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_bezier_segment c;
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, sizeof c);
        fsim::control::BezierSegment& s = w->segments[i];
        std::copy_n(c.north, 6, s.north), std::copy_n(c.east, 6, s.east), std::copy_n(c.down, 6, s.down);
    }
    return true;
}

/// A curve's segments as A-GRA's schema gives them, as the caller's header laid them out (`segments[0].struct_size`
/// apart), into the world's buffer; false if they cannot be read.
bool toNurbs(fsim_world* w, const fsim_nurbs_segment* segments, uint32_t count) {
    w->nurbs.clear();
    if (count == 0) return true;
    if (!segments) return false;
    const uint32_t stride = segments[0].struct_size;
    if (stride < sizeof(fsim_nurbs_segment)) return false; // (its first layout)
    const auto* bytes = reinterpret_cast<const unsigned char*>(segments);
    w->nurbs.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_nurbs_segment c;
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, sizeof c);
        fsim::control::NurbsSegment& s = w->nurbs[i];
        s.points = c.points, s.knots = c.knots;
        std::copy_n(c.north, 10, s.north), std::copy_n(c.east, 10, s.east), std::copy_n(c.down, 10, s.down), std::copy_n(c.weight, 10, s.weight);
        std::copy_n(c.knot, 14, s.knot);
        s.curvature = c.curvature, s.firstIndex = c.first_index, s.lastIndex = c.last_index;
    }
    return true;
}

/// A route's loiters as the caller's header laid them out (`loiters[0].struct_size` apart), into the world's buffer;
/// false if they cannot be read.
bool toLoiters(fsim_world* w, const fsim_route_loiter* loiters, uint32_t count) {
    w->loiters.clear();
    if (count == 0) return true;
    if (!loiters) return false;
    const uint32_t stride = loiters[0].struct_size;
    if (stride < sizeof(fsim_route_loiter)) return false; // (its first layout)
    const auto* bytes = reinterpret_cast<const unsigned char*>(loiters);
    w->loiters.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_route_loiter c;
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, sizeof c);
        fsim::control::RouteLoiter& l = w->loiters[i];
        l = fsim::control::RouteLoiter{};
        l.point = c.point, l.endTimeS = c.end_time_s;
        double* f[fsim::control::RouteLoiter::kFields];
        l.fields(f);
        for (std::size_t k = 0; k < fsim::control::RouteLoiter::kFields; ++k) *f[k] = c.fields[k];
    }
    return true;
}

/// A route's planned states as the caller's header laid them out (`states[0].struct_size` apart), into the world's
/// buffer; false if they cannot be read.
bool toStates(fsim_world* w, const fsim_route_state* states, uint32_t count) {
    w->states.clear();
    if (count == 0) return true;
    if (!states) return false;
    const uint32_t stride = states[0].struct_size;
    if (stride < sizeof(fsim_route_state)) return false; // (its first layout)
    const auto* bytes = reinterpret_cast<const unsigned char*>(states);
    w->states.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_route_state c;
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, sizeof c);
        fsim::control::RouteState& s = w->states[i];
        s = fsim::control::RouteState{};
        s.point = c.point;
        double* f[fsim::control::RouteState::kFields];
        s.fields(f);
        for (std::size_t k = 0; k < fsim::control::RouteState::kFields; ++k) *f[k] = c.fields[k];
    }
    return true;
}

/// A route's paths as the caller's header laid them out (`paths[0].struct_size` apart), into the world's buffer; false if
/// they cannot be read.
bool toPaths(fsim_world* w, const fsim_route_path* paths, uint32_t count) {
    w->paths.clear();
    if (count == 0) return true;
    if (!paths) return false;
    const uint32_t stride = paths[0].struct_size;
    if (stride < sizeof(fsim_route_path)) return false; // (its first layout)
    const auto* bytes = reinterpret_cast<const unsigned char*>(paths);
    w->paths.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_route_path c;
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, sizeof c);
        w->paths[i] = fsim::control::RoutePath{c.id, c.type, c.first, c.count};
    }
    return true;
}

/// A route's conditional branches as the caller's header laid them out (`branches[0].struct_size` apart), into the world's
/// buffer; false if they cannot be read.
bool toBranches(fsim_world* w, const fsim_route_branch* branches, uint32_t count) {
    w->branches.clear();
    if (count == 0) return true;
    if (!branches) return false;
    const uint32_t stride = branches[0].struct_size;
    if (stride < sizeof(fsim_route_branch)) return false; // (its first layout)
    const auto* bytes = reinterpret_cast<const unsigned char*>(branches);
    w->branches.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_route_branch c;
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, sizeof c);
        fsim::control::RouteBranch& b = w->branches[i];
        b = fsim::control::RouteBranch{};
        b.point = c.point;
        double* f[fsim::control::RouteBranch::kFields];
        b.fields(f);
        for (std::size_t k = 0; k < fsim::control::RouteBranch::kFields; ++k) *f[k] = c.fields[k];
    }
    return true;
}

/// A route's civil path terminators' data as the caller's header laid them out (`terminators[0].struct_size` apart), into
/// the world's buffer; false if they cannot be read.
bool toTerminators(fsim_world* w, const fsim_route_terminator* terminators, uint32_t count) {
    w->terminators.clear();
    if (count == 0) return true;
    if (!terminators) return false;
    const uint32_t stride = terminators[0].struct_size;
    if (stride < sizeof(fsim_route_terminator)) return false; // (its first layout)
    const auto* bytes = reinterpret_cast<const unsigned char*>(terminators);
    w->terminators.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_route_terminator c;
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, sizeof c);
        fsim::control::RouteTerminator& t = w->terminators[i];
        t = fsim::control::RouteTerminator{};
        t.point = c.point;
        double* f[fsim::control::RouteTerminator::kFields];
        t.fields(f);
        for (std::size_t k = 0; k < fsim::control::RouteTerminator::kFields; ++k) *f[k] = c.fields[k];
    }
    return true;
}

/// What goes beside a route's waypoints, as the caller's header laid it out, into the world's buffers (none: none); false
/// if it cannot be read.
bool toExtras(fsim_world* w, const fsim_route_extras* extras) {
    fsim_route_extras e;
    fsim_route_extras_init(&e);
    if (extras) {
        if (extras->struct_size < sizeof(uint32_t)) return false;
        std::memcpy(&e, extras, std::min<std::size_t>(extras->struct_size, sizeof e));
        e.struct_size = sizeof e;
    }
    return toLoiters(w, e.loiters, e.loiter_count) && toStates(w, e.states, e.state_count) && toPaths(w, e.paths, e.path_count) &&
           toBranches(w, e.branches, e.branch_count) && toTerminators(w, e.terminators, e.terminator_count);
}

/// What an activity's UPDATE takes: its level (a flight capability), its
/// support kind (a support one), its mode, or none (unknown, ended long ago, a behaviour).
struct UpdateShape {
    int level = FSIM_LEVEL_BEHAVIOR;
    int support = -1;
    int mode = -1;
    uint32_t fields = 0;
};

UpdateShape updateShape(fsim_world* w, fsim::control::ActivityId activity) {
    UpdateShape shape;
    const auto* a = w->world.activity(activity);
    if (!a) return shape;
    const auto& all = w->world.capabilities(a->vehicle);
    if (a->capability >= all.size()) return shape;
    const auto& d = all[a->capability];
    for (std::size_t k = 0; k < fsim::control::kSupportKinds; ++k)
        if (d.id == fsim::control::supportCapability(k)) shape.support = static_cast<int>(k);
    switch (d.setpoint) {
    case fsim::control::SetpointKind::Hsa: shape.mode = FSIM_MODE_HSA; break;
    case fsim::control::SetpointKind::Route: shape.mode = FSIM_MODE_ROUTE; break;
    case fsim::control::SetpointKind::Pattern: shape.mode = FSIM_MODE_PATTERN; break;
    case fsim::control::SetpointKind::Curve: shape.mode = FSIM_MODE_CURVE; break;
    case fsim::control::SetpointKind::MustFly: shape.mode = FSIM_MODE_MUST_FLY; break;
    case fsim::control::SetpointKind::Marshall: shape.mode = FSIM_MODE_MARSHALL; break;
    default: break;
    }
    if (shape.mode >= 0) {
        shape.fields = fsim_mode_field_count(shape.mode);
    } else if (shape.support >= 0) {
        shape.fields = static_cast<uint32_t>(d.parameters.size()); // set beside the cascade: a support effector or the engines
    } else if (d.kind == fsim::control::CapabilityKind::Flight) {
        shape.level = static_cast<int>(d.level);
        shape.fields = fsim_command_field_count(shape.level);
    }
    return shape;
}

/// UPDATE from fields in the activity's own shape; an activity that takes none is answered by the host.
fsim::control::CommandResult updateFrom(fsim_world* w, fsim::control::ActivityId activity, const UpdateShape& shape, const double* fields,
                                        uint32_t count, bool& malformed, fsim::control::Caller caller = {}) {
    malformed = false;
    if (shape.support >= 0) {
        fsim::control::SupportCommand c;
        if (toSupport(shape.support, fields, count, c)) return w->world.update(caller, activity, c);
        malformed = true;
        return {};
    }
    fsim::control::Command c = fsim::control::ActuatorCommand{};
    if (shape.mode == FSIM_MODE_MARSHALL) { // (ABI 1.43)
        fsim::control::MarshallCommand m;
        fsim::control::PatternShape given;
        if (toMarshall(fields, count, m, &given))
            return count > kPatternFields ? w->world.update(caller, activity, m, given) : w->world.update(caller, activity, m);
        malformed = true;
        return {};
    }
    if (shape.mode >= 0) {
        fsim::control::PatternShape given;
        fsim::control::CurveShape curveGiven;
        if (toMode(shape.mode, fields, count, c, &given, &curveGiven)) {
            if (const auto* pattern = std::get_if<fsim::control::PatternCommand>(&c); pattern && count > kPatternFields)
                return w->world.update(caller, activity, *pattern, given);
            if (const auto* curve = std::get_if<fsim::control::CurveCommand>(&c); curve && count > kCurveFields) // (its options alone, with its shape's)
                return w->world.update(caller, activity, *curve, fsim::Span<const fsim::control::NurbsSegment>{}, &curveGiven);
            return w->world.update(caller, activity, c);
        }
        malformed = true;
        return {};
    }
    if (shape.level != FSIM_LEVEL_BEHAVIOR && !toCommand(shape.level, fields, count, c)) {
        malformed = true;
        return {};
    }
    return w->world.update(caller, activity, c); // unknown, ended, or a behaviour: the host says which
}

/// A caller's declared source (fsim_source); false if it is none.
bool toSource(int source, fsim::control::Source& out) noexcept {
    if (source < FSIM_SOURCE_POLICY || source > FSIM_SOURCE_OVERRIDE) return false;
    out = static_cast<fsim::control::Source>(source);
    return true;
}

} // namespace

FSIM_API void fsim_command_options_init(fsim_command_options* options) {
    if (!options) return;
    std::memset(options, 0, sizeof *options);
    options->struct_size = sizeof *options;
    options->source = FSIM_SOURCE_POLICY;
    options->range = FSIM_RANGE_CLAMP;
    options->interactive = 1;
    options->interrupt = 1;
    options->precedence_override = FSIM_NO_PRECEDENCE_OVERRIDE;
    options->start_not_before = options->start_not_after = options->end_not_before = options->end_not_after = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API uint32_t fsim_vehicle_capability_count(fsim_world* world, uint32_t id) {
    return world ? static_cast<uint32_t>(world->world.capabilities(id).size()) : 0;
}

FSIM_API int fsim_vehicle_capability(fsim_world* world, uint32_t id, uint32_t index, fsim_capability_info* out) {
    const auto* d = out ? capabilityAt(world, id, index) : nullptr;
    if (!d) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_capability: no capability " + std::to_string(index) + " on vehicle " + std::to_string(id));
    out->id = world->intern(d->id);
    out->version = d->version;
    out->kind = static_cast<int32_t>(d->kind);
    out->interactions = d->interactions;
    out->level = static_cast<int32_t>(d->level);
    out->axes = d->axes;
    out->axis_groups = d->axisGroups;
    out->terminating = d->persistence == fsim::control::Persistence::Terminating ? 1 : 0;
    out->needs_target = d->needsTarget ? 1 : 0;
    out->parameter_count = static_cast<uint32_t>(d->parameters.size());
    out->behavior = world->intern(d->behavior);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_capability_parameter(fsim_world* world, uint32_t id, uint32_t capability, uint32_t index, fsim_parameter_info* out) {
    const auto* d = out ? capabilityAt(world, id, capability) : nullptr;
    if (!d || index >= d->parameters.size()) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_capability_parameter: no such parameter");
    const auto& p = d->parameters[index];
    out->name = world->intern(p.name);
    out->unit = world->intern(p.unit);
    out->min = p.min;
    out->max = p.max;
    out->default_value = p.defaultValue;
    out->optional = p.optional ? 1 : 0;
    out->unsupported = p.supported ? 0 : 1;
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_capability_status(const fsim_world* world, uint32_t id, const char* capability, int32_t* availability, int32_t* reason) {
    if (!world || !capability) return FSIM_INVALID_ARGUMENT;
    const auto s = world->world.capabilityStatus(id, capability);
    if (availability) *availability = static_cast<int32_t>(s.availability);
    if (reason) *reason = static_cast<int32_t>(s.reason);
    return FSIM_OK;
}

FSIM_API void fsim_performance_init(fsim_performance* performance) {
    if (!performance) return;
    *performance = fsim_performance{};
    performance->struct_size = sizeof *performance;
}

FSIM_API int fsim_vehicle_performance(fsim_world* world, uint32_t id, fsim_performance* out) {
    const fsim::control::Performance* p = world && out ? world->world.performance(id) : nullptr;
    if (!p) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_performance: no vehicle with id " + std::to_string(id));
    fsim_performance c{};
    c.struct_size = sizeof c;
    c.revision = p->revision;
    c.hovers = p->hovers ? 1 : 0;
    c.min_cas_ms = p->minCasMs, c.max_cas_ms = p->maxCasMs, c.max_mach = p->maxMach, c.max_tas_ms = p->maxTasMs;
    c.cruise_tas_ms = p->cruiseTasMs, c.max_ground_speed_ms = p->maxGroundSpeedMs, c.ceiling_m = p->ceilingM;
    c.max_bank_rad = p->maxBankRad, c.min_pitch_rad = p->minPitchRad, c.max_pitch_rad = p->maxPitchRad, c.max_roll_rate_rad_s = p->maxRollRateRadS;
    c.min_load_factor = p->minLoadFactor, c.max_load_factor = p->maxLoadFactor;
    c.max_tilt_rad = p->maxTiltRad, c.max_acceleration_ms2 = p->maxAccelerationMs2, c.max_deceleration_ms2 = p->maxDecelerationMs2;
    c.max_climb_ms = p->maxClimbMs, c.max_descent_ms = p->maxDescentMs;
    c.altitude_gain_per_s = p->altitudeGainPerS, c.heading_gain = p->headingGain, c.heading_reference_tas_ms = p->headingReferenceTasMs;
    c.bank_rate_rad_s = p->bankRateRadS, c.velocity_bandwidth_rad_s = p->velocityBandwidthRadS;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

namespace {

/// An authority call's answer: FSIM_OK, or why the call itself was malformed (an unknown vehicle or capability).
int authority(const char* call, fsim::control::Reason r) {
    using fsim::control::Reason;
    if (r == Reason::UnknownVehicle || r == Reason::UnknownCapability || r == Reason::InvalidParameter || r == Reason::NotSupported ||
        r == Reason::NotImplemented) // (a capability the vehicle does not offer: why)
        return fail(FSIM_INVALID_ARGUMENT, std::string(call) + ": " + (r == Reason::InvalidParameter ? "not a reason the platform gives there" : fsim::control::reasonName(r)));
    return FSIM_OK;
}

bool validReason(int reason) { return reason >= 0 && reason < static_cast<int>(fsim::control::Reason::Count); }

} // namespace

FSIM_API int fsim_vehicle_set_control_mode(fsim_world* world, uint32_t id, int mode) {
    if (!world || mode < FSIM_CONTROL_OPEN || mode > FSIM_CONTROL_GRANTED) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_control_mode: bad arguments");
    return authority("fsim_vehicle_set_control_mode", world->world.setControlMode(id, static_cast<fsim::control::ControlMode>(mode)));
}

FSIM_API int fsim_vehicle_control_mode(const fsim_world* world, uint32_t id, int32_t* mode) {
    if (!world || !mode) return FSIM_INVALID_ARGUMENT;
    *mode = static_cast<int32_t>(world->world.controlMode(id));
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_request_control(fsim_world* world, uint32_t id, const char* capability, int32_t* reason) {
    return fsim_vehicle_request_control_by(world, id, capability, 0, reason);
}

FSIM_API int fsim_vehicle_request_control_by(fsim_world* world, uint32_t id, const char* capability, uint32_t controller, int32_t* reason) {
    if (!world || !capability || !reason) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_request_control: bad arguments");
    const fsim::control::Reason r = world->world.requestControl(id, capability, controller);
    *reason = static_cast<int32_t>(r);
    return authority("fsim_vehicle_request_control", r);
}

FSIM_API int fsim_vehicle_release_control(fsim_world* world, uint32_t id, const char* capability) {
    if (!world || !capability) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_release_control: bad arguments");
    return authority("fsim_vehicle_release_control", world->world.releaseControl(id, capability));
}

FSIM_API int fsim_vehicle_release_control_by(fsim_world* world, uint32_t id, const char* capability, uint32_t controller, int32_t* reason) {
    if (!world || !capability || !reason) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_release_control: bad arguments");
    const fsim::control::Reason r = world->world.releaseControl(id, capability, controller);
    *reason = static_cast<int32_t>(r);
    return authority("fsim_vehicle_release_control", r);
}

FSIM_API int fsim_vehicle_revoke_control(fsim_world* world, uint32_t id, const char* capability, int reason) {
    if (!world || !capability || !validReason(reason)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_revoke_control: bad arguments");
    return authority("fsim_vehicle_revoke_control", world->world.revokeControl(id, capability, static_cast<fsim::control::Reason>(reason)));
}

FSIM_API int fsim_vehicle_set_allowed(fsim_world* world, uint32_t id, const char* capability, int allowed) {
    if (!world || !capability) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_allowed: bad arguments");
    return authority("fsim_vehicle_set_allowed", world->world.setAllowed(id, capability, allowed != 0));
}

FSIM_API int fsim_vehicle_set_capability_precedence(fsim_world* world, uint32_t id, const char* capability, uint32_t precedence) {
    if (!world || !capability) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_capability_precedence: bad arguments");
    return authority("fsim_vehicle_set_capability_precedence", world->world.setCapabilityPrecedence(id, capability, precedence));
}

FSIM_API int fsim_vehicle_capability_precedence(const fsim_world* world, uint32_t id, const char* capability, uint32_t* precedence) {
    if (!world || !capability || !precedence) return FSIM_INVALID_ARGUMENT;
    const fsim::control::Reason known = world->world.capabilityStatus(id, capability).reason; // (an unknown vehicle, one it does not offer)
    if (known == fsim::control::Reason::UnknownVehicle || known == fsim::control::Reason::UnknownCapability ||
        known == fsim::control::Reason::NotSupported || known == fsim::control::Reason::NotImplemented)
        return fail(FSIM_INVALID_ARGUMENT, std::string("fsim_vehicle_capability_precedence: ") + fsim::control::reasonName(known));
    *precedence = world->world.capabilityPrecedence(id, capability);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_control_status(const fsim_world* world, uint32_t id, const char* capability, int32_t* allowed, int32_t* granted) {
    if (!world || !capability) return FSIM_INVALID_ARGUMENT;
    const fsim::control::Reason known = world->world.capabilityStatus(id, capability).reason; // (an unknown vehicle, one it does not offer)
    if (known == fsim::control::Reason::UnknownVehicle || known == fsim::control::Reason::UnknownCapability ||
        known == fsim::control::Reason::NotSupported || known == fsim::control::Reason::NotImplemented)
        return fail(FSIM_INVALID_ARGUMENT, std::string("fsim_vehicle_control_status: ") + fsim::control::reasonName(known));
    const fsim::control::ControlStatus s = world->world.controlStatus(id, capability);
    if (allowed) *allowed = s.allowed ? 1 : 0;
    if (granted) *granted = s.granted ? 1 : 0;
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_control_holder(const fsim_world* world, uint32_t id, const char* capability, int32_t* granted, uint32_t* holder) {
    if (!world || !capability) return FSIM_INVALID_ARGUMENT;
    if (const int r = fsim_vehicle_control_status(world, id, capability, nullptr, granted); r != FSIM_OK) return r;
    if (holder) *holder = world->world.controlStatus(id, capability).holder;
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_set_availability(fsim_world* world, uint32_t id, const char* capability, int availability, int reason) {
    return fsim_vehicle_set_availability_ex(world, id, capability, availability, reason, 0, std::numeric_limits<double>::quiet_NaN());
}

FSIM_API int fsim_vehicle_set_availability_ex(fsim_world* world, uint32_t id, const char* capability, int availability, int reason,
                                              uint64_t associated, double next_available_s) {
    if (!world || !capability || availability < FSIM_AVAILABLE || availability > FSIM_EXPENDED || !validReason(reason))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_availability: bad arguments");
    const auto r = static_cast<fsim::control::Reason>(reason == 0 ? static_cast<int>(fsim::control::Reason::Restricted) : reason);
    return authority("fsim_vehicle_set_availability", world->world.setAvailability(id, capability, static_cast<fsim::control::Availability>(availability),
                                                                                   r, associated, next_available_s));
}

FSIM_API int fsim_vehicle_control_revision(fsim_world* world, uint32_t id, uint32_t* revision) {
    if (!world || !revision || !world->world.performance(id)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_control_revision: no vehicle with id " + std::to_string(id));
    *revision = world->world.controlRevision(id);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_submit(fsim_world* world, uint32_t id, int level, const double* fields, uint32_t count,
                                 const fsim_command_options* options, fsim_command_result* result) {
    fsim::control::Command c;
    if (!world || !result || !toCommand(level, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit: level " + std::to_string(level) + " takes " + fieldCounts(level) + " fields");
    toC(world, id, world->world.submit(id, c, fromC(options)), result);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_submit_behavior(fsim_world* world, uint32_t id, const fsim_behavior_command* command,
                                          const fsim_command_options* options, fsim_command_result* result) {
    if (!world || !command || !command->id || !result) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vehicle_submit_behavior", [&] {
        toC(world, id, world->world.submit(id, toBehavior(command), fromC(options)), result);
        return FSIM_OK;
    });
}

FSIM_API uint32_t fsim_mode_field_count(int mode) {
    if (mode == FSIM_MODE_MARSHALL) { // (its 13 and its pattern's shape's: ABI 1.43)
        fsim::control::MarshallCommand m;
        double* slots[fsim::control::kMaxCommandFields];
        return static_cast<uint32_t>(fsim::control::marshallFields(m, slots) + fsim::control::PatternShape::kFields);
    }
    fsim::control::Command c;
    if (mode == FSIM_MODE_HSA) c = fsim::control::HsaCommand{};
    else if (mode == FSIM_MODE_ROUTE) c = fsim::control::RouteCommand{};
    else if (mode == FSIM_MODE_PATTERN) c = fsim::control::PatternCommand{};
    else if (mode == FSIM_MODE_CURVE) c = fsim::control::CurveCommand{};
    else if (mode == FSIM_MODE_MUST_FLY) c = fsim::control::MustFlyCommand{};
    else return 0;
    double* slots[fsim::control::kMaxCommandFields];
    const auto n = static_cast<uint32_t>(fsim::control::commandFields(c, slots));
    if (mode == FSIM_MODE_PATTERN) return n + static_cast<uint32_t>(fsim::control::PatternShape::kFields); // (and its shape's)
    if (mode == FSIM_MODE_CURVE) return n + static_cast<uint32_t>(fsim::control::CurveShape::kFields);
    return n;
}

FSIM_API void fsim_waypoint_init(fsim_waypoint* waypoint) {
    if (!waypoint) return;
    const double hold = fsim::control::kHold;
    *waypoint = fsim_waypoint{};
    waypoint->struct_size = sizeof *waypoint;
    waypoint->altitude_m = waypoint->altitude_reference = waypoint->speed = waypoint->speed_reference = hold;
    waypoint->turn = FSIM_TURN_FLY_BY;
    waypoint->max_bank_rad = waypoint->climb_rate_ms = hold;
    waypoint->altitude_min_m = waypoint->altitude_max_m = waypoint->kind = waypoint->waypoint_type = hold;
    waypoint->frame = waypoint->frame_rotation = waypoint->frame_offsets = waypoint->frame_x_m = waypoint->frame_y_m = waypoint->frame_z_m = hold;
    waypoint->course_rad = waypoint->turn_radius_m = hold;
    waypoint->speed_optimization = waypoint->climb_optimization = waypoint->acceleration_ms2 = hold;
    waypoint->arrival_begin_s = waypoint->arrival_end_s = hold;
    waypoint->rnp_m = waypoint->next = hold;
    waypoint->terminator = hold;
}

FSIM_API int fsim_vehicle_submit_route(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_waypoint* waypoints,
                                       uint32_t waypoint_count, const fsim_command_options* options, fsim_command_result* result) {
    fsim::control::Command c;
    if (!world || !result || !toMode(FSIM_MODE_ROUTE, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route: a route takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_ROUTE)) + " fields");
    try { // (no std::function: an UPDATE every step allocates nothing)
        if (!toWaypoints(world, waypoints, waypoint_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route: waypoints without their struct_size");
        toC(world, id, world->world.submit(id, std::get<fsim::control::RouteCommand>(c), world->waypoints, fromC(options)), result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_vehicle_submit_route: ") + e.what());
    }
}

FSIM_API int fsim_activity_update_route(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                        const fsim_waypoint* waypoints, uint32_t waypoint_count, fsim_command_result* result) {
    return fsim_activity_update_route_by(world, activity, FSIM_SOURCE_POLICY, 0, fields, count, waypoints, waypoint_count, result);
}

FSIM_API int fsim_activity_update_route_as(fsim_world* world, fsim_activity_id activity, int source, const double* fields, uint32_t count,
                                           const fsim_waypoint* waypoints, uint32_t waypoint_count, fsim_command_result* result) {
    return fsim_activity_update_route_by(world, activity, source, 0, fields, count, waypoints, waypoint_count, result);
}

FSIM_API int fsim_activity_update_route_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                           uint32_t count, const fsim_waypoint* waypoints, uint32_t waypoint_count, fsim_command_result* result) {
    fsim::control::Command c;
    fsim::control::Source from;
    if (!world || !result || !toSource(source, from)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route: bad arguments");
    const fsim::control::Caller caller{from, controller};
    if (!toMode(FSIM_MODE_ROUTE, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route: a route takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_ROUTE)) + " fields");
    try {
        if (!toWaypoints(world, waypoints, waypoint_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route: waypoints without their struct_size");
        toC(world, fsim::control::activityVehicle(activity), world->world.update(caller, activity, std::get<fsim::control::RouteCommand>(c), world->waypoints), result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_activity_update_route: ") + e.what());
    }
}

FSIM_API void fsim_route_loiter_init(fsim_route_loiter* loiter) {
    if (!loiter) return;
    *loiter = fsim_route_loiter{};
    loiter->struct_size = sizeof *loiter;
    for (double& f : loiter->fields) f = std::numeric_limits<double>::quiet_NaN();
    loiter->end_time_s = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_vehicle_submit_route_loiters(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_waypoint* waypoints,
                                               uint32_t waypoint_count, const fsim_route_loiter* loiters, uint32_t loiter_count,
                                               const fsim_command_options* options, fsim_command_result* result) {
    fsim::control::Command c;
    if (!world || !result || !toMode(FSIM_MODE_ROUTE, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_loiters: a route takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_ROUTE)) + " fields");
    try {
        if (!toWaypoints(world, waypoints, waypoint_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_loiters: waypoints without their struct_size");
        if (!toLoiters(world, loiters, loiter_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_loiters: loiters without their struct_size");
        toC(world, id, world->world.submit(id, std::get<fsim::control::RouteCommand>(c), world->waypoints, fromC(options), world->loiters), result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_vehicle_submit_route_loiters: ") + e.what());
    }
}

FSIM_API int fsim_activity_update_route_loiters(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                                uint32_t count, const fsim_waypoint* waypoints, uint32_t waypoint_count,
                                                const fsim_route_loiter* loiters, uint32_t loiter_count, fsim_command_result* result) {
    fsim::control::Command c;
    fsim::control::Source from;
    if (!world || !result || !toSource(source, from)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_loiters: bad arguments");
    if (!toMode(FSIM_MODE_ROUTE, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_loiters: a route takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_ROUTE)) + " fields");
    try {
        if (!toWaypoints(world, waypoints, waypoint_count))
            return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_loiters: waypoints without their struct_size");
        if (!toLoiters(world, loiters, loiter_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_loiters: loiters without their struct_size");
        toC(world, fsim::control::activityVehicle(activity),
            world->world.update(fsim::control::Caller{from, controller}, activity, std::get<fsim::control::RouteCommand>(c), world->waypoints, world->loiters),
            result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_activity_update_route_loiters: ") + e.what());
    }
}

FSIM_API void fsim_route_state_init(fsim_route_state* state) {
    if (!state) return;
    *state = fsim_route_state{};
    state->struct_size = sizeof *state;
    for (double& f : state->fields) f = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_vehicle_submit_route_states(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_waypoint* waypoints,
                                              uint32_t waypoint_count, const fsim_route_loiter* loiters, uint32_t loiter_count,
                                              const fsim_route_state* states, uint32_t state_count, const fsim_command_options* options,
                                              fsim_command_result* result) {
    fsim::control::Command c;
    if (!world || !result || !toMode(FSIM_MODE_ROUTE, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_states: a route takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_ROUTE)) + " fields");
    try {
        if (!toWaypoints(world, waypoints, waypoint_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_states: waypoints without their struct_size");
        if (!toLoiters(world, loiters, loiter_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_states: loiters without their struct_size");
        if (!toStates(world, states, state_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_states: states without their struct_size");
        toC(world, id, world->world.submit(id, std::get<fsim::control::RouteCommand>(c), world->waypoints, fromC(options), world->loiters, world->states),
            result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_vehicle_submit_route_states: ") + e.what());
    }
}

FSIM_API int fsim_activity_update_route_states(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                               uint32_t count, const fsim_waypoint* waypoints, uint32_t waypoint_count,
                                               const fsim_route_loiter* loiters, uint32_t loiter_count, const fsim_route_state* states,
                                               uint32_t state_count, fsim_command_result* result) {
    fsim::control::Command c;
    fsim::control::Source from;
    if (!world || !result || !toSource(source, from)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_states: bad arguments");
    if (!toMode(FSIM_MODE_ROUTE, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_states: a route takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_ROUTE)) + " fields");
    try {
        if (!toWaypoints(world, waypoints, waypoint_count))
            return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_states: waypoints without their struct_size");
        if (!toLoiters(world, loiters, loiter_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_states: loiters without their struct_size");
        if (!toStates(world, states, state_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_states: states without their struct_size");
        toC(world, fsim::control::activityVehicle(activity),
            world->world.update(fsim::control::Caller{from, controller}, activity, std::get<fsim::control::RouteCommand>(c), world->waypoints, world->loiters,
                                world->states),
            result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_activity_update_route_states: ") + e.what());
    }
}

FSIM_API void fsim_route_path_init(fsim_route_path* path) {
    if (!path) return;
    *path = fsim_route_path{};
    path->struct_size = sizeof *path;
    path->type = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API void fsim_route_branch_init(fsim_route_branch* branch) {
    if (!branch) return;
    *branch = fsim_route_branch{};
    branch->struct_size = sizeof *branch;
    std::fill_n(branch->fields, fsim::control::RouteBranch::kFields, std::numeric_limits<double>::quiet_NaN());
}

FSIM_API void fsim_route_terminator_init(fsim_route_terminator* terminator) {
    if (!terminator) return;
    *terminator = fsim_route_terminator{};
    terminator->struct_size = sizeof *terminator;
    std::fill_n(terminator->fields, fsim::control::RouteTerminator::kFields, std::numeric_limits<double>::quiet_NaN());
}

FSIM_API void fsim_route_extras_init(fsim_route_extras* extras) {
    if (!extras) return;
    *extras = fsim_route_extras{};
    extras->struct_size = sizeof *extras;
}

FSIM_API int fsim_vehicle_submit_route_extras(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_waypoint* waypoints,
                                              uint32_t waypoint_count, const fsim_route_extras* extras, const fsim_command_options* options,
                                              fsim_command_result* result) {
    fsim::control::Command c;
    if (!world || !result || !toMode(FSIM_MODE_ROUTE, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_extras: a route takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_ROUTE)) + " fields");
    try {
        if (!toWaypoints(world, waypoints, waypoint_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_extras: waypoints without their struct_size");
        if (!toExtras(world, extras)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_route_extras: extras without their struct_size");
        toC(world, id,
            world->world.submit(id, std::get<fsim::control::RouteCommand>(c), world->waypoints, fromC(options), world->loiters, world->states, world->paths,
                                world->branches, world->terminators),
            result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_vehicle_submit_route_extras: ") + e.what());
    }
}

FSIM_API int fsim_activity_update_route_extras(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                               uint32_t count, const fsim_waypoint* waypoints, uint32_t waypoint_count, const fsim_route_extras* extras,
                                               fsim_command_result* result) {
    fsim::control::Command c;
    fsim::control::Source from;
    if (!world || !result || !toSource(source, from)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_extras: bad arguments");
    if (!toMode(FSIM_MODE_ROUTE, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_extras: a route takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_ROUTE)) + " fields");
    try {
        if (!toWaypoints(world, waypoints, waypoint_count))
            return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_extras: waypoints without their struct_size");
        if (!toExtras(world, extras)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_route_extras: extras without their struct_size");
        toC(world, fsim::control::activityVehicle(activity),
            world->world.update(fsim::control::Caller{from, controller}, activity, std::get<fsim::control::RouteCommand>(c), world->waypoints, world->loiters,
                                world->states, world->paths, world->branches, world->terminators),
            result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_activity_update_route_extras: ") + e.what());
    }
}

FSIM_API void fsim_bezier_segment_init(fsim_bezier_segment* segment) {
    if (!segment) return;
    *segment = fsim_bezier_segment{};
    segment->struct_size = sizeof *segment;
}

FSIM_API int fsim_vehicle_submit_curve(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_bezier_segment* segments,
                                       uint32_t segment_count, const fsim_command_options* options, fsim_command_result* result) {
    fsim::control::Command c;
    fsim::control::CurveShape shape;
    if (!world || !result || !toMode(FSIM_MODE_CURVE, fields, count, c, nullptr, &shape))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_curve: a curve takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_CURVE)) + " fields");
    try {
        if (!toSegments(world, segments, segment_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_curve: segments without their struct_size");
        toC(world, id, world->world.submit(id, std::get<fsim::control::CurveCommand>(c), world->segments, fromC(options), count > kCurveFields ? &shape : nullptr),
            result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_vehicle_submit_curve: ") + e.what());
    }
}

FSIM_API void fsim_nurbs_segment_init(fsim_nurbs_segment* segment) {
    if (!segment) return;
    *segment = fsim_nurbs_segment{};
    segment->struct_size = sizeof *segment;
    for (double& w : segment->weight) w = 1.0;
    segment->curvature = segment->first_index = segment->last_index = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_vehicle_submit_nurbs(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_nurbs_segment* segments,
                                       uint32_t segment_count, const fsim_command_options* options, fsim_command_result* result) {
    fsim::control::Command c;
    fsim::control::CurveShape shape;
    if (!world || !result || !toMode(FSIM_MODE_CURVE, fields, count, c, nullptr, &shape))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_nurbs: a curve takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_CURVE)) + " fields");
    try {
        if (!toNurbs(world, segments, segment_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_nurbs: segments without their struct_size");
        toC(world, id,
            world->world.submit(id, std::get<fsim::control::CurveCommand>(c), fsim::Span<const fsim::control::NurbsSegment>(world->nurbs), fromC(options),
                                count > kCurveFields ? &shape : nullptr),
            result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_vehicle_submit_nurbs: ") + e.what());
    }
}

FSIM_API int fsim_activity_update_nurbs(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                        const fsim_nurbs_segment* segments, uint32_t segment_count, fsim_command_result* result) {
    return fsim_activity_update_nurbs_by(world, activity, FSIM_SOURCE_POLICY, 0, fields, count, segments, segment_count, result);
}

FSIM_API int fsim_activity_update_nurbs_as(fsim_world* world, fsim_activity_id activity, int source, const double* fields, uint32_t count,
                                           const fsim_nurbs_segment* segments, uint32_t segment_count, fsim_command_result* result) {
    return fsim_activity_update_nurbs_by(world, activity, source, 0, fields, count, segments, segment_count, result);
}

FSIM_API int fsim_activity_update_nurbs_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                           uint32_t count, const fsim_nurbs_segment* segments, uint32_t segment_count, fsim_command_result* result) {
    fsim::control::Command c;
    fsim::control::Source from;
    if (!world || !result || !toSource(source, from)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_nurbs: bad arguments");
    const fsim::control::Caller caller{from, controller};
    fsim::control::CurveShape shape;
    if (!toMode(FSIM_MODE_CURVE, fields, count, c, nullptr, &shape))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_nurbs: a curve takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_CURVE)) + " fields");
    try {
        if (!toNurbs(world, segments, segment_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_nurbs: segments without their struct_size");
        toC(world, fsim::control::activityVehicle(activity),
            world->world.update(caller, activity, std::get<fsim::control::CurveCommand>(c), fsim::Span<const fsim::control::NurbsSegment>(world->nurbs),
                                count > kCurveFields ? &shape : nullptr),
            result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_activity_update_nurbs: ") + e.what());
    }
}

FSIM_API int fsim_activity_update_curve(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                        const fsim_bezier_segment* segments, uint32_t segment_count, fsim_command_result* result) {
    return fsim_activity_update_curve_by(world, activity, FSIM_SOURCE_POLICY, 0, fields, count, segments, segment_count, result);
}

FSIM_API int fsim_activity_update_curve_as(fsim_world* world, fsim_activity_id activity, int source, const double* fields, uint32_t count,
                                           const fsim_bezier_segment* segments, uint32_t segment_count, fsim_command_result* result) {
    return fsim_activity_update_curve_by(world, activity, source, 0, fields, count, segments, segment_count, result);
}

FSIM_API int fsim_activity_update_curve_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                           uint32_t count, const fsim_bezier_segment* segments, uint32_t segment_count, fsim_command_result* result) {
    fsim::control::Command c;
    fsim::control::Source from;
    if (!world || !result || !toSource(source, from)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_curve: bad arguments");
    const fsim::control::Caller caller{from, controller};
    fsim::control::CurveShape shape;
    if (!toMode(FSIM_MODE_CURVE, fields, count, c, nullptr, &shape))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_curve: a curve takes " + std::to_string(fsim_mode_field_count(FSIM_MODE_CURVE)) + " fields");
    try {
        if (!toSegments(world, segments, segment_count)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_curve: segments without their struct_size");
        toC(world, fsim::control::activityVehicle(activity),
            world->world.update(caller, activity, std::get<fsim::control::CurveCommand>(c), world->segments, count > kCurveFields ? &shape : nullptr), result);
        return FSIM_OK;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string("fsim_activity_update_curve: ") + e.what());
    }
}

FSIM_API int fsim_vehicle_submit_mode(fsim_world* world, uint32_t id, int mode, const double* fields, uint32_t count,
                                      const fsim_command_options* options, fsim_command_result* result) {
    fsim::control::Command c;
    fsim::control::PatternShape shape;
    if (fsim::control::MarshallCommand m; world && result && mode == FSIM_MODE_MARSHALL) { // (ABI 1.43: its slot the world's)
        if (!toMarshall(fields, count, m, &shape))
            return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_mode: mode " + std::to_string(mode) + " takes " +
                                                   std::to_string(fsim_mode_field_count(mode)) + " fields");
        toC(world, id, world->world.submit(id, m, count > kPatternFields ? shape : fsim::control::PatternShape{}, fromC(options)), result);
        return FSIM_OK;
    }
    if (!world || !result || !toMode(mode, fields, count, c, &shape))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_mode: mode " + std::to_string(mode) + " takes " +
                                               std::to_string(fsim_mode_field_count(mode)) + " fields");
    if (const auto* pattern = std::get_if<fsim::control::PatternCommand>(&c); pattern && count > kPatternFields)
        toC(world, id, world->world.submit(id, *pattern, shape, fromC(options)), result);
    else
        toC(world, id, world->world.submit(id, c, fromC(options)), result);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_submit_support(fsim_world* world, uint32_t id, int kind, const double* fields, uint32_t count,
                                         const fsim_command_options* options, fsim_command_result* result) {
    fsim::control::SupportCommand c;
    if (!world || !result || !toSupport(kind, fields, count, c))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_support: support kind " + std::to_string(kind) + " and " +
                                               std::to_string(count) + " fields do not match");
    toC(world, id, world->world.submit(id, c, fromC(options)), result);
    return FSIM_OK;
}

FSIM_API int fsim_activity_update(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count, fsim_command_result* result) {
    return fsim_activity_update_by(world, activity, FSIM_SOURCE_POLICY, 0, fields, count, result);
}

FSIM_API int fsim_activity_update_as(fsim_world* world, fsim_activity_id activity, int source, const double* fields, uint32_t count,
                                     fsim_command_result* result) {
    return fsim_activity_update_by(world, activity, source, 0, fields, count, result);
}

FSIM_API int fsim_activity_update_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                     uint32_t count, fsim_command_result* result) {
    fsim::control::Source from;
    if (!world || !result || !toSource(source, from)) return FSIM_INVALID_ARGUMENT;
    const UpdateShape shape = updateShape(world, activity);
    bool malformed = false;
    const auto r = updateFrom(world, activity, shape, fields, count, malformed, fsim::control::Caller{from, controller});
    if (malformed)
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update: activity " + std::to_string(activity) + " takes " +
                                               (shape.support >= 0 || shape.mode >= 0 ? std::to_string(shape.fields) : fieldCounts(shape.level)) + " fields");
    toC(world, fsim::control::activityVehicle(activity), r, result);
    return FSIM_OK;
}

FSIM_API int fsim_activity_update_batch(fsim_world* world, const fsim_activity_id* activities, uint32_t count, const double* values, uint32_t stride) {
    if (!world || (count && (!activities || !values))) return FSIM_INVALID_ARGUMENT;
    std::size_t offset = 0;
    for (uint32_t i = 0; i < count; ++i) {
        const UpdateShape shape = updateShape(world, activities[i]);
        const uint32_t fields = shape.fields;
        bool malformed = false;
        const auto r = updateFrom(world, activities[i], shape, values + offset, fields, malformed);
        remember(world, fsim::control::activityVehicle(activities[i]), r);
        if (!r.accepted())
            return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_batch: activity " + std::to_string(activities[i]) + " refused (" +
                                                   fsim::control::reasonName(r.reason) + ")");
        offset += stride ? stride : fields;
    }
    fsim::sdk::lastError().clear();
    return FSIM_OK;
}

FSIM_API int fsim_activity_update_batch_n(fsim_world* world, const fsim_activity_id* activities, uint32_t count, const double* values,
                                          uint32_t stride, uint32_t fields) {
    if (!world || (count && (!activities || !values))) return FSIM_INVALID_ARGUMENT;
    std::size_t offset = 0;
    for (uint32_t i = 0; i < count; ++i) {
        const UpdateShape shape = updateShape(world, activities[i]);
        bool malformed = false;
        const auto r = updateFrom(world, activities[i], shape, values + offset, fields, malformed);
        remember(world, fsim::control::activityVehicle(activities[i]), r);
        if (malformed)
            return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_batch_n: activity " + std::to_string(activities[i]) + " takes " +
                                                   (shape.support >= 0 ? std::to_string(shape.fields) : fieldCounts(shape.level)) + " fields, not " +
                                                   std::to_string(fields));
        if (!r.accepted())
            return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_batch_n: activity " + std::to_string(activities[i]) + " refused (" +
                                                   fsim::control::reasonName(r.reason) + ")");
        offset += stride ? stride : fields;
    }
    fsim::sdk::lastError().clear();
    return FSIM_OK;
}

FSIM_API int fsim_activity_cancel(fsim_world* world, fsim_activity_id activity, fsim_command_result* result) {
    return fsim_activity_cancel_by(world, activity, FSIM_SOURCE_POLICY, 0, result);
}

FSIM_API int fsim_activity_cancel_as(fsim_world* world, fsim_activity_id activity, int source, fsim_command_result* result) {
    return fsim_activity_cancel_by(world, activity, source, 0, result);
}

FSIM_API int fsim_activity_cancel_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, fsim_command_result* result) {
    fsim::control::Source from;
    if (!world || !result || !toSource(source, from)) return FSIM_INVALID_ARGUMENT;
    toC(world, fsim::control::activityVehicle(activity), world->world.cancel(fsim::control::Caller{from, controller}, activity), result);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_set_default(fsim_world* world, uint32_t id, int mode, int32_t* reason) {
    if (!world || (mode != FSIM_DEFAULT_NEUTRAL && mode != FSIM_DEFAULT_HOLD)) return FSIM_INVALID_ARGUMENT;
    const auto r = world->world.setVehicleDefault(id, static_cast<fsim::control::VehicleDefault>(mode));
    if (reason) *reason = static_cast<int32_t>(r);
    if (r == fsim::control::Reason::None) return FSIM_OK;
    return fail(FSIM_INVALID_ARGUMENT, std::string("fsim_vehicle_set_default: refused (") + fsim::control::reasonName(r) + ")");
}

FSIM_API int fsim_vehicle_get_default(const fsim_world* world, uint32_t id, int32_t* mode) {
    if (!world || !mode || !world->world.vehicleState(id)) return FSIM_INVALID_ARGUMENT;
    *mode = static_cast<int32_t>(world->world.vehicleDefault(id));
    return FSIM_OK;
}

static_assert(FSIM_LIMIT_COUNT == fsim::control::kLimitCount, "fsim_envelope_status has a row per limit");

FSIM_API int fsim_vehicle_set_protection(fsim_world* world, uint32_t id, int mode) {
    if (!world || mode < FSIM_PROTECTION_OFF || mode > FSIM_PROTECTION_LIMIT) return FSIM_INVALID_ARGUMENT;
    if (world->world.setProtection(id, static_cast<fsim::control::ProtectionMode>(mode)) != fsim::control::Reason::None)
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_protection: no vehicle " + std::to_string(id));
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_get_protection(const fsim_world* world, uint32_t id, int32_t* mode) {
    if (!world || !mode || !world->world.vehicleState(id)) return FSIM_INVALID_ARGUMENT;
    *mode = static_cast<int32_t>(world->world.protection(id));
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_envelope(fsim_world* world, uint32_t id, fsim_envelope_status* out) {
    if (!world || !out || !world->world.vehicleState(id)) return FSIM_INVALID_ARGUMENT;
    const fsim::control::EnvelopeStatus s = world->world.envelope(id);
    std::memset(out, 0, sizeof *out);
    out->mode = static_cast<int32_t>(s.mode);
    for (std::size_t l = 0; l < fsim::control::kLimitCount; ++l) {
        out->limited_updates[l] = s.limits[l].limitedUpdates;
        out->exceeded_updates[l] = s.limits[l].exceededUpdates;
        out->exceeded_s[l] = s.limits[l].exceededS;
        out->worst_excess[l] = s.limits[l].worstExcess;
    }
    return FSIM_OK;
}

FSIM_API const char* fsim_limit_name(int limit) {
    if (limit < 0 || limit >= static_cast<int>(fsim::control::kLimitCount)) return "?";
    return fsim::control::limitName(static_cast<fsim::control::Limit>(limit));
}

FSIM_API int fsim_activity_get(const fsim_world* world, fsim_activity_id activity, fsim_activity_info* out) {
    const auto* a = world && out ? world->world.activity(activity) : nullptr;
    if (!a) return FSIM_INVALID_ARGUMENT;
    infoToC(*a, out);
    return FSIM_OK;
}

FSIM_API uint32_t fsim_vehicle_activity_count(const fsim_world* world, uint32_t id) {
    return world ? static_cast<uint32_t>(world->world.activities(id).size()) : 0;
}

FSIM_API int fsim_vehicle_activity(const fsim_world* world, uint32_t id, uint32_t index, fsim_activity_info* out) {
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const auto all = world->world.activities(id);
    if (index >= all.size()) return FSIM_INVALID_ARGUMENT;
    infoToC(all[index], out);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_profile_value(const fsim_world* world, uint32_t id, const char* path, double* value) {
    const auto* p = world && path && value ? world->world.profile(id) : nullptr;
    if (!p) return FSIM_INVALID_ARGUMENT;
    *value = fsim::control::profileValue(*p, path);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_profile_section(const fsim_world* world, uint32_t id, const char* section, uint32_t* version, int32_t* provenance) {
    const auto* p = world && section ? world->world.profile(id) : nullptr;
    const auto* h = p ? fsim::control::sectionHeader(*p, section) : nullptr;
    if (!h) return FSIM_INVALID_ARGUMENT;
    if (version) *version = h->version;
    if (provenance) *provenance = static_cast<int32_t>(h->provenance);
    return FSIM_OK;
}

FSIM_API const char* fsim_reason_name(int reason) {
    return reason >= 0 && reason < static_cast<int>(fsim::control::Reason::Count) ? fsim::control::reasonName(static_cast<fsim::control::Reason>(reason))
                                                                                  : "?";
}

FSIM_API const char* fsim_activity_state_name(int state) {
    return state >= 0 && state <= static_cast<int>(fsim::control::ActivityState::Deleted)
               ? fsim::control::activityStateName(static_cast<fsim::control::ActivityState>(state))
               : "?";
}

// --- The Vehicle Interface (ABI 1.6) -------------------------------------------------------

FSIM_API void fsim_command_detail_init(fsim_command_detail* d) {
    if (!d) return;
    std::memset(d, 0, sizeof *d);
    d->struct_size = sizeof *d;
    d->index = -1;
    d->from = d->to = std::numeric_limits<double>::quiet_NaN();
}

namespace {

/// The last command's details: its vehicle's (they stay until that vehicle's next NEW or UPDATE).
const fsim::control::CommandDetails* lastDetails(const fsim_world* world) noexcept {
    return world->lastVehicle ? world->world.commandDetails(world->lastVehicle) : nullptr;
}

/// An answer's detail, its counts aside.
fsim_command_detail detailOf(const fsim::control::CommandResult& r) noexcept {
    fsim_command_detail d;
    fsim_command_detail_init(&d);
    d.reason = static_cast<int32_t>(r.reason);
    d.index = r.index;
    d.constraint = static_cast<int32_t>(r.constraint);
    d.from = r.from, d.to = r.to;
    d.new_activity = r.newActivity ? 1 : 0;
    d.command_id = r.commandId;
    d.associated = r.other;
    d.description = fsim::control::reasonDescription(r.reason);
    return d;
}

} // namespace

FSIM_API int fsim_last_command_detail(const fsim_world* world, fsim_command_detail* out) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    fsim_command_detail d = detailOf(world->last);
    if (const auto* details = lastDetails(world))
        d.finding_count = details->findingCount, d.adjustment_count = details->adjustmentCount, d.suggestion = details->suggestion;
    return copyOut(d, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API void fsim_command_endurance_init(fsim_command_endurance* e) {
    if (!e) return;
    std::memset(e, 0, sizeof *e);
    e->struct_size = sizeof *e;
    e->remaining = e->required = e->remaining_s = e->required_s = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_last_command_endurance(const fsim_world* world, fsim_command_endurance* out) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    fsim_command_endurance e;
    fsim_command_endurance_init(&e);
    if (const auto* details = lastDetails(world); details && details->endurance.energy) {
        const auto& n = details->endurance;
        e.energy = n.energy;
        e.remaining = n.remaining, e.required = n.required, e.remaining_s = n.remainingS, e.required_s = n.requiredS;
    }
    return copyOut(e, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API void fsim_command_terrain_init(fsim_command_terrain* t) {
    if (!t) return;
    std::memset(t, 0, sizeof *t);
    t->struct_size = sizeof *t;
    t->index = -1;
    t->latitude_rad = t->longitude_rad = t->altitude_msl_m = t->ground_m = t->time_s = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_last_command_terrain(const fsim_world* world, fsim_command_terrain* out) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    fsim_command_terrain t;
    fsim_command_terrain_init(&t);
    if (const auto* details = lastDetails(world); details && details->terrain.hit) {
        const auto& h = details->terrain;
        t.hit = 1, t.index = h.index;
        t.latitude_rad = h.latitudeRad, t.longitude_rad = h.longitudeRad, t.altitude_msl_m = h.altitudeMslM, t.ground_m = h.groundM, t.time_s = h.timeS;
    }
    return copyOut(t, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_world_terrain(const fsim_world* world, uint32_t count, const double* latitude_rad, const double* longitude_rad, double* height_m) {
    if (!world || (count && (!latitude_rad || !longitude_rad || !height_m))) return FSIM_INVALID_ARGUMENT;
    int known = 0;
    for (uint32_t i = 0; i < count; ++i) {
        const std::optional<double> h = world->world.terrainHeightM(latitude_rad[i], longitude_rad[i]);
        height_m[i] = h ? *h : std::numeric_limits<double>::quiet_NaN();
        known += h ? 1 : 0;
    }
    return known;
}

FSIM_API void fsim_command_finding_init(fsim_command_finding* f) {
    if (!f) return;
    std::memset(f, 0, sizeof *f);
    f->struct_size = sizeof *f;
    f->index = -1;
    f->from = f->to = std::numeric_limits<double>::quiet_NaN();
    f->description = "";
}

FSIM_API int fsim_last_command_finding(const fsim_world* world, uint32_t index, fsim_command_finding* out) {
    const auto* details = world ? lastDetails(world) : nullptr;
    if (!details || index >= std::min<uint32_t>(details->findingCount, fsim::control::CommandDetails::kMax))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_last_command_finding: no finding " + std::to_string(index));
    const fsim::control::Finding& f = details->findings[index];
    fsim_command_finding c;
    fsim_command_finding_init(&c);
    c.reason = static_cast<int32_t>(f.reason);
    c.index = f.index;
    c.constraint = static_cast<int32_t>(f.constraint);
    c.from = f.from, c.to = f.to;
    c.associated = f.associated;
    c.description = f.description ? f.description : "";
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API void fsim_command_adjustment_init(fsim_command_adjustment* a) {
    if (!a) return;
    std::memset(a, 0, sizeof *a);
    a->struct_size = sizeof *a;
    a->index = a->field = -1;
    a->requested = a->adjusted = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_last_command_adjustment(const fsim_world* world, uint32_t index, fsim_command_adjustment* out) {
    const auto* details = world ? lastDetails(world) : nullptr;
    if (!details || index >= std::min<uint32_t>(details->adjustmentCount, fsim::control::CommandDetails::kMax))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_last_command_adjustment: no adjustment " + std::to_string(index));
    const fsim::control::Adjustment& a = details->adjustments[index];
    fsim_command_adjustment c;
    fsim_command_adjustment_init(&c);
    c.index = a.index;
    c.field = a.field;
    c.constraint = static_cast<int32_t>(a.constraint);
    c.requested = a.requested, c.adjusted = a.adjusted;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API void fsim_activity_envelope_init(fsim_activity_envelope* e) {
    if (!e) return;
    std::memset(e, 0, sizeof *e);
    e->struct_size = sizeof *e;
    e->interactive = 1;
    e->interrupt = 1;
    e->start_not_before = e->start_not_after = e->end_not_before = e->end_not_after = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_activity_get_envelope(const fsim_world* world, fsim_activity_id activity, fsim_activity_envelope* out) {
    const auto* a = world ? world->world.activity(activity) : nullptr;
    if (!a) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_get_envelope: no activity " + std::to_string(activity));
    fsim_activity_envelope c;
    fsim_activity_envelope_init(&c);
    c.interactive = a->interactive ? 1 : 0;
    c.command_id = a->commandId;
    for (std::size_t i = 0; i < fsim::control::kMaxRequirements; ++i) {
        c.trace[i].kind = static_cast<int32_t>(a->trace[i].kind);
        c.trace[i].id = a->trace[i].id;
    }
    c.waiting = static_cast<int32_t>(a->waiting);
    c.basis = static_cast<int32_t>(a->basis());
    c.rank_priority = a->rank.priority, c.rank_precedence = a->rank.precedence;
    c.precedence = a->precedence;
    c.waiting_for = a->waitingFor;
    c.interrupt = a->interrupt ? 1 : 0;
    c.criticality = static_cast<int32_t>(a->window.criticality);
    c.start_not_before = a->window.startNotBefore, c.start_not_after = a->window.startNotAfter;
    c.end_not_before = a->window.endNotBefore, c.end_not_after = a->window.endNotAfter;
    c.suggestion = a->suggestion, c.run = a->run, c.runs = a->runs;
    c.controller = a->controller;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_activity_command(fsim_world* world, fsim_activity_id activity, int command, uint32_t rank_priority, uint32_t rank_precedence,
                                   int source, uint32_t controller, fsim_command_result* result) {
    if (!world || !result || command < 0 || command >= static_cast<int>(fsim::control::ActivityCommand::Count) || source < 0 || source > 2 ||
        rank_priority > 0xFFFF || rank_precedence > 0xFFFF)
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_command: bad arguments");
    return guard("fsim_activity_command", [&]() -> int {
        const fsim::control::Rank rank{static_cast<std::uint16_t>(rank_priority), static_cast<std::uint16_t>(rank_precedence)};
        toC(world, fsim::control::activityVehicle(activity),
            world->world.activityCommand(fsim::control::Caller{static_cast<fsim::control::Source>(source), controller}, activity,
                                         static_cast<fsim::control::ActivityCommand>(command), rank),
            result);
        return FSIM_OK;
    });
}

FSIM_API int fsim_activity_command_branch(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, uint32_t branch, int commanded,
                                          fsim_command_result* result) {
    fsim::control::Source from;
    if (!world || !result || !toSource(source, from)) return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_command_branch: bad arguments");
    return guard("fsim_activity_command_branch", [&]() -> int {
        toC(world, fsim::control::activityVehicle(activity), world->world.commandBranch(fsim::control::Caller{from, controller}, activity, branch, commanded != 0),
            result);
        return FSIM_OK;
    });
}

FSIM_API const char* fsim_activity_command_name(int command) {
    return command >= 0 && command < static_cast<int>(fsim::control::ActivityCommand::Count)
               ? fsim::control::activityCommandName(static_cast<fsim::control::ActivityCommand>(command))
               : "?";
}

FSIM_API const char* fsim_activity_wait_name(int wait) {
    return wait >= 0 && wait < static_cast<int>(fsim::control::ActivityWait::Count) ? fsim::control::activityWaitName(static_cast<fsim::control::ActivityWait>(wait))
                                                                                     : "?";
}

FSIM_API const char* fsim_activity_basis_name(int basis) {
    return basis >= 0 && basis < static_cast<int>(fsim::control::ActivityBasis::Count)
               ? fsim::control::activityBasisName(static_cast<fsim::control::ActivityBasis>(basis))
               : "?";
}

FSIM_API const char* fsim_time_criticality_name(int criticality) {
    return criticality >= 0 && criticality < static_cast<int>(fsim::control::TimeCriticality::Count)
               ? fsim::control::timeCriticalityName(static_cast<fsim::control::TimeCriticality>(criticality))
               : "?";
}

namespace {

/// An operational zone from the caller's (ABI 1.40): false where its arrays cannot be read.
bool zoneFromC(const fsim_op_zone& c, fsim::control::OpZone& z) {
    if (c.struct_size < offsetof(fsim_op_zone, time_s) + sizeof c.time_s) return false;
    if ((c.vertex_count && !c.vertices) || (c.hole_count && (!c.holes || !c.hole_sizes)) || c.vertex_count > 4096 || c.hole_count > 64) return false;
    z = fsim::control::OpZone{};
    z.id = c.op_zone_id, z.shape = c.shape;
    auto vertex = [](const fsim_zone_vertex& v) { return fsim::control::ZoneVertex{v.latitude_rad, v.longitude_rad, v.x_m, v.y_m}; };
    for (uint32_t i = 0; i < c.vertex_count; ++i) z.vertices.push_back(vertex(c.vertices[i]));
    for (uint32_t k = 0; k < c.hole_count; ++k) {
        if ((c.hole_sizes[k] && !c.holes[k]) || c.hole_sizes[k] > 4096) return false;
        auto& hole = z.holes.emplace_back();
        for (uint32_t i = 0; i < c.hole_sizes[k]; ++i) hole.push_back(vertex(c.holes[k][i]));
    }
    z.latitudeRad = c.latitude_rad, z.longitudeRad = c.longitude_rad, z.xM = c.x_m, z.yM = c.y_m;
    z.semiMajorM = c.semi_major_m, z.semiMinorM = c.semi_minor_m, z.widthM = c.width_m, z.heightM = c.height_m;
    z.rangeMinM = c.range_min_m, z.rangeMaxM = c.range_max_m;
    z.azimuthMinRad = c.azimuth_min_rad, z.azimuthMaxRad = c.azimuth_max_rad, z.orientationRad = c.orientation_rad;
    z.altitudeMinM = c.altitude_min_m, z.altitudeMaxM = c.altitude_max_m, z.altitudeReference = c.altitude_reference;
    z.frame = c.frame, z.frameRotation = c.frame_rotation;
    z.northMs = c.north_ms, z.eastMs = c.east_ms, z.timeS = c.time_s;
    return true;
}

/// An operational line from the caller's (ABI 1.41): false where its vertices cannot be read.
bool lineFromC(const fsim_op_line& c, fsim::control::OpLine& l) {
    if (c.struct_size < offsetof(fsim_op_line, time_s) + sizeof c.time_s) return false;
    if ((c.vertex_count && !c.vertices) || c.vertex_count > 4096) return false;
    l = fsim::control::OpLine{};
    l.id = c.op_line_id;
    for (uint32_t i = 0; i < c.vertex_count; ++i) {
        const fsim_line_vertex& v = c.vertices[i];
        l.vertices.push_back(fsim::control::LineVertex{v.latitude_rad, v.longitude_rad, v.x_m, v.y_m, v.altitude_m, v.altitude_min_m, v.altitude_max_m,
                                                       v.altitude_reference, v.left_width_m, v.right_width_m});
    }
    l.projection = c.projection, l.leftWidthM = c.left_width_m, l.rightWidthM = c.right_width_m;
    l.altitudeMinM = c.altitude_min_m, l.altitudeMaxM = c.altitude_max_m, l.altitudeReference = c.altitude_reference;
    l.frame = c.frame, l.frameRotation = c.frame_rotation;
    l.northMs = c.north_ms, l.eastMs = c.east_ms, l.timeS = c.time_s;
    return true;
}

/// An operational volume from the caller's (ABI 1.42): false where its struct is short.
bool volumeFromC(const fsim_op_volume& c, fsim::control::OpVolume& v) {
    if (c.struct_size < offsetof(fsim_op_volume, time_s) + sizeof c.time_s) return false;
    v = fsim::control::OpVolume{};
    v.id = c.op_volume_id, v.shape = c.shape;
    v.latitudeRad = c.latitude_rad, v.longitudeRad = c.longitude_rad, v.xM = c.x_m, v.yM = c.y_m;
    v.altitudeM = c.altitude_m, v.altitudeReference = c.altitude_reference;
    v.radiusM = c.radius_m, v.semiAxisAM = c.semi_axis_a_m, v.semiAxisBM = c.semi_axis_b_m, v.semiAxisCM = c.semi_axis_c_m, v.lengthM = c.length_m;
    v.halfAngleRad = c.half_angle_rad, v.lengthHalfAngleRad = c.length_half_angle_rad, v.widthHalfAngleRad = c.width_half_angle_rad, v.rangeM = c.range_m;
    v.yawRad = c.yaw_rad, v.pitchRad = c.pitch_rad, v.rollRad = c.roll_rad;
    v.latitudeMinRad = c.latitude_min_rad, v.latitudeMaxRad = c.latitude_max_rad, v.longitudeMinRad = c.longitude_min_rad;
    v.longitudeMaxRad = c.longitude_max_rad, v.altitudeMinM = c.altitude_min_m, v.altitudeMaxM = c.altitude_max_m;
    v.frame = c.frame, v.frameRotation = c.frame_rotation;
    v.northMs = c.north_ms, v.eastMs = c.east_ms, v.downMs = c.down_ms, v.timeS = c.time_s;
    return true;
}

/// A batch item (or a task's command) into its C++ command: its waypoints and segments into `route` and `curve` (or
/// `nurbs`: ABI 1.24, where the item's struct has them), a pattern's shape into `shape` (the item pointing at it where it
/// has one).
bool fromBatch(fsim_world* world, const fsim_batch_command& b, fsim::control::BatchCommand& item, std::vector<fsim::control::Waypoint>& route,
               std::vector<fsim::control::BezierSegment>& curve, std::vector<fsim::control::NurbsSegment>& nurbs, fsim::control::PatternShape& shape,
               fsim::control::CurveShape& curveShape, std::vector<fsim::control::RouteLoiter>& loiters, std::vector<fsim::control::RouteState>& states,
               std::vector<fsim::control::RoutePath>& paths, std::vector<fsim::control::RouteBranch>& branches,
               std::vector<fsim::control::RouteTerminator>& terminators, fsim::control::OpZone& zone, fsim::control::OpLine& line,
               fsim::control::OpVolume& volume, fsim::control::MarshallCommand& marshall) {
    item.options = fromC(b.options);
    fsim::control::Command c;
    fsim::control::SupportCommand sc;
    bool ok = false;
    switch (b.kind) {
    case FSIM_BATCH_LEVEL: ok = toCommand(b.code, b.fields, b.count, c), item.command = c; break;
    case FSIM_BATCH_BEHAVIOR:
        ok = b.behavior && b.behavior->id;
        if (ok) item.command = fsim::control::Command(toBehavior(b.behavior));
        break;
    case FSIM_BATCH_SUPPORT: ok = toSupport(b.code, b.fields, b.count, sc), item.command = sc; break;
    case FSIM_BATCH_MODE:
        if (b.code == FSIM_MODE_MARSHALL) { // (ABI 1.43: in place of its command)
            ok = toMarshall(b.fields, b.count, marshall, &shape);
            item.marshall = &marshall;
            if (b.count > kPatternFields) item.shape = &shape;
            break;
        }
        ok = (b.code == FSIM_MODE_HSA || b.code == FSIM_MODE_PATTERN || b.code == FSIM_MODE_MUST_FLY) && toMode(b.code, b.fields, b.count, c, &shape),
        item.command = c;
        if (ok && b.code == FSIM_MODE_PATTERN && b.count > kPatternFields) item.shape = &shape;
        if (ok && b.code == FSIM_MODE_MUST_FLY && b.struct_size >= offsetof(fsim_batch_command, zone) + sizeof b.zone && b.zone) { // (ABI 1.40)
            ok = zoneFromC(*b.zone, zone);
            item.zone = &zone;
        }
        if (ok && b.code == FSIM_MODE_MUST_FLY && b.struct_size >= offsetof(fsim_batch_command, line) + sizeof b.line && b.line) { // (ABI 1.41)
            ok = lineFromC(*b.line, line);
            item.line = &line;
        }
        if (ok && b.code == FSIM_MODE_MUST_FLY && b.struct_size >= offsetof(fsim_batch_command, volume) + sizeof b.volume && b.volume) { // (ABI 1.42)
            ok = volumeFromC(*b.volume, volume);
            item.volume = &volume;
        }
        break;
    case FSIM_BATCH_ROUTE: { // (its loiters and states where the caller's struct has them: ABI 1.28, 1.31)
        const bool loitered = b.struct_size >= offsetof(fsim_batch_command, loiter_count) + sizeof b.loiter_count;
        const bool planned = b.struct_size >= offsetof(fsim_batch_command, states) + sizeof b.states;
        const bool pathed = b.struct_size >= offsetof(fsim_batch_command, path_count) + sizeof b.path_count;
        const bool branched = b.struct_size >= offsetof(fsim_batch_command, branch_count) + sizeof b.branch_count; // (ABI 1.34)
        const bool terminated = b.struct_size >= offsetof(fsim_batch_command, terminator_count) + sizeof b.terminator_count; // (ABI 1.35)
        ok = toMode(FSIM_MODE_ROUTE, b.fields, b.count, c) && toWaypoints(world, b.waypoints, b.waypoint_count) &&
             toLoiters(world, loitered ? b.loiters : nullptr, loitered ? b.loiter_count : 0) &&
             toStates(world, planned ? b.states : nullptr, planned ? b.state_count : 0) &&
             toPaths(world, pathed ? b.paths : nullptr, pathed ? b.path_count : 0) &&
             toBranches(world, branched ? b.branches : nullptr, branched ? b.branch_count : 0) &&
             toTerminators(world, terminated ? b.terminators : nullptr, terminated ? b.terminator_count : 0);
        if (ok) {
            route = world->waypoints, loiters = world->loiters, states = world->states, paths = world->paths, branches = world->branches;
            terminators = world->terminators, item.command = c;
        }
        break;
    }
    case FSIM_BATCH_CURVE:
        ok = toMode(FSIM_MODE_CURVE, b.fields, b.count, c, nullptr, &curveShape) && toSegments(world, b.segments, b.segment_count);
        if (ok) curve = world->segments, item.command = c;
        if (ok && b.count > kCurveFields) item.curveShape = &curveShape;
        break;
    case FSIM_BATCH_NURBS:
        ok = b.struct_size >= offsetof(fsim_batch_command, nurbs) + sizeof b.nurbs && toMode(FSIM_MODE_CURVE, b.fields, b.count, c, nullptr, &curveShape) &&
             toNurbs(world, b.nurbs, b.segment_count);
        if (ok) nurbs = world->nurbs, item.command = c;
        if (ok && b.count > kCurveFields) item.curveShape = &curveShape;
        break;
    default: break;
    }
    return ok;
}

} // namespace

FSIM_API int fsim_vehicle_submit_batch(fsim_world* world, uint32_t id, const fsim_batch_command* batch, uint32_t count, fsim_command_result* results,
                                       fsim_command_detail* details) {
    if (!world || (count && (!batch || !results || batch[0].struct_size < sizeof(uint32_t)))) return FSIM_INVALID_ARGUMENT;
    if (count && details && details[0].struct_size < sizeof(uint32_t)) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vehicle_submit_batch", [&]() -> int {
        const std::size_t stride = count ? batch[0].struct_size : 0;
        std::vector<fsim::control::BatchCommand> items(count);
        std::vector<std::vector<fsim::control::Waypoint>> routes; // each route's own (the world's scratch is one)
        std::vector<std::vector<fsim::control::BezierSegment>> curves;
        std::vector<std::vector<fsim::control::NurbsSegment>> nurbses;
        std::vector<std::vector<fsim::control::RouteLoiter>> loiterses;
        std::vector<std::vector<fsim::control::RouteState>> stateses;
        std::vector<std::vector<fsim::control::RoutePath>> pathses;
        std::vector<std::vector<fsim::control::RouteBranch>> brancheses;
        std::vector<std::vector<fsim::control::RouteTerminator>> terminatorses;
        std::vector<fsim::control::PatternShape> shapes(count); // (each pattern's own: the items point at them)
        std::vector<fsim::control::OpZone> zones(count);          // (each must fly's zone given, likewise)
        std::vector<fsim::control::OpLine> lines(count);          // (and corridor)
        std::vector<fsim::control::OpVolume> volumes(count);      // (and volume)
        std::vector<fsim::control::MarshallCommand> marshalls(count); // (each marshall's, likewise)
        std::vector<fsim::control::CurveShape> curveShapes(count); // (each curve's likewise)
        routes.reserve(count), curves.reserve(count), nurbses.reserve(count), loiterses.reserve(count), stateses.reserve(count), pathses.reserve(count);
        brancheses.reserve(count), terminatorses.reserve(count);
        // every item made into a command first: a malformed one refuses the batch, and none is made
        for (uint32_t i = 0; i < count; ++i) {
            const auto& b = *reinterpret_cast<const fsim_batch_command*>(reinterpret_cast<const char*>(batch) + i * stride);
            fsim::control::BatchCommand& item = items[i];
            std::vector<fsim::control::Waypoint>& route = routes.emplace_back();
            std::vector<fsim::control::BezierSegment>& curve = curves.emplace_back();
            std::vector<fsim::control::NurbsSegment>& nurbs = nurbses.emplace_back();
            std::vector<fsim::control::RouteLoiter>& loiters = loiterses.emplace_back();
            std::vector<fsim::control::RouteState>& states = stateses.emplace_back();
            std::vector<fsim::control::RoutePath>& paths = pathses.emplace_back();
            std::vector<fsim::control::RouteBranch>& branches = brancheses.emplace_back();
            std::vector<fsim::control::RouteTerminator>& terminators = terminatorses.emplace_back();
            if (!fromBatch(world, b, item, route, curve, nurbs, shapes[i], curveShapes[i], loiters, states, paths, branches, terminators, zones[i],
                           lines[i], volumes[i], marshalls[i]))
                return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_batch: item " + std::to_string(i) + " is malformed");
            item.waypoints = route, item.segments = curve, item.nurbs = nurbs, item.loiters = loiters, item.states = states, item.paths = paths;
            item.branches = branches, item.terminators = terminators;
        }
        std::vector<fsim::control::CommandDetails> checks;
        const std::vector<fsim::control::CommandResult> answers = world->world.submitBatch(id, items, &checks);
        const std::size_t detailStride = count && details ? details[0].struct_size : 0;
        for (uint32_t i = 0; i < count; ++i) {
            toC(world, id, answers[i], &results[i]);
            if (!details) continue;
            auto* out = reinterpret_cast<fsim_command_detail*>(reinterpret_cast<char*>(details) + i * detailStride);
            fsim_command_detail d = detailOf(answers[i]);
            d.finding_count = checks[i].findingCount, d.adjustment_count = checks[i].adjustmentCount, d.suggestion = checks[i].suggestion;
            copyOut(d, out);
        }
        return FSIM_OK;
    });
}

FSIM_API void fsim_task_status_init(fsim_task_status* s) {
    if (!s) return;
    std::memset(s, 0, sizeof *s);
    s->struct_size = sizeof *s;
    s->percent = s->start_time = s->end_time = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API const char* fsim_task_state_name(int state) {
    return state >= 0 && state < static_cast<int>(fsim::control::TaskState::Count) ? fsim::control::taskStateName(static_cast<fsim::control::TaskState>(state))
                                                                                    : "?";
}

namespace {

int taskOut(const fsim::control::TaskStatus& t, fsim_task_status* out) {
    fsim_task_status c;
    fsim_task_status_init(&c);
    c.state = static_cast<int32_t>(t.state), c.reason = static_cast<int32_t>(t.reason), c.suggested = t.suggested ? 1 : 0;
    c.task_id = t.id, c.activity = t.activity, c.run = t.run, c.runs = t.runs;
    c.percent = t.percent, c.start_time = t.startTime, c.end_time = t.endTime, c.command_id = t.commandId;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

} // namespace

FSIM_API int fsim_vehicle_store_task(fsim_world* world, uint32_t id, uint64_t task_id, const fsim_batch_command* command, uint32_t attempts,
                                     double interval_s, int32_t* reason) {
    if (!world || !command || !reason || command->struct_size < sizeof(uint32_t)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_store_task: bad arguments");
    return guard("fsim_vehicle_store_task", [&]() -> int {
        fsim::control::BatchCommand item;
        std::vector<fsim::control::Waypoint> route;
        std::vector<fsim::control::BezierSegment> curve;
        std::vector<fsim::control::NurbsSegment> nurbs;
        fsim::control::PatternShape shape;
        fsim::control::CurveShape curveShape;
        std::vector<fsim::control::RouteLoiter> loiters;
        std::vector<fsim::control::RouteState> states;
        std::vector<fsim::control::RoutePath> paths;
        std::vector<fsim::control::RouteBranch> branches;
        std::vector<fsim::control::RouteTerminator> terminators;
        fsim::control::OpZone zone;
        fsim::control::OpLine line;
        fsim::control::OpVolume volume;
        fsim::control::MarshallCommand marshall;
        if (!fromBatch(world, *command, item, route, curve, nurbs, shape, curveShape, loiters, states, paths, branches, terminators, zone, line,
                       volume, marshall) ||
            !std::holds_alternative<fsim::control::Command>(item.command))
            return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_store_task: a flight or guidance command, whole, is kept");
        fsim::control::TaskRepetition repetition;
        repetition.attempts = attempts ? attempts : 1;
        repetition.intervalS = interval_s;
        item.waypoints = route, item.segments = curve, item.nurbs = nurbs, item.loiters = loiters, item.states = states, item.paths = paths;
        item.branches = branches, item.terminators = terminators;
        *reason = static_cast<int32_t>(world->world.storeTask(id, task_id, item, repetition));
        return FSIM_OK;
    });
}

FSIM_API int fsim_vehicle_command_task(fsim_world* world, uint32_t id, uint64_t task_id, const fsim_command_options* options, fsim_command_result* result) {
    if (!world || !result) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_command_task: bad arguments");
    return guard("fsim_vehicle_command_task", [&]() -> int {
        toC(world, id, world->world.commandTask(id, task_id, fromC(options)), result);
        return FSIM_OK;
    });
}

FSIM_API int fsim_vehicle_cancel_task(fsim_world* world, uint32_t id, uint64_t task_id, int source, uint32_t controller, fsim_command_result* result) {
    if (!world || !result || source < 0 || source > 2) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_cancel_task: bad arguments");
    return guard("fsim_vehicle_cancel_task", [&]() -> int {
        toC(world, id, world->world.cancelTask(id, task_id, fsim::control::Caller{static_cast<fsim::control::Source>(source), controller}), result);
        return FSIM_OK;
    });
}

FSIM_API int fsim_vehicle_remove_task(fsim_world* world, uint32_t id, uint64_t task_id, int32_t* reason) {
    if (!world || !reason) return FSIM_INVALID_ARGUMENT;
    *reason = static_cast<int32_t>(world->world.removeTask(id, task_id));
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_task_status(fsim_world* world, uint32_t id, uint64_t task_id, fsim_task_status* out) {
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const auto s = world->world.taskStatus(id, task_id);
    if (!s) return absent(FSIM_INVALID_ARGUMENT, "fsim_vehicle_task_status: no task " + std::to_string(task_id));
    return taskOut(*s, out);
}

FSIM_API uint32_t fsim_vehicle_task_count(fsim_world* world, uint32_t id) {
    return world ? static_cast<uint32_t>(world->world.tasks(id).size()) : 0;
}

FSIM_API int fsim_vehicle_task_at(fsim_world* world, uint32_t id, uint32_t index, fsim_task_status* out) {
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const auto all = world->world.tasks(id);
    if (index >= all.size()) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_task_at: no task " + std::to_string(index));
    return taskOut(all[index], out);
}

namespace {

/// A support command's fields, as fsim_vehicle_submit_support takes them (the engines': all four throttles).
void supportFields(const fsim::control::SupportCommand& c, std::vector<double>& f) {
    using namespace fsim::control;
    if (const auto* g = std::get_if<GearCommand>(&c)) f = {g->down};
    else if (const auto* x = std::get_if<FlapsCommand>(&c)) f = {x->position};
    else if (const auto* b = std::get_if<WheelBrakesCommand>(&c)) f = {b->left, b->right};
    else if (const auto* s = std::get_if<SpeedbrakeCommand>(&c)) f = {s->position};
    else if (const auto* t = std::get_if<PitchTrimCommand>(&c)) f = {t->position};
    else if (const auto* e = std::get_if<EnginesCommand>(&c)) f.assign(e->throttle, e->throttle + 4);
}

} // namespace

namespace {

/// The setpoint read back (readback.setpoint) as the batch item that would command it: its arrays the readback's.
fsim_batch_command setpointOut(fsim_world* world) {
    using namespace fsim::control;
    auto& r = world->readback;
    fsim_batch_command b;
    std::memset(&b, 0, sizeof b);
    b.struct_size = sizeof b;
    r.fields.clear();
    if (r.setpoint.marshall) { // (its pattern and its stack together: ABI 1.43)
        b.kind = FSIM_BATCH_MODE, b.code = FSIM_MODE_MARSHALL;
        MarshallCommand m = *r.setpoint.marshall;
        double* slots[kMaxCommandFields];
        const std::size_t n = marshallFields(m, slots);
        for (std::size_t i = 0; i < n; ++i) r.fields.push_back(*slots[i]);
        double* shaped[PatternShape::kFields];
        r.setpoint.shape.fields(shaped);
        for (double* f : shaped) r.fields.push_back(*f);
    } else if (const auto* support = std::get_if<SupportCommand>(&r.setpoint.command)) {
        b.kind = FSIM_BATCH_SUPPORT, b.code = static_cast<int32_t>(support->index());
        supportFields(*support, r.fields);
    } else if (Command c = std::get<Command>(r.setpoint.command); const auto* behavior = std::get_if<BehaviorCommand>(&c)) {
        b.kind = FSIM_BATCH_BEHAVIOR;
        r.names.clear(), r.values.clear(), r.points.clear();
        for (const auto& [name, value] : behavior->params) r.names.push_back(world->intern(name)), r.values.push_back(value);
        for (const PositionCommand& p : behavior->points)
            r.points.push_back(fsim_position_command{p.latitudeRad, p.longitudeRad, p.altitudeMslM, p.airspeedMs, p.captureRadiusM});
        r.behavior = fsim_behavior_command{world->intern(behavior->id), behavior->target,           r.names.data(), r.values.data(),
                                           static_cast<uint32_t>(r.names.size()), r.points.data(), static_cast<uint32_t>(r.points.size())};
        b.behavior = &r.behavior;
    } else {
        double* slots[kMaxCommandFields];
        const std::size_t n = commandFields(c, slots);
        for (std::size_t i = 0; i < n; ++i) r.fields.push_back(*slots[i]);
        if (std::holds_alternative<PatternCommand>(c)) { // (and its shape's, ABI 1.21)
            double* shaped[PatternShape::kFields];
            r.setpoint.shape.fields(shaped);
            for (double* f : shaped) r.fields.push_back(*f);
        }
        if (std::holds_alternative<CurveCommand>(c)) { // (and its reference in a frame, ABI 1.25)
            double* shaped[CurveShape::kFields];
            r.setpoint.curveShape.fields(shaped);
            for (double* f : shaped) r.fields.push_back(*f);
        }
        if (std::holds_alternative<HsaCommand>(c)) b.kind = FSIM_BATCH_MODE, b.code = FSIM_MODE_HSA;
        else if (std::holds_alternative<PatternCommand>(c)) b.kind = FSIM_BATCH_MODE, b.code = FSIM_MODE_PATTERN;
        else if (std::holds_alternative<MustFlyCommand>(c)) b.kind = FSIM_BATCH_MODE, b.code = FSIM_MODE_MUST_FLY;
        else if (std::holds_alternative<RouteCommand>(c)) b.kind = FSIM_BATCH_ROUTE, b.code = FSIM_MODE_ROUTE;
        else if (std::holds_alternative<CurveCommand>(c)) b.kind = FSIM_BATCH_CURVE, b.code = FSIM_MODE_CURVE;
        else b.kind = FSIM_BATCH_LEVEL, b.code = static_cast<int32_t>(c.index()); // (up to a behaviour's, the index is the level)
    }
    r.waypoints.resize(r.setpoint.waypoints.size());
    for (std::size_t i = 0; i < r.waypoints.size(); ++i) {
        const Waypoint& p = r.setpoint.waypoints[i];
        fsim_waypoint& w = r.waypoints[i];
        fsim_waypoint_init(&w);
        w.latitude_rad = p.latitudeRad, w.longitude_rad = p.longitudeRad, w.altitude_m = p.altitudeM, w.altitude_reference = p.altitudeReference;
        w.speed = p.speed, w.speed_reference = p.speedReference, w.turn = p.turn, w.max_bank_rad = p.maxBankRad, w.climb_rate_ms = p.climbRateMs;
        w.id = p.id;
        w.altitude_min_m = p.altitudeMinM, w.altitude_max_m = p.altitudeMaxM, w.kind = p.kind, w.waypoint_type = p.waypointType;
        w.frame = p.frame, w.frame_rotation = p.frameRotation, w.frame_offsets = p.frameOffsets;
        w.frame_x_m = p.frameXM, w.frame_y_m = p.frameYM, w.frame_z_m = p.frameZM;
        w.course_rad = p.courseRad, w.turn_radius_m = p.turnRadiusM;
        w.speed_optimization = p.speedOptimization, w.climb_optimization = p.climbOptimization, w.acceleration_ms2 = p.accelerationMs2;
        w.arrival_begin_s = p.arrivalBeginS, w.arrival_end_s = p.arrivalEndS;
        w.rnp_m = p.rnpM, w.next = p.next, w.terminator = p.terminator;
    }
    r.segments.resize(r.setpoint.segments.size());
    for (std::size_t i = 0; i < r.segments.size(); ++i) {
        const BezierSegment& p = r.setpoint.segments[i];
        fsim_bezier_segment& s = r.segments[i];
        fsim_bezier_segment_init(&s);
        std::copy_n(p.north, 6, s.north), std::copy_n(p.east, 6, s.east), std::copy_n(p.down, 6, s.down);
    }
    const bool general = b.kind == FSIM_BATCH_CURVE && r.segments.empty() && !r.setpoint.nurbs.empty(); // (not all Bezier's form: ABI 1.24)
    r.nurbs.resize(general ? r.setpoint.nurbs.size() : 0);
    for (std::size_t i = 0; i < r.nurbs.size(); ++i) {
        const NurbsSegment& p = r.setpoint.nurbs[i];
        fsim_nurbs_segment& s = r.nurbs[i];
        fsim_nurbs_segment_init(&s);
        s.points = p.points, s.knots = p.knots;
        std::copy_n(p.north, 10, s.north), std::copy_n(p.east, 10, s.east), std::copy_n(p.down, 10, s.down), std::copy_n(p.weight, 10, s.weight);
        std::copy_n(p.knot, 14, s.knot);
        s.curvature = p.curvature, s.first_index = p.firstIndex, s.last_index = p.lastIndex;
    }
    if (general) b.kind = FSIM_BATCH_NURBS;
    r.loiters.resize(r.setpoint.loiters.size()); // (a route's: ABI 1.28)
    for (std::size_t i = 0; i < r.loiters.size(); ++i) {
        RouteLoiter l = r.setpoint.loiters[i];
        fsim_route_loiter& c = r.loiters[i];
        fsim_route_loiter_init(&c);
        c.point = l.point, c.end_time_s = l.endTimeS;
        double* f[RouteLoiter::kFields];
        l.fields(f);
        for (std::size_t k = 0; k < RouteLoiter::kFields; ++k) c.fields[k] = *f[k];
    }
    b.loiter_count = static_cast<uint32_t>(r.loiters.size()), b.loiters = r.loiters.empty() ? nullptr : r.loiters.data();
    r.states.resize(r.setpoint.states.size()); // (a route's: ABI 1.31)
    for (std::size_t i = 0; i < r.states.size(); ++i) {
        RouteState s = r.setpoint.states[i];
        fsim_route_state& c = r.states[i];
        fsim_route_state_init(&c);
        c.point = s.point;
        double* f[RouteState::kFields];
        s.fields(f);
        for (std::size_t k = 0; k < RouteState::kFields; ++k) c.fields[k] = *f[k];
    }
    b.state_count = static_cast<uint32_t>(r.states.size()), b.states = r.states.empty() ? nullptr : r.states.data();
    r.paths.resize(r.setpoint.paths.size()); // (a route's: ABI 1.33)
    for (std::size_t i = 0; i < r.paths.size(); ++i) {
        const RoutePath& p = r.setpoint.paths[i];
        fsim_route_path& c = r.paths[i];
        fsim_route_path_init(&c);
        c.id = p.id, c.type = p.type, c.first = p.first, c.count = p.count;
    }
    b.path_count = static_cast<uint32_t>(r.paths.size()), b.paths = r.paths.empty() ? nullptr : r.paths.data();
    r.branches.resize(r.setpoint.branches.size()); // (a route's: ABI 1.34)
    for (std::size_t i = 0; i < r.branches.size(); ++i) {
        RouteBranch x = r.setpoint.branches[i];
        fsim_route_branch& c = r.branches[i];
        fsim_route_branch_init(&c);
        c.point = x.point;
        double* f[RouteBranch::kFields];
        x.fields(f);
        for (std::size_t k = 0; k < RouteBranch::kFields; ++k) c.fields[k] = *f[k];
    }
    b.branch_count = static_cast<uint32_t>(r.branches.size()), b.branches = r.branches.empty() ? nullptr : r.branches.data();
    r.terminators.resize(r.setpoint.terminators.size()); // (a route's: ABI 1.35)
    for (std::size_t i = 0; i < r.terminators.size(); ++i) {
        RouteTerminator x = r.setpoint.terminators[i];
        fsim_route_terminator& c = r.terminators[i];
        fsim_route_terminator_init(&c);
        c.point = x.point;
        double* f[RouteTerminator::kFields];
        x.fields(f);
        for (std::size_t k = 0; k < RouteTerminator::kFields; ++k) c.fields[k] = *f[k];
    }
    b.terminator_count = static_cast<uint32_t>(r.terminators.size()), b.terminators = r.terminators.empty() ? nullptr : r.terminators.data();
    b.count = static_cast<uint32_t>(r.fields.size()), b.fields = r.fields.empty() ? nullptr : r.fields.data();
    b.waypoint_count = static_cast<uint32_t>(r.waypoints.size()), b.waypoints = r.waypoints.empty() ? nullptr : r.waypoints.data();
    b.segment_count = static_cast<uint32_t>(general ? r.nurbs.size() : r.segments.size()), b.segments = r.segments.empty() ? nullptr : r.segments.data();
    b.nurbs = r.nurbs.empty() ? nullptr : r.nurbs.data();
    return b;
}

} // namespace

FSIM_API int fsim_activity_get_setpoint(fsim_world* world, fsim_activity_id activity, fsim_batch_command* out) {
    if (!world || !out || out->struct_size < sizeof(uint32_t)) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_activity_get_setpoint", [&]() -> int {
        if (!world->world.activitySetpoint(activity, world->readback.setpoint))
            return absent(FSIM_INVALID_ARGUMENT, "fsim_activity_get_setpoint: no live activity " + std::to_string(activity));
        const fsim_batch_command b = setpointOut(world);
        return copyOut(b, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
    });
}

// --- Route plans (ABI 1.36; docs/flight-autonomy.md, 4.39) ---

FSIM_API void fsim_point_metadata_init(fsim_point_metadata* metadata) {
    if (!metadata) return;
    std::memset(metadata, 0, sizeof *metadata);
    metadata->struct_size = sizeof *metadata;
}

FSIM_API void fsim_path_metadata_init(fsim_path_metadata* metadata) {
    if (!metadata) return;
    std::memset(metadata, 0, sizeof *metadata);
    metadata->struct_size = sizeof *metadata;
    fsim_route_state_init(&metadata->initial);
    metadata->endurance_s = metadata->fuel_kg = metadata->gross_weight_kg = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API void fsim_route_plan_init(fsim_route_plan* plan) {
    if (!plan) return;
    std::memset(plan, 0, sizeof *plan);
    plan->struct_size = sizeof *plan;
    plan->route.struct_size = sizeof plan->route;
    plan->route.kind = FSIM_BATCH_ROUTE, plan->route.code = FSIM_MODE_ROUTE;
}

FSIM_API void fsim_plan_status_init(fsim_plan_status* status) {
    if (!status) return;
    std::memset(status, 0, sizeof *status);
    status->struct_size = sizeof *status;
    status->percent = status->start_time = status->end_time = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API void fsim_plan_command_result_init(fsim_plan_command_result* result) {
    if (!result) return;
    std::memset(result, 0, sizeof *result);
    result->struct_size = sizeof *result;
}

FSIM_API const char* fsim_plan_command_name(int command) {
    return command >= 0 && command < static_cast<int>(fsim::control::PlanCommand::Count)
               ? fsim::control::planCommandName(static_cast<fsim::control::PlanCommand>(command))
               : "?";
}

FSIM_API const char* fsim_plan_state_name(int state) {
    return state >= 0 && state < static_cast<int>(fsim::control::PlanState::Count) ? fsim::control::planStateName(static_cast<fsim::control::PlanState>(state))
                                                                                    : "?";
}

FSIM_API const char* fsim_plan_execution_name(int execution) {
    return execution >= 0 && execution < static_cast<int>(fsim::control::PlanExecution::Count)
               ? fsim::control::planExecutionName(static_cast<fsim::control::PlanExecution>(execution))
               : "?";
}

FSIM_API const char* fsim_point_source_name(int source) {
    return source >= 0 && source < static_cast<int>(fsim::control::PointSource::Count)
               ? fsim::control::pointSourceName(static_cast<fsim::control::PointSource>(source))
               : "?";
}

namespace {

std::string text(const char* s) { return s ? std::string(s) : std::string(); }

/// A plan's points' metadata as the caller's header laid it out (`[0].struct_size` apart); false if it cannot be read.
bool toPointMetadata(const fsim_point_metadata* m, uint32_t count, std::vector<fsim::control::PointMetadata>& out) {
    using fsim::control::PointSource;
    out.clear();
    if (count == 0) return true;
    if (!m || m[0].struct_size < sizeof(fsim_point_metadata)) return false; // (its first layout)
    const uint32_t stride = m[0].struct_size;
    const auto* bytes = reinterpret_cast<const unsigned char*>(m);
    out.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_point_metadata c;
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, sizeof c);
        fsim::control::PointMetadata& p = out[i];
        p.point = c.point;
        p.source = c.source >= 0 && c.source < static_cast<int32_t>(PointSource::Count) ? static_cast<PointSource>(c.source) : PointSource::Count;
        p.locked = c.locked != 0, p.modified = c.modified != 0;
        p.remarksName = text(c.remarks_name), p.remarks = text(c.remarks), p.fixKey = text(c.fix_key), p.fixSystem = text(c.fix_system);
    }
    return true;
}

/// A plan's paths' metadata likewise.
bool toPathMetadata(const fsim_path_metadata* m, uint32_t count, std::vector<fsim::control::PathMetadata>& out) {
    using fsim::control::RouteState;
    out.clear();
    if (count == 0) return true;
    constexpr std::size_t kFirst = offsetof(fsim_path_metadata, airfield); // (ABI 1.36's layout; 1.37 adds its airfield and runway)
    if (!m || m[0].struct_size < kFirst) return false;
    const uint32_t stride = m[0].struct_size;
    const auto* bytes = reinterpret_cast<const unsigned char*>(m);
    out.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        fsim_path_metadata c;
        fsim_path_metadata_init(&c);
        std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, std::min<std::size_t>(stride, sizeof c));
        fsim::control::PathMetadata& p = out[i];
        p.path = c.path;
        p.initial.point = c.initial.point;
        double* f[RouteState::kFields];
        p.initial.fields(f);
        for (std::size_t k = 0; k < RouteState::kFields; ++k) *f[k] = c.initial.fields[k];
        p.enduranceS = c.endurance_s, p.fuelKg = c.fuel_kg, p.grossWeightKg = c.gross_weight_kg, p.transitionPlan = c.transition_plan;
        if (stride >= sizeof c) p.airfield = c.airfield, p.runway = c.runway;
    }
    return true;
}

int planStatusOut(const fsim::control::PlanStatus& s, fsim_plan_status* out) {
    fsim_plan_status c;
    fsim_plan_status_init(&c);
    c.state = static_cast<int32_t>(s.state), c.plan_id = s.id, c.version = s.version, c.revision = s.revision;
    c.execution = static_cast<int32_t>(s.execution), c.reason = static_cast<int32_t>(s.reason), c.for_planning_use_only = s.forPlanningUseOnly ? 1 : 0;
    c.fa_owned = s.faOwned ? 1 : 0;
    c.activity = s.activity, c.percent = s.percent, c.start_time = s.startTime, c.end_time = s.endTime, c.command_id = s.commandId;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

/// Its answer into the caller's struct: a validation's or a NEW's kept as the world's last (fsim_last_command_detail).
int planResultOut(fsim_world* world, uint32_t id, const fsim::control::PlanCommandResult& r, fsim_plan_command_result* out) {
    using fsim::control::PlanCommand;
    fsim_plan_command_result c;
    fsim_plan_command_result_init(&c);
    c.completed = r.completed ? 1 : 0, c.plan_id = r.plan, c.command = static_cast<int32_t>(r.command);
    c.state = static_cast<int32_t>(r.state), c.reason = static_cast<int32_t>(r.reason);
    const bool checked = r.command == PlanCommand::PrepareForActivation || r.command == PlanCommand::Activate;
    toC(checked ? world : nullptr, id, r.check, &c.check);
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

} // namespace

namespace {

/// A plan as the caller laid it out: "" read, else what could not be read.
const char* planFromC(fsim_world* world, const fsim_route_plan& plan, fsim::control::RoutePlan& p) {
    using namespace fsim::control;
    BatchCommand item;
    std::vector<BezierSegment> curve;
    std::vector<NurbsSegment> nurbs;
    PatternShape shape;
    CurveShape curveShape;
    OpZone zone; // (a route's: none)
    OpLine line;
    OpVolume volume;
    MarshallCommand marshall;
    const Command* c = nullptr;
    if (plan.route.kind != FSIM_BATCH_ROUTE ||
        !fromBatch(world, plan.route, item, p.waypoints, curve, nurbs, shape, curveShape, p.loiters, p.states, p.paths, p.branches, p.terminators, zone,
                   line, volume, marshall) ||
        !(c = std::get_if<Command>(&item.command)) || !std::holds_alternative<RouteCommand>(*c))
        return "its route cannot be read";
    if (!toPointMetadata(plan.points, plan.point_count, p.pointMetadata) || !toPathMetadata(plan.paths, plan.path_count, p.pathMetadata))
        return "its metadata cannot be read";
    p.id = plan.plan_id, p.version = plan.version, p.forPlanningUseOnly = plan.for_planning_use_only != 0;
    p.route = std::get<RouteCommand>(*c), p.detailed = plan.detailed != 0;
    p.remarksName = text(plan.remarks_name), p.remarks = text(plan.remarks);
    return "";
}

} // namespace

FSIM_API int fsim_vehicle_publish_plan(fsim_world* world, uint32_t id, const fsim_route_plan* plan, int32_t* reason) {
    if (!world || !plan || !reason || plan->struct_size < sizeof(fsim_route_plan)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_publish_plan: bad arguments");
    return guard("fsim_vehicle_publish_plan", [&]() -> int {
        fsim::control::RoutePlan p;
        if (const char* why = planFromC(world, *plan, p); *why) return fail(FSIM_INVALID_ARGUMENT, std::string("fsim_vehicle_publish_plan: ") + why);
        *reason = static_cast<int32_t>(world->world.publishPlan(id, p));
        return FSIM_OK;
    });
}

FSIM_API int fsim_vehicle_load_plan(fsim_world* world, uint32_t id, const fsim_route_plan* plan, int32_t* reason) {
    if (!world || !plan || !reason || plan->struct_size < sizeof(fsim_route_plan)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_load_plan: bad arguments");
    return guard("fsim_vehicle_load_plan", [&]() -> int {
        fsim::control::RoutePlan p;
        if (const char* why = planFromC(world, *plan, p); *why) return fail(FSIM_INVALID_ARGUMENT, std::string("fsim_vehicle_load_plan: ") + why);
        *reason = static_cast<int32_t>(world->world.loadPlan(id, p));
        return FSIM_OK;
    });
}

FSIM_API void fsim_plan_validation_init(fsim_plan_validation* validation) {
    if (!validation) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::memset(validation, 0, sizeof *validation);
    validation->struct_size = sizeof *validation;
    validation->wind_north_ms = validation->wind_east_ms = validation->gust_ms = nan;
    validation->origin_latitude_rad = validation->origin_longitude_rad = validation->origin_altitude_m = nan;
}

FSIM_API void fsim_plan_validation_result_init(fsim_plan_validation_result* result) {
    if (!result) return;
    std::memset(result, 0, sizeof *result);
    result->struct_size = sizeof *result;
}

namespace {

/// A validation's inputs as the caller laid them out; NULL: as the vehicle is now. False if they cannot be read.
bool validationFromC(const fsim_plan_validation* c, fsim::control::PlanValidation& v) {
    if (!c) return true;
    if (c->struct_size < sizeof(fsim_plan_validation)) return false;
    v.parts = c->parts, v.windNorthMs = c->wind_north_ms, v.windEastMs = c->wind_east_ms, v.gustMs = c->gust_ms;
    v.originLatitudeRad = c->origin_latitude_rad, v.originLongitudeRad = c->origin_longitude_rad, v.originAltitudeM = c->origin_altitude_m;
    v.modifyToValidate = c->modify_to_validate != 0;
    return true;
}

/// Its answer into the caller's struct, the check kept as the world's last (fsim_last_command_detail).
int validationOut(fsim_world* world, uint32_t id, const fsim::control::PlanValidationResult& r, fsim_plan_validation_result* out) {
    fsim_plan_validation_result c;
    fsim_plan_validation_result_init(&c);
    c.valid = r.valid ? 1 : 0;
    toC(world, id, r.check, &c.check);
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

} // namespace

FSIM_API int fsim_vehicle_validate_plan(fsim_world* world, uint32_t id, const fsim_route_plan* plan, const fsim_plan_validation* validation,
                                        fsim_plan_validation_result* out) {
    if (!world || !plan || !out || plan->struct_size < sizeof(fsim_route_plan)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_validate_plan: bad arguments");
    return guard("fsim_vehicle_validate_plan", [&]() -> int {
        fsim::control::RoutePlan p;
        fsim::control::PlanValidation v;
        if (const char* why = planFromC(world, *plan, p); *why) return fail(FSIM_INVALID_ARGUMENT, std::string("fsim_vehicle_validate_plan: ") + why);
        if (!validationFromC(validation, v)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_validate_plan: its validation cannot be read");
        return validationOut(world, id, world->world.validatePlan(id, p, v), out);
    });
}

FSIM_API int fsim_vehicle_validate_stored_plan(fsim_world* world, uint32_t id, uint64_t plan_id, const fsim_plan_validation* validation,
                                               fsim_plan_validation_result* out) {
    if (!world || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_validate_stored_plan: bad arguments");
    return guard("fsim_vehicle_validate_stored_plan", [&]() -> int {
        fsim::control::PlanValidation v;
        if (!validationFromC(validation, v)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_validate_stored_plan: its validation cannot be read");
        return validationOut(world, id, world->world.validatePlan(id, fsim::control::PlanId{plan_id}, v), out);
    });
}

FSIM_API void fsim_runway_init(fsim_runway* runway) {
    if (!runway) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::memset(runway, 0, sizeof *runway);
    runway->struct_size = sizeof *runway;
    runway->direction_rad = runway->available_length_m = nan;
    for (fsim_runway_coordinates* c : {&runway->takeoff, &runway->landing})
        for (fsim_runway_point* q : {&c->start, &c->threshold, &c->limit})
            q->latitude_rad = q->longitude_rad = q->altitude_m = q->altitude_reference = nan;
}

FSIM_API void fsim_airfield_init(fsim_airfield* airfield) {
    if (!airfield) return;
    std::memset(airfield, 0, sizeof *airfield);
    airfield->struct_size = sizeof *airfield;
    airfield->qnh_pa = std::numeric_limits<double>::quiet_NaN();
}

namespace {

fsim::control::RunwayPoint runwayPointFromC(const fsim_runway_point& q) {
    fsim::control::RunwayPoint p;
    p.latitudeRad = q.latitude_rad, p.longitudeRad = q.longitude_rad, p.altitudeM = q.altitude_m, p.altitudeReference = q.altitude_reference;
    return p;
}

fsim_runway_point runwayPointToC(const fsim::control::RunwayPoint& p) {
    return fsim_runway_point{p.latitudeRad, p.longitudeRad, p.altitudeM, p.altitudeReference};
}

/// An airfield read back into the world's readback (its runways and ICAO code the caller's to read until the next).
int airfieldOut(fsim_world* world, fsim::control::Airfield&& a, fsim_airfield* out) {
    auto& r = world->airfieldReadback;
    r.airfield = std::move(a);
    r.runways.resize(r.airfield.runways.size());
    for (std::size_t i = 0; i < r.runways.size(); ++i) {
        const fsim::control::Runway& x = r.airfield.runways[i];
        fsim_runway& c = r.runways[i];
        fsim_runway_init(&c);
        c.runway_id = x.id, c.direction_rad = x.directionRad, c.available_length_m = x.availableLengthM;
        c.takeoff = fsim_runway_coordinates{runwayPointToC(x.takeoff.start), runwayPointToC(x.takeoff.threshold), runwayPointToC(x.takeoff.limit)};
        c.landing = fsim_runway_coordinates{runwayPointToC(x.landing.start), runwayPointToC(x.landing.threshold), runwayPointToC(x.landing.limit)};
    }
    fsim_airfield c;
    fsim_airfield_init(&c);
    c.airfield_id = r.airfield.id, c.icao = r.airfield.icao.c_str(), c.qnh_pa = r.airfield.qnhPa, c.revision = r.airfield.revision;
    c.runway_count = static_cast<uint32_t>(r.runways.size()), c.runways = r.runways.empty() ? nullptr : r.runways.data();
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

} // namespace

FSIM_API int fsim_vehicle_load_airfield(fsim_world* world, uint32_t id, const fsim_airfield* airfield, int32_t* reason) {
    if (!world || !airfield || !reason || airfield->struct_size < sizeof(fsim_airfield) ||
        (airfield->runway_count && (!airfield->runways || airfield->runways[0].struct_size < sizeof(fsim_runway))))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_load_airfield: bad arguments");
    return guard("fsim_vehicle_load_airfield", [&]() -> int {
        fsim::control::Airfield a;
        a.id = airfield->airfield_id, a.icao = text(airfield->icao), a.qnhPa = airfield->qnh_pa;
        const uint32_t stride = airfield->runway_count ? airfield->runways[0].struct_size : 0;
        const auto* bytes = reinterpret_cast<const unsigned char*>(airfield->runways);
        for (uint32_t i = 0; i < airfield->runway_count; ++i) {
            fsim_runway c;
            std::memcpy(&c, bytes + static_cast<std::size_t>(i) * stride, sizeof c);
            fsim::control::Runway& r = a.runways.emplace_back();
            r.id = c.runway_id, r.directionRad = c.direction_rad, r.availableLengthM = c.available_length_m;
            r.takeoff = {runwayPointFromC(c.takeoff.start), runwayPointFromC(c.takeoff.threshold), runwayPointFromC(c.takeoff.limit)};
            r.landing = {runwayPointFromC(c.landing.start), runwayPointFromC(c.landing.threshold), runwayPointFromC(c.landing.limit)};
        }
        *reason = static_cast<int32_t>(world->world.loadAirfield(id, a));
        return FSIM_OK;
    });
}

FSIM_API uint32_t fsim_vehicle_airfield_count(fsim_world* world, uint32_t id) {
    return world ? static_cast<uint32_t>(world->world.airfields(id).size()) : 0;
}

FSIM_API int fsim_vehicle_get_airfield_at(fsim_world* world, uint32_t id, uint32_t index, fsim_airfield* out) {
    if (!world || !out || out->struct_size < sizeof(uint32_t)) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vehicle_get_airfield_at", [&]() -> int {
        auto all = world->world.airfields(id);
        if (index >= all.size()) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_get_airfield_at: no airfield " + std::to_string(index));
        return airfieldOut(world, std::move(all[index]), out);
    });
}

FSIM_API int fsim_vehicle_get_airfield(fsim_world* world, uint32_t id, uint64_t airfield_id, fsim_airfield* out) {
    if (!world || !out || out->struct_size < sizeof(uint32_t)) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vehicle_get_airfield", [&]() -> int {
        auto a = world->world.airfield(id, airfield_id);
        if (!a) return absent(FSIM_INVALID_ARGUMENT, "fsim_vehicle_get_airfield: no airfield " + std::to_string(airfield_id));
        return airfieldOut(world, std::move(*a), out);
    });
}

FSIM_API int fsim_vehicle_plan_command(fsim_world* world, uint32_t id, uint64_t plan_id, int32_t command, const fsim_command_options* options,
                                       fsim_plan_command_result* result) {
    if (!world || !result || command < 0 || command >= static_cast<int32_t>(fsim::control::PlanCommand::Count))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_plan_command: bad arguments");
    return guard("fsim_vehicle_plan_command", [&]() -> int {
        return planResultOut(world, id, world->world.planCommand(id, plan_id, static_cast<fsim::control::PlanCommand>(command), fromC(options)), result);
    });
}

FSIM_API int fsim_vehicle_abort_plan(fsim_world* world, uint32_t id, uint64_t plan_id, int32_t reason, fsim_plan_command_result* result) {
    if (!world || !result || reason < 0 || reason >= static_cast<int32_t>(fsim::control::Reason::Count))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_abort_plan: bad arguments");
    return guard("fsim_vehicle_abort_plan", [&]() -> int {
        const auto why = reason ? static_cast<fsim::control::Reason>(reason) : fsim::control::Reason::Restricted;
        return planResultOut(world, id, world->world.abortPlan(id, plan_id, why), result);
    });
}

FSIM_API int fsim_vehicle_remove_plan(fsim_world* world, uint32_t id, uint64_t plan_id, int32_t* reason) {
    if (!world || !reason) return FSIM_INVALID_ARGUMENT;
    *reason = static_cast<int32_t>(world->world.removePlan(id, plan_id));
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_plan_status(fsim_world* world, uint32_t id, uint64_t plan_id, fsim_plan_status* out) {
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const auto s = world->world.planStatus(id, plan_id);
    if (!s) return absent(FSIM_INVALID_ARGUMENT, "fsim_vehicle_plan_status: no plan " + std::to_string(plan_id));
    return planStatusOut(*s, out);
}

FSIM_API uint32_t fsim_vehicle_plan_count(fsim_world* world, uint32_t id) {
    return world ? static_cast<uint32_t>(world->world.plans(id).size()) : 0;
}

FSIM_API int fsim_vehicle_plan_at(fsim_world* world, uint32_t id, uint32_t index, fsim_plan_status* out) {
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const auto all = world->world.plans(id);
    if (index >= all.size()) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_plan_at: no plan " + std::to_string(index));
    return planStatusOut(all[index], out);
}

FSIM_API int fsim_vehicle_get_plan(fsim_world* world, uint32_t id, uint64_t plan_id, fsim_route_plan* out) {
    if (!world || !out || out->struct_size < sizeof(uint32_t)) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vehicle_get_plan", [&]() -> int {
        using namespace fsim::control;
        auto kept = world->world.plan(id, plan_id);
        if (!kept) return absent(FSIM_INVALID_ARGUMENT, "fsim_vehicle_get_plan: no plan uploaded by id " + std::to_string(plan_id));
        auto& pr = world->planReadback;
        pr.plan = std::move(*kept);
        const RoutePlan& p = pr.plan;
        Setpoint& route = world->readback.setpoint; // (its route, as a setpoint's)
        route = Setpoint{};
        route.command = Command(p.route);
        route.waypoints = p.waypoints, route.loiters = p.loiters, route.states = p.states, route.paths = p.paths;
        route.branches = p.branches, route.terminators = p.terminators;
        fsim_route_plan c;
        fsim_route_plan_init(&c);
        c.plan_id = p.id, c.version = p.version, c.for_planning_use_only = p.forPlanningUseOnly ? 1 : 0, c.detailed = p.detailed ? 1 : 0;
        c.route = setpointOut(world);
        c.remarks_name = p.remarksName.c_str(), c.remarks = p.remarks.c_str();
        pr.points.resize(p.pointMetadata.size());
        for (std::size_t i = 0; i < pr.points.size(); ++i) {
            const PointMetadata& m = p.pointMetadata[i];
            fsim_point_metadata& x = pr.points[i];
            fsim_point_metadata_init(&x);
            x.point = m.point, x.source = static_cast<int32_t>(m.source), x.locked = m.locked ? 1 : 0, x.modified = m.modified ? 1 : 0;
            x.remarks_name = m.remarksName.c_str(), x.remarks = m.remarks.c_str(), x.fix_key = m.fixKey.c_str(), x.fix_system = m.fixSystem.c_str();
        }
        pr.paths.resize(p.pathMetadata.size());
        for (std::size_t i = 0; i < pr.paths.size(); ++i) {
            PathMetadata m = p.pathMetadata[i];
            fsim_path_metadata& x = pr.paths[i];
            fsim_path_metadata_init(&x);
            x.path = m.path, x.initial.point = m.initial.point;
            double* f[RouteState::kFields];
            m.initial.fields(f);
            for (std::size_t k = 0; k < RouteState::kFields; ++k) x.initial.fields[k] = *f[k];
            x.endurance_s = m.enduranceS, x.fuel_kg = m.fuelKg, x.gross_weight_kg = m.grossWeightKg, x.transition_plan = m.transitionPlan;
            x.airfield = m.airfield, x.runway = m.runway;
        }
        c.points = pr.points.empty() ? nullptr : pr.points.data(), c.point_count = static_cast<uint32_t>(pr.points.size());
        c.paths = pr.paths.empty() ? nullptr : pr.paths.data(), c.path_count = static_cast<uint32_t>(pr.paths.size());
        return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
    });
}

FSIM_API void fsim_end_point_init(fsim_end_point* point) {
    if (!point) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::memset(point, 0, sizeof *point);
    point->struct_size = sizeof *point;
    point->latitude_rad = point->longitude_rad = point->altitude_m = point->altitude_reference = point->turn = nan;
    point->index = -1;
}

FSIM_API const char* fsim_end_point_kind_name(int kind) {
    return kind >= 0 && kind < static_cast<int>(fsim::control::EndPointKind::Count)
               ? fsim::control::endPointKindName(static_cast<fsim::control::EndPointKind>(kind))
               : "?";
}

FSIM_API int fsim_activity_end_points(const fsim_world* world, fsim_activity_id activity, fsim_end_point* out, uint32_t max, uint32_t* count) {
    if (!world || !count || (max && (!out || out[0].struct_size < sizeof(uint32_t)))) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_activity_end_points", [&]() -> int {
        const std::vector<fsim::control::EndPoint> points = world->world.endPoints(activity, max);
        const uint32_t stride = max ? out[0].struct_size : 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const fsim::control::EndPoint& p = points[i];
            fsim_end_point c;
            fsim_end_point_init(&c);
            c.kind = static_cast<int32_t>(p.kind);
            c.latitude_rad = p.latitudeRad, c.longitude_rad = p.longitudeRad, c.altitude_m = p.altitudeM, c.altitude_reference = p.altitudeReference;
            c.turn = p.turn, c.id = p.id, c.index = p.index;
            auto* slot = reinterpret_cast<fsim_end_point*>(reinterpret_cast<char*>(out) + i * stride);
            slot->struct_size = stride; // (the caller's header's size: only the first is read)
            copyOut(c, slot);
        }
        *count = static_cast<uint32_t>(points.size());
        return FSIM_OK;
    });
}

// --- The navigation report (ABI 1.13; docs/flight-autonomy.md, 4.14) ---------------------------------------

FSIM_API void fsim_navigation_report_init(fsim_navigation_report* r) {
    if (!r) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::memset(r, 0, sizeof *r);
    r->struct_size = sizeof *r;
    r->fuel_kg = r->remaining = r->capacity = r->percent = r->consumption = r->endurance_s = r->reserve = nan;
    r->playtime_s = r->return_distance_m = r->return_tas_ms = r->return_consumption = nan;
}

FSIM_API int fsim_vehicle_navigation_report(const fsim_world* world, uint32_t id, fsim_navigation_report* out) {
    if (!world || !world->world.controls(id)) return FSIM_INVALID_ARGUMENT;
    const fsim::control::NavigationReport n = world->world.navigationReport(id);
    fsim_navigation_report r;
    fsim_navigation_report_init(&r);
    r.energy = static_cast<int32_t>(n.energy);
    r.fuel_kg = n.fuelKg, r.remaining = n.remaining, r.capacity = n.capacity, r.percent = n.percent, r.consumption = n.consumption;
    r.endurance_s = n.enduranceS, r.reserve = n.reserve, r.playtime_s = n.playtimeS;
    r.return_distance_m = n.returnDistanceM, r.return_tas_ms = n.returnTasMs, r.return_consumption = n.returnConsumption;
    r.contingency = static_cast<int32_t>(n.contingency);
    r.starved = n.starved ? 1 : 0;
    return copyOut(r, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API const char* fsim_energy_name(int energy) {
    return energy >= 0 && energy <= static_cast<int>(fsim::control::Energy::Battery) ? fsim::control::energyName(static_cast<fsim::control::Energy>(energy))
                                                                                      : "?";
}

FSIM_API const char* fsim_contingency_name(int contingency) {
    return contingency >= 0 && contingency <= static_cast<int>(fsim::control::Contingency::LostComms)
               ? fsim::control::contingencyName(static_cast<fsim::control::Contingency>(contingency))
               : "?";
}

FSIM_API void fsim_navigation_settings_init(fsim_navigation_settings* s) {
    if (!s) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::memset(s, 0, sizeof *s);
    s->struct_size = sizeof *s;
    s->latitude_deg = s->longitude_deg = s->altitude_msl_m = nan;
    s->reserve_fraction = fsim::control::NavigationSettings{}.reserveFraction;
}

FSIM_API int fsim_vehicle_set_navigation(fsim_world* world, uint32_t id, const fsim_navigation_settings* settings) {
    if (!world || !settings || !FSIM_HAS(settings, fsim_navigation_settings, reserve_fraction))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_navigation: bad arguments");
    fsim::control::NavigationSettings s;
    s.recovery = settings->recovery != 0;
    s.latitudeDeg = settings->latitude_deg, s.longitudeDeg = settings->longitude_deg, s.altitudeMslM = settings->altitude_msl_m;
    s.reserveFraction = settings->reserve_fraction;
    const auto reason = world->world.setNavigation(id, s);
    if (reason == fsim::control::Reason::UnknownVehicle) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_navigation: unknown vehicle");
    if (reason != fsim::control::Reason::None)
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_navigation: a point off the Earth or a reserve outside [0, 1)");
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_set_qnh(fsim_world* world, uint32_t id, double qnh_pa) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    const auto reason = world->world.setQnh(id, qnh_pa);
    if (reason == fsim::control::Reason::UnknownVehicle) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_qnh: unknown vehicle");
    if (reason != fsim::control::Reason::None) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_set_qnh: out_of_range - outside 850 to 1,100 hPa");
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_qnh(const fsim_world* world, uint32_t id, double* qnh_pa) {
    if (!world || !qnh_pa || !world->world.controls(id)) return FSIM_INVALID_ARGUMENT;
    *qnh_pa = world->world.qnh(id);
    return FSIM_OK;
}

FSIM_API void fsim_state_data_init(fsim_state_data* d) {
    if (!d) return;
    std::memset(d, 0, sizeof *d);
    d->struct_size = sizeof *d;
    d->indicated_altitude_m = d->indicated_altitude_rate_ms = d->kollsman_hpa = d->static_pressure_pa = d->static_temperature_k =
        std::numeric_limits<double>::quiet_NaN();
    d->yaw_rate_rad_s = d->pitch_rate_rad_s = d->roll_rate_rad_s = std::numeric_limits<double>::quiet_NaN();
    d->yaw_acceleration_rad_s2 = d->pitch_acceleration_rad_s2 = d->roll_acceleration_rad_s2 = std::numeric_limits<double>::quiet_NaN();
    d->wander_angle_rad = d->wind_north_ms = d->wind_east_ms = d->wind_down_ms = std::numeric_limits<double>::quiet_NaN();
    d->magnetic_heading_rad = d->declination_rad = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_vehicle_state_data(const fsim_world* world, uint32_t id, fsim_state_data* out) {
    if (!world || !world->world.controls(id)) return FSIM_INVALID_ARGUMENT;
    const fsim::control::StateData s = world->world.stateData(id);
    fsim_state_data d;
    fsim_state_data_init(&d);
    d.indicated_altitude_m = s.indicatedAltitudeM, d.indicated_altitude_rate_ms = s.indicatedAltitudeRateMs, d.kollsman_hpa = s.kollsmanHpa;
    d.static_pressure_pa = s.staticPressurePa, d.static_temperature_k = s.staticTemperatureK;
    d.yaw_rate_rad_s = s.yawRateRadS, d.pitch_rate_rad_s = s.pitchRateRadS, d.roll_rate_rad_s = s.rollRateRadS;
    d.yaw_acceleration_rad_s2 = s.yawAccelerationRadS2, d.pitch_acceleration_rad_s2 = s.pitchAccelerationRadS2, d.roll_acceleration_rad_s2 = s.rollAccelerationRadS2;
    d.wander_angle_rad = s.wanderAngleRad, d.wind_north_ms = s.windNorthMs, d.wind_east_ms = s.windEastMs, d.wind_down_ms = s.windDownMs;
    d.magnetic_heading_rad = s.magneticHeadingRad, d.declination_rad = s.declinationRad;
    return copyOut(d, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API void fsim_magnetic_field_init(fsim_magnetic_field* field) {
    if (!field) return;
    std::memset(field, 0, sizeof *field);
    field->struct_size = sizeof *field;
    field->north_nt = field->east_nt = field->down_nt = field->horizontal_nt = field->total_nt = std::numeric_limits<double>::quiet_NaN();
    field->declination_rad = field->inclination_rad = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_magnetic_field_at(double latitude_rad, double longitude_rad, double height_m, double decimal_year, fsim_magnetic_field* out) {
    if (!std::isfinite(latitude_rad) || std::abs(latitude_rad) > 0.5 * 3.14159265358979323846 || !std::isfinite(longitude_rad) ||
        !std::isfinite(height_m) || !std::isfinite(decimal_year))
        return FSIM_INVALID_ARGUMENT;
    const fsim::control::MagneticField m = fsim::control::magneticField(latitude_rad, longitude_rad, height_m, decimal_year);
    fsim_magnetic_field f;
    fsim_magnetic_field_init(&f);
    f.north_nt = m.northNt, f.east_nt = m.eastNt, f.down_nt = m.downNt, f.horizontal_nt = m.horizontalNt, f.total_nt = m.totalNt;
    f.declination_rad = m.declinationRad, f.inclination_rad = m.inclinationRad;
    return copyOut(f, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API double fsim_decimal_year(double unix_seconds) { return fsim::control::decimalYear(unix_seconds); }

FSIM_API double fsim_world_magnetic_year(const fsim_world* world) {
    if (!world) return std::numeric_limits<double>::quiet_NaN();
    return fsim::control::magneticYear(world->world.environment().epochUtcSeconds + world->world.simTime());
}

FSIM_API void fsim_frame_spec_init(fsim_frame_spec* spec) {
    if (!spec) return;
    std::memset(spec, 0, sizeof *spec);
    spec->struct_size = sizeof *spec;
}

FSIM_API int fsim_world_create_frame(fsim_world* world, const fsim_frame_spec* spec, uint64_t* id) {
    if (!world || !spec || !id || !FSIM_HAS(spec, fsim_frame_spec, time_s)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_create_frame: bad arguments");
    if (spec->origin < 0 || spec->origin >= static_cast<int32_t>(fsim::control::FrameOrigin::Count))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_world_create_frame: an origin not one");
    fsim::control::FrameSpec s;
    s.origin = static_cast<fsim::control::FrameOrigin>(spec->origin);
    s.latitudeRad = spec->latitude_rad, s.longitudeRad = spec->longitude_rad, s.altitudeMslM = spec->altitude_msl_m;
    s.yawRad = spec->yaw_rad, s.pitchRad = spec->pitch_rad, s.rollRad = spec->roll_rad;
    s.northMs = spec->north_ms, s.eastMs = spec->east_ms, s.downMs = spec->down_ms, s.timeS = spec->time_s;
    s.vehicle = spec->vehicle;
    const fsim::control::FrameId made = world->world.createFrame(s);
    if (!made) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_create_frame: a value not finite, a latitude off the Earth, or an unknown vehicle");
    *id = made;
    return FSIM_OK;
}

FSIM_API int fsim_world_remove_frame(fsim_world* world, uint64_t id) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    if (!world->world.removeFrame(id)) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_remove_frame: no such frame");
    return FSIM_OK;
}

FSIM_API void fsim_frame_offset_init(fsim_frame_offset* offset) {
    if (!offset) return;
    std::memset(offset, 0, sizeof *offset);
    offset->struct_size = sizeof *offset;
}

FSIM_API int fsim_world_frame_point(const fsim_world* world, uint64_t id, const fsim_frame_offset* offset, double time_s, double* latitude_rad,
                                    double* longitude_rad, double* altitude_msl_m) {
    if (!world || !offset || !latitude_rad || !longitude_rad || !altitude_msl_m || !FSIM_HAS(offset, fsim_frame_offset, z))
        return FSIM_INVALID_ARGUMENT;
    if (offset->rotation < 0 || offset->rotation >= static_cast<int32_t>(fsim::control::FrameRotation::Count) || offset->offsets < 0 ||
        offset->offsets >= static_cast<int32_t>(fsim::control::FrameOffsets::Count))
        return FSIM_INVALID_ARGUMENT;
    fsim::control::FrameOffset o;
    o.rotation = static_cast<fsim::control::FrameRotation>(offset->rotation);
    o.offsets = static_cast<fsim::control::FrameOffsets>(offset->offsets);
    o.x = offset->x, o.y = offset->y, o.z = offset->z;
    const std::optional<fsim::control::GeoPoint> p = world->world.framePoint(id, o, time_s);
    if (!p) return FSIM_INVALID_ARGUMENT;
    *latitude_rad = p->latitudeRad, *longitude_rad = p->longitudeRad, *altitude_msl_m = p->altitudeMslM;
    return FSIM_OK;
}

/// An operational point from the caller's (ABI 1.39), and back.
fsim::control::OpPoint opPointFromC(const fsim_op_point& c) noexcept {
    fsim::control::OpPoint p;
    p.id = c.op_point_id;
    p.latitudeRad = c.latitude_rad, p.longitudeRad = c.longitude_rad, p.altitudeM = c.altitude_m, p.altitudeReference = c.altitude_reference;
    p.frame = c.frame, p.frameRotation = c.frame_rotation, p.frameOffsets = c.frame_offsets;
    p.frameXM = c.frame_x_m, p.frameYM = c.frame_y_m, p.frameZM = c.frame_z_m;
    p.ingressMinRad = c.ingress_min_rad, p.ingressMaxRad = c.ingress_max_rad;
    return p;
}

void opPointToC(const fsim::control::OpPoint& p, fsim_op_point& c) noexcept {
    const uint32_t size = c.struct_size;
    fsim_op_point_init(&c);
    c.struct_size = size;
    c.revision = p.revision, c.op_point_id = p.id;
    c.latitude_rad = p.latitudeRad, c.longitude_rad = p.longitudeRad, c.altitude_m = p.altitudeM, c.altitude_reference = p.altitudeReference;
    c.frame = p.frame, c.frame_rotation = p.frameRotation, c.frame_offsets = p.frameOffsets;
    c.frame_x_m = p.frameXM, c.frame_y_m = p.frameYM, c.frame_z_m = p.frameZM;
    c.ingress_min_rad = p.ingressMinRad, c.ingress_max_rad = p.ingressMaxRad;
}

FSIM_API void fsim_op_point_init(fsim_op_point* point) {
    if (!point) return;
    std::memset(point, 0, sizeof *point);
    point->struct_size = sizeof *point;
    const double hold = fsim::control::kHold;
    point->latitude_rad = point->longitude_rad = point->altitude_m = point->altitude_reference = hold;
    point->frame = point->frame_rotation = point->frame_offsets = point->frame_x_m = point->frame_y_m = point->frame_z_m = hold;
    point->ingress_min_rad = point->ingress_max_rad = hold;
}

FSIM_API int fsim_world_set_op_point(fsim_world* world, const fsim_op_point* point, int32_t* reason) {
    if (!world || !point || !reason || !FSIM_HAS(point, fsim_op_point, ingress_max_rad)) return FSIM_INVALID_ARGUMENT;
    *reason = static_cast<int32_t>(world->world.setOpPoint(opPointFromC(*point)));
    return FSIM_OK;
}

FSIM_API int fsim_world_remove_op_point(fsim_world* world, uint64_t id) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    if (!world->world.removeOpPoint(id)) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_remove_op_point: no such point");
    return FSIM_OK;
}

FSIM_API uint32_t fsim_world_op_point_count(const fsim_world* world) {
    return world ? static_cast<uint32_t>(world->world.opPoints().size()) : 0u;
}

FSIM_API int fsim_world_get_op_point_at(const fsim_world* world, uint32_t index, fsim_op_point* out) {
    if (!world || !out || !FSIM_HAS(out, fsim_op_point, ingress_max_rad)) return FSIM_INVALID_ARGUMENT;
    const std::vector<fsim::control::OpPointId> ids = world->world.opPoints();
    if (index >= ids.size()) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_get_op_point_at: no such point");
    return fsim_world_get_op_point(world, ids[index], out);
}

FSIM_API int fsim_world_get_op_point(const fsim_world* world, uint64_t id, fsim_op_point* out) {
    if (!world || !out || !FSIM_HAS(out, fsim_op_point, ingress_max_rad)) return FSIM_INVALID_ARGUMENT;
    const std::optional<fsim::control::OpPoint> p = world->world.opPoint(id);
    if (!p) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_get_op_point: no such point");
    opPointToC(*p, *out);
    return FSIM_OK;
}

/// A zone kept, into the caller's struct: its arrays the world's readback's.
void zoneToC(fsim_world* w, const fsim::control::OpZone& z, fsim_op_zone& c) {
    auto& r = w->zoneReadback;
    auto vertex = [](const fsim::control::ZoneVertex& v) { return fsim_zone_vertex{v.latitudeRad, v.longitudeRad, v.xM, v.yM}; };
    r.vertices.clear(), r.holes.clear(), r.holePointers.clear(), r.holeSizes.clear();
    for (const auto& v : z.vertices) r.vertices.push_back(vertex(v));
    for (const auto& h : z.holes) {
        auto& hole = r.holes.emplace_back();
        for (const auto& v : h) hole.push_back(vertex(v));
    }
    for (const auto& h : r.holes) r.holePointers.push_back(h.data()), r.holeSizes.push_back(static_cast<uint32_t>(h.size()));
    const uint32_t size = c.struct_size;
    fsim_op_zone_init(&c);
    c.struct_size = size;
    c.revision = z.revision, c.op_zone_id = z.id, c.shape = z.shape;
    c.vertices = r.vertices.empty() ? nullptr : r.vertices.data(), c.vertex_count = static_cast<uint32_t>(r.vertices.size());
    c.hole_count = static_cast<uint32_t>(r.holes.size());
    c.holes = r.holePointers.empty() ? nullptr : r.holePointers.data(), c.hole_sizes = r.holeSizes.empty() ? nullptr : r.holeSizes.data();
    c.latitude_rad = z.latitudeRad, c.longitude_rad = z.longitudeRad, c.x_m = z.xM, c.y_m = z.yM;
    c.semi_major_m = z.semiMajorM, c.semi_minor_m = z.semiMinorM, c.width_m = z.widthM, c.height_m = z.heightM;
    c.range_min_m = z.rangeMinM, c.range_max_m = z.rangeMaxM;
    c.azimuth_min_rad = z.azimuthMinRad, c.azimuth_max_rad = z.azimuthMaxRad, c.orientation_rad = z.orientationRad;
    c.altitude_min_m = z.altitudeMinM, c.altitude_max_m = z.altitudeMaxM, c.altitude_reference = z.altitudeReference;
    c.frame = z.frame, c.frame_rotation = z.frameRotation;
    c.north_ms = z.northMs, c.east_ms = z.eastMs, c.time_s = z.timeS;
}

FSIM_API void fsim_op_zone_init(fsim_op_zone* zone) {
    if (!zone) return;
    std::memset(zone, 0, sizeof *zone);
    zone->struct_size = sizeof *zone;
    const double hold = fsim::control::kHold;
    zone->shape = hold;
    double* fields[] = {&zone->latitude_rad, &zone->longitude_rad, &zone->x_m, &zone->y_m, &zone->semi_major_m, &zone->semi_minor_m,
                        &zone->width_m, &zone->height_m, &zone->range_min_m, &zone->range_max_m, &zone->azimuth_min_rad, &zone->azimuth_max_rad,
                        &zone->orientation_rad, &zone->altitude_min_m, &zone->altitude_max_m, &zone->altitude_reference, &zone->frame,
                        &zone->frame_rotation, &zone->north_ms, &zone->east_ms, &zone->time_s};
    for (double* f : fields) *f = hold;
}

FSIM_API int fsim_world_set_op_zone(fsim_world* world, const fsim_op_zone* zone, int32_t* reason) {
    fsim::control::OpZone z;
    if (!world || !zone || !reason || !zoneFromC(*zone, z)) return FSIM_INVALID_ARGUMENT;
    *reason = static_cast<int32_t>(world->world.setOpZone(z));
    return FSIM_OK;
}

FSIM_API int fsim_world_remove_op_zone(fsim_world* world, uint64_t id) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    if (!world->world.removeOpZone(id)) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_remove_op_zone: no such zone");
    return FSIM_OK;
}

FSIM_API uint32_t fsim_world_op_zone_count(const fsim_world* world) { return world ? static_cast<uint32_t>(world->world.opZones().size()) : 0u; }

FSIM_API int fsim_world_get_op_zone(fsim_world* world, uint64_t id, fsim_op_zone* out) {
    if (!world || !out || !FSIM_HAS(out, fsim_op_zone, time_s)) return FSIM_INVALID_ARGUMENT;
    const std::optional<fsim::control::OpZone> z = world->world.opZone(id);
    if (!z) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_get_op_zone: no such zone");
    zoneToC(world, *z, *out);
    return FSIM_OK;
}

FSIM_API int fsim_world_get_op_zone_at(fsim_world* world, uint32_t index, fsim_op_zone* out) {
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const std::vector<fsim::control::OpZoneId> ids = world->world.opZones();
    if (index >= ids.size()) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_get_op_zone_at: no such zone");
    return fsim_world_get_op_zone(world, ids[index], out);
}

FSIM_API int fsim_vehicle_submit_must_fly(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_op_zone* zone,
                                          const fsim_command_options* options, fsim_command_result* result) {
    if (!zone) return fsim_vehicle_submit_mode(world, id, FSIM_MODE_MUST_FLY, fields, count, options, result);
    fsim::control::Command c;
    fsim::control::OpZone z;
    if (!world || !result || !toMode(FSIM_MODE_MUST_FLY, fields, count, c) || !zoneFromC(*zone, z))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_must_fly: a must fly takes 10 fields, and a zone's arrays whole");
    toC(world, id, world->world.submit(id, std::get<fsim::control::MustFlyCommand>(c), z, fromC(options)), result);
    return FSIM_OK;
}

FSIM_API int fsim_activity_update_must_fly(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                           const fsim_op_zone* zone, fsim_command_result* result) {
    return fsim_activity_update_must_fly_by(world, activity, FSIM_SOURCE_POLICY, 0, fields, count, zone, result);
}

FSIM_API int fsim_activity_update_must_fly_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                              uint32_t count, const fsim_op_zone* zone, fsim_command_result* result) {
    if (!zone) return fsim_activity_update_by(world, activity, source, controller, fields, count, result);
    fsim::control::Source from;
    fsim::control::Command c;
    fsim::control::OpZone z;
    if (!world || !result || !toSource(source, from) || !toMode(FSIM_MODE_MUST_FLY, fields, count, c) || !zoneFromC(*zone, z))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_must_fly: a must fly takes 10 fields, and a zone's arrays whole");
    toC(world, fsim::control::activityVehicle(activity),
        world->world.update(fsim::control::Caller{from, controller}, activity, std::get<fsim::control::MustFlyCommand>(c), z), result);
    return FSIM_OK;
}

/// A line kept, into the caller's struct: its vertices the world's readback's.
void lineToC(fsim_world* w, const fsim::control::OpLine& l, fsim_op_line& c) {
    auto& r = w->lineReadback;
    r.clear();
    for (const auto& v : l.vertices)
        r.push_back(fsim_line_vertex{v.latitudeRad, v.longitudeRad, v.xM, v.yM, v.altitudeM, v.altitudeMinM, v.altitudeMaxM, v.altitudeReference,
                                     v.leftWidthM, v.rightWidthM});
    const uint32_t size = c.struct_size;
    fsim_op_line_init(&c);
    c.struct_size = size;
    c.revision = l.revision, c.op_line_id = l.id;
    c.vertices = r.empty() ? nullptr : r.data(), c.vertex_count = static_cast<uint32_t>(r.size());
    c.projection = l.projection, c.left_width_m = l.leftWidthM, c.right_width_m = l.rightWidthM;
    c.altitude_min_m = l.altitudeMinM, c.altitude_max_m = l.altitudeMaxM, c.altitude_reference = l.altitudeReference;
    c.frame = l.frame, c.frame_rotation = l.frameRotation;
    c.north_ms = l.northMs, c.east_ms = l.eastMs, c.time_s = l.timeS;
}

FSIM_API void fsim_line_vertex_init(fsim_line_vertex* vertex) {
    if (!vertex) return;
    const double hold = fsim::control::kHold;
    *vertex = fsim_line_vertex{hold, hold, hold, hold, hold, hold, hold, hold, hold, hold};
}

FSIM_API void fsim_op_line_init(fsim_op_line* line) {
    if (!line) return;
    std::memset(line, 0, sizeof *line);
    line->struct_size = sizeof *line;
    const double hold = fsim::control::kHold;
    double* fields[] = {&line->projection, &line->left_width_m, &line->right_width_m, &line->altitude_min_m, &line->altitude_max_m,
                        &line->altitude_reference, &line->frame, &line->frame_rotation, &line->north_ms, &line->east_ms, &line->time_s};
    for (double* f : fields) *f = hold;
}

FSIM_API int fsim_world_set_op_line(fsim_world* world, const fsim_op_line* line, int32_t* reason) {
    fsim::control::OpLine l;
    if (!world || !line || !reason || !lineFromC(*line, l)) return FSIM_INVALID_ARGUMENT;
    *reason = static_cast<int32_t>(world->world.setOpLine(l));
    return FSIM_OK;
}

FSIM_API int fsim_world_remove_op_line(fsim_world* world, uint64_t id) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    if (!world->world.removeOpLine(id)) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_remove_op_line: no such line");
    return FSIM_OK;
}

FSIM_API uint32_t fsim_world_op_line_count(const fsim_world* world) { return world ? static_cast<uint32_t>(world->world.opLines().size()) : 0u; }

FSIM_API int fsim_world_get_op_line(fsim_world* world, uint64_t id, fsim_op_line* out) {
    if (!world || !out || !FSIM_HAS(out, fsim_op_line, time_s)) return FSIM_INVALID_ARGUMENT;
    const std::optional<fsim::control::OpLine> l = world->world.opLine(id);
    if (!l) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_get_op_line: no such line");
    lineToC(world, *l, *out);
    return FSIM_OK;
}

FSIM_API int fsim_world_get_op_line_at(fsim_world* world, uint32_t index, fsim_op_line* out) {
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const std::vector<fsim::control::OpLineId> ids = world->world.opLines();
    if (index >= ids.size()) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_get_op_line_at: no such line");
    return fsim_world_get_op_line(world, ids[index], out);
}

FSIM_API int fsim_vehicle_submit_must_fly_line(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_op_line* line,
                                               const fsim_command_options* options, fsim_command_result* result) {
    if (!line) return fsim_vehicle_submit_mode(world, id, FSIM_MODE_MUST_FLY, fields, count, options, result);
    fsim::control::Command c;
    fsim::control::OpLine l;
    if (!world || !result || !toMode(FSIM_MODE_MUST_FLY, fields, count, c) || !lineFromC(*line, l))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_must_fly_line: a must fly takes 10 fields, and a line's vertices whole");
    toC(world, id, world->world.submit(id, std::get<fsim::control::MustFlyCommand>(c), l, fromC(options)), result);
    return FSIM_OK;
}

FSIM_API int fsim_activity_update_must_fly_line(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                                const fsim_op_line* line, fsim_command_result* result) {
    return fsim_activity_update_must_fly_line_by(world, activity, FSIM_SOURCE_POLICY, 0, fields, count, line, result);
}

FSIM_API int fsim_activity_update_must_fly_line_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller,
                                                   const double* fields, uint32_t count, const fsim_op_line* line, fsim_command_result* result) {
    if (!line) return fsim_activity_update_by(world, activity, source, controller, fields, count, result);
    fsim::control::Source from;
    fsim::control::Command c;
    fsim::control::OpLine l;
    if (!world || !result || !toSource(source, from) || !toMode(FSIM_MODE_MUST_FLY, fields, count, c) || !lineFromC(*line, l))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_must_fly_line: a must fly takes 10 fields, and a line's vertices whole");
    toC(world, fsim::control::activityVehicle(activity),
        world->world.update(fsim::control::Caller{from, controller}, activity, std::get<fsim::control::MustFlyCommand>(c), l), result);
    return FSIM_OK;
}

/// A volume kept, into the caller's struct.
void volumeToC(const fsim::control::OpVolume& v, fsim_op_volume& c) {
    const uint32_t size = c.struct_size;
    fsim_op_volume_init(&c);
    c.struct_size = size;
    c.revision = v.revision, c.op_volume_id = v.id, c.shape = v.shape;
    c.latitude_rad = v.latitudeRad, c.longitude_rad = v.longitudeRad, c.x_m = v.xM, c.y_m = v.yM;
    c.altitude_m = v.altitudeM, c.altitude_reference = v.altitudeReference;
    c.radius_m = v.radiusM, c.semi_axis_a_m = v.semiAxisAM, c.semi_axis_b_m = v.semiAxisBM, c.semi_axis_c_m = v.semiAxisCM, c.length_m = v.lengthM;
    c.half_angle_rad = v.halfAngleRad, c.length_half_angle_rad = v.lengthHalfAngleRad, c.width_half_angle_rad = v.widthHalfAngleRad, c.range_m = v.rangeM;
    c.yaw_rad = v.yawRad, c.pitch_rad = v.pitchRad, c.roll_rad = v.rollRad;
    c.latitude_min_rad = v.latitudeMinRad, c.latitude_max_rad = v.latitudeMaxRad, c.longitude_min_rad = v.longitudeMinRad;
    c.longitude_max_rad = v.longitudeMaxRad, c.altitude_min_m = v.altitudeMinM, c.altitude_max_m = v.altitudeMaxM;
    c.frame = v.frame, c.frame_rotation = v.frameRotation;
    c.north_ms = v.northMs, c.east_ms = v.eastMs, c.down_ms = v.downMs, c.time_s = v.timeS;
}

FSIM_API void fsim_op_volume_init(fsim_op_volume* volume) {
    if (!volume) return;
    std::memset(volume, 0, sizeof *volume);
    volume->struct_size = sizeof *volume;
    const double hold = fsim::control::kHold;
    double* fields[] = {&volume->shape, &volume->latitude_rad, &volume->longitude_rad, &volume->x_m, &volume->y_m, &volume->altitude_m,
                        &volume->altitude_reference, &volume->radius_m, &volume->semi_axis_a_m, &volume->semi_axis_b_m, &volume->semi_axis_c_m,
                        &volume->length_m, &volume->half_angle_rad, &volume->length_half_angle_rad, &volume->width_half_angle_rad, &volume->range_m,
                        &volume->yaw_rad, &volume->pitch_rad, &volume->roll_rad, &volume->latitude_min_rad, &volume->latitude_max_rad,
                        &volume->longitude_min_rad, &volume->longitude_max_rad, &volume->altitude_min_m, &volume->altitude_max_m, &volume->frame,
                        &volume->frame_rotation, &volume->north_ms, &volume->east_ms, &volume->down_ms, &volume->time_s};
    for (double* f : fields) *f = hold;
}

FSIM_API int fsim_world_set_op_volume(fsim_world* world, const fsim_op_volume* volume, int32_t* reason) {
    fsim::control::OpVolume v;
    if (!world || !volume || !reason || !volumeFromC(*volume, v)) return FSIM_INVALID_ARGUMENT;
    *reason = static_cast<int32_t>(world->world.setOpVolume(v));
    return FSIM_OK;
}

FSIM_API int fsim_world_remove_op_volume(fsim_world* world, uint64_t id) {
    if (!world) return FSIM_INVALID_ARGUMENT;
    if (!world->world.removeOpVolume(id)) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_remove_op_volume: no such volume");
    return FSIM_OK;
}

FSIM_API uint32_t fsim_world_op_volume_count(const fsim_world* world) { return world ? static_cast<uint32_t>(world->world.opVolumes().size()) : 0u; }

FSIM_API int fsim_world_get_op_volume(const fsim_world* world, uint64_t id, fsim_op_volume* out) {
    if (!world || !out || !FSIM_HAS(out, fsim_op_volume, time_s)) return FSIM_INVALID_ARGUMENT;
    const std::optional<fsim::control::OpVolume> v = world->world.opVolume(id);
    if (!v) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_get_op_volume: no such volume");
    volumeToC(*v, *out);
    return FSIM_OK;
}

FSIM_API int fsim_world_get_op_volume_at(const fsim_world* world, uint32_t index, fsim_op_volume* out) {
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const std::vector<fsim::control::OpVolumeId> ids = world->world.opVolumes();
    if (index >= ids.size()) return absent(FSIM_INVALID_ARGUMENT, "fsim_world_get_op_volume_at: no such volume");
    return fsim_world_get_op_volume(world, ids[index], out);
}

FSIM_API int fsim_vehicle_submit_must_fly_volume(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_op_volume* volume,
                                                 const fsim_command_options* options, fsim_command_result* result) {
    if (!volume) return fsim_vehicle_submit_mode(world, id, FSIM_MODE_MUST_FLY, fields, count, options, result);
    fsim::control::Command c;
    fsim::control::OpVolume v;
    if (!world || !result || !toMode(FSIM_MODE_MUST_FLY, fields, count, c) || !volumeFromC(*volume, v))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_submit_must_fly_volume: a must fly takes 10 fields, and a volume whole");
    toC(world, id, world->world.submit(id, std::get<fsim::control::MustFlyCommand>(c), v, fromC(options)), result);
    return FSIM_OK;
}

FSIM_API int fsim_activity_update_must_fly_volume(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                                  const fsim_op_volume* volume, fsim_command_result* result) {
    return fsim_activity_update_must_fly_volume_by(world, activity, FSIM_SOURCE_POLICY, 0, fields, count, volume, result);
}

FSIM_API int fsim_activity_update_must_fly_volume_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller,
                                                     const double* fields, uint32_t count, const fsim_op_volume* volume, fsim_command_result* result) {
    if (!volume) return fsim_activity_update_by(world, activity, source, controller, fields, count, result);
    fsim::control::Source from;
    fsim::control::Command c;
    fsim::control::OpVolume v;
    if (!world || !result || !toSource(source, from) || !toMode(FSIM_MODE_MUST_FLY, fields, count, c) || !volumeFromC(*volume, v))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_activity_update_must_fly_volume: a must fly takes 10 fields, and a volume whole");
    toC(world, fsim::control::activityVehicle(activity),
        world->world.update(fsim::control::Caller{from, controller}, activity, std::get<fsim::control::MustFlyCommand>(c), v), result);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_get_navigation(const fsim_world* world, uint32_t id, fsim_navigation_settings* out) {
    if (!world || !world->world.controls(id)) return FSIM_INVALID_ARGUMENT;
    const fsim::control::NavigationSettings n = world->world.navigation(id);
    fsim_navigation_settings s;
    fsim_navigation_settings_init(&s);
    s.recovery = n.recovery ? 1 : 0;
    s.latitude_deg = n.latitudeDeg, s.longitude_deg = n.longitudeDeg, s.altitude_msl_m = n.altitudeMslM, s.reserve_fraction = n.reserveFraction;
    return copyOut(s, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

// --- The performance profile (ABI 1.14; docs/flight-autonomy.md, 4.15) ------------------------------------

// (its points are the C++ ones, field for field: the arrays point into the world's profile)
static_assert(sizeof(fsim_profile_point) == sizeof(fsim::control::ProfilePoint) && offsetof(fsim_profile_point, weight_kg) == offsetof(fsim::control::ProfilePoint, weightKg));
static_assert(sizeof(fsim_profile_acceleration) == sizeof(fsim::control::ProfileAcceleration) &&
              offsetof(fsim_profile_acceleration, mach) == offsetof(fsim::control::ProfileAcceleration, mach) &&
              offsetof(fsim_profile_acceleration, weight_kg) == offsetof(fsim::control::ProfileAcceleration, weightKg));
static_assert(sizeof(fsim_profile_excess_power) == sizeof(fsim::control::ProfileExcessPower) &&
              offsetof(fsim_profile_excess_power, weight_kg) == offsetof(fsim::control::ProfileExcessPower, weightKg));
static_assert(sizeof(fsim_profile_orientation) == sizeof(fsim::control::ProfileOrientation) &&
              offsetof(fsim_profile_orientation, roll_rad) == offsetof(fsim::control::ProfileOrientation, rollRad) &&
              offsetof(fsim_profile_orientation, weight_kg) == offsetof(fsim::control::ProfileOrientation, weightKg));
static_assert(sizeof(fsim_profile_rates) == sizeof(fsim::control::ProfileRates) && offsetof(fsim_profile_rates, tas_ms) == offsetof(fsim::control::ProfileRates, tasMs));

FSIM_API void fsim_performance_profile_init(fsim_performance_profile* p) {
    if (!p) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::memset(p, 0, sizeof *p);
    p->struct_size = sizeof *p;
    p->time_s = p->altitude_msl_m = p->weight_kg = p->tas_ms = nan;
    p->min_altitude_msl_m = p->max_altitude_msl_m = p->max_turn_rate_rad_s = p->max_climb_rate_ms = nan;
}

FSIM_API int fsim_vehicle_performance_profile(fsim_world* world, uint32_t id, int32_t mode, fsim_performance_profile* out, int32_t* reason) {
    if (reason) *reason = static_cast<int32_t>(fsim::control::Reason::None);
    if (!world || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_performance_profile: bad arguments");
    if (mode < 0 || mode >= static_cast<int32_t>(fsim::control::FlightMode::Count)) {
        if (reason) *reason = static_cast<int32_t>(fsim::control::Reason::InvalidParameter);
        return absent(FSIM_INVALID_ARGUMENT, "fsim_vehicle_performance_profile: not a flight mode");
    }
    fsim::control::PerformanceProfile& p = world->profile;
    const auto why = world->world.performanceProfile(id, static_cast<fsim::control::FlightMode>(mode), p);
    if (why != fsim::control::Reason::None) { // (a query's no: its reason, nothing logged)
        if (reason) *reason = static_cast<int32_t>(why);
        return absent(FSIM_INVALID_ARGUMENT, std::string("fsim_vehicle_performance_profile: ") + fsim::control::reasonName(why));
    }
    fsim_performance_profile r;
    fsim_performance_profile_init(&r);
    r.mode = static_cast<int32_t>(p.mode), r.energy = static_cast<int32_t>(p.energy);
    r.clean = p.clean ? 1 : 0, r.flaps_out = p.flapsOut ? 1 : 0, r.gear_down = p.gearDown ? 1 : 0;
    r.time_s = p.timeS, r.altitude_msl_m = p.altitudeMslM, r.weight_kg = p.weightKg, r.tas_ms = p.tasMs;
    r.min_altitude_msl_m = p.minAltitudeMslM, r.max_altitude_msl_m = p.maxAltitudeMslM;
    r.max_turn_rate_rad_s = p.maxTurnRateRadS, r.max_climb_rate_ms = p.maxClimbRateMs;
    auto points = [](const std::vector<fsim::control::ProfilePoint>& v, const fsim_profile_point*& to, uint32_t& n) {
        to = v.empty() ? nullptr : reinterpret_cast<const fsim_profile_point*>(v.data()), n = static_cast<uint32_t>(v.size());
    };
    auto accelerations = [](const std::vector<fsim::control::ProfileAcceleration>& v, const fsim_profile_acceleration*& to, uint32_t& n) {
        to = v.empty() ? nullptr : reinterpret_cast<const fsim_profile_acceleration*>(v.data()), n = static_cast<uint32_t>(v.size());
    };
    points(p.minAirspeed, r.min_airspeed, r.min_airspeed_count), points(p.maxAirspeed, r.max_airspeed, r.max_airspeed_count);
    points(p.bestEnduranceAirspeed, r.best_endurance_airspeed, r.best_endurance_airspeed_count);
    points(p.bestRangeAirspeed, r.best_range_airspeed, r.best_range_airspeed_count);
    accelerations(p.minAcceleration, r.min_acceleration, r.min_acceleration_count);
    accelerations(p.maxAcceleration, r.max_acceleration, r.max_acceleration_count);
    accelerations(p.maxDeceleration, r.max_deceleration, r.max_deceleration_count);
    r.excess_power = p.excessPower.empty() ? nullptr : reinterpret_cast<const fsim_profile_excess_power*>(p.excessPower.data());
    r.excess_power_count = static_cast<uint32_t>(p.excessPower.size());
    points(p.maxDescentRate, r.max_descent_rate, r.max_descent_rate_count), points(p.burn, r.burn, r.burn_count);
    r.max_orientation = p.maxOrientation.empty() ? nullptr : reinterpret_cast<const fsim_profile_orientation*>(p.maxOrientation.data());
    r.max_orientation_count = static_cast<uint32_t>(p.maxOrientation.size());
    r.max_orientation_rate = p.maxOrientationRate.empty() ? nullptr : reinterpret_cast<const fsim_profile_rates*>(p.maxOrientationRate.data());
    r.max_orientation_rate_count = static_cast<uint32_t>(p.maxOrientationRate.size());
    return copyOut(r, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API const char* fsim_requirement_kind_name(int kind) {
    return kind >= 0 && kind < static_cast<int>(fsim::control::RequirementKind::Count)
               ? fsim::control::requirementKindName(static_cast<fsim::control::RequirementKind>(kind))
               : "?";
}

FSIM_API const char* fsim_constraint_name(int constraint) {
    return constraint >= 0 && constraint < static_cast<int>(fsim::control::Constraint::Count)
               ? fsim::control::constraintName(static_cast<fsim::control::Constraint>(constraint))
               : "?";
}

FSIM_API void fsim_activity_progress_init(fsim_activity_progress* p) {
    if (!p) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::memset(p, 0, sizeof *p);
    p->struct_size = sizeof *p;
    p->percent = p->segment_percent = p->distance_to_go_m = p->time_to_go_s = p->cross_track_m = nan;
    p->course_rad = p->heading_rad = p->altitude_msl_m = p->speed_ms = p->speed_reference = nan;
    p->arrival_s = p->arrival_delta_s = nan;
}

FSIM_API int fsim_activity_get_progress(const fsim_world* world, fsim_activity_id activity, fsim_activity_progress* out) {
    const auto* a = world ? world->world.activity(activity) : nullptr;
    if (!a) return FSIM_INVALID_ARGUMENT;
    const auto& g = a->progress;
    fsim_activity_progress p;
    fsim_activity_progress_init(&p);
    p.segment = g.segment, p.segments = g.segments, p.laps = g.laps, p.segment_id = g.segmentId;
    p.percent = g.percent, p.segment_percent = g.segmentPercent;
    p.distance_to_go_m = g.distanceToGoM, p.time_to_go_s = g.timeToGoS, p.cross_track_m = g.crossTrackM;
    p.course_rad = g.courseRad, p.heading_rad = g.headingRad, p.altitude_msl_m = g.altitudeMslM;
    p.speed_ms = g.speedMs, p.speed_reference = g.speedReference;
    fsim::control::ArrivalEstimate arrival; // (1.30: asked of its behaviour, apart from the record's progress)
    if (world->world.activityArrival(activity, arrival)) p.arrival_s = arrival.arrivalS, p.arrival_delta_s = arrival.deltaS;
    return copyOut(p, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API void fsim_commanded_state_init(fsim_commanded_state* c) {
    if (!c) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::memset(c, 0, sizeof *c);
    c->struct_size = sizeof *c;
    c->latitude_rad = c->longitude_rad = c->altitude_msl_m = c->heading_rad = c->turn_rate_rad_s = nan;
    c->airspeed_ms = c->vertical_speed_ms = c->north_ms = c->east_ms = c->roll_rad = c->pitch_rad = nan;
    c->load_factor_g = c->roll_rate_rad_s = c->pitch_rate_rad_s = c->yaw_rate_rad_s = c->throttle = nan;
    c->north_acceleration_ms2 = c->east_acceleration_ms2 = c->down_acceleration_ms2 = c->altitude_m = c->altitude_reference = nan;
}

FSIM_API int fsim_vehicle_commanded(const fsim_world* world, uint32_t id, fsim_commanded_state* out) {
    if (!world || !world->world.controls(id)) return FSIM_INVALID_ARGUMENT;
    const fsim::control::VehicleCommandState s = world->world.commandState(id);
    fsim_commanded_state c;
    fsim_commanded_state_init(&c);
    c.top_level = static_cast<int32_t>(s.top);
    c.latitude_rad = s.latitudeRad, c.longitude_rad = s.longitudeRad, c.altitude_msl_m = s.altitudeMslM;
    c.heading_rad = s.headingRad, c.turn_rate_rad_s = s.turnRateRadS, c.airspeed_ms = s.airspeedMs;
    c.vertical_speed_ms = s.verticalSpeedMs, c.north_ms = s.northMs, c.east_ms = s.eastMs;
    c.roll_rad = s.rollRad, c.pitch_rad = s.pitchRad;
    c.load_factor_g = s.loadFactorG, c.roll_rate_rad_s = s.rollRateRadS, c.pitch_rate_rad_s = s.pitchRateRadS, c.yaw_rate_rad_s = s.yawRateRadS;
    c.throttle = s.throttle;
    c.north_acceleration_ms2 = s.northAccelerationMs2, c.east_acceleration_ms2 = s.eastAccelerationMs2, c.down_acceleration_ms2 = s.downAccelerationMs2;
    c.altitude_m = s.altitudeM, c.altitude_reference = s.altitudeReference;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_vehicle_capability_flight_mode(fsim_world* world, uint32_t id, uint32_t capability) {
    const auto* d = capabilityAt(world, id, capability);
    return d ? static_cast<int>(d->mode) : -1;
}

FSIM_API const char* fsim_flight_mode_name(int mode) {
    return mode >= 0 && mode < static_cast<int>(fsim::control::FlightMode::Count)
               ? fsim::control::flightModeName(static_cast<fsim::control::FlightMode>(mode))
               : "?";
}

// --- Support and availability (ABI 1.7) ------------------------------------------------------

FSIM_API void fsim_capability_status_init(fsim_capability_status* status) {
    if (!status) return;
    *status = fsim_capability_status{};
    status->struct_size = sizeof *status;
    status->next_available_s = std::numeric_limits<double>::quiet_NaN();
    status->description = "";
}

FSIM_API int fsim_vehicle_capability_status_info(const fsim_world* world, uint32_t id, const char* capability, fsim_capability_status* out) {
    if (!world || !capability || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_capability_status_info: bad arguments");
    const fsim::control::CapabilityStatus s = world->world.capabilityStatus(id, capability);
    fsim_capability_status c;
    fsim_capability_status_init(&c);
    c.availability = static_cast<int32_t>(s.availability);
    c.reason = static_cast<int32_t>(s.reason);
    c.range_count = s.rangeCount;
    c.reasons = s.reasons;
    c.associated = s.associated;
    c.next_available_s = s.nextAvailableS;
    c.description = s.description;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_vehicle_capability_limits(const fsim_world* world, uint32_t id, const char* capability, fsim_parameter_limit* out,
                                            uint32_t capacity, uint32_t* count) {
    if (!world || !capability || !count || (capacity && !out)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_capability_limits: bad arguments");
    const fsim::control::CapabilityStatus s = world->world.capabilityStatus(id, capability);
    *count = s.rangeCount;
    for (uint32_t i = 0; i < s.rangeCount && i < capacity; ++i)
        out[i] = fsim_parameter_limit{static_cast<uint32_t>(s.ranges[i].parameter), 0u, s.ranges[i].min, s.ranges[i].max};
    return FSIM_OK;
}

FSIM_API const char* fsim_availability_name(int availability) {
    return availability >= FSIM_AVAILABLE && availability <= FSIM_EXPENDED
               ? fsim::control::availabilityName(static_cast<fsim::control::Availability>(availability))
               : "?";
}

FSIM_API const char* fsim_reason_description(int reason) {
    return reason >= 0 && reason < static_cast<int>(fsim::control::Reason::Count) ? fsim::control::reasonDescription(static_cast<fsim::control::Reason>(reason))
                                                                                  : "";
}

FSIM_API int fsim_vehicle_capability_accepted(fsim_world* world, uint32_t id, uint32_t capability) {
    const auto* d = capabilityAt(world, id, capability);
    return d ? static_cast<int>(d->accepted) : -1;
}

FSIM_API const char* fsim_vehicle_capability_superseded(fsim_world* world, uint32_t id, uint32_t capability) {
    const auto* d = capabilityAt(world, id, capability);
    return d ? world->intern(d->superseded) : nullptr;
}

FSIM_API void fsim_support_info_init(fsim_support_info* info) {
    if (!info) return;
    *info = fsim_support_info{};
    info->struct_size = sizeof *info;
    info->feature = info->capability = info->missing = info->evidence = "";
}

FSIM_API int fsim_vehicle_support(const fsim_world* world, uint32_t id, const char* feature, fsim_support_info* out) {
    const fsim::control::SupportInfo* s = world && feature && out ? world->world.support(id, feature) : nullptr;
    if (!s) return fail(FSIM_INVALID_ARGUMENT, std::string("fsim_vehicle_support: no feature '") + (feature ? feature : "") + "' on vehicle " + std::to_string(id));
    fsim_support_info c;
    fsim_support_info_init(&c);
    c.support = static_cast<int32_t>(s->support);
    c.rules = s->rules;
    c.stage = s->stage;
    c.feature = s->feature;
    c.capability = s->capability;
    c.missing = s->missing;
    c.evidence = s->evidence;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API uint32_t fsim_support_feature_count(void) { return static_cast<uint32_t>(fsim::control::supportFeatureCount()); }

FSIM_API const char* fsim_support_feature(uint32_t index) { return fsim::control::supportFeature(index); }

FSIM_API const char* fsim_support_name(int support) {
    return support >= FSIM_SUPPORTED && support <= FSIM_NOT_SUPPORTED ? fsim::control::supportName(static_cast<fsim::control::Support>(support)) : "?";
}

FSIM_API const char* fsim_rule_name(int rule) {
    return rule > 0 && rule < static_cast<int>(fsim::control::Rule::Count) ? fsim::control::ruleName(static_cast<fsim::control::Rule>(rule)) : "";
}

FSIM_API const char* fsim_rule_description(int rule) {
    return rule > 0 && rule < static_cast<int>(fsim::control::Rule::Count) ? fsim::control::ruleDescription(static_cast<fsim::control::Rule>(rule)) : "";
}

FSIM_API uint32_t fsim_command_field_count(int level) {
    switch (level) {
    case FSIM_LEVEL_ACTUATOR: return 8;
    case FSIM_LEVEL_ATTITUDE: return 6;
    case FSIM_LEVEL_ACCELERATION: return 4;
    case FSIM_LEVEL_VELOCITY: return 4;
    case FSIM_LEVEL_POSITION: return 5;
    default: return 0;
    }
}

FSIM_API uint32_t fsim_command_field_count_full(int level) {
    if (level < FSIM_LEVEL_ACTUATOR || level >= FSIM_LEVEL_BEHAVIOR) return 0;
    fsim::control::Command c;
    switch (level) {
    case FSIM_LEVEL_ACTUATOR: c = fsim::control::ActuatorCommand{}; break;
    case FSIM_LEVEL_ATTITUDE: c = fsim::control::AttitudeCommand{}; break;
    case FSIM_LEVEL_ACCELERATION: c = fsim::control::AccelerationCommand{}; break;
    case FSIM_LEVEL_VELOCITY: c = fsim::control::VelocityCommand{}; break;
    default: c = fsim::control::PositionCommand{}; break;
    }
    double* slots[fsim::control::kMaxCommandFields];
    return static_cast<uint32_t>(fsim::control::commandFields(c, slots));
}

FSIM_API int fsim_world_gather_states(const fsim_world* world, const uint32_t* ids, uint32_t count, int sensed, fsim_vehicle_state* out) {
    if (!world || (count && (!ids || !out))) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_gather_states: bad arguments");
    uint32_t unknown = 0;
    for (uint32_t i = 0; i < count; ++i) {
        const fsim::sim::VehicleState* s = nullptr;
        if (sensed) {
            const auto* z = world->world.sensedState(ids[i]);
            s = z ? &z->state : nullptr;
        } else {
            s = world->world.vehicleState(ids[i]);
        }
        if (s) {
            std::memcpy(&out[i], s, sizeof(fsim_vehicle_state));
        } else {
            std::memset(&out[i], 0, sizeof(fsim_vehicle_state));
            if (!unknown) unknown = ids[i] ? ids[i] : ~0u;
        }
    }
    if (unknown) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_gather_states: no vehicle with id " + std::to_string(unknown));
    fsim::sdk::lastError().clear();
    return FSIM_OK;
}

FSIM_API int fsim_world_command_batch(fsim_world* world, int level, const uint32_t* ids, uint32_t count, const double* values, uint32_t stride) {
    const uint32_t fields = fsim_command_field_count(level);
    if (!world || !fields || (count && (!ids || !values)) || (stride && stride < fields))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_world_command_batch: bad arguments (level " + std::to_string(level) + ")");
    const std::size_t step = stride ? stride : fields;
    int result = FSIM_OK;
    for (uint32_t i = 0; i < count; ++i) {
        const double* row = values + i * step;
        int r = FSIM_INVALID_ARGUMENT;
        switch (level) {
        case FSIM_LEVEL_ACTUATOR: r = fsim_vehicle_command_actuator(world, ids[i], reinterpret_cast<const fsim_actuator_command*>(row)); break;
        case FSIM_LEVEL_ATTITUDE: r = fsim_vehicle_command_attitude(world, ids[i], reinterpret_cast<const fsim_attitude_command*>(row)); break;
        case FSIM_LEVEL_ACCELERATION: r = fsim_vehicle_command_acceleration(world, ids[i], reinterpret_cast<const fsim_acceleration_command*>(row)); break;
        case FSIM_LEVEL_VELOCITY: r = fsim_vehicle_command_velocity(world, ids[i], reinterpret_cast<const fsim_velocity_command*>(row)); break;
        case FSIM_LEVEL_POSITION: r = fsim_vehicle_command_position(world, ids[i], reinterpret_cast<const fsim_position_command*>(row)); break;
        default: break;
        }
        if (r != FSIM_OK && result == FSIM_OK) result = r;
    }
    if (result != FSIM_OK) return fail(result, "fsim_world_command_batch: " + fsim::sdk::lastError());
    fsim::sdk::lastError().clear();
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

FSIM_API int fsim_comm_attach_udp_bridge(fsim_world* world, uint32_t node, uint16_t local_port, const char* remote_host, uint16_t remote_port) {
    if (!world || !world->world.network().node(node)) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_comm_attach_udp_bridge", [&] {
        std::string error;
        auto transport = fsim::comm::createUdpTransport(local_port, remote_host ? remote_host : "", remote_port, &error);
        if (!transport) return fail(FSIM_ERROR, "fsim_comm_attach_udp_bridge: " + error);
        world->world.network().attach(node, std::make_unique<fsim::comm::BridgeProtocol>(std::move(transport)));
        return static_cast<int>(FSIM_OK);
    });
}

} // extern "C"

// --- Scenarios -------------------------------------------------------------------------

struct fsim_scenario {
    fsim::Scenario scenario;
};

FSIM_API int fsim_scenario_load(const char* path, fsim_scenario** out) {
    if (!path || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_scenario_load: bad arguments");
    *out = nullptr;
    return guard("fsim_scenario_load", [&] {
        *out = new fsim_scenario{fsim::loadScenario(path)};
        return FSIM_OK;
    });
}

FSIM_API int fsim_scenario_parse(const char* json, const char* source_name, fsim_scenario** out) {
    if (!json || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_scenario_parse: bad arguments");
    *out = nullptr;
    return guard("fsim_scenario_parse", [&] {
        *out = new fsim_scenario{fsim::parseScenario(json, source_name ? source_name : "scenario")};
        return FSIM_OK;
    });
}

FSIM_API void fsim_scenario_destroy(fsim_scenario* scenario) { delete scenario; }

FSIM_API int fsim_scenario_world_options(const fsim_scenario* scenario, fsim_world_options* out) {
    if (!scenario || !out || out->struct_size < sizeof(fsim_world_options)) return fail(FSIM_INVALID_ARGUMENT, "fsim_scenario_world_options: bad arguments");
    const auto& o = scenario->scenario.world;
    out->name = o.name.c_str();
    out->dt = o.dt;
    out->frame_skip = o.frameSkip;
    out->workers = o.workers;
    out->pin_workers = o.pinWorkers ? 1 : 0;
    out->seed = o.seed;
    out->capacity = o.capacity;
    out->publish = o.publish ? 1 : 0;
    out->publish_interval_s = o.publishIntervalSeconds;
    out->jsbsim_root = o.jsbsimRoot.empty() ? nullptr : o.jsbsimRoot.c_str();
    out->terrain = o.terrain ? 1 : 0;
    out->terrain_url = o.terrainUrl.empty() ? nullptr : o.terrainUrl.c_str();
    out->terrain_zoom = o.terrainZoom;
    out->record_path = o.recordPath.empty() ? nullptr : o.recordPath.c_str();
    out->record_interval_s = o.recordIntervalSeconds;
    return FSIM_OK;
}

FSIM_API uint32_t fsim_scenario_vehicle_count(const fsim_scenario* scenario) {
    if (!scenario) return 0;
    uint32_t n = 0;
    for (const auto& v : scenario->scenario.vehicles) n += v.count;
    return n;
}

FSIM_API int fsim_scenario_apply(fsim_world* world, const fsim_scenario* scenario, uint32_t* ids, size_t capacity, size_t* count) {
    if (!world || !scenario) return fail(FSIM_INVALID_ARGUMENT, "fsim_scenario_apply: bad arguments");
    if (count) *count = 0;
    return guard("fsim_scenario_apply", [&] {
        const auto created = fsim::session::applyScenario(world->world, scenario->scenario);
        if (ids)
            for (size_t i = 0; i < created.size() && i < capacity; ++i) ids[i] = created[i];
        if (count) *count = created.size();
        return FSIM_OK;
    });
}

// --- Recordings ---------------------------------------------------------------------------

struct fsim_recording {
    fsim::Recording recording;
    struct Frame {
        std::vector<fsim_recorded_sample> samples;
        std::vector<fsim_recorded_event> events;
    };
    std::vector<Frame> frames; ///< C views of the frames; strings point into `recording`
};

FSIM_API int fsim_recording_load(const char* path, fsim_recording** out) {
    if (!path || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_recording_load: bad arguments");
    *out = nullptr;
    return guard("fsim_recording_load", [&] {
        auto r = new fsim_recording{fsim::Recording::load(path), {}};
        r->frames.reserve(r->recording.frames().size());
        for (const auto& f : r->recording.frames()) {
            fsim_recording::Frame cf;
            for (const auto& s : f.samples) {
                fsim_recorded_sample cs;
                cs.slot = s.slot;
                std::memcpy(&cs.state, &s.state, sizeof cs.state);
                std::memcpy(&cs.inputs, &s.inputs, sizeof cs.inputs);
                cf.samples.push_back(cs);
            }
            for (const auto& e : f.events) {
                fsim_recorded_event ce;
                ce.slot = e.slot;
                ce.id = e.id;
                ce.generation = e.generation;
                ce.alive = e.alive ? 1 : 0;
                ce.control_level = e.controlLevel;
                ce.name = e.name.c_str();
                ce.type = e.type.c_str();
                ce.model = e.model.c_str();
                ce.initial_latitude_deg = e.initialLatitudeDeg;
                ce.initial_longitude_deg = e.initialLongitudeDeg;
                ce.initial_altitude_msl_m = e.initialAltitudeMslM;
                ce.initial_heading_deg = e.initialHeadingDeg;
                cf.events.push_back(ce);
            }
            r->frames.push_back(std::move(cf));
        }
        *out = r;
        return static_cast<int>(FSIM_OK);
    });
}

FSIM_API void fsim_recording_destroy(fsim_recording* recording) { delete recording; }
FSIM_API uint32_t fsim_recording_frame_count(const fsim_recording* r) { return r ? static_cast<uint32_t>(r->frames.size()) : 0u; }
FSIM_API double fsim_recording_frame_time(const fsim_recording* r, uint32_t frame) {
    return r && frame < r->frames.size() ? r->recording.frames()[frame].simTime : 0.0;
}
FSIM_API const fsim_recorded_sample* fsim_recording_samples(const fsim_recording* r, uint32_t frame, uint32_t* count) {
    if (!r || frame >= r->frames.size()) {
        if (count) *count = 0;
        return nullptr;
    }
    if (count) *count = static_cast<uint32_t>(r->frames[frame].samples.size());
    return r->frames[frame].samples.data();
}
FSIM_API const fsim_recorded_event* fsim_recording_events(const fsim_recording* r, uint32_t frame, uint32_t* count) {
    if (!r || frame >= r->frames.size()) {
        if (count) *count = 0;
        return nullptr;
    }
    if (count) *count = static_cast<uint32_t>(r->frames[frame].events.size());
    return r->frames[frame].events.data();
}
FSIM_API const char* fsim_recording_world_name(const fsim_recording* r) { return r ? r->recording.worldName().c_str() : ""; }
FSIM_API double fsim_recording_dt(const fsim_recording* r) { return r ? r->recording.dt() : 0.0; }
FSIM_API int32_t fsim_recording_frame_skip(const fsim_recording* r) { return r ? r->recording.frameSkip() : 0; }
