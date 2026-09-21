/* fsim C ABI (design 4.4, 9.2): the binding surface for any compiled language.
 *
 * Rules: opaque handles, plain structs with a leading struct_size, buffers
 * owned by the library and valid until the next reset/step/destroy, integer
 * return codes (0 = ok, negative = error; fsim_last_error() has the text),
 * no exceptions, no C++ types. Layouts only grow within a major version. */
#ifndef FSIM_C_H
#define FSIM_C_H

#include <stddef.h>
#include <stdint.h>

#include "fsim/Export.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FSIM_ABI_VERSION 1u

enum fsim_status {
    FSIM_OK = 0,
    FSIM_ERROR = -1,           /* see fsim_last_error() */
    FSIM_INVALID_ARGUMENT = -2,
    FSIM_LOAD_FAILED = -3
};

typedef struct fsim_vecenv fsim_vecenv;

/* Scenario + environment options. Call fsim_options_init() first, then override. */
typedef struct fsim_options {
    uint32_t struct_size;       /* sizeof(fsim_options), set by fsim_options_init */
    uint32_t num_envs;          /* M */
    uint32_t vehicles_per_env;  /* K */
    uint32_t workers;           /* 0 = automatic */
    uint64_t seed;
    const char* aircraft;       /* JSBSim aircraft name, e.g. "c172x" */
    const char* jsbsim_root;    /* NULL = auto-detect */
    const char* task;           /* "altitude_heading_hold" | "level_flight" */
    const char* observation;    /* "state" */
    const char* action;         /* "surfaces" */
    double dt;                  /* FDM step, s */
    int32_t frame_skip;         /* FDM steps per agent step */
    uint32_t max_episode_steps;
    /* initial conditions: centre + uniform jitter */
    double latitude_deg, longitude_deg, altitude_m, heading_deg, airspeed_ms;
    double latitude_jitter_deg, longitude_jitter_deg, altitude_jitter_m, heading_jitter_deg, airspeed_jitter_ms;
    /* task parameters */
    double target_altitude_delta_m, target_heading_delta_deg;
    /* appended in ABI 1.1 */
    const char* world_name;     /* published world name for viewers ("vecenv") */
    int32_t publish;            /* 0 = invisible to viewers */
    int32_t terrain;            /* 1 = physics ground from public elevation tiles */
    const char* scenario_path;  /* NULL, or a scenario file whose environment and world-wide effects apply to the batch */
} fsim_options;

/* Library-owned buffers, vehicle-major: index = env * K + vehicle. */
typedef struct fsim_buffers {
    uint32_t struct_size;
    uint32_t num_envs, vehicles_per_env, observation_size, action_size;
    const float* observations;        /* [M*K][observation_size] */
    const float* rewards;             /* [M*K] */
    const uint8_t* terminated;        /* [M*K] */
    const uint8_t* truncated;         /* [M*K] */
    const float* final_observations;  /* [M*K][observation_size], for environments that auto-reset */
    const uint32_t* episode_steps;    /* [M] */
    double agent_step_seconds;
} fsim_buffers;

FSIM_API uint32_t fsim_abi_version(void);
FSIM_API const char* fsim_version(void);      /* library version string */
FSIM_API const char* fsim_last_error(void);   /* thread-local, empty if none */

FSIM_API void fsim_options_init(fsim_options* options);

FSIM_API int fsim_vecenv_create(const fsim_options* options, fsim_vecenv** out);
FSIM_API void fsim_vecenv_destroy(fsim_vecenv* env);

/* seed 0 keeps the current seed. */
FSIM_API int fsim_vecenv_reset(fsim_vecenv* env, uint64_t seed);
/* actions: M*K*action_size floats. */
FSIM_API int fsim_vecenv_step(fsim_vecenv* env, const float* actions, size_t count);
FSIM_API int fsim_vecenv_buffers(const fsim_vecenv* env, fsim_buffers* out);

FSIM_API const char* fsim_vecenv_observation_name(const fsim_vecenv* env, uint32_t index);
FSIM_API const char* fsim_vecenv_action_name(const fsim_vecenv* env, uint32_t index);
FSIM_API uint64_t fsim_vecenv_vehicle_steps(const fsim_vecenv* env);

/* ---------------------------------------------------------------------------
 * World / vehicle API (design 9.2, 9.8): the object model for any language
 * with a C FFI. Ids are world-unique and never reused within a world.
 * ------------------------------------------------------------------------- */

typedef struct fsim_world fsim_world;

/* "keep the current value / let the controller decide" for optional command fields. */
FSIM_API double fsim_hold(void);

typedef struct fsim_world_options {
    uint32_t struct_size;
    const char* name;            /* viewers attach by this name ("default") */
    double dt;                   /* FDM step, s */
    int32_t frame_skip;          /* FDM steps per world step */
    uint32_t workers;            /* 0 = automatic */
    int32_t pin_workers;         /* 1 = one worker per physical core */
    uint64_t seed;
    uint32_t capacity;           /* vehicle slots visible to viewers */
    int32_t publish;             /* 0 = invisible to viewers */
    double publish_interval_s;
    const char* jsbsim_root;     /* NULL = auto-detect */
    int32_t terrain;             /* 1 = physics ground from public elevation tiles (network or warm cache) */
    const char* terrain_url;     /* NULL = AWS Terrarium */
    uint32_t terrain_zoom;       /* 12 */
    const char* record_path;     /* NULL = no recording; else a file for flightsim-viewer --replay */
    double record_interval_s;    /* simulation time between frames; 0 = every world step */
} fsim_world_options;

typedef struct fsim_vehicle_spec {
    uint32_t struct_size;
    const char* name;            /* NULL = generated */
    const char* type;            /* "jsbsim:c172x" */
    double latitude_deg, longitude_deg, altitude_msl_m, heading_deg, pitch_deg, roll_deg, airspeed_ms;
    int32_t on_ground;
    const char* model;           /* optional glTF for the viewer */
    uint32_t control_divider;    /* control stack every N FDM steps (1) */
} fsim_vehicle_spec;

/* Same layout as fsim::VehicleState (checked at build time). */
#define FSIM_MAX_ENGINES 4
typedef struct fsim_vehicle_state {
    double sim_time;
    double position_ecef[3];
    double attitude_ecef_to_body[4];      /* quaternion w,x,y,z */
    double latitude_rad, longitude_rad, altitude_msl_m, altitude_agl_m;
    double euler_rad[3];                  /* roll, pitch, yaw */
    double velocity_body_ms[3], velocity_ned_ms[3], angular_rate_body_rad_s[3], acceleration_body_ms2[3];
    double airspeed_true_ms, airspeed_calibrated_ms, mach, alpha_rad, beta_rad, load_factor;
    double aileron_rad, elevator_rad, rudder_rad, flaps_rad, gear_position;
    int32_t engine_count;
    double throttle_position[FSIM_MAX_ENGINES], thrust_n[FSIM_MAX_ENGINES], fuel_kg;
    uint32_t step_count;
    uint8_t on_ground, diverged;
    double rotation_body_to_ecef[9];
} fsim_vehicle_state;

typedef struct fsim_environment {
    uint32_t struct_size;
    double epoch_utc_seconds, time_factor;
    double temperature_sl_k, pressure_sl_pa, humidity;
    double wind_direction_deg, wind_speed_ms, wind_gust_ms, turbulence;
    double visibility_m, cloud_base_m, cloud_cover, precipitation;
} fsim_environment;

/* Control levels (fsim::control::Level). */
enum fsim_level { FSIM_LEVEL_ACTUATOR = 0, FSIM_LEVEL_ATTITUDE, FSIM_LEVEL_ACCELERATION, FSIM_LEVEL_VELOCITY, FSIM_LEVEL_POSITION, FSIM_LEVEL_BEHAVIOR };

typedef struct fsim_actuator_command { double aileron, elevator, rudder, throttle, flaps, gear_down, brake_left, brake_right; } fsim_actuator_command;
typedef struct fsim_attitude_command { double roll_rad, pitch_rad, heading_rad, max_bank_rad, throttle, airspeed_ms; } fsim_attitude_command;
typedef struct fsim_acceleration_command { double load_factor_g, roll_rate_rad_s, longitudinal_ms2, throttle; } fsim_acceleration_command;
typedef struct fsim_velocity_command { double airspeed_ms, vertical_speed_ms, heading_rad, turn_rate_rad_s; } fsim_velocity_command;
typedef struct fsim_position_command { double latitude_rad, longitude_rad, altitude_msl_m, airspeed_ms, capture_radius_m; } fsim_position_command;
typedef struct fsim_behavior_command {
    const char* id;                        /* "hold", "waypoints", "loiter", "pursuit", "evade", "formation", "aerobatics", or a registered id */
    uint32_t target;                       /* vehicle id for behaviours that need one */
    const char* const* param_names;
    const double* param_values;
    uint32_t param_count;
    const fsim_position_command* points;   /* route for "waypoints" */
    uint32_t point_count;
} fsim_behavior_command;

typedef struct fsim_message {
    uint32_t from, to, channel, format;
    double time_sent, time_delivered;
    const uint8_t* bytes;                  /* valid until the next world step */
    size_t length;
} fsim_message;

FSIM_API void fsim_world_options_init(fsim_world_options* options);
FSIM_API void fsim_vehicle_spec_init(fsim_vehicle_spec* spec);
FSIM_API void fsim_environment_init(fsim_environment* environment);

FSIM_API int fsim_world_create(const fsim_world_options* options, fsim_world** out);
FSIM_API void fsim_world_destroy(fsim_world* world);
FSIM_API int fsim_world_step(fsim_world* world, uint32_t steps);
FSIM_API double fsim_world_time(const fsim_world* world);
FSIM_API double fsim_world_step_seconds(const fsim_world* world);
FSIM_API uint64_t fsim_world_vehicle_steps(const fsim_world* world);
FSIM_API int fsim_world_published(const fsim_world* world);

FSIM_API int fsim_world_create_vehicle(fsim_world* world, const fsim_vehicle_spec* spec, uint32_t* id);
FSIM_API int fsim_world_remove_vehicle(fsim_world* world, uint32_t id);
/* spec NULL = the vehicle's own initial conditions. Only the initial-state fields of `spec` are used. */
FSIM_API int fsim_world_reset_vehicle(fsim_world* world, uint32_t id, const fsim_vehicle_spec* spec);
FSIM_API uint32_t fsim_world_find_vehicle(const fsim_world* world, const char* name);
FSIM_API uint32_t fsim_world_vehicle_count(const fsim_world* world);
/* Fills up to `capacity` ids; returns the total number of vehicles. */
FSIM_API uint32_t fsim_world_vehicle_ids(const fsim_world* world, uint32_t* ids, uint32_t capacity);
FSIM_API const char* fsim_vehicle_name(const fsim_world* world, uint32_t id);
FSIM_API const char* fsim_vehicle_type(const fsim_world* world, uint32_t id);

/* Pointers stay valid until the vehicle is removed; contents change on every step. */
FSIM_API const fsim_vehicle_state* fsim_vehicle_state_ptr(const fsim_world* world, uint32_t id);
FSIM_API const fsim_vehicle_state* fsim_vehicle_sensed_ptr(const fsim_world* world, uint32_t id);
FSIM_API int fsim_vehicle_get_property(fsim_world* world, uint32_t id, const char* path, double* value);
FSIM_API int fsim_vehicle_set_property(fsim_world* world, uint32_t id, const char* path, double value);

FSIM_API int fsim_vehicle_command_actuator(fsim_world* world, uint32_t id, const fsim_actuator_command* command);
FSIM_API int fsim_vehicle_command_attitude(fsim_world* world, uint32_t id, const fsim_attitude_command* command);
FSIM_API int fsim_vehicle_command_acceleration(fsim_world* world, uint32_t id, const fsim_acceleration_command* command);
FSIM_API int fsim_vehicle_command_velocity(fsim_world* world, uint32_t id, const fsim_velocity_command* command);
FSIM_API int fsim_vehicle_command_position(fsim_world* world, uint32_t id, const fsim_position_command* command);
FSIM_API int fsim_vehicle_command_behavior(fsim_world* world, uint32_t id, const fsim_behavior_command* command);
FSIM_API int fsim_vehicle_active_level(const fsim_world* world, uint32_t id);
/* 1 if the running behaviour reports itself finished. */
FSIM_API int fsim_vehicle_behavior_finished(const fsim_world* world, uint32_t id);
FSIM_API int fsim_vehicle_use_controller(fsim_world* world, uint32_t id, int level, const char* controller_id);
FSIM_API int fsim_vehicle_set_controller_parameter(fsim_world* world, uint32_t id, int level, const char* name, double value);

FSIM_API int fsim_world_get_environment(const fsim_world* world, fsim_environment* out);
FSIM_API int fsim_world_set_environment(fsim_world* world, const fsim_environment* environment);

/* Built-in effects by id ("gaussian_sensor_noise", "sensor_latency", "constant_force", "wind_gusts", "gnss_degradation")
 * with named parameters; id 0 = every vehicle, present and future. */
FSIM_API int fsim_vehicle_add_effect(fsim_world* world, uint32_t id, const char* effect_id, const char* const* param_names,
                                     const double* param_values, uint32_t param_count);
FSIM_API int fsim_vehicle_clear_effects(fsim_world* world, uint32_t id);

/* Communication: nodes are vehicle ids (external nodes: fsim_comm_create_node). `to` 0xFFFFFFFF = broadcast. */
FSIM_API int fsim_comm_create_node(fsim_world* world, uint32_t address);
FSIM_API int fsim_comm_send(fsim_world* world, uint32_t from, uint32_t to, uint32_t channel, uint32_t format, const void* bytes, size_t length);
FSIM_API uint32_t fsim_comm_inbox_count(const fsim_world* world, uint32_t node);
FSIM_API int fsim_comm_inbox_get(const fsim_world* world, uint32_t node, uint32_t index, fsim_message* out);
/* medium: "ideal" | "link" (params: range_m, latency_s, jitter_s, loss_probability) */
FSIM_API int fsim_comm_set_medium(fsim_world* world, const char* medium_id, const char* const* param_names, const double* param_values, uint32_t param_count);
/* protocol: "beacon" (params: period_s, channel) */
FSIM_API int fsim_comm_attach_protocol(fsim_world* world, uint32_t node, const char* protocol_id, const char* const* param_names,
                                       const double* param_values, uint32_t param_count);
/* Bridge a node over UDP: messages delivered to `node` go to remote_host:remote_port as "FSMG" datagrams (format in
 * fsim/Comm.h), datagrams arriving on local_port (0 = any) become messages from `node`. */
FSIM_API int fsim_comm_attach_udp_bridge(fsim_world* world, uint32_t node, uint16_t local_port, const char* remote_host, uint16_t remote_port);

/* ---------------------------------------------------------------------------
 * Scenario files (design 9.13): a JSON document describing the world, the
 * environment and the vehicles with their initial commands and effects
 * (format: include/fsim/Scenario.h). Strings returned through
 * fsim_scenario_world_options stay valid while the scenario lives.
 * ------------------------------------------------------------------------- */

typedef struct fsim_scenario fsim_scenario;

FSIM_API int fsim_scenario_load(const char* path, fsim_scenario** out);
FSIM_API int fsim_scenario_parse(const char* json, const char* source_name, fsim_scenario** out);
FSIM_API void fsim_scenario_destroy(fsim_scenario* scenario);
/* Fill `out` (fsim_world_options_init first) with the scenario's "world" section. */
FSIM_API int fsim_scenario_world_options(const fsim_scenario* scenario, fsim_world_options* out);
FSIM_API uint32_t fsim_scenario_vehicle_count(const fsim_scenario* scenario); /* instances, counting "count" */
/* Apply the environment, effects and vehicles to a world; `ids` receives up to `capacity` vehicle ids, `count` how many were created. */
FSIM_API int fsim_scenario_apply(fsim_world* world, const fsim_scenario* scenario, uint32_t* ids, size_t capacity, size_t* count);

#ifdef __cplusplus
}
#endif

#endif /* FSIM_C_H */
