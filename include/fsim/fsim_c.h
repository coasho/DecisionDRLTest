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
FSIM_API void fsim_set_last_error(const char* message); /* for libraries layered on this ABI (fsim_vision) */

/* The platform's log (written to stderr): records below `level` are dropped. (ABI 1.2) */
enum fsim_log_level { FSIM_LOG_TRACE = 0, FSIM_LOG_DEBUG, FSIM_LOG_INFO, FSIM_LOG_WARN, FSIM_LOG_ERROR, FSIM_LOG_OFF };
FSIM_API void fsim_set_log_level(int level);
FSIM_API int fsim_log_level(void);

FSIM_API void fsim_options_init(fsim_options* options);

FSIM_API int fsim_vecenv_create(const fsim_options* options, fsim_vecenv** out);
FSIM_API void fsim_vecenv_destroy(fsim_vecenv* env);

/* seed 0 keeps the current seed. */
FSIM_API int fsim_vecenv_reset(fsim_vecenv* env, uint64_t seed);
/* The batch's world as a world handle (vehicles "env<e>/<v>"): owned by the environment, do not destroy. */
struct fsim_world;
FSIM_API struct fsim_world* fsim_vecenv_world(fsim_vecenv* env);
/* actions: M*K*action_size floats. */
FSIM_API int fsim_vecenv_step(fsim_vecenv* env, const float* actions, size_t count);
FSIM_API int fsim_vecenv_buffers(const fsim_vecenv* env, fsim_buffers* out);

FSIM_API const char* fsim_vecenv_observation_name(const fsim_vecenv* env, uint32_t index);
FSIM_API const char* fsim_vecenv_action_name(const fsim_vecenv* env, uint32_t index);
FSIM_API uint64_t fsim_vecenv_vehicle_steps(const fsim_vecenv* env);

/* When an environment whose episode ended starts the next one. Either way the
 * episode's last observation is in final_observations and its flags are set
 * in the step that ended it. (ABI 1.2) */
enum fsim_autoreset {
    FSIM_AUTORESET_NEXT_STEP = 0, /* the following step, ignoring its action, returns the new episode's first
                                     observation with zero reward (Gymnasium's default; the default here) */
    FSIM_AUTORESET_SAME_STEP = 1  /* the step that ended the episode already returns the next one's first
                                     observation (Stable-Baselines3, Gymnasium's SAME_STEP) */
};
/* Takes effect from the next step. */
FSIM_API int fsim_vecenv_set_autoreset(fsim_vecenv* env, int mode);
FSIM_API int fsim_vecenv_autoreset(const fsim_vecenv* env);
/* World vehicle ids in batch order (index env * K + vehicle), for fsim_world_* calls on fsim_vecenv_world(env).
 * Fills up to `capacity`; returns M*K. */
FSIM_API uint32_t fsim_vecenv_vehicle_ids(const fsim_vecenv* env, uint32_t* ids, uint32_t capacity);

/* Registered task, observation and action ids (built-ins included), by index; "" past the end. (ABI 1.2) */
enum fsim_registry { FSIM_REGISTRY_TASK = 0, FSIM_REGISTRY_OBSERVATION = 1, FSIM_REGISTRY_ACTION = 2 };
FSIM_API const char* fsim_registered_id(int registry, uint32_t index);

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
#define FSIM_MAX_WHEELS 8
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
    double engine_rpm[FSIM_MAX_ENGINES];      /* propeller rpm; 0 for a jet */
    double engine_n2[FSIM_MAX_ENGINES];       /* turbine core speed, %; 0 otherwise */
    double afterburner[FSIM_MAX_ENGINES];     /* 0 off .. 1 full */
    double nozzle_position[FSIM_MAX_ENGINES]; /* 0 shut .. 1 wide open */
    double leading_edge_flap_rad;             /* + leading edge down */
    /* the wheeled gear units (JSBSim BOGEY contacts, in the aircraft file's order) */
    int32_t wheel_count;
    double wheel_compression_m[FSIM_MAX_WHEELS]; /* strut and tyre compression */
    double wheel_steer_rad[FSIM_MAX_WHEELS];     /* + right */
    double wheel_speed_ms[FSIM_MAX_WHEELS];      /* rolling speed at the rim */
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

/* Command a vehicle at a level (docs/control-architecture.md, 10.7): at the
 * level its own activity flies, the new setpoint updates it; otherwise a new
 * activity takes over. FSIM_INVALID_ARGUMENT with the reason in
 * fsim_last_error() if the vehicle is unknown or the command refused: a
 * behaviour nobody registered ("unknown_capability"; before ABI 1.4 it was
 * ignored and reported as FSIM_OK), or an axis an autopilot or override
 * activity holds ("authority_held"). */
FSIM_API int fsim_vehicle_command_actuator(fsim_world* world, uint32_t id, const fsim_actuator_command* command);
FSIM_API int fsim_vehicle_command_attitude(fsim_world* world, uint32_t id, const fsim_attitude_command* command);
FSIM_API int fsim_vehicle_command_acceleration(fsim_world* world, uint32_t id, const fsim_acceleration_command* command);
FSIM_API int fsim_vehicle_command_velocity(fsim_world* world, uint32_t id, const fsim_velocity_command* command);
FSIM_API int fsim_vehicle_command_position(fsim_world* world, uint32_t id, const fsim_position_command* command);
FSIM_API int fsim_vehicle_command_behavior(fsim_world* world, uint32_t id, const fsim_behavior_command* command);
FSIM_API int fsim_vehicle_active_level(const fsim_world* world, uint32_t id);

/* Batched calls (ABI 1.2): one call for many vehicles, so a binding pays its
 * per-call cost once per step rather than once per vehicle.
 *
 * fsim_world_gather_states copies the states of `count` vehicles, in the order
 * of `ids`, into `out` (`count` structs); `sensed` 1 reads them through the
 * sensor effects. An unknown id leaves its struct zeroed, and the call then
 * returns FSIM_INVALID_ARGUMENT once the others are filled.
 *
 * fsim_world_command_batch commands `count` vehicles at one level. `values`
 * holds a row of doubles per vehicle, `stride` doubles apart (0 = packed): the
 * fields of the level's fsim_*_command struct in order - actuator 8, attitude
 * 6, acceleration 4, velocity 4, position 5 (fsim_command_field_count).
 * fsim_hold() in a field works as it does in the single-vehicle calls.
 * Behaviours carry strings and routes and are not batched. Unknown ids are
 * skipped, and the call then returns FSIM_INVALID_ARGUMENT. */
FSIM_API int fsim_world_gather_states(const fsim_world* world, const uint32_t* ids, uint32_t count, int sensed, fsim_vehicle_state* out);
FSIM_API int fsim_world_command_batch(fsim_world* world, int level, const uint32_t* ids, uint32_t count, const double* values, uint32_t stride);
FSIM_API uint32_t fsim_command_field_count(int level); /* 0 for FSIM_LEVEL_BEHAVIOR and unknown levels */
/* 1 if the running behaviour reports itself finished. */
FSIM_API int fsim_vehicle_behavior_finished(const fsim_world* world, uint32_t id);
FSIM_API int fsim_vehicle_use_controller(fsim_world* world, uint32_t id, int level, const char* controller_id);
FSIM_API int fsim_vehicle_set_controller_parameter(fsim_world* world, uint32_t id, int level, const char* name, double value);
/* A parameter of the controller at `level` as the vehicle flies it: the
 * controller's default, the aircraft's own setting (its JSBSim properties
 * fsim/control/<controller>/<parameter>), or the last one set. (ABI 1.3) */
FSIM_API int fsim_vehicle_controller_parameter(const fsim_world* world, uint32_t id, int level, const char* name, double* value);

/* ---------------------------------------------------------------------------
 * Capabilities and activities (ABI 1.4; docs/control-architecture.md,
 * docs/sdk/control.md): what a vehicle offers, commands answered at once
 * (NEW, UPDATE, CANCEL), and the activities they become. Every call answers
 * in its fsim_command_result and returns FSIM_OK; a non-OK code means the
 * call itself was malformed (null pointers, a wrong field count).
 * ------------------------------------------------------------------------- */

typedef uint64_t fsim_activity_id; /* the vehicle's id in the high 32 bits, a per-vehicle count below */

enum fsim_source { FSIM_SOURCE_POLICY = 0, FSIM_SOURCE_AUTOPILOT = 1, FSIM_SOURCE_OVERRIDE = 2 };
enum fsim_range_policy { FSIM_RANGE_CLAMP = 0, FSIM_RANGE_REJECT = 1, FSIM_RANGE_NONE = 2 };
enum fsim_command_status { FSIM_COMMAND_ACCEPTED = 0, FSIM_COMMAND_REJECTED = 1, FSIM_COMMAND_CANCELED = 2 };
enum fsim_activity_state { FSIM_ACTIVITY_PENDING = 0, FSIM_ACTIVITY_ACTIVE, FSIM_ACTIVITY_COMPLETED, FSIM_ACTIVITY_FAILED, FSIM_ACTIVITY_CANCELED };
enum fsim_availability { FSIM_AVAILABLE = 0, FSIM_TEMPORARILY_UNAVAILABLE, FSIM_FAULTED, FSIM_DISABLED };

/* Call fsim_command_options_init() first: a policy's command, the
 * capability's default axes, values clamped to their ranges. */
typedef struct fsim_command_options {
    uint32_t struct_size;
    int32_t source;       /* fsim_source */
    uint32_t axes;        /* 0 = the capability's default (bits: roll, pitch, yaw, thrust, flaps, gear, brakes, speedbrake, trim) */
    int32_t range;        /* fsim_range_policy */
    uint32_t min_version; /* refuse a capability older than this */
} fsim_command_options;
FSIM_API void fsim_command_options_init(fsim_command_options* options);

typedef struct fsim_command_result {
    int32_t status;            /* fsim_command_status */
    int32_t reason;            /* why it was refused: fsim_reason_name() */
    fsim_activity_id activity; /* the activity made (NEW) or addressed (UPDATE, CANCEL) */
    fsim_activity_id other;    /* the activity holding the authority ("authority_held") */
    uint32_t flags;            /* 1: a value was clamped */
    uint32_t reserved;
} fsim_command_result;

typedef struct fsim_activity_info {
    fsim_activity_id id;
    uint32_t vehicle;
    uint32_t capability;       /* index for fsim_vehicle_capability */
    int32_t source;
    uint32_t axes;
    int32_t state;             /* fsim_activity_state */
    int32_t reason;            /* why it ended */
    fsim_activity_id by;       /* the activity that preempted it */
    uint32_t constraints;      /* last step: 1 saturated, 2 demand limited, 4 limit exceeded, 8 clamped, 16 axes reduced */
    uint32_t constraints_seen; /* every flag since it started */
    double start_time, end_time; /* simulation seconds; end_time NaN while it runs */
} fsim_activity_info;

/* Strings are owned by the world and stay valid until it is destroyed. */
typedef struct fsim_capability_info {
    const char* id;            /* "fsim.flight.attitude", "fsim.guidance.hold", "user.guidance.<id>" */
    uint32_t version;
    int32_t kind;              /* 0 flight, 1 guidance, 2 support, 3 status */
    uint32_t interactions;     /* 1 command, 2 update, 4 cancel, 8 settings, 16 status */
    int32_t level;             /* fsim_level its commands enter at */
    uint32_t axes;             /* owned by default */
    int32_t terminating;       /* 1: completes when it reaches its goal */
    int32_t needs_target;      /* 1: a behaviour that follows fsim_behavior_command.target */
    uint32_t parameter_count;  /* fsim_vehicle_capability_parameter */
    const char* behavior;      /* a guidance capability's behaviour id, else "" */
} fsim_capability_info;

typedef struct fsim_parameter_info {
    const char* name;          /* a level's command field ("roll_rad") or a behaviour's parameter ("radius_m") */
    const char* unit;
    double min, max, default_value; /* the range for this aircraft; NaN default = as at the start */
    int32_t optional;          /* 1: accepts fsim_hold() */
    int32_t reserved;
} fsim_parameter_info;

FSIM_API uint32_t fsim_vehicle_capability_count(fsim_world* world, uint32_t id);
FSIM_API int fsim_vehicle_capability(fsim_world* world, uint32_t id, uint32_t index, fsim_capability_info* out);
FSIM_API int fsim_vehicle_capability_parameter(fsim_world* world, uint32_t id, uint32_t capability, uint32_t index, fsim_parameter_info* out);
FSIM_API int fsim_vehicle_capability_status(const fsim_world* world, uint32_t id, const char* capability, int32_t* availability, int32_t* reason);

/* NEW at a level: `fields` in that level's fsim_*_command order (fsim_command_field_count values). */
FSIM_API int fsim_vehicle_submit(fsim_world* world, uint32_t id, int level, const double* fields, uint32_t count,
                                 const fsim_command_options* options, fsim_command_result* result);
FSIM_API int fsim_vehicle_submit_behavior(fsim_world* world, uint32_t id, const fsim_behavior_command* command,
                                          const fsim_command_options* options, fsim_command_result* result);
/* UPDATE: a new setpoint for a live activity, in its level's field order (the per-step path). */
FSIM_API int fsim_activity_update(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count, fsim_command_result* result);
/* UPDATE for many activities at once: rows of fields at the given stride (0 = the field count of each activity's level,
 * all rows the same level). FSIM_OK if every update was accepted, else FSIM_INVALID_ARGUMENT naming the first refused. */
FSIM_API int fsim_activity_update_batch(fsim_world* world, const fsim_activity_id* activities, uint32_t count, const double* values, uint32_t stride);
/* CANCEL: the activity ends and the vehicle flies its neutral default. */
FSIM_API int fsim_activity_cancel(fsim_world* world, fsim_activity_id activity, fsim_command_result* result);
/* A live or recently ended activity: FSIM_INVALID_ARGUMENT if the vehicle does not remember it. */
FSIM_API int fsim_activity_get(const fsim_world* world, fsim_activity_id activity, fsim_activity_info* out);
/* The vehicle's live activities, then the ended ones it remembers (newest first). */
FSIM_API uint32_t fsim_vehicle_activity_count(const fsim_world* world, uint32_t id);
FSIM_API int fsim_vehicle_activity(const fsim_world* world, uint32_t id, uint32_t index, fsim_activity_info* out);
FSIM_API const char* fsim_reason_name(int reason);             /* "authority_held", "goal_reached", ... */
/* The vehicle's profile (docs/control-architecture.md, 7): a field by its path
 * as the aircraft file names it, in the unit its name gives -
 * "envelope/clean/n_max", "envelope/clean/alpha_max_deg", "plant/roll/tau_s",
 * "identity/class", "control/pid_attitude/pitch/kp" - NaN if unknown; and a
 * section's version (0: the aircraft has none) and provenance (0 default,
 * 1 hangar, 2 identified, 3 user, 4 derived from the flight model). */
FSIM_API int fsim_vehicle_profile_value(const fsim_world* world, uint32_t id, const char* path, double* value);
FSIM_API int fsim_vehicle_profile_section(const fsim_world* world, uint32_t id, const char* section, uint32_t* version, int32_t* provenance);
FSIM_API const char* fsim_activity_state_name(int state);      /* "pending", "active", ... */

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
 * Recordings (design 9.5): an .fsrec file written by fsim_world_options.record_path,
 * read back as frames of samples (one per live vehicle) and vehicle events.
 * All pointers are owned by the recording handle.
 * ------------------------------------------------------------------------- */

typedef struct fsim_recording fsim_recording;

/* Same layout as fsim::ControlInputs (checked at build time). */
typedef struct fsim_control_inputs {
    double aileron, elevator, rudder;
    double throttle[FSIM_MAX_ENGINES];
    double flaps, gear_down, brake_left, brake_right;
} fsim_control_inputs;

typedef struct fsim_recorded_sample {
    uint32_t slot;
    fsim_vehicle_state state;
    fsim_control_inputs inputs;
} fsim_recorded_sample;

typedef struct fsim_recorded_event {
    uint32_t slot, id;
    uint64_t generation;
    int32_t alive;
    int32_t control_level;
    const char* name;
    const char* type;
    const char* model;
    double initial_latitude_deg, initial_longitude_deg, initial_altitude_msl_m, initial_heading_deg;
} fsim_recorded_event;

FSIM_API int fsim_recording_load(const char* path, fsim_recording** out);
FSIM_API void fsim_recording_destroy(fsim_recording* recording);
FSIM_API uint32_t fsim_recording_frame_count(const fsim_recording* recording);
FSIM_API double fsim_recording_frame_time(const fsim_recording* recording, uint32_t frame);
FSIM_API const fsim_recorded_sample* fsim_recording_samples(const fsim_recording* recording, uint32_t frame, uint32_t* count);
FSIM_API const fsim_recorded_event* fsim_recording_events(const fsim_recording* recording, uint32_t frame, uint32_t* count);
FSIM_API const char* fsim_recording_world_name(const fsim_recording* recording);
FSIM_API double fsim_recording_dt(const fsim_recording* recording);
FSIM_API int32_t fsim_recording_frame_skip(const fsim_recording* recording);

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

/* C++ interop: the object-model view of a C world handle (owned by the handle). */
namespace fsim { class World; }
FSIM_API fsim::World* fsim_world_object(fsim_world* world);
#endif

#endif /* FSIM_C_H */
