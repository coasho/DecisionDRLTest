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

/* The action's ranges: "fixed" (its own, the default) or "aircraft" - where the
 * aircraft's profile narrows a command's range (a fighter's load factor to its
 * n_min .. n_max), that range. From the next step. (ABI 1.4) */
FSIM_API int fsim_vecenv_set_action_ranges(fsim_vecenv* env, const char* mode);

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
/* Every field a level's command has (ABI 1.5; docs/rotorcraft.md): the
 * fsim_command_field_count fields, then the rotorcraft's - acceleration
 * pitch_rate_rad_s, yaw_rate_rad_s; velocity north_ms, east_ms; position
 * heading_rad: actuator 8, attitude 6, acceleration 6, velocity 6, position 6.
 * The capability calls (fsim_vehicle_submit, fsim_activity_update,
 * fsim_activity_update_batch_n) take either count; the fields left out are fsim_hold(). */
FSIM_API uint32_t fsim_command_field_count_full(int level);
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
/* ABI 1.8 appends FSIM_COMMAND_VALID: a validation's answer (fsim_command_options.validate_only) - it would be accepted; nothing flies. */
enum fsim_command_status { FSIM_COMMAND_ACCEPTED = 0, FSIM_COMMAND_REJECTED = 1, FSIM_COMMAND_CANCELED = 2, FSIM_COMMAND_VALID = 3 };
/* ABI 1.10 appends DISABLED (live: kept, flying nothing, until enabled) and DELETED (a sticky disable: ended). */
enum fsim_activity_state { FSIM_ACTIVITY_PENDING = 0, FSIM_ACTIVITY_ACTIVE, FSIM_ACTIVITY_COMPLETED, FSIM_ACTIVITY_FAILED, FSIM_ACTIVITY_CANCELED,
                           FSIM_ACTIVITY_DISABLED, FSIM_ACTIVITY_DELETED };
/* An activity command (ABI 1.10; docs/flight-autonomy.md, 4.10): fsim_activity_command. */
enum fsim_activity_command_kind { FSIM_ACTIVITY_DISABLE = 0, FSIM_ACTIVITY_ENABLE, FSIM_ACTIVITY_RESET, FSIM_ACTIVITY_DELETE, FSIM_ACTIVITY_CHANGE_RANK,
                                  FSIM_ACTIVITY_UNASSIGN };
/* fsim_command_result.flags (ABI 1.9 adds DEFERRED: accepted to wait - its start window, or axes held by what it may not interrupt). */
enum fsim_command_flag { FSIM_COMMAND_CLAMPED = 1, FSIM_COMMAND_DEFERRED = 2,
                         FSIM_COMMAND_OVERRIDDEN = 4 /* ABI 1.16: accepted over a soft rejection (override_rejection): fsim_last_command_endurance */ };
/* Ranks, queues and time windows (ABI 1.9; docs/flight-autonomy.md, 4.9). Why a pending activity has not started: */
enum fsim_activity_wait { FSIM_WAIT_NONE = 0, FSIM_WAIT_SCHEDULED, FSIM_WAIT_QUEUED };
/* What its record rests on (A-GRA's ActivityBasisEnum): flown (actual), or waiting to start (planned). */
enum fsim_activity_basis { FSIM_BASIS_ACTUAL = 0, FSIM_BASIS_SENSED, FSIM_BASIS_PREDICTED, FSIM_BASIS_PLANNED };
/* Which of a command's time windows must be met (A-GRA's SchedulingCriticalityEnum). */
enum fsim_time_criticality { FSIM_CRITICAL_NONE = 0, FSIM_CRITICAL_START, FSIM_CRITICAL_END, FSIM_CRITICAL_START_AND_END };
/* fsim_command_options.precedence_override: the capability's own precedence. */
#define FSIM_NO_PRECEDENCE_OVERRIDE 0xFFFFFFFFu
/* ABI 1.7 appends FSIM_UNAVAILABLE (when it returns is not known: a capability the vehicle does not offer)
 * and FSIM_EXPENDED; FSIM_DISABLED means switched off (fsim_vehicle_set_availability), never "not supported". */
enum fsim_availability { FSIM_AVAILABLE = 0, FSIM_TEMPORARILY_UNAVAILABLE, FSIM_FAULTED, FSIM_DISABLED, FSIM_UNAVAILABLE, FSIM_EXPENDED };

/* A requirement a command comes from (ABI 1.8; A-GRA's Traceability): the caller's ids. */
enum fsim_requirement_kind { FSIM_REQUIREMENT_NONE = 0, FSIM_REQUIREMENT_EFFECT, FSIM_REQUIREMENT_ACTION, FSIM_REQUIREMENT_TASK, FSIM_REQUIREMENT_COMMAND };
typedef struct fsim_requirement {
    int32_t kind;     /* fsim_requirement_kind; 0: none */
    uint32_t reserved;
    uint64_t id;
} fsim_requirement;
#define FSIM_MAX_REQUIREMENTS 4

/* Call fsim_command_options_init() first: a policy's command, the
 * capability's default axes, values clamped to their ranges. */
typedef struct fsim_command_options {
    uint32_t struct_size;
    int32_t source;       /* fsim_source */
    uint32_t axes;        /* 0 = the capability's default (bits: roll, pitch, yaw, thrust, flaps, gear, brakes, speedbrake, trim) */
    int32_t range;        /* fsim_range_policy */
    uint32_t min_version; /* refuse a capability older than this */
    /* ABI 1.8, the command envelope (docs/flight-autonomy.md, 4.8): */
    uint32_t reserved;
    uint64_t command_id;  /* the caller's id for it, echoed in its answers (fsim_last_command_detail) and kept with its activity; 0 none */
    fsim_requirement trace[FSIM_MAX_REQUIREMENTS]; /* the requirements it comes from */
    int32_t interactive;  /* 1 (fsim_command_options_init): its activity takes activity commands */
    int32_t validate_only; /* 1: checked and answered as a NEW would be - FSIM_COMMAND_VALID, or rejected with every finding - flying nothing */
    /* ABI 1.9, how it is arbitrated and scheduled (docs/flight-autonomy.md, 4.9): */
    int32_t interrupt;     /* 1 (fsim_command_options_init): takes contested axes from what it may interrupt; 0: waits until they are free
                              (a policy's), or defers to rank (the platform's). A-GRA's omission is 0 */
    int32_t override_rejection; /* fly it where a soft rejection would refuse it; none exists yet */
    uint16_t rank_priority, rank_precedence; /* its rank (A-GRA's Ranking.Rank): lower first; 0, 0 (the default) first of all */
    uint32_t precedence_override; /* the capability's precedence for this command alone; FSIM_NO_PRECEDENCE_OVERRIDE (the default): its own.
                                     The platform's sources only: a policy's is refused not_allowed */
    double start_not_before, start_not_after, end_not_before, end_not_after; /* simulation seconds; NaN (the default): no bound */
    int32_t criticality;   /* fsim_time_criticality: which windows must be met (a start missed, an end missed: failed, time_constraint) */
    uint32_t controller;   /* ABI 1.12: a policy's controller (docs/flight-autonomy.md, 4.12), 0 (the default) the default policy; under
                              FSIM_CONTROL_GRANTED its capability's grant must be this controller's */
} fsim_command_options;
FSIM_API void fsim_command_options_init(fsim_command_options* options);

typedef struct fsim_command_result {
    int32_t status;            /* fsim_command_status */
    int32_t reason;            /* why it was refused: fsim_reason_name() */
    fsim_activity_id activity; /* the activity made (NEW) or addressed (UPDATE, CANCEL) */
    fsim_activity_id other;    /* the activity holding the authority ("authority_held") */
    uint32_t flags;            /* fsim_command_flag: 1 a value was clamped; 2 deferred - it waits to start, `other` naming what it waits for */
    uint32_t reserved;         /* ABI 1.6: the field, route point or curve segment the answer is about, plus one (0: none);
                                  fsim_last_command_detail() has the rest */
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
    uint32_t constraints;      /* last step: 1 saturated, 2 demand limited, 4 limit exceeded, 8 clamped, 16 axes reduced, 32 (ABI 1.32)
                                  a route off its path by more than its segment's required navigation performance */
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
    uint32_t axis_groups;      /* what it may own apart (fsim_command_options.axes): 1 roll+yaw, 2 pitch, 4 thrust, 8 any primary axis; 0 all or nothing */
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
    int32_t unsupported;       /* 1: this aircraft has nothing the field moves (a wing's pitch rate, a helicopter's
                                  flaps): a command that sets it other than to fsim_hold() or its default is refused
                                  "not_supported" with the field (ABI 1.7; 1.5 and 1.6: "invalid_parameter") */
} fsim_parameter_info;

FSIM_API uint32_t fsim_vehicle_capability_count(fsim_world* world, uint32_t id);
FSIM_API int fsim_vehicle_capability(fsim_world* world, uint32_t id, uint32_t index, fsim_capability_info* out);
FSIM_API int fsim_vehicle_capability_parameter(fsim_world* world, uint32_t id, uint32_t capability, uint32_t index, fsim_parameter_info* out);
FSIM_API int fsim_vehicle_capability_status(const fsim_world* world, uint32_t id, const char* capability, int32_t* availability, int32_t* reason);

/* NEW at a level: `fields` in that level's fsim_*_command order - fsim_command_field_count values, or
 * fsim_command_field_count_full with the rotorcraft's (ABI 1.5). */
FSIM_API int fsim_vehicle_submit(fsim_world* world, uint32_t id, int level, const double* fields, uint32_t count,
                                 const fsim_command_options* options, fsim_command_result* result);
FSIM_API int fsim_vehicle_submit_behavior(fsim_world* world, uint32_t id, const fsim_behavior_command* command,
                                          const fsim_command_options* options, fsim_command_result* result);
/* NEW for a support effector the vehicle has, set directly beside the cascade.
 * Fields: gear [down: 1 or 0], flaps [position 0..1], wheel brakes [left,
 * right 0..1], speedbrake [position 0..1], pitch trim [position -1..1, + nose
 * down]. Refused as "unavailable" by the placards: no gear up on the ground, no
 * gear or flaps out above their speeds. fsim_activity_update takes the same fields.
 * FSIM_SUPPORT_ENGINES is fsim.flight.engines, where the aircraft has more than
 * one engine: a throttle 0..1 per engine (1 to 4 fields; fsim_hold() keeps one),
 * owning thrust beside the cascade. */
enum fsim_support { FSIM_SUPPORT_GEAR = 0, FSIM_SUPPORT_FLAPS, FSIM_SUPPORT_WHEEL_BRAKES, FSIM_SUPPORT_SPEEDBRAKE, FSIM_SUPPORT_PITCH_TRIM,
                    FSIM_SUPPORT_ENGINES };
FSIM_API int fsim_vehicle_submit_support(fsim_world* world, uint32_t id, int kind, const double* fields, uint32_t count,
                                         const fsim_command_options* options, fsim_command_result* result);
/* UPDATE: a new setpoint for a live activity, in its level's field order (the per-step path; either count, as NEW). */
FSIM_API int fsim_activity_update(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count, fsim_command_result* result);
/* UPDATE for many activities at once: rows of fields at the given stride (0 = the field count of each activity's level,
 * all rows the same level). FSIM_OK if every update was accepted, else FSIM_INVALID_ARGUMENT naming the first refused. */
FSIM_API int fsim_activity_update_batch(fsim_world* world, const fsim_activity_id* activities, uint32_t count, const double* values, uint32_t stride);
/* As fsim_activity_update_batch with `fields` values in each row (ABI 1.5): the level's fsim_command_field_count or
 * fsim_command_field_count_full (a support activity: its own count); stride 0 = `fields`. */
FSIM_API int fsim_activity_update_batch_n(fsim_world* world, const fsim_activity_id* activities, uint32_t count, const double* values,
                                          uint32_t stride, uint32_t fields);
/* CANCEL: the activity ends and its axes fly the vehicle default. */
FSIM_API int fsim_activity_cancel(fsim_world* world, fsim_activity_id activity, fsim_command_result* result);
/* UPDATE and CANCEL declaring the caller's source (fsim_source), as a NEW's options
 * do (ABI 1.6; docs/vehicle-interface.md, 6.1). Under FSIM_CONTROL_GRANTED a source
 * below the activity's may not address it: rejected "authority_held", `other` the
 * activity - a policy cannot change or end what the platform's own sources fly.
 * The calls without _as are the policy's; in Open mode the source changes nothing. */
FSIM_API int fsim_activity_update_as(fsim_world* world, fsim_activity_id activity, int source, const double* fields, uint32_t count,
                                     fsim_command_result* result);
FSIM_API int fsim_activity_cancel_as(fsim_world* world, fsim_activity_id activity, int source, fsim_command_result* result);
/* As the *_as calls, declaring a policy's controller too (ABI 1.12; docs/flight-autonomy.md, 4.12): under
 * FSIM_CONTROL_GRANTED a controller may not address another's activity either - rejected "authority_held", `other` the
 * activity. The *_as calls are controller 0's. */
FSIM_API int fsim_activity_update_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                     uint32_t count, fsim_command_result* result);
FSIM_API int fsim_activity_cancel_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, fsim_command_result* result);
/* An activity command (ABI 1.10) for a live activity - flying, waiting or disabled - declaring `source` and `controller`
 * as the *_by calls do: disable (it stops flying and is kept), enable, reset (over from its beginning), delete (a sticky
 * disable: it ends), change its rank (to rank_priority, rank_precedence), unassign (it gives up its axes and waits for
 * them again). Answered as an UPDATE: rejected not_interactive where its command said it takes none
 * (fsim_command_options.interactive 0), queue_full where a flying one has no room to be kept. FSIM_INVALID_ARGUMENT for
 * a command beyond the enum. */
FSIM_API int fsim_activity_command(fsim_world* world, fsim_activity_id activity, int command, uint32_t rank_priority, uint32_t rank_precedence,
                                   int source, uint32_t controller, fsim_command_result* result);
FSIM_API const char* fsim_activity_command_name(int command); /* "disable", "enable", "reset", "delete", "change_rank", "unassign" */
/* The operator's input to a route's conditional branch (ABI 1.34; docs/flight-autonomy.md, 4.37): branch `branch` of the
 * route `activity` flies or waits to fly - one that takes it - `commanded` (1), or no longer (0), declaring the caller's
 * source and controller as fsim_activity_update_by. Held until the route is flown afresh (an UPDATE, a Reset). Refused
 * "invalid_parameter", `reserved` the branch + 1, for one it has not or one that takes no operator input;
 * "wrong_command_type" for an activity that is no route. */
FSIM_API int fsim_activity_command_branch(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, uint32_t branch, int commanded,
                                          fsim_command_result* result);
/* What flies the primary axes nobody owns: FSIM_DEFAULT_NEUTRAL (surfaces
 * centred, throttle 0 - every vehicle's default) or FSIM_DEFAULT_HOLD (the
 * heading, airspeed and height each had when it was let go). `reason` (may be
 * null) says why it was refused: "controller_not_axis_aware". */
enum fsim_vehicle_default { FSIM_DEFAULT_NEUTRAL = 0, FSIM_DEFAULT_HOLD = 1 };
FSIM_API int fsim_vehicle_set_default(fsim_world* world, uint32_t id, int mode, int32_t* reason);
FSIM_API int fsim_vehicle_get_default(const fsim_world* world, uint32_t id, int32_t* mode);
/* Envelope protection (docs/control-architecture.md, 11): LIMIT (the default for
 * an aircraft whose profile has an envelope) limits what the control system
 * demands to the envelope and reports the state's exceedances; REPORT only
 * reports; OFF (the default without an envelope) does neither. It never keeps
 * the aircraft inside: what crosses a limit is reported, not prevented.
 * fsim_vehicle_envelope gives what it saw since the last call, per limit
 * (fsim_limit_name(i): "load_factor_max", "alpha_max", "cas_min", ...), and
 * starts a new count. Excesses are in g, rad, rad/s, m/s (calibrated) or Mach. */
enum fsim_protection { FSIM_PROTECTION_OFF = 0, FSIM_PROTECTION_REPORT = 1, FSIM_PROTECTION_LIMIT = 2 };
#define FSIM_LIMIT_COUNT 10
typedef struct fsim_envelope_status {
    int32_t mode;                                /* fsim_protection */
    uint32_t reserved;
    uint32_t limited_updates[FSIM_LIMIT_COUNT];  /* control updates in which the demand was limited for it */
    uint32_t exceeded_updates[FSIM_LIMIT_COUNT]; /* control updates in which the state was beyond it */
    double exceeded_s[FSIM_LIMIT_COUNT];         /* how long, s */
    double worst_excess[FSIM_LIMIT_COUNT];       /* how far, at most */
} fsim_envelope_status;
FSIM_API int fsim_vehicle_set_protection(fsim_world* world, uint32_t id, int mode);
FSIM_API int fsim_vehicle_get_protection(const fsim_world* world, uint32_t id, int32_t* mode);
FSIM_API int fsim_vehicle_envelope(fsim_world* world, uint32_t id, fsim_envelope_status* out);
FSIM_API const char* fsim_limit_name(int limit);
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

/* ---------------------------------------------------------------------------
 * The Vehicle Interface (ABI 1.6; docs/vehicle-interface.md): what a command's
 * answer was about, how far an activity has got, what the cascade commands,
 * and which of A-GRA's flight capability types a capability is. Structs with a
 * struct_size are filled up to the size the caller gives (its _init() sets it),
 * so a caller built against an older header keeps working.
 * ------------------------------------------------------------------------- */

/* What the world's last answer to a NEW, UPDATE or CANCEL was about: a
 * rejection's, or the first value a clamp changed. */
typedef struct fsim_command_detail {
    uint32_t struct_size;
    int32_t reason;     /* the answer's: fsim_reason_name() */
    int32_t index;      /* the field (the command struct's order; a behaviour's parameter in its descriptor's), route point or
                           curve segment; -1: none */
    int32_t constraint; /* the performance limit the value broke: fsim_constraint_name(); 0 none */
    double from, to;    /* a curve segment's section that breaks it, its parameter 0..1; NaN otherwise */
    /* ABI 1.8 (docs/flight-autonomy.md, 4.8): */
    int32_t new_activity;      /* 1: a NEW that made an activity (A-GRA's NewActivity) */
    uint32_t finding_count;    /* every reason it cannot be flown as asked, the first the answer's: fsim_last_command_finding */
    uint32_t adjustment_count; /* every value it is flown with other than asked: fsim_last_command_adjustment */
    uint32_t reserved;
    uint64_t command_id;       /* the command's id: a NEW's, or the addressed activity's */
    uint64_t associated;       /* an id the reason is about: the activity holding the authority; 0 none */
    const char* description;   /* the reason in words; "" when accepted */
    uint64_t suggestion;       /* ABI 1.11: a refused command's suggestion - the task the platform keeps with what it can fly in its
                                  place (fsim_vehicle_task_status; FSIM_SUGGESTED_TASK set); 0 none */
} fsim_command_detail;
FSIM_API void fsim_command_detail_init(fsim_command_detail* detail);
FSIM_API int fsim_last_command_detail(const fsim_world* world, fsim_command_detail* out);

/* ABI 1.16 (docs/flight-autonomy.md, 4.18; A-GRA's MA_InsufficientEnduranceType): what the last command's flight needs -
 * a route that does not repeat, a timed pattern, a curve - against what the vehicle has above its reserve. Given when it
 * needs more: refused insufficient_endurance (a soft rejection), or accepted over it with override_rejection
 * (FSIM_COMMAND_OVERRIDDEN); `energy` 0 otherwise. */
typedef struct fsim_command_endurance {
    uint32_t struct_size;
    int32_t energy;     /* fsim_energy: FSIM_ENERGY_FUEL (kg), FSIM_ENERGY_BATTERY (J); 0: none judged */
    double remaining;   /* what the vehicle has above its reserve */
    double required;    /* what the flight needs, flown level at each leg's speed and altitude */
    double remaining_s; /* how long `remaining` lasts at what it consumes now */
    double required_s;  /* how long the flight takes */
} fsim_command_endurance;
FSIM_API void fsim_command_endurance_init(fsim_command_endurance* endurance);
FSIM_API int fsim_last_command_endurance(const fsim_world* world, fsim_command_endurance* out);

/* ABI 1.17 (docs/flight-autonomy.md, 4.19; A-GRA's TerrainConstraint, a Point4D): where the last command's path first goes
 * below the terrain - a route's legs and turns and what it flies after its last point, a pattern's lap, a curve, an hsa's
 * line a minute ahead. Given when refused terrain_conflict; `hit` 0 otherwise. */
typedef struct fsim_command_terrain {
    uint32_t struct_size;
    int32_t hit;           /* 1: its path meets the ground there */
    int32_t index;         /* the route point it flies to there, or the curve segment; -1 otherwise */
    int32_t reserved;
    double latitude_rad, longitude_rad;
    double altitude_msl_m; /* the path's there, above the WGS-84 ellipsoid */
    double ground_m;       /* the terrain's there, likewise */
    double time_s;         /* from the command: when it would be there, at its planned speeds */
} fsim_command_terrain;
FSIM_API void fsim_command_terrain_init(fsim_command_terrain* terrain);
FSIM_API int fsim_last_command_terrain(const fsim_world* world, fsim_command_terrain* out);
/* ABI 1.17 (4.19; VI 1.2.6.9, A-GRA's elevation request): the ground's height above the WGS-84 ellipsoid at `count`
 * places - the physics' own ground, which commanded paths are checked against - into `height_m`, NaN where it has no
 * data (a terrain tile it cannot load). Returns how many it knows, or a negative fsim_status. */
FSIM_API int fsim_world_terrain(const fsim_world* world, uint32_t count, const double* latitude_rad, const double* longitude_rad,
                                double* height_m);
FSIM_API const char* fsim_constraint_name(int constraint); /* "max_airspeed", "max_orientation", "max_climb_rate", ... */

/* How far an activity has got and what it commands: a guidance mode's (a route,
 * a loiter, a hold), as its behaviour reported it after the last world step.
 * NaN, or 0 counts, where it says nothing. */
typedef struct fsim_activity_progress {
    uint32_t struct_size;
    uint32_t segment;       /* the waypoint, curve segment or pattern leg flown now (from 0) */
    uint32_t segments;      /* of how many; 0: nothing segmented */
    uint32_t laps;          /* a pattern's or a repeating route's, completed */
    uint64_t segment_id;    /* the waypoint's id, where the route gave one */
    double percent;         /* of the whole activity, 0..100 */
    double segment_percent;
    double distance_to_go_m, time_to_go_s; /* to the end, at the ground speed now */
    double cross_track_m;   /* + right of the path */
    double course_rad, heading_rad, altitude_msl_m; /* what it commands: A-GRA's VehicleCommandState */
    double speed_ms;
    double speed_reference; /* 0 true airspeed, 1 calibrated, 2 ground speed, 3 Mach */
    /* ABI 1.30 (4.33): a route's next point with an arrival window - when it is estimated to arrive there (simulation
       seconds), and that against its window: + late, - early, 0 within (NaN: none) */
    double arrival_s, arrival_delta_s;
} fsim_activity_progress;
FSIM_API void fsim_activity_progress_init(fsim_activity_progress* progress);
FSIM_API int fsim_activity_get_progress(const fsim_world* world, fsim_activity_id activity, fsim_activity_progress* out);

/* What the cascade asked for in its last control update, level by level; NaN
 * where no level set it. */
typedef struct fsim_commanded_state {
    uint32_t struct_size;
    int32_t top_level;                                /* the highest level that ran: fsim_level */
    double latitude_rad, longitude_rad, altitude_msl_m; /* the position level's point */
    double heading_rad, turn_rate_rad_s;              /* the velocity level's (the heading else the attitude level's) */
    double airspeed_ms, vertical_speed_ms, north_ms, east_ms;
    double roll_rad, pitch_rad;                       /* the attitude level's */
    double load_factor_g, roll_rate_rad_s, pitch_rate_rad_s, yaw_rate_rad_s; /* the acceleration level's */
    double throttle;                                  /* what the actuators were given (the first engine's) */
    /* appended in ABI 1.12 (A-GRA's VehicleCommandState; docs/flight-autonomy.md, 4.12) */
    double north_acceleration_ms2, east_acceleration_ms2, down_acceleration_ms2; /* a wing's, over the Earth, at the attitude it
                                                      flies; NaN where no longitudinal acceleration is commanded, and for a rotorcraft */
    double altitude_m, altitude_reference;            /* as its mode commanded it, in its reference (fsim_altitude_reference): a live
                                                      hsa's or pattern's, a route's point flown to's; a curve's and the position
                                                      level's above sea level; NaN none */
} fsim_commanded_state;
FSIM_API void fsim_commanded_state_init(fsim_commanded_state* state);
FSIM_API int fsim_vehicle_commanded(const fsim_world* world, uint32_t id, fsim_commanded_state* out);

/* The Vehicle Interface's modes: fixed-size setpoints, as a level's are, that
 * take UPDATE (docs/vehicle-interface.md, 4).
 * - FSIM_MODE_HSA is fsim.guidance.hsa (A-GRA's HSA/CSA): fields heading_rad,
 *   course_rad (one of them), speed, speed_reference (fsim_speed_reference),
 *   altitude_m, altitude_reference (fsim_altitude_reference) and, from ABI
 *   1.15, speed_optimization (fsim_speed_optimization: the performance
 *   tables' best-range or best-endurance speed, flown at the altitude and
 *   weight now; a speed replaces it, it a speed) and, from ABI 1.20,
 *   direction_reference (fsim_direction_reference: magnetic, the heading or
 *   course is flown turned by the declination where the aircraft is, at the
 *   world's date); fsim_hold() leaves one out.
 *   A NEW continues what a live hsa commanded, else what the aircraft flies
 *   now; a reference alone takes the aircraft's own value in it.
 *   fsim_activity_update takes the same fields and keeps the ones left out.
 *   Six fields (ABI 1.14's) leave the optimisation out, seven (1.15's) the
 *   direction reference.
 * - FSIM_MODE_ROUTE is fsim.guidance.route (A-GRA's waypoint following):
 *   fields projection (fsim_projection), repeat (1: fly it again from its first
 *   point), end (fsim_end_behavior), start (the waypoint flown to first). Its
 *   waypoints go beside them, through fsim_vehicle_submit_route below: with
 *   fsim_vehicle_submit_mode it has none and is refused (invalid_waypoint).
 *   fsim_activity_update with a route's fields keeps its waypoints and flies
 *   it afresh from its start; fsim_hold() keeps an option.
 * - FSIM_MODE_PATTERN is fsim.guidance.pattern (A-GRA's loiter): fields
 *   pattern (fsim_pattern_kind), latitude_rad, longitude_rad (its centre or
 *   fix), altitude_m, altitude_reference, radius_m, clockwise (1 right turns),
 *   course_rad (the inbound course, a figure-eight's axis), leg_m, speed,
 *   speed_reference, duration_s and, from ABI 1.15, speed_optimization (as
 *   an hsa's; twelve fields leave it out). fsim_hold() leaves one out: a NEW
 *   takes its default (an orbit here, as the aircraft flies now), an UPDATE keeps it.
 *   From ABI 1.21 (docs/flight-autonomy.md, 4.23; A-GRA's orbit as its schema
 *   gives it), after them: direction_reference (fsim_direction_reference of the
 *   course or heading), heading_rad (or the inbound heading: the course it
 *   makes good on it), leg_s (or the legs by time: the inbound leg's), bank_rad
 *   (or the turns by bank), orbits (laps: then it completes; with a duration,
 *   the first), latitude2_rad, longitude2_rad, radius2_m (a racetrack's or a
 *   figure-eight's second circle), entry_latitude_rad, entry_longitude_rad
 *   (where it joins the pattern, flown to directly), exit_latitude_rad,
 *   exit_longitude_rad (where it leaves, its duration or laps flown). Of a
 *   course and a heading, legs and their time, a radius and a bank, given both
 *   the first flies; in an UPDATE either replaces both. From ABI 1.22 (4.24,
 *   A-GRA's hold): turn_rate_rad_s, turn_type (fsim_hold_turn) - other ways to
 *   give the radius, after the bank - hold_entry (fsim_hold_entry: a
 *   racetrack's or a hold's way in; left out, direct to the fix) and
 *   hold_context (fsim_hold_context: ATC's defaults for every one); a hold's
 *   entry and exit times are the command's time window. From ABI 1.23 (4.25):
 *   FSIM_PATTERN_HOVER, a rotorcraft's hover over its point, its duration
 *   from its arrival (a wing's refused "not_supported", field 0);
 *   and its point in a frame (A-GRA's relative point): frame (the frame's id,
 *   fsim_world_create_frame), frame_rotation (fsim_frame_rotation),
 *   frame_offsets (fsim_frame_offsets), frame_x_m, frame_y_m, frame_z_m (z
 *   down; left out, the pattern's own altitude) - the pattern's point is the
 *   frame's, carried as it moves (a vehicle's frame whose vehicle is gone:
 *   the activity fails "target_lost"). A point replaces a frame in
 *   an UPDATE, and a frame a point. Twelve or more fields are taken, the rest
 *   left out.
 * - FSIM_MODE_CURVE is fsim.guidance.curve (A-GRA's curve following): fields
 *   latitude_rad, longitude_rad, altitude_m (the reference its segments are
 *   from; left out: the aircraft at the NEW), speed_min_ms, speed_max_ms (the
 *   ground speeds to fly it within), duration_s (or the time to fly all of
 *   it), end (fsim_end_behavior), append (1 in an UPDATE: its segments after
 *   the curve's end). From ABI 1.25 (docs/flight-autonomy.md, 4.27), after
 *   them: altitude_reference (fsim_altitude_reference of altitude_m and its
 *   range), altitude_min_m, altitude_max_m (the reference's altitude range:
 *   left out, the aircraft's is held within it), point_rotation
 *   (fsim_frame_rotation of its control points' axes: turned as its frame is,
 *   so only with one - ATTITUDE in three dimensions), point_offsets
 *   (fsim_frame_offsets: CARTESIAN the plane every local path is laid out in,
 *   GREAT_CIRCLE A-GRA's azimuthal equidistant layout, RHUMB), point_z
 *   (fsim_curve_z of their third), then its reference in a frame: frame,
 *   frame_rotation, frame_offsets, frame_x_m, frame_y_m, frame_z_m (as a
 *   pattern's, 1.23). Where a curve is changes only with a new curve's
 *   segments: those fields in an UPDATE of its options alone are refused;
 *   segments appended go on from its reference, their points read as its (as
 *   A-GRA's append does), those given with them not used. Eight or more fields
 *   are taken, the rest left out. Its segments go beside them, through
 *   fsim_vehicle_submit_curve below; fsim_activity_update with a curve's
 *   fields changes how it is flown, not where.
 * - FSIM_MODE_MUST_FLY (ABI 1.39; docs/flight-autonomy.md, 4.42) is fsim.guidance.must_fly (A-GRA's must fly): fields
 *   location (fsim_must_fly_location), latitude_rad, longitude_rad (a point's), altitude_m, altitude_reference (the
 *   altitude it flies over the location at; left out, a point's the aircraft's, a vehicle's as far above it as the
 *   aircraft is, at least 500 ft, an operational point's its own), target (a vehicle's id, or an operational point's:
 *   fsim_world_set_op_point), ingress_min_rad, ingress_max_rad (the window of bearings from the location it approaches
 *   from, the least clockwise to the most: both or neither; left out, an operational point's own), speed,
 *   speed_reference. It is laid out as a route from where the aircraft is - the points it approaches through, then the
 *   location, flown over - and completes as the location is passed, flying on along its course there.
 *   fsim_activity_update merges the fields given, a location another than it was replacing the location's own. */
enum fsim_mode { FSIM_MODE_HSA = 0, FSIM_MODE_ROUTE = 1, FSIM_MODE_PATTERN = 2, FSIM_MODE_CURVE = 3, FSIM_MODE_MUST_FLY = 4 /* ABI 1.39 */ };
/* A must fly's location (ABI 1.39; A-GRA's MustFlyLocationType): a point, another vehicle, an operational point by its id;
 * from ABI 1.40 a zone given with it (fsim_vehicle_submit_must_fly) or an operational zone by its id, entered */
enum fsim_must_fly_location { FSIM_MUST_FLY_POINT = 0, FSIM_MUST_FLY_ENTITY, FSIM_MUST_FLY_OP_POINT, FSIM_MUST_FLY_ZONE, FSIM_MUST_FLY_OP_ZONE };
enum fsim_pattern_kind { FSIM_PATTERN_ORBIT = 0, FSIM_PATTERN_RACETRACK, FSIM_PATTERN_FIGURE_EIGHT, FSIM_PATTERN_HOLD,
                         FSIM_PATTERN_HOVER /* ABI 1.23: a rotorcraft's */ };
enum fsim_hold_turn { FSIM_HOLD_TURN_STANDARD = 0, FSIM_HOLD_TURN_MIL_POWER, FSIM_HOLD_TURN_RELAX }; /* A-GRA's MA_HoldTurnTypeEnum (ABI 1.22) */
enum fsim_hold_entry { FSIM_HOLD_ENTRY_DIRECT = 0, FSIM_HOLD_ENTRY_ANCHOR, FSIM_HOLD_ENTRY_INBOUND, FSIM_HOLD_ENTRY_OUTBOUND,
                       FSIM_HOLD_ENTRY_PARALLEL, FSIM_HOLD_ENTRY_TEARDROP }; /* A-GRA's MA_HoldEntryTypeEnum and ATC's (ABI 1.22) */
enum fsim_hold_context { FSIM_HOLD_CONTEXT_ADMIN = 0, FSIM_HOLD_CONTEXT_TACTICAL, FSIM_HOLD_CONTEXT_ATC }; /* A-GRA's MA_HoldContextEnum (ABI 1.22) */
enum fsim_speed_reference { FSIM_SPEED_TRUE_AIRSPEED = 0, FSIM_SPEED_CALIBRATED_AIRSPEED, FSIM_SPEED_GROUND_SPEED, FSIM_SPEED_MACH };
enum fsim_altitude_reference { FSIM_ALTITUDE_MSL = 0, FSIM_ALTITUDE_ABOVE_GROUND, FSIM_ALTITUDE_ELLIPSOID,
                               FSIM_ALTITUDE_BAROMETRIC /* ABI 1.18: what its altimeter reads, set to its QNH (fsim_vehicle_set_qnh); an hsa's
                                                           or a pattern's, a route's being FA-6's */ };
enum fsim_speed_optimization { FSIM_SPEED_LONG_RANGE_CRUISE = 0, FSIM_SPEED_MAX_ENDURANCE }; /* A-GRA's SpeedOptimizationEnum (ABI 1.15) */
enum fsim_direction_reference { FSIM_DIRECTION_TRUE_NORTH = 0, FSIM_DIRECTION_MAGNETIC_NORTH }; /* A-GRA's MA_HeadingReferenceEnum (ABI 1.20) */
enum fsim_curve_z { FSIM_CURVE_Z_DOWN = 0, FSIM_CURVE_Z_ALTITUDE_OFFSET, FSIM_CURVE_Z_ABSOLUTE_ALTITUDE }; /* A-GRA's Z_ChoiceType (ABI 1.25) */
enum fsim_turn_type { FSIM_TURN_FLY_BY = 0, FSIM_TURN_FLY_OVER = 1,
                      /* ABI 1.27 (docs/flight-autonomy.md, 4.30): A-GRA's other turn points */
                      FSIM_TURN_CAPTURE_OUTBOUND_COURSE = 2, FSIM_TURN_START_TURN = 3, FSIM_TURN_END_TURN = 4 };
enum fsim_projection { FSIM_PROJECTION_GREAT_CIRCLE = 0, FSIM_PROJECTION_RHUMB };
enum fsim_end_behavior { FSIM_END_CONTINUE = 0, FSIM_END_LOITER }; /* after the last point: on along its leg; orbit it (a wing), hover over it (a rotorcraft after a route; after a curve it circles it) */
FSIM_API uint32_t fsim_mode_field_count(int mode); /* hsa 8, route 4, pattern 35, curve 20, must fly 10 (1.14: hsa 6, pattern 12; 1.15: hsa 7,
                                                      pattern 13; 1.20: hsa 8; 1.21: pattern 25; 1.22: pattern 29; 1.25: curve 20; 1.39: must fly);
                                                      0 for an unknown mode */
FSIM_API int fsim_vehicle_submit_mode(fsim_world* world, uint32_t id, int mode, const double* fields, uint32_t count,
                                      const fsim_command_options* options, fsim_command_result* result);

/* One waypoint of a route, and the segment that ends at it. fsim_waypoint_init
 * leaves every optional field out (fsim_hold()): the previous point's, the
 * first point's the aircraft's own now (a rotorcraft given no speed flies its
 * cruise speed over the ground). */
typedef struct fsim_waypoint {
    uint32_t struct_size;
    double latitude_rad, longitude_rad;
    double altitude_m, altitude_reference; /* fsim_altitude_reference; reached at the point along a straight profile */
    double speed, speed_reference;         /* fsim_speed_reference; flown on the segment to the point */
    double turn;                           /* fsim_turn_type; fsim_waypoint_init: fly-by */
    double max_bank_rad;                   /* the bank its fly-by turn is planned with; left out: 80 % of the aircraft's */
    double climb_rate_ms;                  /* climb or descend at it, then level; left out: along the segment's gradient */
    uint64_t id;                           /* the caller's, reported back in the progress */
    /* ABI 1.26 (docs/flight-autonomy.md, 4.29): A-GRA's end point as its schema gives it; read where the caller's
       struct_size has them */
    double altitude_min_m, altitude_max_m; /* its altitude block, in its reference: an altitude left out held within it */
    double kind;                           /* fsim_end_point_kind: left out, a turn point as `turn` says; a waypoint (no turn) flown over */
    double waypoint_type;                  /* a waypoint's fsim_waypoint_type (left out: nav only; given alone, a waypoint) */
    double frame;                          /* a point in this frame (fsim_world_create_frame's id): placed where it is, its latitude and
                                              longitude not read; a moving one's placed as it is flown */
    double frame_rotation, frame_offsets;  /* fsim_frame_rotation, fsim_frame_offsets of its offsets */
    double frame_x_m, frame_y_m, frame_z_m; /* its offsets (z down: given, its altitude the frame's there) */
    /* ABI 1.27 (4.30): a turn point's */
    double course_rad;                     /* the course at the point: a start's arc's (left out, the leg in's), an end's, a capture's */
    double turn_radius_m;                  /* its turn's radius: a fly-by's, a start's arc's (agreeing with the arc through the next point) */
    /* ABI 1.29 (4.32): the segment's performance */
    double speed_optimization;             /* fsim_speed_optimization: the segment at the tables' best now, its speed replaced (no tables:
                                              "not_implemented"); left out with the speed, the point before's */
    double climb_optimization;             /* fsim_climb_optimization: its climb or descent at a rate the aircraft chooses */
    double acceleration_ms2;               /* the speed change into the segment at this acceleration (above 0); left out, as the loops change it */
    /* ABI 1.30 (4.33): its required time of arrival - the window it is to arrive at the point in, the world's simulation
       seconds; either side left out, open (a begin alone: no earlier; an end alone: no later) */
    double arrival_begin_s;
    double arrival_end_s;
    /* ABI 1.32 (4.35): its required navigation performance - how far off its path the segment may be flown, m (above 0; left
       out, none); farther, its activity's constraints say so (32) */
    double rnp_m;
    /* ABI 1.33 (4.36): A-GRA's NextPathSegment - the waypoint flown after it, its index; -1, the route's end there; left
       out, the next in its path (after its path's last, the route's end), or - no paths given - the next as they are */
    double next;
    /* ABI 1.35 (4.38): A-GRA's CivilPathTerminator - the ARINC 424 leg type of the leg into it (fsim_path_terminator), its
       data (a course to fix's course, a radius to fix's arc) in an fsim_route_terminator beside it; left out, the route's
       own leg */
    double terminator;
} fsim_waypoint;
enum fsim_climb_optimization { FSIM_CLIMB_BEST_RATE = 0, FSIM_CLIMB_EXTENDED_RANGE }; /* A-GRA's ClimbOptimizationEnum (ABI 1.29) */
/* What a waypoint is for (A-GRA's WaypointTypeEnum; ABI 1.26): nav only and passive are flown; the end of a path at its
   path's last point (1.33: a route without paths, its last; elsewhere "invalid_waypoint"); the others not yet
   ("not_implemented", naming the point: their support table rows say when) */
enum fsim_waypoint_type {
    FSIM_WAYPOINT_NAV_ONLY = 0, FSIM_WAYPOINT_TAXI, FSIM_WAYPOINT_RUNWAY_START, FSIM_WAYPOINT_RUNWAY_THRESHOLD, FSIM_WAYPOINT_RUNWAY_LIMIT,
    FSIM_WAYPOINT_APPROACH, FSIM_WAYPOINT_APPROACH_INITIAL_POINT, FSIM_WAYPOINT_APPROACH_FINAL_POINT, FSIM_WAYPOINT_TAKEOFF,
    FSIM_WAYPOINT_TAKEOFF_INITIAL_POINT, FSIM_WAYPOINT_TAKEOFF_FINAL_POINT, FSIM_WAYPOINT_TOUCHDOWN, FSIM_WAYPOINT_PASSIVE,
    FSIM_WAYPOINT_HARD_DITCH, FSIM_WAYPOINT_END_OF_PATH
};
FSIM_API void fsim_waypoint_init(fsim_waypoint* waypoint);
/* A route (FSIM_MODE_ROUTE's fields, `count` of them) with at most 256
 * waypoints, `waypoints[0].struct_size` bytes apart (the caller's header's
 * size). A rejection names the waypoint at fault: the result's reserved is
 * its index + 1, and fsim_last_command_detail has it. */
FSIM_API int fsim_vehicle_submit_route(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_waypoint* waypoints,
                                       uint32_t waypoint_count, const fsim_command_options* options, fsim_command_result* result);
/* UPDATE of a route: its options (fsim_hold() keeps one) and new waypoints
 * (none: those it has), checked as a NEW's; flown afresh from its start. */
FSIM_API int fsim_activity_update_route(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                        const fsim_waypoint* waypoints, uint32_t waypoint_count, fsim_command_result* result);
/* As fsim_activity_update_route, declaring the caller's source (fsim_activity_update_as), and its controller (_by). */
FSIM_API int fsim_activity_update_route_as(fsim_world* world, fsim_activity_id activity, int source, const double* fields, uint32_t count,
                                           const fsim_waypoint* waypoints, uint32_t waypoint_count, fsim_command_result* result);
FSIM_API int fsim_activity_update_route_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                           uint32_t count, const fsim_waypoint* waypoints, uint32_t waypoint_count, fsim_command_result* result);

/* The loiter a route's loiter point flies (ABI 1.28; docs/flight-autonomy.md, 4.31; A-GRA's LoiterPoint): at waypoint
 * `point` (its kind FSIM_END_POINT_LOITER_POINT), the pattern in `fields` - FSIM_MODE_PATTERN's 35, in its order - and
 * `end_time_s`, when it ends (the world's time, as fsim_world_time reads it). Its place is its point's: the pattern's
 * latitude, longitude, altitude, altitude reference, frame and its offsets left out; its speed left out, the point's
 * segment's. It ends when its duration or laps are flown (on round to its exit point, if it has one) or at its end time,
 * the first; with none it is the route's end, and only a last point's (a route that does not repeat). Flown where the leg
 * meets it - a radius outside an orbit's circle, else at its point (a rotorcraft stops for a hover) - from where the
 * aircraft is; then on to the next point from where it ended. Refused as a point is, naming it (invalid_waypoint; a
 * wing's hover and an optimisation as a pattern's are); its limits named by the point, its field numbered from 100 -
 * past the waypoint's own, room left for them - the pattern's 100 + 0..34, the end time 135. fsim_route_loiter_init
 * leaves every field out (fsim_hold()). */
typedef struct fsim_route_loiter {
    uint32_t struct_size;
    uint32_t point;
    double fields[35];
    double end_time_s;
} fsim_route_loiter;
FSIM_API void fsim_route_loiter_init(fsim_route_loiter* loiter);
/* A route with its loiter points' loiters (at most 16), `loiters[0].struct_size` bytes apart; else as fsim_vehicle_submit_route. */
FSIM_API int fsim_vehicle_submit_route_loiters(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_waypoint* waypoints,
                                               uint32_t waypoint_count, const fsim_route_loiter* loiters, uint32_t loiter_count,
                                               const fsim_command_options* options, fsim_command_result* result);
/* UPDATE of a route with new waypoints and their loiters (none: those it has, and their loiters), declaring the caller's
 * source and controller as fsim_activity_update_route_by. */
FSIM_API int fsim_activity_update_route_loiters(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                                uint32_t count, const fsim_waypoint* waypoints, uint32_t waypoint_count,
                                                const fsim_route_loiter* loiters, uint32_t loiter_count, fsim_command_result* result);

/* A planned inertial state inside a route's segment (ABI 1.31; docs/flight-autonomy.md, 4.34; A-GRA's InertialState): on
 * the segment ending at waypoint `point`, `fields` in this order - its place (latitude, longitude, altitude, altitude
 * reference), when it is to be there (the world's time, as fsim_world_time reads it), a place in a frame instead
 * (fsim_frame_create's id, its offsets' rotation and form, x, y, z - z down: where the frame is at its time, a vehicle's
 * where its velocity now carries it), its position's uncertainty (m), its ground velocity (north, east), its velocity
 * through the air (north, east, down), in its frame (north, east, down), its acceleration (north, east, down), its
 * orientation (yaw, pitch, roll) and their rates - 29. The segment's first lap flies through its altitude (in its point's
 * reference: left out, its point's) and arrives at it at its time; the rest is kept and read back. Refused as a point is,
 * naming it: invalid_waypoint (past the route or out of order, not finite, no place, another altitude reference, a time
 * past or not after the one before, an altitude beside a climb rate or optimisation, a place off its leg by more than its
 * uncertainty - 50 m at least, or 1 % of the leg); performance_limit (a climb or descent steeper than the aircraft's, a
 * time it cannot make); not_implemented (at or after a loiter point, beside points in moving frames, a time without
 * performance tables). fsim_route_state_init leaves every field out (fsim_hold()). */
typedef struct fsim_route_state {
    uint32_t struct_size;
    uint32_t point;
    double fields[29];
} fsim_route_state;
FSIM_API void fsim_route_state_init(fsim_route_state* state);
/* A route with its loiter points' loiters and its planned states (at most 64, in order along it), `states[0].struct_size`
 * bytes apart; else as fsim_vehicle_submit_route_loiters. */
FSIM_API int fsim_vehicle_submit_route_states(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_waypoint* waypoints,
                                              uint32_t waypoint_count, const fsim_route_loiter* loiters, uint32_t loiter_count,
                                              const fsim_route_state* states, uint32_t state_count, const fsim_command_options* options,
                                              fsim_command_result* result);
/* UPDATE of a route with new waypoints and their loiters and states (none: those it has, and theirs), as
 * fsim_activity_update_route_loiters. */
FSIM_API int fsim_activity_update_route_states(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                               uint32_t count, const fsim_waypoint* waypoints, uint32_t waypoint_count,
                                               const fsim_route_loiter* loiters, uint32_t loiter_count, const fsim_route_state* states,
                                               uint32_t state_count, fsim_command_result* result);

/* What a route's path is for (A-GRA's MA_PathTypeEnum; ABI 1.33): a label, reported back. */
enum fsim_path_type {
    FSIM_PATH_PRIMARY = 0, FSIM_PATH_ALTERNATE, FSIM_PATH_LOSS_OF_COMM, FSIM_PATH_RETURN_TO_BASE, FSIM_PATH_SOFT_DITCH, FSIM_PATH_HARD_DITCH,
    FSIM_PATH_INGRESS, FSIM_PATH_EGRESS, FSIM_PATH_TAKEOFF, FSIM_PATH_LANDING, FSIM_PATH_EMERGENCY_LANDING, FSIM_PATH_TAXI, FSIM_PATH_AIRBORNE,
    FSIM_PATH_ARCING, FSIM_PATH_BREAKING, FSIM_PATH_ON_DEPARTURE_RADIAL, FSIM_PATH_INITIAL_APPROACH, FSIM_PATH_INTERMEDIATE_APPROACH,
    FSIM_PATH_FINAL_APPROACH, FSIM_PATH_BOLTER_WAVEOFF
};
/* One of a route's paths (ABI 1.33; docs/flight-autonomy.md, 4.36; A-GRA's MA_RoutePathType): `count` of its waypoints
 * from `first`, flown in order unless a point's `next` says otherwise - its last the route's end unless its `next` goes on
 * - with its id and `type` (fsim_path_type; NaN, primary). 16 a route at most, tiling its waypoints in order; the route
 * begins at its first path's first point (its start option picks another). Paths that do not tile them: "invalid_waypoint"
 * at the first point at fault; a type that is none, one id twice: at the path's first point; 17 or more: at point 0.
 * fsim_route_path_init: none of it given. */
typedef struct fsim_route_path {
    uint32_t struct_size;
    uint32_t first;
    uint32_t count;
    uint32_t reserved;
    uint64_t id;
    double type;
} fsim_route_path;
FSIM_API void fsim_route_path_init(fsim_route_path* path);
/* How a value is compared with the one given (A-GRA's EqualityExpressionEnum; ABI 1.34): the value under test on the left. */
enum fsim_comparison {
    FSIM_COMPARISON_GREATER = 0, FSIM_COMPARISON_GREATER_EQUAL, FSIM_COMPARISON_LESS, FSIM_COMPARISON_LESS_EQUAL, FSIM_COMPARISON_EQUAL,
    FSIM_COMPARISON_NOT_EQUAL
};
/* A route's conditional branch (ABI 1.34; docs/flight-autonomy.md, 4.37; A-GRA's ConditionalPathSegment): as the aircraft
 * comes to waypoint `point` - where its turn there begins, at a point flown over, or as a loiter point's loiter ends - the
 * point flown after it is the branch's next where every condition given holds; the route planned again from there.
 * `fields` in this order - next (its index; -1: the route's end there), altitude min and max (m) and their reference
 * (fsim_altitude_reference; NaN: above mean sea level), the time window's begin and end (fsim_world_time's clock), captures
 * (the times it has come to the point, this one too) and their comparison (fsim_comparison), operator input (1: only once
 * the operator has commanded it, fsim_activity_command_branch), endurance comparison (fsim_comparison), fuel (kg),
 * endurance (s), its end (fsim_world_time's clock), percent, contingency (fsim_contingency) - 15. 16 a route at most; those
 * at one point tried in their order, the first that holds taken. Refused naming its point: invalid_waypoint (at a point it
 * has not, a next that is none or its own point, a range or a window upside down, captures and their comparison apart, a
 * code that is none, a flight on from it round one point); not_implemented for a mission critical or lost comms
 * contingency (FA-16). What it has left and its contingency are its navigation report's (fsim_vehicle_navigation_report).
 * fsim_route_branch_init leaves every field out (fsim_hold()). */
typedef struct fsim_route_branch {
    uint32_t struct_size;
    uint32_t point;
    double fields[15];
} fsim_route_branch;
FSIM_API void fsim_route_branch_init(fsim_route_branch* branch);
/* A-GRA's civil path terminators (CivilPathTerminatorType; ABI 1.35; docs/flight-autonomy.md, 4.38): the ARINC 424 leg type
 * of the leg into a waypoint, in the schema's order, ARINC 424's code beside each. Flown: TF (the great circle from the point
 * before), IF and DF (straight to the point from where the aircraft is as the leg begins), CF (its course into the point:
 * fsim_route_terminator's course), RF (an arc round its centre from the point before: fsim_route_terminator's); CA, FA, VA
 * (the planned leg's course, a heading's held, until the point's altitude), CI, VI (until the next leg, a course to fix's
 * line), FC (as TF); FM, VM (on until a branch at the point that takes the operator's input is commanded; at a route's last
 * point, on along it), HA, HF, HM (the loiter point's hold ended at its altitude, once round, by the operator's branch).
 * Refused "invalid_waypoint", a leg its segment does not define (A-GRA 6.0a gives no navaid, nor a procedure turn's data): AF,
 * CD, CR, FD, PI, VD, VR. */
enum fsim_path_terminator {
    FSIM_PATH_TERMINATOR_ARC_TO_FIX = 0,                        /* AF */
    FSIM_PATH_TERMINATOR_COURSE_TO_ALTITUDE,                    /* CA */
    FSIM_PATH_TERMINATOR_COURSE_TO_DME_DISTANCE,                /* CD */
    FSIM_PATH_TERMINATOR_COURSE_TO_FIX,                         /* CF */
    FSIM_PATH_TERMINATOR_COURSE_TO_INTERCEPT,                   /* CI */
    FSIM_PATH_TERMINATOR_COURSE_TO_RADIAL,                      /* CR */
    FSIM_PATH_TERMINATOR_DIRECT_TO_FIX,                         /* DF */
    FSIM_PATH_TERMINATOR_TRACK_TO_ALTITUDE,                     /* FA */
    FSIM_PATH_TERMINATOR_TRACK_FROM_FIX_TO_DISTANCE_ALONG_TRACK, /* FC */
    FSIM_PATH_TERMINATOR_TRACK_FROM_FIX_TO_DME_DISTANCE,        /* FD */
    FSIM_PATH_TERMINATOR_FIX_TO_MANUAL_TERMINATION,             /* FM */
    FSIM_PATH_TERMINATOR_HOLDING_WITH_ALTITUDE_TERMINATION,     /* HA */
    FSIM_PATH_TERMINATOR_HOLDING_WITH_FIX_TERMINATION,          /* HF */
    FSIM_PATH_TERMINATOR_HOLDING_WITH_MANUAL_TERMINATION,       /* HM */
    FSIM_PATH_TERMINATOR_INITIAL_FIX,                           /* IF */
    FSIM_PATH_TERMINATOR_PROCEDURE_TURN_TO_INTERCEPT,           /* PI */
    FSIM_PATH_TERMINATOR_RADIUS_TO_FIX,                         /* RF */
    FSIM_PATH_TERMINATOR_TRACK_TO_FIX,                          /* TF */
    FSIM_PATH_TERMINATOR_HEADING_TO_ALTITUDE,                   /* VA */
    FSIM_PATH_TERMINATOR_HEADING_TO_DME_DISTANCE_TERMINATION,   /* VD */
    FSIM_PATH_TERMINATOR_HEADING_TO_INTERCEPT,                  /* VI */
    FSIM_PATH_TERMINATOR_HEADING_TO_MANUAL,                     /* VM */
    FSIM_PATH_TERMINATOR_HEADING_TO_RADIAL_TERMINATION          /* VR */
};
/* A civil path terminator's data (ABI 1.35; docs/flight-autonomy.md, 4.38; A-GRA's CF_CourseToFixType, RF_RadiusToFixType):
 * the leg into waypoint `point`, whose `terminator` names it. `fields` in this order - CF's course (from true north); RF's
 * centre latitude and longitude, radius (m), courses in and out (the arc's at its start and end), initial point's latitude
 * and longitude (the point before's), end point's latitude and longitude (its point's), arc distance (m), direct distance
 * (m), way round (1 right, 0 left) - 13. 64 a route at most, one a point. A course to fix needs its course; a radius to fix
 * its centre and its way round, the rest checked against its arc where given (a metre or half a percent; a degree). Refused
 * "invalid_waypoint" naming the point for data that is none, or not its leg's. fsim_route_terminator_init leaves every
 * field out (fsim_hold()). */
typedef struct fsim_route_terminator {
    uint32_t struct_size;
    uint32_t point;
    double fields[13];
} fsim_route_terminator;
FSIM_API void fsim_route_terminator_init(fsim_route_terminator* terminator);
/* What goes beside a route's waypoints (ABI 1.33): its loiter points' loiters, its planned states and its paths - and its
 * conditional branches (ABI 1.34) and civil path terminators' data (ABI 1.35), where the caller's struct_size has them -
 * each array's struct_size apart. fsim_route_extras_init: none. */
typedef struct fsim_route_extras {
    uint32_t struct_size;
    uint32_t loiter_count;
    const fsim_route_loiter* loiters;
    uint32_t state_count;
    uint32_t path_count;
    const fsim_route_state* states;
    const fsim_route_path* paths;
    uint32_t branch_count;
    const fsim_route_branch* branches;
    uint32_t terminator_count;                  /* ABI 1.35: its civil path terminators' data */
    const fsim_route_terminator* terminators;
} fsim_route_extras;
FSIM_API void fsim_route_extras_init(fsim_route_extras* extras);
/* A route with what goes beside its waypoints (`extras` NULL: none); else as fsim_vehicle_submit_route. */
FSIM_API int fsim_vehicle_submit_route_extras(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_waypoint* waypoints,
                                              uint32_t waypoint_count, const fsim_route_extras* extras, const fsim_command_options* options,
                                              fsim_command_result* result);
/* UPDATE of a route with new waypoints and what goes beside them (none: those it has, and theirs), declaring the caller's
 * source and controller as fsim_activity_update_route_by. */
FSIM_API int fsim_activity_update_route_extras(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                               uint32_t count, const fsim_waypoint* waypoints, uint32_t waypoint_count, const fsim_route_extras* extras,
                                               fsim_command_result* result);

/* One segment of a curve: a quintic Bezier by its six control points (weights
 * 1, the clamped knots), metres north, east and down from the curve's
 * reference. fsim_bezier_segment_init sets struct_size and zeroes the rest. */
typedef struct fsim_bezier_segment {
    uint32_t struct_size;
    double north[6], east[6], down[6];
} fsim_bezier_segment;
FSIM_API void fsim_bezier_segment_init(fsim_bezier_segment* segment);
/* A curve (FSIM_MODE_CURVE's fields, `count` of them) with 1 to 10 segments,
 * `segments[0].struct_size` bytes apart, each starting within a metre of where
 * the one before ends. A rejection names the segment at fault in the result's
 * reserved (its index + 1) and fsim_last_command_detail; one too tight for
 * the aircraft names its section too (the detail's from and to, 0 to 1). */
FSIM_API int fsim_vehicle_submit_curve(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_bezier_segment* segments,
                                       uint32_t segment_count, const fsim_command_options* options, fsim_command_result* result);
/* UPDATE of a curve: its options (fsim_hold() keeps one) and segments - with
 * the append field 1 after its end, from the same reference; else a new curve,
 * flown afresh (none: those it has, flown on). */
FSIM_API int fsim_activity_update_curve(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                        const fsim_bezier_segment* segments, uint32_t segment_count, fsim_command_result* result);
/* As fsim_activity_update_curve, declaring the caller's source (fsim_activity_update_as), and its controller (_by). */
FSIM_API int fsim_activity_update_curve_as(fsim_world* world, fsim_activity_id activity, int source, const double* fields, uint32_t count,
                                           const fsim_bezier_segment* segments, uint32_t segment_count, fsim_command_result* result);
FSIM_API int fsim_activity_update_curve_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                           uint32_t count, const fsim_bezier_segment* segments, uint32_t segment_count, fsim_command_result* result);
/* ABI 1.24 (docs/flight-autonomy.md, 4.26): a curve's segment as A-GRA's schema gives it (MA_NURBS_PointType) - a
 * clamped rational B-spline: `points` control points (4 to 10), metres north, east and down from the curve's
 * reference, each with its `weight` (above 0), and `knots` knots (4 to 14, at least points + 2: its degree is knots -
 * points - 1), from 0 and never decreasing, the first and the last each as many times as the degree and once more.
 * `curvature` (1/m; NaN: not said) is the most it turns, checked against it; `first_index` and `last_index` (NaN: not
 * said) its first and last control points: 0 and points - 1. A Bezier segment is one: six points, weights 1, knots
 * [0 x6, 1 x6]. fsim_nurbs_segment_init sets struct_size, every weight 1, the last three NaN and the rest 0. */
typedef struct fsim_nurbs_segment {
    uint32_t struct_size;
    uint32_t points, knots;
    uint32_t reserved;
    double north[10], east[10], down[10], weight[10];
    double knot[14];
    double curvature, first_index, last_index;
} fsim_nurbs_segment;
FSIM_API void fsim_nurbs_segment_init(fsim_nurbs_segment* segment);
/* A curve with its segments as A-GRA's schema gives them (1 to 10, `segments[0].struct_size` bytes apart), as
 * fsim_vehicle_submit_curve takes Bezier segments: a malformed one (not a clamped curve, or turning tighter than its
 * curvature says) refused invalid_curve naming it, and the section. */
FSIM_API int fsim_vehicle_submit_nurbs(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_nurbs_segment* segments,
                                       uint32_t segment_count, const fsim_command_options* options, fsim_command_result* result);
/* UPDATE of a curve with segments as A-GRA's schema gives them, as fsim_activity_update_curve (_as, _by). */
FSIM_API int fsim_activity_update_nurbs(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                        const fsim_nurbs_segment* segments, uint32_t segment_count, fsim_command_result* result);
FSIM_API int fsim_activity_update_nurbs_as(fsim_world* world, fsim_activity_id activity, int source, const double* fields, uint32_t count,
                                           const fsim_nurbs_segment* segments, uint32_t segment_count, fsim_command_result* result);
FSIM_API int fsim_activity_update_nurbs_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                           uint32_t count, const fsim_nurbs_segment* segments, uint32_t segment_count, fsim_command_result* result);

/* What a vehicle can do, as its guidance plans with it (docs/vehicle-interface.md,
 * 7.1; A-GRA's performance profile): NaN where the aircraft's profile and loops
 * say nothing. Computed afresh when its loops change (revision counts it).
 * fsim_performance_init sets struct_size; the call fills up to it. */
typedef struct fsim_performance {
    uint32_t struct_size;
    uint32_t revision;       /* counts recomputations */
    int32_t hovers;          /* 1: a rotorcraft (holds a point, flies any direction over the ground) */
    double min_cas_ms, max_cas_ms, max_mach; /* calibrated: the envelope's (the least else 1.2 times the stall speed) */
    double max_tas_ms;       /* the fastest it flies (level, at full power) */
    double cruise_tas_ms;    /* what a mode flies given no speed */
    double max_ground_speed_ms; /* a rotorcraft's fastest over the ground */
    double ceiling_m;
    double max_bank_rad, min_pitch_rad, max_pitch_rad, max_roll_rate_rad_s, min_load_factor, max_load_factor;
    double max_tilt_rad, max_acceleration_ms2, max_deceleration_ms2; /* a rotorcraft's */
    double max_climb_ms, max_descent_ms;                             /* what guidance asks for */
    double altitude_gain_per_s, heading_gain, heading_reference_tas_ms, bank_rate_rad_s, velocity_bandwidth_rad_s; /* how fast it answers */
} fsim_performance;
FSIM_API void fsim_performance_init(fsim_performance* performance);
FSIM_API int fsim_vehicle_performance(fsim_world* world, uint32_t id, fsim_performance* out);

/* Authority: grants over the priorities (docs/vehicle-interface.md, 6). Open,
 * the default, is as ever; Granted: a policy's NEW (and the fsim_vehicle_command_*
 * calls) needs a grant for its capability - else "not_granted" - and what the
 * policy flies without one ends canceled(not_granted). The platform's own
 * sources (autopilot, override) never need one. Capabilities by id. */
enum fsim_control_mode { FSIM_CONTROL_OPEN = 0, FSIM_CONTROL_GRANTED };
FSIM_API int fsim_vehicle_set_control_mode(fsim_world* world, uint32_t id, int mode);
FSIM_API int fsim_vehicle_control_mode(const fsim_world* world, uint32_t id, int32_t* mode);
/* A policy asks for control of a capability: *reason 0 if granted, else why not
 * (fsim_reason_name: "not_allowed", or why it is unavailable). */
FSIM_API int fsim_vehicle_request_control(fsim_world* world, uint32_t id, const char* capability, int32_t* reason);
/* The policy lets go: its grant ends, and its live activities of the capability end canceled(released). */
FSIM_API int fsim_vehicle_release_control(fsim_world* world, uint32_t id, const char* capability);
/* The platform takes it back: the grant ends, and the policy's live activities of
 * the capability end canceled with `reason`: 0 or "revoked", "collision_avoidance"
 * or "restricted" - another is refused (FSIM_INVALID_ARGUMENT) and nothing changes. */
FSIM_API int fsim_vehicle_revoke_control(fsim_world* world, uint32_t id, const char* capability, int reason);
/* Whether the policy may request the capability (all may, by default); a grant for one no longer allowed is revoked. */
FSIM_API int fsim_vehicle_set_allowed(fsim_world* world, uint32_t id, const char* capability, int allowed);
FSIM_API int fsim_vehicle_control_status(const fsim_world* world, uint32_t id, const char* capability, int32_t* allowed, int32_t* granted);
/* Named controllers (ABI 1.12; docs/flight-autonomy.md, 4.12; A-GRA's several MA services): the calls above are
 * controller 0's, the default policy's. A grant is one controller's: a request while another holds it is refused
 * "authority_held", a release of another's "not_granted" (*reason) - and nothing changes - and under
 * FSIM_CONTROL_GRANTED only the holder commands the capability (fsim_command_options.controller). */
FSIM_API int fsim_vehicle_request_control_by(fsim_world* world, uint32_t id, const char* capability, uint32_t controller, int32_t* reason);
FSIM_API int fsim_vehicle_release_control_by(fsim_world* world, uint32_t id, const char* capability, uint32_t controller, int32_t* reason);
/* Whose grant a capability's is: *granted 1 and *holder the controller, or *granted 0 (and *holder 0). */
FSIM_API int fsim_vehicle_control_holder(const fsim_world* world, uint32_t id, const char* capability, int32_t* granted, uint32_t* holder);
/* The platform restricts a capability (fsim_availability; FSIM_AVAILABLE lifts it):
 * a policy's NEW for it, and a request, are refused with `reason` - 0 or
 * "restricted", "collision_avoidance" or "unavailable"; another is refused
 * (FSIM_INVALID_ARGUMENT) - and what flies goes on. fsim_vehicle_capability_status
 * reports it. */
FSIM_API int fsim_vehicle_set_availability(fsim_world* world, uint32_t id, const char* capability, int availability, int reason);
/* Counts every change to the grants, what is allowed, the control mode, availability and the performance: poll it. */
FSIM_API int fsim_vehicle_control_revision(fsim_world* world, uint32_t id, uint32_t* revision);

/* A-GRA's flight capability types (MA_FlightCapabilityEnum). */
enum fsim_flight_mode {
    FSIM_FLIGHT_MODE_NONE = 0,
    FSIM_FLIGHT_MODE_HSA_CSA,
    FSIM_FLIGHT_MODE_WAYPOINT_FOLLOWING,
    FSIM_FLIGHT_MODE_CURVE_FOLLOWING,
    FSIM_FLIGHT_MODE_LOITER,
    FSIM_FLIGHT_MODE_FORMATION,
    FSIM_FLIGHT_MODE_MUST_FLY,
    FSIM_FLIGHT_MODE_ALTITUDE_STACKED_MARSHALL,
    FSIM_FLIGHT_MODE_LAUNCH,
    FSIM_FLIGHT_MODE_RECOVERY,
    FSIM_FLIGHT_MODE_ROUTE_INTERCEPT
};
/* The A-GRA type a capability is (fsim_flight_mode); -1 if the vehicle has no such capability. */
FSIM_API int fsim_vehicle_capability_flight_mode(fsim_world* world, uint32_t id, uint32_t capability);
FSIM_API const char* fsim_flight_mode_name(int mode); /* "hsa_csa", "waypoint_following", "loiter", ... */

/* ---------------------------------------------------------------------------
 * Support and availability (ABI 1.7; docs/flight-autonomy.md, 4): whether the
 * aircraft can do something at all - supported, partial, not implemented yet
 * (with the stage that builds it) or not supported (a physical exception, with
 * the rules and the aircraft's evidence) - apart from whether it can be
 * commanded now. A NEW for a feature the vehicle does not offer is refused
 * "not_supported" or "not_implemented"; an id no platform defines
 * "unknown_capability". A policy's NEW of the platform's airborne guidance on
 * the ground is refused "on_ground" (the platform's own sources never are).
 * ------------------------------------------------------------------------- */

/* A capability's availability now, as a policy is answered: every reason that
 * holds (a bit per reason code), the first as `reason`; what it is about and
 * when it is expected back; and how many parameters a placard narrows now
 * (fsim_vehicle_capability_limits). One the vehicle does not offer is
 * FSIM_UNAVAILABLE with "not_supported", "not_implemented" or "unknown_capability". */
typedef struct fsim_capability_status {
    uint32_t struct_size;
    int32_t availability;    /* fsim_availability */
    int32_t reason;          /* fsim_reason_name(): the first of `reasons` */
    uint32_t range_count;    /* parameters narrower now than their advertised range */
    uint64_t reasons;        /* every reason that holds: bit (1 << code) */
    uint64_t associated;     /* an id the reason is about (the vehicle avoided); 0 none */
    double next_available_s; /* simulation time it is expected back; NaN not known */
    const char* description; /* the reason in words; static, "" for none */
} fsim_capability_status;
FSIM_API void fsim_capability_status_init(fsim_capability_status* status);
FSIM_API int fsim_vehicle_capability_status_info(const fsim_world* world, uint32_t id, const char* capability, fsim_capability_status* out);
/* The parameters narrower now (a placard: the gear down on the ground, the flaps in above their speed): up to
 * `capacity` of them, `*count` how many there are. A NEW outside one is refused with the status's reason. */
typedef struct fsim_parameter_limit {
    uint32_t parameter; /* its index: fsim_vehicle_capability_parameter */
    uint32_t reserved;
    double min, max;
} fsim_parameter_limit;
FSIM_API int fsim_vehicle_capability_limits(const fsim_world* world, uint32_t id, const char* capability, fsim_parameter_limit* out,
                                            uint32_t capacity, uint32_t* count);
FSIM_API const char* fsim_availability_name(int availability); /* "available", "temporarily_unavailable", ... */
FSIM_API const char* fsim_reason_description(int reason);       /* the reason in words */

/* How a capability is controlled (A-GRA's AcceptedInterface): bits of fsim_accepted_interface; -1 if the vehicle has no
 * such capability. And the A-GRA capability a platform behaviour is superseded by ("fsim.guidance.hold":
 * "fsim.guidance.hsa"); "" for the others, NULL for no such capability. */
enum fsim_accepted_interface { FSIM_ACCEPTS_CAPABILITY_COMMAND = 1, FSIM_ACCEPTS_ACTIVITY_COMMAND = 2, FSIM_ACCEPTS_TASK_COMMAND = 4,
                               FSIM_ACCEPTS_AUTO_MDF = 8 };
FSIM_API int fsim_vehicle_capability_accepted(fsim_world* world, uint32_t id, uint32_t capability);
FSIM_API const char* fsim_vehicle_capability_superseded(fsim_world* world, uint32_t id, uint32_t capability);

/* One public feature's support on a vehicle (fsim_support_feature(i) lists every identifier). Strings are the
 * world's, valid while the vehicle is. */
enum fsim_support_state { FSIM_SUPPORTED = 0, FSIM_PARTIAL, FSIM_NOT_IMPLEMENTED, FSIM_NOT_SUPPORTED };
typedef struct fsim_support_info {
    uint32_t struct_size;
    int32_t support;        /* fsim_support_state: fsim_support_name() */
    uint32_t rules;         /* not supported: the rules that exclude it, else those that govern it; bit (1 << rule), fsim_rule_name() */
    uint32_t stage;         /* partial, not implemented: the stage that builds it (FA-n: n); 0 none */
    const char* feature;    /* its identifier */
    const char* capability; /* the capability that carries it; "" for the command interface's own */
    const char* missing;    /* partial: what is not built yet */
    const char* evidence;   /* not supported: the aircraft's declarations the rules rest on, with their sources */
} fsim_support_info;
FSIM_API void fsim_support_info_init(fsim_support_info* info);
/* By identifier ("fsim.guidance.hover", "fsim.guidance.hsa/direction/magnetic_north") or a behaviour's id;
 * FSIM_INVALID_ARGUMENT for an unknown vehicle or feature. */
FSIM_API int fsim_vehicle_support(const fsim_world* world, uint32_t id, const char* feature, fsim_support_info* out);
FSIM_API uint32_t fsim_support_feature_count(void);
FSIM_API const char* fsim_support_feature(uint32_t index); /* NULL past the end */
FSIM_API const char* fsim_support_name(int support);       /* "supported", "partial", "not_implemented", "not_supported" */
FSIM_API const char* fsim_rule_name(int rule);             /* "R1" ... "R13" */
FSIM_API const char* fsim_rule_description(int rule);
/* fsim_vehicle_set_availability, with the id the restriction is about and when it is expected back (NaN: not known). */
FSIM_API int fsim_vehicle_set_availability_ex(fsim_world* world, uint32_t id, const char* capability, int availability, int reason,
                                              uint64_t associated, double next_available_s);

/* ---------------------------------------------------------------------------
 * The command envelope (ABI 1.8; docs/flight-autonomy.md, 4.8): a command's
 * id and the requirements it comes from (fsim_command_options), kept with its
 * activity; a validation that flies nothing; every reason a command cannot be
 * flown as asked, and every value it is flown with other than asked; several
 * NEWs in one call, each answered on its own.
 * ------------------------------------------------------------------------- */

/* One reason the last command cannot be flown as asked (A-GRA's ValidationResult): the answer's reason is the first. */
typedef struct fsim_command_finding {
    uint32_t struct_size;
    int32_t reason;          /* fsim_reason_name() */
    int32_t index;           /* the field, route point or curve segment; -1 none */
    int32_t constraint;      /* the performance limit its value breaks: fsim_constraint_name() */
    double from, to;         /* a curve segment's section, its parameter 0..1; NaN otherwise */
    uint64_t associated;     /* an id it is about; 0 none */
    const char* description; /* in words */
} fsim_command_finding;
FSIM_API void fsim_command_finding_init(fsim_command_finding* finding);
FSIM_API int fsim_last_command_finding(const fsim_world* world, uint32_t index, fsim_command_finding* out);

/* One value the last command is flown with other than asked: held to what the aircraft can do. */
typedef struct fsim_command_adjustment {
    uint32_t struct_size;
    int32_t index;       /* the command's field, a route point or a curve segment */
    int32_t field;       /* a route point's field (fsim_waypoint's order from latitude_rad = 0; its loiter's from 100,
                            ABI 1.28: fsim_route_loiter's fields from 100, its end time 135); -1 none */
    int32_t constraint;  /* the limit it was held to: fsim_constraint_name() */
    double requested;    /* NaN where it is not one number (a fly-by turn flown smaller) */
    double adjusted;
} fsim_command_adjustment;
FSIM_API void fsim_command_adjustment_init(fsim_command_adjustment* adjustment);
FSIM_API int fsim_last_command_adjustment(const fsim_world* world, uint32_t index, fsim_command_adjustment* out);

/* The command an activity came from: its id, the requirements it traces to, whether it takes activity commands;
 * ABI 1.9: how it is arbitrated and scheduled, and whether it waits to start. */
typedef struct fsim_activity_envelope {
    uint32_t struct_size;
    int32_t interactive;
    uint64_t command_id;
    fsim_requirement trace[FSIM_MAX_REQUIREMENTS];
    int32_t waiting;           /* fsim_activity_wait (ABI 1.9) */
    int32_t basis;             /* fsim_activity_basis */
    uint16_t rank_priority, rank_precedence;
    uint32_t precedence;       /* its capability's precedence it is arbitrated by (its command's override, else the capability's) */
    fsim_activity_id waiting_for; /* queued: an activity on its axes it may not interrupt */
    int32_t interrupt;
    int32_t criticality;       /* fsim_time_criticality */
    double start_not_before, start_not_after, end_not_before, end_not_after; /* its window; NaN: none */
    uint64_t suggestion;       /* ABI 1.11: failed as it would start - the task the platform suggests in its place; 0 none */
    uint32_t run, runs;        /* a task's repetition: the run flying, of how many; 0, 0 none */
    uint32_t controller;       /* ABI 1.12: its command's controller (fsim_command_options.controller) */
    uint32_t reserved;
} fsim_activity_envelope;
FSIM_API void fsim_activity_envelope_init(fsim_activity_envelope* envelope);
FSIM_API int fsim_activity_get_envelope(const fsim_world* world, fsim_activity_id activity, fsim_activity_envelope* out);
FSIM_API const char* fsim_requirement_kind_name(int kind); /* "effect", "action", "task", "command"; "none" */
FSIM_API const char* fsim_activity_wait_name(int wait);    /* "none", "scheduled", "queued" */
FSIM_API const char* fsim_activity_basis_name(int basis);  /* "actual", "sensed", "predicted", "planned" */
FSIM_API const char* fsim_time_criticality_name(int criticality); /* "none", "start", "end", "start_and_end" */
/* A capability's precedence (ABI 1.9; lower first, 0 until set): the platform's setting, by which two activities of one
 * source contest axes before their ranks. What waits may start at once. */
FSIM_API int fsim_vehicle_set_capability_precedence(fsim_world* world, uint32_t id, const char* capability, uint32_t precedence);
FSIM_API int fsim_vehicle_capability_precedence(const fsim_world* world, uint32_t id, const char* capability, uint32_t* precedence);

/* Flight tasks (ABI 1.11; docs/flight-autonomy.md, 4.11): a command kept by id and flown on a task command. */
#define FSIM_SUGGESTED_TASK (1ull << 63) /* set in the ids of the platform's suggestions */
enum fsim_task_state { FSIM_TASK_AWAITING_EXECUTION = 0, FSIM_TASK_EXECUTION_PENDING, FSIM_TASK_EXECUTING, FSIM_TASK_COMPLETED, FSIM_TASK_DROPPED,
                       FSIM_TASK_FAILED, FSIM_TASK_CANCELED };
typedef struct fsim_task_status {
    uint32_t struct_size;
    int32_t state;             /* fsim_task_state */
    int32_t reason;            /* why it failed, was dropped or canceled: its activity's end */
    int32_t suggested;         /* 1: the platform's suggestion */
    uint64_t task_id;
    fsim_activity_id activity; /* its activity (every run's); 0 before it is commanded */
    uint32_t run, runs;        /* the run flying or flown last, of how many */
    double percent;            /* of the whole task */
    double start_time, end_time; /* when it was commanded; when its activity ended (NaN until) */
    uint64_t command_id;       /* its task command's */
} fsim_task_status;
FSIM_API void fsim_task_status_init(fsim_task_status* status);
FSIM_API const char* fsim_task_state_name(int state); /* "awaiting_execution", "execution_pending", "executing", "completed", "dropped", ... */

/* One command of a batch NEW: which call it would be, and that call's arguments. */
enum fsim_batch_kind { FSIM_BATCH_LEVEL = 0, FSIM_BATCH_BEHAVIOR, FSIM_BATCH_SUPPORT, FSIM_BATCH_MODE, FSIM_BATCH_ROUTE, FSIM_BATCH_CURVE,
                       FSIM_BATCH_NURBS /* ABI 1.24: a curve with segments as A-GRA's schema gives them */ };
typedef struct fsim_batch_command {
    uint32_t struct_size;
    int32_t kind;                           /* fsim_batch_kind */
    int32_t code;                           /* the level (FSIM_BATCH_LEVEL), fsim_support, or fsim_mode (FSIM_BATCH_MODE: hsa, pattern) */
    uint32_t count;                         /* fields: as fsim_vehicle_submit, _support, _mode, _route or _curve takes them */
    const double* fields;
    const fsim_behavior_command* behavior;  /* FSIM_BATCH_BEHAVIOR */
    const fsim_waypoint* waypoints;         /* FSIM_BATCH_ROUTE, waypoints[0].struct_size bytes apart */
    uint32_t waypoint_count;
    uint32_t segment_count;
    const fsim_bezier_segment* segments;    /* FSIM_BATCH_CURVE, segments[0].struct_size bytes apart */
    const fsim_command_options* options;    /* NULL: fsim_command_options_init's */
    const fsim_nurbs_segment* nurbs;        /* FSIM_BATCH_NURBS (ABI 1.24): segment_count of them, nurbs[0].struct_size bytes apart */
    const fsim_route_loiter* loiters;       /* FSIM_BATCH_ROUTE's loiters (ABI 1.28), loiters[0].struct_size bytes apart */
    uint32_t loiter_count;
    uint32_t state_count;                   /* FSIM_BATCH_ROUTE's planned states (ABI 1.31), states[0].struct_size bytes apart */
    const fsim_route_state* states;
    const fsim_route_path* paths;           /* FSIM_BATCH_ROUTE's paths (ABI 1.33), paths[0].struct_size bytes apart */
    uint32_t path_count;
    const fsim_route_branch* branches;      /* FSIM_BATCH_ROUTE's conditional branches (ABI 1.34), branches[0].struct_size bytes apart */
    uint32_t branch_count;
    const fsim_route_terminator* terminators; /* FSIM_BATCH_ROUTE's civil path terminators' data (ABI 1.35), struct_size bytes apart */
    uint32_t terminator_count;
    const struct fsim_op_zone* zone;        /* ABI 1.40: a must fly's zone given with it (FSIM_BATCH_MODE, FSIM_MODE_MUST_FLY); NULL none */
} fsim_batch_command;
/* Several NEWs at once (A-GRA's several command instances in one message), `batch[0].struct_size` bytes apart, made in
 * order at this simulation time: `results[i]` answers `batch[i]`, and `details[i]` (may be NULL; `details[0].struct_size`
 * bytes apart) has what fsim_last_command_detail would have right after it - its counts included, though only the last
 * one's findings and adjustments stay to be read (fsim_last_command_finding). A malformed item (a wrong field count, an
 * unknown kind): FSIM_INVALID_ARGUMENT, and none is made. */
FSIM_API int fsim_vehicle_submit_batch(fsim_world* world, uint32_t id, const fsim_batch_command* batch, uint32_t count, fsim_command_result* results,
                                       fsim_command_detail* details);

/* Keep a task (ABI 1.11): `command` names the call its NEW would be, as a batch item does (its options are not kept: a
 * task command gives them); `attempts` runs (0 as 1), each `interval_s` after the one before completes (NaN: at once).
 * FSIM_OK and `*reason` the answer: 0, or invalid_parameter (id 0, one with FSIM_SUGGESTED_TASK, runs of what never
 * completes), task_active (it flies), why the vehicle cannot command it. FSIM_INVALID_ARGUMENT for a malformed command. */
FSIM_API int fsim_vehicle_store_task(fsim_world* world, uint32_t id, uint64_t task_id, const fsim_batch_command* command, uint32_t attempts,
                                     double interval_s, int32_t* reason);
/* Fly a task: its command's NEW with `options` (NULL: fsim_command_options_init's), the task among the requirements it
 * traces to; answered as the NEW (fsim_last_command_detail), unknown_task, task_active. */
FSIM_API int fsim_vehicle_command_task(fsim_world* world, uint32_t id, uint64_t task_id, const fsim_command_options* options, fsim_command_result* result);
/* Its live activity canceled, declaring `source` and `controller` (fsim_activity_cancel_by); one never commanded will not be. */
FSIM_API int fsim_vehicle_cancel_task(fsim_world* world, uint32_t id, uint64_t task_id, int source, uint32_t controller, fsim_command_result* result);
FSIM_API int fsim_vehicle_remove_task(fsim_world* world, uint32_t id, uint64_t task_id, int32_t* reason);
/* A task's status: FSIM_INVALID_ARGUMENT for one not kept. */
FSIM_API int fsim_vehicle_task_status(fsim_world* world, uint32_t id, uint64_t task_id, fsim_task_status* out);
/* Every task kept, the caller's and the platform's, in the order they were made. */
FSIM_API uint32_t fsim_vehicle_task_count(fsim_world* world, uint32_t id);
FSIM_API int fsim_vehicle_task_at(fsim_world* world, uint32_t id, uint32_t index, fsim_task_status* out);

/* Route plans (ABI 1.36; docs/flight-autonomy.md, 4.39): A-GRA's route plans, kept by id and version and taken through the
 * plan activation states (VI 1.2.5) before they fly, with their planning metadata (WPT-23), which nothing flies by. */
enum fsim_plan_command { FSIM_PLAN_PREPARE_FOR_UPLOAD = 0, FSIM_PLAN_UPLOAD, FSIM_PLAN_PREPARE_FOR_ACTIVATION, FSIM_PLAN_ACTIVATE,
                         FSIM_PLAN_DEACTIVATE };
enum fsim_plan_state { FSIM_PLAN_INACTIVE = 0, FSIM_PLAN_READY_FOR_UPLOAD, FSIM_PLAN_PREPARATION_FOR_UPLOAD_FAILED, FSIM_PLAN_UPLOAD_FAILED,
                       FSIM_PLAN_UPLOADED, FSIM_PLAN_PREPARATION_FOR_ACTIVATION_FAILED, FSIM_PLAN_READY_FOR_ACTIVATION,
                       FSIM_PLAN_ACTIVATION_FAILED, FSIM_PLAN_ACTIVATED, FSIM_PLAN_DEACTIVATED };
enum fsim_plan_execution { FSIM_PLAN_EXECUTION_NONE = 0, FSIM_PLAN_EXECUTION_PENDING, FSIM_PLAN_EXECUTION_EXECUTING, FSIM_PLAN_EXECUTION_COMPLETE,
                           FSIM_PLAN_EXECUTION_SUPERSEDED, FSIM_PLAN_EXECUTION_CANCELED, FSIM_PLAN_EXECUTION_FAILED };
enum fsim_point_source { FSIM_POINT_AUTO_ROUTED = 0, FSIM_POINT_OPERATOR_DEFINED };
/* A route plan's point's planning metadata (A-GRA's MA_PathSegmentType's Source, Locked, Modified, Remarks and
 * Fix_Identifier). Its texts printable ASCII, no longer than A-GRA's: 32, 1,024, 256 and 256 characters; NULL as "". */
typedef struct fsim_point_metadata {
    uint32_t struct_size;
    uint32_t point;           /* the waypoint's index */
    int32_t source;           /* fsim_point_source */
    int32_t locked;           /* 1: not to be modified (operator-driven) */
    int32_t modified;         /* 1: modified in a modified route plan */
    const char* remarks_name; /* its Remarks' DisplayName */
    const char* remarks;      /* its Remarks' Detail */
    const char* fix_key;      /* its Fix_Identifier's Key */
    const char* fix_system;   /* its Fix_Identifier's SystemName */
} fsim_point_metadata;
FSIM_API void fsim_point_metadata_init(fsim_point_metadata* metadata);
/* A route plan's path's planning metadata (A-GRA's MA_RoutePathType.InitialConditions): the aircraft's state as planned or
 * assessed where the path begins. */
typedef struct fsim_path_metadata {
    uint32_t struct_size;
    uint32_t path;                 /* the path's index among the route's paths; 0 without paths */
    fsim_route_state initial;      /* its InertialState, as a planned state's fields (its point not used) */
    double endurance_s, fuel_kg;   /* its Endurance's Duration and Fuel (NaN: left out) */
    double gross_weight_kg;        /* its GrossWeight */
    uint64_t transition_plan;      /* its TransitionRoute: a route plan's id; 0 none */
    /* ABI 1.37, where the caller's struct_size has them (4.40): its airfield and runway (A-GRA's AirfieldID, RunwayID) - a
       takeoff's or a landing's path, on FA's own plan, names ones the vehicle keeps; 0 none */
    uint64_t airfield, runway;
} fsim_path_metadata;
FSIM_API void fsim_path_metadata_init(fsim_path_metadata* metadata);
/* A route plan (A-GRA's MA_RoutePlanMT): its route a FSIM_BATCH_ROUTE item - its four options, its waypoints and their
 * extras (its command options not kept) - and its planning metadata. */
typedef struct fsim_route_plan {
    uint32_t struct_size;
    uint32_t version;                  /* A-GRA's RoutePlanID.Version */
    uint64_t plan_id;                  /* the caller's, not 0 */
    int32_t for_planning_use_only;     /* 1: never prepared for activation, or activated */
    int32_t detailed;                  /* A-GRA's MA_RouteType.Detailed */
    fsim_batch_command route;
    const char* remarks_name;          /* its route's Remarks' DisplayName */
    const char* remarks;               /* its route's Remarks' Detail */
    const fsim_point_metadata* points; /* one a point at most, points[0].struct_size bytes apart */
    uint32_t point_count;
    uint32_t path_count;
    const fsim_path_metadata* paths;   /* one a path at most, paths[0].struct_size bytes apart */
} fsim_route_plan;
FSIM_API void fsim_route_plan_init(fsim_route_plan* plan);
/* A plan's status (A-GRA's plan activation status, and its route plan's execution status). */
typedef struct fsim_plan_status {
    uint32_t struct_size;
    int32_t state;                 /* fsim_plan_state */
    uint64_t plan_id;
    uint32_t version;              /* the version kept; 0 before an upload */
    uint32_t revision;             /* its uploads kept: its content's revision */
    int32_t execution;             /* fsim_plan_execution */
    int32_t reason;                /* why its last command failed, or why its execution ended */
    int32_t for_planning_use_only;
    int32_t fa_owned;              /* ABI 1.37: 1, FA's own - loaded by the platform, read only to MA (4.40) */
    fsim_activity_id activity;     /* its activity, since it was activated last; 0 before */
    double percent;                /* of its route: as it flies, or as it ended */
    double start_time, end_time;   /* when it was activated last; when its activity ended (NaN until) */
    uint64_t command_id;           /* its activation's */
} fsim_plan_status;
FSIM_API void fsim_plan_status_init(fsim_plan_status* status);
/* A plan command's answer (A-GRA's MA_MissionPlanActivationCommandStatus): completed or failed, and the plan's state
 * after it. `check` is PrepareForActivation's validation or Activate's NEW as it answered - the point a refusal names
 * (reserved: its index + 1), its activity; fsim_last_command_detail has its details - or Deactivate's CANCEL. */
typedef struct fsim_plan_command_result {
    uint32_t struct_size;
    int32_t completed;             /* 1: A-GRA's COMPLETED; 0: FAILED */
    uint64_t plan_id;
    int32_t command;               /* fsim_plan_command */
    int32_t state;                 /* fsim_plan_state: the plan's after it (FSIM_PLAN_INACTIVE: not kept) */
    int32_t reason;                /* why it failed */
    int32_t reserved;
    fsim_command_result check;
} fsim_plan_command_result;
FSIM_API void fsim_plan_command_result_init(fsim_plan_command_result* result);
/* A plan published (A-GRA's MA_RoutePlanMT): FSIM_OK and `*reason` 0 where FA listens for its id (prepared for upload: A-GRA's
 * notification CONFIRMED); wrong_plan_state where it does not; invalid_parameter for id 0 or malformed metadata.
 * FSIM_INVALID_ARGUMENT for a route that cannot be read. Its texts copied. */
FSIM_API int fsim_vehicle_publish_plan(fsim_world* world, uint32_t id, const fsim_route_plan* plan, int32_t* reason);
/* A plan activation command (fsim_plan_command), answered at once. Activate is its route's NEW with `options` (NULL:
 * fsim_command_options_init's); PrepareForActivation its validation with them; Deactivate cancels, as their source and
 * controller, an activity not flying yet. */
FSIM_API int fsim_vehicle_plan_command(fsim_world* world, uint32_t id, uint64_t plan_id, int32_t command, const fsim_command_options* options,
                                       fsim_plan_command_result* result);
/* FA's own deactivation (VI 1.2.5.7), the platform's: a plan ready for activation or activated is deactivated, its live
 * activity canceled with `reason` (0: restricted), its execution canceled. */
FSIM_API int fsim_vehicle_abort_plan(fsim_world* world, uint32_t id, uint64_t plan_id, int32_t reason, fsim_plan_command_result* result);
/* Forget a plan: `*reason` unknown_plan, or wrong_plan_state while its activity is live. */
FSIM_API int fsim_vehicle_remove_plan(fsim_world* world, uint32_t id, uint64_t plan_id, int32_t* reason);
/* A plan's status: FSIM_INVALID_ARGUMENT for one not kept. */
FSIM_API int fsim_vehicle_plan_status(fsim_world* world, uint32_t id, uint64_t plan_id, fsim_plan_status* out);
/* Every plan kept, in the order they were first prepared for upload (A-GRA's query for identifiers only). */
FSIM_API uint32_t fsim_vehicle_plan_count(fsim_world* world, uint32_t id);
FSIM_API int fsim_vehicle_plan_at(fsim_world* world, uint32_t id, uint32_t index, fsim_plan_status* out);
/* A plan's content as uploaded last, its metadata with it (A-GRA's query for a route plan): its arrays and texts the
 * library's, until the next fsim_vehicle_get_plan or fsim_activity_get_setpoint. FSIM_INVALID_ARGUMENT for one not kept,
 * or not yet uploaded. */
FSIM_API int fsim_vehicle_get_plan(fsim_world* world, uint32_t id, uint64_t plan_id, fsim_route_plan* out);
FSIM_API const char* fsim_plan_command_name(int command);     /* "prepare_for_upload", "upload", ... */
FSIM_API const char* fsim_plan_state_name(int state);         /* "inactive", "ready_for_upload", ... */
FSIM_API const char* fsim_plan_execution_name(int execution); /* "none", "pending", "executing", ... */
FSIM_API const char* fsim_point_source_name(int source);      /* "auto_routed", "operator_defined" */

/* FA's own plans and the airfields (ABI 1.37; docs/flight-autonomy.md, 4.40): the platform's, loaded before a mission and read
 * only to MA. A place on a runway (A-GRA's Point3D_Type): above the WGS-84 ellipsoid unless its reference says otherwise; its
 * latitude NaN, none. */
typedef struct fsim_runway_point {
    double latitude_rad, longitude_rad, altitude_m;
    double altitude_reference;     /* fsim_altitude_reference; NaN, above the ellipsoid */
} fsim_runway_point;
/* A runway's takeoff or landing coordinates (A-GRA's RunwayCoordinatesType): its start - required where any is given - its
 * threshold and limit. */
typedef struct fsim_runway_coordinates {
    fsim_runway_point start, threshold, limit;
} fsim_runway_coordinates;
/* A runway (A-GRA's AirfieldRunwayType): its takeoff coordinates or its landing coordinates, or both. */
typedef struct fsim_runway {
    uint32_t struct_size;
    uint32_t reserved;
    uint64_t runway_id;            /* not 0 */
    double direction_rad;          /* from true north, 0 to 2 pi; NaN none */
    double available_length_m;     /* above 0; NaN none */
    fsim_runway_coordinates takeoff, landing;
} fsim_runway;
FSIM_API void fsim_runway_init(fsim_runway* runway); /* no id, every place and value left out */
/* An airfield (A-GRA's AirfieldReportMDT): 16 runways at most. */
typedef struct fsim_airfield {
    uint32_t struct_size;
    uint32_t runway_count;
    uint64_t airfield_id;          /* not 0 */
    const char* icao;              /* four capitals, or NULL or "": none */
    double qnh_pa;                 /* 850 to 1,100 hPa; NaN none */
    const fsim_runway* runways;    /* runways[0].struct_size bytes apart */
    uint32_t revision;             /* read back: its loads kept (ignored as loaded) */
    uint32_t reserved;
} fsim_airfield;
FSIM_API void fsim_airfield_init(fsim_airfield* airfield);
/* An airfield loaded by the platform: kept in place of any by its id, its revision one more. `*reason` 0, invalid_parameter
 * (a malformed one), plan_store_full (32 kept). */
FSIM_API int fsim_vehicle_load_airfield(fsim_world* world, uint32_t id, const fsim_airfield* airfield, int32_t* reason);
/* Every airfield kept, in the order they were first loaded; one read back, its runways and ICAO code the library's until the
 * next of these reads. FSIM_INVALID_ARGUMENT for one not kept. */
FSIM_API uint32_t fsim_vehicle_airfield_count(fsim_world* world, uint32_t id);
FSIM_API int fsim_vehicle_get_airfield_at(fsim_world* world, uint32_t id, uint32_t index, fsim_airfield* out);
FSIM_API int fsim_vehicle_get_airfield(fsim_world* world, uint32_t id, uint64_t airfield_id, fsim_airfield* out);
/* FA's own plan, the platform's: kept uploaded and read only to MA, in place of any plan by its id not flying. `*reason` 0,
 * invalid_parameter (as a published plan's), unknown_airfield (a takeoff's or a landing's path naming an airfield or a
 * runway the vehicle does not keep), wrong_plan_state (the plan it replaces flies), plan_store_full. */
FSIM_API int fsim_vehicle_load_plan(fsim_world* world, uint32_t id, const fsim_route_plan* plan, int32_t* reason);

/* A route plan's validation (ABI 1.38; docs/flight-autonomy.md, 4.41; A-GRA's RoutePlanValidationCommand): what it is
 * validated in and against - each NaN, as the vehicle is now. */
typedef struct fsim_plan_validation {
    uint32_t struct_size;
    uint32_t parts;                    /* A-GRA's PlanPart: bit i for fsim_path_type i - a patch's; its verdict theirs; 0 the whole plan */
    double wind_north_ms, wind_east_ms; /* A-GRA's WindData: where it blows to; NaN, what the air data measure now */
    double gust_ms;                    /* its gusts, behind the aircraft with it */
    double origin_latitude_rad, origin_longitude_rad; /* A-GRA's Origin: where it is validated from; NaN, where the aircraft is */
    double origin_altitude_m;          /* above the WGS-84 ellipsoid; NaN, the aircraft's */
    int32_t modify_to_validate;        /* A-GRA's ModifyToValidate: 1, values held to the aircraft's limits (adjustments); 0, findings */
    int32_t reserved;
} fsim_plan_validation;
FSIM_API void fsim_plan_validation_init(fsim_plan_validation* validation);
/* Its answer (A-GRA's RoutePlanValidation): `valid` over its parts, and `check`, the route's validation as it answered - its
 * reason and the point it names (reserved: its index + 1); fsim_last_command_detail and _finding have the rest. */
typedef struct fsim_plan_validation_result {
    uint32_t struct_size;
    int32_t valid;                     /* 1: A-GRA's VALID; 0: INVALID */
    fsim_command_result check;
} fsim_plan_validation_result;
FSIM_API void fsim_plan_validation_result_init(fsim_plan_validation_result* result);
/* A route plan validated without flying it - kept or not; nothing is kept or flown. `validation` NULL: as the vehicle is now.
 * The check's reason invalid_parameter for a malformed plan or validation. */
FSIM_API int fsim_vehicle_validate_plan(fsim_world* world, uint32_t id, const fsim_route_plan* plan, const fsim_plan_validation* validation,
                                        fsim_plan_validation_result* out);
/* A kept plan's, as uploaded last: the check's reason unknown_plan, or wrong_plan_state before its first upload. */
FSIM_API int fsim_vehicle_validate_stored_plan(fsim_world* world, uint32_t id, uint64_t plan_id, const fsim_plan_validation* validation,
                                               fsim_plan_validation_result* out);

/* Reports (ABI 1.12; docs/flight-autonomy.md, 4.12): what an activity flies, and where to. */
/* What a live activity flies now, or waits to fly (A-GRA's last flight command), as the batch item that would command
 * it: its kind and code (a level, fsim_support, fsim_mode); its fields - a level's all of them (as
 * fsim_command_field_count_full counts them), a mode's, a route's or a curve's options, a support command's (the
 * engines': four throttles); a behaviour's command; a route's waypoints, or a curve's segments with the appended ones
 * (its flyout curve, from the reference in fields 0-2) - FSIM_BATCH_CURVE's where each is a Bezier's form, else
 * FSIM_BATCH_NURBS's (ABI 1.24; `nurbs` set where the caller's struct has it); a route's loiters (ABI 1.28, where the
 * caller's struct has them: complete, their place their points'), and its planned states (ABI 1.31, likewise: as placed,
 * their altitude reference completed), and its paths (ABI 1.33, likewise). A waiting one's is as given. Its arrays are the library's,
 * valid until the next setpoint read, world step, reset or destroy; `options` NULL. `out->struct_size` set by the
 * caller. FSIM_INVALID_ARGUMENT for an activity not live. */
FSIM_API int fsim_activity_get_setpoint(fsim_world* world, fsim_activity_id activity, fsim_batch_command* out);

/* Where an activity flies to (A-GRA's ActualEndPoint, MA_EndPointType): a point, a turn flown by or over it, a loiter. */
enum fsim_end_point_kind { FSIM_END_POINT_WAYPOINT = 0, FSIM_END_POINT_TURN_POINT, FSIM_END_POINT_LOITER_POINT };
typedef struct fsim_end_point {
    uint32_t struct_size;
    int32_t kind;                          /* fsim_end_point_kind */
    double latitude_rad, longitude_rad;
    double altitude_m, altitude_reference; /* fsim_altitude_reference; a waiting route's as given (NaN: the point before's) */
    double turn;                           /* a turn point's fsim_turn_type; NaN otherwise */
    uint64_t id;                           /* a route waypoint's id; 0 none */
    int32_t index;                         /* its waypoint or curve segment; 0 a pattern's or the position level's point */
} fsim_end_point;
FSIM_API void fsim_end_point_init(fsim_end_point* point);
FSIM_API const char* fsim_end_point_kind_name(int kind); /* "waypoint", "turn_point", "loiter_point" */
/* A live activity's end points: the point it flies to now, then those after it - a route's waypoints (a repeating
 * route's round again), a curve's segment ends, a pattern's fix, the position level's point - `max` at most into
 * `out` (`out[0].struct_size` bytes apart), `*count` how many. None for one not live, an hsa's, a behaviour's. */
FSIM_API int fsim_activity_end_points(const fsim_world* world, fsim_activity_id activity, fsim_end_point* out, uint32_t max, uint32_t* count);

/* The navigation report (ABI 1.13; docs/flight-autonomy.md, 4.14; A-GRA's MA_NavigationReport): what the vehicle flies
 * on, how much is left and for how long, its playtime to its recovery point and its contingency. Fuel in kg, a
 * battery's charge in J; its consumption now - the engines' fuel flow (kg/s) or the power the battery gives (W). */
enum fsim_energy { FSIM_ENERGY_UNKNOWN = 0, FSIM_ENERGY_FUEL, FSIM_ENERGY_BATTERY };
/* A-GRA's SystemContingencyLevelEnum: the platform reports a low fuel state (flight critical: at or below the reserve,
 * or an engine starved); it models no subsystem failures and no communications. */
enum fsim_contingency { FSIM_CONTINGENCY_NORMAL = 0, FSIM_CONTINGENCY_MISSION_CRITICAL, FSIM_CONTINGENCY_FLIGHT_CRITICAL, FSIM_CONTINGENCY_LOST_COMMS };
typedef struct fsim_navigation_report {
    uint32_t struct_size;
    int32_t energy;              /* fsim_energy: FSIM_ENERGY_UNKNOWN where its flight model tells of neither */
    double fuel_kg;              /* A-GRA's Fuel: in its tanks (0 for a battery) */
    double remaining, capacity;  /* fuel (kg) or charge (J): left, and full */
    double percent;              /* A-GRA's Percent: remaining over capacity */
    double consumption;          /* now: fuel flow (kg/s) or power (W) */
    double endurance_s;          /* A-GRA's Duration: remaining over consumption now (infinite while it consumes nothing,
                                  * 0 with nothing left) */
    double reserve;              /* kept for the end: the reserve fraction of capacity */
    double playtime_s;           /* A-GRA's Playtime: remaining less the reserve and the return, over consumption now (0 once
                                  * past it); NaN without a recovery point */
    double return_distance_m;    /* to the recovery point, over the ground */
    double return_tas_ms;        /* it would fly back at: its best-range speed (the performance tables), else its cruise */
    double return_consumption;   /* it would burn back: kg/s or W */
    int32_t contingency;         /* fsim_contingency */
    int32_t starved;             /* its engines have nothing left: a fuel burner's tanks empty, or the battery spent */
} fsim_navigation_report;
FSIM_API void fsim_navigation_report_init(fsim_navigation_report* report);
FSIM_API int fsim_vehicle_navigation_report(const fsim_world* world, uint32_t id, fsim_navigation_report* out);
FSIM_API const char* fsim_energy_name(int energy);           /* "unknown", "fuel", "battery" */
FSIM_API const char* fsim_contingency_name(int contingency); /* "NORMAL", "MISSION_CRITICAL", "FLIGHT_CRITICAL", "LOST_COMMS" */
/* Where the vehicle recovers to and what it keeps for the end: its playtime counts the return and the reserve. */
typedef struct fsim_navigation_settings {
    uint32_t struct_size;
    int32_t recovery;                                  /* a recovery point set */
    double latitude_deg, longitude_deg, altitude_msl_m;
    double reserve_fraction;                           /* of capacity, fuel or charge, [0, 1): 0.1 by default */
} fsim_navigation_settings;
FSIM_API void fsim_navigation_settings_init(fsim_navigation_settings* settings); /* no recovery point, a tenth kept */
/* FSIM_INVALID_ARGUMENT for a point off the Earth or a reserve outside [0, 1). */
FSIM_API int fsim_vehicle_set_navigation(fsim_world* world, uint32_t id, const fsim_navigation_settings* settings);
FSIM_API int fsim_vehicle_get_navigation(const fsim_world* world, uint32_t id, fsim_navigation_settings* out);

/* ABI 1.18 (docs/flight-autonomy.md, 4.20; A-GRA's QNH setting, VI 1.2.6.5): the QNH a vehicle's barometric altimeter
 * is set to, Pa - what its barometric altitudes are read and flown at; 1013.25 hPa until set. FSIM_OK applied;
 * FSIM_INVALID_ARGUMENT outside 850 to 1,100 hPa, or an unknown vehicle (nothing changes; fsim_last_error() says). */
FSIM_API int fsim_vehicle_set_qnh(fsim_world* world, uint32_t id, double qnh_pa);
FSIM_API int fsim_vehicle_qnh(const fsim_world* world, uint32_t id, double* qnh_pa);
/* What A-GRA's detailed position report carries beyond the state (A-GRA's MA_AirDataType, VI 1.2.6.8): what the
 * vehicle's altimeter reads, and the air it reads it in. */
typedef struct fsim_state_data {
    uint32_t struct_size;
    uint32_t reserved;
    double indicated_altitude_m;      /* A-GRA's IndicatedBaroAltitude: the standard atmosphere's height of the static
                                       * pressure above the QNH it is set to */
    double indicated_altitude_rate_ms; /* A-GRA's BarometricAltitudeRate */
    double kollsman_hpa;              /* A-GRA's Kollsman: its QNH, hPa */
    double static_pressure_pa;        /* the air's where the vehicle is */
    double static_temperature_k;
    /* ABI 1.19 (docs/flight-autonomy.md, 4.21): A-GRA's OrientationRate and OrientationAcceleration - how fast its Euler
     * angles (yaw, pitch, roll over north, east and down) change, and how that changes; NaN pitched within 0.06 degrees
     * of straight up or down */
    double yaw_rate_rad_s, pitch_rate_rad_s, roll_rate_rad_s;
    double yaw_acceleration_rad_s2, pitch_acceleration_rad_s2, roll_acceleration_rad_s2;
    double wander_angle_rad;          /* A-GRA's WanderAngle: 0 (its navigation frame is north's) */
    double wind_north_ms, wind_east_ms, wind_down_ms; /* A-GRA's wind data: the air's velocity over the ground where it is */
    /* ABI 1.20 (docs/flight-autonomy.md, 4.22): A-GRA's MagneticHeading - its heading from magnetic north - and the
     * declination that turns it true: the World Magnetic Model 2025's where it is, at the world's date */
    double magnetic_heading_rad, declination_rad;
} fsim_state_data;
FSIM_API void fsim_state_data_init(fsim_state_data* data);
FSIM_API int fsim_vehicle_state_data(const fsim_world* world, uint32_t id, fsim_state_data* out);
/* ABI 1.20 (4.22): the Earth's magnetic field, the World Magnetic Model 2025 (NOAA NCEI and the British Geological
 * Survey), at a geodetic place, a height above the WGS-84 ellipsoid and a decimal year (outside 2025.0 to 2030.0,
 * carried on at its rates of change): north, east and down, horizontal and total (nT), declination and inclination.
 * FSIM_INVALID_ARGUMENT for a place or date not finite. */
typedef struct fsim_magnetic_field {
    uint32_t struct_size;
    uint32_t reserved;
    double north_nt, east_nt, down_nt, horizontal_nt, total_nt;
    double declination_rad, inclination_rad;
} fsim_magnetic_field;
FSIM_API void fsim_magnetic_field_init(fsim_magnetic_field* field);
FSIM_API int fsim_magnetic_field_at(double latitude_rad, double longitude_rad, double height_m, double decimal_year, fsim_magnetic_field* out);
/* A UTC time (Unix seconds) as a decimal year; and the date the model is read at for a world now: its UTC's, held
 * within 2025.0 to 2030.0 (a world whose clock was never set reads 2025.0). NaN for no world. */
FSIM_API double fsim_decimal_year(double unix_seconds);
FSIM_API double fsim_world_magnetic_year(const fsim_world* world);

/* ABI 1.19 (docs/flight-autonomy.md, 4.21; A-GRA's ReferenceFrame and its relative points): frames by id - fixed, moving
 * at a constant velocity from a time, or following a vehicle - and where a point in one is at a time. The Earth is a
 * sphere of the mean radius, as for all the platform's local geometry. */
enum fsim_frame_origin { FSIM_FRAME_FIXED = 0, FSIM_FRAME_MOVING, FSIM_FRAME_VEHICLE };
/* how a point's offsets are turned (A-GRA's RotationEnum): x north, y east, z down; by the origin's yaw; by its body's
 * axes (x forward, y right, z down, pitched and rolled); by its track over the ground */
enum fsim_frame_rotation { FSIM_FRAME_UNROTATED = 0, FSIM_FRAME_YAW, FSIM_FRAME_ATTITUDE, FSIM_FRAME_HEADING };
/* how they are laid out on the Earth (A-GRA's OffsetXY_Enum): in the plane square to the vertical at the origin; along
 * the great circle their way; along the rhumb line */
enum fsim_frame_offsets { FSIM_FRAME_CARTESIAN = 0, FSIM_FRAME_GREAT_CIRCLE, FSIM_FRAME_RHUMB };
typedef struct fsim_frame_spec {
    uint32_t struct_size;
    int32_t origin;                                   /* fsim_frame_origin */
    uint32_t vehicle;                                 /* FSIM_FRAME_VEHICLE: the vehicle it follows */
    uint32_t reserved;
    double latitude_rad, longitude_rad, altitude_msl_m; /* a fixed origin's; a moving one's at time_s */
    double yaw_rad, pitch_rad, roll_rad;              /* a fixed or moving origin's orientation */
    double north_ms, east_ms, down_ms;                /* a moving origin's velocity */
    double time_s;                                    /* when a moving origin is at its place (fsim_world_time) */
} fsim_frame_spec;
FSIM_API void fsim_frame_spec_init(fsim_frame_spec* spec);
/* FSIM_INVALID_ARGUMENT: refused - a value not finite, a latitude off the Earth, an origin not one, an unknown vehicle */
FSIM_API int fsim_world_create_frame(fsim_world* world, const fsim_frame_spec* spec, uint64_t* id);
FSIM_API int fsim_world_remove_frame(fsim_world* world, uint64_t id);
typedef struct fsim_frame_offset {
    uint32_t struct_size;
    int32_t rotation;  /* fsim_frame_rotation */
    int32_t offsets;   /* fsim_frame_offsets */
    int32_t reserved;
    double x, y, z;    /* m along the axes; z down */
} fsim_frame_offset;
FSIM_API void fsim_frame_offset_init(fsim_frame_offset* offset);
/* Where a point in a frame is at a time (NaN: now) - a vehicle's frame carried on at its velocity to another time.
 * FSIM_INVALID_ARGUMENT for an unknown frame, one whose vehicle is gone, or an offset's code not one. */
FSIM_API int fsim_world_frame_point(const fsim_world* world, uint64_t id, const fsim_frame_offset* offset, double time_s,
                                    double* latitude_rad, double* longitude_rad, double* altitude_msl_m);

/* Operational points (ABI 1.39; docs/flight-autonomy.md, 4.42; A-GRA's OpPoint): kept by the world by id, for a must fly to
 * name - a place on the Earth (latitude and longitude), or in a frame (frame and its offsets, as a route point's) - and the
 * window of bearings from it it is approached from. fsim_op_point_init leaves every field out (fsim_hold(); frame 0). */
typedef struct fsim_op_point {
    uint32_t struct_size;
    uint32_t revision;                  /* read back: one more each time it is set (ignored as set) */
    uint64_t op_point_id;               /* not 0 */
    double latitude_rad, longitude_rad; /* on the Earth; with a frame, left out */
    double altitude_m, altitude_reference; /* fsim_altitude_reference; left out, none (flown at the aircraft's) */
    double frame;                       /* in this frame (fsim_world_create_frame's id) */
    double frame_rotation, frame_offsets; /* fsim_frame_rotation, fsim_frame_offsets */
    double frame_x_m, frame_y_m, frame_z_m; /* its offsets (z down: given, its altitude the frame's there) */
    double ingress_min_rad, ingress_max_rad; /* the bearings from it it is approached from: both or neither, within half a turn */
} fsim_op_point;
FSIM_API void fsim_op_point_init(fsim_op_point* point);
/* Kept in place of any by its id, its revision one more. `*reason` 0, or invalid_parameter: id 0; neither a place nor a frame,
 * or both; a latitude off the Earth, a value not finite, a code not one; a frame the world does not have, or offsets
 * without one; an altitude reference without its altitude; a window given one way alone, or beyond half a turn. */
FSIM_API int fsim_world_set_op_point(fsim_world* world, const fsim_op_point* point, int32_t* reason);
FSIM_API int fsim_world_remove_op_point(fsim_world* world, uint64_t id); /* FSIM_INVALID_ARGUMENT for one not kept */
/* The points kept, by id; one read back. FSIM_INVALID_ARGUMENT for one not kept. */
FSIM_API uint32_t fsim_world_op_point_count(const fsim_world* world);
FSIM_API int fsim_world_get_op_point_at(const fsim_world* world, uint32_t index, fsim_op_point* out);
FSIM_API int fsim_world_get_op_point(const fsim_world* world, uint64_t id, fsim_op_point* out);

/* Operational zones (ABI 1.40; docs/flight-autonomy.md, 4.43; A-GRA's OpZone and its ZoneType): kept by the world by id for a
 * must fly to name, or given with one. Its shape: a polygon - 3 to 32 vertices, and up to 4 holes inside it of 3 to 32 each -
 * an ellipse (semi-axes, the major's bearing from true north within a quarter turn), a rectangle (width across and height
 * along its bearing) or a slant range area (its greatest and least range over the ground, its bearings from its point, the
 * least clockwise to the most, turned by its orientation); on the Earth, or in a frame (its x and y along the frame's axes,
 * turned with its yaw or track as frame_rotation says); its band of altitudes (left out, from the surface, with no top); a
 * velocity from time_s (World::time; left out, when set). fsim_op_zone_init leaves every field out (fsim_hold(); none of
 * its arrays). */
enum fsim_zone_shape { FSIM_ZONE_POLYGON = 0, FSIM_ZONE_ELLIPSE, FSIM_ZONE_RECTANGLE, FSIM_ZONE_SLANT_RANGE };
typedef struct fsim_zone_vertex {
    double latitude_rad, longitude_rad; /* on the Earth */
    double x_m, y_m;                    /* or in the zone's frame */
} fsim_zone_vertex;
typedef struct fsim_op_zone {
    uint32_t struct_size;
    uint32_t revision;                  /* read back: one more each time it is set (ignored as set) */
    uint64_t op_zone_id;                /* not 0 */
    double shape;                       /* fsim_zone_shape */
    const fsim_zone_vertex* vertices;   /* a polygon's, vertex_count of them */
    uint32_t vertex_count;
    uint32_t hole_count;                /* its holes: hole_sizes[k] vertices each from holes[k] */
    const fsim_zone_vertex* const* holes;
    const uint32_t* hole_sizes;
    double latitude_rad, longitude_rad, x_m, y_m; /* the others' centre (a slant range area's point): on the Earth, or in the frame */
    double semi_major_m, semi_minor_m, width_m, height_m, range_min_m, range_max_m;
    double azimuth_min_rad, azimuth_max_rad, orientation_rad;
    double altitude_min_m, altitude_max_m, altitude_reference; /* fsim_altitude_reference of the band */
    double frame, frame_rotation;       /* fsim_world_create_frame's id; fsim_frame_rotation */
    double north_ms, east_ms, time_s;   /* moving */
} fsim_op_zone;
FSIM_API void fsim_op_zone_init(fsim_op_zone* zone);
/* Kept in place of any by its id, its revision one more. `*reason` 0, or invalid_parameter for one A-GRA's schema would not
 * take: a shape not one; a polygon not simple, its holes not inside it; a centre off the Earth; dimensions not above 0 (a
 * least range not below the most); bearings beyond their turn; a band upside down; a frame the world does not have; a
 * velocity with a frame. */
FSIM_API int fsim_world_set_op_zone(fsim_world* world, const fsim_op_zone* zone, int32_t* reason);
FSIM_API int fsim_world_remove_op_zone(fsim_world* world, uint64_t id); /* FSIM_INVALID_ARGUMENT for one not kept */
/* The zones kept, by id; one read back, its vertices and holes the library's until the next of these reads. FSIM_INVALID_ARGUMENT
 * for one not kept. */
FSIM_API uint32_t fsim_world_op_zone_count(const fsim_world* world);
FSIM_API int fsim_world_get_op_zone_at(fsim_world* world, uint32_t index, fsim_op_zone* out);
FSIM_API int fsim_world_get_op_zone(fsim_world* world, uint64_t id, fsim_op_zone* out);
/* A must fly (FSIM_MODE_MUST_FLY's fields, `count` of them) with its zone given (location FSIM_MUST_FLY_ZONE): the zone
 * checked - a field at fault named from 10 - and laid out as it is given, then entered; NULL none, as fsim_vehicle_submit_mode.
 * fsim_activity_update_must_fly: its fields merged, and a zone given replacing its own. */
FSIM_API int fsim_vehicle_submit_must_fly(fsim_world* world, uint32_t id, const double* fields, uint32_t count, const fsim_op_zone* zone,
                                          const fsim_command_options* options, fsim_command_result* result);
FSIM_API int fsim_activity_update_must_fly(fsim_world* world, fsim_activity_id activity, const double* fields, uint32_t count,
                                           const fsim_op_zone* zone, fsim_command_result* result);
FSIM_API int fsim_activity_update_must_fly_by(fsim_world* world, fsim_activity_id activity, int source, uint32_t controller, const double* fields,
                                              uint32_t count, const fsim_op_zone* zone, fsim_command_result* result);

/* A flight mode's performance profile (ABI 1.14; docs/flight-autonomy.md, 4.15; A-GRA's
 * MA_FlightControlModesPerformanceProfileType, VI 1.2.6.7): the guard rails a mission autonomy shapes its commands
 * within, worked out at the vehicle's condition now (its altitude, weight and airspeed; its flaps and gear) from its
 * performance tables, its envelope and its loops. Airspeeds are true, altitudes above sea level; NaN where a value
 * does not depend on one, or where there is none. */
typedef struct fsim_profile_point {                    /* A-GRA's MA_AirspeedLimitType, MA_SpeedType, MA_FuelBurnRateType */
    double value;                                      /* m/s (an airspeed, a climb or descent rate), or a burn: kg/s, W */
    double tas_ms, altitude_msl_m, weight_kg;
} fsim_profile_point;
typedef struct fsim_profile_acceleration {             /* A-GRA's MA_AccelerationLimitsType: body axes, x forward, y right, */
    double x_ms2, y_ms2, z_ms2;                        /* z down; the specific force (1 g of lift is -9.81 in z) */
    double mach, tas_ms, altitude_msl_m, weight_kg;
} fsim_profile_acceleration;
typedef struct fsim_profile_excess_power {             /* A-GRA's MA_SpecificExcessPowerType, at full power */
    double climb_ms, acceleration_ms2;                 /* holding the speed; holding the height */
    double tas_ms, altitude_msl_m, weight_kg;
} fsim_profile_excess_power;
typedef struct fsim_profile_orientation {              /* A-GRA's MA_OrientationLimitType; pitch_min_rad the most nose-down */
    double yaw_rad, pitch_rad, pitch_min_rad, roll_rad;
    double tas_ms, altitude_msl_m, weight_kg;
} fsim_profile_orientation;
typedef struct fsim_profile_rates {                    /* A-GRA's MA_OrientationRateLimitsType, body axes */
    double roll_rad_s, pitch_rad_s, yaw_rad_s, tas_ms;
} fsim_profile_rates;
typedef struct fsim_performance_profile {
    uint32_t struct_size;
    int32_t mode;                                      /* fsim_flight_mode */
    int32_t energy;                                    /* fsim_energy: what `burn` is, fuel (kg/s) or a battery's power (W) */
    int32_t clean, flaps_out, gear_down;               /* flaps and gear up: what the tables give (flown clean) is in */
    double time_s, altitude_msl_m, weight_kg, tas_ms;  /* the condition it was worked out at */
    double min_altitude_msl_m, max_altitude_msl_m;     /* none; the ceiling at the weight now */
    double max_turn_rate_rad_s, max_climb_rate_ms;     /* at the airspeed now; what guidance asks at most */
    /* arrays the library owns until the next profile asked of this world, or its destroy */
    const fsim_profile_point *min_airspeed, *max_airspeed, *best_endurance_airspeed, *best_range_airspeed; /* against altitude */
    uint32_t min_airspeed_count, max_airspeed_count, best_endurance_airspeed_count, best_range_airspeed_count;
    const fsim_profile_acceleration *min_acceleration, *max_acceleration, *max_deceleration; /* against speed and altitude */
    uint32_t min_acceleration_count, max_acceleration_count, max_deceleration_count;
    const fsim_profile_excess_power* excess_power;
    uint32_t excess_power_count;
    const fsim_profile_point *max_descent_rate, *burn;
    uint32_t max_descent_rate_count, burn_count;
    const fsim_profile_orientation* max_orientation;   /* at the condition now */
    const fsim_profile_rates* max_orientation_rate;
    uint32_t max_orientation_count, max_orientation_rate_count;
} fsim_performance_profile;
FSIM_API void fsim_performance_profile_init(fsim_performance_profile* profile);
/* FSIM_OK and `out` filled; FSIM_INVALID_ARGUMENT for an unknown vehicle, a mode A-GRA gives no profile (another than
 * HSA/CSA, waypoint and curve following), or one the vehicle does not offer - `reason` (may be NULL) says which
 * (fsim_reason_name: "unknown_vehicle", "invalid_parameter", "not_supported", "not_implemented"). */
FSIM_API int fsim_vehicle_performance_profile(fsim_world* world, uint32_t id, int32_t mode, fsim_performance_profile* out, int32_t* reason);

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
