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

#ifdef __cplusplus
}
#endif

#endif /* FSIM_C_H */
