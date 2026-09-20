/* The C ABI must be consumable from plain C: this file is compiled as C99. */
#include "fsim/fsim_c.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            fprintf(stderr, "%s:%d: check failed: %s (%s)\n", __FILE__, __LINE__, #cond, fsim_last_error()); \
            return 1;                                                                                \
        }                                                                                            \
    } while (0)

int main(int argc, char** argv) {
    fsim_options opt;
    fsim_buffers buf;
    fsim_vecenv* env = NULL;
    float* actions;
    float* first;
    const float* obs0;
    size_t n, i, k;

    CHECK(fsim_abi_version() == FSIM_ABI_VERSION);
    CHECK(strlen(fsim_version()) > 0);

    fsim_options_init(&opt);
    CHECK(opt.struct_size == sizeof(fsim_options));
    opt.num_envs = 2;
    opt.vehicles_per_env = 2;
    opt.workers = 2;
    opt.seed = 42;
    opt.max_episode_steps = 6;
    if (argc > 1) opt.jsbsim_root = argv[1];

    /* Bad arguments are reported, not crashed on. */
    CHECK(fsim_vecenv_create(NULL, &env) == FSIM_INVALID_ARGUMENT);
    CHECK(strlen(fsim_last_error()) > 0);
    opt.aircraft = "no_such_aircraft";
    CHECK(fsim_vecenv_create(&opt, &env) != FSIM_OK);
    CHECK(env == NULL);
    opt.aircraft = "c172x";

    CHECK(fsim_vecenv_create(&opt, &env) == FSIM_OK);
    CHECK(env != NULL);
    CHECK(strlen(fsim_last_error()) == 0);

    memset(&buf, 0, sizeof buf);
    buf.struct_size = sizeof buf;
    CHECK(fsim_vecenv_buffers(env, &buf) == FSIM_OK);
    CHECK(buf.num_envs == 2 && buf.vehicles_per_env == 2);
    CHECK(buf.observation_size == 20 && buf.action_size == 4);
    CHECK(buf.observations && buf.rewards && buf.terminated && buf.truncated && buf.final_observations && buf.episode_steps);
    CHECK(fabs(buf.agent_step_seconds - 4.0 / 120.0) < 1e-12);
    CHECK(strcmp(fsim_vecenv_observation_name(env, 0), "alt_msl_km") == 0);
    CHECK(strcmp(fsim_vecenv_action_name(env, 3), "throttle") == 0);
    CHECK(strcmp(fsim_vecenv_observation_name(env, 999), "") == 0);

    n = (size_t)buf.num_envs * buf.vehicles_per_env;
    first = (float*)malloc(n * buf.observation_size * sizeof(float));
    memcpy(first, buf.observations, n * buf.observation_size * sizeof(float));

    actions = (float*)calloc(n * buf.action_size, sizeof(float));
    CHECK(fsim_vecenv_step(env, actions, 1) == FSIM_INVALID_ARGUMENT);
    for (k = 0; k < 6; ++k) CHECK(fsim_vecenv_step(env, actions, n * buf.action_size) == FSIM_OK);
    CHECK(fsim_vecenv_buffers(env, &buf) == FSIM_OK);
    CHECK(buf.episode_steps[0] == 6 && buf.episode_steps[1] == 6);
    for (i = 0; i < n; ++i) CHECK(buf.truncated[i] == 1);
    CHECK(fsim_vecenv_vehicle_steps(env) == n * 6 * 4);

    /* Buffers are stable pointers across steps. */
    obs0 = buf.observations;
    CHECK(fsim_vecenv_step(env, actions, n * buf.action_size) == FSIM_OK);
    CHECK(fsim_vecenv_buffers(env, &buf) == FSIM_OK);
    CHECK(buf.observations == obs0);
    CHECK(buf.episode_steps[0] == 0);

    /* reset with the same seed reproduces the first observations (JSBSim's
     * IC solver leaves ~1e-16 residue from the previous state, so not bit-exact). */
    CHECK(fsim_vecenv_reset(env, 42) == FSIM_OK);
    CHECK(fsim_vecenv_buffers(env, &buf) == FSIM_OK);
    for (i = 0; i < n * buf.observation_size; ++i) CHECK(fabs((double)first[i] - (double)buf.observations[i]) < 1e-6);

    fsim_vecenv_destroy(env);
    fsim_vecenv_destroy(NULL);
    free(actions);
    free(first);
    printf("c abi ok\n");
    return 0;
}
