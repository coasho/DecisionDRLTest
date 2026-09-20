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

    /* --- World / vehicle API ------------------------------------------------ */
    {
        fsim_world_options wo;
        fsim_world* world = NULL;
        fsim_vehicle_spec spec;
        uint32_t a = 0, b = 0, ids[4], count;
        const fsim_vehicle_state* st;
        fsim_attitude_command att;
        fsim_behavior_command beh;
        fsim_environment env;
        fsim_message msg;
        const char* pnames[] = {"range_m"};
        double pvalues[] = {150.0};
        const char* enames[] = {"position_sigma_m"};
        double evalues[] = {2.0};
        double prop;

        fsim_world_options_init(&wo);
        wo.name = "c-abi-test";
        wo.workers = 1;
        wo.pin_workers = 0;
        wo.publish = 0;
        if (argc > 1) wo.jsbsim_root = argv[1];
        CHECK(fsim_world_create(&wo, &world) == FSIM_OK);
        CHECK(world != NULL);
        CHECK(fsim_world_published(world) == 0);

        fsim_vehicle_spec_init(&spec);
        spec.name = "alpha";
        spec.altitude_msl_m = 1500.0;
        spec.airspeed_ms = 60.0;
        CHECK(fsim_world_create_vehicle(world, &spec, &a) == FSIM_OK);
        CHECK(a != 0);
        spec.name = "bravo";
        spec.longitude_deg += 0.02;
        CHECK(fsim_world_create_vehicle(world, &spec, &b) == FSIM_OK);
        CHECK(b != 0 && b != a);
        spec.name = "alpha"; /* duplicate */
        CHECK(fsim_world_create_vehicle(world, &spec, &count) != FSIM_OK);
        CHECK(fsim_world_vehicle_count(world) == 2);
        CHECK(fsim_world_vehicle_ids(world, ids, 4) == 2);
        CHECK(fsim_world_find_vehicle(world, "bravo") == b);
        CHECK(strcmp(fsim_vehicle_name(world, a), "alpha") == 0);
        CHECK(strcmp(fsim_vehicle_type(world, a), "jsbsim:c172x") == 0);

        st = fsim_vehicle_state_ptr(world, a);
        CHECK(st != NULL);
        CHECK(fabs(st->altitude_msl_m - 1500.0) < 1.0);

        att.roll_rad = 0.3; att.pitch_rad = 0.03; att.heading_rad = fsim_hold(); att.max_bank_rad = 0.785; att.throttle = fsim_hold(); att.airspeed_ms = 60.0;
        CHECK(fsim_vehicle_command_attitude(world, a, &att) == FSIM_OK);
        CHECK(fsim_vehicle_active_level(world, a) == FSIM_LEVEL_ATTITUDE);
        memset(&beh, 0, sizeof beh);
        beh.id = "pursuit";
        beh.target = a;
        beh.param_names = pnames; beh.param_values = pvalues; beh.param_count = 1;
        CHECK(fsim_vehicle_command_behavior(world, b, &beh) == FSIM_OK);
        CHECK(fsim_vehicle_active_level(world, b) == FSIM_LEVEL_BEHAVIOR);
        beh.id = "no_such_behaviour";
        CHECK(fsim_vehicle_command_behavior(world, b, &beh) == FSIM_OK); /* logged and ignored, previous command kept */
        CHECK(fsim_vehicle_use_controller(world, a, FSIM_LEVEL_ATTITUDE, "pid_attitude") == FSIM_OK);
        CHECK(fsim_vehicle_use_controller(world, a, FSIM_LEVEL_ATTITUDE, "nope") != FSIM_OK);
        CHECK(fsim_vehicle_set_controller_parameter(world, a, FSIM_LEVEL_ATTITUDE, "roll.kp", 2.5) == FSIM_OK);
        CHECK(fsim_vehicle_set_controller_parameter(world, a, FSIM_LEVEL_ATTITUDE, "nope", 2.5) != FSIM_OK);

        CHECK(fsim_vehicle_add_effect(world, b, "gaussian_sensor_noise", enames, evalues, 1) == FSIM_OK);
        CHECK(fsim_vehicle_add_effect(world, 0, "wind_gusts", NULL, NULL, 0) == FSIM_OK);
        CHECK(fsim_vehicle_add_effect(world, b, "no_such_effect", NULL, NULL, 0) != FSIM_OK);

        env.struct_size = sizeof env;
        CHECK(fsim_world_get_environment(world, &env) == FSIM_OK);
        env.wind_direction_deg = 180.0;
        env.wind_speed_ms = 5.0;
        CHECK(fsim_world_set_environment(world, &env) == FSIM_OK);

        CHECK(fsim_comm_attach_protocol(world, a, "beacon", NULL, NULL, 0) == FSIM_OK);
        CHECK(fsim_comm_send(world, b, a, 4, 1, "hi", 2) == FSIM_OK);
        CHECK(fsim_comm_set_medium(world, "link", NULL, NULL, 0) == FSIM_OK);

        CHECK(fsim_world_step(world, 90) == FSIM_OK); /* 3 s */
        CHECK(fabs(fsim_world_time(world) - 3.0) < 1e-9);
        CHECK(fsim_world_vehicle_steps(world) == 2 * 90 * 4);
        st = fsim_vehicle_state_ptr(world, a);
        CHECK(st->euler_rad[0] > 0.1); /* rolling right as commanded */
        CHECK(fsim_vehicle_sensed_ptr(world, b) != NULL);
        CHECK(fsim_vehicle_get_property(world, a, "atmosphere/wind-north-fps", &prop) == FSIM_OK);
        CHECK(prop > 10.0); /* wind from the south blows north: +5 m/s = +16.4 ft/s */
        CHECK(fsim_vehicle_get_property(world, a, "no/such/property", &prop) != FSIM_OK);
        CHECK(fsim_comm_inbox_get(world, a, 999, &msg) != FSIM_OK);

        CHECK(fsim_world_reset_vehicle(world, a, NULL) == FSIM_OK);
        CHECK(fsim_world_remove_vehicle(world, b) == FSIM_OK);
        CHECK(fsim_world_remove_vehicle(world, b) != FSIM_OK);
        CHECK(fsim_world_vehicle_count(world) == 1);
        fsim_world_destroy(world);
        fsim_world_destroy(NULL);
    }
    printf("c abi ok\n");
    return 0;
}
