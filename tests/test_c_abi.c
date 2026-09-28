/* The C ABI must be consumable from plain C: this file is compiled as C99. */
#include "fsim/fsim_c.h"

#include <math.h>
#include <stddef.h>
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

    /* the action's ranges: the aircraft's own where its profile narrows them (1.4) */
    CHECK(fsim_vecenv_set_action_ranges(env, "aircraft") == FSIM_OK);
    CHECK(fsim_vecenv_set_action_ranges(env, "roomy") != FSIM_OK);
    CHECK(fsim_vecenv_set_action_ranges(env, NULL) != FSIM_OK);
    CHECK(fsim_vecenv_set_action_ranges(env, "fixed") == FSIM_OK);

    /* reset with the same seed reproduces the first observations (JSBSim's
     * IC solver leaves ~1e-16 residue from the previous state, so not bit-exact). */
    CHECK(fsim_vecenv_reset(env, 42) == FSIM_OK);
    CHECK(fsim_vecenv_buffers(env, &buf) == FSIM_OK);
    for (i = 0; i < n * buf.observation_size; ++i) CHECK(fabs((double)first[i] - (double)buf.observations[i]) < 1e-6);

    fsim_vecenv_destroy(env);
    fsim_vecenv_destroy(NULL);
    free(actions);
    free(first);

    /* Same-step auto-reset (Stable-Baselines3): the step that ends an episode
     * already returns the next one's first observation, and the step after it
     * is an ordinary one rather than an ignored reset. */
    {
        uint32_t ids[4];
        const char* id0;
        CHECK(fsim_vecenv_create(&opt, &env) == FSIM_OK);
        CHECK(fsim_vecenv_autoreset(env) == FSIM_AUTORESET_NEXT_STEP);
        CHECK(fsim_vecenv_set_autoreset(env, 7) == FSIM_INVALID_ARGUMENT);
        CHECK(fsim_vecenv_set_autoreset(env, FSIM_AUTORESET_SAME_STEP) == FSIM_OK);
        CHECK(fsim_vecenv_autoreset(env) == FSIM_AUTORESET_SAME_STEP);
        CHECK(fsim_vecenv_buffers(env, &buf) == FSIM_OK);
        n = (size_t)buf.num_envs * buf.vehicles_per_env;
        actions = (float*)calloc(n * buf.action_size, sizeof(float));
        for (k = 0; k < 6; ++k) CHECK(fsim_vecenv_step(env, actions, n * buf.action_size) == FSIM_OK);
        CHECK(fsim_vecenv_buffers(env, &buf) == FSIM_OK);
        for (i = 0; i < n; ++i) CHECK(buf.truncated[i] == 1);   /* flags of the step that ended it */
        CHECK(buf.episode_steps[0] == 0);                       /* ... and already a new episode */
        {
            double diff = 0.0;
            for (i = 0; i < buf.observation_size; ++i) diff += fabs((double)buf.observations[i] - (double)buf.final_observations[i]);
            CHECK(diff > 1e-3); /* the first observation of the new episode, not the last of the old */
        }
        CHECK(fsim_vecenv_step(env, actions, n * buf.action_size) == FSIM_OK);
        CHECK(fsim_vecenv_buffers(env, &buf) == FSIM_OK);
        CHECK(buf.episode_steps[0] == 1); /* no step swallowed by a reset */
        CHECK(buf.truncated[0] == 0 && buf.rewards[0] != 0.0f);
        /* Batch order as world ids, for the world calls on the batch's world. */
        CHECK(fsim_vecenv_vehicle_ids(env, ids, 4) == 4);
        CHECK(fsim_vecenv_vehicle_ids(env, NULL, 0) == 4);
        CHECK(strcmp(fsim_vehicle_name(fsim_vecenv_world(env), ids[3]), "env1/1") == 0);
        fsim_vecenv_destroy(env);
        free(actions);
        /* What can be named in fsim_options. */
        id0 = fsim_registered_id(FSIM_REGISTRY_TASK, 0);
        CHECK(strlen(id0) > 0);
        CHECK(strlen(fsim_registered_id(FSIM_REGISTRY_ACTION, 0)) > 0);
        CHECK(strcmp(fsim_registered_id(FSIM_REGISTRY_OBSERVATION, 9999), "") == 0);
        CHECK(strcmp(fsim_registered_id(42, 0), "") == 0);
    }

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
        CHECK(fsim_vehicle_command_behavior(world, b, &beh) == FSIM_INVALID_ARGUMENT); /* refused, previous command kept */
        CHECK(strstr(fsim_last_error(), "unknown_capability") != NULL);
        CHECK(fsim_vehicle_active_level(world, b) == FSIM_LEVEL_BEHAVIOR);
        CHECK(fsim_vehicle_use_controller(world, a, FSIM_LEVEL_ATTITUDE, "pid_attitude") == FSIM_OK);
        CHECK(fsim_vehicle_use_controller(world, a, FSIM_LEVEL_ATTITUDE, "nope") != FSIM_OK);
        CHECK(fsim_vehicle_set_controller_parameter(world, a, FSIM_LEVEL_ATTITUDE, "roll.kp", 2.5) == FSIM_OK);
        CHECK(fsim_vehicle_set_controller_parameter(world, a, FSIM_LEVEL_ATTITUDE, "nope", 2.5) != FSIM_OK);
        {
            double kp = 0.0;
            CHECK(fsim_vehicle_controller_parameter(world, a, FSIM_LEVEL_ATTITUDE, "roll.kp", &kp) == FSIM_OK);
            CHECK(kp == 2.5);
            CHECK(fsim_vehicle_controller_parameter(world, a, FSIM_LEVEL_ATTITUDE, "pitch.kp", &kp) == FSIM_OK);
            CHECK(kp == 2.5); /* the c172x keeps the shared default */
            CHECK(fsim_vehicle_controller_parameter(world, a, FSIM_LEVEL_ATTITUDE, "nope", &kp) != FSIM_OK);
            CHECK(fsim_vehicle_controller_parameter(world, a, FSIM_LEVEL_ATTITUDE, "roll.kp", NULL) != FSIM_OK);
        }

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

        /* A write that runs model code which fails - JSBSim asked to trim an
         * aircraft at 3 m/s throws - is reported, and the process lives on. */
        {
            uint32_t slow = 0;
            spec.name = "too-slow";
            spec.airspeed_ms = 3.0;
            CHECK(fsim_world_create_vehicle(world, &spec, &slow) == FSIM_OK);
            CHECK(fsim_vehicle_set_property(world, slow, "simulation/do_simple_trim", 1.0) == FSIM_ERROR);
            CHECK(strstr(fsim_last_error(), "do_simple_trim") != NULL);
            CHECK(fsim_world_remove_vehicle(world, slow) == FSIM_OK);
            spec.airspeed_ms = 60.0;
        }

        /* A state pointer holds while its vehicle lives, however many vehicles
         * are created after it: bindings keep views on it (the Python SDK's
         * are zero-copy), and a growing store used to move it at 3, 5, 9, 17. */
        {
            char extra[32];
            const fsim_vehicle_state* held = fsim_vehicle_state_ptr(world, a);
            const double before = held->sim_time;
            uint32_t e = 0;
            for (k = 0; k < 20; ++k) {
                snprintf(extra, sizeof extra, "extra-%u", (unsigned)k);
                spec.name = extra;
                CHECK(fsim_world_create_vehicle(world, &spec, &e) == FSIM_OK);
            }
            CHECK(fsim_vehicle_state_ptr(world, a) == held);
            CHECK(fsim_world_step(world, 1) == FSIM_OK);
            CHECK(held->sim_time > before);
        }

        /* Batched calls: one call for many vehicles. */
        {
            fsim_vehicle_state gathered[3];
            uint32_t some[3];
            double rows[2 * 7]; /* two attitude commands, 7 doubles apart */
            some[0] = a; some[1] = b; some[2] = fsim_world_find_vehicle(world, "extra-3");
            CHECK(fsim_world_gather_states(world, some, 3, 0, gathered) == FSIM_OK);
            CHECK(memcmp(&gathered[0], fsim_vehicle_state_ptr(world, a), sizeof(fsim_vehicle_state)) == 0);
            CHECK(memcmp(&gathered[2], fsim_vehicle_state_ptr(world, some[2]), sizeof(fsim_vehicle_state)) == 0);
            CHECK(fsim_world_gather_states(world, some, 3, 1, gathered) == FSIM_OK);
            CHECK(memcmp(&gathered[1], fsim_vehicle_sensed_ptr(world, b), sizeof(fsim_vehicle_state)) == 0);
            some[1] = 99999;
            CHECK(fsim_world_gather_states(world, some, 3, 0, gathered) == FSIM_INVALID_ARGUMENT);
            CHECK(gathered[1].sim_time == 0.0 && gathered[2].sim_time > 0.0); /* the rest are still filled */

            CHECK(fsim_command_field_count(FSIM_LEVEL_ACTUATOR) == 8);
            CHECK(fsim_command_field_count(FSIM_LEVEL_POSITION) == 5);
            CHECK(fsim_command_field_count(FSIM_LEVEL_BEHAVIOR) == 0);
            for (i = 0; i < 2; ++i) {
                double* r = rows + i * 7;
                r[0] = -0.2; r[1] = 0.02; r[2] = fsim_hold(); r[3] = 0.6; r[4] = fsim_hold(); r[5] = 55.0; r[6] = 12345.0;
            }
            some[1] = b;
            CHECK(fsim_world_command_batch(world, FSIM_LEVEL_ATTITUDE, some + 1, 2, rows, 7) == FSIM_OK);
            CHECK(fsim_vehicle_active_level(world, b) == FSIM_LEVEL_ATTITUDE);
            CHECK(fsim_vehicle_active_level(world, some[2]) == FSIM_LEVEL_ATTITUDE);
            CHECK(fsim_world_command_batch(world, FSIM_LEVEL_ATTITUDE, some, 2, rows, 3) == FSIM_INVALID_ARGUMENT); /* stride too short */
            CHECK(fsim_world_command_batch(world, FSIM_LEVEL_BEHAVIOR, some, 2, rows, 0) == FSIM_INVALID_ARGUMENT);
            some[0] = 99999;
            CHECK(fsim_world_command_batch(world, FSIM_LEVEL_ATTITUDE, some, 2, rows, 7) == FSIM_INVALID_ARGUMENT);
            CHECK(fsim_world_step(world, 30) == FSIM_OK); /* 1 s */
            CHECK(fsim_vehicle_state_ptr(world, some[2])->euler_rad[0] < -0.05); /* banking left as commanded */
        }

        CHECK(fsim_world_reset_vehicle(world, a, NULL) == FSIM_OK);
        CHECK(fsim_world_remove_vehicle(world, b) == FSIM_OK);
        CHECK(fsim_world_remove_vehicle(world, b) != FSIM_OK);
        CHECK(fsim_world_vehicle_count(world) == 21);
        fsim_world_destroy(world);
        fsim_world_destroy(NULL);
    }
    /* Scenario files. */
    {
        fsim_scenario* sc = NULL;
        fsim_world_options wo;
        fsim_world* world = NULL;
        uint32_t ids[8];
        size_t count = 0;
        const char* json =
            "{ \"world\": { \"name\": \"c-scenario\", \"publish\": false, \"workers\": 1, \"pin_workers\": false },"
            "  \"environment\": { \"wind\": { \"direction_deg\": 180, \"speed_ms\": 5 } },"
            "  \"vehicles\": [ { \"name\": \"v\", \"count\": 2, \"initial\": { \"alt_msl_m\": 1200, \"heading_deg\": 45, \"airspeed_ms\": 60 },"
            "                   \"command\": { \"level\": \"attitude\", \"pitch_deg\": 3 } } ] }";
        CHECK(fsim_scenario_parse("{ \"vehicles\": [ { } ] }", "bad.json", &sc) != FSIM_OK);
        CHECK(sc == NULL);
        CHECK(strstr(fsim_last_error(), "bad.json") != NULL);
        CHECK(fsim_scenario_load("no/such/file.json", &sc) != FSIM_OK);
        CHECK(fsim_scenario_parse(json, "inline.json", &sc) == FSIM_OK);
        CHECK(fsim_scenario_vehicle_count(sc) == 2);
        fsim_world_options_init(&wo);
        CHECK(fsim_scenario_world_options(sc, &wo) == FSIM_OK);
        CHECK(strcmp(wo.name, "c-scenario") == 0);
        CHECK(wo.publish == 0);
        wo.jsbsim_root = argc > 1 ? argv[1] : NULL;
        CHECK(fsim_world_create(&wo, &world) == FSIM_OK);
        CHECK(fsim_scenario_apply(world, sc, ids, 8, &count) == FSIM_OK);
        CHECK(count == 2);
        CHECK(fsim_world_find_vehicle(world, "v-2") == ids[1]);
        CHECK(fsim_world_step(world, 10) == FSIM_OK);
        CHECK(fsim_vehicle_state_ptr(world, ids[0]) != NULL);
        CHECK(fabs(fsim_vehicle_state_ptr(world, ids[0])->altitude_msl_m - 1200.0) < 30.0);
        fsim_scenario_destroy(sc);
        fsim_scenario_destroy(NULL);
        fsim_world_destroy(world);
    }
    /* Recordings: write one through a world, read it back. */
    {
        fsim_world_options wo;
        fsim_world* world = NULL;
        fsim_vehicle_spec spec;
        uint32_t id = 0, n = 0;
        fsim_recording* rec = NULL;
        const fsim_recorded_sample* samples;
        const fsim_recorded_event* events;
        fsim_world_options_init(&wo);
        wo.name = "c-record";
        wo.publish = 0;
        wo.workers = 1;
        wo.pin_workers = 0;
        wo.jsbsim_root = argc > 1 ? argv[1] : NULL;
        wo.record_path = "c-abi-test.fsrec";
        CHECK(fsim_world_create(&wo, &world) == FSIM_OK);
        fsim_vehicle_spec_init(&spec);
        spec.name = "rec";
        spec.altitude_msl_m = 1200.0;
        spec.airspeed_ms = 60.0;
        CHECK(fsim_world_create_vehicle(world, &spec, &id) == FSIM_OK);
        CHECK(fsim_world_step(world, 12) == FSIM_OK);
        fsim_world_destroy(world);
        CHECK(fsim_recording_load("no-such.fsrec", &rec) != FSIM_OK);
        CHECK(fsim_recording_load("c-abi-test.fsrec", &rec) == FSIM_OK);
        CHECK(fsim_recording_frame_count(rec) == 12);
        CHECK(strcmp(fsim_recording_world_name(rec), "c-record") == 0);
        CHECK(fsim_recording_frame_skip(rec) == 4);
        CHECK(fsim_recording_frame_time(rec, 11) > fsim_recording_frame_time(rec, 0));
        samples = fsim_recording_samples(rec, 5, &n);
        CHECK(samples != NULL && n == 1);
        CHECK(samples[0].slot == 0 && fabs(samples[0].state.altitude_msl_m - 1200.0) < 50.0);
        CHECK(samples[0].inputs.gear_down >= 0.0);
        events = fsim_recording_events(rec, 0, &n);
        CHECK(events != NULL && n == 1 && strcmp(events[0].name, "rec") == 0 && events[0].alive == 1);
        CHECK(fsim_recording_samples(rec, 99, &n) == NULL && n == 0);
        fsim_recording_destroy(rec);
        fsim_recording_destroy(NULL);
        remove("c-abi-test.fsrec");
    }
    /* Capabilities and activities (ABI 1.4), in a world of their own. */
    {
        fsim_world_options wo;
        fsim_world* world = NULL;
        fsim_vehicle_spec spec;
        fsim_attitude_command att;
        uint32_t a = 0, b = 0;
        fsim_world_options_init(&wo);
        wo.name = "c-capabilities";
        wo.publish = 0;
        wo.workers = 1;
        wo.pin_workers = 0;
        wo.jsbsim_root = argc > 1 ? argv[1] : NULL;
        CHECK(fsim_world_create(&wo, &world) == FSIM_OK);
        fsim_vehicle_spec_init(&spec);
        spec.name = "cap-a";
        spec.altitude_msl_m = 1500.0;
        spec.airspeed_ms = 55.0;
        CHECK(fsim_world_create_vehicle(world, &spec, &a) == FSIM_OK);
        spec.name = "cap-b";
        spec.longitude_deg += 0.01;
        CHECK(fsim_world_create_vehicle(world, &spec, &b) == FSIM_OK);
        memset(&att, 0, sizeof att);
        att.roll_rad = 0.1; att.pitch_rad = 0.03; att.heading_rad = fsim_hold(); att.max_bank_rad = 0.785;
        att.throttle = fsim_hold(); att.airspeed_ms = 55.0;
        {
        /* Capabilities and activities (ABI 1.4) */
        fsim_capability_info ci;
        fsim_parameter_info pi;
        fsim_command_options co;
        fsim_command_result cr;
        fsim_activity_info ai;
        const uint32_t ncap = fsim_vehicle_capability_count(world, a);
        uint32_t c, attitude = ncap, velocity = ncap;
        int32_t availability = -1, why = -1;
        double climb[4], cruise[4], rows[8];
        fsim_activity_id acts[2], operator_id;
        climb[0] = 55.0; climb[1] = 1.0; climb[2] = fsim_hold(); climb[3] = fsim_hold();
        cruise[0] = 55.0; cruise[1] = 0.0; cruise[2] = fsim_hold(); cruise[3] = fsim_hold();

        CHECK(ncap >= 12);
        for (c = 0; c < ncap; ++c) {
            CHECK(fsim_vehicle_capability(world, a, c, &ci) == FSIM_OK);
            if (strcmp(ci.id, "fsim.flight.attitude") == 0) attitude = c;
            if (strcmp(ci.id, "fsim.flight.velocity") == 0) velocity = c;
        }
        CHECK(attitude < ncap);
        CHECK(fsim_vehicle_capability(world, a, attitude, &ci) == FSIM_OK);
        CHECK(ci.level == FSIM_LEVEL_ATTITUDE && ci.parameter_count == 6 && (ci.interactions & 2) != 0 && ci.terminating == 0);
        CHECK(ci.axis_groups == (1u | 2u | 4u)); /* lateral, pitch and thrust, apart */
        CHECK(fsim_vehicle_capability_parameter(world, a, attitude, 0, &pi) == FSIM_OK);
        CHECK(strcmp(pi.name, "roll_rad") == 0 && strcmp(pi.unit, "rad") == 0 && pi.optional == 1);
        CHECK(fsim_vehicle_capability(world, a, ncap, &ci) != FSIM_OK);
        CHECK(fsim_vehicle_capability_status(world, a, "fsim.guidance.hold", &availability, &why) == FSIM_OK);
        CHECK(availability == FSIM_AVAILABLE);

        fsim_command_options_init(&co);
        CHECK(co.struct_size == sizeof co && co.source == FSIM_SOURCE_POLICY && co.range == FSIM_RANGE_CLAMP);
        CHECK(fsim_vehicle_submit(world, a, FSIM_LEVEL_VELOCITY, climb, 4, &co, &cr) == FSIM_OK);
        CHECK(cr.status == FSIM_COMMAND_ACCEPTED && (uint32_t)(cr.activity >> 32) == a);
        acts[0] = cr.activity;
        CHECK(fsim_vehicle_submit(world, a, FSIM_LEVEL_VELOCITY, climb, 3, &co, &cr) != FSIM_OK); /* malformed: 4 fields */
        CHECK(fsim_vehicle_submit(world, b, FSIM_LEVEL_VELOCITY, cruise, 4, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
        acts[1] = cr.activity;
        CHECK(fsim_activity_get(world, acts[0], &ai) == FSIM_OK);
        CHECK(ai.state == FSIM_ACTIVITY_PENDING && strcmp(fsim_activity_state_name(ai.state), "pending") == 0);
        CHECK(fsim_world_step(world, 1) == FSIM_OK);
        CHECK(fsim_activity_get(world, acts[0], &ai) == FSIM_OK && ai.state == FSIM_ACTIVITY_ACTIVE && isnan(ai.end_time));
        CHECK(fsim_activity_update(world, acts[0], cruise, 4, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
        memcpy(rows, climb, sizeof climb);
        memcpy(rows + 4, cruise, sizeof cruise);
        CHECK(fsim_activity_update_batch(world, acts, 2, rows, 0) == FSIM_OK);
        /* ABI 1.5: every field, the rotorcraft's appended - which a wing has nothing for */
        CHECK(fsim_command_field_count(FSIM_LEVEL_VELOCITY) == 4 && fsim_command_field_count_full(FSIM_LEVEL_VELOCITY) == 6);
        CHECK(fsim_command_field_count_full(FSIM_LEVEL_ACCELERATION) == 6 && fsim_command_field_count_full(FSIM_LEVEL_POSITION) == 6);
        CHECK(fsim_command_field_count_full(FSIM_LEVEL_ATTITUDE) == 6 && fsim_command_field_count_full(FSIM_LEVEL_BEHAVIOR) == 0);
        CHECK(velocity < ncap && fsim_vehicle_capability_parameter(world, a, velocity, 4, &pi) == FSIM_OK);
        CHECK(strcmp(pi.name, "north_ms") == 0 && pi.unsupported == 1);
        CHECK(fsim_vehicle_capability_parameter(world, a, velocity, 0, &pi) == FSIM_OK && pi.unsupported == 0);
        {
            double full[12];
            int k;
            for (k = 0; k < 2; ++k) {
                full[6 * k + 0] = 55.0; full[6 * k + 1] = 0.0; full[6 * k + 2] = fsim_hold(); full[6 * k + 3] = fsim_hold();
                full[6 * k + 4] = fsim_hold(); full[6 * k + 5] = fsim_hold();
            }
            CHECK(fsim_activity_update(world, acts[0], full, 6, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_update(world, acts[0], full, 5, &cr) != FSIM_OK); /* malformed: 4 or 6 */
            CHECK(fsim_activity_update_batch_n(world, acts, 2, full, 0, 6) == FSIM_OK);
            CHECK(fsim_activity_update_batch_n(world, acts, 2, full, 6, 4) == FSIM_OK); /* the legacy width, rows 6 apart */
            CHECK(fsim_activity_update_batch_n(world, acts, 2, full, 6, 5) != FSIM_OK);
            full[4] = 3.0; /* north_ms, on a wing */
            CHECK(fsim_activity_update(world, acts[0], full, 6, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "not_supported") == 0); /* ABI 1.7: a field it has nothing for */
            {
                /* ABI 1.6: what the answer was about - the field, and the limit its value broke */
                fsim_command_detail d;
                double wild[6];
                CHECK(cr.reserved == 5); /* north_ms, the fifth field */
                fsim_command_detail_init(&d);
                CHECK(d.struct_size == sizeof d && d.index == -1 && isnan(d.from));
                CHECK(fsim_last_command_detail(world, &d) == FSIM_OK && d.index == 4 && d.constraint == 0);
                CHECK(strcmp(fsim_reason_name(d.reason), "not_supported") == 0);
                wild[0] = 4.0; wild[1] = 0.0; wild[2] = fsim_hold(); wild[3] = 0.785; wild[4] = fsim_hold(); wild[5] = fsim_hold();
                fsim_command_options_init(&co);
                co.range = FSIM_RANGE_REJECT;
                CHECK(fsim_vehicle_submit(world, b, FSIM_LEVEL_ATTITUDE, wild, 6, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
                CHECK(fsim_last_command_detail(world, &d) == FSIM_OK && d.index == 0 && cr.reserved == 1);
                CHECK(strcmp(fsim_constraint_name(d.constraint), "max_orientation") == 0);
                CHECK(strcmp(fsim_constraint_name(99), "?") == 0);
                d.struct_size = 2; /* too small to fill */
                CHECK(fsim_last_command_detail(world, &d) != FSIM_OK);
            }
        }

        /* an operator's override: the policy's activity ends preempted, a policy command is refused */
        co.source = FSIM_SOURCE_OVERRIDE;
        CHECK(fsim_vehicle_submit(world, a, FSIM_LEVEL_VELOCITY, climb, 4, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
        operator_id = cr.activity;
        CHECK(fsim_activity_get(world, acts[0], &ai) == FSIM_OK && ai.state == FSIM_ACTIVITY_CANCELED && ai.by == operator_id);
        CHECK(strcmp(fsim_reason_name(ai.reason), "preempted") == 0);
        CHECK(fsim_activity_update(world, acts[0], cruise, 4, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
        CHECK(strcmp(fsim_reason_name(cr.reason), "activity_ended") == 0);
        CHECK(fsim_vehicle_command_attitude(world, a, &att) != FSIM_OK);
        CHECK(strstr(fsim_last_error(), "authority_held") != NULL);
        CHECK(fsim_activity_update_batch(world, acts, 2, rows, 0) != FSIM_OK); /* acts[0] has ended */
        CHECK(fsim_activity_cancel(world, operator_id, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_CANCELED);
        CHECK(fsim_vehicle_activity_count(world, a) >= 2);
        CHECK(fsim_vehicle_activity(world, a, 0, &ai) == FSIM_OK && ai.vehicle == a);
        CHECK(fsim_activity_get(world, ((fsim_activity_id)a << 32) | 999u, &ai) != FSIM_OK);
        CHECK(fsim_vehicle_command_attitude(world, a, &att) == FSIM_OK); /* nothing holds its axes now */
        {
            /* support effectors: flaps set beside the flight activity; no speedbrake on a c172x */
            double position = 0.4, two[2] = {0.4, 0.4};
            fsim_activity_id flaps_id;
            fsim_command_options_init(&co);
            CHECK(fsim_vehicle_submit_support(world, a, FSIM_SUPPORT_FLAPS, &position, 1, &co, &cr) == FSIM_OK);
            CHECK(cr.status == FSIM_COMMAND_ACCEPTED);
            flaps_id = cr.activity;
            CHECK(fsim_vehicle_submit_support(world, a, FSIM_SUPPORT_FLAPS, two, 2, &co, &cr) != FSIM_OK); /* malformed */
            position = 0.2;
            CHECK(fsim_activity_update(world, flaps_id, &position, 1, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_update(world, flaps_id, two, 2, &cr) != FSIM_OK); /* flaps take one field */
            CHECK(fsim_vehicle_submit_support(world, a, FSIM_SUPPORT_SPEEDBRAKE, &position, 1, &co, &cr) == FSIM_OK);
            CHECK(cr.status == FSIM_COMMAND_REJECTED && strcmp(fsim_reason_name(cr.reason), "not_implemented") == 0); /* a stock c172x: declares nothing */
            CHECK(fsim_activity_cancel(world, flaps_id, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_CANCELED);
        }
        {
            /* axes owned apart: an autopilot's height and speed beside a policy's bank */
            double hold[4], bank[6];
            fsim_activity_id held;
            int32_t mode = -1, reason = -1;
            hold[0] = 55.0; hold[1] = 0.0; hold[2] = fsim_hold(); hold[3] = fsim_hold();
            bank[0] = 0.3; bank[1] = fsim_hold(); bank[2] = fsim_hold(); bank[3] = 0.785; bank[4] = fsim_hold(); bank[5] = fsim_hold();
            fsim_command_options_init(&co);
            co.source = FSIM_SOURCE_AUTOPILOT;
            co.axes = 2u | 8u; /* pitch, thrust */
            CHECK(fsim_vehicle_submit(world, b, FSIM_LEVEL_VELOCITY, hold, 4, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            held = cr.activity;
            co.source = FSIM_SOURCE_POLICY;
            co.axes = 1u; /* roll: yaw goes with it above the actuators */
            CHECK(fsim_vehicle_submit(world, b, FSIM_LEVEL_ATTITUDE, bank, 6, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_get(world, cr.activity, &ai) == FSIM_OK && ai.axes == (1u | 4u));
            CHECK(fsim_activity_get(world, held, &ai) == FSIM_OK && ai.state == FSIM_ACTIVITY_PENDING); /* not preempted */
            co.axes = 16u; /* flaps alone: no primary axis to fly */
            CHECK(fsim_vehicle_submit(world, b, FSIM_LEVEL_ATTITUDE, bank, 6, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "invalid_axes") == 0);
            /* the vehicle default */
            CHECK(fsim_vehicle_get_default(world, a, &mode) == FSIM_OK && mode == FSIM_DEFAULT_NEUTRAL);
            CHECK(fsim_vehicle_set_default(world, a, FSIM_DEFAULT_HOLD, &reason) == FSIM_OK && reason == 0);
            CHECK(fsim_vehicle_get_default(world, a, &mode) == FSIM_OK && mode == FSIM_DEFAULT_HOLD);
            CHECK(fsim_vehicle_set_default(world, a, 7, &reason) != FSIM_OK);
            CHECK(fsim_vehicle_set_default(world, 999, FSIM_DEFAULT_HOLD, &reason) != FSIM_OK);
            CHECK(strcmp(fsim_reason_name(reason), "unknown_vehicle") == 0);
            CHECK(fsim_vehicle_get_default(world, 999, &mode) != FSIM_OK);
            CHECK(fsim_world_step(world, 1) == FSIM_OK);
            CHECK(fsim_activity_get(world, held, &ai) == FSIM_OK && ai.state == FSIM_ACTIVITY_ACTIVE);
        }
        {
            /* the engines' throttles, one per engine, where there is more than one */
            uint32_t twin = 0;
            double throttles[2] = {0.9, 0.4}, five[5] = {0.5, 0.5, 0.5, 0.5, 0.5}, value = -1.0;
            fsim_activity_id engines;
            int32_t protection = -1;
            fsim_envelope_status envelope;
            spec.name = "cap-twin";
            spec.type = "jsbsim:a10c";
            spec.altitude_msl_m = 3000.0;
            spec.airspeed_ms = 140.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &twin) == FSIM_OK);
            fsim_command_options_init(&co);
            CHECK(fsim_vehicle_submit_support(world, twin, FSIM_SUPPORT_ENGINES, throttles, 2, &co, &cr) == FSIM_OK);
            CHECK(cr.status == FSIM_COMMAND_ACCEPTED);
            engines = cr.activity;
            CHECK(fsim_world_step(world, 1) == FSIM_OK);
            throttles[0] = 0.7;
            throttles[1] = fsim_hold(); /* keeps what it flew: 0.4 */
            CHECK(fsim_activity_update(world, engines, throttles, 2, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_vehicle_submit_support(world, twin, FSIM_SUPPORT_ENGINES, five, 5, &co, &cr) != FSIM_OK); /* at most 4 */
            CHECK(fsim_vehicle_submit_support(world, a, FSIM_SUPPORT_ENGINES, throttles, 2, &co, &cr) == FSIM_OK);
            CHECK(cr.status == FSIM_COMMAND_REJECTED && strcmp(fsim_reason_name(cr.reason), "not_implemented") == 0); /* its profile says nothing of its engines */
            CHECK(fsim_world_step(world, 1) == FSIM_OK);
            CHECK(fsim_vehicle_get_property(world, twin, "fcs/throttle-cmd-norm[0]", &value) == FSIM_OK && value == 0.7);
            CHECK(fsim_vehicle_get_property(world, twin, "fcs/throttle-cmd-norm[1]", &value) == FSIM_OK && value == 0.4);
            /* envelope protection: on for a design with an envelope, off for a stock aircraft */
            CHECK(fsim_vehicle_get_protection(world, twin, &protection) == FSIM_OK && protection == FSIM_PROTECTION_LIMIT);
            CHECK(fsim_vehicle_get_protection(world, a, &protection) == FSIM_OK && protection == FSIM_PROTECTION_OFF);
            CHECK(fsim_vehicle_set_protection(world, twin, FSIM_PROTECTION_REPORT) == FSIM_OK);
            CHECK(fsim_vehicle_set_protection(world, twin, 9) != FSIM_OK);
            CHECK(fsim_vehicle_set_protection(world, 999, FSIM_PROTECTION_OFF) != FSIM_OK);
            CHECK(fsim_world_step(world, 1) == FSIM_OK);
            CHECK(fsim_vehicle_envelope(world, twin, &envelope) == FSIM_OK && envelope.mode == FSIM_PROTECTION_REPORT);
            CHECK(envelope.exceeded_updates[2] == 0 && envelope.worst_excess[2] == 0.0); /* level flight: well inside */
            CHECK(fsim_vehicle_envelope(world, 999, &envelope) != FSIM_OK);
            CHECK(strcmp(fsim_limit_name(2), "alpha_max") == 0 && strcmp(fsim_limit_name(FSIM_LIMIT_COUNT), "?") == 0);
        }
        {
            /* ABI 1.6: a route's progress, the commanded state, the A-GRA types */
            fsim_behavior_command route;
            fsim_position_command points[2];
            fsim_activity_progress progress;
            fsim_commanded_state commanded;
            const fsim_vehicle_state* s = fsim_vehicle_state_ptr(world, b);
            uint32_t k, formation = 0;
            memset(&route, 0, sizeof route);
            memset(points, 0, sizeof points);
            points[0].latitude_rad = s->latitude_rad + 0.0005;
            points[0].longitude_rad = s->longitude_rad;
            points[0].altitude_msl_m = 1500.0;
            points[0].airspeed_ms = fsim_hold();
            points[0].capture_radius_m = 300.0;
            points[1] = points[0];
            points[1].latitude_rad += 0.0005;
            route.id = "waypoints";
            route.points = points;
            route.point_count = 2;
            fsim_command_options_init(&co);
            co.source = FSIM_SOURCE_OVERRIDE; /* an autopilot holds b's height and speed (above) */
            CHECK(fsim_vehicle_submit_behavior(world, b, &route, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_world_step(world, 5) == FSIM_OK);
            fsim_activity_progress_init(&progress);
            CHECK(progress.struct_size == sizeof progress && isnan(progress.percent));
            CHECK(fsim_activity_get_progress(world, cr.activity, &progress) == FSIM_OK);
            CHECK(progress.segment == 0 && progress.segments == 2);
            CHECK(progress.percent >= 0.0 && progress.percent < 100.0 && progress.distance_to_go_m > 1000.0);
            CHECK(progress.altitude_msl_m == 1500.0 && progress.speed_reference == 0.0);
            CHECK(fsim_activity_get_progress(world, ((fsim_activity_id)b << 32) | 999u, &progress) != FSIM_OK);
            fsim_commanded_state_init(&commanded);
            CHECK(fsim_vehicle_commanded(world, b, &commanded) == FSIM_OK);
            CHECK(commanded.top_level == FSIM_LEVEL_BEHAVIOR && commanded.altitude_msl_m == 1500.0 && !isnan(commanded.heading_rad));
            CHECK(fsim_vehicle_commanded(world, 999, &commanded) != FSIM_OK);
            for (k = 0; k < ncap; ++k)
                if (fsim_vehicle_capability(world, b, k, &ci) == FSIM_OK && strcmp(ci.id, "fsim.guidance.formation") == 0) formation = k;
            CHECK(fsim_vehicle_capability_flight_mode(world, b, formation) == FSIM_FLIGHT_MODE_FORMATION);
            CHECK(fsim_vehicle_capability_flight_mode(world, b, attitude) == FSIM_FLIGHT_MODE_NONE);
            CHECK(fsim_vehicle_capability_flight_mode(world, b, 9999) == -1);
            CHECK(strcmp(fsim_flight_mode_name(FSIM_FLIGHT_MODE_HSA_CSA), "hsa_csa") == 0);
            CHECK(strcmp(fsim_flight_mode_name(-1), "?") == 0);
            CHECK(strcmp(fsim_reason_name(21), "invalid_waypoint") == 0);
        }
        {
            /* ABI 1.6: the HSA/CSA mode - a fixed-size setpoint in fields, a partial UPDATE keeping the rest */
            double hsa[6], alt[6], bad[5];
            fsim_activity_progress progress;
            fsim_activity_id mode_id;
            CHECK(fsim_mode_field_count(FSIM_MODE_HSA) == 8 && fsim_mode_field_count(99) == 0); /* (1.15: six leave the optimisation out; 1.20: seven the direction reference) */
            hsa[0] = 3.0; hsa[1] = fsim_hold(); hsa[2] = 50.0; hsa[3] = FSIM_SPEED_TRUE_AIRSPEED; hsa[4] = 1600.0; hsa[5] = FSIM_ALTITUDE_MSL;
            fsim_command_options_init(&co);
            co.source = FSIM_SOURCE_OVERRIDE; /* b's autopilot holds its height (above) */
            CHECK(fsim_vehicle_submit_mode(world, b, FSIM_MODE_HSA, hsa, 6, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            mode_id = cr.activity;
            CHECK(fsim_vehicle_submit_mode(world, b, FSIM_MODE_HSA, bad, 5, &co, &cr) != FSIM_OK); /* malformed: 6 fields */
            alt[0] = alt[1] = alt[2] = alt[3] = alt[5] = fsim_hold();
            alt[4] = 1700.0;
            CHECK(fsim_activity_update(world, mode_id, alt, 6, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_update(world, mode_id, alt, 4, &cr) != FSIM_OK); /* malformed */
            CHECK(fsim_world_step(world, 1) == FSIM_OK);
            fsim_activity_progress_init(&progress);
            CHECK(fsim_activity_get_progress(world, mode_id, &progress) == FSIM_OK);
            CHECK(progress.altitude_msl_m == 1700.0 && progress.speed_ms == 50.0 && fabs(progress.heading_rad - 3.0) < 1e-9);
            alt[4] = fsim_hold();
            alt[3] = FSIM_SPEED_MACH; /* a reference without its value: an UPDATE has nothing to take it from */
            CHECK(fsim_activity_update(world, mode_id, alt, 6, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "invalid_parameter") == 0 && cr.reserved == 3);
            CHECK(fsim_activity_cancel(world, mode_id, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_CANCELED);
        }
        {
            /* ABI 1.6: a route - its options in fields, its waypoints beside them; an UPDATE with new ones or none */
            double options[4], later[4];
            fsim_waypoint way[3];
            double older[3 * (offsetof(fsim_waypoint, id) / sizeof(double))]; /* an older header's waypoints: no id */
            fsim_activity_progress progress;
            fsim_command_detail detail;
            fsim_activity_id route_id;
            const fsim_vehicle_state* s = fsim_vehicle_state_ptr(world, b);
            int i;
            CHECK(fsim_mode_field_count(FSIM_MODE_ROUTE) == 4);
            options[0] = FSIM_PROJECTION_GREAT_CIRCLE; options[1] = 0.0; options[2] = FSIM_END_CONTINUE; options[3] = 0.0;
            for (i = 0; i < 3; ++i) {
                fsim_waypoint_init(&way[i]);
                way[i].latitude_rad = s->latitude_rad + 0.00035 * (i + 1); /* 2.2 km apart, north */
                way[i].longitude_rad = s->longitude_rad + (i == 1 ? 0.0005 : 0.0);
                way[i].id = 7 + (uint64_t)i;
            }
            CHECK(way[0].struct_size == sizeof way[0] && isnan(way[0].speed) && way[0].turn == FSIM_TURN_FLY_BY);
            way[0].altitude_m = 1550.0;
            fsim_command_options_init(&co);
            co.source = FSIM_SOURCE_OVERRIDE; /* b's autopilot holds its height (above) */
            CHECK(fsim_vehicle_submit_route(world, b, options, 4, way, 3, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            route_id = cr.activity;
            CHECK(fsim_vehicle_submit_route(world, b, options, 3, way, 3, &co, &cr) != FSIM_OK); /* malformed: 4 fields */
            CHECK(fsim_world_step(world, 2) == FSIM_OK);
            fsim_activity_progress_init(&progress);
            CHECK(fsim_activity_get_progress(world, route_id, &progress) == FSIM_OK);
            CHECK(progress.segment == 0 && progress.segments == 3 && progress.segment_id == 7 && progress.altitude_msl_m < 1550.0);
            /* a waypoint it cannot fly: refused, naming it; its reserved is the index + 1 */
            way[2].latitude_rad = NAN;
            CHECK(fsim_activity_update_route(world, route_id, options, 4, way, 3, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "invalid_waypoint") == 0 && cr.reserved == 3);
            fsim_command_detail_init(&detail);
            CHECK(fsim_last_command_detail(world, &detail) == FSIM_OK && detail.index == 2);
            /* new waypoints laid out by an older header, its struct_size apart (without the id): the rest as fsim_waypoint_init leaves it */
            way[2].latitude_rad = s->latitude_rad + 0.00105;
            for (i = 0; i < 3; ++i) {
                way[i].struct_size = (uint32_t)offsetof(fsim_waypoint, id);
                memcpy((char*)older + (size_t)i * offsetof(fsim_waypoint, id), &way[i], offsetof(fsim_waypoint, id));
            }
            CHECK(fsim_activity_update_route(world, route_id, options, 4, (const fsim_waypoint*)(const void*)older, 3, &cr) == FSIM_OK &&
                  cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_world_step(world, 1) == FSIM_OK && fsim_activity_get_progress(world, route_id, &progress) == FSIM_OK);
            CHECK(progress.segment_id == 0); /* (the ids were beyond the size given) */
            /* its options alone keep its waypoints: flown again from the point it starts at */
            later[0] = later[1] = later[2] = fsim_hold(); later[3] = 2.0;
            CHECK(fsim_activity_update(world, route_id, later, 4, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_world_step(world, 1) == FSIM_OK && fsim_activity_get_progress(world, route_id, &progress) == FSIM_OK);
            CHECK(progress.segment == 2 && progress.segments == 3);
            /* a route through the modes' call has no waypoints */
            CHECK(fsim_vehicle_submit_mode(world, b, FSIM_MODE_ROUTE, options, 4, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "invalid_waypoint") == 0 && cr.reserved == 1);
            CHECK(fsim_activity_cancel(world, route_id, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_CANCELED);
        }
        {
            /* ABI 1.6: a loiter pattern - twelve fields, fsim_hold() taking the defaults (an orbit here), an UPDATE merging */
            double pattern[12], wider[12];
            fsim_activity_progress progress;
            fsim_activity_id orbit_id;
            int k;
            CHECK(fsim_mode_field_count(FSIM_MODE_PATTERN) == 25); /* (1.15: twelve leave the optimisation out; 1.21: 13 A-GRA's other ways) */
            for (k = 0; k < 12; ++k) pattern[k] = wider[k] = fsim_hold();
            pattern[0] = FSIM_PATTERN_ORBIT;
            pattern[5] = 800.0; /* radius_m */
            fsim_command_options_init(&co);
            co.source = FSIM_SOURCE_OVERRIDE; /* b's autopilot holds its height (above) */
            CHECK(fsim_vehicle_submit_mode(world, b, FSIM_MODE_PATTERN, pattern, 12, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            orbit_id = cr.activity;
            CHECK(fsim_vehicle_submit_mode(world, b, FSIM_MODE_PATTERN, pattern, 11, &co, &cr) != FSIM_OK); /* malformed: 12 fields */
            fsim_activity_progress_init(&progress);
            CHECK(fsim_world_step(world, 2) == FSIM_OK && fsim_activity_get_progress(world, orbit_id, &progress) == FSIM_OK);
            CHECK(progress.segments == 1 && progress.segment == 0 && progress.speed_ms > 20.0); /* the airspeed it flies */
            wider[5] = 1200.0; /* only the radius */
            CHECK(fsim_activity_update(world, orbit_id, wider, 12, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            wider[5] = fsim_hold();
            wider[6] = 0.5; /* clockwise: 0 or 1 */
            CHECK(fsim_activity_update(world, orbit_id, wider, 12, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED && cr.reserved == 7);
            CHECK(fsim_activity_cancel(world, orbit_id, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_CANCELED);
        }
        {
            /* ABI 1.6: a curve - its options in fields, its Bezier segments beside them; appended to by UPDATE */
            double fields[8], later[8];
            fsim_bezier_segment seg[2], more, hairpin;
            fsim_activity_progress progress;
            fsim_command_detail detail;
            fsim_activity_id curve_id;
            int k;
            CHECK(fsim_mode_field_count(FSIM_MODE_CURVE) == 8);
            for (k = 0; k < 8; ++k) fields[k] = later[k] = fsim_hold();
            fsim_bezier_segment_init(&seg[0]);
            fsim_bezier_segment_init(&seg[1]);
            fsim_bezier_segment_init(&more);
            fsim_bezier_segment_init(&hairpin);
            CHECK(seg[0].struct_size == sizeof seg[0] && seg[0].north[3] == 0.0 && seg[0].down[5] == 0.0);
            for (k = 0; k < 6; ++k) { /* north, 2 km a segment, from where the aircraft is (the reference, left out) */
                seg[0].north[k] = 400.0 * k;
                seg[1].north[k] = 2000.0 + 400.0 * k;
                more.north[k] = 4000.0 + 400.0 * k;
            }
            fsim_command_options_init(&co);
            co.source = FSIM_SOURCE_OVERRIDE; /* b's autopilot holds its height (above) */
            CHECK(fsim_vehicle_submit_curve(world, b, fields, 8, seg, 2, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            curve_id = cr.activity;
            CHECK(fsim_vehicle_submit_curve(world, b, fields, 7, seg, 2, &co, &cr) != FSIM_OK); /* malformed: 8 fields */
            fsim_activity_progress_init(&progress);
            CHECK(fsim_world_step(world, 2) == FSIM_OK && fsim_activity_get_progress(world, curve_id, &progress) == FSIM_OK);
            CHECK(progress.segment == 0 && progress.segments == 2 && progress.distance_to_go_m > 3000.0);
            /* appended after its end, from the same reference */
            later[7] = 1.0; /* append */
            CHECK(fsim_activity_update_curve(world, curve_id, later, 8, &more, 1, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_world_step(world, 1) == FSIM_OK && fsim_activity_get_progress(world, curve_id, &progress) == FSIM_OK);
            CHECK(progress.segments == 3);
            /* where the curve does not end: refused, naming the segment (reserved: its index + 1) */
            more.north[0] += 5.0;
            CHECK(fsim_activity_update_curve(world, curve_id, later, 8, &more, 1, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "invalid_curve") == 0 && cr.reserved == 1);
            /* a hairpin tighter than it can turn: the segment, and the section of it */
            for (k = 0; k < 6; ++k) {
                hairpin.north[k] = k < 3 ? 30.0 * k : 30.0 * (5 - k);
                hairpin.east[k] = k < 3 ? 0.0 : 20.0;
            }
            CHECK(fsim_vehicle_submit_curve(world, b, fields, 8, &hairpin, 1, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            fsim_command_detail_init(&detail);
            CHECK(fsim_last_command_detail(world, &detail) == FSIM_OK && detail.index == 0);
            CHECK(strcmp(fsim_constraint_name(detail.constraint), "max_turn_rate") == 0);
            CHECK(detail.from > 0.0 && detail.from < detail.to && detail.to < 1.0);
            /* its options alone: how it is flown, not where */
            later[7] = fsim_hold();
            later[4] = 50.0; /* speed_max_ms */
            CHECK(fsim_activity_update(world, curve_id, later, 8, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_world_step(world, 1) == FSIM_OK && fsim_activity_get_progress(world, curve_id, &progress) == FSIM_OK);
            CHECK(progress.segments == 3);
            CHECK(fsim_activity_cancel(world, curve_id, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_CANCELED);
        }
        {
            /* ABI 1.6: the performance, and grants over the priorities (on a, whose live activities are its policy's) */
            fsim_performance perf;
            int32_t mode = -1, reason = -1, allowed = -1, granted = -1, availability = -1;
            uint32_t rev0 = 0, rev1 = 0;
            double hsa[6], bank0 = 0.0;
            fsim_activity_id flown;
            int k;
            fsim_performance_init(&perf);
            CHECK(perf.struct_size == sizeof perf);
            CHECK(fsim_vehicle_performance(world, a, &perf) == FSIM_OK && perf.hovers == 0 && perf.max_bank_rad > 0.0);
            CHECK(fsim_vehicle_performance(world, 999, &perf) != FSIM_OK);
            /* a loop's parameter changed: the performance computed afresh, and the revision counts it */
            CHECK(fsim_vehicle_controller_parameter(world, a, FSIM_LEVEL_VELOCITY, "max_bank", &bank0) == FSIM_OK);
            CHECK(fsim_vehicle_control_revision(world, a, &rev0) == FSIM_OK);
            CHECK(fsim_vehicle_set_controller_parameter(world, a, FSIM_LEVEL_VELOCITY, "max_bank", 0.45) == FSIM_OK);
            CHECK(fsim_vehicle_performance(world, a, &perf) == FSIM_OK && perf.max_bank_rad == 0.45);
            CHECK(fsim_vehicle_control_revision(world, a, &rev1) == FSIM_OK && rev1 == rev0 + 1);
            CHECK(fsim_vehicle_set_controller_parameter(world, a, FSIM_LEVEL_VELOCITY, "max_bank", bank0) == FSIM_OK);
            /* Granted: a policy's NEW needs a grant; the gate is the grant, and nothing more */
            CHECK(fsim_vehicle_control_mode(world, a, &mode) == FSIM_OK && mode == FSIM_CONTROL_OPEN);
            CHECK(fsim_vehicle_set_control_mode(world, a, FSIM_CONTROL_GRANTED) == FSIM_OK);
            CHECK(fsim_vehicle_control_mode(world, a, &mode) == FSIM_OK && mode == FSIM_CONTROL_GRANTED);
            CHECK(fsim_vehicle_set_control_mode(world, a, 5) != FSIM_OK);
            for (k = 0; k < 6; ++k) hsa[k] = fsim_hold();
            hsa[0] = 1.0; /* heading_rad */
            fsim_command_options_init(&co);
            CHECK(fsim_vehicle_submit_mode(world, a, FSIM_MODE_HSA, hsa, 6, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "not_granted") == 0);
            CHECK(fsim_vehicle_command_attitude(world, a, &att) != FSIM_OK); /* the existing entry points too */
            CHECK(fsim_vehicle_request_control(world, a, "fsim.guidance.hsa", &reason) == FSIM_OK && reason == 0);
            CHECK(fsim_vehicle_control_status(world, a, "fsim.guidance.hsa", &allowed, &granted) == FSIM_OK && allowed == 1 && granted == 1);
            CHECK(fsim_vehicle_submit_mode(world, a, FSIM_MODE_HSA, hsa, 6, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            flown = cr.activity;
            /* released: the policy lets go, its activity ends */
            CHECK(fsim_vehicle_release_control(world, a, "fsim.guidance.hsa") == FSIM_OK);
            CHECK(fsim_activity_get(world, flown, &ai) == FSIM_OK && ai.state == FSIM_ACTIVITY_CANCELED && strcmp(fsim_reason_name(ai.reason), "released") == 0);
            CHECK(fsim_vehicle_control_status(world, a, "fsim.guidance.hsa", &allowed, &granted) == FSIM_OK && granted == 0);
            /* restricted by the platform (0: "restricted"): requests and a policy's NEW refused with it */
            CHECK(fsim_vehicle_set_availability(world, a, "fsim.guidance.hsa", FSIM_TEMPORARILY_UNAVAILABLE, 0) == FSIM_OK);
            CHECK(fsim_vehicle_capability_status(world, a, "fsim.guidance.hsa", &availability, &reason) == FSIM_OK);
            CHECK(availability == FSIM_TEMPORARILY_UNAVAILABLE && strcmp(fsim_reason_name(reason), "restricted") == 0);
            CHECK(fsim_vehicle_request_control(world, a, "fsim.guidance.hsa", &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "restricted") == 0);
            CHECK(fsim_vehicle_set_availability(world, a, "fsim.guidance.hsa", FSIM_AVAILABLE, 0) == FSIM_OK);
            /* not allowed: refused; revoked with a reason: the grant gone */
            CHECK(fsim_vehicle_set_allowed(world, a, "fsim.guidance.hsa", 0) == FSIM_OK);
            CHECK(fsim_vehicle_request_control(world, a, "fsim.guidance.hsa", &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "not_allowed") == 0);
            CHECK(fsim_vehicle_set_allowed(world, a, "fsim.guidance.hsa", 1) == FSIM_OK);
            CHECK(fsim_vehicle_request_control(world, a, "fsim.guidance.hsa", &reason) == FSIM_OK && reason == 0);
            CHECK(fsim_vehicle_revoke_control(world, a, "fsim.guidance.hsa", 0) == FSIM_OK);
            CHECK(fsim_vehicle_control_status(world, a, "fsim.guidance.hsa", &allowed, &granted) == FSIM_OK && granted == 0);
            CHECK(fsim_vehicle_request_control(world, a, "fsim.guidance.nonsense", &reason) != FSIM_OK);
            CHECK(fsim_vehicle_revoke_control(world, a, "fsim.guidance.hsa", 999) != FSIM_OK);
            /* only the platform's reasons: one that would misreport an end is refused */
            for (k = 0; k < 64 && strcmp(fsim_reason_name(k), "preempted") != 0; ++k) {}
            CHECK(k < 64 && fsim_vehicle_revoke_control(world, a, "fsim.guidance.hsa", k) != FSIM_OK);
            CHECK(fsim_vehicle_set_availability(world, a, "fsim.guidance.hsa", FSIM_FAULTED, k) != FSIM_OK);
            /* UPDATE and CANCEL declare the caller's source: a policy cannot change or end the platform's */
            fsim_command_options_init(&co);
            co.source = FSIM_SOURCE_OVERRIDE;
            CHECK(fsim_vehicle_submit_mode(world, a, FSIM_MODE_HSA, hsa, 6, &co, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            flown = cr.activity;
            CHECK(fsim_activity_update(world, flown, hsa, 6, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "authority_held") == 0 && cr.other == flown);
            CHECK(fsim_activity_cancel(world, flown, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED && cr.other == flown);
            CHECK(fsim_activity_update_as(world, flown, FSIM_SOURCE_OVERRIDE, hsa, 6, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_cancel_as(world, flown, 7, &cr) != FSIM_OK); /* no such source */
            CHECK(fsim_activity_cancel_as(world, flown, FSIM_SOURCE_OVERRIDE, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_CANCELED);
            /* back to Open: as ever */
            CHECK(fsim_vehicle_set_control_mode(world, a, FSIM_CONTROL_OPEN) == FSIM_OK);
            CHECK(fsim_vehicle_command_attitude(world, a, &att) == FSIM_OK);
        }
        {
            /* the profile: a stock c172x carries no sections */
            double value = 0.0;
            uint32_t version = 9;
            int32_t provenance = -1;
            CHECK(fsim_vehicle_profile_value(world, a, "envelope/clean/n_max", &value) == FSIM_OK && isnan(value));
            CHECK(fsim_vehicle_profile_section(world, a, "control", &version, &provenance) == FSIM_OK);
            CHECK(version == 0 && provenance == 0);
            CHECK(fsim_vehicle_profile_section(world, a, "nonsense", &version, &provenance) != FSIM_OK);
            CHECK(fsim_vehicle_profile_value(world, 999, "identity/class", &value) != FSIM_OK);
        }
        {
            /* ABI 1.7: support and availability (docs/flight-autonomy.md, 4) */
            fsim_support_info si;
            fsim_capability_status cs;
            fsim_parameter_limit limits[4];
            fsim_command_result cr;
            fsim_behavior_command hover;
            uint32_t viper = 0, count = 99, n, c, found = 0;
            const uint32_t features = fsim_support_feature_count();
            CHECK(features > 100 && fsim_support_feature(0) != NULL && fsim_support_feature(features) == NULL);
            spec.name = "cap-viper";
            spec.type = "jsbsim:f16c";
            spec.altitude_msl_m = 3000.0;
            spec.airspeed_ms = 160.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &viper) == FSIM_OK);
            /* a physical exception: not supported, with its rule and the aircraft's evidence */
            fsim_support_info_init(&si);
            CHECK(si.struct_size == sizeof si);
            CHECK(fsim_vehicle_support(world, viper, "fsim.guidance.hover", &si) == FSIM_OK);
            CHECK(si.support == FSIM_NOT_SUPPORTED && strcmp(fsim_support_name(si.support), "not_supported") == 0);
            CHECK(si.rules == 2u && strcmp(fsim_rule_name(1), "R1") == 0 && strlen(fsim_rule_description(1)) > 0);
            CHECK(strstr(si.evidence, "vertical_flight = false") == si.evidence);
            memset(&hover, 0, sizeof hover);
            hover.id = "hover";
            CHECK(fsim_vehicle_submit_behavior(world, viper, &hover, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "not_supported") == 0);
            /* applicable, not built: the stage that builds it */
            CHECK(fsim_vehicle_support(world, viper, "fsim.guidance.route/altitude/barometric", &si) == FSIM_OK);
            CHECK(si.support == FSIM_NOT_IMPLEMENTED && si.stage == 6);
            CHECK(fsim_vehicle_support(world, viper, "fsim.guidance.route", &si) == FSIM_OK && si.support == FSIM_PARTIAL && strlen(si.missing) > 0);
            CHECK(fsim_vehicle_support(world, viper, "fsim.guidance.warp_drive", &si) != FSIM_OK);
            /* the status as a policy is answered: what it does not offer is unavailable, with why */
            fsim_capability_status_init(&cs);
            CHECK(cs.struct_size == sizeof cs && isnan(cs.next_available_s));
            CHECK(fsim_vehicle_capability_status_info(world, viper, "fsim.guidance.hover", &cs) == FSIM_OK);
            CHECK(cs.availability == FSIM_UNAVAILABLE && strcmp(fsim_availability_name(cs.availability), "unavailable") == 0);
            CHECK(strcmp(fsim_reason_name(cs.reason), "not_supported") == 0 && cs.reasons == (1ull << cs.reason));
            CHECK(strlen(fsim_reason_description(cs.reason)) > 0);
            CHECK(fsim_vehicle_set_availability_ex(world, viper, "fsim.flight.velocity", FSIM_TEMPORARILY_UNAVAILABLE, 0, 7, 30.0) == FSIM_OK);
            CHECK(fsim_vehicle_capability_status_info(world, viper, "fsim.flight.velocity", &cs) == FSIM_OK);
            CHECK(strcmp(fsim_reason_name(cs.reason), "restricted") == 0 && cs.associated == 7 && cs.next_available_s == 30.0);
            CHECK(fsim_vehicle_capability_limits(world, viper, "fsim.support.gear", limits, 4, &count) == FSIM_OK && count == 0); /* flying */
            /* how each capability is controlled, and what supersedes a platform behaviour */
            n = fsim_vehicle_capability_count(world, viper);
            for (c = 0; c < n; ++c) {
                fsim_capability_info ci;
                CHECK(fsim_vehicle_capability(world, viper, c, &ci) == FSIM_OK);
                CHECK(fsim_vehicle_capability_accepted(world, viper, c) > 0);
                if (strcmp(ci.id, "fsim.guidance.hold") == 0) {
                    CHECK(strcmp(fsim_vehicle_capability_superseded(world, viper, c), "fsim.guidance.hsa") == 0);
                    ++found;
                }
                if (strcmp(ci.id, "fsim.envelope.protection") == 0) CHECK(fsim_vehicle_capability_accepted(world, viper, c) == FSIM_ACCEPTS_AUTO_MDF);
            }
            CHECK(found == 1);
            CHECK(fsim_vehicle_capability_accepted(world, viper, 9999) == -1 && fsim_vehicle_capability_superseded(world, viper, 9999) == NULL);
            /* a capability it does not offer: the authority calls fail, naming why */
            CHECK(fsim_vehicle_release_control(world, viper, "fsim.guidance.hover") != FSIM_OK);
            CHECK(strstr(fsim_last_error(), "not_supported") != NULL);
        }
        {
            /* ABI 1.8: the command envelope (docs/flight-autonomy.md, 4.8) */
            fsim_command_options o, reject;
            fsim_command_result cr, results[3];
            fsim_command_detail d, details[3];
            fsim_command_finding f;
            fsim_command_adjustment adj;
            fsim_activity_envelope ae;
            fsim_batch_command batch[3];
            uint32_t eagle = 0, b;
            const double hold = fsim_hold();
            double hsa[6], gear[1], velocity[4];
            spec.name = "cap-eagle";
            spec.type = "jsbsim:f16c";
            spec.altitude_msl_m = 3000.0;
            spec.airspeed_ms = 160.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &eagle) == FSIM_OK);
            /* its id and requirements: echoed, and kept with its activity */
            fsim_command_options_init(&o);
            CHECK(o.interactive == 1 && o.validate_only == 0 && o.command_id == 0);
            o.command_id = 42;
            o.trace[0].kind = FSIM_REQUIREMENT_TASK;
            o.trace[0].id = 7;
            o.interactive = 0;
            hsa[0] = 0.5, hsa[1] = hold, hsa[2] = hold, hsa[3] = hold, hsa[4] = hold, hsa[5] = hold;
            CHECK(fsim_vehicle_submit_mode(world, eagle, FSIM_MODE_HSA, hsa, 6, &o, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            fsim_command_detail_init(&d);
            CHECK(fsim_last_command_detail(world, &d) == FSIM_OK);
            CHECK(d.new_activity == 1 && d.command_id == 42 && d.finding_count == 0 && strcmp(d.description, "") == 0);
            fsim_activity_envelope_init(&ae);
            CHECK(fsim_activity_get_envelope(world, cr.activity, &ae) == FSIM_OK);
            CHECK(ae.command_id == 42 && ae.interactive == 0 && ae.trace[0].kind == FSIM_REQUIREMENT_TASK && ae.trace[0].id == 7);
            CHECK(ae.trace[1].kind == FSIM_REQUIREMENT_NONE && strcmp(fsim_requirement_kind_name(FSIM_REQUIREMENT_TASK), "task") == 0);
            /* every finding with Reject: too fast and too high, the altitude checked first */
            fsim_command_options_init(&reject);
            reject.range = FSIM_RANGE_REJECT;
            hsa[2] = 600.0, hsa[3] = FSIM_SPEED_TRUE_AIRSPEED, hsa[4] = 25000.0;
            CHECK(fsim_vehicle_submit_mode(world, eagle, FSIM_MODE_HSA, hsa, 6, &reject, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(fsim_last_command_detail(world, &d) == FSIM_OK && d.finding_count == 2 && d.index == 4 && strlen(d.description) > 0);
            fsim_command_finding_init(&f);
            CHECK(fsim_last_command_finding(world, 1, &f) == FSIM_OK && f.index == 2 && strcmp(fsim_constraint_name(f.constraint), "max_airspeed") == 0);
            CHECK(fsim_last_command_finding(world, 2, &f) != FSIM_OK);
            /* validated with Clamp: valid, with the values it would hold; nothing flies */
            fsim_command_options_init(&o);
            o.validate_only = 1;
            CHECK(fsim_vehicle_submit_mode(world, eagle, FSIM_MODE_HSA, hsa, 6, &o, &cr) == FSIM_OK);
            CHECK(cr.status == FSIM_COMMAND_VALID && cr.activity == 0);
            CHECK(fsim_last_command_detail(world, &d) == FSIM_OK && d.adjustment_count == 2 && d.new_activity == 0);
            fsim_command_adjustment_init(&adj);
            CHECK(fsim_last_command_adjustment(world, 1, &adj) == FSIM_OK && adj.index == 2 && adj.field == -1);
            CHECK(adj.requested == 600.0 && adj.adjusted < 450.0);
            /* several NEWs at once, each answered on its own */
            memset(batch, 0, sizeof batch);
            hsa[2] = hold, hsa[3] = hold, hsa[4] = hold;
            gear[0] = 2.0; /* out of its range */
            velocity[0] = 160.0, velocity[1] = 0.0, velocity[2] = 0.5, velocity[3] = hold;
            for (b = 0; b < 3; ++b) batch[b].struct_size = sizeof batch[0];
            batch[0].kind = FSIM_BATCH_MODE, batch[0].code = FSIM_MODE_HSA, batch[0].fields = hsa, batch[0].count = 6;
            batch[1].kind = FSIM_BATCH_SUPPORT, batch[1].code = FSIM_SUPPORT_GEAR, batch[1].fields = gear, batch[1].count = 1, batch[1].options = &reject;
            batch[2].kind = FSIM_BATCH_LEVEL, batch[2].code = FSIM_LEVEL_VELOCITY, batch[2].fields = velocity, batch[2].count = 4;
            fsim_command_options_init(&o);
            o.command_id = 3;
            batch[2].options = &o;
            for (b = 0; b < 3; ++b) fsim_command_detail_init(&details[b]);
            CHECK(fsim_vehicle_submit_batch(world, eagle, batch, 3, results, details) == FSIM_OK);
            CHECK(results[0].status == FSIM_COMMAND_ACCEPTED && results[2].status == FSIM_COMMAND_ACCEPTED);
            CHECK(results[1].status == FSIM_COMMAND_REJECTED && strcmp(fsim_reason_name(results[1].reason), "out_of_range") == 0);
            CHECK(details[1].finding_count == 1 && details[1].index == 0 && details[0].new_activity == 1 && details[2].command_id == 3);
            CHECK(fsim_vehicle_submit_batch(world, eagle, batch, 3, results, NULL) == FSIM_OK); /* (the details are optional) */
            batch[2].count = 1; /* malformed: none is made */
            CHECK(fsim_vehicle_submit_batch(world, eagle, batch, 3, results, NULL) != FSIM_OK);
        }
        {
            /* ABI 1.9: ranks, queues and time windows (docs/flight-autonomy.md, 4.9) */
            fsim_command_options o;
            fsim_command_result held, waits, later, cr;
            fsim_activity_envelope ae;
            uint32_t falcon = 0, precedence = 99;
            const double hold = fsim_hold();
            double velocity[4], hsa[6];
            spec.name = "cap-falcon";
            spec.type = "jsbsim:f16c";
            spec.altitude_msl_m = 3000.0;
            spec.airspeed_ms = 160.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &falcon) == FSIM_OK);
            fsim_command_options_init(&o);
            CHECK(o.interrupt == 1 && o.rank_priority == 0 && o.precedence_override == FSIM_NO_PRECEDENCE_OVERRIDE && isnan(o.start_not_before));
            velocity[0] = 160.0, velocity[1] = 0.0, velocity[2] = 1.5, velocity[3] = hold;
            o.rank_priority = 1;
            CHECK(fsim_vehicle_submit(world, falcon, FSIM_LEVEL_VELOCITY, velocity, 4, &o, &held) == FSIM_OK && held.status == FSIM_COMMAND_ACCEPTED);
            CHECK((held.flags & FSIM_COMMAND_DEFERRED) == 0);
            /* ranked behind what flies: it waits, the answer naming what for */
            hsa[0] = 1.0, hsa[1] = hold, hsa[2] = hold, hsa[3] = hold, hsa[4] = hold, hsa[5] = hold;
            o.rank_priority = 5, o.rank_precedence = 2;
            CHECK(fsim_vehicle_submit_mode(world, falcon, FSIM_MODE_HSA, hsa, 6, &o, &waits) == FSIM_OK && waits.status == FSIM_COMMAND_ACCEPTED);
            CHECK((waits.flags & FSIM_COMMAND_DEFERRED) != 0 && waits.other == held.activity);
            fsim_activity_envelope_init(&ae);
            CHECK(fsim_activity_get_envelope(world, waits.activity, &ae) == FSIM_OK);
            CHECK(ae.waiting == FSIM_WAIT_QUEUED && ae.basis == FSIM_BASIS_PLANNED && ae.waiting_for == held.activity);
            CHECK(ae.rank_priority == 5 && ae.rank_precedence == 2 && ae.interrupt == 1 && ae.precedence == 0 && isnan(ae.end_not_after));
            CHECK(strcmp(fsim_activity_wait_name(ae.waiting), "queued") == 0 && strcmp(fsim_activity_basis_name(ae.basis), "planned") == 0);
            /* what it waits for ends: it starts */
            CHECK(fsim_activity_cancel(world, held.activity, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_CANCELED);
            CHECK(fsim_activity_get_envelope(world, waits.activity, &ae) == FSIM_OK && ae.waiting == FSIM_WAIT_NONE && ae.basis == FSIM_BASIS_ACTUAL);
            /* a start window: scheduled; the capability's precedence the platform's */
            fsim_command_options_init(&o);
            o.start_not_before = fsim_world_time(world) + 1.0;
            o.criticality = FSIM_CRITICAL_START;
            CHECK(fsim_vehicle_submit(world, falcon, FSIM_LEVEL_VELOCITY, velocity, 4, &o, &later) == FSIM_OK && (later.flags & FSIM_COMMAND_DEFERRED) != 0);
            CHECK(fsim_activity_get_envelope(world, later.activity, &ae) == FSIM_OK && ae.waiting == FSIM_WAIT_SCHEDULED && ae.criticality == FSIM_CRITICAL_START);
            CHECK(strcmp(fsim_time_criticality_name(FSIM_CRITICAL_START_AND_END), "start_and_end") == 0);
            CHECK(fsim_world_step(world, (uint32_t)(1.1 / fsim_world_step_seconds(world)) + 1) == FSIM_OK);
            CHECK(fsim_activity_get_envelope(world, later.activity, &ae) == FSIM_OK && ae.waiting == FSIM_WAIT_NONE);
            CHECK(fsim_vehicle_set_capability_precedence(world, falcon, "fsim.flight.velocity", 3) == FSIM_OK);
            CHECK(fsim_vehicle_capability_precedence(world, falcon, "fsim.flight.velocity", &precedence) == FSIM_OK && precedence == 3);
            CHECK(fsim_vehicle_set_capability_precedence(world, falcon, "fsim.guidance.hover", 1) != FSIM_OK); /* (not offered: why) */
            /* a policy's precedence override is refused; an end window already closed cannot be met */
            fsim_command_options_init(&o);
            o.precedence_override = 0;
            CHECK(fsim_vehicle_submit(world, falcon, FSIM_LEVEL_VELOCITY, velocity, 4, &o, &cr) == FSIM_OK && strcmp(fsim_reason_name(cr.reason), "not_allowed") == 0);
            fsim_command_options_init(&o);
            o.end_not_after = fsim_world_time(world);
            CHECK(fsim_vehicle_submit(world, falcon, FSIM_LEVEL_VELOCITY, velocity, 4, &o, &cr) == FSIM_OK && strcmp(fsim_reason_name(cr.reason), "time_constraint") == 0);
        }
        {
            /* ABI 1.10: activity commands (docs/flight-autonomy.md, 4.10) */
            fsim_command_options o;
            fsim_command_result flying, cr;
            fsim_activity_info ai;
            uint32_t hawk = 0;
            const double hold = fsim_hold();
            double velocity[4];
            spec.name = "cap-hawk";
            spec.type = "jsbsim:f16c";
            spec.altitude_msl_m = 3000.0;
            spec.airspeed_ms = 160.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &hawk) == FSIM_OK);
            velocity[0] = 160.0, velocity[1] = 0.0, velocity[2] = 1.5, velocity[3] = hold;
            fsim_command_options_init(&o);
            CHECK(fsim_vehicle_submit(world, hawk, FSIM_LEVEL_VELOCITY, velocity, 4, &o, &flying) == FSIM_OK && flying.status == FSIM_COMMAND_ACCEPTED);
            /* disabled: live, kept, flying nothing; enabled again; re-ranked; deleted for good */
            CHECK(fsim_activity_command(world, flying.activity, FSIM_ACTIVITY_DISABLE, 0, 0, FSIM_SOURCE_POLICY, 0, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_get(world, flying.activity, &ai) == FSIM_OK && ai.state == FSIM_ACTIVITY_DISABLED && isnan(ai.end_time));
            CHECK(strcmp(fsim_activity_state_name(FSIM_ACTIVITY_DISABLED), "disabled") == 0);
            CHECK(fsim_activity_command(world, flying.activity, FSIM_ACTIVITY_ENABLE, 0, 0, FSIM_SOURCE_POLICY, 0, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_get(world, flying.activity, &ai) == FSIM_OK && ai.state == FSIM_ACTIVITY_PENDING);
            CHECK(fsim_activity_command(world, flying.activity, FSIM_ACTIVITY_CHANGE_RANK, 2, 1, FSIM_SOURCE_POLICY, 0, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_command(world, flying.activity, FSIM_ACTIVITY_DELETE, 0, 0, FSIM_SOURCE_POLICY, 0, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_get(world, flying.activity, &ai) == FSIM_OK && ai.state == FSIM_ACTIVITY_DELETED && strcmp(fsim_activity_state_name(ai.state), "deleted") == 0);
            CHECK(fsim_activity_command(world, flying.activity, FSIM_ACTIVITY_ENABLE, 0, 0, FSIM_SOURCE_POLICY, 0, &cr) == FSIM_OK && strcmp(fsim_reason_name(cr.reason), "activity_ended") == 0);
            /* one whose command takes none refuses them */
            fsim_command_options_init(&o);
            o.interactive = 0;
            CHECK(fsim_vehicle_submit(world, hawk, FSIM_LEVEL_VELOCITY, velocity, 4, &o, &flying) == FSIM_OK && flying.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_command(world, flying.activity, FSIM_ACTIVITY_RESET, 0, 0, FSIM_SOURCE_POLICY, 0, &cr) == FSIM_OK && strcmp(fsim_reason_name(cr.reason), "not_interactive") == 0);
            CHECK(fsim_activity_command(world, flying.activity, 99, 0, 0, FSIM_SOURCE_POLICY, 0, &cr) != FSIM_OK); /* (no such command) */
            CHECK(strcmp(fsim_activity_command_name(FSIM_ACTIVITY_UNASSIGN), "unassign") == 0);
        }
        {
            /* ABI 1.11: flight tasks and suggestions (docs/flight-autonomy.md, 4.11) */
            fsim_batch_command task;
            fsim_command_options o;
            fsim_command_result cr;
            fsim_command_detail d;
            fsim_task_status ts;
            fsim_activity_envelope ae;
            uint32_t kestrel = 0;
            int32_t reason = -1;
            const double hold = fsim_hold();
            double hsa[6];
            spec.name = "cap-kestrel";
            spec.type = "jsbsim:f16c";
            spec.altitude_msl_m = 3000.0;
            spec.airspeed_ms = 160.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &kestrel) == FSIM_OK);
            hsa[0] = 1.0, hsa[1] = hold, hsa[2] = hold, hsa[3] = hold, hsa[4] = hold, hsa[5] = hold;
            memset(&task, 0, sizeof task);
            task.struct_size = sizeof task;
            task.kind = FSIM_BATCH_MODE, task.code = FSIM_MODE_HSA, task.fields = hsa, task.count = 6;
            CHECK(fsim_vehicle_store_task(world, kestrel, 5, &task, 1, fsim_hold(), &reason) == FSIM_OK && reason == 0);
            CHECK(fsim_vehicle_store_task(world, kestrel, 6, &task, 3, 1.0, &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "invalid_parameter") == 0);
            fsim_task_status_init(&ts);
            CHECK(fsim_vehicle_task_status(world, kestrel, 5, &ts) == FSIM_OK && ts.state == FSIM_TASK_AWAITING_EXECUTION && ts.runs == 1);
            fsim_command_options_init(&o);
            o.command_id = 9;
            CHECK(fsim_vehicle_command_task(world, kestrel, 5, &o, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_vehicle_task_status(world, kestrel, 5, &ts) == FSIM_OK && ts.state == FSIM_TASK_EXECUTION_PENDING && ts.activity == cr.activity);
            CHECK(ts.command_id == 9 && strcmp(fsim_task_state_name(ts.state), "execution_pending") == 0);
            fsim_activity_envelope_init(&ae);
            CHECK(fsim_activity_get_envelope(world, cr.activity, &ae) == FSIM_OK && ae.run == 1 && ae.runs == 1 && ae.trace[0].kind == FSIM_REQUIREMENT_TASK);
            CHECK(fsim_vehicle_command_task(world, kestrel, 5, NULL, &cr) == FSIM_OK && strcmp(fsim_reason_name(cr.reason), "task_active") == 0);
            CHECK(fsim_vehicle_cancel_task(world, kestrel, 5, FSIM_SOURCE_POLICY, 0, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_CANCELED);
            CHECK(fsim_vehicle_task_status(world, kestrel, 5, &ts) == FSIM_OK && ts.state == FSIM_TASK_CANCELED);
            CHECK(fsim_vehicle_task_count(world, kestrel) == 1 && fsim_vehicle_task_at(world, kestrel, 0, &ts) == FSIM_OK && ts.task_id == 5);
            CHECK(fsim_vehicle_remove_task(world, kestrel, 5, &reason) == FSIM_OK && reason == 0);
            CHECK(fsim_vehicle_task_status(world, kestrel, 5, &ts) != FSIM_OK);
            /* refused, what Clamp would fly is suggested as a task */
            fsim_command_options_init(&o);
            o.range = FSIM_RANGE_REJECT;
            hsa[2] = 600.0, hsa[3] = FSIM_SPEED_TRUE_AIRSPEED;
            CHECK(fsim_vehicle_submit_mode(world, kestrel, FSIM_MODE_HSA, hsa, 6, &o, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            fsim_command_detail_init(&d);
            CHECK(fsim_last_command_detail(world, &d) == FSIM_OK && (d.suggestion & FSIM_SUGGESTED_TASK) != 0);
            CHECK(fsim_vehicle_task_status(world, kestrel, d.suggestion, &ts) == FSIM_OK && ts.suggested == 1);
            CHECK(fsim_vehicle_command_task(world, kestrel, d.suggestion, &o, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
        }
        {
            /* ABI 1.12: reports - an activity's setpoint read back, its end points, the commanded state grown
             * (docs/flight-autonomy.md, 4.12) */
            fsim_batch_command sp;
            fsim_command_result cr;
            fsim_end_point points[4];
            fsim_commanded_state cs;
            fsim_waypoint route[3];
            fsim_bezier_segment piece;
            const fsim_vehicle_state* st;
            uint32_t merlin = 0, count = 0, i;
            const double hold = fsim_hold();
            double hsa[6], options[4] = {0.0, 0.0, 0.0, 0.0}, curve[8];
            spec.name = "cap-merlin";
            spec.type = "jsbsim:f16c";
            spec.altitude_msl_m = 3000.0;
            spec.airspeed_ms = 160.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &merlin) == FSIM_OK);
            CHECK(fsim_world_step(world, 5) == FSIM_OK);
            /* an hsa, read back completed; the commanded altitude in its reference */
            hsa[0] = 1.0, hsa[1] = hold, hsa[2] = hold, hsa[3] = hold, hsa[4] = 3200.0, hsa[5] = FSIM_ALTITUDE_MSL;
            CHECK(fsim_vehicle_submit_mode(world, merlin, FSIM_MODE_HSA, hsa, 6, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            memset(&sp, 0, sizeof sp);
            sp.struct_size = sizeof sp;
            CHECK(fsim_activity_get_setpoint(world, cr.activity, &sp) == FSIM_OK && sp.kind == FSIM_BATCH_MODE && sp.code == FSIM_MODE_HSA);
            CHECK(sp.count == 8 && sp.fields[0] == 1.0 && sp.fields[4] == 3200.0 && sp.fields[3] == FSIM_SPEED_TRUE_AIRSPEED && sp.options == NULL);
            CHECK(isnan(sp.fields[6]));                         /* (no speed optimisation) */
            CHECK(sp.fields[7] == FSIM_DIRECTION_TRUE_NORTH); /* (1.20: its heading from true north, as it was given none) */
            CHECK(fsim_activity_end_points(world, cr.activity, points, 0, &count) == FSIM_OK && count == 0);
            CHECK(fsim_world_step(world, 2) == FSIM_OK);
            fsim_commanded_state_init(&cs);
            CHECK(isnan(cs.north_acceleration_ms2) && isnan(cs.altitude_m));
            CHECK(fsim_vehicle_commanded(world, merlin, &cs) == FSIM_OK && cs.altitude_m == 3200.0 && cs.altitude_reference == FSIM_ALTITUDE_MSL);
            CHECK(!isnan(cs.north_acceleration_ms2) && !isnan(cs.down_acceleration_ms2));
            /* a caller built before 1.12 is given what its header has */
            fsim_commanded_state_init(&cs);
            cs.struct_size = (uint32_t)offsetof(fsim_commanded_state, north_acceleration_ms2);
            CHECK(fsim_vehicle_commanded(world, merlin, &cs) == FSIM_OK && isnan(cs.north_acceleration_ms2) && !isnan(cs.load_factor_g));
            /* a route: its waypoints read back, its end points */
            st = fsim_vehicle_state_ptr(world, merlin);
            CHECK(st != NULL);
            for (i = 0; i < 3; ++i) {
                fsim_waypoint_init(&route[i]);
                route[i].latitude_rad = st->latitude_rad + 0.0005 * (i + 1);
                route[i].longitude_rad = st->longitude_rad + 0.0008;
                route[i].altitude_m = 3100.0;
                route[i].id = 20 + i;
            }
            CHECK(fsim_vehicle_submit_route(world, merlin, options, 4, route, 3, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_get_setpoint(world, cr.activity, &sp) == FSIM_OK && sp.kind == FSIM_BATCH_ROUTE && sp.waypoint_count == 3);
            CHECK(sp.waypoints[2].id == 22 && sp.waypoints[2].altitude_m == 3100.0 && sp.waypoints[0].struct_size == sizeof(fsim_waypoint));
            for (i = 0; i < 4; ++i) fsim_end_point_init(&points[i]);
            CHECK(fsim_activity_end_points(world, cr.activity, points, 4, &count) == FSIM_OK && count == 3);
            CHECK(points[0].kind == FSIM_END_POINT_TURN_POINT && points[0].id == 20 && points[2].kind == FSIM_END_POINT_WAYPOINT && points[2].index == 2);
            CHECK(strcmp(fsim_end_point_kind_name(points[2].kind), "waypoint") == 0 && isnan(points[2].turn));
            /* a curve: its reference in its fields, its segments */
            for (i = 0; i < 8; ++i) curve[i] = hold;
            fsim_bezier_segment_init(&piece);
            for (i = 0; i < 6; ++i) piece.north[i] = 400.0 * i;
            CHECK(fsim_vehicle_submit_curve(world, merlin, curve, 8, &piece, 1, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_get_setpoint(world, cr.activity, &sp) == FSIM_OK && sp.kind == FSIM_BATCH_CURVE && sp.segment_count == 1);
            CHECK(sp.count == 8 && !isnan(sp.fields[0]) && sp.segments[0].north[5] == 2000.0);
            /* named controllers: one holds a grant, its NEW flies, another's calls are refused */
            {
                fsim_command_options oc;
                fsim_activity_envelope env;
                uint32_t holder = 99;
                int32_t granted = -1, why = -1;
                const double east[6] = {1.57, NAN, NAN, NAN, NAN, NAN};
                CHECK(fsim_vehicle_set_control_mode(world, merlin, FSIM_CONTROL_GRANTED) == FSIM_OK);
                CHECK(fsim_vehicle_request_control_by(world, merlin, "fsim.guidance.hsa", 7, &why) == FSIM_OK && why == 0);
                CHECK(fsim_vehicle_control_holder(world, merlin, "fsim.guidance.hsa", &granted, &holder) == FSIM_OK && granted == 1 && holder == 7);
                CHECK(fsim_vehicle_request_control_by(world, merlin, "fsim.guidance.hsa", 8, &why) == FSIM_OK &&
                      strcmp(fsim_reason_name(why), "authority_held") == 0);
                fsim_command_options_init(&oc);
                CHECK(oc.controller == 0);
                CHECK(fsim_vehicle_submit_mode(world, merlin, FSIM_MODE_HSA, east, 6, &oc, &cr) == FSIM_OK && strcmp(fsim_reason_name(cr.reason), "not_granted") == 0);
                oc.controller = 7;
                CHECK(fsim_vehicle_submit_mode(world, merlin, FSIM_MODE_HSA, east, 6, &oc, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
                fsim_activity_envelope_init(&env);
                CHECK(fsim_activity_get_envelope(world, cr.activity, &env) == FSIM_OK && env.controller == 7);
                {
                    const fsim_activity_id mine = cr.activity;
                    CHECK(fsim_activity_cancel_by(world, mine, FSIM_SOURCE_POLICY, 8, &cr) == FSIM_OK && strcmp(fsim_reason_name(cr.reason), "authority_held") == 0);
                    CHECK(fsim_activity_update_by(world, mine, FSIM_SOURCE_POLICY, 7, east, 6, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
                    CHECK(fsim_activity_command(world, mine, FSIM_ACTIVITY_DISABLE, 0, 0, FSIM_SOURCE_POLICY, 8, &cr) == FSIM_OK &&
                          strcmp(fsim_reason_name(cr.reason), "authority_held") == 0);
                    CHECK(fsim_vehicle_release_control_by(world, merlin, "fsim.guidance.hsa", 8, &why) == FSIM_OK && strcmp(fsim_reason_name(why), "not_granted") == 0);
                    CHECK(fsim_vehicle_release_control_by(world, merlin, "fsim.guidance.hsa", 7, &why) == FSIM_OK && why == 0);
                    CHECK(fsim_activity_get_setpoint(world, mine, &sp) == FSIM_INVALID_ARGUMENT); /* (released: ended) */
                }
                CHECK(fsim_vehicle_set_control_mode(world, merlin, FSIM_CONTROL_OPEN) == FSIM_OK);
            }
            /* a support command; one not live */
            {
                const double up[1] = {0.0};
                fsim_activity_id gone = cr.activity;
                CHECK(fsim_vehicle_submit_support(world, merlin, FSIM_SUPPORT_GEAR, up, 1, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
                CHECK(fsim_activity_get_setpoint(world, cr.activity, &sp) == FSIM_OK && sp.kind == FSIM_BATCH_SUPPORT && sp.code == FSIM_SUPPORT_GEAR);
                CHECK(sp.count == 1 && sp.fields[0] == 0.0);
                CHECK(fsim_activity_cancel(world, gone, &cr) == FSIM_OK);
                CHECK(fsim_activity_get_setpoint(world, gone, &sp) == FSIM_INVALID_ARGUMENT);
            }
        }
        {
            /* ABI 1.13: the navigation report - its fuel, endurance, playtime to a recovery point, contingency
             * (docs/flight-autonomy.md, 4.14) */
            fsim_navigation_report nr;
            fsim_navigation_settings ns, got;
            uint32_t navigator = 0;
            spec.name = "cap-navigator";
            spec.type = "jsbsim:c172";
            spec.altitude_msl_m = 1500.0;
            spec.airspeed_ms = 50.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &navigator) == FSIM_OK);
            CHECK(fsim_world_step(world, 5) == FSIM_OK);
            fsim_navigation_report_init(&nr);
            CHECK(nr.struct_size == sizeof nr && isnan(nr.remaining) && isnan(nr.playtime_s) && nr.energy == FSIM_ENERGY_UNKNOWN);
            CHECK(fsim_vehicle_navigation_report(world, navigator, &nr) == FSIM_OK);
            CHECK(nr.energy == FSIM_ENERGY_FUEL && nr.fuel_kg == nr.remaining && nr.remaining > 0.0 && nr.capacity >= nr.remaining);
            CHECK(nr.percent > 90.0 && nr.consumption > 0.0 && fabs(nr.endurance_s - nr.remaining / nr.consumption) < 1e-6 * nr.endurance_s);
            CHECK(fabs(nr.reserve - 0.1 * nr.capacity) < 1e-9 * nr.capacity && isnan(nr.playtime_s) && isnan(nr.return_distance_m));
            CHECK(nr.contingency == FSIM_CONTINGENCY_NORMAL && nr.starved == 0);
            CHECK(fsim_vehicle_navigation_report(world, 999, &nr) != FSIM_OK);
            CHECK(strcmp(fsim_energy_name(FSIM_ENERGY_BATTERY), "battery") == 0 && strcmp(fsim_energy_name(9), "?") == 0);
            CHECK(strcmp(fsim_contingency_name(FSIM_CONTINGENCY_FLIGHT_CRITICAL), "FLIGHT_CRITICAL") == 0);
            CHECK(strcmp(fsim_contingency_name(-1), "?") == 0);
            /* a recovery point: the playtime to it */
            fsim_navigation_settings_init(&ns);
            CHECK(ns.struct_size == sizeof ns && ns.recovery == 0 && ns.reserve_fraction == 0.1 && isnan(ns.latitude_deg));
            ns.recovery = 1;
            ns.latitude_deg = spec.latitude_deg;
            ns.longitude_deg = spec.longitude_deg - 0.2;
            ns.altitude_msl_m = 300.0;
            CHECK(fsim_vehicle_set_navigation(world, navigator, &ns) == FSIM_OK);
            fsim_navigation_settings_init(&got);
            CHECK(fsim_vehicle_get_navigation(world, navigator, &got) == FSIM_OK && got.recovery == 1 && got.longitude_deg == ns.longitude_deg);
            CHECK(fsim_vehicle_navigation_report(world, navigator, &nr) == FSIM_OK);
            CHECK(nr.return_distance_m > 15000.0 && nr.return_tas_ms > 0.0 && nr.return_consumption > 0.0);
            CHECK(nr.playtime_s > 0.0 && nr.playtime_s < nr.endurance_s);
            /* what it refuses: a reserve outside [0, 1), a point off the Earth, a short struct, no such vehicle */
            ns.reserve_fraction = 1.0;
            CHECK(fsim_vehicle_set_navigation(world, navigator, &ns) != FSIM_OK);
            ns.reserve_fraction = 0.2;
            ns.latitude_deg = 95.0;
            CHECK(fsim_vehicle_set_navigation(world, navigator, &ns) != FSIM_OK);
            ns.latitude_deg = spec.latitude_deg;
            ns.struct_size = (uint32_t)offsetof(fsim_navigation_settings, reserve_fraction);
            CHECK(fsim_vehicle_set_navigation(world, navigator, &ns) != FSIM_OK);
            ns.struct_size = sizeof ns;
            CHECK(fsim_vehicle_set_navigation(world, 999, &ns) != FSIM_OK);
            CHECK(fsim_vehicle_get_navigation(world, 999, &got) != FSIM_OK);
            CHECK(fsim_vehicle_get_navigation(world, navigator, &got) == FSIM_OK && got.reserve_fraction == 0.1); /* (kept) */
            /* a caller whose struct ends earlier is given what its header has */
            fsim_navigation_report_init(&nr);
            nr.struct_size = (uint32_t)offsetof(fsim_navigation_report, contingency);
            nr.contingency = 77;
            CHECK(fsim_vehicle_navigation_report(world, navigator, &nr) == FSIM_OK && nr.contingency == 77 && nr.playtime_s > 0.0);
        }
        {
            /* ABI 1.14: the performance profile - a flight mode's guard rails at the condition now
             * (docs/flight-autonomy.md, 4.15) */
            fsim_performance_profile pp;
            int32_t why = -1;
            uint32_t viper = 0, k;
            spec.name = "cap-profiled";
            spec.type = "jsbsim:f16c";
            spec.altitude_msl_m = 3000.0;
            spec.airspeed_ms = 160.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &viper) == FSIM_OK);
            CHECK(fsim_world_step(world, 300) == FSIM_OK); /* (10 s: spawned in the air with its gear down, it retracts it) */
            fsim_performance_profile_init(&pp);
            CHECK(pp.struct_size == sizeof pp && isnan(pp.max_altitude_msl_m) && pp.max_airspeed == NULL && pp.max_airspeed_count == 0);
            CHECK(fsim_vehicle_performance_profile(world, viper, FSIM_FLIGHT_MODE_HSA_CSA, &pp, &why) == FSIM_OK && why == 0);
            CHECK(pp.mode == FSIM_FLIGHT_MODE_HSA_CSA && pp.energy == FSIM_ENERGY_FUEL && pp.clean == 1 && pp.gear_down == 0);
            CHECK(pp.weight_kg > 0.0 && fabs(pp.altitude_msl_m - 3000.0) < 200.0);
            CHECK(pp.max_airspeed_count >= 3 && pp.max_airspeed != NULL && pp.min_airspeed_count == pp.max_airspeed_count);
            for (k = 0; k < pp.max_airspeed_count; ++k)
                CHECK(pp.max_airspeed[k].value > pp.min_airspeed[k].value && pp.max_airspeed[k].altitude_msl_m == pp.min_airspeed[k].altitude_msl_m);
            CHECK(pp.best_range_airspeed_count > 0 && pp.best_range_airspeed[0].value > pp.min_airspeed[0].value);
            CHECK(pp.excess_power_count >= 48 && pp.excess_power[0].climb_ms > 0.0 && pp.excess_power[0].tas_ms > 0.0);
            CHECK(pp.burn_count > 0 && pp.burn[0].value > 0.0);
            CHECK(pp.min_acceleration_count > 0 && pp.min_acceleration[0].z_ms2 < -9.8); /* (a pull: body z up) */
            CHECK(pp.max_deceleration_count > 0 && pp.max_deceleration[0].x_ms2 < 0.0);
            CHECK(pp.max_orientation_count == 1 && pp.max_orientation[0].roll_rad > 0.0 && isnan(pp.max_orientation[0].yaw_rad));
            CHECK(pp.max_orientation_rate_count == 1 && pp.max_altitude_msl_m > 10000.0 && isnan(pp.min_altitude_msl_m));
            CHECK(pp.max_turn_rate_rad_s > 0.0 && pp.max_climb_rate_ms > 0.0);
            CHECK(fsim_vehicle_performance_profile(world, viper, FSIM_FLIGHT_MODE_CURVE_FOLLOWING, &pp, NULL) == FSIM_OK && pp.mode == FSIM_FLIGHT_MODE_CURVE_FOLLOWING);
            /* what has none: another mode, no such vehicle, not a mode */
            CHECK(fsim_vehicle_performance_profile(world, viper, FSIM_FLIGHT_MODE_LOITER, &pp, &why) != FSIM_OK && strcmp(fsim_reason_name(why), "invalid_parameter") == 0);
            CHECK(fsim_vehicle_performance_profile(world, 999, FSIM_FLIGHT_MODE_HSA_CSA, &pp, &why) != FSIM_OK && strcmp(fsim_reason_name(why), "unknown_vehicle") == 0);
            CHECK(fsim_vehicle_performance_profile(world, viper, 99, &pp, NULL) != FSIM_OK);
        }
        {
            /* ABI 1.15: speed optimisation - an hsa's seventh field and a pattern's thirteenth: the performance
             * tables' best speed, flown at the altitude and weight now (docs/flight-autonomy.md, 4.17) */
            fsim_activity_progress progress;
            fsim_batch_command sp;
            fsim_command_result cr;
            uint32_t cruiser = 0, stock = 0;
            const double hold = fsim_hold();
            double hsa[7], faster[7], pattern[13];
            int k;
            spec.name = "cap-cruiser";
            spec.type = "jsbsim:f16c";
            spec.altitude_msl_m = 3000.0;
            spec.airspeed_ms = 160.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &cruiser) == FSIM_OK);
            CHECK(fsim_world_step(world, 30) == FSIM_OK);
            for (k = 0; k < 7; ++k) hsa[k] = faster[k] = hold;
            hsa[0] = 1.0, hsa[6] = FSIM_SPEED_MAX_ENDURANCE;
            CHECK(fsim_vehicle_submit_mode(world, cruiser, FSIM_MODE_HSA, hsa, 7, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            memset(&sp, 0, sizeof sp);
            sp.struct_size = sizeof sp;
            CHECK(fsim_activity_get_setpoint(world, cr.activity, &sp) == FSIM_OK && sp.count == 8 && sp.fields[6] == FSIM_SPEED_MAX_ENDURANCE);
            CHECK(sp.fields[3] == FSIM_SPEED_TRUE_AIRSPEED && sp.fields[2] > 50.0 && sp.fields[2] < 250.0); /* resolved: the optimum as given */
            CHECK(fsim_world_step(world, 2) == FSIM_OK);
            fsim_activity_progress_init(&progress);
            CHECK(fsim_activity_get_progress(world, cr.activity, &progress) == FSIM_OK && progress.speed_reference == FSIM_SPEED_TRUE_AIRSPEED);
            CHECK(fabs(progress.speed_ms - sp.fields[2]) < 5.0); /* (the optimum now: the same tables, a little lighter and lower) */
            /* a speed replaces it */
            faster[2] = 200.0;
            CHECK(fsim_activity_update(world, cr.activity, faster, 7, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_get_setpoint(world, cr.activity, &sp) == FSIM_OK && sp.fields[2] == 200.0 && isnan(sp.fields[6]));
            /* a pattern's, the same */
            for (k = 0; k < 13; ++k) pattern[k] = hold;
            pattern[12] = FSIM_SPEED_LONG_RANGE_CRUISE;
            CHECK(fsim_vehicle_submit_mode(world, cruiser, FSIM_MODE_PATTERN, pattern, 13, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK(fsim_activity_get_setpoint(world, cr.activity, &sp) == FSIM_OK && sp.count == 25 && sp.fields[12] == FSIM_SPEED_LONG_RANGE_CRUISE); /* (1.21: 25) */
            /* a stock aircraft has no performance tables to fly one from: not implemented, the field named */
            spec.name = "cap-stock";
            spec.type = "jsbsim:c172x";
            spec.altitude_msl_m = 1500.0;
            spec.airspeed_ms = 55.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &stock) == FSIM_OK);
            CHECK(fsim_vehicle_submit_mode(world, stock, FSIM_MODE_HSA, hsa, 7, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "not_implemented") == 0 && cr.reserved == 7); /* (field 6, plus one) */
        }
        {
            /* ABI 1.16: endurance - a flight with an end needs no more than the vehicle has above its reserve, a soft
             * rejection override_rejection overrides (docs/flight-autonomy.md, 4.18) */
            fsim_command_endurance e;
            fsim_command_options oo;
            fsim_command_result cr;
            fsim_waypoint far[1];
            const fsim_vehicle_state* st;
            uint32_t ranger = 0;
            double options[4] = {0.0, 0.0, 0.0, 0.0};
            spec.name = "cap-ranger";
            spec.type = "jsbsim:c172";
            spec.altitude_msl_m = 1500.0;
            spec.airspeed_ms = 50.0;
            spec.longitude_deg += 0.01;
            CHECK(fsim_world_create_vehicle(world, &spec, &ranger) == FSIM_OK);
            CHECK(fsim_world_step(world, 30) == FSIM_OK);
            st = fsim_vehicle_state_ptr(world, ranger);
            CHECK(st != NULL);
            fsim_waypoint_init(&far[0]);
            far[0].latitude_rad = st->latitude_rad + 3000e3 / 6371000.0; /* 3,000 km north: far beyond its fuel */
            far[0].longitude_rad = st->longitude_rad;
            far[0].altitude_m = 1500.0, far[0].speed = 50.0, far[0].speed_reference = FSIM_SPEED_TRUE_AIRSPEED;
            CHECK(fsim_vehicle_submit_route(world, ranger, options, 4, far, 1, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
            CHECK(strcmp(fsim_reason_name(cr.reason), "insufficient_endurance") == 0 && cr.reserved == 0);
            fsim_command_endurance_init(&e);
            CHECK(e.struct_size == sizeof e && e.energy == 0 && isnan(e.required));
            CHECK(fsim_last_command_endurance(world, &e) == FSIM_OK && e.energy == FSIM_ENERGY_FUEL);
            CHECK(e.required > e.remaining && e.remaining > 0.0 && fabs(e.required_s - 60000.0) < 600.0 && e.remaining_s > 0.0);
            fsim_command_options_init(&oo);
            oo.override_rejection = 1;
            CHECK(fsim_vehicle_submit_route(world, ranger, options, 4, far, 1, &oo, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
            CHECK((cr.flags & FSIM_COMMAND_OVERRIDDEN) != 0);
            CHECK(fsim_last_command_endurance(world, &e) == FSIM_OK && e.energy == FSIM_ENERGY_FUEL && e.required > e.remaining);
            /* a caller built before 1.16 is given what its header has */
            fsim_command_endurance_init(&e);
            e.struct_size = (uint32_t)offsetof(fsim_command_endurance, remaining);
            e.remaining = 77.0;
            CHECK(fsim_last_command_endurance(world, &e) == FSIM_OK && e.energy == FSIM_ENERGY_FUEL && e.remaining == 77.0);
            {
                /* ABI 1.17: the terrain - a path into the ground is refused terrain_conflict, with the place it would meet
                 * it and when (docs/flight-autonomy.md, 4.19); the ground is asked for */
                fsim_command_terrain t;
                double hsa[6], lat[2], lon[2], h[2];
                hsa[0] = hsa[1] = hsa[2] = hsa[3] = fsim_hold();
                hsa[4] = -50.0, hsa[5] = FSIM_ALTITUDE_MSL; /* under the flat ground, at sea level */
                CHECK(fsim_vehicle_submit_mode(world, ranger, FSIM_MODE_HSA, hsa, 6, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED);
                CHECK(strcmp(fsim_reason_name(cr.reason), "terrain_conflict") == 0);
                fsim_command_terrain_init(&t);
                CHECK(t.struct_size == sizeof t && t.hit == 0 && t.index == -1 && isnan(t.time_s));
                CHECK(fsim_last_command_terrain(world, &t) == FSIM_OK && t.hit == 1 && t.index == -1);
                CHECK(t.altitude_msl_m == -50.0 && t.ground_m == 0.0 && fabs(t.time_s) < 1e-9); /* where it is, at once */
                st = fsim_vehicle_state_ptr(world, ranger);
                CHECK(fabs(t.latitude_rad - st->latitude_rad) < 1e-9 && fabs(t.longitude_rad - st->longitude_rad) < 1e-9);
                hsa[4] = 1200.0;
                CHECK(fsim_vehicle_submit_mode(world, ranger, FSIM_MODE_HSA, hsa, 6, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
                CHECK(fsim_last_command_terrain(world, &t) == FSIM_OK && t.hit == 0);
                /* the ground: flat, at sea level */
                lat[0] = st->latitude_rad, lon[0] = st->longitude_rad, lat[1] = 0.1, lon[1] = 0.2;
                CHECK(fsim_world_terrain(world, 2, lat, lon, h) == 2 && h[0] == 0.0 && h[1] == 0.0);
                CHECK(fsim_world_terrain(world, 0, NULL, NULL, NULL) == 0);
                CHECK(fsim_world_terrain(world, 1, NULL, lon, h) == FSIM_INVALID_ARGUMENT);
            }
            {
                /* ABI 1.18: the barometric altimeter - its QNH set, what it reads, a barometric altitude flown
                 * (docs/flight-autonomy.md, 4.20) */
                fsim_state_data sd;
                double qnh = 0.0, standard = 0.0, hsa[6];
                fsim_state_data_init(&sd);
                CHECK(sd.struct_size == sizeof sd && isnan(sd.indicated_altitude_m));
                CHECK(fsim_vehicle_qnh(world, ranger, &qnh) == FSIM_OK && qnh == 101325.0);
                CHECK(fsim_vehicle_state_data(world, ranger, &sd) == FSIM_OK && sd.kollsman_hpa == 1013.25);
                st = fsim_vehicle_state_ptr(world, ranger);
                CHECK(fabs(sd.indicated_altitude_m - st->altitude_msl_m) < 1.0); /* (its geopotential height: 0.35 m less at 1,500 m) */
                CHECK(sd.static_pressure_pa > 80000.0 && sd.static_pressure_pa < 90000.0 && sd.static_temperature_k > 270.0);
                standard = sd.indicated_altitude_m;
                CHECK(fsim_vehicle_set_qnh(world, ranger, 80000.0) == FSIM_INVALID_ARGUMENT);
                CHECK(fsim_vehicle_set_qnh(world, 99999, 100000.0) == FSIM_INVALID_ARGUMENT);
                CHECK(fsim_vehicle_set_qnh(world, ranger, 100000.0) == FSIM_OK);
                CHECK(fsim_vehicle_qnh(world, ranger, &qnh) == FSIM_OK && qnh == 100000.0);
                CHECK(fsim_vehicle_state_data(world, ranger, &sd) == FSIM_OK && sd.kollsman_hpa == 1000.0);
                CHECK(fabs(standard - sd.indicated_altitude_m - 110.9) < 0.2); /* (1000 hPa is 110.9 m up the standard atmosphere) */
                hsa[0] = hsa[1] = hsa[2] = hsa[3] = fsim_hold();
                hsa[4] = 1400.0, hsa[5] = FSIM_ALTITUDE_BAROMETRIC;
                CHECK(fsim_vehicle_submit_mode(world, ranger, FSIM_MODE_HSA, hsa, 6, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
                CHECK(fsim_vehicle_state_data(world, 99999, &sd) == FSIM_INVALID_ARGUMENT);
            }
            {
                /* ABI 1.19: the state data's orientation rates, wander angle and wind; reference frames
                 * (docs/flight-autonomy.md, 4.21) */
                fsim_state_data sd;
                fsim_frame_spec fs;
                fsim_frame_offset fo;
                uint64_t frame = 0;
                double lat = 0.0, lon = 0.0, alt = 0.0;
                fsim_state_data_init(&sd);
                CHECK(isnan(sd.yaw_rate_rad_s) && isnan(sd.wind_down_ms));
                CHECK(fsim_vehicle_state_data(world, ranger, &sd) == FSIM_OK);
                CHECK(sd.wander_angle_rad == 0.0 && isfinite(sd.yaw_rate_rad_s) && isfinite(sd.roll_acceleration_rad_s2));
                CHECK(sqrt(sd.wind_north_ms * sd.wind_north_ms + sd.wind_east_ms * sd.wind_east_ms + sd.wind_down_ms * sd.wind_down_ms) < 1e-6); /* (calm) */
                /* a caller built before 1.19 is given what its header has */
                fsim_state_data_init(&sd);
                sd.struct_size = (uint32_t)offsetof(fsim_state_data, yaw_rate_rad_s);
                sd.yaw_rate_rad_s = 77.0;
                CHECK(fsim_vehicle_state_data(world, ranger, &sd) == FSIM_OK && sd.yaw_rate_rad_s == 77.0 && isfinite(sd.kollsman_hpa));
                /* a fixed frame: 1 km north round the sphere */
                fsim_frame_spec_init(&fs);
                CHECK(fs.struct_size == sizeof fs && fs.origin == FSIM_FRAME_FIXED);
                fs.latitude_rad = 0.6, fs.longitude_rad = -2.1, fs.altitude_msl_m = 100.0;
                CHECK(fsim_world_create_frame(world, &fs, &frame) == FSIM_OK && frame != 0);
                fsim_frame_offset_init(&fo);
                fo.offsets = FSIM_FRAME_GREAT_CIRCLE, fo.x = 1000.0;
                CHECK(fsim_world_frame_point(world, frame, &fo, NAN, &lat, &lon, &alt) == FSIM_OK);
                CHECK(fabs(lat - (0.6 + 1000.0 / 6371008.8)) < 1e-12 && fabs(lon + 2.1) < 1e-12 && alt == 100.0);
                CHECK(fsim_world_remove_frame(world, frame) == FSIM_OK && fsim_world_remove_frame(world, frame) == FSIM_INVALID_ARGUMENT);
                /* a vehicle's: where it is */
                fs.origin = FSIM_FRAME_VEHICLE, fs.vehicle = ranger;
                CHECK(fsim_world_create_frame(world, &fs, &frame) == FSIM_OK);
                fsim_frame_offset_init(&fo);
                CHECK(fsim_world_frame_point(world, frame, &fo, NAN, &lat, &lon, &alt) == FSIM_OK);
                st = fsim_vehicle_state_ptr(world, ranger);
                CHECK(fabs(lat - st->latitude_rad) < 1e-12 && fabs(lon - st->longitude_rad) < 1e-12 && fabs(alt - st->altitude_msl_m) < 1e-6);
                fo.rotation = 9; /* not one */
                CHECK(fsim_world_frame_point(world, frame, &fo, NAN, &lat, &lon, &alt) == FSIM_INVALID_ARGUMENT);
                fs.vehicle = 99999;
                CHECK(fsim_world_create_frame(world, &fs, &frame) == FSIM_INVALID_ARGUMENT);
            }
            {
                /* ABI 1.20: the magnetic model, and a heading from magnetic north (docs/flight-autonomy.md, 4.22) */
                const double pi = 3.14159265358979323846;
                fsim_magnetic_field mf;
                fsim_state_data sd;
                double hsa[8];
                fsim_magnetic_field_init(&mf);
                CHECK(mf.struct_size == sizeof mf && isnan(mf.declination_rad));
                /* the technical report's first test value: 2025.0, at sea level, 80 N 0 E - X 6521.6 nT, D 1.28 deg */
                CHECK(fsim_magnetic_field_at(80.0 * pi / 180.0, 0.0, 0.0, 2025.0, &mf) == FSIM_OK);
                CHECK(fabs(mf.north_nt - 6521.6) < 0.05 && fabs(mf.declination_rad * 180.0 / pi - 1.28) < 0.005);
                CHECK(fsim_magnetic_field_at(2.0, 0.0, 0.0, 2025.0, &mf) == FSIM_INVALID_ARGUMENT); /* (off the Earth) */
                CHECK(fsim_decimal_year(1735689600.0) == 2025.0);
                CHECK(fsim_world_magnetic_year(world) == 2025.0); /* (its clock never set) */
                /* an hsa's eighth field */
                hsa[0] = 0.0, hsa[1] = hsa[2] = hsa[3] = hsa[4] = hsa[5] = hsa[6] = fsim_hold(), hsa[7] = FSIM_DIRECTION_MAGNETIC_NORTH;
                CHECK(fsim_vehicle_submit_mode(world, ranger, FSIM_MODE_HSA, hsa, 8, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
                hsa[7] = 2.0; /* not one */
                CHECK(fsim_vehicle_submit_mode(world, ranger, FSIM_MODE_HSA, hsa, 8, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED &&
                      cr.reserved == 8); /* (the field's index plus one) */
                fsim_state_data_init(&sd);
                CHECK(isnan(sd.magnetic_heading_rad) && isnan(sd.declination_rad));
                CHECK(fsim_vehicle_state_data(world, ranger, &sd) == FSIM_OK && isfinite(sd.declination_rad));
                st = fsim_vehicle_state_ptr(world, ranger);
                CHECK(fabs(remainder(sd.magnetic_heading_rad - (st->euler_rad[2] - sd.declination_rad), 2.0 * pi)) < 1e-9);
            }
            {
                /* ABI 1.21: A-GRA's orbit as its schema gives it (docs/flight-autonomy.md, 4.23) - a racetrack by two
                   circles, read back complete; one refused naming its field; laps that complete it */
                double pattern[25];
                fsim_batch_command sp2;
                fsim_activity_progress progress;
                int k;
                st = fsim_vehicle_state_ptr(world, ranger);
                for (k = 0; k < 25; ++k) pattern[k] = fsim_hold();
                pattern[0] = FSIM_PATTERN_RACETRACK;
                pattern[1] = st->latitude_rad, pattern[2] = st->longitude_rad + 0.0005; /* the first circle's centre */
                pattern[5] = 900.0;
                pattern[18] = st->latitude_rad + 0.0006, pattern[19] = pattern[2]; /* the second's, about 3.8 km north */
                pattern[20] = 1200.0;
                pattern[17] = 1.0; /* orbits */
                CHECK(fsim_vehicle_submit_mode(world, ranger, FSIM_MODE_PATTERN, pattern, 25, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
                memset(&sp2, 0, sizeof sp2);
                sp2.struct_size = sizeof sp2;
                CHECK(fsim_activity_get_setpoint(world, cr.activity, &sp2) == FSIM_OK && sp2.code == FSIM_MODE_PATTERN && sp2.count == 25);
                CHECK(sp2.fields[20] == 1200.0 && sp2.fields[17] == 1.0 && isnan(sp2.fields[7]) && isnan(sp2.fields[8])); /* (the circles give the legs) */
                fsim_activity_progress_init(&progress);
                CHECK(fsim_world_step(world, 2) == FSIM_OK && fsim_activity_get_progress(world, cr.activity, &progress) == FSIM_OK);
                CHECK(progress.segments >= 4 && progress.laps == 0);
                pattern[0] = FSIM_PATTERN_ORBIT; /* an orbit is one circle */
                CHECK(fsim_vehicle_submit_mode(world, ranger, FSIM_MODE_PATTERN, pattern, 25, NULL, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_REJECTED &&
                      cr.reserved == 19); /* (the field's index plus one) */
                CHECK(fsim_vehicle_submit_mode(world, ranger, FSIM_MODE_PATTERN, pattern, 26, NULL, &cr) != FSIM_OK); /* malformed: 25 at most */
            }
        }
        }
        fsim_world_destroy(world);
    }
    printf("c abi ok\n");
    return 0;
}
