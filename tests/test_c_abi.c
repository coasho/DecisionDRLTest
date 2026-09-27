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
            CHECK(strcmp(fsim_reason_name(cr.reason), "invalid_parameter") == 0);
            {
                /* ABI 1.6: what the answer was about - the field, and the limit its value broke */
                fsim_command_detail d;
                double wild[6];
                CHECK(cr.reserved == 5); /* north_ms, the fifth field */
                fsim_command_detail_init(&d);
                CHECK(d.struct_size == sizeof d && d.index == -1 && isnan(d.from));
                CHECK(fsim_last_command_detail(world, &d) == FSIM_OK && d.index == 4 && d.constraint == 0);
                CHECK(strcmp(fsim_reason_name(d.reason), "invalid_parameter") == 0);
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
            CHECK(cr.status == FSIM_COMMAND_REJECTED && strcmp(fsim_reason_name(cr.reason), "unknown_capability") == 0);
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
            CHECK(cr.status == FSIM_COMMAND_REJECTED && strcmp(fsim_reason_name(cr.reason), "unknown_capability") == 0);
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
            CHECK(fsim_mode_field_count(FSIM_MODE_HSA) == 6 && fsim_mode_field_count(99) == 0);
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
        }
        fsim_world_destroy(world);
    }
    printf("c abi ok\n");
    return 0;
}
