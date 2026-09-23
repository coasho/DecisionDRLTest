/* The C side of python/benchmarks/overhead.py: the loops the Python benchmark
 * runs, run here through the C ABI directly, so that the difference between
 * the two is what Python costs and nothing else.
 *
 *   fsim_python_baseline vecenv <envs> <frame_skip> <workers> <steps>
 *   fsim_python_baseline world <vehicles> <workers> <steps>
 *
 * Prints microseconds per step (best of three runs). */
#include "fsim/fsim_c.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static double now(void) {
    struct timespec t;
    timespec_get(&t, TIME_UTC);
    return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

static int vecenv(unsigned envs, int frame_skip, unsigned workers, unsigned steps) {
    fsim_options o;
    fsim_options_init(&o);
    o.num_envs = envs;
    o.frame_skip = frame_skip;
    o.workers = workers;
    o.seed = 1;
    o.publish = 0;
    fsim_vecenv* env = NULL;
    if (fsim_vecenv_create(&o, &env) != FSIM_OK) {
        fprintf(stderr, "%s\n", fsim_last_error());
        return 1;
    }
    fsim_buffers b;
    memset(&b, 0, sizeof b);
    b.struct_size = sizeof b;
    fsim_vecenv_buffers(env, &b);
    const size_t n = (size_t)b.num_envs * b.vehicles_per_env * b.action_size;
    float* actions = (float*)calloc(n, sizeof(float));
    for (size_t i = 3; i < n; i += b.action_size) actions[i] = 0.6f;
    double best = 1e30;
    for (int run = 0; run < 3; ++run) {
        fsim_vecenv_reset(env, 1);
        for (unsigned k = 0; k < 20; ++k) fsim_vecenv_step(env, actions, n);
        const double t0 = now();
        for (unsigned k = 0; k < steps; ++k) fsim_vecenv_step(env, actions, n);
        const double dt = (now() - t0) / steps;
        if (dt < best) best = dt;
    }
    printf("%.3f\n", best * 1e6);
    free(actions);
    fsim_vecenv_destroy(env);
    return 0;
}

static int world(unsigned vehicles, unsigned workers, unsigned steps, int step_only) {
    fsim_world_options o;
    fsim_world_options_init(&o);
    o.name = "baseline";
    o.workers = workers;
    o.pin_workers = 0;
    o.publish = 0;
    fsim_world* w = NULL;
    if (fsim_world_create(&o, &w) != FSIM_OK) {
        fprintf(stderr, "%s\n", fsim_last_error());
        return 1;
    }
    uint32_t* ids = (uint32_t*)calloc(vehicles, sizeof(uint32_t));
    for (unsigned i = 0; i < vehicles; ++i) {
        fsim_vehicle_spec s;
        fsim_vehicle_spec_init(&s);
        char name[32];
        snprintf(name, sizeof name, "v%u", i);
        s.name = name;
        s.longitude_deg += 0.01 * i;
        s.altitude_msl_m = 1500.0;
        s.airspeed_ms = 60.0;
        fsim_world_create_vehicle(w, &s, &ids[i]);
    }
    double best = 1e30, sink = 0.0;
    if (step_only) {
        for (unsigned i = 0; i < vehicles; ++i) {
            fsim_attitude_command c = {0.1, 0.02, fsim_hold(), 0.785, fsim_hold(), 60.0};
            fsim_vehicle_command_attitude(w, ids[i], &c);
        }
    }
    for (int run = 0; run < 3; ++run) {
        const double t0 = now();
        for (unsigned k = 0; k < steps; ++k) {
            for (unsigned i = 0; !step_only && i < vehicles; ++i) {
                const fsim_vehicle_state* st = fsim_vehicle_state_ptr(w, ids[i]);
                fsim_attitude_command c = {0.2 * sin(0.01 * k), 0.02, fsim_hold(), 0.785, fsim_hold(), 60.0};
                sink += st->altitude_msl_m;
                fsim_vehicle_command_attitude(w, ids[i], &c);
            }
            fsim_world_step(w, 1);
        }
        const double dt = (now() - t0) / steps;
        if (dt < best) best = dt;
    }
    printf("%.3f\n", best * 1e6);
    fprintf(stderr, "(%g)\n", sink); /* keep the reads */
    free(ids);
    fsim_world_destroy(w);
    return 0;
}

int main(int argc, char** argv) {
    fsim_set_log_level(FSIM_LOG_WARN);
    if (argc == 6 && strcmp(argv[1], "vecenv") == 0)
        return vecenv((unsigned)atoi(argv[2]), atoi(argv[3]), (unsigned)atoi(argv[4]), (unsigned)atoi(argv[5]));
    if (argc == 5 && strcmp(argv[1], "world") == 0)
        return world((unsigned)atoi(argv[2]), (unsigned)atoi(argv[3]), (unsigned)atoi(argv[4]), 0);
    if (argc == 5 && strcmp(argv[1], "step") == 0) /* world steps alone, commanded once */
        return world((unsigned)atoi(argv[2]), (unsigned)atoi(argv[3]), (unsigned)atoi(argv[4]), 1);
    fprintf(stderr, "usage: fsim_python_baseline vecenv <envs> <frame_skip> <workers> <steps> | world <vehicles> <workers> <steps>\n");
    return 2;
}
