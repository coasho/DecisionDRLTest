// Minimal C++ trainer over the fsim SDK (design 9.1): the shape of a real
// training loop without the learner. Runs a fixed-gain PD policy on the
// altitude/heading-hold task, then random actions, and reports episode
// returns and throughput. Replace `policy()` with your network.
//
//   minimal_trainer [--envs N] [--vehicles K] [--steps S] [--seed X] [--random]

#include <fsim/VecEnv.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

namespace {

struct Args {
    unsigned envs = 16, vehicles = 1, seed = 1;
    int steps = 3000;
    bool random = false;
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (k == "--envs") a.envs = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--vehicles") a.vehicles = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--steps") a.steps = std::atoi(next());
        else if (k == "--seed") a.seed = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--random") a.random = true;
    }
    return a;
}

// Index of a named observation channel (resolved once; names are stable).
std::size_t channel(const fsim::VecEnv& env, const char* name) {
    const auto& names = env.observationNames();
    for (std::size_t i = 0; i < names.size(); ++i)
        if (names[i] == name) return i;
    std::fprintf(stderr, "no observation channel '%s'\n", name);
    std::exit(1);
}

struct Channels {
    std::size_t altErr, hdgErrSin, hdgErrCos, roll, pitch, p, q, r, vzDown;
};

// Hand-tuned PD controller in the observation space: the kind of scripted
// baseline a training run is compared against.
void policy(const Channels& c, const float* obs, float* act) {
    const float altErrM = obs[c.altErr] * 1000.0f;          // target - current
    const float hdgErr = std::atan2(obs[c.hdgErrSin], obs[c.hdgErrCos]);
    const float rollTarget = std::clamp(hdgErr * 1.2f, -0.6f, 0.6f);
    const float pitchTarget = std::clamp(altErrM * 0.0015f - obs[c.vzDown] * -0.02f, -0.25f, 0.25f);
    act[0] = std::clamp((rollTarget - obs[c.roll]) * 1.5f - obs[c.p] * 0.3f, -1.0f, 1.0f);  // aileron
    act[1] = std::clamp((pitchTarget - obs[c.pitch]) * -3.0f + obs[c.q] * 0.5f, -1.0f, 1.0f); // elevator (positive = nose down)
    act[2] = std::clamp(-obs[c.r] * 0.5f, -1.0f, 1.0f);                                     // rudder
    act[3] = std::clamp(0.2f + altErrM * 0.002f, -1.0f, 1.0f);                              // throttle [-1,1] -> [0,1]
}

} // namespace

int main(int argc, char** argv) {
    const Args args = parse(argc, argv);

    fsim::VecEnvOptions opt;
    opt.numEnvs = args.envs;
    opt.vehiclesPerEnv = args.vehicles;
    opt.seed = args.seed;
    opt.maxEpisodeSteps = 1000;

    std::printf("fsim %s, %u env(s) x %u vehicle(s), policy: %s\n", fsim::version(), opt.numEnvs, opt.vehiclesPerEnv,
                args.random ? "random" : "PD");
    fsim::VecEnv env(opt);

    const Channels ch{channel(env, "alt_err_km"), channel(env, "hdg_err_sin"), channel(env, "hdg_err_cos"),
                      channel(env, "roll"),       channel(env, "pitch"),       channel(env, "p"),
                      channel(env, "q"),          channel(env, "r"),           channel(env, "vz_down_100ms")};

    const std::size_t n = env.numVehicles(), o = env.observationSize(), a = env.actionSize();
    std::vector<float> actions(n * a, 0.0f);
    std::vector<double> episodeReturn(n, 0.0);
    std::vector<double> finished;
    std::mt19937 rng(args.seed);
    std::uniform_real_distribution<float> uni(-1.0f, 1.0f);

    fsim::StepResult r = env.reset();
    const auto t0 = std::chrono::steady_clock::now();
    for (int step = 0; step < args.steps; ++step) {
        for (std::size_t i = 0; i < n; ++i) {
            if (args.random)
                for (std::size_t j = 0; j < a; ++j) actions[i * a + j] = uni(rng);
            else
                policy(ch, r.observations.data + i * o, actions.data() + i * a);
        }
        r = env.step(actions);
        for (std::size_t i = 0; i < n; ++i) {
            episodeReturn[i] += static_cast<double>(r.rewards[i]);
            if (r.terminated[i] || r.truncated[i]) {
                finished.push_back(episodeReturn[i]);
                episodeReturn[i] = 0.0;
            }
        }
        if ((step + 1) % 500 == 0) {
            double mean = 0.0;
            for (double v : finished) mean += v;
            mean = finished.empty() ? 0.0 : mean / static_cast<double>(finished.size());
            std::printf("step %5d  episodes %4zu  mean return %8.2f  alt_err %+7.1f m  hdg_err %+6.1f deg\n", step + 1,
                        finished.size(), mean, static_cast<double>(r.observations[ch.altErr]) * 1000.0,
                        static_cast<double>(std::atan2(r.observations[ch.hdgErrSin], r.observations[ch.hdgErrCos])) * 180.0 / 3.14159265);
        }
    }
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const double agentSteps = static_cast<double>(args.steps) * static_cast<double>(n);
    std::printf("%.2f s: %.0f agent-steps/s, %.2fM vehicle-steps/s (%.1f x realtime)\n", seconds, agentSteps / seconds,
                static_cast<double>(env.vehicleSteps()) / seconds / 1e6,
                agentSteps * env.agentStepSeconds() / seconds);
    return 0;
}
