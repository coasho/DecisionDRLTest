// PPO on the fsim batch environment, dependency-free: a two-hidden-layer MLP
// policy/value network with hand-written backpropagation and Adam, clipped
// PPO objective, GAE, running observation normalisation. Small enough to read
// in one sitting; fast enough to train altitude/heading hold on the c172x in
// minutes. Replace the network with LibTorch (or anything) and keep the loop.
//
//   ppo_trainer [--envs 64] [--horizon 64] [--iterations 300] [--action attitude|surfaces|velocity]
//               [--lr 3e-4] [--seed 1] [--save policy.bin] [--load policy.bin] [--eval] [--scenario file.json]
//
// While it runs, flightsim-viewer.exe shows the batch (world "ppo").

#include <fsim/Scenario.h>
#include <fsim/VecEnv.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace {

struct Args {
    unsigned envs = 64, horizon = 64, iterations = 300, epochs = 4, minibatch = 512, hidden = 64, seed = 1;
    double lr = 3e-4, gamma = 0.99, lambda = 0.95, clip = 0.2, valueCoef = 0.5, entropyCoef = 0.0, maxGradNorm = 0.5;
    std::string action = "attitude", save, load, scenario;
    bool eval = false;
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (k == "--envs") a.envs = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--horizon") a.horizon = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--iterations") a.iterations = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--epochs") a.epochs = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--minibatch") a.minibatch = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--hidden") a.hidden = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--seed") a.seed = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--lr") a.lr = std::atof(next());
        else if (k == "--gamma") a.gamma = std::atof(next());
        else if (k == "--clip") a.clip = std::atof(next());
        else if (k == "--entropy") a.entropyCoef = std::atof(next());
        else if (k == "--action") a.action = next();
        else if (k == "--save") a.save = next();
        else if (k == "--load") a.load = next();
        else if (k == "--eval") a.eval = true;
        else if (k == "--scenario") a.scenario = next();
    }
    return a;
}

// --- Minimal dense network ----------------------------------------------------

struct Adam {
    std::vector<float> m, v;
    int t = 0;
    void step(std::vector<float>& w, const std::vector<float>& g, double lr) {
        if (m.empty()) { m.assign(w.size(), 0.0f); v.assign(w.size(), 0.0f); }
        ++t;
        const float b1 = 0.9f, b2 = 0.999f, eps = 1e-8f;
        const float c1 = 1.0f - std::pow(b1, static_cast<float>(t)), c2 = 1.0f - std::pow(b2, static_cast<float>(t));
        for (std::size_t i = 0; i < w.size(); ++i) {
            m[i] = b1 * m[i] + (1 - b1) * g[i];
            v[i] = b2 * v[i] + (1 - b2) * g[i] * g[i];
            w[i] -= static_cast<float>(lr) * (m[i] / c1) / (std::sqrt(v[i] / c2) + eps);
        }
    }
};

/// in -> tanh(h1) -> tanh(h2) -> out (linear). Batched forward with cached
/// activations for backward.
struct Mlp {
    std::size_t in, hidden, out;
    std::vector<float> w1, b1, w2, b2, w3, b3; // row-major [out][in]
    std::vector<float> g1, gb1, g2, gb2, g3, gb3;
    Adam a1, ab1, a2, ab2, a3, ab3;

    Mlp(std::size_t i, std::size_t h, std::size_t o, std::mt19937& rng, float outScale) : in(i), hidden(h), out(o) {
        auto init = [&](std::vector<float>& w, std::size_t rows, std::size_t cols, float scale) {
            w.resize(rows * cols);
            std::normal_distribution<float> n(0.0f, 1.0f);
            for (auto& x : w) x = n(rng) * scale / std::sqrt(static_cast<float>(cols)); // orthogonal-ish scaling
        };
        init(w1, h, i, 1.0f); b1.assign(h, 0.0f);
        init(w2, h, h, 1.0f); b2.assign(h, 0.0f);
        init(w3, o, h, outScale); b3.assign(o, 0.0f);
        g1.assign(w1.size(), 0); gb1.assign(h, 0); g2.assign(w2.size(), 0); gb2.assign(h, 0); g3.assign(w3.size(), 0); gb3.assign(o, 0);
    }

    struct Cache {
        std::vector<float> x, h1, h2, y;
        std::size_t n = 0;
    };

    void forward(const float* x, std::size_t n, Cache& c) const {
        c.n = n;
        c.x.assign(x, x + n * in);
        c.h1.assign(n * hidden, 0.0f);
        c.h2.assign(n * hidden, 0.0f);
        c.y.assign(n * out, 0.0f);
        for (std::size_t r = 0; r < n; ++r) {
            const float* xi = x + r * in;
            float* h1 = c.h1.data() + r * hidden;
            for (std::size_t j = 0; j < hidden; ++j) {
                float s = b1[j];
                const float* w = w1.data() + j * in;
                for (std::size_t k = 0; k < in; ++k) s += w[k] * xi[k];
                h1[j] = std::tanh(s);
            }
            float* h2 = c.h2.data() + r * hidden;
            for (std::size_t j = 0; j < hidden; ++j) {
                float s = b2[j];
                const float* w = w2.data() + j * hidden;
                for (std::size_t k = 0; k < hidden; ++k) s += w[k] * h1[k];
                h2[j] = std::tanh(s);
            }
            float* y = c.y.data() + r * out;
            for (std::size_t j = 0; j < out; ++j) {
                float s = b3[j];
                const float* w = w3.data() + j * hidden;
                for (std::size_t k = 0; k < hidden; ++k) s += w[k] * h2[k];
                y[j] = s;
            }
        }
    }

    void zeroGrad() {
        for (auto* g : {&g1, &gb1, &g2, &gb2, &g3, &gb3}) std::fill(g->begin(), g->end(), 0.0f);
    }

    /// Accumulate gradients for dL/dy (n x out).
    void backward(const Cache& c, const float* dy) {
        std::vector<float> dh2(hidden), dh1(hidden);
        for (std::size_t r = 0; r < c.n; ++r) {
            const float* h1 = c.h1.data() + r * hidden;
            const float* h2 = c.h2.data() + r * hidden;
            const float* x = c.x.data() + r * in;
            const float* d = dy + r * out;
            std::fill(dh2.begin(), dh2.end(), 0.0f);
            for (std::size_t j = 0; j < out; ++j) {
                gb3[j] += d[j];
                float* g = g3.data() + j * hidden;
                const float* w = w3.data() + j * hidden;
                for (std::size_t k = 0; k < hidden; ++k) { g[k] += d[j] * h2[k]; dh2[k] += d[j] * w[k]; }
            }
            std::fill(dh1.begin(), dh1.end(), 0.0f);
            for (std::size_t j = 0; j < hidden; ++j) {
                const float dz = dh2[j] * (1.0f - h2[j] * h2[j]);
                gb2[j] += dz;
                float* g = g2.data() + j * hidden;
                const float* w = w2.data() + j * hidden;
                for (std::size_t k = 0; k < hidden; ++k) { g[k] += dz * h1[k]; dh1[k] += dz * w[k]; }
            }
            for (std::size_t j = 0; j < hidden; ++j) {
                const float dz = dh1[j] * (1.0f - h1[j] * h1[j]);
                gb1[j] += dz;
                float* g = g1.data() + j * in;
                for (std::size_t k = 0; k < in; ++k) g[k] += dz * x[k];
            }
        }
    }

    float gradNorm() const {
        double s = 0.0;
        for (const auto* g : {&g1, &gb1, &g2, &gb2, &g3, &gb3}) for (float v : *g) s += static_cast<double>(v) * v;
        return static_cast<float>(std::sqrt(s));
    }
    void scaleGrad(float k) {
        for (auto* g : {&g1, &gb1, &g2, &gb2, &g3, &gb3}) for (float& v : *g) v *= k;
    }
    void adam(double lr) {
        a1.step(w1, g1, lr); ab1.step(b1, gb1, lr); a2.step(w2, g2, lr); ab2.step(b2, gb2, lr); a3.step(w3, g3, lr); ab3.step(b3, gb3, lr);
    }
    void save(std::ostream& o) const {
        for (const auto* w : {&w1, &b1, &w2, &b2, &w3, &b3}) o.write(reinterpret_cast<const char*>(w->data()), static_cast<std::streamsize>(w->size() * sizeof(float)));
    }
    void load(std::istream& i) {
        for (auto* w : {&w1, &b1, &w2, &b2, &w3, &b3}) i.read(reinterpret_cast<char*>(w->data()), static_cast<std::streamsize>(w->size() * sizeof(float)));
    }
};

/// Running mean/variance of observations (Welford), frozen at evaluation.
struct Normaliser {
    std::vector<double> mean, m2;
    double count = 1e-4;
    explicit Normaliser(std::size_t n) : mean(n, 0.0), m2(n, 0.0) {}
    void update(const float* x, std::size_t n) {
        for (std::size_t r = 0; r < n; ++r) {
            count += 1.0;
            for (std::size_t j = 0; j < mean.size(); ++j) {
                const double v = x[r * mean.size() + j], d = v - mean[j];
                mean[j] += d / count;
                m2[j] += d * (v - mean[j]);
            }
        }
    }
    void apply(const float* x, std::size_t n, std::vector<float>& out) const {
        out.resize(n * mean.size());
        for (std::size_t r = 0; r < n; ++r)
            for (std::size_t j = 0; j < mean.size(); ++j) {
                const double sd = std::sqrt(std::max(m2[j] / count, 1e-8));
                out[r * mean.size() + j] = static_cast<float>(std::clamp((x[r * mean.size() + j] - mean[j]) / sd, -10.0, 10.0));
            }
    }
};

} // namespace

int main(int argc, char** argv) {
    const Args args = parse(argc, argv);

    fsim::VecEnvOptions opt;
    if (!args.scenario.empty()) {
        // The scenario file sets the world, the environment (wind, time, ...), effects and the
        // episode template; the command line still picks the batch size, seed and action level.
        opt = fsim::vecEnvOptions(fsim::loadScenario(args.scenario));
    } else {
        opt.maxEpisodeSteps = 600; // 20 s episodes at 30 Hz
        opt.worldName = "ppo";
    }
    opt.numEnvs = args.envs;
    opt.seed = args.seed;
    opt.action = args.action;
    fsim::VecEnv env(opt);
    const std::size_t N = env.numVehicles(), O = env.observationSize(), A = env.actionSize();
    std::printf("fsim %s PPO: %zu envs, obs %zu, act %zu (%s), horizon %u -> %zu samples/iteration\n", fsim::version(), N, O, A, args.action.c_str(),
                args.horizon, N * args.horizon);

    std::mt19937 rng(args.seed);
    Mlp policy(O, args.hidden, A, rng, 0.01f);
    Mlp value(O, args.hidden, 1, rng, 1.0f);
    std::vector<float> logStd(A, -0.5f), gLogStd(A, 0.0f);
    Adam adamLogStd;
    Normaliser norm(O);
    if (!args.load.empty()) {
        std::ifstream in(args.load, std::ios::binary);
        if (!in) { std::fprintf(stderr, "cannot open %s\n", args.load.c_str()); return 1; }
        policy.load(in); value.load(in);
        in.read(reinterpret_cast<char*>(logStd.data()), static_cast<std::streamsize>(A * sizeof(float)));
        in.read(reinterpret_cast<char*>(norm.mean.data()), static_cast<std::streamsize>(O * sizeof(double)));
        in.read(reinterpret_cast<char*>(norm.m2.data()), static_cast<std::streamsize>(O * sizeof(double)));
        in.read(reinterpret_cast<char*>(&norm.count), sizeof(double));
        std::printf("loaded %s\n", args.load.c_str());
    }

    // Rollout storage
    const std::size_t T = args.horizon, S = N * T;
    std::vector<float> obs(S * O), act(S * A), logp(S), val(S), rew(S), adv(S), ret(S);
    std::vector<std::uint8_t> done(S);
    std::vector<float> obsNorm, actions(N * A), lastObsNorm;
    Mlp::Cache pc, vc;
    std::vector<double> episodeReturn(N, 0.0), finished;
    std::vector<unsigned> episodeLength(N, 0), finishedLengths;

    fsim::StepResult r = env.reset(args.seed);
    std::normal_distribution<float> gauss(0.0f, 1.0f);
    const auto t0 = std::chrono::steady_clock::now();

    for (unsigned it = 0; it < args.iterations; ++it) {
        finished.clear();
        finishedLengths.clear();
        // --- collect -----------------------------------------------------------------
        for (std::size_t t = 0; t < T; ++t) {
            if (!args.eval) norm.update(r.observations.data, N);
            norm.apply(r.observations.data, N, obsNorm);
            policy.forward(obsNorm.data(), N, pc);
            value.forward(obsNorm.data(), N, vc);
            for (std::size_t i = 0; i < N; ++i) {
                const std::size_t s = t * N + i;
                std::copy_n(obsNorm.data() + i * O, O, obs.data() + s * O);
                float lp = 0.0f;
                for (std::size_t j = 0; j < A; ++j) {
                    const float mu = pc.y[i * A + j], sd = std::exp(logStd[j]);
                    const float z = args.eval ? 0.0f : gauss(rng);
                    const float a = mu + sd * z;
                    actions[i * A + j] = std::clamp(a, -1.0f, 1.0f);
                    act[s * A + j] = a;
                    lp += -0.5f * z * z - logStd[j] - 0.9189385f;
                }
                logp[s] = lp;
                val[s] = vc.y[i];
            }
            r = env.step(actions);
            // Time-limit truncation is not a terminal state: bootstrap the cut-off
            // return with the value of the final observation (the one before the
            // auto-reset), folded into the reward so GAE can treat it as done.
            std::vector<float> finalNorm;
            Mlp::Cache fc;
            for (std::size_t i = 0; i < N; ++i) {
                const std::size_t s = t * N + i;
                rew[s] = r.rewards[i];
                if (r.truncated[i] && !r.terminated[i]) {
                    norm.apply(r.finalObservations.data + i * O, 1, finalNorm);
                    value.forward(finalNorm.data(), 1, fc);
                    rew[s] += static_cast<float>(args.gamma) * fc.y[0];
                }
                done[s] = r.terminated[i] || r.truncated[i];
                episodeReturn[i] += r.rewards[i];
                ++episodeLength[i];
                if (done[s]) {
                    finished.push_back(episodeReturn[i]);
                    finishedLengths.push_back(episodeLength[i]);
                    episodeReturn[i] = 0.0;
                    episodeLength[i] = 0;
                }
            }
        }
        // Bootstrap value of the observation after the last step.
        norm.apply(r.observations.data, N, lastObsNorm);
        value.forward(lastObsNorm.data(), N, vc);
        // --- GAE ---------------------------------------------------------------------
        for (std::size_t i = 0; i < N; ++i) {
            float gae = 0.0f;
            float nextValue = vc.y[i];
            for (std::size_t t = T; t-- > 0;) {
                const std::size_t s = t * N + i;
                const float nonTerminal = done[s] ? 0.0f : 1.0f;
                const float delta = rew[s] + static_cast<float>(args.gamma) * nextValue * nonTerminal - val[s];
                gae = delta + static_cast<float>(args.gamma * args.lambda) * nonTerminal * gae;
                adv[s] = gae;
                ret[s] = gae + val[s];
                nextValue = val[s];
            }
        }
        {
            double m = 0.0, v = 0.0;
            for (float a : adv) m += a;
            m /= static_cast<double>(S);
            for (float a : adv) v += (a - m) * (a - m);
            const float sd = static_cast<float>(std::sqrt(v / static_cast<double>(S)) + 1e-8);
            for (float& a : adv) a = (a - static_cast<float>(m)) / sd;
        }

        // --- update (skipped in --eval) -----------------------------------------------
        double policyLoss = 0.0, valueLoss = 0.0, approxKl = 0.0, clipFrac = 0.0;
        unsigned updates = 0;
        if (!args.eval) {
            std::vector<std::size_t> index(S);
            std::iota(index.begin(), index.end(), 0);
            std::vector<float> mbObs, dy, dv;
            for (unsigned epoch = 0; epoch < args.epochs; ++epoch) {
                std::shuffle(index.begin(), index.end(), rng);
                for (std::size_t start = 0; start < S; start += args.minibatch) {
                    const std::size_t n = std::min<std::size_t>(args.minibatch, S - start);
                    mbObs.resize(n * O);
                    for (std::size_t k = 0; k < n; ++k) std::copy_n(obs.data() + index[start + k] * O, O, mbObs.data() + k * O);
                    policy.forward(mbObs.data(), n, pc);
                    value.forward(mbObs.data(), n, vc);
                    policy.zeroGrad();
                    value.zeroGrad();
                    std::fill(gLogStd.begin(), gLogStd.end(), 0.0f);
                    dy.assign(n * A, 0.0f);
                    dv.assign(n, 0.0f);
                    for (std::size_t k = 0; k < n; ++k) {
                        const std::size_t s = index[start + k];
                        // New log-prob and its gradient wrt mean and log-std.
                        float lp = 0.0f;
                        std::vector<float> dmu(A), dls(A);
                        for (std::size_t j = 0; j < A; ++j) {
                            const float mu = pc.y[k * A + j], ls = logStd[j], sd = std::exp(ls);
                            const float z = (act[s * A + j] - mu) / sd;
                            lp += -0.5f * z * z - ls - 0.9189385f;
                            dmu[j] = z / sd;            // d logp / d mu
                            dls[j] = z * z - 1.0f;      // d logp / d logstd
                        }
                        const float ratio = std::exp(lp - logp[s]);
                        const float a = adv[s];
                        const bool clipped = (a > 0 && ratio > 1 + args.clip) || (a < 0 && ratio < 1 - args.clip);
                        const float surrogate = std::min(ratio * a, std::clamp(ratio, 1 - static_cast<float>(args.clip), 1 + static_cast<float>(args.clip)) * a);
                        policyLoss -= surrogate;
                        approxKl += (ratio - 1.0f) - (lp - logp[s]);
                        clipFrac += clipped ? 1.0 : 0.0;
                        // d(-surrogate)/d logp = -ratio * a when unclipped, else 0; entropy bonus on log-std.
                        const float dLoss = clipped ? 0.0f : -ratio * a / static_cast<float>(n);
                        for (std::size_t j = 0; j < A; ++j) {
                            dy[k * A + j] = dLoss * dmu[j];
                            gLogStd[j] += dLoss * dls[j] - static_cast<float>(args.entropyCoef) / static_cast<float>(n);
                        }
                        // Value: 0.5 * (v - ret)^2
                        const float dvk = vc.y[k] - ret[s];
                        valueLoss += 0.5 * dvk * dvk;
                        dv[k] = static_cast<float>(args.valueCoef) * dvk / static_cast<float>(n);
                    }
                    policy.backward(pc, dy.data());
                    value.backward(vc, dv.data());
                    const float gn = policy.gradNorm();
                    if (gn > args.maxGradNorm) policy.scaleGrad(static_cast<float>(args.maxGradNorm) / gn);
                    const float gv = value.gradNorm();
                    if (gv > args.maxGradNorm) value.scaleGrad(static_cast<float>(args.maxGradNorm) / gv);
                    policy.adam(args.lr);
                    value.adam(args.lr);
                    adamLogStd.step(logStd, gLogStd, args.lr);
                    for (float& l : logStd) l = std::clamp(l, -3.0f, 0.5f);
                    ++updates;
                }
            }
        }

        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        double meanReturn = 0.0, meanLength = 0.0;
        for (double v : finished) meanReturn += v;
        for (unsigned v : finishedLengths) meanLength += v;
        if (!finished.empty()) { meanReturn /= static_cast<double>(finished.size()); meanLength /= static_cast<double>(finished.size()); }
        const std::size_t samples = static_cast<std::size_t>(it + 1) * S;
        std::printf("iter %4u  episodes %4zu  return %8.2f  length %6.1f  reward/step %6.3f  | ploss %7.4f vloss %8.3f kl %6.4f clip %4.2f std %5.2f | %5.1f s, %6.0f steps/s\n",
                    it + 1, finished.size(), meanReturn, meanLength, finished.empty() ? 0.0 : meanReturn / meanLength,
                    updates ? policyLoss / (updates * args.minibatch) : 0.0, updates ? valueLoss / (updates * args.minibatch) : 0.0,
                    updates ? approxKl / (updates * args.minibatch) : 0.0, updates ? clipFrac / (updates * args.minibatch) : 0.0,
                    std::exp(std::accumulate(logStd.begin(), logStd.end(), 0.0f) / static_cast<float>(A)), wall, static_cast<double>(samples) / wall);
        std::fflush(stdout);

        if (!args.save.empty() && !args.eval && (it + 1) % 10 == 0) {
            std::ofstream out(args.save, std::ios::binary);
            policy.save(out); value.save(out);
            out.write(reinterpret_cast<const char*>(logStd.data()), static_cast<std::streamsize>(A * sizeof(float)));
            out.write(reinterpret_cast<const char*>(norm.mean.data()), static_cast<std::streamsize>(O * sizeof(double)));
            out.write(reinterpret_cast<const char*>(norm.m2.data()), static_cast<std::streamsize>(O * sizeof(double)));
            out.write(reinterpret_cast<const char*>(&norm.count), sizeof(double));
        }
    }
    return 0;
}
