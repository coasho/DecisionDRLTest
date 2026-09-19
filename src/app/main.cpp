// flightsim.exe - headless entry point (design 4.4, 6.1).
//
// M0/M1 scope: load N JSBSim vehicles, step them in lockstep on a worker
// pool with fixed controls, print state and throughput. The RL environment
// layer, the C ABI and the viewer attach to the same objects in later
// milestones.

#include "app/Options.h"
#include "core/Log.h"
#include "core/Rng.h"
#include "core/Units.h"
#include "io/AssetResolver.h"
#include "platform/Clock.h"
#include "platform/Threads.h"
#include "sim/GroundProvider.h"
#include "sim/JsbsimModel.h"
#include "sim/VehiclePool.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <vector>

using namespace fsim;

namespace {

log::Level parseLevel(const std::string& s) {
    if (s == "trace") return log::Level::Trace;
    if (s == "debug") return log::Level::Debug;
    if (s == "warn") return log::Level::Warn;
    if (s == "error") return log::Level::Error;
    return log::Level::Info;
}

void printState(std::size_t index, const sim::VehicleState& s) {
    using namespace units;
    std::printf("v%02zu t=%7.2fs lat=%9.5f lon=%10.5f alt=%7.1fm agl=%7.1fm tas=%5.1fm/s "
                "roll=%6.1f pitch=%6.1f hdg=%6.1f alpha=%5.2f n=%4.2f%s%s\n",
                index, s.simTime, radiansToDegrees(s.latitudeRad), radiansToDegrees(s.longitudeRad), s.altitudeMslM,
                s.altitudeAglM, s.airspeedTrueMs, radiansToDegrees(s.eulerRad[0]), radiansToDegrees(s.eulerRad[1]),
                radiansToDegrees(s.eulerRad[2]), radiansToDegrees(s.alphaRad), s.loadFactor, s.onGround ? " [ground]" : "",
                s.diverged ? " [DIVERGED]" : "");
}

} // namespace

int main(int argc, char** argv) {
    const auto parsed = app::parseOptions(argc, argv);
    if (!parsed) {
        app::printUsage(argv[0]);
        return 2;
    }
    const app::Options& opt = *parsed;
    if (opt.help) {
        app::printUsage(argv[0]);
        return 0;
    }
    log::setLevel(parseLevel(opt.logLevel));

    // --- Resolve data -------------------------------------------------------
    io::AssetResolver assets;
    const auto root = assets.jsbsimRoot(opt.jsbsimRoot);
    if (!root) {
        LOG_ERROR("app") << "JSBSim data root not found; pass --jsbsim-root <dir>";
        return 1;
    }
    LOG_INFO("app") << "JSBSim data: " << root->string();

    // --- Build the vehicle pool -------------------------------------------
    const unsigned physical = platform::physicalCoreCount();
    const unsigned workers = opt.workers ? opt.workers : std::max(1u, physical > 2 ? physical - 2 : 1u);
    const unsigned effectiveWorkers = std::min<unsigned>(workers, opt.vehicles);

    auto ground = std::make_shared<sim::FlatGround>(0.0);
    sim::VehiclePool pool(effectiveWorkers, opt.pinWorkers);

    const sim::AircraftSpec aircraft{opt.aircraft, *root};
    platform::Stopwatch loadTimer;
    for (unsigned i = 0; i < opt.vehicles; ++i) {
        // Per-vehicle deterministic initial-condition jitter (design 6.4).
        Rng rng = Rng::forVehicle(opt.seed, 0, i);
        sim::InitialConditions ic;
        ic.latitudeDeg += rng.uniform(-0.05, 0.05);
        ic.longitudeDeg += rng.uniform(-0.05, 0.05);
        ic.altitudeMslM = 1500.0 + rng.uniform(-200.0, 200.0);
        ic.headingDeg = rng.uniform(0.0, 360.0);
        ic.airspeedTrueMs = 55.0 + rng.uniform(-5.0, 5.0);

        auto model = std::make_unique<sim::JsbsimModel>(opt.dt, ground);
        if (!model->load(aircraft, ic)) {
            LOG_ERROR("app") << "vehicle " << i << " failed to load";
            return 1;
        }
        pool.add(std::move(model));
    }
    pool.refreshStates();
    LOG_INFO("app") << opt.vehicles << " x " << opt.aircraft << " loaded in " << loadTimer.elapsedSeconds() << " s, "
                    << (effectiveWorkers <= 1 ? 1u : effectiveWorkers) << " worker(s), dt=" << opt.dt
                    << ", frame-skip=" << opt.frameSkip;

    if (!opt.benchmark) printState(0, pool.states()[0]);

    // --- Lockstep run -------------------------------------------------------
    std::vector<sim::ControlInputs> inputs(opt.vehicles);
    for (auto& in : inputs) {
        in.setThrottleAll(opt.throttle);
        in.gearDown = 0.0;
        in.elevator = 0.0;
    }

    platform::Stopwatch runTimer;
    for (unsigned step = 1; step <= opt.steps; ++step) {
        pool.step(Span<const sim::ControlInputs>(inputs.data(), inputs.size()), opt.frameSkip);
        if (!opt.benchmark && opt.printEvery && step % opt.printEvery == 0) printState(0, pool.states()[0]);
    }
    const double seconds = runTimer.elapsedSeconds();

    // --- Report -------------------------------------------------------------
    const auto states = pool.states();
    const std::size_t diverged = static_cast<std::size_t>(
        std::count_if(states.begin(), states.end(), [](const sim::VehicleState& s) { return s.diverged; }));
    const double fdmSteps = static_cast<double>(opt.vehicles) * opt.steps * opt.frameSkip;
    const auto& timing = pool.stepTiming();

    std::printf("\n%u vehicle(s) x %u agent steps x %d = %.0f vehicle-steps in %.3f s\n", opt.vehicles, opt.steps,
                opt.frameSkip, fdmSteps, seconds);
    std::printf("throughput: %.0f vehicle-steps/s   (%.1f x real time per vehicle)\n", fdmSteps / seconds,
                (fdmSteps * opt.dt / opt.vehicles) / seconds * opt.vehicles);
    std::printf("step(): mean %.3f ms, min %.3f ms, max %.3f ms over %llu calls\n", timing.meanSeconds() * 1e3,
                timing.minSeconds * 1e3, timing.maxSeconds * 1e3, static_cast<unsigned long long>(timing.samples));
    if (diverged) std::printf("WARNING: %zu vehicle(s) diverged\n", diverged);

    return diverged ? 3 : 0;
}
