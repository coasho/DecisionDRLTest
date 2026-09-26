// A short manoeuvre suite for CI (docs/control-architecture.md, section 14,
// step 6): one hangar design per adapter it flies through - the B-52H through
// jsbsim.direct, the F-16C through jsbsim.fbw - flown through hangar's
// manoeuvres at its reference speed (tools/hangar/hangar/autopilot.py,
// Evaluate), each after 40 s of velocity hold from a fresh start, and judged by
// the suite's rule. A step fails if it loses control, never reaches 90 % of its
// target, or overshoots it by more than half; the hold fails if it drifts over
// 100 m or oscillates (vertical speed sd over 1 m/s, bank sd over 2 deg); and
// the 90 deg turn must end within 3 deg of it. The stock adapter's c172x flies
// the shared gains, frozen with its flights (C1), so its checkpoints hold it
// exactly instead (test_control_compat.cpp).
#include "control/Adapter.h"
#include "fsim/Control.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kG = 9.80665;
constexpr double kInf = std::numeric_limits<double>::infinity();

session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 7;
    return o;
}

session::VehicleSpec spec(const std::string& name, const char* type, double altitudeM, double tasMs, double northDeg) {
    session::VehicleSpec s;
    s.name = name;
    s.type = type;
    s.initial.latitudeDeg += northDeg;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 0.0; // hangar's: north, and the hold keeps it
    s.initial.airspeedTrueMs = tasMs;
    return s;
}

/// A step from 0 towards `target` (hangar's step_metrics).
struct Step {
    bool lost = false;
    double riseS = kInf;    ///< to 90 % of the target
    double overshoot = 0.0; ///< beyond it, a fraction of the step
    bool passes() const noexcept { return !lost && std::isfinite(riseS) && overshoot <= 0.5; }
};

Step stepOf(const std::vector<double>& y, double target, double dt, bool lost) {
    Step s;
    s.lost = lost || y.size() < 10;
    if (s.lost) return s;
    double most = -kInf;
    for (std::size_t i = 0; i < y.size(); ++i) {
        const double progress = y[i] / target;
        if (!std::isfinite(s.riseS) && progress >= 0.9) s.riseS = static_cast<double>(i + 1) * dt;
        most = std::max(most, progress);
    }
    s.overshoot = std::max(0.0, most - 1.0);
    return s;
}

double deviation(const std::vector<double>& x) {
    const double mean = std::accumulate(x.begin(), x.end(), 0.0) / static_cast<double>(x.size());
    double sum = 0.0;
    for (const double v : x) sum += (v - mean) * (v - mean);
    return std::sqrt(sum / static_cast<double>(x.size()));
}

enum Manoeuvre : std::size_t { Bank, Pitch, Climb, LoadFactor, RollRate, Turn, kManoeuvres };
constexpr const char* kNames[kManoeuvres] = {"30 deg bank", "5 deg pitch", "climb at 5 % of the speed", "1.5 g", "roll rate 0.25 rad/s",
                                             "90 deg turn"};

/// What one aircraft did.
struct Flown {
    std::string family; ///< the adapter it flew through
    double speedMs = 0.0;
    bool holdLost = false;
    double heightChangeM = 0.0, verticalSpeedSd = 0.0, bankSdDeg = 0.0;
    std::array<Step, kManoeuvres - 1> steps; ///< Bank .. RollRate
    bool turnLost = false;
    double turnEndDeg = 0.0; ///< the heading change the turn ended at
};

Flown fly(const char* type) {
    session::World w(options("manoeuvres"));
    const double dt = w.dt() * w.frameSkip(); // a world step
    // the reference condition hangar identified the aircraft at; the c172x's own for the stock aircraft
    double altitudeM = 1500.0, speed = 55.0;
    {
        const auto probe = w.createVehicle(spec("probe", type, 3000.0, 150.0, 0.0));
        REQUIRE(probe != 0);
        const PlantSection& plant = w.profile(probe)->plant;
        if (plant.header.present() && std::isfinite(plant.tasMs)) altitudeM = plant.altitudeM, speed = plant.tasMs;
        REQUIRE(w.removeVehicle(probe));
    }
    Flown out;
    out.speedMs = speed;
    // a vehicle per manoeuvre, 40 s of velocity hold for each
    std::array<std::uint32_t, kManoeuvres> v{};
    for (std::size_t m = 0; m < kManoeuvres; ++m) {
        v[m] = w.createVehicle(spec(std::string("m") + std::to_string(m), type, altitudeM, speed, 0.03 * static_cast<double>(m + 1)));
        REQUIRE(v[m] != 0);
        REQUIRE(w.command(v[m], VelocityCommand{speed, 0.0, 0.0, kHold}));
    }
    out.family = adapterFor(w.profile(v[0])->identity.family).family();
    auto lost = [&](std::uint32_t id) {
        const auto& s = *w.vehicleState(id);
        return s.diverged || !(s.altitudeMslM >= 200.0);
    };
    const int settle = static_cast<int>(std::lround(40.0 / dt));
    std::vector<double> vz, bank;
    for (int k = 0; k < settle; ++k) {
        w.step();
        const auto& s = *w.vehicleState(v[0]);
        if (lost(v[0])) {
            out.holdLost = true;
            break;
        }
        vz.push_back(-s.velocityNedMs[2]);
        bank.push_back(s.eulerRad[0]);
    }
    if (!out.holdLost) {
        const auto quarter = static_cast<std::ptrdiff_t>(vz.size() * 3 / 4);
        out.verticalSpeedSd = deviation(std::vector<double>(vz.begin() + quarter, vz.end()));
        out.bankSdDeg = deviation(std::vector<double>(bank.begin() + quarter, bank.end())) / kDeg;
        out.heightChangeM = w.vehicleState(v[0])->altitudeMslM - altitudeM;
    }

    // then each its manoeuvre
    std::array<sim::VehicleState, kManoeuvres> s0;
    for (std::size_t m = 0; m < kManoeuvres; ++m) s0[m] = *w.vehicleState(v[m]);
    const double maxBank = w.controls(v[Turn])->controller(Level::Velocity)->parameter("max_bank").value_or(0.5);
    const double turnS = 0.5 * kPi / (kG * std::tan(maxBank) / speed);
    const double seconds[kManoeuvres] = {12.0, 12.0, 20.0, 4.0, 3.0, turnS + 30.0};
    REQUIRE(w.command(v[Bank], AttitudeCommand{30.0 * kDeg, s0[Bank].eulerRad[1], kHold, 0.785, kHold, speed}));
    REQUIRE(w.command(v[Pitch], AttitudeCommand{0.0, s0[Pitch].eulerRad[1] + 5.0 * kDeg, kHold, 0.785, kHold, speed}));
    REQUIRE(w.command(v[Climb], VelocityCommand{speed, 0.05 * speed, 0.0, kHold}));
    REQUIRE(w.command(v[LoadFactor], AccelerationCommand{1.5, 0.0, kHold, kHold}));
    REQUIRE(w.command(v[RollRate], AccelerationCommand{1.0, 0.25, kHold, kHold}));
    REQUIRE(w.command(v[Turn], VelocityCommand{speed, 0.0, s0[Turn].eulerRad[2] + 90.0 * kDeg, kHold}));
    std::array<std::vector<double>, kManoeuvres> y;
    std::array<bool, kManoeuvres> gone{};
    double turned = 0.0, lastHeading = s0[Turn].eulerRad[2];
    const double longest = *std::max_element(std::begin(seconds), std::end(seconds));
    for (long k = 0; k < std::lround(longest / dt); ++k) {
        w.step();
        const double t = static_cast<double>(k + 1) * dt;
        for (std::size_t m = 0; m < kManoeuvres; ++m) {
            if (gone[m] || t > seconds[m] + 1e-9) continue;
            if (lost(v[m])) {
                gone[m] = true;
                continue;
            }
            const auto& s = *w.vehicleState(v[m]);
            switch (m) {
            case Bank: y[m].push_back(s.eulerRad[0] / kDeg); break;
            case Pitch: y[m].push_back((s.eulerRad[1] - s0[m].eulerRad[1]) / kDeg); break;
            case Climb: y[m].push_back(-s.velocityNedMs[2]); break;
            case LoadFactor: y[m].push_back(s.loadFactor - 1.0); break;
            case RollRate: y[m].push_back(s.angularRateBodyRadS[0]); break;
            default: {
                double d = s.eulerRad[2] - lastHeading;
                while (d > kPi) d -= 2.0 * kPi;
                while (d < -kPi) d += 2.0 * kPi;
                lastHeading = s.eulerRad[2];
                turned += d;
                y[m].push_back(turned / kDeg);
            }
            }
        }
    }
    const double targets[kManoeuvres - 1] = {30.0, 5.0, 0.05 * speed, 0.5, 0.25};
    for (std::size_t m = 0; m < Turn; ++m) out.steps[m] = stepOf(y[m], targets[m], dt, gone[m]);
    out.turnLost = gone[Turn] || y[Turn].empty();
    if (!out.turnLost) out.turnEndDeg = y[Turn].back();
    return out;
}

} // namespace

TEST_CASE("conformance: one design per adapter flies hangar's manoeuvres at its reference speed within the suite's rule", "[conformance]") {
    struct Aircraft {
        const char* type;
        const char* family;
    };
    for (const Aircraft& a : {Aircraft{"jsbsim:b52h", "jsbsim.direct"}, Aircraft{"jsbsim:f16c", "jsbsim.fbw"}}) {
        INFO(a.type << " (" << a.family << ")");
        const Flown f = fly(a.type);
        CHECK(f.family == a.family);
        INFO("at " << f.speedMs << " m/s");
        {
            INFO("velocity hold, 40 s: height " << f.heightChangeM << " m, vertical speed sd " << f.verticalSpeedSd << " m/s, bank sd "
                                               << f.bankSdDeg << " deg");
            CHECK_FALSE(f.holdLost);
            CHECK(std::abs(f.heightChangeM) <= 100.0);
            CHECK(f.verticalSpeedSd <= 1.0);
            CHECK(f.bankSdDeg <= 2.0);
        }
        for (std::size_t m = 0; m < Turn; ++m) {
            const Step& s = f.steps[m];
            INFO(kNames[m] << ": " << (s.lost ? "lost" : "") << " rise " << s.riseS << " s, overshoot " << 100.0 * s.overshoot << " %");
            CHECK(s.passes());
        }
        INFO("90 deg turn: ended at " << f.turnEndDeg << " deg");
        CHECK_FALSE(f.turnLost);
        CHECK(std::abs(f.turnEndDeg - 90.0) <= 3.0);
    }
}
