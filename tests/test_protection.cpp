// Envelope protection (docs/control-architecture.md, section 11): the demand
// limited to the envelope, the state's exceedances reported - and never a
// promise that the aircraft stays inside.
#include "control/Protection.h"
#include "env/VecEnv.h"
#include "fsim/BuiltinEffects.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kG = 9.80665;

session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 5;
    return o;
}

session::VehicleSpec spec(const char* name, const char* type, double altitudeM, double tasMs) {
    session::VehicleSpec s;
    s.name = name;
    s.type = type;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = tasMs;
    return s;
}

/// Every limit given, a surface-controlled aircraft with an identified elevator.
Protection everyLimit() {
    Protection p;
    p.mode = ProtectionMode::Limit;
    EnvelopeLimits& e = p.clean;
    e.loadFactorMin = -2.0, e.loadFactorMax = 6.0;
    e.alphaMaxRad = 18.0 * kDeg;
    e.bankMaxRad = 60.0 * kDeg, e.pitchMinRad = -15.0 * kDeg, e.pitchMaxRad = 25.0 * kDeg;
    e.rollRateMaxRadS = 180.0 * kDeg;
    e.casMinMs = 40.0, e.casMaxMs = 150.0, e.machMax = 0.8;
    p.flaps = e;
    p.alphaZeroLiftRad = -3.0 * kDeg;
    p.pitchSurface = true;
    p.elevatorGainG = 4.0;
    p.elevatorGainCasMs = 100.0;
    return p;
}

sim::VehicleState randomState(std::mt19937_64& rng) {
    auto u = [&rng](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
    sim::VehicleState s;
    s.eulerRad[0] = u(-80, 80) * kDeg;
    s.eulerRad[1] = u(-30, 30) * kDeg;
    s.alphaRad = u(-5, 25) * kDeg;
    s.loadFactor = u(-3, 8);
    s.airspeedTrueMs = u(30, 250);
    s.airspeedCalibratedMs = s.airspeedTrueMs * u(0.6, 1.0);
    s.mach = s.airspeedTrueMs / 340.0;
    s.angularRateBodyRadS[0] = u(-4, 4);
    return s;
}

/// A setpoint at `level` with fields anywhere, some of them kHold.
Command randomCommand(std::mt19937_64& rng, int level) {
    auto u = [&rng](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
    auto maybe = [&](double v) { return u(0, 1) < 0.15 ? kHold : v; };
    switch (level) {
    case 0: return AttitudeCommand{maybe(u(-3, 3)), maybe(u(-1.5, 1.5)), u(0, 1) < 0.3 ? u(-3, 3) : kHold, maybe(u(0, 1.5)), kHold, maybe(u(0, 400))};
    case 1: return AccelerationCommand{maybe(u(-5, 12)), maybe(u(-6, 6)), kHold, maybe(u(0, 1))};
    case 2: return VelocityCommand{maybe(u(0, 400)), maybe(u(-80, 80)), maybe(u(-3, 3)), maybe(u(-1, 1))};
    default: return PositionCommand{0.8, 0.1, 2000.0, maybe(u(0, 400)), 200.0};
    }
}

bool within(double v, double lo, double hi, double tolerance = 1e-12) { return isHold(v) || (v >= lo - tolerance && v <= hi + tolerance); }

/// The limits that go through cos(bank) use a polynomial for it, within 0.001 (Protection.cpp).
constexpr double kCosTolerance = 1e-3;

} // namespace

TEST_CASE("protection is Limit for an aircraft with an envelope and Off without one", "[protection]") {
    session::World w(options("protection-defaults"));
    const auto stock = w.createVehicle(spec("stock", "jsbsim:c172x", 1500.0, 55.0));
    const auto viper = w.createVehicle(spec("viper", "jsbsim:f16c", 3000.0, 160.0));
    REQUIRE(viper != 0);
    CHECK(w.protection(stock) == ProtectionMode::Off);
    CHECK(w.protection(viper) == ProtectionMode::Limit);
    auto offered = [&](std::uint32_t v) {
        const auto& all = w.capabilities(v);
        return std::find_if(all.begin(), all.end(), [](const CapabilityDescriptor& d) { return d.id == "fsim.envelope.protection"; });
    };
    CHECK(offered(stock) == w.capabilities(stock).end());
    const auto p = offered(viper);
    REQUIRE(p != w.capabilities(viper).end());
    CHECK(p->kind == CapabilityKind::Status);
    CHECK(p->interactions == (kSettings | kStatus));
    CHECK(p->axes == 0); // it owns nothing: it limits what the owners demand
    REQUIRE(w.setProtection(viper, ProtectionMode::Report) == Reason::None);
    CHECK(w.protection(viper) == ProtectionMode::Report);
    CHECK(w.envelope(viper).mode == ProtectionMode::Report);
    CHECK(w.setProtection(999, ProtectionMode::Off) == Reason::UnknownVehicle);
    // the fly-by-wire law enforces its own g, alpha and roll rate
    CHECK(w.profile(viper)->envelope.lawAlpha);
    CHECK(w.profile(viper)->envelope.lawLoadFactor);
    CHECK(std::string(limitName(Limit::AlphaMax)) == "alpha_max");
}

TEST_CASE("each clamp leaves the demand within its limits, for random setpoints and states", "[protection]") {
    const Protection p = everyLimit();
    const EnvelopeLimits& e = p.clean;
    std::mt19937_64 rng(42);
    for (int i = 0; i < 20000; ++i) {
        const sim::VehicleState s = randomState(rng);
        const int level = i % 4;
        Command c = randomCommand(rng, level);
        const Command before = c;
        limitSetpoint(c, e, p, s);
        const double ratio = s.airspeedTrueMs / s.airspeedCalibratedMs;
        const double tasLo = e.casMinMs * ratio, tasHi = std::min(e.casMaxMs * ratio, e.machMax * 340.0);
        const double bankCos = std::max(std::cos(s.eulerRad[0]), 0.1);
        if (const auto* a = std::get_if<AttitudeCommand>(&c)) {
            const auto& b = std::get<AttitudeCommand>(before);
            if (isHold(a->headingRad)) CHECK(within(a->rollRad, -e.bankMaxRad, e.bankMaxRad));
            else CHECK(within(a->maxBankRad, 0.0, e.bankMaxRad));
            const double alphaPitch = s.eulerRad[1] + (e.alphaMaxRad - s.alphaRad) * bankCos;
            const double slack = kCosTolerance * std::abs(e.alphaMaxRad - s.alphaRad) + 1e-12;
            CHECK(within(a->pitchRad, std::min(e.pitchMinRad, alphaPitch), std::min(e.pitchMaxRad, alphaPitch), slack));
            // the pitch minimum yields to alpha: nothing else pushes the nose down
            if (!isHold(b.pitchRad) && b.pitchRad >= e.pitchMinRad) CHECK(a->pitchRad <= b.pitchRad);
            CHECK(within(a->airspeedMs, tasLo, tasHi));
        } else if (const auto* n = std::get_if<AccelerationCommand>(&c)) {
            double nMax = e.loadFactorMax;
            const double span = s.alphaRad - p.alphaZeroLiftRad;
            if (span > 0.035 && s.loadFactor > 0.3) nMax = std::min(nMax, s.loadFactor * (e.alphaMaxRad - p.alphaZeroLiftRad) / span);
            CHECK(within(n->loadFactorG, std::min(e.loadFactorMin, nMax), nMax));
            CHECK(within(n->rollRateRadS, -e.rollRateMaxRadS, e.rollRateMaxRadS));
        } else if (const auto* v = std::get_if<VelocityCommand>(&c)) {
            CHECK(within(v->airspeedMs, tasLo, tasHi));
            const double w = kG * std::tan(e.bankMaxRad) / std::max(s.airspeedTrueMs, 10.0);
            CHECK(within(v->turnRateRadS, -w, w));
            const double tas = std::max(s.airspeedTrueMs, 10.0), onPath = s.alphaRad * bankCos;
            CHECK(within(v->verticalSpeedMs, tas * std::sin(e.pitchMinRad - onPath), tas * std::sin(e.pitchMaxRad - onPath),
                         tas * kCosTolerance * std::abs(s.alphaRad) + 1e-9));
        } else if (const auto* q = std::get_if<PositionCommand>(&c)) {
            CHECK(within(q->airspeedMs, tasLo, tasHi));
        }
    }
}

TEST_CASE("the elevator limiter only ever moves the elevator away from the limit it nears", "[protection]") {
    const Protection p = everyLimit();
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> elevator(-1.0, 1.0);
    int down = 0, up = 0, still = 0;
    for (int i = 0; i < 20000; ++i) {
        const sim::VehicleState s = randomState(rng);
        double e = elevator(rng);
        const double before = e;
        const LimitMask hit = limitElevator(e, p.clean, p, s);
        CHECK(e >= -1.0);
        CHECK(e <= 1.0);
        const bool high = (hit & (limitBit(Limit::AlphaMax) | limitBit(Limit::LoadFactorMax))) != 0;
        if (high) { // nose-down (+) near alpha_max or n_max
            CHECK(e > before);
            ++down;
        } else if (hit) { // nose-up near n_min
            CHECK(e < before);
            ++up;
        } else {
            CHECK(e == before);
            ++still;
        }
    }
    CHECK(down > 1000);
    CHECK(up > 100);
    CHECK(still > 1000);
    // a law that enforces alpha and g: no limiter of ours; nor on a law's stick
    Protection law = p;
    law.lawEnforces = limitBit(Limit::AlphaMax) | limitBit(Limit::LoadFactorMax) | limitBit(Limit::LoadFactorMin);
    Protection stick = p;
    stick.pitchSurface = false;
    sim::VehicleState s{};
    s.alphaRad = 30.0 * kDeg, s.loadFactor = 7.0, s.airspeedCalibratedMs = 100.0;
    double e1 = -1.0, e2 = -1.0;
    CHECK(limitElevator(e1, p.clean, law, s) == 0);
    CHECK(limitElevator(e2, p.clean, stick, s) == 0);
    CHECK(e1 == -1.0);
    CHECK(e2 == -1.0);
}

TEST_CASE("Limit reduces a demand beyond the envelope and says so; Report leaves it; Off neither", "[protection]") {
    // the hangar c172: alpha_max 17.8 deg; at 55 m/s the wing gives about 4 g there
    auto fly = [](ProtectionMode mode, double& demanded, EnvelopeStatus& status, std::uint16_t& flags) {
        session::World w(options("protection-modes"));
        const auto v = w.createVehicle(spec("a", "jsbsim:c172", 1500.0, 55.0));
        REQUIRE(v != 0);
        REQUIRE(w.setProtection(v, mode) == Reason::None);
        const auto pull = w.submit(v, AccelerationCommand{6.0, 0.0, kHold, 0.8}).activity;
        REQUIRE(pull != 0);
        w.envelope(v); // from here
        for (int k = 0; k < 90; ++k) w.step();
        demanded = std::get<AccelerationCommand>(*w.controls(v)->derived(Level::Acceleration)).loadFactorG;
        status = w.envelope(v);
        flags = w.activity(pull)->constraintsSeen;
        CHECK(w.activity(pull)->state == ActivityState::Active); // whatever happened, the activity flies on
    };
    double demanded = 0.0;
    EnvelopeStatus status;
    std::uint16_t flags = 0;
    fly(ProtectionMode::Limit, demanded, status, flags);
    CHECK(demanded < 5.0); // what the wing gives at alpha_max, not 6 g
    CHECK(status[Limit::AlphaMax].limitedUpdates > 0);
    CHECK((flags & kActivityDemandLimited) != 0);
    fly(ProtectionMode::Report, demanded, status, flags);
    CHECK(demanded == 6.0);
    CHECK(status[Limit::AlphaMax].limitedUpdates == 0);
    CHECK((flags & kActivityDemandLimited) == 0);
    fly(ProtectionMode::Off, demanded, status, flags);
    CHECK(demanded == 6.0);
    CHECK(status[Limit::AlphaMax].exceededUpdates == 0); // nobody looked
    CHECK(status.mode == ProtectionMode::Off);
}

TEST_CASE("a crossing the controls did not cause is reported, and nothing else happens", "[protection]") {
    const ProtectionMode mode = GENERATE(ProtectionMode::Limit, ProtectionMode::Report);
    session::World w(options("protection-crossing"));
    const auto v = w.createVehicle(spec("a", "jsbsim:c172", 1500.0, 55.0));
    REQUIRE(w.protection(v) == ProtectionMode::Limit); // the design has an envelope
    REQUIRE(w.setProtection(v, mode) == Reason::None);
    const auto hold = w.submit(v, AttitudeCommand{0.0, 0.05, kHold, 0.785, kHold, 55.0}).activity;
    w.step(60);
    w.envelope(v);
    // a nose-up moment no elevator can hold - damage, a shifted load
    auto force = std::make_unique<effects::ConstantForce>();
    force->momentNm[1] = 25000.0;
    REQUIRE(w.addEffect(v, std::move(force)));
    double worst = 0.0;
    int beyond = 0;
    for (int k = 0; k < 150; ++k) {
        w.step();
        const double alpha = w.vehicleState(v)->alphaRad;
        if (alpha > 17.8481 * kDeg) ++beyond, worst = std::max(worst, alpha - 17.8481 * kDeg);
    }
    const EnvelopeStatus s = w.envelope(v);
    const LimitStatus& a = s[Limit::AlphaMax];
    REQUIRE(beyond > 0);
    CHECK(a.exceededUpdates >= static_cast<std::uint32_t>(beyond)); // each control update, not each step
    CHECK(std::abs(a.exceededS - a.exceededUpdates / 120.0) < 1e-9); // updates x the control period
    CHECK(a.worstExcess >= worst - 1e-6);                           // at least as far as the steps saw
    if (mode == ProtectionMode::Limit) CHECK(a.limitedUpdates > 0); // the pitch demand was cut as alpha neared
    else CHECK(a.limitedUpdates == 0);                                // Report: nothing touched
    CHECK((w.activity(hold)->constraintsSeen & kActivityExceeded) != 0);
    CHECK(w.activity(hold)->state == ActivityState::Active);          // no recovery, no end, no reset
    CHECK(w.simTime() > 6.9);
    // each read starts a new count
    const EnvelopeStatus again = w.envelope(v);
    CHECK(again[Limit::AlphaMax].exceededUpdates == 0);
    CHECK(again[Limit::AlphaMax].worstExcess == 0.0);
}

TEST_CASE("an elevator commanded directly is pushed nose-down as alpha nears its limit", "[protection]") {
    auto fly = [](ProtectionMode mode, double& highest, double& noseDown) {
        session::World w(options("protection-elevator"));
        const auto v = w.createVehicle(spec("a", "jsbsim:c172", 1500.0, 55.0));
        REQUIRE(w.setProtection(v, mode) == Reason::None);
        REQUIRE(w.submit(v, ActuatorCommand{0.0, -0.9, 0.0, 0.8}).accepted());
        highest = -1.0, noseDown = -1.0;
        for (int k = 0; k < 150; ++k) {
            w.step();
            highest = std::max(highest, w.vehicleState(v)->alphaRad);
            noseDown = std::max(noseDown, w.inputs(v)->elevator);
        }
    };
    double limited = 0.0, reported = 0.0, limitedElevator = 0.0, reportedElevator = 0.0;
    fly(ProtectionMode::Limit, limited, limitedElevator);
    fly(ProtectionMode::Report, reported, reportedElevator);
    CHECK(reportedElevator == -0.9);          // Report touches nothing
    CHECK(limitedElevator > -0.9);            // Limit eased the stick forward
    CHECK(limited < reported);                // and the wing flew at less alpha
}

TEST_CASE("protection changes nothing while no limit is near", "[protection]") {
    auto fly = [](ProtectionMode mode) {
        session::World w(options("protection-inert"));
        const auto v = w.createVehicle(spec("a", "jsbsim:f16c", 3000.0, 160.0));
        REQUIRE(w.setProtection(v, mode) == Reason::None);
        REQUIRE(w.submit(v, AttitudeCommand{0.3, 0.05, kHold, 0.785, kHold, 160.0}).accepted());
        std::vector<double> trace;
        for (int k = 0; k < 450; ++k) {
            w.step();
            const auto& s = *w.vehicleState(v);
            trace.insert(trace.end(), {w.inputs(v)->aileron, w.inputs(v)->elevator, s.eulerRad[0], s.altitudeMslM});
        }
        return trace;
    };
    CHECK(fly(ProtectionMode::Limit) == fly(ProtectionMode::Off)); // bit for bit
}

TEST_CASE("a limit the aircraft's law enforces gets its setpoint clamp and no feedback limiter of ours", "[protection]") {
    session::World w(options("protection-law"));
    const auto v = w.createVehicle(spec("viper", "jsbsim:f16c", 3000.0, 110.0));
    CommandOptions unchecked; // past the catalog's range check, which would clamp 12 g itself
    unchecked.range = RangePolicy::None;
    REQUIRE(w.submit(v, AccelerationCommand{12.0, 0.0, kHold, 1.0}, unchecked).accepted());
    double lowest = 1e9;
    for (int k = 0; k < 60; ++k) {
        w.step();
        lowest = std::min(lowest, std::get<AccelerationCommand>(*w.controls(v)->derived(Level::Acceleration)).loadFactorG);
    }
    CHECK(lowest == 9.0); // n_max, however high alpha went: the law holds alpha itself
    CHECK(w.envelope(v)[Limit::LoadFactorMax].limitedUpdates > 0);
}

TEST_CASE("VecEnv flies an aircraft's own ranges when asked", "[protection][env]") {
    env::Scenario s;
    s.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    s.aircraft = "f16c";
    s.action = "acceleration";
    s.initial.centre.altitudeMslM = 3000.0;
    s.initial.centre.airspeedTrueMs = 160.0;
    env::VecEnv::Options o;
    o.numEnvs = 1;
    o.publish = false;
    env::VecEnv e(s, o);
    e.reset(1);
    auto commanded = [&](float a) {
        const float action[3] = {a, 1.0f, 0.0f};
        e.step(Span<const float>(action, 3));
        return std::get<AccelerationCommand>(*e.world().controls(e.vehicleId(0))->activeCommand());
    };
    CHECK(commanded(1.0f).loadFactorG == 5.0); // its own: -1 .. 5 g, roll rate +-3 rad/s
    CHECK(commanded(1.0f).rollRateRadS == 3.0);
    REQUIRE(e.setActionRanges("aircraft"));
    CHECK(commanded(1.0f).loadFactorG == 9.0); // the F-16's: -3 .. 9 g, 308 deg/s
    CHECK(commanded(-1.0f).loadFactorG == -3.0);
    CHECK(std::abs(commanded(1.0f).rollRateRadS - 308.0 * kDeg) < 1e-9);
    REQUIRE(e.setActionRanges("fixed"));
    CHECK(commanded(1.0f).loadFactorG == 5.0);
    CHECK_FALSE(e.setActionRanges("roomy"));
}
