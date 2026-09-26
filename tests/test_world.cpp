// session::World: vehicles by name/type, control levels flying a real
// aircraft, environment, effects, communication, determinism, and the
// shared-memory mirror round trip.
#include "session/World.h"

#include "core/Geodesy.h"
#include "core/Units.h"
#include "fsim/BuiltinEffects.h"
#include "ipc/Recording.h"
#include "ipc/VisionSegment.h"
#include "ipc/WorldMirror.h"
#include "ipc/WorldRegistry.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace fsim;

namespace {

session::WorldOptions options(const char* name, unsigned workers = 1, bool publish = false) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = workers;
    o.pinWorkers = false;
    o.publish = publish;
    o.seed = 5;
    return o;
}

session::VehicleSpec spec(const char* name, double headingDeg = 90.0) {
    session::VehicleSpec s;
    s.name = name;
    s.type = "jsbsim:c172x";
    s.initial.altitudeMslM = 1500.0;
    s.initial.headingDeg = headingDeg;
    s.initial.airspeedTrueMs = 60.0;
    return s;
}

} // namespace

TEST_CASE("World creates, finds, resets and removes vehicles; slots are reused", "[world]") {
    session::World w(options("test-vehicles"));
    const auto a = w.createVehicle(spec("alpha"));
    const auto b = w.createVehicle(spec("bravo"));
    REQUIRE(a != 0);
    REQUIRE(b != 0);
    REQUIRE(a != b);
    REQUIRE(w.vehicleCount() == 2);
    REQUIRE(w.find("bravo") == b);
    REQUIRE(w.find("charlie") == 0);
    REQUIRE(w.createVehicle(spec("alpha")) == 0); // duplicate name
    REQUIRE(w.info(a)->type == "jsbsim:c172x");
    REQUIRE(w.vehicleState(a) != nullptr);
    REQUIRE_THAT(w.vehicleState(a)->altitudeMslM, Catch::Matchers::WithinAbs(1500.0, 1.0));

    session::VehicleSpec bad = spec("delta");
    bad.type = "jsbsim:no_such_aircraft";
    REQUIRE(w.createVehicle(bad) == 0);
    bad.type = "other:c172x";
    REQUIRE(w.createVehicle(bad) == 0);

    w.step(10);
    REQUIRE(w.simTime() > 0.0);
    REQUIRE(w.vehicleSteps() == 2 * 10 * 4);

    REQUIRE(w.removeVehicle(a));
    REQUIRE_FALSE(w.removeVehicle(a));
    REQUIRE(w.vehicleState(a) == nullptr);
    REQUIRE(w.vehicleCount() == 1);
    const auto c = w.createVehicle(spec("charlie", 180.0)); // reuses alpha's slot (same aircraft)
    REQUIRE(c != 0);
    REQUIRE(c != a);
    REQUIRE_THAT(units::radiansToDegrees(w.vehicleState(c)->eulerRad[2]), Catch::Matchers::WithinAbs(180.0, 1.0));
    REQUIRE(w.vehicleIds().size() == 2);

    sim::InitialConditions ic = spec("x").initial;
    ic.altitudeMslM = 2500.0;
    REQUIRE(w.resetVehicle(b, &ic));
    REQUIRE_THAT(w.vehicleState(b)->altitudeMslM, Catch::Matchers::WithinAbs(2500.0, 1.0));
    REQUIRE(w.info(b)->generation == 2);
}

TEST_CASE("every control level flies the aircraft", "[world]") {
    session::World w(options("test-levels", 2));
    const auto att = w.createVehicle(spec("attitude"));
    const auto vel = w.createVehicle(spec("velocity"));
    const auto pos = w.createVehicle(spec("position"));
    const auto acc = w.createVehicle(spec("acceleration"));

    REQUIRE(w.command(att, control::AttitudeCommand{units::degreesToRadians(20.0), units::degreesToRadians(2.0), control::kHold, 0.785, control::kHold, 60.0}));
    control::VelocityCommand v;
    v.airspeedMs = 60.0;
    v.verticalSpeedMs = 3.0;
    v.headingRad = units::degreesToRadians(180.0);
    REQUIRE(w.command(vel, v));
    control::PositionCommand p;
    geo::offsetLatLon(w.vehicleState(pos)->latitudeRad, w.vehicleState(pos)->longitudeRad, 4000.0, 0.0, p.latitudeRad, p.longitudeRad); // 4 km north
    p.altitudeMslM = 1700.0;
    p.airspeedMs = 60.0;
    REQUIRE(w.command(pos, p));
    control::AccelerationCommand a;
    a.loadFactorG = 1.0;
    a.rollRateRadS = 0.0;
    a.throttle = 0.7;
    REQUIRE(w.command(acc, a));
    REQUIRE_FALSE(w.command(999, a));

    w.step(static_cast<unsigned>(40.0 / (w.dt() * w.frameSkip()))); // 40 s

    const auto& sa = *w.vehicleState(att);
    REQUIRE_THAT(units::radiansToDegrees(sa.eulerRad[0]), Catch::Matchers::WithinAbs(20.0, 4.0));
    REQUIRE_THAT(sa.airspeedTrueMs, Catch::Matchers::WithinAbs(60.0, 4.0));
    REQUIRE(w.controls(att)->activeLevel() == control::Level::Attitude);

    const auto& sv = *w.vehicleState(vel);
    REQUIRE_THAT(units::radiansToDegrees(sv.eulerRad[2]), Catch::Matchers::WithinAbs(180.0, 8.0));
    REQUIRE_THAT(-sv.velocityNedMs[2], Catch::Matchers::WithinAbs(3.0, 1.5));

    const auto& sp = *w.vehicleState(pos);
    REQUIRE(sp.altitudeMslM > 1600.0); // climbing towards 1700
    const double bearing = units::radiansToDegrees(sp.eulerRad[2]);
    REQUIRE((bearing < 25.0 || bearing > 335.0)); // heading north-ish

    const auto& sc = *w.vehicleState(acc);
    REQUIRE_THAT(sc.loadFactor, Catch::Matchers::WithinAbs(1.0, 0.15));
    REQUIRE(std::abs(units::radiansToDegrees(sc.eulerRad[0])) < 10.0);
    for (auto id : {att, vel, pos, acc}) REQUIRE_FALSE(w.vehicleState(id)->diverged);
}

TEST_CASE("behaviours use other vehicles through the world view", "[world]") {
    session::World w(options("test-behaviours", 2));
    const auto leader = w.createVehicle(spec("leader"));
    auto chaser = spec("chaser");
    chaser.initial.longitudeDeg -= 0.03; // ~2.6 km west
    const auto follower = w.createVehicle(chaser);
    control::BehaviorCommand hold;
    hold.id = "hold";
    hold.params["airspeed_ms"] = 45.0; // slow leader: a c172 chaser can close at ~15 m/s
    w.command(leader, hold);
    control::BehaviorCommand pursue;
    pursue.id = "pursuit";
    pursue.target = leader;
    pursue.params["range_m"] = 100.0;
    pursue.params["max_airspeed_ms"] = 65.0;
    w.command(follower, pursue);
    const double d0 = geo::distanceM(w.vehicleState(leader)->latitudeRad, w.vehicleState(leader)->longitudeRad, w.vehicleState(follower)->latitudeRad,
                                     w.vehicleState(follower)->longitudeRad);
    w.step(static_cast<unsigned>(60.0 / (w.dt() * w.frameSkip())));
    const double d1 = geo::distanceM(w.vehicleState(leader)->latitudeRad, w.vehicleState(leader)->longitudeRad, w.vehicleState(follower)->latitudeRad,
                                     w.vehicleState(follower)->longitudeRad);
    REQUIRE(d1 < d0 * 0.75); // closing in
    REQUIRE(w.controls(follower)->activeLevel() == control::Level::Behavior);
    REQUIRE(w.controls(follower)->derived(control::Level::Position) != nullptr);
}

TEST_CASE("environment wind reaches the flight model and effects perturb it", "[world]") {
    session::World w(options("test-env"));
    const auto id = w.createVehicle(spec("v"));
    auto env = w.environment();
    env.windDirectionDeg = 90.0; // from the east: air moves west
    env.windSpeedMs = 10.0;
    w.setEnvironment(env);
    w.step(1);
    auto* model = w.model(id);
    REQUIRE(model != nullptr);
    // JSBSim wind properties are in ft/s NED.
    REQUIRE_THAT(model->property("atmosphere/wind-east-fps").get(), Catch::Matchers::WithinAbs(-10.0 * units::kMetresToFeet, 0.01));
    REQUIRE_THAT(model->property("atmosphere/wind-north-fps").get(), Catch::Matchers::WithinAbs(0.0, 0.01));

    // A constant body force changes the longitudinal acceleration.
    session::World w2(options("test-force"));
    const auto a = w2.createVehicle(spec("a"));
    const auto b = w2.createVehicle(spec("b"));
    auto force = std::make_unique<effects::ConstantForce>();
    force->forceN[0] = 3000.0; // forward push (~2.6 m/s^2 on a c172)
    REQUIRE(w2.addEffect(b, std::move(force)));
    REQUIRE_FALSE(w2.addEffect(999, std::make_unique<effects::ConstantForce>()));
    for (auto v2 : {a, b}) w2.command(v2, control::AttitudeCommand{0.0, 0.0, control::kHold, 0.785, 0.6, control::kHold}); // same attitude, same power
    w2.step(static_cast<unsigned>(10.0 / (w2.dt() * w2.frameSkip())));
    REQUIRE(w2.vehicleState(b)->airspeedTrueMs > w2.vehicleState(a)->airspeedTrueMs + 3.0);

    // Sensor noise: sensed differs from truth, truth untouched.
    session::World w3(options("test-noise"));
    const auto n = w3.createVehicle(spec("n"));
    w3.addEffectToAll([] { return std::make_unique<effects::GaussianSensorNoise>(); });
    w3.step(2);
    REQUIRE(w3.sensedState(n)->state.altitudeMslM != w3.vehicleState(n)->altitudeMslM);
    REQUIRE(w3.sensedState(n)->gnssValid);
}

TEST_CASE("vehicles exchange messages through the network", "[world]") {
    session::World w(options("test-comm"));
    const auto a = w.createVehicle(spec("a"));
    auto far = spec("b");
    far.initial.latitudeDeg += 0.01; // ~1.1 km north
    const auto b = w.createVehicle(far);
    auto& net = w.network();
    net.attach(a, std::make_unique<comm::BeaconProtocol>(0.5, 3));
    comm::Message m;
    m.from = b;
    m.to = a;
    m.channel = 9;
    m.payload = comm::JsonCodec::encode({{"hello", 1.0}, {"x", -2.5}});
    net.send(std::move(m));
    w.step(1);
    REQUIRE(net.node(a)->inbox().size() == 1);
    REQUIRE(net.node(a)->inbox()[0].channel == 9);
    comm::Fields fields;
    REQUIRE(comm::JsonCodec::decode(net.node(a)->inbox()[0].payload, fields));
    REQUIRE(fields.size() == 2);
    REQUIRE(fields[1].first == "x");
    REQUIRE(fields[1].second == -2.5);
    w.step(static_cast<unsigned>(2.0 / (w.dt() * w.frameSkip())));
    REQUIRE(net.node(b)->received() >= 3); // beacons every 0.5 s
    REQUIRE(net.node(a)->sent() >= 3);

    // A lossy, range-limited link drops everything beyond range.
    auto link = std::make_unique<comm::LinkModel>();
    link->rangeM = 1.0;
    net.setMedium(std::move(link));
    const auto before = net.node(b)->received();
    w.step(static_cast<unsigned>(2.0 / (w.dt() * w.frameSkip())));
    REQUIRE(net.node(b)->received() == before);
    REQUIRE(net.dropped() > 0);
}

TEST_CASE("trajectories with control cascades are identical across worker counts", "[world]") {
    auto run = [](unsigned workers) {
        session::World w(options("test-det", workers));
        std::vector<std::uint32_t> ids;
        for (int i = 0; i < 8; ++i) {
            auto s = spec(("v" + std::to_string(i)).c_str(), 30.0 * i);
            s.initial.longitudeDeg += 0.01 * i;
            ids.push_back(w.createVehicle(s));
        }
        control::BehaviorCommand hold;
        hold.id = "hold";
        control::CommandOptions autopilot, lateral;
        autopilot.source = control::Source::Autopilot;
        autopilot.axes = control::axisBit(control::Axis::Pitch) | control::axisBit(control::Axis::Thrust);
        lateral.axes = control::kLateral;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            switch (i % 4) {
            case 0: w.command(ids[i], control::AttitudeCommand{0.2, 0.05, control::kHold, 0.785, control::kHold, 55.0}); break;
            case 1: w.command(ids[i], hold); break;
            case 2: // the axes owned apart, merged down the cascade
                w.submit(ids[i], control::VelocityCommand{55.0, 1.0, control::kHold, control::kHold}, autopilot);
                w.submit(ids[i], control::AttitudeCommand{-0.2, control::kHold, control::kHold, 0.785, control::kHold, control::kHold}, lateral);
                break;
            default: w.setVehicleDefault(ids[i], control::VehicleDefault::Hold); break; // nothing commanded: the default's hold
            }
        }
        w.addEffectToAll([] { return std::make_unique<effects::WindGusts>(); });
        w.step(200);
        std::vector<double> trace;
        for (auto id : ids) {
            const auto& s = *w.vehicleState(id);
            trace.insert(trace.end(), {s.latitudeRad, s.longitudeRad, s.altitudeMslM, s.eulerRad[0], s.airspeedTrueMs});
        }
        return trace;
    };
    REQUIRE(run(1) == run(3));
}

TEST_CASE("behaviours that follow other vehicles see them as the step began, with any number of workers", "[world]") {
    // Followers before and after their targets in slot order, so the targets
    // fall in other workers' ranges or earlier in the same one: each must read
    // its target as the step began, not as far as a worker has got with it.
    auto run = [](unsigned workers) {
        session::World w(options("test-follow", workers));
        std::vector<std::uint32_t> ids;
        for (int i = 0; i < 8; ++i) {
            auto s = spec(("f" + std::to_string(i)).c_str(), 45.0 * i);
            s.initial.latitudeDeg += 0.004 * i;
            ids.push_back(w.createVehicle(s));
        }
        w.command(ids[7], control::VelocityCommand{58.0, 1.0, control::kHold, 0.05});
        w.command(ids[3], control::AttitudeCommand{-0.2, 0.04, control::kHold, 0.785, control::kHold, 56.0});
        auto follow = [&](std::size_t who, const char* behavior, std::size_t target) {
            control::BehaviorCommand b;
            b.id = behavior;
            b.target = ids[target];
            w.command(ids[who], b);
        };
        follow(0, "pursuit", 7);
        follow(1, "formation", 7);
        follow(2, "evade", 5);
        follow(4, "formation", 3);
        follow(5, "pursuit", 3);
        follow(6, "pursuit", 0);
        w.step(300);
        std::vector<double> trace;
        for (auto id : ids) {
            const auto& s = *w.vehicleState(id);
            trace.insert(trace.end(), {s.latitudeRad, s.longitudeRad, s.altitudeMslM, s.eulerRad[0], s.eulerRad[2], s.airspeedTrueMs});
        }
        return trace;
    };
    const auto one = run(1);
    REQUIRE(one == run(2));
    REQUIRE(one == run(4));
}

TEST_CASE("a published world is discoverable and mirrored", "[world][ipc]") {
    session::World w(options("test-mirror", 1, true));
    REQUIRE(w.published());
    bool listed = false;
    for (const auto& info : ipc::WorldRegistry::list()) listed = listed || info.name == "test-mirror";
    REQUIRE(listed);

    const auto a = w.createVehicle(spec("mirror-a"));
    ipc::WorldMirror mirror;
    REQUIRE(mirror.open("test-mirror"));
    REQUIRE(mirror.capacity() == 256);
    REQUIRE(mirror.publisherAlive());
    REQUIRE(mirror.pollTable());
    REQUIRE(mirror.vehicles()[0].alive);
    REQUIRE(mirror.vehicles()[0].name == "mirror-a");
    REQUIRE(mirror.vehicles()[0].type == "jsbsim:c172x");
    REQUIRE(mirror.vehicles()[0].id == a);
    REQUIRE_FALSE(mirror.pollTable()); // unchanged

    sim::SnapshotBatch batch;
    REQUIRE(mirror.pollSnapshot(batch)); // the creation forced a publish
    REQUIRE(batch.states.size() == 256);
    REQUIRE_THAT(batch.states[0].altitudeMslM, Catch::Matchers::WithinAbs(1500.0, 1.0));
    REQUIRE_FALSE(mirror.pollSnapshot(batch)); // nothing new

    w.command(a, control::VelocityCommand{});
    REQUIRE(mirror.pollTable());
    REQUIRE(mirror.vehicles()[0].controlLevel == static_cast<std::uint8_t>(control::Level::Velocity));

    w.step(30);
    w.publishNow();
    REQUIRE(mirror.pollSnapshot(batch));
    REQUIRE_THAT(batch.simTime, Catch::Matchers::WithinAbs(w.simTime(), 1e-9));
    REQUIRE(mirror.vehicleSteps() == w.vehicleSteps());

    auto env = w.environment();
    env.windSpeedMs = 7.5;
    w.setEnvironment(env);
    REQUIRE(mirror.environment().windSpeedMs == 7.5);

    w.removeVehicle(a);
    REQUIRE(mirror.pollTable());
    REQUIRE_FALSE(mirror.vehicles()[0].alive);
}

TEST_CASE("a world can be recorded and the recording read back", "[world][ipc]") {
    const auto path = std::filesystem::temp_directory_path() / "fsim-test-record.fsrec";
    {
        auto o = options("test-record");
        o.recordPath = path;
        session::World w(o);
        const auto a = w.createVehicle(spec("rec-a"));
        w.step(10);
        auto b = spec("rec-b");
        b.initial.longitudeDeg += 0.01;
        w.createVehicle(b);
        w.step(5);
        w.removeVehicle(a);
        w.step(5);
    }
    ipc::Recording rec;
    std::string error;
    REQUIRE(rec.load(path, &error));
    REQUIRE(rec.header().capacity == 256);
    REQUIRE(rec.frames().size() == 20);
    REQUIRE_THAT(rec.duration(), Catch::Matchers::WithinAbs(19.0 * 4.0 / 120.0, 1e-9));
    // First frame: vehicle a created (table change before it), one sample.
    REQUIRE(rec.frames()[0].tableChanges.size() == 1);
    REQUIRE(std::string(rec.frames()[0].tableChanges[0].second.name) == "rec-a");
    REQUIRE(rec.frames()[0].samples.size() == 1);
    // Frame 10: b appears; frame 15: a removed.
    REQUIRE(rec.frames()[10].tableChanges.size() == 1);
    REQUIRE(rec.frames()[10].samples.size() == 2);
    REQUIRE(rec.frames()[15].tableChanges.size() == 1);
    REQUIRE(rec.frames()[15].tableChanges[0].second.alive == 0);
    REQUIRE(rec.frames()[15].samples.size() == 1);
    REQUIRE_THAT(rec.frames()[19].samples[0].second.state.altitudeMslM, Catch::Matchers::WithinAbs(1500.0, 20.0));
    std::filesystem::remove(path);
}

TEST_CASE("a recording made before the vehicle state grew still reads", "[world][ipc]") {
    // version 1 wrote each sample's state up to rotationBodyToEcef; version 2
    // appended the engines' and moving parts' state. Rewrite a new recording
    // the way version 1 wrote it: it reads back the same, the appended fields
    // at their defaults.
    const auto v2 = std::filesystem::temp_directory_path() / "fsim-test-record-v2.fsrec";
    const auto v1 = std::filesystem::temp_directory_path() / "fsim-test-record-v1.fsrec";
    {
        auto o = options("test-record-v1");
        o.recordPath = v2;
        session::World w(o);
        w.createVehicle(spec("rec-a"));
        w.step(5);
    }
    std::vector<char> in;
    {
        std::ifstream f(v2, std::ios::binary);
        in.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    constexpr std::size_t kState = offsetof(sim::VehicleState, engineRpm);
    constexpr std::size_t kInputs = offsetof(ipc::VehicleSample, inputs);
    constexpr std::size_t kInputsV1 =
        (kState + alignof(sim::ControlInputs) - 1) / alignof(sim::ControlInputs) * alignof(sim::ControlInputs);
    struct SnapshotHead {
        double simTime;
        std::uint32_t count;
    };
    std::size_t p = 6 + sizeof(ipc::RecordingHeader);
    REQUIRE(in.size() > p);
    std::vector<char> out(in.begin(), in.begin() + static_cast<std::ptrdiff_t>(p));
    out[5] = 1;
    const auto copy = [&](std::size_t from, std::size_t n) {
        out.insert(out.end(), in.begin() + static_cast<std::ptrdiff_t>(from), in.begin() + static_cast<std::ptrdiff_t>(from + n));
    };
    while (p < in.size()) {
        if (in[p] == 'T') {
            copy(p, 1 + sizeof(std::uint32_t) + sizeof(ipc::RecordedVehicle));
            p += 1 + sizeof(std::uint32_t) + sizeof(ipc::RecordedVehicle);
            continue;
        }
        REQUIRE(in[p] == 'S');
        SnapshotHead head{};
        std::memcpy(&head, &in[p + 1], sizeof(head));
        copy(p, 1 + sizeof(head));
        p += 1 + sizeof(head);
        for (std::uint32_t i = 0; i < head.count; ++i, p += sizeof(std::uint32_t) + sizeof(ipc::VehicleSample)) {
            copy(p, sizeof(std::uint32_t) + kState); // the slot, the old state
            out.resize(out.size() + (kInputsV1 - kState), 0);
            copy(p + sizeof(std::uint32_t) + kInputs, sizeof(ipc::VehicleSample) - kInputs);
        }
    }
    {
        std::ofstream f(v1, std::ios::binary);
        f.write(out.data(), static_cast<std::streamsize>(out.size()));
    }
    ipc::Recording now, old;
    std::string error;
    REQUIRE(now.load(v2, &error));
    REQUIRE(old.load(v1, &error));
    REQUIRE(now.frames().size() == 5);
    REQUIRE(old.frames().size() == now.frames().size());
    for (std::size_t i = 0; i < now.frames().size(); ++i) {
        REQUIRE(old.frames()[i].samples.size() == 1);
        const auto& a = now.frames()[i].samples[0].second;
        const auto& b = old.frames()[i].samples[0].second;
        REQUIRE(std::memcmp(&a.state, &b.state, kState) == 0);
        REQUIRE(std::memcmp(&a.inputs, &b.inputs, sizeof(a.inputs)) == 0);
        REQUIRE(std::memcmp(a.derived, b.derived, sizeof(a.derived)) == 0);
        REQUIRE(a.state.engineRpm[0] > 0.0);
        REQUIRE(b.state.engineRpm[0] == 0.0);
        REQUIRE(b.state.leadingEdgeFlapRad == 0.0);
    }
    std::filesystem::remove(v1);
    std::filesystem::remove(v2);
}

TEST_CASE("camera images round-trip through the vision segment", "[ipc][vision]") {
    ipc::VisionPublisher pub;
    REQUIRE(pub.create("test-vision", 1u << 20));
    std::vector<ipc::VisionCameraDesc> cams(2);
    cams[0] = {7, 4, 3, "nose"};
    cams[1] = {9, 2, 2, "chase"};
    REQUIRE(pub.setCameras(cams) == 2);
    std::vector<std::uint8_t> a(4 * 3 * 3), b(2 * 2 * 3);
    for (std::size_t i = 0; i < a.size(); ++i) a[i] = static_cast<std::uint8_t>(i);
    for (std::size_t i = 0; i < b.size(); ++i) b[i] = static_cast<std::uint8_t>(200 + i);
    pub.write(0, a.data(), a.size());
    pub.write(1, b.data(), b.size());
    pub.endFrame();

    ipc::VisionMirror mirror;
    REQUIRE(mirror.open("test-vision"));
    REQUIRE(mirror.cameras().size() == 2);
    REQUIRE(mirror.cameras()[0].label == "nose");
    REQUIRE(mirror.cameras()[0].width == 4);
    REQUIRE(mirror.cameras()[1].vehicleId == 9);
    REQUIRE(mirror.frame() == 1);
    REQUIRE(mirror.publisherAlive());
    std::vector<std::uint8_t> got;
    REQUIRE(mirror.read(0, got));
    REQUIRE(got == a);
    REQUIRE(mirror.read(1, got));
    REQUIRE(got == b);
    REQUIRE_FALSE(mirror.pollTable()); // unchanged

    // A new camera set: the mirror sees the new table under a new generation.
    cams.resize(1);
    cams[0] = {11, 8, 8, "wide"};
    REQUIRE(pub.setCameras(cams) == 1);
    REQUIRE(mirror.pollTable());
    REQUIRE(mirror.cameras().size() == 1);
    REQUIRE(mirror.cameras()[0].label == "wide");
    // Too many bytes for the segment: dropped, not written.
    cams.assign(1, ipc::VisionCameraDesc{1, 1024, 1024, "huge"});
    REQUIRE(pub.setCameras(cams) == 0);
    pub.close();
    REQUIRE(mirror.pollTable());
    REQUIRE(mirror.cameras().empty());
}

TEST_CASE("an aircraft designed with hangar flies the built-in loops with its own gains", "[world]") {
    session::World w(options("test-aircraft-gains"));
    auto s = spec("viper");
    s.type = "jsbsim:f16c";
    s.initial.altitudeMslM = 3000.0;
    s.initial.airspeedTrueMs = 160.0;
    const auto viper = w.createVehicle(s);
    REQUIRE(viper != 0);
    // what the JSBSim file declares under fsim/control is what its controllers fly with
    const double tas = w.model(viper)->property("fsim/control/pid_attitude/schedule/tas_ms").get();
    const double kp = w.model(viper)->property("fsim/control/pid_velocity/vertical_speed/kp").get();
    REQUIRE(tas > 0.0);
    REQUIRE(*w.controls(viper)->controller(control::Level::Attitude)->parameter("schedule.tas_ms") == tas);
    REQUIRE(*w.controls(viper)->controller(control::Level::Velocity)->parameter("vertical_speed.kp") == kp);
    // and holds its height on them
    REQUIRE(w.command(viper, control::VelocityCommand{160.0, 0.0, control::kHold, control::kHold}));
    w.step(600);
    REQUIRE(std::abs(w.vehicleState(viper)->altitudeMslM - 3000.0) < 50.0);
    // a stock aircraft keeps the shared defaults
    const auto cessna = w.createVehicle(spec("cessna"));
    REQUIRE(*w.controls(cessna)->controller(control::Level::Attitude)->parameter("schedule.tas_ms") == 0.0);
    REQUIRE(*w.controls(cessna)->controller(control::Level::Attitude)->parameter("pitch.kp") == 2.5);
}
