// Multi-level control demo over the fsim SDK (design 9.2, 9.3): one world,
// several vehicles, each commanded at a different level of the control stack,
// plus wind, effects and a beacon protocol. Start flightsim-viewer.exe in
// another window at any time to watch it.
//
//   multi_level_control [--seconds S] [--realtime] [--aircraft c172x] [--name demo] [--extra N] [--quiet] [--terrain]
//                       [--record file.fsrec] [--bridge <local_port>:<peer_port>]
//
// --bridge adds an external node (address 1000) whose traffic goes over UDP:
// every beacon reaches the peer as an "FSMG" datagram and whatever the peer
// sends arrives in the vehicles' inboxes (see examples/udp_peer).

#include <fsim/BuiltinEffects.h>
#include <fsim/World.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

using namespace fsim;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

struct Args {
    double seconds = 120.0;
    bool realtime = false;
    std::string aircraft = "c172x";
    std::string name = "demo";
    int extra = 0; ///< additional vehicles on the "hold" behaviour (throughput tests)
    bool quiet = false;
    bool terrain = false;
    std::string record;
    std::string bridge; ///< "<local_port>:<peer_port>"
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (k == "--seconds") a.seconds = std::atof(next());
        else if (k == "--realtime") a.realtime = true;
        else if (k == "--aircraft") a.aircraft = next();
        else if (k == "--name") a.name = next();
        else if (k == "--extra") a.extra = std::atoi(next());
        else if (k == "--quiet") a.quiet = true;
        else if (k == "--terrain") a.terrain = true;
        else if (k == "--record") a.record = next();
        else if (k == "--bridge") a.bridge = next();
    }
    return a;
}

void report(double t, const Vehicle& v) {
    const auto& s = v.state();
    std::printf("  t=%6.1f %-10s %-12s alt %7.1f m  hdg %5.1f  tas %5.1f m/s  roll %6.1f  pitch %5.1f  vz %5.1f%s\n", t, v.name().c_str(),
                control::levelName(v.activeLevel()), s.altitudeMslM, s.eulerRad[2] / kDeg, s.airspeedTrueMs, s.eulerRad[0] / kDeg,
                s.eulerRad[1] / kDeg, -s.velocityNedMs[2], s.diverged ? "  DIVERGED" : "");
}

} // namespace

int main(int argc, char** argv) {
    const Args args = parse(argc, argv);

    WorldOptions wo;
    wo.name = args.name;
    wo.frameSkip = 4; // 30 Hz world steps
    wo.terrain = args.terrain; // real relief under the vehicles (downloads tiles on first use)
    wo.recordPath = args.record; // replay later with flightsim-viewer --replay <file>
    World world(wo);
    std::printf("fsim %s: world '%s' %s\n", version(), world.name().c_str(), world.published() ? "(published for viewers)" : "(not published)");

    // Environment: a spring afternoon with a 6 m/s westerly and light turbulence.
    world.environment().setTime(Utc{2026, 4, 12, 22, 30, 0});
    world.environment().setWind(Wind{270.0, 6.0, 0.0, 0.15});

    // Vehicles: same aircraft, different control levels.
    const std::string type = "jsbsim:" + args.aircraft;
    auto spawn = [&](const char* name, double dLat, double dLon, double headingDeg) {
        VehicleSpec spec;
        spec.name = name;
        spec.type = type;
        spec.initial.latitudeDeg = 37.62 + dLat;
        spec.initial.longitudeDeg = -122.38 + dLon;
        spec.initial.altitudeMslM = 1200.0;
        spec.initial.headingDeg = headingDeg;
        spec.initial.airspeedTrueMs = 60.0;
        return world.createVehicle(spec);
    };
    Vehicle attitude = spawn("attitude", 0.00, 0.00, 90.0);
    Vehicle velocity = spawn("velocity", 0.01, 0.00, 90.0);
    Vehicle route = spawn("waypoints", 0.02, 0.00, 90.0);
    Vehicle chaser = spawn("pursuit", -0.01, -0.02, 90.0);
    Vehicle loiter = spawn("loiter", 0.03, 0.01, 180.0);

    // 1. Attitude level: gentle right bank, slight nose-up, airspeed hold.
    attitude.command(control::AttitudeCommand{15.0 * kDeg, 3.0 * kDeg, control::kHold, 0.785, control::kHold, 60.0});
    // 2. Velocity level: climb at 3 m/s on heading 045 at 65 m/s.
    velocity.command(control::VelocityCommand{65.0, 3.0, 45.0 * kDeg, control::kHold});
    // 3. Behaviour: a looping triangle of waypoints.
    control::BehaviorCommand wp;
    wp.id = "waypoints";
    wp.params["loop"] = 1.0;
    for (auto [dLat, dLon, alt] : {std::tuple{0.03, 0.02, 1400.0}, std::tuple{0.00, 0.04, 1500.0}, std::tuple{0.00, 0.00, 1300.0}}) {
        control::PositionCommand p;
        p.latitudeRad = (37.62 + dLat) * kDeg;
        p.longitudeRad = (-122.38 + dLon) * kDeg;
        p.altitudeMslM = alt;
        p.airspeedMs = 60.0;
        p.captureRadiusM = 250.0;
        wp.points.push_back(p);
    }
    route.command(wp);
    // 4. Behaviour: chase the attitude-level vehicle at 200 m.
    control::BehaviorCommand pursue;
    pursue.id = "pursuit";
    pursue.target = attitude.id();
    pursue.params["range_m"] = 200.0;
    pursue.params["max_airspeed_ms"] = 70.0;
    chaser.command(pursue);
    // 5. Behaviour: orbit a point.
    control::BehaviorCommand orbit;
    orbit.id = "loiter";
    orbit.params["radius_m"] = 1200.0;
    orbit.params["altitude_m"] = 1350.0;
    loiter.command(orbit);

    // Extra vehicles holding their initial course (for throughput tests).
    for (int i = 0; i < args.extra; ++i) {
        Vehicle v = spawn(("hold-" + std::to_string(i)).c_str(), -0.05 + 0.002 * (i % 50), 0.05 + 0.002 * (i / 50), 270.0);
        control::BehaviorCommand hold;
        hold.id = "hold";
        v.command(hold);
    }

    // Effects and communication: noisy sensors on the chaser, gusts everywhere, beacons on all.
    chaser.addEffect<effects::GaussianSensorNoise>();
    world.addEffectToAll<effects::WindGusts>();
    for (auto& v : world.vehicles()) world.network().attach(v.id(), std::make_unique<comm::BeaconProtocol>(2.0));
    if (!args.bridge.empty()) {
        const auto colon = args.bridge.find(':');
        const auto local = static_cast<std::uint16_t>(std::atoi(args.bridge.substr(0, colon).c_str()));
        const auto peer = static_cast<std::uint16_t>(colon == std::string::npos ? local + 1 : std::atoi(args.bridge.substr(colon + 1).c_str()));
        std::string error;
        auto transport = comm::createUdpTransport(local, "127.0.0.1", peer, &error);
        if (!transport) {
            std::fprintf(stderr, "bridge: %s\n", error.c_str());
            return 1;
        }
        world.network().createNode(1000);
        world.network().attach(1000, std::make_unique<comm::BridgeProtocol>(std::move(transport)));
        std::printf("bridge: node 1000 on udp port %u -> 127.0.0.1:%u\n", unsigned(local), unsigned(peer));
    }

    const double stepS = world.stepSeconds();
    const int steps = static_cast<int>(args.seconds / stepS);
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 1; k <= steps; ++k) {
        world.step();
        if (!args.quiet && k % static_cast<int>(10.0 / stepS) == 0) {
            std::printf("--- %.0f s (received beacons: %llu on '%s')\n", world.time(), static_cast<unsigned long long>(chaser.node()->received()),
                        chaser.name().c_str());
            for (auto& v : world.vehicles()) report(world.time(), v);
        }
        if (args.realtime) {
            const auto target = t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(k * stepS));
            std::this_thread::sleep_until(target);
        }
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("%d world steps, %llu vehicle-steps in %.2f s (%.0fx real time)\n", steps, static_cast<unsigned long long>(world.vehicleSteps()), wall,
                args.seconds / wall);
    return 0;
}
