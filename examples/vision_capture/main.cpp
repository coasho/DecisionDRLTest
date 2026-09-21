// Vehicle cameras (design 8.4): a world with a few aircraft over real terrain,
// cameras mounted on one of them (nose, chase, downward), rendered offscreen
// each step and written as PNGs. No window; fsim_vision.dll only needs a
// Vulkan device.
//
//   vision_capture [--lat 37.72 --lon -119.55 --alt 3200] [--seconds 20] [--every 2] [--width 320 --height 240]
//                  [--out captures] [--terrain] [--quiet] [--fleet N] [--realtime] [--no-save]
//
// --fleet N adds N more aircraft, each with a 128x128 nose camera that is
// rendered but not saved: the throughput case (64 cameras per step).

#include <fsim/Vision.h>
#include <fsim/World.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>

using namespace fsim;

int main(int argc, char** argv) {
    double lat = 37.72, lon = -119.55, alt = 3200.0, seconds = 20.0, every = 2.0;
    unsigned width = 320, height = 240;
    std::string out = "captures";
    bool terrain = false, quiet = false, realtime = false, save = true;
    unsigned fleet = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (k == "--lat") lat = std::atof(next());
        else if (k == "--lon") lon = std::atof(next());
        else if (k == "--alt") alt = std::atof(next());
        else if (k == "--seconds") seconds = std::atof(next());
        else if (k == "--every") every = std::atof(next());
        else if (k == "--width") width = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--height") height = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--out") out = next();
        else if (k == "--terrain") terrain = true;
        else if (k == "--quiet") quiet = true;
        else if (k == "--fleet") fleet = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--realtime") realtime = true;
        else if (k == "--no-save") save = false;
    }
    try {
        WorldOptions wo;
        wo.name = "vision";
        wo.terrain = terrain;
        World world(wo);
        world.environment().setTime(Utc{2026, 6, 21, 17, 30, 0.0}); // late afternoon sun
        auto spawn = [&](const char* name, double dLat, double dLon, double heading) {
            VehicleSpec s;
            s.name = name;
            s.type = "jsbsim:c172x";
            s.initial.latitudeDeg = lat + dLat;
            s.initial.longitudeDeg = lon + dLon;
            s.initial.altitudeMslM = alt;
            s.initial.headingDeg = heading;
            s.initial.airspeedTrueMs = 55.0;
            return world.createVehicle(s);
        };
        Vehicle lead = spawn("lead", 0.0, 0.0, 90.0);
        Vehicle wing = spawn("wing", -0.0008, 0.0003, 90.0); // right and slightly behind
        Vehicle high = spawn("high", 0.002, 0.004, 270.0);
        for (Vehicle v : {lead, wing, high}) {
            control::AttitudeCommand hold;
            hold.rollRad = 0.0;
            hold.pitchRad = 0.03;
            v.command(hold);
        }

        vision::Options vo;
        vision::Sensors sensors(world, vo);
        vision::CameraSpec nose;
        nose.width = width; nose.height = height; nose.fovDeg = 70.0;
        nose.offsetBodyM[0] = 2.0; nose.offsetBodyM[2] = -0.3;
        nose.depth = true; // metres per pixel as well
        const unsigned camNose = sensors.addCamera(lead, nose);
        vision::CameraSpec chase = nose;
        chase.offsetBodyM[0] = -25.0; chase.offsetBodyM[2] = -6.0; chase.pitchDeg = -10.0;
        const unsigned camChase = sensors.addCamera(lead, chase);
        vision::CameraSpec down = nose;
        down.pitchDeg = -90.0; down.fovDeg = 60.0;
        const unsigned camDown = sensors.addCamera(lead, down);
        vision::CameraSpec side = nose;
        side.yawDeg = 90.0; side.pitchDeg = -15.0;
        const unsigned camSide = sensors.addCamera(lead, side);
        for (unsigned i = 0; i < fleet; ++i) {
            Vehicle v = spawn(("fleet-" + std::to_string(i)).c_str(), 0.004 * (i / 8 + 1), 0.003 * (i % 8), 90.0);
            control::AttitudeCommand hold;
            hold.pitchRad = 0.03;
            v.command(hold);
            vision::CameraSpec small;
            small.offsetBodyM[0] = 2.0;
            sensors.addCamera(v, small);
        }

        std::filesystem::create_directories(out);
        // Let the tiles around the start stream in before the first images.
        sensors.render();
        sensors.settle(60);

        const double stepS = world.stepSeconds();
        const int steps = static_cast<int>(seconds / stepS);
        const int everySteps = std::max(1, static_cast<int>(every / stepS));
        int shot = 0;
        double renderMs = 0.0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int k = 1; k <= steps; ++k) {
            world.step();
            if (realtime) std::this_thread::sleep_until(t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(k * stepS)));
            if (k % everySteps != 0) continue;
            sensors.render();
            renderMs += sensors.lastRenderMs();
            char name[128];
            if (save) {
                for (auto [cam, tag] : {std::pair{camNose, "nose"}, std::pair{camChase, "chase"}, std::pair{camDown, "down"}, std::pair{camSide, "side"}}) {
                    std::snprintf(name, sizeof name, "%s/%s_%03d.png", out.c_str(), tag, shot);
                    sensors.savePng(cam, name);
                }
                std::snprintf(name, sizeof name, "%s/nose_depth_%03d.png", out.c_str(), shot);
                sensors.saveDepthPng(camNose, name);
            }
            if (!quiet) {
                const auto d = sensors.depth(camNose);
                const auto at = [&](unsigned x, unsigned y) { return static_cast<double>(d.metres[y * d.width + x]); };
                std::printf("   depth: top %.1f  centre %.1f  bottom %.1f m" "\n", at(d.width / 2, 2), at(d.width / 2, d.height / 2),
                            at(d.width / 2, d.height - 3));
            }
            if (!quiet) {
                const auto& s = lead.state();
                const auto t = sensors.timing();
                std::printf("t=%5.1f  shot %03d  alt %6.0f m  agl %6.0f m  hdg %5.1f  render %.1f ms (advance %.1f update %.1f record %.1f gpu %.1f copy %.1f)\n",
                            world.time(), shot, s.altitudeMslM, s.altitudeAglM, s.eulerRad[2] * 57.2958, sensors.lastRenderMs(), t.advance, t.update, t.record,
                            t.wait, t.readback);
            }
            ++shot;
        }
        std::printf("%d shots x %zu cameras in %s, %.1f ms per render\n", shot, sensors.cameraCount(), out.c_str(), shot ? renderMs / shot : 0.0);
        return 0;
    } catch (const Error& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
