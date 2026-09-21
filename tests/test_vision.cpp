// fsim_vision.dll through the public SDK: offscreen cameras on a vehicle
// (needs a Vulkan device; sky and vehicles only, so no tiles are fetched).
#include <fsim/Vision.h>
#include <fsim/World.h>

#include <cstdio>
#include <cstdlib>
#include <string>

#define CHECK(cond)                                                                  \
    do {                                                                             \
        if (!(cond)) {                                                               \
            std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #cond); \
            return 1;                                                                \
        }                                                                            \
    } while (0)

namespace {
bool skyish(const std::uint8_t* p) { return p[2] > p[0] + 20 && p[2] > 120; } // blue dominant
}

int main(int argc, char** argv) {
    using namespace fsim;
    WorldOptions wo;
    wo.name = "vision-test";
    wo.publish = false;
    wo.workers = 1;
    wo.pinWorkers = false;
    if (argc > 1) wo.jsbsimRoot = argv[1];
    World world(wo);
    world.environment().setTime(Utc{2026, 6, 21, 20, 0, 0.0}); // local noon at -122 deg
    VehicleSpec spec;
    spec.name = "cam";
    spec.type = "jsbsim:c172x";
    spec.initial.altitudeMslM = 2000.0;
    spec.initial.headingDeg = 90.0;
    spec.initial.airspeedTrueMs = 60.0;
    Vehicle v = world.createVehicle(spec);
    VehicleSpec other = spec;
    other.name = "target";
    other.initial.longitudeDeg += 0.0006; // ~50 m ahead, same track
    world.createVehicle(other);

    vision::Options vo;
    vo.earth = false;
    vision::Sensors sensors(world, vo);
    vision::CameraSpec nose;
    nose.width = 96;
    nose.height = 64;
    nose.fovDeg = 60.0;
    nose.offsetBodyM[0] = 2.0;
    const unsigned camNose = sensors.addCamera(v, nose);   // sees the target ahead, not itself
    vision::CameraSpec chase = nose;
    chase.offsetBodyM[0] = -20.0;
    chase.offsetBodyM[2] = -3.0;
    chase.pitchDeg = -8.0;
    chase.hideOwnVehicle = false;
    const unsigned camChase = sensors.addCamera(v, chase); // sees its own aircraft
    vision::CameraSpec noseSelf = nose;
    noseSelf.hideOwnVehicle = false;
    const unsigned camNoseSelf = sensors.addCamera(v, noseSelf);
    vision::CameraSpec up = nose;
    up.pitchDeg = 60.0;                                     // sky only
    const unsigned camUp = sensors.addCamera(v, up);
    CHECK(sensors.cameraCount() == 4);

    world.step();
    sensors.render();
    sensors.render();
    CHECK(sensors.frames() == 2);

    if (argc > 2) {
        sensors.savePng(camUp, std::string(argv[2]) + "/vision_up.png");
        sensors.savePng(camNose, std::string(argv[2]) + "/vision_nose.png");
        sensors.savePng(camNoseSelf, std::string(argv[2]) + "/vision_nose_self.png");
        sensors.savePng(camChase, std::string(argv[2]) + "/vision_chase.png");
    }
    const auto img = sensors.image(camNose);
    CHECK(img.rgb != nullptr && img.width == 96 && img.height == 64);
    // Sky-only camera: sky everywhere (the sun's disc may be in the frame).
    const auto sky = sensors.image(camUp);
    std::size_t skyPixels = 0;
    for (unsigned i = 0; i < sky.width * sky.height; ++i) skyPixels += skyish(sky.rgb + 3 * i) ? 1u : 0u;
    CHECK(skyPixels * 10 >= static_cast<std::size_t>(sky.width) * sky.height * 9);
    // Chase camera: its own aircraft occupies the middle of the frame (non-sky pixels there).
    const auto chaseImg = sensors.image(camChase);
    std::size_t nonSkyCentre = 0;
    for (unsigned y = chaseImg.height / 3; y < 2 * chaseImg.height / 3; ++y)
        for (unsigned x = chaseImg.width / 3; x < 2 * chaseImg.width / 3; ++x)
            nonSkyCentre += skyish(chaseImg.rgb + 3 * (y * chaseImg.width + x)) ? 0u : 1u;
    CHECK(nonSkyCentre > 20);
    // Nose camera with the own vehicle hidden: the bottom rows (where the fuselage would be) are the
    // flat below-horizon colour of the sky dome; with the vehicle drawn they are not.
    auto bottomUniform = [](const vision::Image& im) {
        const std::uint8_t* ref = im.rgb + 3 * ((im.height - 1) * im.width);
        for (unsigned y = im.height - 6; y < im.height; ++y)
            for (unsigned x = 0; x < im.width; ++x) {
                const std::uint8_t* p = im.rgb + 3 * (y * im.width + x);
                for (int c = 0; c < 3; ++c)
                    if (std::abs(static_cast<int>(p[c]) - static_cast<int>(ref[c])) > 6) return false;
            }
        return true;
    };
    std::size_t anyNonSky = 0;
    for (unsigned i = 0; i < img.width * img.height; ++i) anyNonSky += skyish(img.rgb + 3 * i) ? 0u : 1u;
    CHECK(anyNonSky > 0);
    CHECK(bottomUniform(img));
    CHECK(!bottomUniform(sensors.image(camNoseSelf)));
    std::printf("vision ok (%.1f ms per render)\n", sensors.lastRenderMs());
    return 0;
}
