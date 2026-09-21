// fsim_vision.dll through the public SDK: offscreen cameras on a vehicle
// (needs a Vulkan device; sky and vehicles only, so no tiles are fetched).
#include <fsim/VecEnv.h>
#include <fsim/Vision.h>
#include <fsim/World.h>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <cstdlib>
#include <string>
#include <vector>

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
    Vehicle target = world.createVehicle(other);

    vision::Options vo;
    vo.earth = false;
    vo.segmentation = true;
    std::unique_ptr<vision::Sensors> sensorsPtr;
    try {
        sensorsPtr = std::make_unique<vision::Sensors>(world, vo);
    } catch (const Error& e) {
        std::fprintf(stderr, "skipping: %s" "\n", e.what());
        return 77; // no Vulkan device here (CI runners); ctest treats 77 as skipped
    }
    vision::Sensors& sensors = *sensorsPtr;
    vision::CameraSpec nose;
    nose.width = 96;
    nose.height = 64;
    nose.fovDeg = 60.0;
    nose.offsetBodyM[0] = 2.0;
    nose.depth = true;
    nose.segmentation = true;
    const unsigned camNose = sensors.addCamera(v, nose);   // sees the target ahead, not itself
    vision::CameraSpec chase = nose;
    chase.offsetBodyM[0] = -20.0;
    chase.offsetBodyM[2] = -3.0;
    chase.pitchDeg = -8.0;
    chase.hideOwnVehicle = false;
    chase.depth = false;
    chase.segmentation = true;
    const unsigned camChase = sensors.addCamera(v, chase); // sees its own aircraft
    vision::CameraSpec noseSelf = nose;
    noseSelf.hideOwnVehicle = false;
    const unsigned camNoseSelf = sensors.addCamera(v, noseSelf);
    vision::CameraSpec up = nose;
    up.pitchDeg = 60.0;                                     // sky only
    up.segmentation = false;
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
        sensors.saveSegmentationPng(camNose, std::string(argv[2]) + "/vision_nose_seg.png");
        sensors.saveSegmentationPng(camChase, std::string(argv[2]) + "/vision_chase_seg.png");
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
    // Depth: the sky dome is tens of km away, the target aircraft ahead is within 200 m of the nose camera,
    // and the nearest pixel of the frame belongs to it.
    const auto dimg = sensors.depth(camNose);
    CHECK(dimg.metres != nullptr && dimg.size() == img.width * img.height);
    CHECK(sensors.depth(camChase).metres == nullptr); // not requested
    float nearest = 1e30f;
    for (std::size_t i = 0; i < dimg.size(); ++i) nearest = std::min(nearest, dimg.metres[i]);
    CHECK(nearest > 5.0f && nearest < 200.0f);
    CHECK(dimg.metres[2 * dimg.width + dimg.width / 2] > 10000.0f);
    // Segmentation: ids name the vehicle a pixel belongs to, 0 elsewhere.
    const unsigned idSelf = sensors.segmentationId(v), idTarget = sensors.segmentationId(target);
    CHECK(idSelf != 0 && idTarget != 0 && idSelf != idTarget);
    const auto segNose = sensors.segmentation(camNose);
    CHECK(segNose.ids != nullptr && segNose.size() == img.width * img.height);
    CHECK(sensors.segmentation(camUp).ids == nullptr); // not requested
    // The nose camera hides its own aircraft, so every labelled pixel is the target.
    std::size_t targetPixels = 0, foreignPixels = 0;
    for (std::size_t i = 0; i < segNose.size(); ++i) {
        if (segNose.ids[i] == idTarget) ++targetPixels;
        else if (segNose.ids[i] != 0) ++foreignPixels;
    }
    CHECK(targetPixels > 0);
    CHECK(foreignPixels == 0);
    // Those are exactly the pixels the depth image finds near, and the sky is unlabelled.
    for (std::size_t i = 0; i < segNose.size(); ++i)
        CHECK((segNose.ids[i] == idTarget) == (dimg.metres[i] < 1000.0f));
    // The chase camera draws its own aircraft, so the centre of its frame is that vehicle.
    const auto segChase = sensors.segmentation(camChase);
    std::size_t selfCentre = 0;
    for (unsigned y = segChase.height / 3; y < 2 * segChase.height / 3; ++y)
        for (unsigned x = segChase.width / 3; x < 2 * segChase.width / 3; ++x)
            selfCentre += segChase.ids[y * segChase.width + x] == idSelf ? 1u : 0u;
    CHECK(selfCentre > 20);

    // Cameras added after rendering and removed later: the scene recompiles on the next render.
    {
        vision::CameraSpec late = up;
        late.width = 32;
        late.height = 32;
        const unsigned camLate = sensors.addCamera(v, late);
        CHECK(camLate == 4);
        CHECK(sensors.image(camLate).width == 32); // buffer exists (zeros) before the first render
        sensors.render();
        const auto lateImg = sensors.image(camLate);
        CHECK(lateImg.rgb != nullptr && lateImg.width == 32 && lateImg.height == 32);
        CHECK(skyish(lateImg.rgb + 3 * (16 * 32 + 16)));
        CHECK(sensors.image(camNose).width == 96); // the others still render
        sensors.removeCamera(camLate);
        sensors.render();
        CHECK(sensors.image(camLate).rgb == nullptr);
        CHECK(sensors.image(camNose).rgb != nullptr);
        CHECK(sensors.cameraCount() == 5);
    }

    // A VecEnv exposes its world: one camera per batch vehicle.
    {
        VecEnvOptions vo2;
        vo2.numEnvs = 2;
        vo2.workers = 1;
        vo2.publish = false;
        vo2.worldName = "vision-batch";
        vo2.jsbsimRoot = wo.jsbsimRoot;
        VecEnv env(vo2);
        World& bw = env.world();
        CHECK(bw.vehicleCount() == 2);
        CHECK(bw.vehicle("env1/0").valid());
        vision::Sensors batchSensors(bw, vo);
        std::vector<unsigned> cams;
        for (Vehicle bv : bw.vehicles()) cams.push_back(batchSensors.addCamera(bv, nose));
        env.reset(3);
        env.step(std::vector<float>(env.numVehicles() * env.actionSize(), 0.0f));
        batchSensors.render();
        for (unsigned c : cams) CHECK(batchSensors.image(c).rgb != nullptr);
        CHECK(batchSensors.image(cams[0]).width == 96);

        // BatchCameras packs the same images as tensors in batch order.
        vision::CameraSpec bspec = up;
        bspec.width = 24;
        bspec.height = 16;
        bspec.depth = true;
        vision::BatchCameras batch(env, bspec, vo);
        CHECK(batch.count() == 2 && batch.width() == 24 && batch.height() == 16);
        batch.render();
        CHECK(batch.rgb().size == 2 * 24 * 16 * 3);
        CHECK(batch.depth().size == 2 * 24 * 16);
        const auto one = batch.sensors().image(batch.camera(1));
        CHECK(one.rgb != nullptr);
        std::size_t same = 0;
        for (std::size_t i = 0; i < one.size(); ++i) same += batch.rgb()[24 * 16 * 3 + i] == one.rgb[i] ? 1u : 0u;
        CHECK(same == one.size());
        CHECK(batch.depth()[0] > 1000.0f); // sky
    }
    std::printf("vision ok (%.1f ms per render)\n", sensors.lastRenderMs());
    return 0;
}
