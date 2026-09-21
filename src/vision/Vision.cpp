// fsim vision SDK: cameras on vehicles over the public World API, rendered
// by vision::Offscreen with the viewer's scene builders (design 8.4).

#include "fsim/Vision.h"

#include "core/Log.h"
#include "io/AssetResolver.h"
#include "ipc/VisionSegment.h"
#include "vision/Offscreen.h"
#include "world/Earth.h"
#include "world/Frames.h"
#include "world/Sky.h"
#include "world/SkyDome.h"
#include "world/Terrain.h"
#include "world/VehicleVisuals.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC // vsgXchange carries its own copy
#define STBI_WRITE_NO_STDIO
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#include <stb_image_write.h>
#pragma GCC diagnostic pop

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <unordered_map>

namespace fsim::vision {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
}

struct Sensors::Impl {
    World* world = nullptr;
    Options options;
    Offscreen offscreen;
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid;
    std::unique_ptr<world::VehicleVisuals> visuals;
    std::unique_ptr<world::SkyDome> sky;
    vsg::ref_ptr<vsg::Node> sunLight;
    double lastSunUpdate = -1e9;

    struct Mount {
        std::uint32_t vehicleId = 0;
        CameraSpec spec;
        vsg::dmat4 mount; ///< body -> camera frame (translation + attitude)
        vsg::dmat4 lastBodyToEcef;
        bool posed = false;
    };
    std::vector<Mount> mounts;
    std::vector<std::uint32_t> slotVehicle; ///< visuals slot -> vehicle id (0 = free)
    std::unordered_map<std::uint32_t, std::size_t> vehicleSlot;
    std::vector<sim::VehicleState> states;
    std::unordered_map<std::uint32_t, vsg::Mask> ownBit; ///< vehicles that hide themselves from their own cameras
    double lastRenderMs = 0.0;
    bool compiled = false;

    // Sharing with viewers: the segment follows the active camera set.
    ipc::VisionPublisher publisher;
    std::vector<unsigned> publishedCameras; ///< active camera index per segment slot
    std::size_t publishedActive = 0;        ///< active-camera signature the table was built for
    std::uint64_t publishedTableVersion = 0;
    std::uint64_t cameraTableVersion = 0;   ///< bumped on add/remove

    void publish() {
        if (!options.publish || !world->published() || mounts.empty()) return;
        if (!publisher.valid() && !publisher.create(world->name())) {
            options.publish = false; // once
            return;
        }
        if (publishedTableVersion != cameraTableVersion) {
            std::vector<ipc::VisionCameraDesc> table;
            publishedCameras.clear();
            std::unordered_map<std::uint32_t, unsigned> perVehicle;
            for (std::size_t i = 0; i < mounts.size(); ++i) {
                if (!offscreen.active(static_cast<unsigned>(i))) continue;
                ipc::VisionCameraDesc d;
                d.vehicleId = mounts[i].vehicleId;
                d.width = offscreen.width(static_cast<unsigned>(i));
                d.height = offscreen.height(static_cast<unsigned>(i));
                const Vehicle v = world->vehicle(mounts[i].vehicleId);
                d.label = v.valid() ? v.name() : "vehicle " + std::to_string(mounts[i].vehicleId);
                const unsigned n = perVehicle[mounts[i].vehicleId]++;
                if (n > 0) d.label += " #" + std::to_string(n + 1);
                table.push_back(std::move(d));
                publishedCameras.push_back(static_cast<unsigned>(i));
            }
            const unsigned kept = publisher.setCameras(table);
            publishedCameras.resize(kept);
            publishedTableVersion = cameraTableVersion;
        }
        for (unsigned slot = 0; slot < publishedCameras.size(); ++slot) {
            const auto& rgb = offscreen.rgb(publishedCameras[slot]);
            publisher.write(slot, rgb.data(), rgb.size());
        }
        publisher.endFrame();
    }

    std::size_t slotFor(const Vehicle& v) {
        const auto it = vehicleSlot.find(v.id());
        if (it != vehicleSlot.end()) return it->second;
        for (std::size_t s = 0; s < slotVehicle.size(); ++s)
            if (slotVehicle[s] == 0) {
                slotVehicle[s] = v.id();
                vehicleSlot[v.id()] = s;
                visuals->setModel(s, v.model(), v.type());
                const auto bit = ownBit.find(v.id());
                visuals->setMask(s, bit != ownBit.end() ? bit->second : vsg::MASK_ALL);
                visuals->setVisible(s, true);
                return s;
            }
        return static_cast<std::size_t>(-1); // beyond maxVehicles: not drawn
    }

    void syncVehicles() {
        // Vehicles present now: assign slots to new ones, free the slots of removed ones.
        std::vector<Vehicle> live = world->vehicles();
        std::vector<unsigned char> seen(slotVehicle.size(), 0);
        for (const auto& v : live) {
            const std::size_t s = slotFor(v);
            if (s < seen.size()) {
                seen[s] = 1;
                states[s] = v.state();
            }
        }
        for (std::size_t s = 0; s < slotVehicle.size(); ++s)
            if (slotVehicle[s] != 0 && !seen[s]) {
                vehicleSlot.erase(slotVehicle[s]);
                slotVehicle[s] = 0;
                visuals->setVisible(s, false);
            }
        visuals->update(Span<const sim::VehicleState>(states));
    }

    void aim() {
        for (std::size_t i = 0; i < mounts.size(); ++i) {
            auto& m = mounts[i];
            const Vehicle v = world->vehicle(m.vehicleId);
            if (v.valid()) {
                m.lastBodyToEcef = world::bodyToEcef(v.state());
                m.posed = true;
            }
            if (!m.posed || !offscreen.active(static_cast<unsigned>(i))) continue;
            const vsg::dmat4 cam = m.lastBodyToEcef * m.mount;
            const vsg::dvec3 eye = cam * vsg::dvec3(0.0, 0.0, 0.0);
            const vsg::dvec3 forward = cam * vsg::dvec3(1.0, 0.0, 0.0);
            const vsg::dvec3 above = cam * vsg::dvec3(0.0, 0.0, -1.0);
            offscreen.setView(static_cast<unsigned>(i), eye, forward, above - eye);
            if (i == 0 && sky) sky->update(eye);
        }
        // Sun from the world's clock (once a second of wall time is plenty).
        if (sunLight) {
            const auto now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            if (now - lastSunUpdate > 1.0) {
                lastSunUpdate = now;
                int day = 172;
                double hours = 12.0;
                world::utcOf(world->environment().utcSeconds(), day, hours);
                const vsg::dvec3 sunDir = world::sunDirectionEcef(day, hours);
                world::setSunDirection(sunLight, sunDir);
                if (sky) sky->setSun(sunDir);
            }
        }
    }
};

namespace {
[[noreturn]] void rethrow(const char* where, const vsg::Exception& e) {
    throw Error(std::string("vision: ") + where + ": " + e.message + " (" + std::to_string(e.result) + ")");
}
} // namespace

Sensors::Sensors(World& world, const Options& options) : impl_(std::make_unique<Impl>()) {
  try {
    impl_->world = &world;
    impl_->options = options;
    Offscreen::Settings s;
    s.debugLayer = options.debugLayer;
    std::string error;
    if (!impl_->offscreen.create(s, &error)) throw Error("vision: " + error);

    auto vsgOptions = impl_->offscreen.options();
    impl_->ellipsoid = vsg::EllipsoidModel::create();
    auto scene = vsg::Group::create();

    int day = 172;
    double hours = 12.0;
    world::utcOf(world.environment().utcSeconds(), day, hours);
    const vsg::dvec3 sunDir = world::sunDirectionEcef(day, hours);
    if (options.sky) {
        impl_->sky = std::make_unique<world::SkyDome>(vsgOptions, sunDir);
        scene->addChild(impl_->sky->node());
        impl_->sunLight = world::createSunLight(day, hours);
        scene->addChild(impl_->sunLight);
    } else {
        scene->addChild(vsg::createHeadlight());
    }

    world::EarthSettings earth;
    earth.source = options.imagery ? (options.imageryUrl.empty() ? world::EarthSettings::Source::EsriWorldImagery : world::EarthSettings::Source::Custom)
                                   : world::EarthSettings::Source::OpenStreetMap;
    if (!options.imageryUrl.empty()) earth.imageryUrl = options.imageryUrl;
    earth.maxLevel = options.maxLevel;
    earth.elevationMaxLevel = std::min(15u, options.maxLevel);
    if (options.elevation) earth.elevationUrl = options.elevationUrl.empty() ? world::kAwsTerrariumUrl : options.elevationUrl;
    if (!options.earth) {
        // nothing: sky and vehicles only
    } else if (auto e = world::createEarth(earth, vsgOptions, impl_->ellipsoid)) {
        scene->addChild(e);
        if (auto caps = world::createPolarCaps(impl_->ellipsoid, vsgOptions)) scene->addChild(caps);
    } else {
        LOG_WARN("vision") << "no Earth in the camera scene";
    }

    io::AssetResolver assets;
    if (!options.assetDir.empty()) assets.addSearchPath(options.assetDir);
    world::VehicleVisuals::Settings vs;
    if (auto sample = assets.find("models/Cesium_Air.glb")) vs.modelPath = sample->string();
    world::VehicleVisuals::applyManifest(vs);
    for (const auto& dir : assets.searchPaths())
        if (std::filesystem::is_directory(dir / "models")) vs.modelDirs.push_back(dir / "models");
    vs.segmentation = options.segmentation;
    impl_->visuals = std::make_unique<world::VehicleVisuals>(options.maxVehicles, vs, vsgOptions);
    for (unsigned i = 0; i < options.maxVehicles; ++i) impl_->visuals->setVisible(i, false);
    scene->addChild(impl_->visuals->node());
    impl_->slotVehicle.assign(options.maxVehicles, 0);
    impl_->states.assign(options.maxVehicles, sim::VehicleState{});

    impl_->offscreen.setScene(scene, impl_->ellipsoid);
    if (options.segmentation) impl_->offscreen.setSegmentationScene(impl_->visuals->segmentationNode());
  } catch (const vsg::Exception& e) {
    rethrow("scene", e);
  }
}

Sensors::~Sensors() = default;

unsigned Sensors::addCamera(const Vehicle& vehicle, const CameraSpec& spec) {
    Impl::Mount m;
    m.vehicleId = vehicle.id();
    m.spec = spec;
    // Body -> camera: translate to the mount, then yaw (about z, down), pitch (about y, right), roll (about x, forward).
    m.mount = vsg::translate(spec.offsetBodyM[0], spec.offsetBodyM[1], spec.offsetBodyM[2]) *
              vsg::rotate(spec.yawDeg * kDeg, vsg::dvec3(0.0, 0.0, 1.0)) *
              vsg::rotate(spec.pitchDeg * kDeg, vsg::dvec3(0.0, 1.0, 0.0)) *
              vsg::rotate(spec.rollDeg * kDeg, vsg::dvec3(1.0, 0.0, 0.0));
    impl_->mounts.push_back(m);
    ++impl_->cameraTableVersion;
    // Own-vehicle hiding: the vehicle's visuals carry one mask bit, its cameras' views clear that bit.
    vsg::Mask viewMask = vsg::MASK_ALL;
    if (spec.hideOwnVehicle) {
        auto it = impl_->ownBit.find(vehicle.id());
        if (it == impl_->ownBit.end() && impl_->ownBit.size() < 31)
            it = impl_->ownBit.emplace(vehicle.id(), vsg::Mask(1u << (impl_->ownBit.size() + 1))).first;
        if (it != impl_->ownBit.end()) viewMask = vsg::MASK_ALL & ~it->second;
    }
    if (spec.segmentation && !impl_->options.segmentation)
        LOG_WARN("vision") << "CameraSpec::segmentation needs Options::segmentation; this camera has no ids";
    return impl_->offscreen.addCamera(std::max(1u, spec.width), std::max(1u, spec.height), spec.fovDeg, viewMask, spec.depth,
                                      spec.segmentation);
}

std::size_t Sensors::cameraCount() const noexcept { return impl_->mounts.size(); }

void Sensors::removeCamera(unsigned camera) {
    impl_->offscreen.removeCamera(camera);
    ++impl_->cameraTableVersion;
}

void Sensors::render() {
    const auto t0 = std::chrono::steady_clock::now();
    try {
        if (impl_->offscreen.activeCameraCount() == 0) throw Error("vision: no cameras");
        impl_->syncVehicles();
        impl_->aim();
        impl_->offscreen.render(); // compiles first when cameras were added or removed
        impl_->compiled = true;
        impl_->publish();
    } catch (const vsg::Exception& e) {
        rethrow("render", e);
    }
    impl_->lastRenderMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

}

void Sensors::settle(unsigned frames) {
    if (!impl_->compiled) return;
    impl_->syncVehicles();
    impl_->aim();
    for (unsigned i = 0; i < frames; ++i) {
        impl_->offscreen.advance();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    vkDeviceWaitIdle(*impl_->offscreen.device());
}

Image Sensors::image(unsigned camera) const noexcept {
    Image img;
    if (!impl_->offscreen.active(camera)) return img;
    img.rgb = impl_->offscreen.rgb(camera).data();
    img.width = impl_->offscreen.width(camera);
    img.height = impl_->offscreen.height(camera);
    return img;
}

DepthImage Sensors::depth(unsigned camera) const noexcept {
    DepthImage img;
    if (!impl_->offscreen.active(camera) || impl_->offscreen.depth(camera).empty()) return img;
    img.metres = impl_->offscreen.depth(camera).data();
    img.width = impl_->offscreen.width(camera);
    img.height = impl_->offscreen.height(camera);
    return img;
}

SegmentationImage Sensors::segmentation(unsigned camera) const noexcept {
    SegmentationImage img;
    if (!impl_->offscreen.active(camera) || impl_->offscreen.segmentation(camera).empty()) return img;
    img.ids = impl_->offscreen.segmentation(camera).data();
    img.width = impl_->offscreen.width(camera);
    img.height = impl_->offscreen.height(camera);
    return img;
}

unsigned Sensors::segmentationId(const Vehicle& vehicle) const noexcept {
    const auto it = impl_->vehicleSlot.find(vehicle.id());
    return it == impl_->vehicleSlot.end() ? 0u : static_cast<unsigned>(it->second) + 1u;
}

bool Sensors::saveSegmentationPng(unsigned camera, const std::string& path) const {
    const SegmentationImage seg = segmentation(camera);
    if (!seg.ids) return false;
    // A readable palette rather than the raw id: golden-ratio hues, black for 0.
    std::vector<std::uint8_t> rgb(seg.size() * 3, 0);
    for (std::size_t i = 0; i < seg.size(); ++i) {
        const unsigned id = seg.ids[i];
        if (id == 0) continue;
        const double h = std::fmod(static_cast<double>(id) * 0.61803398875, 1.0) * 6.0;
        const int sector = static_cast<int>(h);
        const double f = h - sector;
        const double v = 1.0, p = 0.25, q = 1.0 - 0.75 * f, t = 0.25 + 0.75 * f;
        double r = v, g = v, b = v;
        switch (sector) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
        }
        rgb[i * 3 + 0] = static_cast<std::uint8_t>(r * 255.0 + 0.5);
        rgb[i * 3 + 1] = static_cast<std::uint8_t>(g * 255.0 + 0.5);
        rgb[i * 3 + 2] = static_cast<std::uint8_t>(b * 255.0 + 0.5);
    }
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const int ok = stbi_write_png_to_func([](void* ctx, void* data, int size) { std::fwrite(data, 1, static_cast<std::size_t>(size), static_cast<std::FILE*>(ctx)); }, f,
                                          static_cast<int>(seg.width), static_cast<int>(seg.height), 3, rgb.data(), static_cast<int>(seg.width) * 3);
    std::fclose(f);
    return ok != 0;
}

bool Sensors::saveDepthPng(unsigned camera, const std::string& path, double farM) const {
    const DepthImage d = depth(camera);
    if (!d.metres) return false;
    std::vector<std::uint8_t> grey(d.size());
    const double lf = std::log(std::max(2.0, farM));
    for (std::size_t i = 0; i < d.size(); ++i) {
        const double m = std::max(1.0, static_cast<double>(d.metres[i]));
        const double v = 1.0 - std::min(1.0, std::log(m) / lf);
        grey[i] = static_cast<std::uint8_t>(v * 255.0 + 0.5);
    }
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const int ok = stbi_write_png_to_func([](void* ctx, void* data, int size) { std::fwrite(data, 1, static_cast<std::size_t>(size), static_cast<std::FILE*>(ctx)); }, f,
                                          static_cast<int>(d.width), static_cast<int>(d.height), 1, grey.data(), static_cast<int>(d.width));
    std::fclose(f);
    return ok != 0;
}

bool Sensors::savePng(unsigned camera, const std::string& path) const {
    const Image img = image(camera);
    if (!img.rgb) return false;
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const int ok = stbi_write_png_to_func([](void* ctx, void* data, int size) { std::fwrite(data, 1, static_cast<std::size_t>(size), static_cast<std::FILE*>(ctx)); }, f,
                                          static_cast<int>(img.width), static_cast<int>(img.height), 3, img.rgb, static_cast<int>(img.width) * 3);
    std::fclose(f);
    return ok != 0;
}

double Sensors::lastRenderMs() const noexcept { return impl_->lastRenderMs; }
Sensors::Timing Sensors::timing() const noexcept {
    const auto& t = impl_->offscreen.timing();
    return Timing{t.advance, t.update, t.record, t.wait, t.readback};
}
std::uint64_t Sensors::frames() const noexcept { return impl_->offscreen.frames(); }

// --- BatchCameras -----------------------------------------------------------------------

struct BatchCameras::Impl {
    std::unique_ptr<Sensors> sensors;
    std::vector<unsigned> cameras; ///< per batch vehicle
    std::vector<std::uint8_t> rgb;
    std::vector<float> depth;
    std::vector<std::uint16_t> ids;
    unsigned width = 0, height = 0;
    bool wantDepth = false, wantSegmentation = false;
};

BatchCameras::BatchCameras(VecEnv& env, const CameraSpec& spec, const Options& options) : impl_(std::make_unique<Impl>()) {
    impl_->sensors = std::make_unique<Sensors>(env.world(), options);
    impl_->width = std::max(1u, spec.width);
    impl_->height = std::max(1u, spec.height);
    impl_->wantDepth = spec.depth;
    impl_->wantSegmentation = spec.segmentation && options.segmentation;
    // Vehicles are named "env<e>/<v>" in batch order; mount in that order.
    const unsigned K = env.vehiclesPerEnv();
    for (unsigned e = 0; e < env.numEnvs(); ++e)
        for (unsigned k = 0; k < K; ++k) {
            const Vehicle v = env.world().vehicle("env" + std::to_string(e) + "/" + std::to_string(k));
            if (!v.valid()) throw Error("vision: batch vehicle env" + std::to_string(e) + "/" + std::to_string(k) + " not found");
            impl_->cameras.push_back(impl_->sensors->addCamera(v, spec));
        }
    const std::size_t pixels = static_cast<std::size_t>(impl_->width) * impl_->height;
    impl_->rgb.assign(impl_->cameras.size() * pixels * 3, 0);
    if (impl_->wantDepth) impl_->depth.assign(impl_->cameras.size() * pixels, 0.0f);
    if (impl_->wantSegmentation) impl_->ids.assign(impl_->cameras.size() * pixels, 0);
}

BatchCameras::~BatchCameras() = default;

void BatchCameras::render() {
    impl_->sensors->render();
    const std::size_t pixels = static_cast<std::size_t>(impl_->width) * impl_->height;
    for (std::size_t i = 0; i < impl_->cameras.size(); ++i) {
        const Image img = impl_->sensors->image(impl_->cameras[i]);
        if (img.rgb) std::memcpy(impl_->rgb.data() + i * pixels * 3, img.rgb, pixels * 3);
        if (impl_->wantDepth) {
            const DepthImage d = impl_->sensors->depth(impl_->cameras[i]);
            if (d.metres) std::memcpy(impl_->depth.data() + i * pixels, d.metres, pixels * sizeof(float));
        }
        if (impl_->wantSegmentation) {
            const SegmentationImage seg = impl_->sensors->segmentation(impl_->cameras[i]);
            if (seg.ids) std::memcpy(impl_->ids.data() + i * pixels, seg.ids, pixels * sizeof(std::uint16_t));
        }
    }
}

ConstSpan<std::uint8_t> BatchCameras::rgb() const noexcept { return ConstSpan<std::uint8_t>{impl_->rgb.data(), impl_->rgb.size()}; }
ConstSpan<float> BatchCameras::depth() const noexcept { return ConstSpan<float>{impl_->depth.data(), impl_->depth.size()}; }
ConstSpan<std::uint16_t> BatchCameras::segmentation() const noexcept { return ConstSpan<std::uint16_t>{impl_->ids.data(), impl_->ids.size()}; }
std::size_t BatchCameras::count() const noexcept { return impl_->cameras.size(); }
unsigned BatchCameras::width() const noexcept { return impl_->width; }
unsigned BatchCameras::height() const noexcept { return impl_->height; }
Sensors& BatchCameras::sensors() noexcept { return *impl_->sensors; }
unsigned BatchCameras::camera(std::size_t vehicle) const noexcept { return vehicle < impl_->cameras.size() ? impl_->cameras[vehicle] : 0u; }

} // namespace fsim::vision
