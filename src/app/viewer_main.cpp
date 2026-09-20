// flightsim-viewer.exe - the prebuilt visualisation application (design 9.1,
// 9.7, 9.10).
//
// Default ("mirror") mode: discover a training application's world through
// shared memory, attach, and mirror its vehicles: creations, resets, state,
// control level and environment. The training process never knows the
// viewer exists. `--demo` runs the built-in scenario on an internal simulation
// thread instead (development, screenshots).

#include "app/DemoAutopilot.h"
#include "core/Log.h"
#include "core/Rng.h"
#include "core/Units.h"
#include "fsim/Control.h"
#include "io/AssetResolver.h"
#include "ipc/WorldMirror.h"
#include "ipc/WorldRegistry.h"
#include "platform/Clock.h"
#include "platform/Paths.h"
#include "platform/Threads.h"
#include "render/Viewer.h"
#include "sim/GroundProvider.h"
#include "sim/JsbsimModel.h"
#include "sim/SimRunner.h"
#include "sim/VehiclePool.h"
#include "ui/KeyHandler.h"
#include "ui/MonitorGui.h"
#include "ui/ViewerControls.h"
#include "world/CameraController.h"
#include "world/Earth.h"
#include "world/Interpolator.h"
#include "world/Sky.h"
#include "world/SkyDome.h"
#include "world/Terrain.h"
#include "world/Trails.h"
#include "world/VehicleVisuals.h"

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/SendEventsToImGui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using namespace fsim;

namespace {

struct ViewerOptions {
    // Source
    bool demo = false;
    bool list = false;
    std::string worldName;       // --world: attach to this world (default: the newest)
    unsigned capacity = 256;     // slots to prepare before a world is attached

    // Demo scenario
    std::string aircraft = "c172x";
    std::filesystem::path jsbsimRoot;
    unsigned vehicles = 8;
    unsigned workers = 0;
    double dt = 1.0 / 120.0;
    int frameSkip = 2; // 60 Hz agent rate for smooth motion
    std::uint64_t seed = 1;
    double throttle = 0.65;
    double timeFactor = 1.0;
    double latitudeDeg = 37.6188, longitudeDeg = -122.375, altitudeM = 1500.0, spreadDeg = 0.03;
    bool onGround = false;

    // Scene
    world::EarthSettings earth;
    unsigned terrainZoom = 12;
    bool sun = true;
    double sunUtcHours = -1.0;
    std::string modelPath;
    std::string modelForward, modelUp;
    double modelScaleOverride = 0.0;

    render::ViewerSettings window;
    int cameraMode = 0;
    double chaseDistance = 40.0;
    double chaseAzimuth = 180.0, chaseElevation = 14.0;
    bool probe = false;
    double stats = 0.0;   // --stats <seconds>: print per-second frame statistics, exit after this long
    int trace = -1;
    bool interpolate = true;
    bool help = false;
};

void usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "Mirror mode (default): attach to a training application's world through shared memory.\n"
        "  --world <name>           attach to this world (default: the newest published one)\n"
        "  --list                   list live worlds and exit\n"
        "  --capacity <n>           vehicle slots to prepare (256)\n"
        "Demo mode: run the built-in scenario on an internal simulation thread.\n"
        "  --demo                   enable demo mode\n"
        "  --aircraft <name>        JSBSim aircraft (c172x)\n"
        "  --jsbsim-root <dir>      JSBSim data tree (auto)\n"
        "  --vehicles <n>           number of vehicles (8)\n"
        "  --workers <n>            sim workers (auto)\n"
        "  --time-factor <x>        simulation speed (1.0)\n"
        "  --lat <deg> --lon <deg>  spawn centre (KSFO)\n"
        "  --alt <m>                spawn altitude MSL (1500)\n"
        "  --spread <deg>           spawn scatter (0.03)\n"
        "  --on-ground              spawn parked on the terrain (brakes on)\n"
        "  --terrain-zoom <z>       tile level used for physics ground height (12)\n"
        "  --probe                  print motion smoothness statistics after ~5 s and exit\n"
        "  --stats <seconds>        print per-second frame statistics (fps, frame-time breakdown, CPU) and exit\n"
        "  --trace <i>              print vehicle i's state once per second\n"
        "  --no-interpolate         draw raw snapshots (sample-and-hold) instead of interpolating\n"
        "Scene:\n"
        "  --imagery satellite|osm|bing|none|<url template with {z}/{x}/{y}>   (satellite = Esri World Imagery)\n"
        "  --elevation terrarium|none|<url template>   relief from Terrarium-encoded tiles (default terrarium)\n"
        "  --no-sun                 headlight instead of sun + ambient lighting\n"
        "  --sun-utc <hours>        sun position for this UTC hour (default: the world's time)\n"
        "  --bing-key <key>         Bing Maps key for --imagery bing\n"
        "  --max-level <n>          custom pyramid max level (17)\n"
        "  --model <file>           glTF/OBJ vehicle model (placeholder if omitted)\n"
        "  --model-scale <x>        model scale (1.0)\n"
        "  --width <px> --height <px> --fullscreen --msaa <1|2|4|8> --fov <deg> --max-fps <n> (0 = vsync only)\n"
        "  --debug-layer            Vulkan validation layer\n"
        "  --camera chase|orbit|overview|free   initial camera (chase)\n"
        "  --log-level <lvl>\n"
        "Keys: space pause (demo), . step (demo), tab next vehicle, c camera, -/= zoom, r reset view, [ ] time factor (demo),\n"
        "      l list, m monitor, n labels, t trails, esc quit\n"
        "Mouse (OSG feel): left drag rotates, middle drag pans, wheel zooms (6 m .. whole Earth); right drag zooms while following a vehicle\n"
        "      and drags the globe in free mode / while no vehicle exists; the eye never goes below the terrain\n",
        prog);
}

bool parse(int argc, char** argv, ViewerOptions& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
            return argv[++i];
        };
        try {
            if (a == "-h" || a == "--help") o.help = true;
            else if (a == "--demo") o.demo = true;
            else if (a == "--list") o.list = true;
            else if (a == "--world") o.worldName = next();
            else if (a == "--capacity") o.capacity = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--aircraft") o.aircraft = next();
            else if (a == "--jsbsim-root") o.jsbsimRoot = next();
            else if (a == "--vehicles") o.vehicles = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--workers") o.workers = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--time-factor") o.timeFactor = std::stod(next());
            else if (a == "--lat") o.latitudeDeg = std::stod(next());
            else if (a == "--lon") o.longitudeDeg = std::stod(next());
            else if (a == "--alt") o.altitudeM = std::stod(next());
            else if (a == "--spread") o.spreadDeg = std::stod(next());
            else if (a == "--seed") o.seed = std::stoull(next());
            else if (a == "--throttle") o.throttle = std::stod(next());
            else if (a == "--imagery") {
                const std::string v = next();
                using S = world::EarthSettings::Source;
                if (v == "osm") o.earth.source = S::OpenStreetMap;
                else if (v == "esri" || v == "satellite") o.earth.source = S::EsriWorldImagery;
                else if (v == "bing") o.earth.source = S::Bing;
                else if (v == "none") o.earth.source = S::None;
                else { o.earth.source = S::Custom; o.earth.imageryUrl = v; }
            } else if (a == "--elevation") {
                const std::string v = next();
                if (v == "terrarium") o.earth.elevationUrl = world::kAwsTerrariumUrl;
                else if (v == "none") o.earth.elevationUrl.clear();
                else o.earth.elevationUrl = v;
            } else if (a == "--terrain-zoom") o.terrainZoom = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--no-sun") o.sun = false;
            else if (a == "--sun-utc") o.sunUtcHours = std::stod(next());
            else if (a == "--on-ground") o.onGround = true;
            else if (a == "--bing-key") o.earth.bingKey = next();
            else if (a == "--max-level") o.earth.maxLevel = static_cast<unsigned>(std::stoul(next()));
            else if (a == "--model") o.modelPath = next();
            else if (a == "--model-scale") o.modelScaleOverride = std::stod(next());
            else if (a == "--model-forward") o.modelForward = next();
            else if (a == "--model-up") o.modelUp = next();
            else if (a == "--width") o.window.width = static_cast<std::uint32_t>(std::stoul(next()));
            else if (a == "--height") o.window.height = static_cast<std::uint32_t>(std::stoul(next()));
            else if (a == "--fullscreen") o.window.fullscreen = true;
            else if (a == "--fov") o.window.fieldOfViewDeg = std::stod(next());
            else if (a == "--max-fps") o.window.maxFps = std::stod(next());
            else if (a == "--msaa") {
                const int s = std::stoi(next());
                o.window.samples = s >= 8 ? VK_SAMPLE_COUNT_8_BIT : s >= 4 ? VK_SAMPLE_COUNT_4_BIT : s >= 2 ? VK_SAMPLE_COUNT_2_BIT : VK_SAMPLE_COUNT_1_BIT;
            } else if (a == "--debug-layer") o.window.debugLayer = true;
            else if (a == "--chase-distance") o.chaseDistance = std::stod(next());
            else if (a == "--chase-azimuth") o.chaseAzimuth = std::stod(next());
            else if (a == "--chase-elevation") o.chaseElevation = std::stod(next());
            else if (a == "--camera") {
                const std::string v = next();
                o.cameraMode = v == "orbit" ? 1 : v == "overview" ? 2 : v == "free" ? 3 : 0;
            } else if (a == "--probe") o.probe = true;
            else if (a == "--stats") o.stats = std::stod(next());
            else if (a == "--trace") o.trace = std::stoi(next());
            else if (a == "--no-interpolate") o.interpolate = false;
            else if (a == "--log-level") {
                const std::string v = next();
                log::setLevel(v == "trace" ? log::Level::Trace : v == "debug" ? log::Level::Debug : v == "warn" ? log::Level::Warn : v == "error" ? log::Level::Error : log::Level::Info);
            } else {
                std::fprintf(stderr, "unknown option %s\n", a.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "bad argument: %s\n", e.what());
            return false;
        }
    }
    return o.vehicles >= 1;
}

/// Environment one-liner for the monitor.
std::string describe(const sim::EnvironmentState& e, double simTime) {
    int day = 0; double hours = 0.0;
    world::utcOf(e.epochUtcSeconds + simTime, day, hours);
    char buf[256];
    std::snprintf(buf, sizeof buf, "UTC day %d %02d:%02d  wind %.0f deg / %.1f m/s  turb %.2f  T0 %.1f K  p0 %.0f hPa  vis %.0f km",
                  day, static_cast<int>(hours), static_cast<int>((hours - std::floor(hours)) * 60.0), e.windDirectionDeg, e.windSpeedMs,
                  e.turbulence, e.temperatureSeaLevelK, e.pressureSeaLevelPa / 100.0, e.visibilityM / 1000.0);
    return buf;
}

} // namespace

int main(int argc, char** argv) {
    // GUI subsystem: print to the terminal we were started from, else keep a log file.
    if (!platform::attachParentConsole()) {
        const auto logPath = platform::configDir() / "viewer.log";
        FILE* f = nullptr;
        if (freopen_s(&f, logPath.string().c_str(), "w", stderr) == 0) freopen_s(&f, logPath.string().c_str(), "a", stdout);
    }
    ViewerOptions opt;
    if (!parse(argc, argv, opt)) {
        usage(argv[0]);
        return 2;
    }
    if (opt.help) {
        usage(argv[0]);
        return 0;
    }
    if (opt.list) {
        const auto worlds = ipc::WorldRegistry::list();
        if (worlds.empty()) std::printf("no published worlds\n");
        for (const auto& w : worlds) std::printf("%-32s pid %llu\n", w.name.c_str(), static_cast<unsigned long long>(w.pid));
        return 0;
    }

    // --- Rendering (window first: the terrain ground provider uses VSG's readers) ---
    render::Viewer viewer;
    opt.window.title = opt.demo ? "flightsim - demo " + opt.aircraft : "flightsim - waiting for a training application";
    if (opt.earth.elevationUrl.empty()) opt.earth.elevationUrl = world::kAwsTerrariumUrl;
    if (opt.earth.source == world::EarthSettings::Source::None) opt.earth.elevationUrl.clear();
    opt.window.headlight = !opt.sun;
    if (!viewer.create(opt.window)) return 1;

    io::AssetResolver assets;
    auto ellipsoid = vsg::EllipsoidModel::create(); // WGS-84

    // Elevation tiles on the CPU: camera terrain collision in every mode, physics in demo mode.
    std::shared_ptr<world::TileGroundProvider> terrain;
    if (!opt.earth.elevationUrl.empty())
        terrain = std::make_shared<world::TileGroundProvider>(opt.earth.elevationUrl, opt.earth.elevationEncoding, opt.terrainZoom, viewer.options());

    // --- Demo simulation (only with --demo) --------------------------------------
    std::unique_ptr<sim::SimRunner> runner;
    std::size_t slots = opt.capacity;
    if (opt.demo) {
        const auto root = assets.jsbsimRoot(opt.jsbsimRoot);
        if (!root) {
            LOG_ERROR("app") << "JSBSim data root not found; pass --jsbsim-root <dir>";
            return 1;
        }
        std::shared_ptr<sim::GroundProvider> ground;
        if (terrain) {
            terrain->prefetch(units::degreesToRadians(opt.latitudeDeg), units::degreesToRadians(opt.longitudeDeg), 6000.0 + opt.spreadDeg * 111000.0);
            ground = terrain;
        } else {
            ground = std::make_shared<sim::FlatGround>(0.0);
        }
        const unsigned physical = platform::physicalCoreCount();
        const unsigned workers = std::min<unsigned>(opt.workers ? opt.workers : std::max(1u, physical > 3 ? physical - 3 : 1u), opt.vehicles);
        auto pool = std::make_unique<sim::VehiclePool>(workers);
        const sim::AircraftSpec aircraft{opt.aircraft, *root};
        auto initialConditions = std::make_shared<std::vector<sim::InitialConditions>>();
        for (unsigned i = 0; i < opt.vehicles; ++i) {
            Rng rng = Rng::forVehicle(opt.seed, 0, i);
            sim::InitialConditions ic;
            ic.latitudeDeg = opt.latitudeDeg + rng.uniform(-opt.spreadDeg, opt.spreadDeg);
            ic.longitudeDeg = opt.longitudeDeg + rng.uniform(-opt.spreadDeg, opt.spreadDeg);
            ic.headingDeg = rng.uniform(0.0, 360.0);
            if (opt.onGround) {
                ic.onGround = true;
                ic.airspeedTrueMs = 0.0;
            } else {
                const double terrainM = ground->heightAboveEllipsoidM(units::degreesToRadians(ic.latitudeDeg), units::degreesToRadians(ic.longitudeDeg));
                ic.altitudeMslM = std::max(opt.altitudeM + rng.uniform(-150.0, 150.0), terrainM + 300.0);
                ic.airspeedTrueMs = 58.0 + rng.uniform(-4.0, 4.0);
            }
            auto model = std::make_unique<sim::JsbsimModel>(opt.dt, ground);
            if (!model->load(aircraft, ic)) {
                LOG_ERROR("app") << "vehicle " << i << " failed to load";
                return 1;
            }
            pool->add(std::move(model));
            initialConditions->push_back(ic);
        }
        auto autopilot = std::make_shared<app::DemoAutopilot>(opt.vehicles, opt.throttle, opt.onGround);
        runner = std::make_unique<sim::SimRunner>(std::move(pool), opt.frameSkip,
            [autopilot, initialConditions](const sim::SnapshotBatch& prev, sim::VehiclePool& vehicles, std::vector<sim::ControlInputs>& out) {
                for (std::size_t i = 0; i < prev.states.size(); ++i) {
                    if (!prev.states[i].diverged) continue;
                    LOG_WARN("app") << "vehicle " << i << " diverged; resetting to its initial conditions";
                    vehicles.vehicle(i).reset((*initialConditions)[i]);
                    autopilot->forget(i);
                }
                autopilot->compute(prev.states, out);
            });
        runner->setTimeFactor(opt.timeFactor);
        slots = opt.vehicles;
        LOG_INFO("app") << "demo: " << opt.vehicles << " x " << opt.aircraft << " on " << workers << " worker(s)";
    }

    // --- Scene -------------------------------------------------------------------
    auto scene = vsg::Group::create();
    int day = 172; double hours = 12.0;
    world::currentUtc(day, hours);
    if (opt.sunUtcHours >= 0.0) hours = opt.sunUtcHours;
    vsg::dvec3 sunDir = world::sunDirectionEcef(day, hours);
    world::SkyDome sky(viewer.options(), sunDir);
    scene->addChild(sky.node());
    vsg::ref_ptr<vsg::Node> sunLight;
    if (opt.sun) {
        sunLight = world::createSunLight(day, hours);
        scene->addChild(sunLight);
    }
    if (auto earth = world::createEarth(opt.earth, viewer.options(), ellipsoid)) scene->addChild(earth);

    world::VehicleVisuals::Settings visualSettings;
    if (opt.modelPath.empty()) {
        if (auto sample = assets.find("models/Cesium_Air.glb")) opt.modelPath = sample->string();
    } else if (opt.modelPath == "none") {
        opt.modelPath.clear();
    }
    visualSettings.modelPath = opt.modelPath;
    world::VehicleVisuals::applyManifest(visualSettings);
    if (opt.modelScaleOverride > 0.0) visualSettings.modelScale = opt.modelScaleOverride;
    auto axis = [](const std::string& a, const vsg::dvec3& fallback) {
        if (a.size() != 2) return fallback;
        const double sgn = a[0] == '-' ? -1.0 : 1.0;
        switch (a[1]) {
        case 'x': return vsg::dvec3(sgn, 0.0, 0.0);
        case 'y': return vsg::dvec3(0.0, sgn, 0.0);
        case 'z': return vsg::dvec3(0.0, 0.0, sgn);
        default: return fallback;
        }
    };
    visualSettings.modelForward = axis(opt.modelForward, visualSettings.modelForward);
    visualSettings.modelUp = axis(opt.modelUp, visualSettings.modelUp);
    world::VehicleVisuals visuals(slots, visualSettings, viewer.options());
    scene->addChild(visuals.node());
    world::Trails trails(slots, 900, 0.25, viewer.options());
    scene->addChild(trails.node());
    if (!opt.demo)
        for (std::size_t i = 0; i < slots; ++i) { visuals.setVisible(i, false); trails.setEnabled(i, false); }

    auto controls = std::make_shared<ui::ViewerControls>();
    controls->timeFactor.store(opt.timeFactor);
    controls->cameraMode.store(opt.cameraMode);
    auto gui = ui::MonitorGui::create(controls, opt.aircraft);
    auto imgui = vsgImGui::RenderImGui::create(viewer.window(), gui);
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::GetIO().ConfigWindowsMoveFromTitleBarOnly = true; // a drag that starts inside a panel is never a panel move
    if (const double dpi = platform::systemDpiScale(); dpi > 1.01) {
        ImGui::GetIO().FontGlobalScale = static_cast<float>(dpi);
        ImGui::GetStyle().ScaleAllSizes(static_cast<float>(dpi));
    }
    viewer.addEventHandler(vsgImGui::SendEventsToImGui::create());
    viewer.addEventHandler(ui::KeyHandler::create(controls));
    if (!viewer.setScene(scene, ellipsoid, imgui)) return 1;
    for (auto& anim : visuals.animations()) viewer.viewer()->animationManager->play(anim);

    auto camera = world::CameraController::create(viewer.camera(), viewer.lookAt(), ellipsoid);
    camera->setChaseOffset(opt.chaseDistance, opt.chaseElevation, opt.chaseAzimuth);
    if (terrain) {
        std::shared_ptr<world::TileGroundProvider> tiles = terrain;
        camera->setGroundQuery([tiles](double lat, double lon) { return tiles->cachedHeightAboveEllipsoidM(lat, lon); });
    }
    // Detached camera focus: the demo spawn area / default location, on the ground.
    camera->setFocus(ellipsoid->convertLatLongAltitudeToECEF(vsg::dvec3(opt.latitudeDeg, opt.longitudeDeg, 0.0)));
    if (!opt.demo) camera->zoom(300.0); // no vehicle yet: start with a regional view
    viewer.addEventHandler(camera);

    // --- Source state shared by both modes -----------------------------------------
    world::Interpolator interpolator(slots);
    std::vector<unsigned char> alive(slots, opt.demo ? 1 : 0);
    std::vector<ui::MonitorGui::VehicleMeta> meta;
    const sim::SnapshotBatch* batch = nullptr;  // latest raw batch (demo: runner's; mirror: `mirrored`)
    sim::SnapshotBatch mirrored, interpBatch;    // mirror copy; interpolation copy keyed by wall time

    ipc::WorldMirror mirror;
    double lastDiscovery = -10.0, lastSunUpdate = -1e9;
    std::vector<std::string> available;
    std::uint64_t lastVehicleSteps = 0;
    double lastStepsWall = 0.0, publishedSps = 0.0;

    if (runner) {
        runner->start();
        if (const auto* first = runner->snapshots().acquire()) {
            batch = first;
            interpolator.push(*first);
            visuals.update(Span<const sim::VehicleState>(first->states));
            camera->update(&first->states[0], 0.0);
        }
    }

    // --- Frame loop ------------------------------------------------------------------
    double wallSeconds = 0.0;
    struct Stats {
        double window = 0.0, cpu0 = platform::processCpuSeconds(), maxFrame = 0.0;
        int frames = 0;
        render::Viewer::FrameTiming sum;
    } stats;
    while (viewer.active() && !controls->quit.load(std::memory_order_relaxed)) {
        wallSeconds += viewer.frameSeconds();
        bool paused = false;
        double timeFactor = 1.0;

        if (runner) {
            paused = controls->paused.load(std::memory_order_relaxed);
            timeFactor = controls->timeFactor.load(std::memory_order_relaxed);
            runner->setPaused(paused);
            runner->setTimeFactor(timeFactor);
            if (controls->singleStep.exchange(false)) runner->singleStep();
            if (const auto* fresh = runner->snapshots().acquire()) {
                batch = fresh;
                interpolator.push(*fresh);
            }
            controls->simThroughput.store(runner->throughput(), std::memory_order_relaxed);
            if (batch) controls->simTime.store(batch->simTime, std::memory_order_relaxed);
        } else {
            // Discovery: attach to the requested (or newest) world; re-attach after a restart.
            if (!mirror.valid() && wallSeconds - lastDiscovery > 0.5) {
                lastDiscovery = wallSeconds;
                const auto worlds = ipc::WorldRegistry::list();
                available.clear();
                for (const auto& w : worlds) available.push_back(w.name);
                std::string pick;
                if (!opt.worldName.empty()) {
                    for (const auto& w : worlds) if (w.name == opt.worldName) pick = w.name;
                } else if (!worlds.empty()) {
                    pick = worlds.front().name;
                }
                if (!pick.empty() && mirror.open(pick)) {
                    if (mirror.capacity() != slots) LOG_WARN("app") << "world capacity " << mirror.capacity() << " differs from --capacity " << slots
                                                                     << "; only the first " << std::min<std::size_t>(slots, mirror.capacity()) << " slots are shown";
                    LOG_INFO("app") << "attached to world '" << pick << "'";
                    interpolator = world::Interpolator(slots);
                    for (std::size_t i = 0; i < slots; ++i) trails.setEnabled(i, false);
                    lastVehicleSteps = mirror.vehicleSteps();
                    lastStepsWall = wallSeconds;
                }
            }
            if (mirror.valid()) {
                if (mirror.pollTable()) {
                    meta.assign(slots, ui::MonitorGui::VehicleMeta{});
                    const auto& table = mirror.vehicles();
                    for (std::size_t i = 0; i < slots && i < table.size(); ++i) {
                        const auto& v = table[i];
                        alive[i] = v.alive ? 1 : 0;
                        meta[i].name = v.name;
                        meta[i].type = v.type;
                        meta[i].alive = v.alive;
                        meta[i].level = control::levelName(static_cast<control::Level>(v.controlLevel));
                        visuals.setVisible(i, v.alive);
                        trails.setEnabled(i, v.alive);
                    }
                    gui->setVehicles(meta);
                }
                if (mirror.pollSnapshot(mirrored)) {
                    if (mirrored.states.size() != slots) mirrored.states.resize(slots);
                    batch = &mirrored;
                    interpBatch = mirrored;
                    interpBatch.simTime = static_cast<double>(mirrored.wallNs) * 1e-9; // interpolate in wall time: the trainer's pace is arbitrary
                    interpolator.push(interpBatch);
                }
                if (wallSeconds - lastStepsWall >= 1.0) {
                    const std::uint64_t steps = mirror.vehicleSteps();
                    publishedSps = static_cast<double>(steps - lastVehicleSteps) / (wallSeconds - lastStepsWall);
                    lastVehicleSteps = steps;
                    lastStepsWall = wallSeconds;
                }
                // Sun from the world's clock (unless overridden on the command line).
                const auto env = mirror.environment();
                if (opt.sunUtcHours < 0.0 && env.epochUtcSeconds > 0.0 && wallSeconds - lastSunUpdate > 1.0 && batch) {
                    lastSunUpdate = wallSeconds;
                    world::utcOf(env.epochUtcSeconds + batch->simTime, day, hours);
                    sunDir = world::sunDirectionEcef(day, hours);
                    sky.setSun(sunDir);
                    if (sunLight) world::setSunDirection(sunLight, sunDir);
                }
                if (!mirror.publisherAlive() && mirror.ageSeconds() > 5.0) {
                    LOG_INFO("app") << "world '" << mirror.name() << "' has gone; waiting for another";
                    mirror.close();
                    batch = nullptr;
                    std::fill(alive.begin(), alive.end(), 0);
                    for (std::size_t i = 0; i < slots; ++i) { visuals.setVisible(i, false); trails.setEnabled(i, false); }
                    meta.clear();
                    gui->setVehicles(meta);
                }
            }
            ui::MonitorGui::Source src;
            src.mirror = true;
            src.attached = mirror.valid();
            src.world = mirror.name();
            src.ageSeconds = mirror.ageSeconds();
            src.publisherAlive = mirror.publisherAlive();
            src.simTime = batch ? batch->simTime : 0.0;
            src.vehicleStepsPerSecond = publishedSps;
            src.available = available;
            if (mirror.valid()) src.environment = describe(mirror.environment(), src.simTime);
            gui->setSource(src);
            if (batch) controls->simTime.store(batch->simTime, std::memory_order_relaxed);
        }

        // Selection: only live slots.
        int selected = controls->selectedVehicle.load(std::memory_order_relaxed);
        if (const int step = controls->selectStep.exchange(0); step != 0) {
            const int n = static_cast<int>(slots);
            for (int k = 1; k <= n; ++k) {
                const int candidate = ((selected + step * k) % n + n) % n;
                if (alive[static_cast<std::size_t>(candidate)]) { selected = candidate; break; }
            }
        }
        if (selected < 0 || static_cast<std::size_t>(selected) >= slots || !alive[static_cast<std::size_t>(selected)]) {
            for (std::size_t i = 0; i < slots; ++i)
                if (alive[i]) { selected = static_cast<int>(i); break; }
        }
        controls->selectedVehicle.store(selected, std::memory_order_relaxed);

        // Sim -> scene
        const bool anyAlive = std::any_of(alive.begin(), alive.end(), [](unsigned char a) { return a != 0; });
        if (batch) {
            interpolator.update(viewer.frameSeconds(), timeFactor, paused);
            const auto& states = opt.interpolate ? interpolator.states() : batch->states;
            visuals.update(Span<const sim::VehicleState>(states));
            visuals.setSelected(selected);
            trails.setSelected(selected);
            trails.setVisible(controls->showTrails.load(std::memory_order_relaxed));
            trails.update(Span<const sim::VehicleState>(batch->states), runner ? batch->simTime : static_cast<double>(batch->wallNs) * 1e-9);

            gui->setShowLabels(controls->showLabels.load(std::memory_order_relaxed));
            std::vector<ui::MonitorGui::Label> labels;
            const vsg::dmat4 viewProj = viewer.camera()->projectionMatrix->transform() * viewer.camera()->viewMatrix->transform();
            const auto extent = viewer.window()->extent2D();
            for (std::size_t i = 0; i < states.size(); ++i) {
                if (!alive[i]) continue;
                const vsg::dvec4 clip = viewProj * vsg::dvec4(states[i].positionEcef[0], states[i].positionEcef[1], states[i].positionEcef[2], 1.0);
                if (clip.w <= 0.0) continue;
                const double nx = clip.x / clip.w, ny = clip.y / clip.w;
                if (nx < -1.1 || nx > 1.1 || ny < -1.1 || ny > 1.1) continue;
                ui::MonitorGui::Label l;
                l.x = static_cast<float>((nx * 0.5 + 0.5) * extent.width);
                l.y = static_cast<float>((ny * 0.5 + 0.5) * extent.height);
                l.text = (i < meta.size() && !meta[i].name.empty() ? meta[i].name : "v" + std::to_string(i)) + (states[i].diverged ? " !" : "");
                l.selected = static_cast<int>(i) == selected;
                labels.push_back(std::move(l));
            }
            gui->setLabels(std::move(labels));
            controls->snapshotSequence.store(batch->sequence, std::memory_order_relaxed);
        }
        camera->setMode(static_cast<world::CameraController::Mode>(controls->cameraMode.load(std::memory_order_relaxed)));
        if (const double z = controls->cameraZoom.exchange(1.0); z != 1.0) camera->zoom(z);
        if (controls->cameraReset.exchange(false)) camera->resetView();
        const sim::VehicleState* target = (batch && anyAlive && selected >= 0)
            ? &(opt.interpolate ? interpolator.states() : batch->states)[static_cast<std::size_t>(selected)] : nullptr;
        camera->update(target, viewer.frameSeconds());
        sky.update(viewer.lookAt()->eye);
        {
            const vsg::dvec3 lla = ellipsoid->convertECEFToLatLongAltitude(viewer.lookAt()->eye);
            controls->eyeLatDeg.store(lla.x, std::memory_order_relaxed);
            controls->eyeLonDeg.store(lla.y, std::memory_order_relaxed);
            controls->eyeAltM.store(lla.z, std::memory_order_relaxed);
            controls->eyeDistanceM.store(camera->distance(), std::memory_order_relaxed);
        }

        if (opt.trace >= 0 && batch && static_cast<std::size_t>(opt.trace) < batch->states.size()) {
            static double lastTrace = -1.0;
            if (batch->simTime - lastTrace >= 1.0) {
                lastTrace = batch->simTime;
                const auto& st = batch->states[static_cast<std::size_t>(opt.trace)];
                std::printf("trace t=%6.2f v%d lat %.5f lon %.5f alt %8.2f agl %8.2f tas %7.2f vz %7.2f roll %6.1f pitch %6.1f ground %d%s\n",
                            batch->simTime, opt.trace, units::radiansToDegrees(st.latitudeRad), units::radiansToDegrees(st.longitudeRad),
                            st.altitudeMslM, st.altitudeAglM, st.airspeedTrueMs, -st.velocityNedMs[2],
                            units::radiansToDegrees(st.eulerRad[0]), units::radiansToDegrees(st.eulerRad[1]), st.onGround ? 1 : 0,
                            st.diverged ? " DIVERGED" : "");
                std::fflush(stdout);
            }
        }
        static int tileTick = 0;
        if (terrain && batch && ++tileTick % 60 == 0)
            for (const auto& st : batch->states) terrain->requestAround(st.latitudeRad, st.longitudeRad);
        gui->setBatch(batch);
        controls->fps.store(viewer.fps(), std::memory_order_relaxed);
        controls->frameMs.store(viewer.frameSeconds() * 1e3, std::memory_order_relaxed);

        if (!viewer.frame()) break;

        if (opt.stats > 0.0) {
            const auto& t = viewer.timing();
            stats.window += viewer.frameSeconds();
            stats.maxFrame = std::max(stats.maxFrame, viewer.frameSeconds());
            ++stats.frames;
            stats.sum.app += t.app; stats.sum.advance += t.advance; stats.sum.events += t.events; stats.sum.update += t.update;
            stats.sum.record += t.record; stats.sum.present += t.present; stats.sum.sleep += t.sleep;
            if (stats.window >= 1.0) {
                const double cpu = platform::processCpuSeconds();
                const double n = static_cast<double>(stats.frames);
                std::printf("stats t=%5.1f  %5.1f fps  frame avg %5.2f max %5.2f ms | app %4.2f advance %5.2f events %4.2f update %4.2f record %4.2f present %5.2f sleep %5.2f ms | cpu %5.1f%% | vehicles %zu\n",
                            wallSeconds, n / stats.window, stats.window / n * 1e3, stats.maxFrame * 1e3, stats.sum.app / n * 1e3, stats.sum.advance / n * 1e3,
                            stats.sum.events / n * 1e3, stats.sum.update / n * 1e3, stats.sum.record / n * 1e3, stats.sum.present / n * 1e3,
                            stats.sum.sleep / n * 1e3, (cpu - stats.cpu0) / stats.window * 100.0,
                            static_cast<std::size_t>(std::count(alive.begin(), alive.end(), 1)));
                std::fflush(stdout);
                stats = Stats{};
                stats.cpu0 = cpu;
            }
            if (wallSeconds >= opt.stats) break;
        }

        if (opt.probe && batch && runner) {
            static std::vector<double> speeds, distances;
            static vsg::dvec3 lastPos;
            static bool havePos = false;
            static int warmup = 60;
            if (warmup > 0) { --warmup; havePos = false; }
            const auto& st = (opt.interpolate ? interpolator.states() : batch->states)[static_cast<std::size_t>(selected)];
            const vsg::dvec3 pos(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
            if (havePos && viewer.frameSeconds() > 0.0) {
                // Per displayed frame (the monitor's cadence), not per CPU-measured frame time.
                speeds.push_back(vsg::length(pos - lastPos) * viewer.fps());
                distances.push_back(vsg::length(viewer.lookAt()->eye - pos));
                if (std::getenv("FSIM_PROBE_TRACE") && speeds.size() <= 60)
                    std::printf("frame %3zu dt %6.3f ms  speed %6.2f  renderTime %.4f  simTime %.4f\n", speeds.size(), viewer.frameSeconds() * 1e3,
                                speeds.back(), interpolator.renderSimTime(), batch->simTime);
            }
            lastPos = pos;
            havePos = true;
            if (speeds.size() >= 600) {
                auto summarise = [](const std::vector<double>& v) {
                    double mean = 0, m2 = 0, lo = 1e300, hi = -1e300;
                    for (double x : v) { mean += x; lo = std::min(lo, x); hi = std::max(hi, x); }
                    mean /= static_cast<double>(v.size());
                    for (double x : v) m2 += (x - mean) * (x - mean);
                    return std::tuple{mean, std::sqrt(m2 / static_cast<double>(v.size())), lo, hi};
                };
                const auto [sm, ss, slo, shi] = summarise(speeds);
                const auto [dm, ds, dlo, dhi] = summarise(distances);
                std::printf("probe (%s, %zu frames): apparent speed mean %.2f m/s sd %.2f min %.2f max %.2f | "
                            "eye-target mean %.2f m sd %.4f min %.2f max %.2f | sim %.0f veh-steps/s simTime %.2f tas %.1f fps %.0f\n",
                            opt.interpolate ? "interpolated" : "sample-and-hold", speeds.size(), sm, ss, slo, shi, dm, ds, dlo, dhi,
                            runner->throughput(), batch->simTime, st.airspeedTrueMs, viewer.fps());
                break;
            }
        }
    }

    if (runner) runner->stop();
    return 0;
}
