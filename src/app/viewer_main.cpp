// flightsim-viewer.exe - full-Earth VSG viewer attached to a live simulation
// (design 4.4 entry point 3, section 8).
//
// The simulation runs on its own thread (sim::SimRunner) paced to wall time;
// the render thread reads snapshots through the triple buffer and never blocks
// the simulation.

#include "app/DemoAutopilot.h"
#include "core/Log.h"
#include "core/Rng.h"
#include "core/Units.h"
#include "io/AssetResolver.h"
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
#include "world/Trails.h"
#include "world/Terrain.h"
#include "world/VehicleVisuals.h"

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/SendEventsToImGui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using namespace fsim;

namespace {

struct ViewerOptions {
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

    world::EarthSettings earth;
    unsigned terrainZoom = 12;   // physics ground sampling level (~30 m/px at mid latitudes)
    bool sun = true;             // sun + ambient light instead of the headlight
    double sunUtcHours = -1.0;   // <0 = now
    bool onGround = false;       // spawn parked on the terrain instead of airborne
    std::string modelPath;
    double modelScale = 1.0;
    std::string modelForward, modelUp; // empty = from the manifest / glTF default
    double modelScaleOverride = 0.0;    // >0 overrides the manifest

    render::ViewerSettings window;
    int cameraMode = 0; // 0 chase, 1 orbit, 2 overview
    double chaseDistance = 40.0; // chase camera distance behind the vehicle (m)
    double chaseAzimuth = 180.0, chaseElevation = 14.0; // initial view offset (deg)
    bool probe = false;       // print motion-smoothness statistics and exit after ~5 s
    int trace = -1;           // --trace <i>: print vehicle i's state once per second
    bool interpolate = true;  // --no-interpolate reproduces sample-and-hold for comparison
    bool help = false;
};

void usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  --aircraft <name>        JSBSim aircraft (c172x)\n"
        "  --jsbsim-root <dir>      JSBSim data tree (auto)\n"
        "  --vehicles <n>           number of vehicles (8)\n"
        "  --workers <n>            sim workers (auto)\n"
        "  --time-factor <x>        simulation speed (1.0)\n"
        "  --lat <deg> --lon <deg>  spawn centre (KSFO)\n"
        "  --alt <m>                spawn altitude MSL (1500)\n"
        "  --spread <deg>           spawn scatter (0.03)\n"
        "  --imagery satellite|osm|bing|none|<url template with {z}/{x}/{y}>   (satellite = Esri World Imagery)\n"
        "  --elevation terrarium|none|<url template>   relief from Terrarium-encoded tiles (default terrarium)\n"
        "  --terrain-zoom <z>       tile level used for physics ground height (12)\n"
        "  --no-sun                 headlight instead of sun + ambient lighting\n"
        "  --sun-utc <hours>        sun position for this UTC hour (default: now)\n"
        "  --on-ground              spawn parked on the terrain (brakes on)\n"
        "  --bing-key <key>         Bing Maps key for --imagery bing\n"
        "  --max-level <n>          custom pyramid max level (17)\n"
        "  --model <file>           glTF/OBJ vehicle model (placeholder if omitted)\n"
        "  --model-scale <x>        model scale (1.0)\n"
        "  --width <px> --height <px> --fullscreen --msaa <1|2|4|8> --fov <deg>\n"
        "  --debug-layer            Vulkan validation layer\n"
        "  --camera chase|orbit|overview   initial camera (chase)\n"
        "  --probe                  print motion smoothness statistics after ~5 s and exit\n"
        "  --trace <i>              print vehicle i's state once per second\n"
        "  --no-interpolate         draw raw snapshots (sample-and-hold) instead of interpolating\n"
        "  --log-level <lvl>\n"
        "Keys: space pause, . step, tab next vehicle, c camera, -/= zoom, r reset view, [ ] time factor, l list, m monitor, n labels, t trails, esc quit\n"
        "Mouse: left drag orbits around the vehicle, wheel / right drag changes distance, middle click resets the view\n",
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
            else if (a == "--msaa") {
                const int s = std::stoi(next());
                o.window.samples = s >= 8 ? VK_SAMPLE_COUNT_8_BIT : s >= 4 ? VK_SAMPLE_COUNT_4_BIT : s >= 2 ? VK_SAMPLE_COUNT_2_BIT : VK_SAMPLE_COUNT_1_BIT;
            } else if (a == "--debug-layer") o.window.debugLayer = true;
            else if (a == "--chase-distance") o.chaseDistance = std::stod(next());
            else if (a == "--chase-azimuth") o.chaseAzimuth = std::stod(next());
            else if (a == "--chase-elevation") o.chaseElevation = std::stod(next());
            else if (a == "--camera") {
                const std::string v = next();
                o.cameraMode = v == "orbit" ? 1 : v == "overview" ? 2 : 0;
            } else if (a == "--probe") o.probe = true;
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

} // namespace

int main(int argc, char** argv) {
    ViewerOptions opt;
    if (!parse(argc, argv, opt)) {
        usage(argv[0]);
        return 2;
    }
    if (opt.help) {
        usage(argv[0]);
        return 0;
    }

    // --- Rendering (window first: the terrain ground provider uses VSG's readers) ---
    render::Viewer viewer;
    opt.window.title = "flightsim - " + opt.aircraft;
    if (opt.earth.elevationUrl.empty()) opt.earth.elevationUrl = world::kAwsTerrariumUrl;
    if (opt.earth.source == world::EarthSettings::Source::None) opt.earth.elevationUrl.clear();
    opt.window.headlight = !opt.sun;
    if (!viewer.create(opt.window)) return 1;

    // --- Simulation ------------------------------------------------------------
    io::AssetResolver assets;
    const auto root = assets.jsbsimRoot(opt.jsbsimRoot);
    if (!root) {
        LOG_ERROR("app") << "JSBSim data root not found; pass --jsbsim-root <dir>";
        return 1;
    }

    // Physics ground: the same elevation tiles the renderer displays (ADR-7).
    std::shared_ptr<sim::GroundProvider> ground;
    std::shared_ptr<world::TileGroundProvider> terrain; // non-null when relief is on
    if (!opt.earth.elevationUrl.empty()) {
        auto tiles = std::make_shared<world::TileGroundProvider>(opt.earth.elevationUrl, opt.earth.elevationEncoding,
                                                                 opt.terrainZoom, viewer.options());
        tiles->prefetch(units::degreesToRadians(opt.latitudeDeg), units::degreesToRadians(opt.longitudeDeg),
                        6000.0 + opt.spreadDeg * 111000.0);
        ground = tiles;
        terrain = tiles;
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
            // Altitude is MSL; keep at least 300 m above the terrain under the spawn point.
            const double terrainM = ground->heightAboveEllipsoidM(units::degreesToRadians(ic.latitudeDeg),
                                                                  units::degreesToRadians(ic.longitudeDeg));
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
    auto runner = std::make_unique<sim::SimRunner>(std::move(pool), opt.frameSkip,
        [autopilot, initialConditions](const sim::SnapshotBatch& prev, sim::VehiclePool& vehicles, std::vector<sim::ControlInputs>& out) {
            // A diverged vehicle is reset to its initial conditions (design 13 "Errors").
            for (std::size_t i = 0; i < prev.states.size(); ++i) {
                if (!prev.states[i].diverged) continue;
                LOG_WARN("app") << "vehicle " << i << " diverged; resetting to its initial conditions";
                vehicles.vehicle(i).reset((*initialConditions)[i]);
                autopilot->forget(i);
            }
            autopilot->compute(prev.states, out);
        });
    runner->setTimeFactor(opt.timeFactor);
    LOG_INFO("app") << opt.vehicles << " x " << opt.aircraft << " on " << workers << " worker(s), agent rate "
                    << 1.0 / (opt.dt * opt.frameSkip) << " Hz";

    // --- Scene -------------------------------------------------------------------
    auto ellipsoid = vsg::EllipsoidModel::create(); // WGS-84
    auto scene = vsg::Group::create();
    int day = 172; double hours = 12.0;
    world::currentUtc(day, hours);
    if (opt.sunUtcHours >= 0.0) hours = opt.sunUtcHours;
    const vsg::dvec3 sunDir = world::sunDirectionEcef(day, hours);
    world::SkyDome sky(viewer.options(), sunDir); // first child: painted before everything, no depth
    scene->addChild(sky.node());
    if (opt.sun) scene->addChild(world::createSunLight(day, hours));
    if (auto earth = world::createEarth(opt.earth, viewer.options(), ellipsoid)) scene->addChild(earth);

    world::VehicleVisuals::Settings visualSettings;
    // Model: --model, else the sample aircraft from the asset tree, else the placeholder.
    if (opt.modelPath.empty()) {
        if (auto sample = assets.find("models/Cesium_Air.glb")) opt.modelPath = sample->string();
    } else if (opt.modelPath == "none") {
        opt.modelPath.clear();
    }
    visualSettings.modelPath = opt.modelPath;
    world::VehicleVisuals::applyManifest(visualSettings); // forward/up/scale from <model>.manifest
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
    world::VehicleVisuals visuals(opt.vehicles, visualSettings, viewer.options());
    scene->addChild(visuals.node());
    world::Trails trails(opt.vehicles, 900, 0.25, viewer.options()); // ~3.75 min of path per vehicle
    scene->addChild(trails.node());

    auto controls = std::make_shared<ui::ViewerControls>();
    controls->timeFactor.store(opt.timeFactor);
    controls->cameraMode.store(opt.cameraMode);
    auto gui = ui::MonitorGui::create(controls, opt.aircraft);
    auto imgui = vsgImGui::RenderImGui::create(viewer.window(), gui);
    ImGui::GetIO().IniFilename = nullptr; // layout is deterministic; don't litter the cwd with imgui.ini
    if (const double dpi = platform::systemDpiScale(); dpi > 1.01) { // process is DPI-aware: scale the UI to match
        ImGui::GetIO().FontGlobalScale = static_cast<float>(dpi);
        ImGui::GetStyle().ScaleAllSizes(static_cast<float>(dpi));
    }

    // ImGui must see events first (design 9.6), then our keys, then the trackball.
    viewer.addEventHandler(vsgImGui::SendEventsToImGui::create());
    viewer.addEventHandler(ui::KeyHandler::create(controls, static_cast<int>(opt.vehicles)));
    if (!viewer.setScene(scene, ellipsoid, imgui)) return 1;

    // Play the model's animations (propellers) in a loop; shared subgraph, so one play covers all vehicles.
    for (auto& anim : visuals.animations()) viewer.viewer()->animationManager->play(anim);

    auto camera = world::CameraController::create(viewer.camera(), viewer.lookAt(), ellipsoid);
    camera->setChaseOffset(opt.chaseDistance, opt.chaseElevation, opt.chaseAzimuth);
    viewer.addEventHandler(camera); // mouse orbit / distance; after ImGui so panels keep the pointer

    // Place the first snapshot so the camera has a target before the sim thread runs.
    if (const auto* first = runner->snapshots().acquire()) {
        visuals.update(Span<const sim::VehicleState>(first->states));
        camera->update(first->states[0], 0.0);
        gui->setBatch(first);
    }
    runner->start();

    // --- Frame loop ------------------------------------------------------------
    world::Interpolator interpolator(opt.vehicles);
    const sim::SnapshotBatch* batch = runner->snapshots().current();
    if (batch) interpolator.push(*batch);
    while (viewer.active() && !controls->quit.load(std::memory_order_relaxed)) {
        // Controls -> sim
        const bool paused = controls->paused.load(std::memory_order_relaxed);
        const double timeFactor = controls->timeFactor.load(std::memory_order_relaxed);
        runner->setPaused(paused);
        runner->setTimeFactor(timeFactor);
        if (controls->singleStep.exchange(false)) runner->singleStep();

        // Sim -> scene: snapshots arrive at the agent rate; the interpolator turns
        // them into smooth per-frame states against a render-side sim clock.
        if (const auto* fresh = runner->snapshots().acquire()) {
            batch = fresh;
            interpolator.push(*fresh);
        }
        const int selected = std::clamp(controls->selectedVehicle.load(std::memory_order_relaxed), 0, static_cast<int>(opt.vehicles) - 1);
        if (batch) {
            interpolator.update(viewer.frameSeconds(), timeFactor, paused);
            const auto& states = opt.interpolate ? interpolator.states() : batch->states;
            visuals.update(Span<const sim::VehicleState>(states));
            visuals.setSelected(selected);
            camera->setMode(static_cast<world::CameraController::Mode>(controls->cameraMode.load(std::memory_order_relaxed)));
            if (const double z = controls->cameraZoom.exchange(1.0); z != 1.0) camera->zoom(z);
            if (controls->cameraReset.exchange(false)) camera->resetView();
            camera->update(states[static_cast<std::size_t>(selected)], viewer.frameSeconds());
            sky.update(viewer.lookAt()->eye);

            trails.setSelected(selected);
            trails.setVisible(controls->showTrails.load(std::memory_order_relaxed));
            trails.update(Span<const sim::VehicleState>(batch->states), batch->simTime);

            // Vehicle labels: project ECEF positions to window pixels (Vulkan NDC, y down).
            gui->setShowLabels(controls->showLabels.load(std::memory_order_relaxed));
            std::vector<ui::MonitorGui::Label> labels;
            const vsg::dmat4 viewProj = viewer.camera()->projectionMatrix->transform() * viewer.camera()->viewMatrix->transform();
            const auto extent = viewer.window()->extent2D();
            for (std::size_t i = 0; i < states.size(); ++i) {
                const vsg::dvec4 clip = viewProj * vsg::dvec4(states[i].positionEcef[0], states[i].positionEcef[1], states[i].positionEcef[2], 1.0);
                if (clip.w <= 0.0) continue;
                const double nx = clip.x / clip.w, ny = clip.y / clip.w;
                if (nx < -1.1 || nx > 1.1 || ny < -1.1 || ny > 1.1) continue;
                ui::MonitorGui::Label l;
                l.x = static_cast<float>((nx * 0.5 + 0.5) * extent.width);
                l.y = static_cast<float>((ny * 0.5 + 0.5) * extent.height);
                l.text = "v" + std::to_string(i) + (states[i].diverged ? " !" : "");
                l.selected = static_cast<int>(i) == selected;
                labels.push_back(std::move(l));
            }
            gui->setLabels(std::move(labels));

            controls->simTime.store(batch->simTime, std::memory_order_relaxed);
            controls->snapshotSequence.store(batch->sequence, std::memory_order_relaxed);
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
        // Keep the physics tiles around every vehicle warm (background fetch).
        static int tileTick = 0;
        if (terrain && batch && ++tileTick % 60 == 0)
            for (const auto& st : batch->states) terrain->requestAround(st.latitudeRad, st.longitudeRad);
        gui->setBatch(batch);
        controls->fps.store(viewer.fps(), std::memory_order_relaxed);
        controls->frameMs.store(viewer.frameSeconds() * 1e3, std::memory_order_relaxed);
        controls->simThroughput.store(runner->throughput(), std::memory_order_relaxed);

        if (!viewer.frame()) break;

        if (opt.probe && batch) {
            // Motion-smoothness probe: per-frame speed of the followed vehicle as seen
            // by the renderer (should be near-constant), and eye-target distance.
            static std::vector<double> speeds, distances;
            static vsg::dvec3 lastPos;
            static bool havePos = false;
            static int warmup = 60; // skip start-up frames (clock snap, vsync not yet engaged)
            if (warmup > 0) { --warmup; havePos = false; }
            const auto& st = (opt.interpolate ? interpolator.states() : batch->states)[static_cast<std::size_t>(selected)];
            const vsg::dvec3 pos(st.positionEcef[0], st.positionEcef[1], st.positionEcef[2]);
            if (havePos && viewer.frameSeconds() > 0.0) {
                speeds.push_back(vsg::length(pos - lastPos) / viewer.frameSeconds());
                distances.push_back(vsg::length(viewer.lookAt()->eye - pos));
            }
            lastPos = pos;
            havePos = true;
            if (speeds.size() >= 600) {
                auto stats = [](const std::vector<double>& v) {
                    double mean = 0, m2 = 0, lo = 1e300, hi = -1e300;
                    for (double x : v) { mean += x; lo = std::min(lo, x); hi = std::max(hi, x); }
                    mean /= static_cast<double>(v.size());
                    for (double x : v) m2 += (x - mean) * (x - mean);
                    return std::tuple{mean, std::sqrt(m2 / static_cast<double>(v.size())), lo, hi};
                };
                const auto [sm, ss, slo, shi] = stats(speeds);
                const auto [dm, ds, dlo, dhi] = stats(distances);
                std::printf("probe (%s, %zu frames): apparent speed mean %.2f m/s sd %.2f min %.2f max %.2f | "
                            "eye-target mean %.2f m sd %.4f min %.2f max %.2f | sim %.0f veh-steps/s simTime %.2f tas %.1f fps %.0f\n",
                            opt.interpolate ? "interpolated" : "sample-and-hold", speeds.size(), sm, ss, slo, shi, dm, ds, dlo, dhi,
                            runner->throughput(), batch->simTime, st.airspeedTrueMs, viewer.fps());
                break;
            }
        }
    }

    runner->stop();
    return 0;
}
