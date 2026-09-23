// Terrain decoding for the viewer's tiles (VSG data types, no Vulkan):
// vertex-grid resampling that keeps tile seams closed, and elevation tiles
// synthesised below the pyramid's deepest level.
#include "core/HeightGrid.h"
#include "world/CameraController.h"
#include "world/ElevationUpsampler.h"
#include "world/OfflineTiles.h"
#include "world/Scattering.h"
#include "world/Terrain.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <filesystem>

using namespace fsim;
using Catch::Matchers::WithinAbs;

namespace {

/// A 256x256 float tile whose height is a linear function of the geographic position
/// (fraction of the tile), so any correct resampling reproduces it exactly.
vsg::ref_ptr<vsg::floatArray2D> ramp(double ox, double oy, double scale, double slopeX, double slopeY) {
    auto t = vsg::floatArray2D::create(256, 256, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
    t->properties.origin = vsg::TOP_LEFT;
    for (std::uint32_t y = 0; y < 256; ++y)
        for (std::uint32_t x = 0; x < 256; ++x) {
            const double fx = ox + (x + 0.5) / 256.0 * scale, fy = oy + (y + 0.5) / 256.0 * scale; // pixel centre, in "world" fractions
            t->set(x, y, static_cast<float>(slopeX * fx + slopeY * fy));
        }
    return t;
}

/// Serves the ramp as the level-15 tile 5/7 (and nothing else).
class FakeTiles : public vsg::Inherit<vsg::ReaderWriter, FakeTiles> {
public:
    vsg::ref_ptr<vsg::Object> read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options>) const override {
        ++reads;
        if (filename.string() == "https://tiles/15/5/7.png") return ramp(0.0, 0.0, 1.0, 1000.0, 100.0);
        return {};
    }
    mutable int reads = 0;
};

} // namespace

TEST_CASE("elevation resampling places texel k at vertex k, extrapolating at the edges", "[terrain][render]") {
    // Heights rise 1000 m across the tile in x: vertex k of a 64-vertex grid sits at k/63.
    auto src = ramp(0.0, 0.0, 1.0, 1000.0, 0.0);
    auto out = world::decodeElevation(src, world::ElevationEncoding::Float, 64);
    REQUIRE(out);
    REQUIRE(out->width() == 64);
    REQUIRE(out->height() == 64);
    for (std::uint32_t k = 0; k < 64; ++k) REQUIRE_THAT(out->at(k, 10), WithinAbs(1000.0 * k / 63.0, 0.05));
    // Both edges are exact (the first pixel centre is half a pixel inside the tile: extrapolated).
    REQUIRE_THAT(out->at(0, 0), WithinAbs(0.0, 0.05));
    REQUIRE_THAT(out->at(63, 0), WithinAbs(1000.0, 0.05));
    // A neighbouring tile to the right starts where this one ends.
    auto right = world::decodeElevation(ramp(1.0, 0.0, 1.0, 1000.0, 0.0), world::ElevationEncoding::Float, 64);
    REQUIRE_THAT(right->at(0, 0), WithinAbs(out->at(63, 0), 0.05));
    // Full resolution is passed through untouched.
    auto same = world::decodeElevation(src, world::ElevationEncoding::Terrarium, 0);
    REQUIRE(same.get() == src.get());
}

TEST_CASE("the camera and the mesh are built from one surface", "[terrain][render]") {
    // The mesh the vertex shader displaces and the heights the camera is told
    // have to be the same surface. They were not: world box-filtered and
    // resampled its raster to 64 vertices while io::TerrainTiles sampled the
    // raw 256, and a box filter lowers peaks and raises valley floors. Over 40
    // cached tiles of real relief the drawn mesh sat up to 11.3 m above what
    // the camera believed - more than the clearance it keeps at close range,
    // so the eye finished up inside the hillside. Both now call
    // core::resampleHeightGrid; this is the guard that they still do.
    //
    // A notched ramp, because the notch is exactly what filtering fills in.
    constexpr std::uint32_t kSize = 256, kMesh = 64;
    auto src = vsg::floatArray2D::create(kSize, kSize, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
    std::vector<float> raw(static_cast<std::size_t>(kSize) * kSize);
    for (std::uint32_t y = 0; y < kSize; ++y)
        for (std::uint32_t x = 0; x < kSize; ++x) {
            const double along = 1000.0 * x / (kSize - 1.0);
            const bool inNotch = x > 120 && x < 130;
            const float h = static_cast<float>(inNotch ? along - 400.0 : along);
            raw[static_cast<std::size_t>(y) * kSize + x] = h;
            src->set(x, y, h);
        }

    auto mesh = world::decodeElevation(src, world::ElevationEncoding::Float, kMesh);
    REQUIRE(mesh);
    REQUIRE(mesh->width() == kMesh);
    const auto camera = core::resampleHeightGrid(raw.data(), kSize, kMesh);
    REQUIRE(camera.size() == static_cast<std::size_t>(kMesh) * kMesh);

    double worst = 0.0;
    for (std::uint32_t y = 0; y < kMesh; ++y)
        for (std::uint32_t x = 0; x < kMesh; ++x)
            worst = std::max(worst, std::abs(static_cast<double>(mesh->at(x, y))
                                             - camera[static_cast<std::size_t>(y) * kMesh + x]));
    INFO("largest disagreement between the drawn mesh and the camera's surface: " << worst << " m");
    REQUIRE(worst == 0.0);

    // And the filtering is doing something worth agreeing about: over the
    // notch the resampled surface stands well above the raster it came from.
    double lift = 0.0;
    for (std::uint32_t x = 0; x < kMesh; ++x) {
        const double fraction = static_cast<double>(x) / (kMesh - 1.0);
        const auto texel = static_cast<std::uint32_t>(std::lround(fraction * (kSize - 1)));
        lift = std::max(lift, static_cast<double>(camera[static_cast<std::size_t>(10) * kMesh + x])
                                  - static_cast<double>(raw[static_cast<std::size_t>(10) * kSize + texel]));
    }
    INFO("the filtered surface rises " << lift << " m above the raw raster over the notch");
    REQUIRE(lift > 50.0);
}

TEST_CASE("an offline pyramid's holes are filled from the nearest real ancestor", "[terrain][render]") {
    // VSG draws a tile only with both layers present: elevation without
    // imagery is not drawn at all, imagery without elevation is drawn at sea
    // level, and one failed sibling stops all four refining. A map fetched to
    // a budget is full of such holes - the whole planet at level 7 was flat -
    // so the offline reader must make each missing tile from its nearest real
    // ancestor wherever anything real exists at that level, and decline where
    // nothing does. Built here from VSG's own file format, so no image codec
    // is involved in what is being tested.
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "fsim_offline_tiles_test";
    fs::remove_all(dir);
    const std::string img = (dir / "img/{z}/{y}/{x}.vsgt").generic_string(); // Esri's order: y before x
    const std::string elev = (dir / "elev/{z}/{x}/{y}.vsgt").generic_string();
    auto at = [](std::string t, unsigned z, unsigned x, unsigned y) {
        auto sub = [&](const char* k, unsigned v) {
            for (auto p = t.find(k); p != std::string::npos; p = t.find(k)) t.replace(p, 3, std::to_string(v));
        };
        sub("{z}", z);
        sub("{x}", x);
        sub("{y}", y);
        return t;
    };
    auto io = vsg::Options::create();
    io->readerWriters = {vsg::VSG::create()};
    auto put = [&](const std::string& p, const vsg::ref_ptr<vsg::Object>& o) {
        fs::create_directories(fs::path(p).parent_path());
        REQUIRE(vsg::write(o, p, io));
    };

    // The root: imagery in four solid quadrants, elevation a ramp west to east.
    constexpr std::uint32_t N = 16;
    const vsg::ubvec4 nw(200, 0, 0, 255), ne(0, 200, 0, 255), sw(0, 0, 200, 255), se(200, 200, 0, 255);
    auto root = vsg::ubvec4Array2D::create(N, N, vsg::Data::Properties{VK_FORMAT_R8G8B8A8_UNORM});
    auto ramp = vsg::floatArray2D::create(N, N, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
    for (std::uint32_t y = 0; y < N; ++y)
        for (std::uint32_t x = 0; x < N; ++x) {
            const bool east = x >= N / 2, south = y >= N / 2;
            root->set(x, y, south ? (east ? se : sw) : (east ? ne : nw));
            ramp->set(x, y, 100.0f * static_cast<float>(x));
        }
    put(at(img, 0, 0, 0), root);
    put(at(elev, 0, 0, 0), ramp);
    // Level 1 has one real imagery tile and no elevation at all: the shape of
    // the global base that drew every level-7 tile on Earth at sea level.
    put(at(img, 1, 1, 0), root);

    auto reader = world::OfflineTiles::create(img, elev, world::ElevationEncoding::Float);
    auto options = vsg::Options::create();
    options->readerWriters = {reader, vsg::VSG::create()};

    // A tile that is really there is left to the ordinary readers.
    REQUIRE_FALSE(reader->read(at(img, 0, 0, 0), options));

    // Missing elevation, north-east quarter of the root: the eastern half of
    // the ramp - not the flat fallback that sat the planet at 0 m.
    auto heights = vsg::read_cast<vsg::floatArray2D>(at(elev, 1, 1, 0), options);
    REQUIRE(heights);
    INFO("west edge " << heights->at(0, N / 2) << " m, east edge " << heights->at(N - 1, N / 2) << " m");
    REQUIRE_THAT(heights->at(0, N / 2), Catch::Matchers::WithinAbs(775.0, 1.0));
    REQUIRE_THAT(heights->at(N - 1, N / 2), Catch::Matchers::WithinAbs(1500.0, 1.0));

    // Missing imagery, south-west quarter: that quarter's colour.
    auto picture = vsg::read_cast<vsg::ubvec4Array2D>(at(img, 1, 0, 1), options);
    REQUIRE(picture);
    const vsg::ubvec4 mid = picture->at(N / 2, N / 2);
    REQUIRE(mid.r == sw.r);
    REQUIRE(mid.g == sw.g);
    REQUIRE(mid.b == sw.b);

    // Level 2 has nothing real in any quad: declined, so the read of the four
    // children fails and the parent stays - refinement stops with the data.
    REQUIRE_FALSE(reader->read(at(elev, 2, 0, 0), options));
    REQUIRE_FALSE(reader->read(at(img, 2, 3, 3), options));
    fs::remove_all(dir);
}

TEST_CASE("aerial perspective takes the ray's zenith from the ellipsoid, not the relief", "[terrain][render]") {
    // How much haze a ray gathers depends on how steeply it crosses the air.
    // Taken from the relief's normal instead, every slope seen edge-on hazed
    // like a horizon and every ridge shone as if wet.
    auto options = vsg::Options::create();
    auto shaderSet = vsg::createPhongShaderSet(options);
    REQUIRE(shaderSet);
    world::addAerialPerspective(*shaderSet);

    vsg::ref_ptr<vsg::ShaderStage> vertex, fragment;
    for (auto& stage : shaderSet->stages) {
        if (stage->stage == VK_SHADER_STAGE_VERTEX_BIT) vertex = stage;
        if (stage->stage == VK_SHADER_STAGE_FRAGMENT_BIT) fragment = stage;
    }
    REQUIRE((vertex && fragment));
    const std::string& vs = vertex->module->source;
    const std::string& fs = fragment->module->source;
    // The up leaves the vertex stage before the relief bends the normal...
    CHECK(vs.find("out vec3 fsimUp;") != std::string::npos);
    CHECK(vs.find("fsimUp = (mv * vec4(vsg_Normal, 0.0)).xyz;") != std::string::npos);
    // ... and the fragment stage measures the ray against it, not the slope.
    CHECK(fs.find("in vec3 fsimUp;") != std::string::npos);
    CHECK(fs.find("dot(fsimView, normalize(fsimUp))") != std::string::npos);
    CHECK(fs.find("dot(fsimView, nd)") == std::string::npos);

    // Both stages compile for the defines a relief tile is drawn with. The
    // patched stages carry source only, so the viewer compiles them at run
    // time too: without glslang the terrain would not draw at all.
    auto compiler = vsg::ShaderCompiler::create();
    REQUIRE(compiler->supported());
    vsg::ShaderStages stages{vertex, fragment};
    CHECK(compiler->compile(stages, {"VSG_TEXTURECOORD_0", "VSG_DIFFUSE_MAP", "VSG_DISPLACEMENT_MAP"}, options));
}

TEST_CASE("elevation upsampler synthesises deeper tiles from the deepest real level", "[terrain][render]") {
    world::ElevationUpsampler up("https://tiles/{z}/{x}/{y}.png", 15, world::ElevationEncoding::Float);
    unsigned z, x, y;
    REQUIRE(up.parse("https://tiles/17/21/30.png", z, x, y));
    REQUIRE((z == 17 && x == 21 && y == 30));
    REQUIRE_FALSE(up.parse("https://images/17/21/30.jpg", z, x, y));
    REQUIRE(up.url(15, 5, 7) == "https://tiles/15/5/7.png");

    auto options = vsg::Options::create();
    auto fake = FakeTiles::create();
    options->readerWriters.push_back(fake);
    // Level 15 itself is not the upsampler's business.
    REQUIRE_FALSE(up.read("https://tiles/15/5/7.png", options));
    // Level 17, tile (21, 30): ancestor (5, 7), quadrant path x = 21 & 3 = 1, y = 30 & 3 = 2 of 4.
    auto tile = up.read("https://tiles/17/21/30.png", options).cast<vsg::floatArray2D>();
    REQUIRE(tile);
    REQUIRE(tile->width() == 256);
    REQUIRE(fake->reads >= 1);
    // The synthesised pixel centres must reproduce the ramp at their true positions.
    for (std::uint32_t j : {0u, 100u, 255u})
        for (std::uint32_t i : {0u, 77u, 255u}) {
            const double fx = 0.25 + (i + 0.5) / 256.0 * 0.25, fy = 0.5 + (j + 0.5) / 256.0 * 0.25;
            REQUIRE_THAT(tile->at(i, j), WithinAbs(1000.0 * fx + 100.0 * fy, 0.05));
        }
    // And through the vertex-grid resampling, a synthesised tile agrees with its parent on the shared edge.
    auto parent = world::decodeElevation(fake->read("https://tiles/15/5/7.png", options).cast<vsg::Data>(), world::ElevationEncoding::Float, 64);
    auto child = world::decodeElevation(up.read("https://tiles/16/10/14.png", options).cast<vsg::Data>(), world::ElevationEncoding::Float, 64); // top-left quadrant
    REQUIRE_THAT(child->at(0, 0), WithinAbs(parent->at(0, 0), 0.05));
    REQUIRE_THAT(child->at(63, 63), WithinAbs(1000.0 * 0.5 + 100.0 * 0.5, 0.05)); // the parent's centre
}

// The globe drag used to be a displacement reprojected through
// latitude/longitude, which is singular at the poles: dragging a pole through
// the centre of the screen swung the focus' longitude, and with it the local
// frame the view direction is built from, so the Earth span out of control.
namespace {

struct Rig {
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoid = vsg::EllipsoidModel::create();
    vsg::ref_ptr<vsg::LookAt> lookAt = vsg::LookAt::create();
    vsg::ref_ptr<fsim::world::CameraController> camera;
    vsg::ref_ptr<vsg::Camera> view;
    static constexpr double kWidth = 800.0, kHeight = 600.0;

    Rig() {
        auto projection = vsg::Perspective::create(30.0, kWidth / kHeight, 1.0, 1.0e7);
        view = vsg::Camera::create(projection, lookAt,
                                   vsg::ViewportState::create(VkExtent2D{static_cast<uint32_t>(kWidth),
                                                                        static_cast<uint32_t>(kHeight)}));
        camera = fsim::world::CameraController::create(view, lookAt, ellipsoid);
    }

    /// Where a world point lands in window pixels, by the same mapping the
    /// controller uses for the cursor.
    vsg::dvec2 toScreen(const vsg::dvec3& world) const {
        const vsg::dmat4 viewProjection = view->projectionMatrix->transform() * view->viewMatrix->transform();
        const vsg::dvec4 clip = viewProjection * vsg::dvec4(world.x, world.y, world.z, 1.0);
        const double w = std::abs(clip.w) > 1e-12 ? clip.w : 1e-12;
        // Vulkan NDC: y down, so +1 is the bottom of the window.
        return vsg::dvec2((clip.x / w + 1.0) * kWidth * 0.5, (clip.y / w + 1.0) * kHeight * 0.5);
    }

    /// Hold the left button - osgEarth's pan - and drag `steps` times by
    /// `dyPixels` (negative = up the screen). Returns the largest
    /// frame-to-frame turn of the view direction, in degrees.
    double drag(int steps, int dyPixels) {
        auto press = vsg::ButtonPressEvent::create();
        press->x = 400;
        press->y = 300;
        press->button = 1;
        camera->apply(*press);
        camera->update(nullptr, 1.0 / 60.0);

        vsg::dvec3 previous = vsg::normalize(lookAt->eye - lookAt->center);
        double worst = 0.0;
        int y = 300;
        for (int i = 0; i < steps; ++i) {
            y += dyPixels;
            auto move = vsg::MoveEvent::create();
            move->x = 400;
            move->y = y;
            move->mask = vsg::BUTTON_MASK_1;
            camera->apply(*move);
            camera->update(nullptr, 1.0 / 60.0);
            const vsg::dvec3 now = vsg::normalize(lookAt->eye - lookAt->center);
            const double turn = std::acos(std::clamp(vsg::dot(previous, now), -1.0, 1.0)) * 57.29577951308232;
            worst = std::max(worst, turn);
            previous = now;
        }
        return worst;
    }
};

} // namespace

TEST_CASE("dragging the globe over a pole does not spin the view", "[camera]") {
    // Dragging down the screen walks the focus north (the eye starts south of
    // it). At osgEarth's pan scale, 5 px a step over 1400 steps is ~2100 km,
    // which carries it from 80 N across the pole and well down the far side.
    Rig rig;
    rig.camera->setFreeView(80.0, 0.0, 0.0, 3.0e5);
    const double worst = rig.drag(1400, 5);
    INFO("largest single-step turn of the view direction: " << worst << " deg");
    REQUIRE(worst < 5.0);

    // The focus really did cross the pole rather than stalling short of it.
    const vsg::dvec3 lla = rig.ellipsoid->convertECEFToLatLongAltitude(rig.camera->focus());
    REQUIRE(std::abs(lla.y) > 90.0);  // longitude flipped to the far side
    REQUIRE(lla.x < 89.0);            // and came back down from the pole
}

TEST_CASE("the right mouse button is not a camera control", "[camera]") {
    Rig rig;
    rig.camera->setFreeView(45.0, 8.0, 0.0, 2.0e4);
    rig.camera->update(nullptr, 1.0 / 60.0);
    const vsg::dvec3 before = rig.camera->focus();
    const double distance = rig.camera->distance();

    auto press = vsg::ButtonPressEvent::create();
    press->x = 400;
    press->y = 300;
    press->button = 3;
    rig.camera->apply(*press);
    REQUIRE_FALSE(press->handled); // left for the window manager

    auto move = vsg::MoveEvent::create();
    move->x = 500;
    move->y = 400;
    move->mask = vsg::BUTTON_MASK_3;
    rig.camera->apply(*move);
    rig.camera->update(nullptr, 1.0 / 60.0);

    REQUIRE(vsg::length(rig.camera->focus() - before) < 1.0);
    REQUIRE_THAT(rig.camera->distance(), Catch::Matchers::WithinRel(distance, 1e-9));
}

TEST_CASE("the cursor's height on screen aims the ray", "[camera]") {
    // Vulkan's NDC has y pointing down, and VSG's perspective matrix carries
    // the flip - the -f in its second row. Reading the cursor with the OpenGL
    // sign aims the ray at the mirror image of where the pointer is, so
    // zooming towards the cursor worked left to right and did nothing useful
    // up and down. The rig's own toScreen() shared the mistake, which is why
    // the round-trip tests could not see it; this one asks about nothing but
    // geometry. A camera looking down at the ground sees farther ground
    // higher up the screen, whatever anyone's sign convention.
    Rig rig;
    rig.camera->setFreeView(20.0, 10.0, 0.0, 2.0e4, 0.0, 20.0);
    rig.camera->update(nullptr, 1.0 / 60.0);

    const auto rangeAt = [&](int y) {
        auto move = vsg::MoveEvent::create();
        move->x = 400;
        move->y = y;
        rig.camera->apply(*move);
        const auto hit = rig.camera->groundUnderCursor();
        REQUIRE(hit.has_value());
        return vsg::length(*hit - rig.lookAt->eye);
    };
    const double high = rangeAt(150), middle = rangeAt(300), low = rangeAt(450);
    INFO("range to the ground at screen y = 150 / 300 / 450: " << high << " / " << middle << " / " << low);
    REQUIRE(high > middle);
    REQUIRE(middle > low);
}

TEST_CASE("the wheel zooms towards the point under the cursor", "[camera]") {
    // osgEarth's zoomToMouse, which it has on by default: what the pointer is
    // over keeps its place on screen, so zooming goes where you are looking.
    Rig rig;
    const auto zoom = [&rig](int cursorX, float notches) {
        rig.camera->setFreeView(45.0, 8.0, 0.0, 2.0e5);
        rig.camera->update(nullptr, 1.0 / 60.0);
        const vsg::dvec3 before = rig.camera->focus();
        auto move = vsg::MoveEvent::create();
        move->x = cursorX;
        move->y = 300;
        rig.camera->apply(*move);
        auto scroll = vsg::ScrollWheelEvent::create();
        scroll->delta = vsg::vec3(0.0f, notches, 0.0f);
        rig.camera->apply(*scroll);
        rig.camera->update(nullptr, 1.0 / 60.0);
        return rig.camera->focus() - before;
    };

    // Cursor well right of centre: zooming in and out move the focus along the
    // same line, in opposite directions.
    const vsg::dvec3 in = zoom(700, 1.0f);
    const vsg::dvec3 out = zoom(700, -1.0f);
    REQUIRE(vsg::length(in) > 1.0);
    REQUIRE(vsg::length(out) > 1.0);
    REQUIRE(vsg::dot(vsg::normalize(in), vsg::normalize(out)) < -0.9);

    // Cursor at the centre is already looking at the focus, so it stays put.
    REQUIRE(vsg::length(zoom(400, 1.0f)) < vsg::length(in) * 0.2);
}

TEST_CASE("turning the view does not zoom it", "[camera]") {
    // The complaint this guards: "rotating the camera with the middle mouse
    // button causes slight, unintended zooming." Framing the planet by sliding
    // only the aim left the eye off the axis it was looking along, so the
    // distance from the eye to whatever was centred depended on the bearing,
    // and every drag changed it. The range the wheel set - from the eye to the
    // ground it is looking at - has to survive a turn, whichever way the
    // camera ends up pointing.
    Rig rig;
    for (const double distance : {8.0e3, 1.0e6, 1.2e7}) {
        rig.camera->setFreeView(20.0, 10.0, 0.0, distance, 180.0, 35.0);
        for (int k = 0; k < 200; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
        const double reach = vsg::length(rig.lookAt->eye - rig.camera->focus());

        double worst = 0.0;
        auto press = vsg::ButtonPressEvent::create();
        press->x = 400;
        press->y = 300;
        press->button = 2; // middle: osgEarth binds it to rotate
        rig.camera->apply(*press);
        int x = 400, y = 300;
        for (int i = 0; i < 60; ++i) {
            x += 4;
            y += 3;
            auto move = vsg::MoveEvent::create();
            move->x = x;
            move->y = y;
            move->mask = vsg::BUTTON_MASK_2;
            rig.camera->apply(*move);
            for (int k = 0; k < 4; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
            worst = std::max(worst, std::abs(vsg::length(rig.lookAt->eye - rig.camera->focus()) - reach) / reach);
        }
        INFO("at " << distance << " m, turning changed the drawn distance by up to "
                   << worst * 100.0 << " % of it");
        REQUIRE(worst < 1e-9);
    }
}

TEST_CASE("under the auto-level range zooming does not turn the view", "[camera]") {
    // Framing the planet by moving the orbit out to the Earth's centre put the
    // eye directly above what it was looking at, so the apparent pitch slid
    // from straight-down to oblique on the way in - "the view becomes
    // increasingly horizontal, looking towards the horizon". The tilt does the
    // framing now, which is visible and undoes itself, and under the range
    // where it applies the wheel turns the view by nothing whatsoever.
    Rig rig;
    rig.camera->setFreeView(20.0, 10.0, 0.0, 1.4e6, 180.0, 20.0); // under the auto-level range
    const auto settle = [&rig] {
        for (int k = 0; k < 200; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
    };
    settle();
    auto move = vsg::MoveEvent::create();
    move->x = 400;
    move->y = 300;
    rig.camera->apply(*move);

    const vsg::dvec3 first = vsg::normalize(rig.lookAt->center - rig.lookAt->eye);
    double worst = 0.0;
    for (int i = 0; i < 24; ++i) {
        auto scroll = vsg::ScrollWheelEvent::create();
        scroll->delta = vsg::vec3(0.0f, 1.0f, 0.0f);
        rig.camera->apply(*scroll);
        settle();
        const vsg::dvec3 now = vsg::normalize(rig.lookAt->center - rig.lookAt->eye);
        worst = std::max(worst, std::acos(std::clamp(vsg::dot(first, now), -1.0, 1.0)) * 57.29577951308232);
    }
    REQUIRE(rig.camera->distance() < 2.0e5); // it really did come in
    INFO("zooming from 1400 km to " << rig.camera->distance() / 1000.0
                                    << " km turned the view " << worst << " deg");
    REQUIRE(worst < 0.01);
}

TEST_CASE("zoomed out, the globe is in the middle of the screen", "[camera]") {
    // The complaint this guards: "the Earth is no longer centered, it appears
    // too low on the screen, almost completely outside the visible area."
    // The focus is a point on the *surface*, and aiming at it puts that point
    // in the middle and leaves the planet hanging below - measured off a
    // rendered frame, the globe's centre sat 27.5 % of the frame height low,
    // with most of it past the bottom edge. Far enough out the subject is the
    // planet, so that is what the camera points at.
    Rig rig;
    for (const double distance : {3.0e7, 1.5e7, 9.6e6}) { // past 1.5 Earth radii: framing fully engaged
        rig.camera->setFreeView(0.0, 0.0, 0.0, distance, 180.0, 14.0);
        for (int k = 0; k < 200; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
        const vsg::dvec2 centreOfEarth = rig.toScreen(vsg::dvec3(0.0, 0.0, 0.0));
        const double dx = (centreOfEarth.x - Rig::kWidth * 0.5) / Rig::kWidth;
        const double dy = (centreOfEarth.y - Rig::kHeight * 0.5) / Rig::kHeight;
        INFO("at " << distance << " m the Earth's centre lands at " << centreOfEarth.x << ", "
                   << centreOfEarth.y << " (offset " << dx << ", " << dy << " of the frame)");
        REQUIRE(std::abs(dx) < 0.02);
        REQUIRE(std::abs(dy) < 0.02);
    }
}

TEST_CASE("close in, the camera still looks at the ground it was given", "[camera]") {
    // The other half of the bargain: framing the planet must not disturb the
    // view at any altitude anyone flies at. Below a quarter of an Earth radius
    // the aim is the focus and nothing else.
    Rig rig;
    for (const double distance : {5.0e3, 1.0e5, 1.0e6}) {
        rig.camera->setFreeView(46.0, 8.0, 0.0, distance, 180.0, 14.0);
        for (int k = 0; k < 200; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
        const vsg::dvec2 focusOnScreen = rig.toScreen(rig.camera->focus());
        INFO("at " << distance << " m the focus lands at " << focusOnScreen.x << ", " << focusOnScreen.y);
        REQUIRE_THAT(focusOnScreen.x, Catch::Matchers::WithinAbs(Rig::kWidth * 0.5, 1.0));
        REQUIRE_THAT(focusOnScreen.y, Catch::Matchers::WithinAbs(Rig::kHeight * 0.5, 1.0));
    }
}

TEST_CASE("under the auto-level range the wheel only changes the distance", "[camera]") {
    // Scrolling used to turn the Earth at every distance, because the tilt
    // was a function of the distance everywhere. It is now confined to the
    // auto-level range - beyond a quarter of an Earth radius, where the
    // subject is the planet - and below that the wheel moves the eye in and
    // out and leaves the bearing and the tilt exactly where they were. Every
    // altitude anyone flies at is below it.
    Rig rig;
    rig.camera->setFreeView(20.0, 10.0, 0.0, 1.0e6, 180.0, 25.0); // well under a quarter of an Earth radius
    const auto settle = [&rig] {
        for (int k = 0; k < 200; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
    };
    settle();
    const double azimuth = rig.camera->azimuthDeg();
    const double elevation = rig.camera->shownElevationDeg();

    // Pointer dead centre, so there is nothing to zoom towards but the focus
    // itself and any turn that appears is the wheel's own doing.
    auto move = vsg::MoveEvent::create();
    move->x = 400;
    move->y = 300;
    rig.camera->apply(*move);

    const auto wheel = [&](float notches, int times) {
        for (int i = 0; i < times; ++i) {
            auto scroll = vsg::ScrollWheelEvent::create();
            scroll->delta = vsg::vec3(0.0f, notches, 0.0f);
            rig.camera->apply(*scroll);
            settle();
        }
    };

    wheel(1.0f, 8);
    REQUIRE(rig.camera->distance() < 5.0e5); // it did zoom
    INFO("after zooming in: azimuth " << azimuth << " -> " << rig.camera->azimuthDeg()
         << ", tilt " << elevation << " -> " << rig.camera->shownElevationDeg());
    REQUIRE_THAT(rig.camera->azimuthDeg(), Catch::Matchers::WithinAbs(azimuth, 0.01));
    REQUIRE_THAT(rig.camera->shownElevationDeg(), Catch::Matchers::WithinAbs(elevation, 0.01));

    wheel(-1.0f, 8);
    INFO("and back out: azimuth " << rig.camera->azimuthDeg() << ", tilt " << rig.camera->shownElevationDeg());
    REQUIRE_THAT(rig.camera->azimuthDeg(), Catch::Matchers::WithinAbs(azimuth, 0.01));
    REQUIRE_THAT(rig.camera->shownElevationDeg(), Catch::Matchers::WithinAbs(elevation, 0.01));
}

TEST_CASE("over flat ground the camera looks where it is pointed", "[camera]") {
    // The complaint this guards: terrain clearance pinned the view about
    // thirty degrees above level whatever the ground was doing, so a level
    // horizontal view was unreachable even high in the air. The margin was
    // demanded in full at every sample along the line of sight, including the
    // ones a probe spacing from a focus that sits on the terrain by
    // definition - asin(80 m / 150 m) is 32 degrees, and that is a property
    // of how often the line is sampled, not of the landscape. Over ground
    // this flat nothing should lift the view at all.
    Rig rig;
    rig.camera->setGroundQuery([](double, double) { return std::optional<double>(0.0); });
    for (const double distance : {2.0e3, 4.0e3, 1.0e4, 5.0e4, 1.5e5}) {
        rig.camera->setFreeView(20.0, 10.0, 0.0, distance, 180.0, 2.0);
        for (int k = 0; k < 250; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
        INFO("at " << distance << " m: asked for 2 deg of tilt, drawn "
                   << rig.camera->shownElevationDeg() << " deg");
        REQUIRE(rig.camera->shownElevationDeg() < 3.0);
    }
}

TEST_CASE("zoom to cursor goes where it is pointed without swinging the bearing", "[camera]") {
    // Asked for, it behaves as osgEarth's does: the centre slides towards
    // what the pointer is over and the orientation relative to the ground is
    // left alone. That trade is deliberate. Holding the point exactly still
    // instead means re-aiming the camera as the focus slides, and over a
    // whole-Earth zoom that swung the compass bearing ten degrees, which is
    // the more disorienting of the two. The point drifts some; the horizon
    // does not roll.
    Rig rig;
    REQUIRE(rig.camera->zoomToCursor()); // on, as osgEarth has it
    rig.camera->setFreeView(45.0, 8.0, 0.0, 3.0e5);
    const auto settle = [&rig] {
        for (int k = 0; k < 120; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
    };
    settle();
    const double elevationBefore = rig.camera->elevationDeg();

    auto move = vsg::MoveEvent::create();
    move->x = 620;
    move->y = 200;
    rig.camera->apply(*move);
    const auto target = rig.camera->groundUnderCursor();
    REQUIRE(target.has_value());
    const double reachBefore = vsg::length(*target - rig.camera->focus());

    for (int i = 0; i < 14; ++i) {
        auto scroll = vsg::ScrollWheelEvent::create();
        scroll->delta = vsg::vec3(0.0f, 1.0f, 0.0f);
        rig.camera->apply(*scroll);
        settle();
    }

    // It closed on what the pointer was over ...
    const double reachAfter = vsg::length(*target - rig.camera->focus());
    INFO("distance from focus to the pointed-at ground: " << reachBefore << " -> " << reachAfter);
    REQUIRE(reachAfter < reachBefore * 0.4);
    // ... without tilting on the way.
    REQUIRE_THAT(rig.camera->elevationDeg(), Catch::Matchers::WithinAbs(elevationBefore, 0.01));
}

TEST_CASE("zooming out and back leaves the view where it was", "[camera]") {
    // Nothing about the view may depend on the distance in a way that does not
    // undo itself. A tilt that grew with distance used to, and the wheel
    // rotated the Earth by the difference; the framing aim that replaced it is
    // a pure function of the distance, so going out and coming back has to
    // land on the same view it left.
    Rig rig;
    rig.camera->setFreeView(20.0, 10.0, 0.0, 3.0e5, 180.0, 25.0);
    const auto settle = [&rig] {
        for (int k = 0; k < 120; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
    };
    settle();
    const vsg::dvec3 before = vsg::normalize(rig.lookAt->eye - rig.lookAt->center);
    const double elevationBefore = rig.camera->elevationDeg();
    const double distanceBefore = rig.camera->distance();
    const vsg::dvec3 focusBefore = rig.camera->focus();
    const double azimuthBefore = rig.camera->azimuthDeg();
    const double shownBefore = rig.camera->shownElevationDeg();

    // Cursor at the centre, so zoom-to-cursor has nothing to move.
    auto move = vsg::MoveEvent::create();
    move->x = 400;
    move->y = 300;
    rig.camera->apply(*move);

    const auto notch = [&rig, &settle](float delta) {
        auto scroll = vsg::ScrollWheelEvent::create();
        scroll->delta = vsg::vec3(0.0f, delta, 0.0f);
        rig.camera->apply(*scroll);
        settle();
    };
    // Out through the whole auto-level range, to where the view is levelled
    // right off, and back again.
    for (int i = 0; i < 40; ++i) notch(-1.0f);
    REQUIRE(rig.camera->distance() > 1.0e7);
    REQUIRE(rig.camera->shownElevationDeg() > 80.0); // it really did level out
    for (int i = 0; i < 40; ++i) notch(1.0f);

    REQUIRE_THAT(rig.camera->distance(), Catch::Matchers::WithinRel(distanceBefore, 1e-6));
    const vsg::dvec3 after = vsg::normalize(rig.lookAt->eye - rig.lookAt->center);
    const double turn = std::acos(std::clamp(vsg::dot(before, after), -1.0, 1.0)) * 57.29577951308232;
    INFO("out and back turned the view " << turn << " deg; elevation " << elevationBefore << " -> "
                                         << rig.camera->elevationDeg()
         << "; focus moved " << vsg::length(rig.camera->focus() - focusBefore)
         << " m; azimuth " << azimuthBefore << " -> " << rig.camera->azimuthDeg()
         << "; shown tilt " << shownBefore << " -> " << rig.camera->shownElevationDeg());
    REQUIRE_THAT(rig.camera->elevationDeg(), Catch::Matchers::WithinAbs(elevationBefore, 1.0));
    REQUIRE(turn < 1.0);
}

TEST_CASE("the eye never goes below the terrain", "[camera]") {
    // osgEarth's rule, and the one kept here: whatever is in the way, the eye
    // itself stays in clear air. The ridge is swept along the line of sight
    // rather than placed once, because a ridge that falls between two samples
    // is a ridge nobody looked at.
    constexpr double kDegPerRad = 57.29577951308232;
    const double ridgeHeightM = 1500.0, ridgeWidthDeg = 0.004;
    double worstClearance = 1e9;
    double worstRidgeLon = 0.0;

    for (int step = 0; step <= 40; ++step) {
        const double ridgeLonDeg = 8.02 + 0.07 * step / 40.0; // across the whole line of sight
        const auto groundAt = [=](double latDeg, double lonDeg) {
            (void)latDeg;
            const double d = (lonDeg - ridgeLonDeg) / ridgeWidthDeg;
            return ridgeHeightM * std::exp(-d * d);
        };
        Rig rig;
        rig.camera->setGroundQuery([=](double latRad, double lonRad) -> std::optional<double> {
            return groundAt(latRad * kDegPerRad, lonRad * kDegPerRad);
        });
        // Focus east of the ridge at sea level, eye 12 km west and low.
        rig.camera->setFreeView(46.0, 8.10, 0.0, 12000.0, 270.0, 2.0);
        for (int k = 0; k < 300; ++k) rig.camera->update(nullptr, 1.0 / 60.0);

        const vsg::dvec3 lla = rig.ellipsoid->convertECEFToLatLongAltitude(rig.lookAt->eye);
        const double clear = lla.z - groundAt(lla.x, lla.y);
        if (clear < worstClearance) {
            worstClearance = clear;
            worstRidgeLon = ridgeLonDeg;
        }
    }
    INFO("least clearance under the eye " << worstClearance << " m, with the ridge at longitude " << worstRidgeLon);
    REQUIRE(worstClearance > 0.0);
}

TEST_CASE("terrain between the eye and the focus does not tilt the view", "[camera]") {
    // The complaint this guards: a level view was unreachable even with the
    // camera high in the air. Measured in the viewer, 60 km across the
    // Bernese Alps with the eye at 18 km - above every summit in Europe -
    // peaks between the focus and the eye still forced 14 degrees of tilt,
    // because the clearance swept the whole line of sight and lifted the view
    // until nothing crossed it. osgEarth makes no such promise and neither
    // does this any more: a hill in the way is not a reason to move a camera
    // that is nowhere near it. The hill hides the view, as hills do.
    constexpr double kDegPerRad = 57.29577951308232;
    const double ridgeLonDeg = 8.02, ridgeWidthDeg = 0.02, ridgeHeightM = 4000.0;
    Rig rig;
    rig.camera->setGroundQuery([=](double, double lonRad) -> std::optional<double> {
        const double d = (lonRad * kDegPerRad - ridgeLonDeg) / ridgeWidthDeg;
        return ridgeHeightM * std::exp(-d * d);
    });
    // Focus well east of the ridge, eye 60 km west of it, asked to look level.
    rig.camera->setFreeView(46.0, 8.20, 0.0, 6.0e4, 270.0, 0.0);
    for (int k = 0; k < 400; ++k) rig.camera->update(nullptr, 1.0 / 60.0);

    INFO("asked for 0 deg of tilt across a 4 km ridge, drawn " << rig.camera->shownElevationDeg() << " deg");
    REQUIRE(rig.camera->shownElevationDeg() < 1.0);
}
