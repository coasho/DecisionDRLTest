// Terrain decoding for the viewer's tiles (VSG data types, no Vulkan):
// vertex-grid resampling that keeps tile seams closed, and elevation tiles
// synthesised below the pyramid's deepest level.
#include "world/CameraController.h"
#include "world/ElevationUpsampler.h"
#include "world/Terrain.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

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
        return vsg::dvec2((clip.x / w + 1.0) * kWidth * 0.5, (1.0 - clip.y / w) * kHeight * 0.5);
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

TEST_CASE("zooming holds the point under the cursor in place", "[camera]") {
    // What zoomToMouse actually promises: whatever the pointer is over stays
    // where it is on screen as the view closes in. Checking only that the
    // focus shifts the right way would miss getting the magnitude wrong,
    // which on a globe means missing the curvature.
    Rig rig;
    rig.camera->setFreeView(45.0, 8.0, 0.0, 3.0e5);
    rig.camera->update(nullptr, 1.0 / 60.0);

    const int cursorX = 620, cursorY = 200;
    auto move = vsg::MoveEvent::create();
    move->x = cursorX;
    move->y = cursorY;
    rig.camera->apply(*move);

    const auto target = rig.camera->groundUnderCursor();
    REQUIRE(target.has_value());

    // The projection has to agree with where the cursor is, or the rest of
    // this measures nothing.
    const vsg::dvec2 before = rig.toScreen(*target);
    REQUIRE_THAT(before.x, Catch::Matchers::WithinAbs(static_cast<double>(cursorX), 2.0));
    REQUIRE_THAT(before.y, Catch::Matchers::WithinAbs(static_cast<double>(cursorY), 2.0));

    // Fourteen notches in - a six-fold zoom - letting the smoothed distance
    // settle between each.
    for (int i = 0; i < 14; ++i) {
        auto scroll = vsg::ScrollWheelEvent::create();
        scroll->delta = vsg::vec3(0.0f, 1.0f, 0.0f);
        rig.camera->apply(*scroll);
        for (int k = 0; k < 120; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
    }

    // Measured drift over that zoom is about 2 px across and 4 px down, from
    // 17 px when the camera was still being rotated by its own focus moving.
    // What is left is the view's up vector following the focus, and the
    // target being re-picked from a cursor ray each notch; osgEarth re-picks
    // per scroll event too.
    const vsg::dvec2 after = rig.toScreen(*target);
    INFO("drifted from (" << cursorX << ", " << cursorY << ") to (" << after.x << ", " << after.y << ")");
    REQUIRE_THAT(after.x, Catch::Matchers::WithinAbs(static_cast<double>(cursorX), 6.0));
    REQUIRE_THAT(after.y, Catch::Matchers::WithinAbs(static_cast<double>(cursorY), 6.0));
}

TEST_CASE("zooming out and back leaves the view where it was", "[camera]") {
    // Far out the view is tilted towards straight down on purpose, so the
    // globe is seen from above its focus rather than at an angle that puts the
    // eye past the horizon. That tilt is a function of distance, so coming
    // back in has to undo it. If it does not, the wheel rotates the Earth -
    // and the farther out it went, the more it rotates.
    Rig rig;
    rig.camera->setFreeView(20.0, 10.0, 0.0, 3.0e5, 180.0, 25.0);
    const auto settle = [&rig] {
        for (int k = 0; k < 120; ++k) rig.camera->update(nullptr, 1.0 / 60.0);
    };
    settle();
    const vsg::dvec3 before = vsg::normalize(rig.lookAt->eye - rig.lookAt->center);
    const double elevationBefore = rig.camera->elevationDeg();
    const double distanceBefore = rig.camera->distance();

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
    // Out to ten times the distance, well into the tilt, and back again.
    for (int i = 0; i < 20; ++i) notch(-1.0f);
    REQUIRE(rig.camera->distance() > 2.0e6);
    for (int i = 0; i < 20; ++i) notch(1.0f);

    REQUIRE_THAT(rig.camera->distance(), Catch::Matchers::WithinRel(distanceBefore, 1e-6));
    const vsg::dvec3 after = vsg::normalize(rig.lookAt->eye - rig.lookAt->center);
    const double turn = std::acos(std::clamp(vsg::dot(before, after), -1.0, 1.0)) * 57.29577951308232;
    INFO("out and back turned the view " << turn << " deg; elevation " << elevationBefore << " -> "
                                         << rig.camera->elevationDeg());
    REQUIRE_THAT(rig.camera->elevationDeg(), Catch::Matchers::WithinAbs(elevationBefore, 1.0));
    REQUIRE(turn < 1.0);
}
