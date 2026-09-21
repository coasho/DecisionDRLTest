// Terrain decoding for the viewer's tiles (VSG data types, no Vulkan):
// vertex-grid resampling that keeps tile seams closed, and elevation tiles
// synthesised below the pyramid's deepest level.
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
