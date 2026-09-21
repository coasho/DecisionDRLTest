// Headless terrain tiles: Terrarium decoding, Mercator addressing, bilinear
// sampling and the cache/loader paths, with tiles synthesised in the test
// (a minimal PNG writer: stored deflate blocks, so no zlib is needed).
#include "io/TerrainTiles.h"

#include "core/Units.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

using namespace fsim;

namespace {

std::uint32_t crc32(const std::uint8_t* data, std::size_t n, std::uint32_t crc = 0) {
    crc = ~crc;
    for (std::size_t i = 0; i < n; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

void put32(std::vector<std::uint8_t>& v, std::uint32_t x) {
    v.push_back(static_cast<std::uint8_t>(x >> 24)); v.push_back(static_cast<std::uint8_t>(x >> 16));
    v.push_back(static_cast<std::uint8_t>(x >> 8)); v.push_back(static_cast<std::uint8_t>(x));
}

void chunk(std::vector<std::uint8_t>& png, const char* type, const std::vector<std::uint8_t>& payload) {
    put32(png, static_cast<std::uint32_t>(payload.size()));
    std::vector<std::uint8_t> body(type, type + 4);
    body.insert(body.end(), payload.begin(), payload.end());
    png.insert(png.end(), body.begin(), body.end());
    put32(png, crc32(body.data(), body.size()));
}

/// RGB8 PNG of `size` x `size` from a height function (Terrarium encoding).
std::vector<std::uint8_t> terrariumPng(std::uint32_t size, const std::function<double(std::uint32_t x, std::uint32_t y)>& heightM) {
    // Raw scanlines: filter byte 0 + RGB per pixel.
    std::vector<std::uint8_t> raw;
    for (std::uint32_t y = 0; y < size; ++y) {
        raw.push_back(0);
        for (std::uint32_t x = 0; x < size; ++x) {
            const double v = heightM(x, y) + 32768.0;
            const double r = std::floor(v / 256.0), g = std::floor(v - r * 256.0), b = std::floor((v - r * 256.0 - g) * 256.0);
            raw.push_back(static_cast<std::uint8_t>(r)); raw.push_back(static_cast<std::uint8_t>(g)); raw.push_back(static_cast<std::uint8_t>(b));
        }
    }
    // zlib stream with stored (uncompressed) deflate blocks.
    std::vector<std::uint8_t> z{0x78, 0x01};
    std::size_t pos = 0;
    while (pos < raw.size()) {
        const std::size_t n = std::min<std::size_t>(65535, raw.size() - pos);
        z.push_back(pos + n == raw.size() ? 1 : 0);
        z.push_back(static_cast<std::uint8_t>(n & 0xFF)); z.push_back(static_cast<std::uint8_t>(n >> 8));
        z.push_back(static_cast<std::uint8_t>(~n & 0xFF)); z.push_back(static_cast<std::uint8_t>((~n >> 8) & 0xFF));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos), raw.begin() + static_cast<std::ptrdiff_t>(pos + n));
        pos += n;
    }
    std::uint32_t a = 1, b = 0;
    for (std::uint8_t c : raw) { a = (a + c) % 65521; b = (b + a) % 65521; }
    put32(z, (b << 16) | a);

    std::vector<std::uint8_t> png{0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::vector<std::uint8_t> ihdr;
    put32(ihdr, size); put32(ihdr, size);
    ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0}); // 8-bit RGB
    chunk(png, "IHDR", ihdr);
    chunk(png, "IDAT", z);
    chunk(png, "IEND", {});
    return png;
}

} // namespace

TEST_CASE("Terrarium PNG decodes to metres with the sea floor clamped", "[terrain]") {
    const auto png = terrariumPng(4, [](std::uint32_t x, std::uint32_t y) { return x == 0 && y == 0 ? -120.0 : 100.0 * x + 1000.0 * y + 0.5; });
    const auto tile = io::decodeTerrariumPng(png);
    REQUIRE(tile.valid());
    REQUIRE(tile.size == 4);
    REQUIRE(tile.heights[0] == 0.0f); // bathymetry -> sea level
    REQUIRE_THAT(tile.heights[1], Catch::Matchers::WithinAbs(100.5, 0.01));
    REQUIRE_THAT(tile.heights[4 * 2 + 3], Catch::Matchers::WithinAbs(2300.5, 0.01)); // row 2, column 3
    REQUIRE_FALSE(io::decodeTerrariumPng({1, 2, 3}).valid());
}

TEST_CASE("Mercator tile addressing", "[terrain]") {
    double tx, ty;
    io::mercatorTile(0.0, 0.0, 1, tx, ty);
    REQUIRE_THAT(tx, Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(ty, Catch::Matchers::WithinAbs(1.0, 1e-9));
    io::mercatorTile(units::degreesToRadians(37.72), units::degreesToRadians(-119.55), 12, tx, ty);
    REQUIRE(static_cast<int>(tx) == 687);  // the Yosemite tile the viewer requests
    REQUIRE(static_cast<int>(ty) == 1583);
}

TEST_CASE("TerrainTiles samples, caches and prefetches synthetic tiles", "[terrain]") {
    // Every tile is a ramp: height = 10 * x + 1000 * (tile x parity)
    std::atomic<int> fetches{0};
    io::TerrainTiles::Options o;
    o.zoom = 10;
    o.cacheTiles = 8;
    io::TerrainTiles terrain(o, [&](unsigned z, unsigned x, unsigned, std::vector<std::uint8_t>& png) {
        REQUIRE(z == 10);
        ++fetches;
        png = terrariumPng(16, [x](std::uint32_t px, std::uint32_t) { return 10.0 * px + 1000.0 * (x % 2); });
        return true;
    });
    const double lat = units::degreesToRadians(37.0), lon = units::degreesToRadians(-120.0);
    double tx, ty;
    io::mercatorTile(lat, lon, 10, tx, ty);
    const double expected = 10.0 * (tx - std::floor(tx)) * 15.0 + 1000.0 * (static_cast<unsigned>(tx) % 2);

    const double h = terrain.heightAboveEllipsoidM(lat, lon);
    REQUIRE_THAT(h, Catch::Matchers::WithinAbs(expected, 1e-6));
    REQUIRE(fetches == 1);
    REQUIRE(terrain.cachedTiles() == 1);
    terrain.heightAboveEllipsoidM(lat, lon);
    REQUIRE(fetches == 1); // cached

    // Non-blocking query: unknown tile -> nullopt, then loaded in the background.
    const double lat2 = units::degreesToRadians(45.0), lon2 = units::degreesToRadians(10.0);
    REQUIRE_FALSE(terrain.cachedHeightAboveEllipsoidM(lat2, lon2).has_value());
    for (int i = 0; i < 200 && !terrain.cachedHeightAboveEllipsoidM(lat2, lon2); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    REQUIRE(terrain.cachedHeightAboveEllipsoidM(lat2, lon2).has_value());
    REQUIRE(terrain.cachedTiles() >= 2);

    // Prefetch a radius: several tiles, all cached, LRU bounded.
    terrain.prefetch(lat, lon, 60000.0, 4);
    REQUIRE(fetches >= 9);
    REQUIRE(terrain.cachedTiles() <= 8);
    REQUIRE(terrain.failures() == 0);
}

TEST_CASE("TerrainTiles reports failures as sea level", "[terrain]") {
    io::TerrainTiles::Options o;
    o.zoom = 5;
    io::TerrainTiles terrain(o, [](unsigned, unsigned, unsigned, std::vector<std::uint8_t>&) { return false; });
    REQUIRE(terrain.heightAboveEllipsoidM(0.5, 0.5) == 0.0);
    REQUIRE(terrain.failures() == 1);
}
