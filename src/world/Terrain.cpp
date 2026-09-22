#include "core/HeightGrid.h"
#include "world/Terrain.h"

#include "core/Log.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace fsim::world {

namespace {

float terrarium(const vsg::ubvec4& p) noexcept {
    return std::max(0.0f, static_cast<float>(p.r) * 256.0f + static_cast<float>(p.g) + static_cast<float>(p.b) / 256.0f - 32768.0f);
}
float terrarium(const vsg::ubvec3& p) noexcept {
    return std::max(0.0f, static_cast<float>(p.r) * 256.0f + static_cast<float>(p.g) + static_cast<float>(p.b) / 256.0f - 32768.0f);
}

// Resample to the mesh's vertex grid: output texel k holds the height at tile
// fraction k / (n - 1), which is where vsg::tile places vertex k (u = c / (cols - 1)).
// Sampling the source bilinearly at those positions makes both tiles of a
// shared edge (and a tile and its upsampled children) agree on the edge
// heights, so LOD transitions and tile seams do not open cracks on cliffs.
// (Nearest-pixel decimation put vertex k at pixel 4k: the edge vertices of
// neighbouring tiles then stood 3 pixels apart, 15 m at level 15 - tall fins
// on a 60-degree wall.)
template <typename Array, typename Fn>
vsg::ref_ptr<vsg::floatArray2D> convert(const Array& srcArray, std::uint32_t maxDim, Fn fn) {
    const std::uint32_t w = srcArray.width(), h = srcArray.height();
    const std::uint32_t ow = (maxDim > 0 && w > maxDim) ? maxDim : w, oh = (maxDim > 0 && h > maxDim) ? maxDim : h;

    std::vector<float> decoded(static_cast<std::size_t>(w) * h);
    for (std::uint32_t y = 0; y < h; ++y)
        for (std::uint32_t x = 0; x < w; ++x) decoded[static_cast<std::size_t>(y) * w + x] = fn(srcArray.at(x, y));

    // The filtering and resampling live in core so that io::TerrainTiles - which
    // answers the camera's "how high is the ground here?" - can build the same
    // surface from the same tiles. Two implementations of this drifted apart
    // once already and the camera flew into a hill it could see.
    std::vector<float> resampled;
    const std::vector<float>* grid = &decoded;
    if (w == h && ow < w) {
        resampled = core::resampleHeightGrid(decoded.data(), w, ow);
        grid = &resampled;
    }

    auto out = vsg::floatArray2D::create(ow, oh, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
    out->properties.origin = srcArray.properties.origin;
    for (std::uint32_t y = 0; y < oh; ++y)
        for (std::uint32_t x = 0; x < ow; ++x) out->set(x, y, (*grid)[static_cast<std::size_t>(y) * ow + x]);
    return out;
}

} // namespace

vsg::ref_ptr<vsg::floatArray2D> decodeElevation(vsg::ref_ptr<vsg::Data> image, ElevationEncoding encoding,
                                                std::uint32_t maxDimension) {
    if (!image) return {};
    // Already heights (a synthesised tile from ElevationUpsampler): pass through, downsampled if asked.
    if (auto f = image.cast<vsg::floatArray2D>()) {
        if (maxDimension == 0 || f->width() <= maxDimension) return f;
        return convert(*f, maxDimension, [](float v) { return v; });
    }
    switch (encoding) {
    case ElevationEncoding::Float:
        if (auto f = image.cast<vsg::floatArray2D>()) {
            if (maxDimension == 0 || f->width() <= maxDimension) return f;
            return convert(*f, maxDimension, [](float v) { return v; });
        }
        if (auto s = image.cast<vsg::ushortArray2D>())
            return convert(*s, maxDimension, [](std::uint16_t v) { return static_cast<float>(v); });
        break;
    case ElevationEncoding::Terrarium:
        if (auto rgba = image.cast<vsg::ubvec4Array2D>()) return convert(*rgba, maxDimension, [](const vsg::ubvec4& p) { return terrarium(p); });
        if (auto rgb = image.cast<vsg::ubvec3Array2D>()) return convert(*rgb, maxDimension, [](const vsg::ubvec3& p) { return terrarium(p); });
        break;
    }
    LOG_WARN("world") << "elevation tile has an unexpected pixel format (" << image->className() << ")";
    return {};
}

} // namespace fsim::world
