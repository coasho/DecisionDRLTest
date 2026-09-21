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
vsg::ref_ptr<vsg::floatArray2D> convert(const Array& src, std::uint32_t maxDim, Fn fn) {
    const std::uint32_t w = src.width(), h = src.height();
    const std::uint32_t ow = (maxDim > 0 && w > maxDim) ? maxDim : w, oh = (maxDim > 0 && h > maxDim) ? maxDim : h;
    auto out = vsg::floatArray2D::create(ow, oh, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
    out->properties.origin = src.properties.origin;
    // Beyond the outermost pixel centres (the first and last vertex sit half a
    // pixel outside them) extrapolate linearly: the neighbouring tile does the
    // same from its side, so both estimate the edge height from the same
    // slope and the seam stays closed to second order.
    auto sampleAxis = [](std::uint32_t k, std::uint32_t n, std::uint32_t size, std::uint32_t& i0, std::uint32_t& i1, float& t) {
        const double f = n > 1 ? static_cast<double>(k) / static_cast<double>(n - 1) : 0.0; // tile fraction of vertex k
        const double s = f * size - 0.5;                                                    // pixel centres at (j + 0.5) / size
        if (size < 2) { i0 = i1 = 0; t = 0.0f; return; }
        const double lo = 0.0, hi = static_cast<double>(size - 1);
        const double sc = std::clamp(s, lo, hi);
        i0 = static_cast<std::uint32_t>(std::min(std::floor(sc), hi - 1.0));
        i1 = i0 + 1;
        t = static_cast<float>(s - static_cast<double>(i0)); // < 0 or > 1 at the edges: extrapolation
    };
    for (std::uint32_t y = 0; y < oh; ++y) {
        std::uint32_t y0, y1;
        float ty;
        sampleAxis(y, oh, h, y0, y1, ty);
        for (std::uint32_t x = 0; x < ow; ++x) {
            std::uint32_t x0, x1;
            float tx;
            sampleAxis(x, ow, w, x0, x1, tx);
            const float top = fn(src.at(x0, y0)) * (1.0f - tx) + fn(src.at(x1, y0)) * tx;
            const float bottom = fn(src.at(x0, y1)) * (1.0f - tx) + fn(src.at(x1, y1)) * tx;
            out->set(x, y, top * (1.0f - ty) + bottom * ty);
        }
    }
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
