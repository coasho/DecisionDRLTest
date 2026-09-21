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

template <typename Array, typename Fn>
vsg::ref_ptr<vsg::floatArray2D> convert(const Array& src, std::uint32_t maxDim, Fn fn) {
    const std::uint32_t w = src.width(), h = src.height();
    const std::uint32_t step = (maxDim > 0 && w > maxDim) ? (w + maxDim - 1) / maxDim : 1;
    const std::uint32_t ow = (w + step - 1) / step, oh = (h + step - 1) / step;
    auto out = vsg::floatArray2D::create(ow, oh, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
    out->properties.origin = src.properties.origin;
    for (std::uint32_t y = 0; y < oh; ++y)
        for (std::uint32_t x = 0; x < ow; ++x)
            out->set(x, y, fn(src.at(std::min(x * step, w - 1), std::min(y * step, h - 1))));
    return out;
}

} // namespace

vsg::ref_ptr<vsg::floatArray2D> decodeElevation(vsg::ref_ptr<vsg::Data> image, ElevationEncoding encoding,
                                                std::uint32_t maxDimension) {
    if (!image) return {};
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
