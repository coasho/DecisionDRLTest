#include "core/HeightGrid.h"

#include <algorithm>
#include <cmath>

namespace fsim::core {
namespace {

/// Value at index `i`, continuing the edge slope past either end.
float extended(const std::vector<float>& v, int i, int n, std::size_t base, std::size_t stride) {
    if (i < 0) return v[base] + static_cast<float>(i) * (v[base + stride] - v[base]);
    if (i >= n) {
        const std::size_t last = base + static_cast<std::size_t>(n - 1) * stride;
        return v[last] + static_cast<float>(i - (n - 1)) * (v[last] - v[last - stride]);
    }
    return v[base + static_cast<std::size_t>(i) * stride];
}

/// Where sample k of `n` reads the raster: the two indices and the weight
/// between them, which falls outside [0, 1] at the edges (extrapolation).
void sampleAxis(std::uint32_t k, std::uint32_t n, std::uint32_t size, std::uint32_t& i0, std::uint32_t& i1, float& t) {
    if (size < 2) {
        i0 = i1 = 0;
        t = 0.0f;
        return;
    }
    const double f = n > 1 ? static_cast<double>(k) / static_cast<double>(n - 1) : 0.0;
    const double s = f * size - 0.5; // pixel centres at (j + 0.5) / size
    const double sc = std::clamp(s, 0.0, static_cast<double>(size - 1));
    i0 = static_cast<std::uint32_t>(std::min(std::floor(sc), static_cast<double>(size - 1) - 1.0));
    i1 = i0 + 1;
    t = static_cast<float>(s - static_cast<double>(i0));
}

} // namespace

std::vector<float> resampleHeightGrid(const float* src, std::uint32_t srcSize, std::uint32_t outSize) {
    if (!src || srcSize == 0) return {};
    const std::size_t count = static_cast<std::size_t>(srcSize) * srcSize;
    if (outSize == 0 || outSize >= srcSize) return std::vector<float>(src, src + count);

    // Box-filter to the sample spacing, separably.
    const int radius = static_cast<int>(std::max(1u, (srcSize / outSize) / 2u)); // 2 for 256 -> 64
    const std::vector<float> decoded(src, src + count);
    std::vector<float> rows(count), filtered(count);
    const int w = static_cast<int>(srcSize), h = static_cast<int>(srcSize);
    for (std::uint32_t y = 0; y < srcSize; ++y)
        for (std::uint32_t x = 0; x < srcSize; ++x) {
            float sum = 0.0f;
            for (int d = -radius; d <= radius; ++d)
                sum += extended(decoded, static_cast<int>(x) + d, w, static_cast<std::size_t>(y) * srcSize, 1);
            rows[static_cast<std::size_t>(y) * srcSize + x] = sum / static_cast<float>(2 * radius + 1);
        }
    for (std::uint32_t y = 0; y < srcSize; ++y)
        for (std::uint32_t x = 0; x < srcSize; ++x) {
            float sum = 0.0f;
            for (int d = -radius; d <= radius; ++d)
                sum += extended(rows, static_cast<int>(y) + d, h, x, srcSize);
            filtered[static_cast<std::size_t>(y) * srcSize + x] = sum / static_cast<float>(2 * radius + 1);
        }

    std::vector<float> out(static_cast<std::size_t>(outSize) * outSize);
    for (std::uint32_t y = 0; y < outSize; ++y) {
        std::uint32_t y0, y1;
        float ty;
        sampleAxis(y, outSize, srcSize, y0, y1, ty);
        for (std::uint32_t x = 0; x < outSize; ++x) {
            std::uint32_t x0, x1;
            float tx;
            sampleAxis(x, outSize, srcSize, x0, x1, tx);
            const auto at = [&](std::uint32_t px, std::uint32_t py) {
                return filtered[static_cast<std::size_t>(py) * srcSize + px];
            };
            const float top = at(x0, y0) * (1.0f - tx) + at(x1, y0) * tx;
            const float bottom = at(x0, y1) * (1.0f - tx) + at(x1, y1) * tx;
            out[static_cast<std::size_t>(y) * outSize + x] = top * (1.0f - ty) + bottom * ty;
        }
    }
    return out;
}

} // namespace fsim::core
