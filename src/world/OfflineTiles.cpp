#include "world/OfflineTiles.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <vector>

namespace fsim::world {

namespace {

std::string forwardSlashes(std::string s) {
    std::replace(s.begin(), s.end(), '\\', '/');
    return s;
}

// A template split around its placeholders: literal pieces and the order of {z}/{x}/{y}.
struct Pieces {
    std::vector<std::string> literals; // literals.size() == keys.size() + 1
    std::vector<char> keys;
};

Pieces split(const std::string& tmpl) {
    Pieces p;
    std::string current;
    for (std::size_t i = 0; i < tmpl.size(); ++i) {
        if (tmpl[i] == '{' && i + 2 < tmpl.size() && tmpl[i + 2] == '}' &&
            (tmpl[i + 1] == 'z' || tmpl[i + 1] == 'x' || tmpl[i + 1] == 'y')) {
            p.literals.push_back(current);
            current.clear();
            p.keys.push_back(tmpl[i + 1]);
            i += 2;
        } else {
            current += tmpl[i];
        }
    }
    p.literals.push_back(current);
    return p;
}

bool parse(const std::string& tmpl, const std::string& u, unsigned& z, unsigned& x, unsigned& y) {
    const Pieces p = split(tmpl);
    if (p.keys.size() != 3) return false;
    std::size_t pos = 0;
    unsigned values[3] = {0, 0, 0};
    for (std::size_t i = 0; i < p.keys.size(); ++i) {
        const std::string& lit = p.literals[i];
        if (u.compare(pos, lit.size(), lit) != 0) return false;
        pos += lit.size();
        std::size_t end = pos;
        while (end < u.size() && u[end] >= '0' && u[end] <= '9') ++end;
        if (end == pos) return false;
        values[i] = static_cast<unsigned>(std::strtoul(u.substr(pos, end - pos).c_str(), nullptr, 10));
        pos = end;
    }
    if (u.compare(pos, std::string::npos, p.literals.back()) != 0) return false;
    for (std::size_t i = 0; i < 3; ++i) {
        if (p.keys[i] == 'z') z = values[i];
        else if (p.keys[i] == 'x') x = values[i];
        else y = values[i];
    }
    return true;
}

std::string expand(std::string t, unsigned z, unsigned x, unsigned y) {
    auto replace = [&](const char* key, unsigned v) {
        for (auto pos = t.find(key); pos != std::string::npos; pos = t.find(key)) t.replace(pos, 3, std::to_string(v));
    };
    replace("{z}", z);
    replace("{x}", x);
    replace("{y}", y);
    return t;
}

// Bilinear blends, one per texel type. Colour is blended in float and rounded
// once, so a resampled tile does not drift darker through repeated rounding.
float blend(float a, float b, float c, float d, float fx, float fy) {
    return (a * (1.0f - fx) + b * fx) * (1.0f - fy) + (c * (1.0f - fx) + d * fx) * fy;
}
template <typename V, std::size_t N>
V blendChannels(const V& a, const V& b, const V& c, const V& d, float fx, float fy) {
    V out;
    for (std::size_t k = 0; k < N; ++k) {
        const float v = blend(a[k], b[k], c[k], d[k], fx, fy);
        out[k] = static_cast<std::uint8_t>(std::clamp(std::lround(v), 0L, 255L));
    }
    return out;
}
vsg::ubvec4 blend(const vsg::ubvec4& a, const vsg::ubvec4& b, const vsg::ubvec4& c, const vsg::ubvec4& d, float fx, float fy) {
    return blendChannels<vsg::ubvec4, 4>(a, b, c, d, fx, fy);
}
vsg::ubvec3 blend(const vsg::ubvec3& a, const vsg::ubvec3& b, const vsg::ubvec3& c, const vsg::ubvec3& d, float fx, float fy) {
    return blendChannels<vsg::ubvec3, 3>(a, b, c, d, fx, fy);
}

/// The share of `src` that a descendant `d` levels down, at quadrant (qx, qy)
/// counted from the top-left, occupies - resampled back to src's size, so the
/// result looks to everything downstream exactly like a real tile would.
/// Texels are sampled at their centres, the same convention the tiles
/// themselves use, so a synthesised tile lines up with its real neighbours.
template <typename T>
vsg::ref_ptr<vsg::Array2D<T>> shareOf(const vsg::Array2D<T>& src, unsigned d, unsigned qx, unsigned qy) {
    const std::uint32_t W = src.width(), H = src.height();
    if (W < 2 || H < 2) return {};
    const double scale = 1.0 / static_cast<double>(1u << d);
    const double ox = static_cast<double>(qx) * scale, oy = static_cast<double>(qy) * scale;
    auto out = vsg::Array2D<T>::create(W, H, src.properties);
    auto axis = [](double s, std::uint32_t size, std::uint32_t& a, std::uint32_t& b, float& t) {
        const double hi = static_cast<double>(size - 1);
        a = static_cast<std::uint32_t>(std::min(std::floor(std::clamp(s, 0.0, hi)), hi - 1.0));
        b = a + 1;
        t = static_cast<float>(std::clamp(s - static_cast<double>(a), 0.0, 1.0));
    };
    for (std::uint32_t j = 0; j < H; ++j) {
        const double sy = (oy + (static_cast<double>(j) + 0.5) / H * scale) * H - 0.5;
        std::uint32_t y0, y1;
        float fy;
        axis(sy, H, y0, y1, fy);
        for (std::uint32_t i = 0; i < W; ++i) {
            const double sx = (ox + (static_cast<double>(i) + 0.5) / W * scale) * W - 0.5;
            std::uint32_t x0, x1;
            float fx;
            axis(sx, W, x0, x1, fx);
            out->set(i, j, blend(src.at(x0, y0), src.at(x1, y0), src.at(x0, y1), src.at(x1, y1), fx, fy));
        }
    }
    return out;
}

} // namespace

OfflineTiles::OfflineTiles(std::string imageryTemplate, std::string elevationTemplate, ElevationEncoding encoding)
    : imagery_(forwardSlashes(std::move(imageryTemplate))),
      elevation_(forwardSlashes(std::move(elevationTemplate))),
      encoding_(encoding) {}

bool OfflineTiles::classify(const std::string& p, Layer& layer, unsigned& z, unsigned& x, unsigned& y) const {
    const std::string u = forwardSlashes(p);
    if (!imagery_.empty() && parse(imagery_, u, z, x, y)) {
        layer = Layer::Imagery;
        return true;
    }
    if (!elevation_.empty() && parse(elevation_, u, z, x, y)) {
        layer = Layer::Elevation;
        return true;
    }
    return false;
}

std::string OfflineTiles::path(Layer layer, unsigned z, unsigned x, unsigned y) const {
    return expand(layer == Layer::Imagery ? imagery_ : elevation_, z, x, y);
}

bool OfflineTiles::exists(Layer layer, unsigned z, unsigned x, unsigned y) const {
    if ((layer == Layer::Imagery ? imagery_ : elevation_).empty()) return false;
    std::error_code ec;
    return std::filesystem::is_regular_file(path(layer, z, x, y), ec);
}

vsg::ref_ptr<vsg::Object> OfflineTiles::read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options) const {
    Layer layer;
    unsigned z = 0, x = 0, y = 0;
    if (!classify(filename.string(), layer, z, x, y)) return {}; // not a map tile
    if (exists(layer, z, x, y)) return {};                      // real: the image readers take it
    if (z == 0) return {};

    // Only where something real is: VSG reads the four children of a tile
    // together, so the question is whether any of them has anything at this
    // level. If none does, decline, the read fails, and the parent stays.
    const unsigned bx = x & ~1u, by = y & ~1u;
    bool anyReal = false;
    for (unsigned dy = 0; dy < 2 && !anyReal; ++dy)
        for (unsigned dx = 0; dx < 2 && !anyReal; ++dx)
            anyReal = exists(Layer::Imagery, z, bx + dx, by + dy) || exists(Layer::Elevation, z, bx + dx, by + dy);
    if (!anyReal) return {};

    // The nearest real ancestor in this layer, and this tile's share of it.
    for (unsigned d = 1; d <= z; ++d) {
        const unsigned az = z - d, ax = x >> d, ay = y >> d;
        if (!exists(layer, az, ax, ay)) continue;
        auto source = vsg::read_cast<vsg::Data>(path(layer, az, ax, ay), options);
        if (!source) return {};
        const unsigned mask = (1u << d) - 1u, qx = x & mask, qy = y & mask;
        if (layer == Layer::Elevation) {
            // Handed back as heights, which decodeElevation passes through:
            // the elevation callback then treats it like any real tile.
            auto heights = decodeElevation(source, encoding_, 0);
            if (!heights) return {};
            return shareOf(*heights, d, qx, qy);
        }
        if (auto rgba = source.cast<vsg::ubvec4Array2D>()) return shareOf(*rgba, d, qx, qy);
        if (auto rgb = source.cast<vsg::ubvec3Array2D>()) return shareOf(*rgb, d, qx, qy);
        return {};
    }
    return {};
}

} // namespace fsim::world
