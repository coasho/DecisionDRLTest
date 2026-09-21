#include "world/ElevationUpsampler.h"

#include "core/Log.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace fsim::world {

namespace {

// The template split around its placeholders: literal pieces and the order of {z}/{x}/{y}.
struct Pieces {
    std::vector<std::string> literals; // literals.size() == keys.size() + 1
    std::vector<char> keys;
};

Pieces split(const std::string& tmpl) {
    Pieces p;
    std::string current;
    for (std::size_t i = 0; i < tmpl.size(); ++i) {
        if (tmpl[i] == '{' && i + 2 < tmpl.size() && tmpl[i + 2] == '}' && (tmpl[i + 1] == 'z' || tmpl[i + 1] == 'x' || tmpl[i + 1] == 'y')) {
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

} // namespace

ElevationUpsampler::ElevationUpsampler(std::string urlTemplate, unsigned maxSourceLevel, ElevationEncoding encoding)
    : template_(std::move(urlTemplate)), maxSourceLevel_(maxSourceLevel), encoding_(encoding) {}

std::string ElevationUpsampler::url(unsigned z, unsigned x, unsigned y) const {
    std::string u = template_;
    auto replace = [&](const char* key, unsigned v) {
        for (auto pos = u.find(key); pos != std::string::npos; pos = u.find(key)) u.replace(pos, 3, std::to_string(v));
    };
    replace("{z}", z);
    replace("{x}", x);
    replace("{y}", y);
    return u;
}

bool ElevationUpsampler::parse(const std::string& u, unsigned& z, unsigned& x, unsigned& y) const {
    const Pieces p = split(template_);
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
    const std::string& tail = p.literals.back();
    if (u.compare(pos, std::string::npos, tail) != 0) return false;
    for (std::size_t i = 0; i < 3; ++i) {
        if (p.keys[i] == 'z') z = values[i];
        else if (p.keys[i] == 'x') x = values[i];
        else y = values[i];
    }
    return true;
}

vsg::ref_ptr<vsg::Object> ElevationUpsampler::read(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options) const {
    unsigned z = 0, x = 0, y = 0;
    if (!parse(filename.string(), z, x, y) || z <= maxSourceLevel_) return {}; // not ours: the real readers take it

    // The deepest real ancestor and the quadrant path down to the requested tile.
    const unsigned d = z - maxSourceLevel_;
    const unsigned ax = x >> d, ay = y >> d;
    auto source = vsg::read_cast<vsg::Data>(url(maxSourceLevel_, ax, ay), options);
    if (!source) return {};
    auto heights = decodeElevation(source, encoding_, 0);
    if (!heights || heights->width() < 2 || heights->height() < 2) return {};

    const std::uint32_t W = heights->width(), H = heights->height();
    const double scale = 1.0 / static_cast<double>(1u << d); // this tile's share of the ancestor
    const double ox = static_cast<double>(x & ((1u << d) - 1u)) * scale, oy = static_cast<double>(y & ((1u << d) - 1u)) * scale;
    auto out = vsg::floatArray2D::create(W, H, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
    out->properties.origin = heights->properties.origin;
    // Sample positions in the ancestor's pixel grid (texel centres), bilinear;
    // half a pixel beyond the outermost centres the value is extrapolated, as
    // Terrain.cpp does for the vertex grid, so tile seams agree.
    auto axis = [](double s, std::uint32_t size, std::uint32_t& a, std::uint32_t& b, float& t) {
        const double hi = static_cast<double>(size - 1);
        a = static_cast<std::uint32_t>(std::min(std::floor(std::clamp(s, 0.0, hi)), hi - 1.0));
        b = a + 1;
        t = static_cast<float>(s - static_cast<double>(a));
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
            const float top = heights->at(x0, y0) * (1.0f - fx) + heights->at(x1, y0) * fx;
            const float bottom = heights->at(x0, y1) * (1.0f - fx) + heights->at(x1, y1) * fx;
            out->set(i, j, top * (1.0f - fy) + bottom * fy);
        }
    }
    return out;
}

} // namespace fsim::world
