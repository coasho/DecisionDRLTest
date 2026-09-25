#include "polygonize.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <thread>
#include <unordered_map>
#include <utility>

namespace meshkit {
namespace {

constexpr int kBrick = 32; // cells along a brick's side

// cube corners: bit 0 x, bit 1 y, bit 2 z; Kuhn's six tetrahedra along 0-7
constexpr int kTets[6][4] = {{0, 1, 3, 7}, {0, 1, 5, 7}, {0, 2, 3, 7}, {0, 2, 6, 7}, {0, 4, 5, 7}, {0, 4, 6, 7}};

struct EdgeKey {
    std::uint64_t a, b;
    bool operator==(const EdgeKey& o) const { return a == o.a && b == o.b; }
};

struct EdgeHash {
    std::size_t operator()(const EdgeKey& k) const {
        return static_cast<std::size_t>(k.a * 0x9E3779B97F4A7C15ULL ^ (k.b + 0x632BE59BD9B4E019ULL + (k.a << 6)));
    }
};

struct BrickOut {
    std::vector<V3> v;
    std::vector<EdgeKey> key;
    std::vector<std::array<std::uint32_t, 3>> f;
};

struct Grid {
    V3 lo;
    double h = 0.01;
    long long nx = 0, ny = 0, nz = 0; // cells

    std::uint64_t corner(long long i, long long j, long long k) const {
        return static_cast<std::uint64_t>(i) +
               static_cast<std::uint64_t>(nx + 1) *
                   (static_cast<std::uint64_t>(j) + static_cast<std::uint64_t>(ny + 1) * static_cast<std::uint64_t>(k));
    }
    V3 at(long long i, long long j, long long k) const {
        return lo + V3{static_cast<double>(i) * h, static_cast<double>(j) * h, static_cast<double>(k) * h};
    }
};

void brick(const Node& root, const Grid& g, long long bi, long long bj, long long bk, double safety, BrickOut& out) {
    const long long i0 = bi * kBrick, j0 = bj * kBrick, k0 = bk * kBrick;
    // refine: keep the leaf cells the surface may pass through
    struct Cell {
        long long i, j, k, s;
    };
    std::vector<Cell> stack{{i0, j0, k0, kBrick}};
    std::vector<Cell> leaves;
    while (!stack.empty()) {
        const Cell c = stack.back();
        stack.pop_back();
        if (c.i >= g.nx || c.j >= g.ny || c.k >= g.nz) continue;
        const double half = 0.5 * static_cast<double>(c.s);
        const V3 centre = g.lo + V3{(static_cast<double>(c.i) + half) * g.h, (static_cast<double>(c.j) + half) * g.h,
                                    (static_cast<double>(c.k) + half) * g.h};
        const double reach = 0.8660254 * static_cast<double>(c.s) * g.h * safety;
        if (std::fabs(root.eval(centre).d) > reach) continue;
        if (c.s == 1) {
            leaves.push_back(c);
            continue;
        }
        const long long s = c.s / 2;
        for (int o = 0; o < 8; ++o)
            stack.push_back({c.i + ((o & 1) ? s : 0), c.j + ((o & 2) ? s : 0), c.k + ((o & 4) ? s : 0), s});
    }
    if (leaves.empty()) return;
    // corner values, evaluated once each
    const long long w = kBrick + 1;
    std::vector<double> value(static_cast<std::size_t>(w * w * w), std::numeric_limits<double>::quiet_NaN());
    auto sample = [&](long long i, long long j, long long k) -> double {
        const auto idx = static_cast<std::size_t>((i - i0) + w * ((j - j0) + w * (k - k0)));
        double& v = value[idx];
        if (std::isnan(v)) {
            v = root.eval(g.at(i, j, k)).d;
            if (std::fabs(v) < 1e-12) v = 1e-12; // never exactly on a corner
        }
        return v;
    };
    std::unordered_map<EdgeKey, std::uint32_t, EdgeHash> made;
    auto vertex = [&](const long long (&ci)[8][3], const double (&cv)[8], int a, int b) -> std::uint32_t {
        std::uint64_t ka = g.corner(ci[a][0], ci[a][1], ci[a][2]), kb = g.corner(ci[b][0], ci[b][1], ci[b][2]);
        int lo = a, hi = b;
        if (ka > kb) {
            std::swap(ka, kb);
            std::swap(lo, hi);
        }
        const EdgeKey key{ka, kb};
        const auto it = made.find(key);
        if (it != made.end()) return it->second;
        const V3 pa = g.at(ci[lo][0], ci[lo][1], ci[lo][2]), pb = g.at(ci[hi][0], ci[hi][1], ci[hi][2]);
        const double t = cv[lo] / (cv[lo] - cv[hi]);
        const auto idx = static_cast<std::uint32_t>(out.v.size());
        out.v.push_back(lerp(pa, pb, t));
        out.key.push_back(key);
        made.emplace(key, idx);
        return idx;
    };
    for (const Cell& c : leaves) {
        long long ci[8][3];
        double cv[8];
        int inside = 0;
        for (int o = 0; o < 8; ++o) {
            ci[o][0] = c.i + (o & 1);
            ci[o][1] = c.j + ((o >> 1) & 1);
            ci[o][2] = c.k + ((o >> 2) & 1);
            cv[o] = sample(ci[o][0], ci[o][1], ci[o][2]);
            inside += cv[o] < 0.0 ? 1 : 0;
        }
        if (inside == 0 || inside == 8) continue;
        for (const auto& tet : kTets) {
            int in[4], outs[4], ni = 0, no = 0;
            for (int q = 0; q < 4; ++q) {
                if (cv[tet[q]] < 0.0) in[ni++] = tet[q];
                else outs[no++] = tet[q];
            }
            if (ni == 0 || ni == 4) continue;
            // outward: from the inside corners towards the outside ones
            V3 cin{}, cout{};
            for (int q = 0; q < ni; ++q) cin = cin + g.at(ci[in[q]][0], ci[in[q]][1], ci[in[q]][2]);
            for (int q = 0; q < no; ++q) cout = cout + g.at(ci[outs[q]][0], ci[outs[q]][1], ci[outs[q]][2]);
            const V3 dir = cout * (1.0 / no) - cin * (1.0 / ni);
            auto emit = [&](std::uint32_t a, std::uint32_t b, std::uint32_t d) {
                const V3 n = cross(out.v[b] - out.v[a], out.v[d] - out.v[a]);
                if (dot(n, dir) < 0.0) std::swap(b, d);
                out.f.push_back({a, b, d});
            };
            if (ni == 1 || no == 1) {
                const int apex = ni == 1 ? in[0] : outs[0];
                const int* rest = ni == 1 ? outs : in;
                emit(vertex(ci, cv, apex, rest[0]), vertex(ci, cv, apex, rest[1]), vertex(ci, cv, apex, rest[2]));
            } else {
                // two in (a, b), two out (c, d): the quad ac, ad, bd, bc
                const std::uint32_t ac = vertex(ci, cv, in[0], outs[0]), ad = vertex(ci, cv, in[0], outs[1]);
                const std::uint32_t bd = vertex(ci, cv, in[1], outs[1]), bc = vertex(ci, cv, in[1], outs[0]);
                emit(ac, ad, bd);
                emit(ac, bd, bc);
            }
        }
    }
}

} // namespace

Mesh polygonize(const Node& root, const Box& domain, const PolygonizeOptions& options) {
    const double h = options.cell;
    const Box d = domain.padded(2.0 * h);
    Grid g;
    // off the round numbers designs are made of, so that no face of a box or
    // a cylinder lies on a grid plane (its corners exactly on the surface)
    g.lo = d.lo - V3{0.2718, 0.3141, 0.1618} * h;
    g.h = h;
    g.nx = static_cast<long long>(std::ceil((d.hi.x - d.lo.x) / h));
    g.ny = static_cast<long long>(std::ceil((d.hi.y - d.lo.y) / h));
    g.nz = static_cast<long long>(std::ceil((d.hi.z - d.lo.z) / h));
    const long long bx = (g.nx + kBrick - 1) / kBrick, by = (g.ny + kBrick - 1) / kBrick, bz = (g.nz + kBrick - 1) / kBrick;
    const long long total = bx * by * bz;
    unsigned threads = options.threads ? options.threads : std::max(1u, std::thread::hardware_concurrency());
    threads = std::min(threads, 32u);
    // each brick fills a slot of its own, and the slots are joined in brick
    // order: the mesh is the same whichever thread made which brick
    std::vector<BrickOut> outs(static_cast<std::size_t>(total));
    std::atomic<long long> next{0};
    auto work = [&]() {
        for (;;) {
            const long long b = next.fetch_add(1);
            if (b >= total) break;
            brick(root, g, b % bx, (b / bx) % by, b / (bx * by), options.safety, outs[static_cast<std::size_t>(b)]);
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 1; t < threads; ++t) pool.emplace_back(work);
    work();
    for (auto& t : pool) t.join();

    // weld: a vertex on a brick's face is made by both bricks, from the same
    // grid edge and the same corner values; the first brick's copy stays
    struct Tag {
        EdgeKey key;
        std::uint32_t index;
    };
    std::vector<Tag> tags;
    std::vector<V3> all;
    std::vector<std::array<std::uint32_t, 3>> faces;
    for (const auto& o : outs) {
        const auto base = static_cast<std::uint32_t>(all.size());
        for (std::size_t i = 0; i < o.v.size(); ++i) {
            tags.push_back({o.key[i], static_cast<std::uint32_t>(base + i)});
            all.push_back(o.v[i]);
        }
        for (const auto& f : o.f) faces.push_back({base + f[0], base + f[1], base + f[2]});
    }
    std::sort(tags.begin(), tags.end(), [](const Tag& a, const Tag& b) {
        if (a.key.a != b.key.a) return a.key.a < b.key.a;
        if (a.key.b != b.key.b) return a.key.b < b.key.b;
        return a.index < b.index;
    });
    std::vector<std::uint32_t> remap(all.size());
    Mesh m;
    for (std::size_t i = 0; i < tags.size(); ++i) {
        if (i == 0 || !(tags[i].key == tags[i - 1].key)) m.v.push_back(all[tags[i].index]);
        remap[tags[i].index] = static_cast<std::uint32_t>(m.v.size() - 1);
    }
    m.f.reserve(faces.size());
    for (const auto& f : faces) {
        const std::array<std::uint32_t, 3> r{remap[f[0]], remap[f[1]], remap[f[2]]};
        if (r[0] != r[1] && r[1] != r[2] && r[0] != r[2]) m.f.push_back(r);
    }
    return m;
}

} // namespace meshkit
