#include "decimate.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iterator>
#include <queue>
#include <thread>
#include <unordered_map>

namespace meshkit {
namespace {

using Face = std::array<std::uint32_t, 3>;

/// A symmetric 4x4 quadric: a00 a01 a02 a03 a11 a12 a13 a22 a23 a33.
struct Quadric {
    double a[10] = {};

    void plane(V3 n, double d, double w) {
        a[0] += w * n.x * n.x;
        a[1] += w * n.x * n.y;
        a[2] += w * n.x * n.z;
        a[3] += w * n.x * d;
        a[4] += w * n.y * n.y;
        a[5] += w * n.y * n.z;
        a[6] += w * n.y * d;
        a[7] += w * n.z * n.z;
        a[8] += w * n.z * d;
        a[9] += w * d * d;
    }
    Quadric& operator+=(const Quadric& q) {
        for (int i = 0; i < 10; ++i) a[i] += q.a[i];
        return *this;
    }
    double eval(V3 v) const {
        return a[0] * v.x * v.x + 2.0 * a[1] * v.x * v.y + 2.0 * a[2] * v.x * v.z + 2.0 * a[3] * v.x + a[4] * v.y * v.y +
               2.0 * a[5] * v.y * v.z + 2.0 * a[6] * v.y + a[7] * v.z * v.z + 2.0 * a[8] * v.z + a[9];
    }
    /// The point of least error, when the quadric pins one down.
    bool optimum(V3& out) const {
        const double m00 = a[0], m01 = a[1], m02 = a[2], m11 = a[4], m12 = a[5], m22 = a[7];
        const double c0 = m11 * m22 - m12 * m12, c1 = m02 * m12 - m01 * m22, c2 = m01 * m12 - m02 * m11;
        const double det = m00 * c0 + m01 * c1 + m02 * c2;
        const double scale = (m00 + m11 + m22) / 3.0;
        if (!(std::fabs(det) > 1e-9 * scale * scale * scale) || scale <= 0.0) return false;
        const double b0 = -a[3], b1 = -a[6], b2 = -a[8];
        const double i01 = m02 * m12 - m01 * m22, i02 = m01 * m12 - m02 * m11;
        const double i11 = m00 * m22 - m02 * m02, i12 = m01 * m02 - m00 * m12, i22 = m00 * m11 - m01 * m01;
        out = V3{c0 * b0 + i01 * b1 + i02 * b2, i01 * b0 + i11 * b1 + i12 * b2, i02 * b0 + i12 * b1 + i22 * b2} * (1.0 / det);
        return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z);
    }
};

struct Entry {
    double cost;
    std::uint32_t a, b;
    // equal costs go by their vertices, so the collapses come in the same
    // order however the queue was filled
    bool operator>(const Entry& o) const {
        if (cost != o.cost) return cost > o.cost;
        return a != o.a ? a > o.a : b > o.b;
    }
};

/// Quadric simplification of m in place. A locked vertex and its edges stay
/// as they are.
void simplify(Mesh& m, std::vector<std::uint16_t>& material, const std::vector<char>& locked, double maxError,
              std::size_t targetFaces, double sharpDeg, double featureWeight,
              std::vector<std::uint32_t>* origin = nullptr) {
    const std::size_t nv = m.v.size(), nf = m.f.size();
    if (nf < 8) { // too small to simplify: every vertex stays where it was
        if (origin != nullptr) {
            origin->resize(nv);
            for (std::size_t i = 0; i < nv; ++i) (*origin)[i] = static_cast<std::uint32_t>(i);
        }
        return;
    }
    std::vector<Quadric> Q(nv);
    std::vector<double> weight(nv, 0.0);
    std::vector<V3> fn(nf);
    for (std::size_t f = 0; f < nf; ++f) {
        const V3 p0 = m.v[m.f[f][0]], p1 = m.v[m.f[f][1]], p2 = m.v[m.f[f][2]];
        const V3 n = cross(p1 - p0, p2 - p0);
        const double a2 = length(n);
        if (a2 <= 0.0) continue;
        fn[f] = n * (1.0 / a2);
        const double d = -dot(fn[f], p0);
        for (auto v : m.f[f]) {
            Q[v].plane(fn[f], d, 0.5 * a2);
            weight[v] += 0.5 * a2;
        }
    }
    std::vector<std::vector<std::uint32_t>> vf(nv);
    for (std::size_t f = 0; f < nf; ++f)
        for (auto v : m.f[f]) vf[v].push_back(static_cast<std::uint32_t>(f));

    // features: planes across edges between materials and across sharp edges
    struct HalfEdge {
        std::uint32_t lo, hi, face;
    };
    std::vector<HalfEdge> he;
    he.reserve(3 * nf);
    for (std::size_t f = 0; f < nf; ++f)
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint32_t a = m.f[f][e], b = m.f[f][(e + 1) % 3];
            he.push_back({std::min(a, b), std::max(a, b), static_cast<std::uint32_t>(f)});
        }
    // by edge, then by face: an edge's two feature planes are added in one order
    std::sort(he.begin(), he.end(), [](const HalfEdge& x, const HalfEdge& y) {
        if (x.lo != y.lo) return x.lo < y.lo;
        if (x.hi != y.hi) return x.hi < y.hi;
        return x.face < y.face;
    });
    const double sharpCos = std::cos(sharpDeg * 3.14159265358979323846 / 180.0);
    for (std::size_t i = 0; i < he.size();) {
        std::size_t j = i + 1;
        while (j < he.size() && he[j].lo == he[i].lo && he[j].hi == he[i].hi) ++j;
        if (j - i == 2) {
            const std::uint32_t f1 = he[i].face, f2 = he[i + 1].face;
            if (material[f1] != material[f2] || dot(fn[f1], fn[f2]) < sharpCos) {
                const V3 pa = m.v[he[i].lo], pb = m.v[he[i].hi];
                const V3 e = pb - pa;
                const double l2 = dot(e, e);
                for (auto f : {f1, f2}) {
                    const V3 n = normalize(cross(e, fn[f]));
                    const double d = -dot(n, pa);
                    Q[he[i].lo].plane(n, d, featureWeight * l2);
                    Q[he[i].hi].plane(n, d, featureWeight * l2);
                }
            }
        }
        i = j;
    }
    // an edge at a locked vertex stays: its neighbours beyond this part of the
    // mesh are out of sight, and the link condition could not be checked
    auto cost = [&](std::uint32_t a, std::uint32_t b, V3& target) -> double {
        if (locked[a] || locked[b]) return 1e300;
        Quadric q = Q[a];
        q += Q[b];
        const V3 pa = m.v[a], pb = m.v[b];
        const V3 mid = (pa + pb) * 0.5;
        if (!q.optimum(target) || length(target - mid) > length(pb - pa)) {
            target = mid;
            double best = q.eval(mid);
            for (V3 c : {pa, pb}) {
                const double e = q.eval(c);
                if (e < best) {
                    best = e;
                    target = c;
                }
            }
        }
        return std::max(q.eval(target), 0.0) / std::max(weight[a] + weight[b], 1e-30);
    };
    std::vector<Entry> seed;
    seed.reserve(he.size() / 2 + 1);
    for (std::size_t i = 0; i < he.size(); ++i) {
        if (i > 0 && he[i].lo == he[i - 1].lo && he[i].hi == he[i - 1].hi) continue;
        V3 t;
        const double c = cost(he[i].lo, he[i].hi, t);
        if (c < 1e299) seed.push_back({c, he[i].lo, he[i].hi});
    }
    he.clear();
    he.shrink_to_fit();
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> heap(std::greater<Entry>(), std::move(seed));

    std::vector<char> deadV(nv, 0), deadF(nf, 0);
    std::size_t faces = nf;
    const double maxCost = maxError * maxError;
    std::vector<std::uint32_t> fa, fb, shared, na, nb, common;
    auto alive = [&](std::uint32_t v, std::vector<std::uint32_t>& out) {
        out.clear();
        for (auto f : vf[v])
            if (!deadF[f] && (m.f[f][0] == v || m.f[f][1] == v || m.f[f][2] == v)) out.push_back(f);
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    };
    auto neighbours = [&](std::uint32_t v, const std::vector<std::uint32_t>& fs, std::vector<std::uint32_t>& out) {
        out.clear();
        for (auto f : fs)
            for (auto u : m.f[f])
                if (u != v) out.push_back(u);
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    };
    // edge (a, b) collapsed to target - b merged into a - if the surface stays
    // a manifold (the link condition) and no triangle folds over, collapses or
    // turns into a needle; slivers (the pass below) leave other slivers be,
    // their turn comes
    auto collapse = [&](std::uint32_t a, std::uint32_t b, V3 target, bool sliver) -> bool {
        alive(a, fa);
        alive(b, fb);
        shared.clear();
        for (auto f : fb)
            if (m.f[f][0] == a || m.f[f][1] == a || m.f[f][2] == a) shared.push_back(f);
        if (shared.size() != 2) return false; // gone, or not a manifold edge
        // link condition: a and b share exactly the two vertices across the edge
        neighbours(a, fa, na);
        neighbours(b, fb, nb);
        common.clear();
        std::set_intersection(na.begin(), na.end(), nb.begin(), nb.end(), std::back_inserter(common));
        if (common.size() != 2) return false;
        for (const auto* fs : {&fa, &fb}) {
            for (auto f : *fs) {
                if (f == shared[0] || f == shared[1]) continue;
                V3 p[3], q[3];
                for (std::size_t k = 0; k < 3; ++k) {
                    const auto v = m.f[f][k];
                    p[k] = m.v[v];
                    q[k] = (v == a || v == b) ? target : m.v[v];
                }
                const V3 n0 = cross(p[1] - p[0], p[2] - p[0]), n1 = cross(q[1] - q[0], q[2] - q[0]);
                const double l0 = length(n0), l1 = length(n1);
                if (sliver && l0 <= 1e-14) continue;
                if (l1 <= 1e-14 || dot(n0, n1) < 0.3 * l0 * l1) return false;
                const double e0 = dot(q[1] - q[0], q[1] - q[0]), e1 = dot(q[2] - q[1], q[2] - q[1]), e2 = dot(q[0] - q[2], q[0] - q[2]);
                const double f0 = dot(p[1] - p[0], p[1] - p[0]), f1 = dot(p[2] - p[1], p[2] - p[1]), f2 = dot(p[0] - p[2], p[0] - p[2]);
                const double qNew = l1 / std::max({e0, e1, e2}), qOld = l0 / std::max({f0, f1, f2, 1e-30});
                if (qNew < 0.02 && qNew < 0.5 * qOld) return false;
            }
        }
        m.v[a] = target;
        Q[a] += Q[b];
        weight[a] += weight[b];
        for (auto f : shared) deadF[f] = 1;
        faces -= 2;
        for (auto f : fb) {
            if (deadF[f]) continue;
            for (auto& v : m.f[f])
                if (v == b) v = a;
            vf[a].push_back(f);
        }
        deadV[b] = 1;
        vf[b].clear();
        vf[b].shrink_to_fit();
        alive(a, fa);
        vf[a] = fa;
        return true;
    };
    while (!heap.empty() && (targetFaces == 0 || faces > targetFaces)) {
        const Entry e = heap.top();
        heap.pop();
        if (deadV[e.a] || deadV[e.b]) continue;
        V3 target;
        const double c = cost(e.a, e.b, target);
        if (c > e.cost * (1.0 + 1e-9) + 1e-30) { // the quadrics grew since: requeue
            if (c < 1e299) heap.push({c, e.a, e.b});
            continue;
        }
        if (c > maxCost) break;
        if (!collapse(e.a, e.b, target, false)) continue;
        neighbours(e.a, fa, na);
        for (auto n : na) {
            V3 t;
            const double cn = cost(e.a, n, t);
            if (cn < 1e299) heap.push({cn, e.a, n});
        }
    }
    // slivers: an edge far shorter than any cell (a cut close to a grid point,
    // a seam meeting a sharp edge) goes whatever its cost - its two ends are
    // one point already - so no triangle is left without an area
    constexpr double kSliver = 1e-4; // m
    for (int pass = 0; pass < 3; ++pass) {
        bool any = false;
        for (std::size_t f = 0; f < nf; ++f) {
            if (deadF[f]) continue;
            std::uint32_t a = 0, b = 0;
            double shortest = 1e300;
            for (std::size_t k = 0; k < 3; ++k) {
                const std::uint32_t u = m.f[f][k], w = m.f[f][(k + 1) % 3];
                const V3 d = m.v[w] - m.v[u];
                if (dot(d, d) < shortest) {
                    shortest = dot(d, d);
                    a = u;
                    b = w;
                }
            }
            if (shortest > kSliver * kSliver || locked[a] || locked[b]) continue;
            any |= collapse(a, b, (m.v[a] + m.v[b]) * 0.5, true);
        }
        if (!any) break;
    }
    // compact
    std::vector<std::uint32_t> remap(nv, 0);
    std::vector<V3> v2;
    if (origin != nullptr) origin->clear();
    for (std::size_t i = 0; i < nv; ++i)
        if (!deadV[i]) {
            remap[i] = static_cast<std::uint32_t>(v2.size());
            v2.push_back(m.v[i]);
            if (origin != nullptr) origin->push_back(static_cast<std::uint32_t>(i));
        }
    std::vector<Face> f2;
    std::vector<std::uint16_t> m2;
    for (std::size_t f = 0; f < nf; ++f)
        if (!deadF[f]) {
            f2.push_back({remap[m.f[f][0]], remap[m.f[f][1]], remap[m.f[f][2]]});
            m2.push_back(material[f]);
        }
    m.v.swap(v2);
    m.f.swap(f2);
    material.swap(m2);
}

/// The first, parallel pass over a large mesh: space cut into blocks, each
/// block's triangles simplified on its own thread down to the error bound,
/// the vertices on the blocks' borders locked where they are.
void blocks(Mesh& m, std::vector<std::uint16_t>& material, const DecimateOptions& o, unsigned threads) {
    const std::size_t nv = m.v.size(), nf = m.f.size();
    Box box;
    for (const auto& p : m.v) box.add(p);
    const double side = std::max({box.hi.x - box.lo.x, box.hi.y - box.lo.y, box.hi.z - box.lo.z}) / 14.0;
    auto cellOf = [&](V3 p) -> std::uint64_t {
        const auto ix = static_cast<std::uint64_t>(std::max(0.0, std::floor((p.x - box.lo.x) / side)));
        const auto iy = static_cast<std::uint64_t>(std::max(0.0, std::floor((p.y - box.lo.y) / side)));
        const auto iz = static_cast<std::uint64_t>(std::max(0.0, std::floor((p.z - box.lo.z) / side)));
        return ix + 64 * (iy + 64 * iz);
    };
    std::vector<std::uint64_t> fcell(nf);
    for (std::size_t f = 0; f < nf; ++f)
        fcell[f] = cellOf((m.v[m.f[f][0]] + m.v[m.f[f][1]] + m.v[m.f[f][2]]) * (1.0 / 3.0));
    // a vertex whose triangles lie in more than one block is on a border
    std::vector<std::uint64_t> vcell(nv, ~std::uint64_t{0});
    std::vector<char> border(nv, 0);
    for (std::size_t f = 0; f < nf; ++f)
        for (auto v : m.f[f]) {
            if (vcell[v] == ~std::uint64_t{0}) vcell[v] = fcell[f];
            else if (vcell[v] != fcell[f]) border[v] = 1;
        }
    std::vector<std::uint32_t> order(nf);
    for (std::size_t f = 0; f < nf; ++f) order[f] = static_cast<std::uint32_t>(f);
    std::sort(order.begin(), order.end(), [&](std::uint32_t x, std::uint32_t y) { return fcell[x] != fcell[y] ? fcell[x] < fcell[y] : x < y; });
    std::vector<std::pair<std::size_t, std::size_t>> groups; // [begin, end) in order
    for (std::size_t i = 0; i < nf;) {
        std::size_t j = i + 1;
        while (j < nf && fcell[order[j]] == fcell[order[i]]) ++j;
        groups.emplace_back(i, j);
        i = j;
    }
    struct Part {
        Mesh mesh;
        std::vector<std::uint16_t> mat;
        std::vector<std::uint32_t> global; ///< local vertex -> global (border vertices only; else ~0)
    };
    std::vector<Part> parts(groups.size());
    std::atomic<std::size_t> next{0};
    auto work = [&]() {
        for (;;) {
            const std::size_t g = next.fetch_add(1);
            if (g >= groups.size()) break;
            Part& part = parts[g];
            std::unordered_map<std::uint32_t, std::uint32_t> local;
            std::vector<char> lock;
            std::vector<std::uint32_t> source;
            for (std::size_t i = groups[g].first; i < groups[g].second; ++i) {
                const auto f = order[i];
                Face t{};
                for (std::size_t k = 0; k < 3; ++k) {
                    const auto v = m.f[f][k];
                    auto it = local.find(v);
                    if (it == local.end()) {
                        it = local.emplace(v, static_cast<std::uint32_t>(part.mesh.v.size())).first;
                        part.mesh.v.push_back(m.v[v]);
                        lock.push_back(border[v]);
                        source.push_back(v);
                    }
                    t[k] = it->second;
                }
                part.mesh.f.push_back(t);
                part.mat.push_back(material[f]);
            }
            std::vector<std::uint32_t> kept;
            simplify(part.mesh, part.mat, lock, o.maxError, 0, o.sharpDeg, o.featureWeight, &kept);
            part.global.assign(part.mesh.v.size(), ~std::uint32_t{0});
            for (std::size_t i = 0; i < kept.size(); ++i)
                if (lock[kept[i]]) part.global[i] = source[kept[i]];
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 1; t < threads; ++t) pool.emplace_back(work);
    work();
    for (auto& t : pool) t.join();
    // merge: border vertices keep one index, the rest are the blocks' own
    Mesh out;
    std::vector<std::uint16_t> mat;
    std::vector<std::uint32_t> borderIndex(nv, ~std::uint32_t{0});
    for (const auto& part : parts) {
        std::vector<std::uint32_t> remap(part.mesh.v.size());
        for (std::size_t i = 0; i < part.mesh.v.size(); ++i) {
            const auto g = part.global[i];
            if (g != ~std::uint32_t{0}) {
                if (borderIndex[g] == ~std::uint32_t{0}) {
                    borderIndex[g] = static_cast<std::uint32_t>(out.v.size());
                    out.v.push_back(m.v[g]);
                }
                remap[i] = borderIndex[g];
            } else {
                remap[i] = static_cast<std::uint32_t>(out.v.size());
                out.v.push_back(part.mesh.v[i]);
            }
        }
        for (std::size_t f = 0; f < part.mesh.f.size(); ++f) {
            out.f.push_back({remap[part.mesh.f[f][0]], remap[part.mesh.f[f][1]], remap[part.mesh.f[f][2]]});
            mat.push_back(part.mat[f]);
        }
    }
    m.v.swap(out.v);
    m.f.swap(out.f);
    material.swap(mat);
}

} // namespace

void decimate(Mesh& m, std::vector<std::uint16_t>& material, const DecimateOptions& o) {
    // the blocks are cut by space, not by thread: however many threads run
    // them, the mesh is the same (one just runs them in turn)
    unsigned threads = o.threads ? o.threads : std::max(1u, std::thread::hardware_concurrency());
    threads = std::min(threads, 32u);
    if (o.blocks && m.f.size() > 400000) blocks(m, material, o, threads);
    if (o.serial)
        simplify(m, material, std::vector<char>(m.v.size(), 0), o.maxError, o.targetFaces, o.sharpDeg, o.featureWeight);
}

} // namespace meshkit
