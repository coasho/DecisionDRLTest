#include "meshkit.h"

#include "decimate.h"
#include "polygonize.h"
#include "sdf.h"

#include "core/Json.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <numeric>
#include <thread>

namespace {

using namespace meshkit;

struct Stats {
    int boundary = 0, nonmanifold = 0, misoriented = 0, components = 0, degenerate = 0;
    double volume = 0.0, area = 0.0;
};

std::uint32_t findRoot(std::vector<std::uint32_t>& parent, std::uint32_t x) {
    while (parent[x] != x) {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    return x;
}

Stats measure(const Mesh& m) {
    Stats s;
    struct HalfEdge {
        std::uint32_t lo, hi, face;
        bool forward;
    };
    std::vector<HalfEdge> he;
    he.reserve(3 * m.f.size());
    for (std::size_t f = 0; f < m.f.size(); ++f) {
        const auto& t = m.f[f];
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint32_t a = t[e], b = t[(e + 1) % 3];
            he.push_back({std::min(a, b), std::max(a, b), static_cast<std::uint32_t>(f), a < b});
        }
        const V3 p0 = m.v[t[0]], p1 = m.v[t[1]], p2 = m.v[t[2]];
        const double a2 = length(cross(p1 - p0, p2 - p0));
        s.area += 0.5 * a2;
        if (0.5 * a2 < 1e-10) ++s.degenerate;
        s.volume += dot(p0, cross(p1, p2)) / 6.0;
    }
    std::sort(he.begin(), he.end(), [](const HalfEdge& x, const HalfEdge& y) { return x.lo < y.lo || (x.lo == y.lo && x.hi < y.hi); });
    std::vector<std::uint32_t> parent(m.f.size());
    std::iota(parent.begin(), parent.end(), 0u);
    for (std::size_t i = 0; i < he.size();) {
        std::size_t j = i + 1;
        while (j < he.size() && he[j].lo == he[i].lo && he[j].hi == he[i].hi) ++j;
        const std::size_t n = j - i;
        if (n == 1) ++s.boundary;
        else if (n > 2) ++s.nonmanifold;
        else if (he[i].forward == he[i + 1].forward) ++s.misoriented;
        for (std::size_t k = i + 1; k < j; ++k) {
            const auto ra = findRoot(parent, he[i].face), rb = findRoot(parent, he[k].face);
            if (ra != rb) parent[ra] = rb;
        }
        i = j;
    }
    for (std::size_t f = 0; f < m.f.size(); ++f)
        if (findRoot(parent, static_cast<std::uint32_t>(f)) == f) ++s.components;
    return s;
}

/// Drops the pieces of a mesh no bigger than size across (their bounding
/// box's diagonal): specks the surface leaves where two cuts almost meet.
/// Returns how many.
int dropFragments(Mesh& m, std::vector<std::uint16_t>* mat, double size) {
    const std::size_t nf = m.f.size();
    std::vector<std::uint32_t> parent(m.v.size());
    std::iota(parent.begin(), parent.end(), 0u);
    for (const auto& t : m.f)
        for (std::size_t e = 1; e < 3; ++e) {
            const auto a = findRoot(parent, t[0]), b = findRoot(parent, t[e]);
            if (a != b) parent[a] = b;
        }
    std::vector<Box> box(m.v.size());
    for (std::size_t v = 0; v < m.v.size(); ++v) box[findRoot(parent, static_cast<std::uint32_t>(v))].add(m.v[v]);
    int dropped = 0;
    std::vector<char> small(m.v.size(), 0);
    for (std::size_t v = 0; v < m.v.size(); ++v)
        if (parent[v] == v && !box[v].empty() && length(box[v].hi - box[v].lo) < size) {
            small[v] = 1;
            ++dropped;
        }
    if (dropped == 0) return 0;
    std::size_t out = 0;
    for (std::size_t f = 0; f < nf; ++f) {
        if (small[findRoot(parent, m.f[f][0])]) continue;
        m.f[out] = m.f[f];
        if (mat != nullptr) (*mat)[out] = (*mat)[f];
        ++out;
    }
    m.f.resize(out);
    if (mat != nullptr) mat->resize(out);
    return dropped;
}

/// Vertex normals, a vertex split where its triangles meet at a sharp edge
/// or change material.
void shade(const Mesh& m, const std::vector<std::uint16_t>& mat, double sharpDeg, mk_mesh* out) {
    const std::size_t nv = m.v.size(), nf = m.f.size();
    std::vector<V3> fn(nf);
    std::vector<double> fa(nf);
    for (std::size_t f = 0; f < nf; ++f) {
        const V3 p0 = m.v[m.f[f][0]], p1 = m.v[m.f[f][1]], p2 = m.v[m.f[f][2]];
        const V3 n = cross(p1 - p0, p2 - p0);
        fa[f] = 0.5 * length(n);
        fn[f] = normalize(n);
    }
    std::vector<std::vector<std::uint32_t>> vf(nv);
    for (std::size_t f = 0; f < nf; ++f)
        for (auto v : m.f[f]) vf[v].push_back(static_cast<std::uint32_t>(f));
    const double sharpCos = std::cos(sharpDeg * 3.14159265358979323846 / 180.0);
    std::vector<V3> pos, nrm;
    std::vector<std::uint32_t> corner(3 * nf, 0);
    std::vector<std::uint32_t> group;
    for (std::size_t v = 0; v < nv; ++v) {
        const auto& fs = vf[v];
        const std::size_t k = fs.size();
        if (k == 0) continue;
        group.assign(k, 0);
        std::iota(group.begin(), group.end(), 0u);
        for (std::size_t i = 0; i < k; ++i)
            for (std::size_t j = i + 1; j < k; ++j) {
                const auto& A = m.f[fs[i]];
                const auto& B = m.f[fs[j]];
                int sharedCount = 0;
                for (auto x : A)
                    for (auto y : B)
                        if (x == y) ++sharedCount;
                if (sharedCount < 2) continue; // not across an edge through v
                if (mat[fs[i]] != mat[fs[j]] || dot(fn[fs[i]], fn[fs[j]]) < sharpCos) continue;
                const auto ri = findRoot(group, static_cast<std::uint32_t>(i)), rj = findRoot(group, static_cast<std::uint32_t>(j));
                if (ri != rj) group[ri] = rj;
            }
        std::vector<std::pair<std::uint32_t, std::uint32_t>> made; // root -> output vertex
        for (std::size_t i = 0; i < k; ++i) {
            const auto r = findRoot(group, static_cast<std::uint32_t>(i));
            std::uint32_t out_index = 0;
            bool found = false;
            for (const auto& pr : made)
                if (pr.first == r) {
                    out_index = pr.second;
                    found = true;
                }
            if (!found) {
                V3 n{};
                for (std::size_t j = 0; j < k; ++j)
                    if (findRoot(group, static_cast<std::uint32_t>(j)) == r) n = n + fn[fs[j]] * fa[fs[j]];
                out_index = static_cast<std::uint32_t>(pos.size());
                pos.push_back(m.v[v]);
                nrm.push_back(normalize(n));
                made.emplace_back(r, out_index);
            }
            const auto f = fs[i];
            for (std::size_t c = 0; c < 3; ++c)
                if (m.f[f][c] == v) corner[3 * f + c] = out_index;
        }
    }
    out->vertex_count = static_cast<int>(pos.size());
    out->triangle_count = static_cast<int>(nf);
    out->positions = new float[3 * pos.size()];
    out->normals = new float[3 * pos.size()];
    for (std::size_t i = 0; i < pos.size(); ++i) {
        out->positions[3 * i] = static_cast<float>(pos[i].x);
        out->positions[3 * i + 1] = static_cast<float>(pos[i].y);
        out->positions[3 * i + 2] = static_cast<float>(pos[i].z);
        out->normals[3 * i] = static_cast<float>(nrm[i].x);
        out->normals[3 * i + 1] = static_cast<float>(nrm[i].y);
        out->normals[3 * i + 2] = static_cast<float>(nrm[i].z);
    }
    out->indices = new std::uint32_t[3 * nf];
    out->materials = new std::uint16_t[nf];
    for (std::size_t i = 0; i < 3 * nf; ++i) out->indices[i] = corner[i];
    for (std::size_t f = 0; f < nf; ++f) out->materials[f] = mat[f];
}

/// fn(i) for every i below n, on `threads` threads (0: all cores).
template <typename F>
void parallel(std::size_t n, unsigned threads, F fn) {
    threads = std::min(32u, threads ? threads : std::max(1u, std::thread::hardware_concurrency()));
    std::atomic<std::size_t> next{0};
    auto work = [&]() {
        for (;;) {
            const std::size_t start = next.fetch_add(4096);
            if (start >= n) break;
            const std::size_t end = std::min(n, start + 4096);
            for (std::size_t i = start; i < end; ++i) fn(i);
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 1; t < threads; ++t) pool.emplace_back(work);
    work();
    for (auto& t : pool) t.join();
}

void setError(char* dst, std::size_t size, const char* what) {
    if (size == 0) return;
    std::snprintf(dst, size, "%s", what);
}

} // namespace

MK_API int mk_build(const char* scene_json, mk_mesh* out) {
    std::memset(out, 0, sizeof(*out));
    const auto t0 = std::chrono::steady_clock::now();
    try {
        const auto doc = fsim::core::Json::parse(scene_json, "scene");
        const Scene scene = buildScene(doc);
        Box domain = scene.root->box;
        if (const auto* d = doc.find("domain"); d != nullptr && d->isArray() && d->asArray().size() == 2) {
            const auto& a = d->asArray();
            domain = Box{};
            domain.add({a[0].asArray()[0].asNumber(), a[0].asArray()[1].asNumber(), a[0].asArray()[2].asNumber()});
            domain.add({a[1].asArray()[0].asNumber(), a[1].asArray()[1].asNumber(), a[1].asArray()[2].asNumber()});
        }
        if (domain.empty()) throw std::runtime_error("meshkit: the scene is empty");
        PolygonizeOptions po;
        po.cell = doc.number("cell", 0.01);
        po.safety = doc.number("safety", 1.5);
        po.threads = static_cast<unsigned>(std::max(0.0, doc.number("threads", 0.0)));
        auto lap = std::chrono::steady_clock::now();
        Mesh m = polygonize(*scene.root, domain, po);
        out->seconds_polygonize = std::chrono::duration<double>(std::chrono::steady_clock::now() - lap).count();
        out->raw_triangles = static_cast<int>(m.f.size());
        if (m.f.empty()) throw std::runtime_error("meshkit: no surface in the domain");
        out->fragments_removed = dropFragments(m, nullptr, doc.number("fragment", 5.0 * po.cell));
        // each triangle takes the material of the solid nearest its centre
        std::vector<std::uint16_t> mat(m.f.size());
        parallel(m.f.size(), po.threads, [&](std::size_t f) {
            const V3 c = (m.v[m.f[f][0]] + m.v[m.f[f][1]] + m.v[m.f[f][2]]) * (1.0 / 3.0);
            mat[f] = static_cast<std::uint16_t>(std::max(0, scene.root->eval(c).material));
        });
        DecimateOptions dop;
        dop.maxError = doc.number("error", 0.0008);
        dop.targetFaces = static_cast<std::size_t>(std::max(0.0, doc.number("max_triangles", 0.0)));
        dop.sharpDeg = doc.number("sharp_deg", 50.0);
        dop.blocks = doc.boolean("blocks", true);
        dop.serial = doc.boolean("serial", true);
        dop.threads = po.threads;
        lap = std::chrono::steady_clock::now();
        decimate(m, mat, dop);
        out->seconds_decimate = std::chrono::duration<double>(std::chrono::steady_clock::now() - lap).count();
        const Stats s = measure(m);
        out->boundary_edges = s.boundary;
        out->nonmanifold_edges = s.nonmanifold;
        out->misoriented_edges = s.misoriented;
        out->components = s.components;
        out->degenerate_triangles = s.degenerate;
        out->volume = s.volume;
        out->area = s.area;
        shade(m, mat, doc.number("sharp_deg", 50.0), out);
    } catch (const std::exception& e) {
        mk_release(out);
        setError(out->error, sizeof(out->error), e.what());
        return 1;
    }
    out->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return 0;
}

MK_API void mk_release(mk_mesh* mesh) {
    if (mesh == nullptr) return;
    delete[] mesh->positions;
    delete[] mesh->normals;
    delete[] mesh->indices;
    delete[] mesh->materials;
    mesh->positions = nullptr;
    mesh->normals = nullptr;
    mesh->indices = nullptr;
    mesh->materials = nullptr;
    mesh->vertex_count = mesh->triangle_count = 0;
}

struct mk_scene {
    Scene scene;
};

namespace {
void evalAll(const Scene& scene, const double* points, int n, double* distance, int* material) {
    parallel(static_cast<std::size_t>(std::max(n, 0)), 0, [&](std::size_t i) {
        const Sample s = scene.root->eval({points[3 * i], points[3 * i + 1], points[3 * i + 2]});
        distance[i] = s.d;
        if (material != nullptr) material[i] = s.material;
    });
}
} // namespace

MK_API int mk_eval(const char* scene_json, const double* points, int n, double* distance, int* material, char* error,
                   int error_size) {
    try {
        evalAll(buildScene(fsim::core::Json::parse(scene_json, "scene")), points, n, distance, material);
    } catch (const std::exception& e) {
        if (error != nullptr) setError(error, static_cast<std::size_t>(std::max(error_size, 0)), e.what());
        return 1;
    }
    return 0;
}

MK_API mk_scene* mk_scene_new(const char* scene_json, char* error, int error_size) {
    try {
        return new mk_scene{buildScene(fsim::core::Json::parse(scene_json, "scene"))};
    } catch (const std::exception& e) {
        if (error != nullptr) setError(error, static_cast<std::size_t>(std::max(error_size, 0)), e.what());
        return nullptr;
    }
}

MK_API void mk_scene_eval(const mk_scene* scene, const double* points, int n, double* distance, int* material) {
    evalAll(scene->scene, points, n, distance, material);
}

MK_API void mk_scene_free(mk_scene* scene) { delete scene; }
