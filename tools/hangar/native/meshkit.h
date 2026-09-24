#pragma once

// meshkit: closed, simplified triangle meshes of signed-distance scenes, for
// hangar's 3D models (tools/hangar/hangar/shape). One call builds one mesh:
//
//   {"cell": 0.012, "safety": 1.5, "error": 0.0008, "max_triangles": 0,
//    "sharp_deg": 50, "domain": [[x0, y0, z0], [x1, y1, z1]], "fragment": 5 * cell,
//    "foils": [{"x": [...], "upper": [...], "lower": [...]}],
//    "root": <node>}
//
// Nodes (distances in metres, negative inside; "material" an integer that
// the triangles carry; "mirror": true copies a part to y < 0):
//   {"op": "union", "k": r, "children": [...]}           smooth over r: fillets
//   {"op": "subtract", "k": r, "a": n, "b": n, "cut_material": m}
//   {"op": "intersect", "k": r, "children": [...]}
//   {"op": "mirror", "child": n}   {"op": "paint", "material": m, "child": n}
//   {"op": "offset", "r": r, "child": n}                   grown by r (shrunk if negative)
//   {"op": "reflect", "child": n}                          reflected to y < 0 (the left of a pair)
//   {"prim": "loft", "x", "yc", "zc", "hw", "hu", "hl", "nu", "nl"}    lists per station
//   {"prim": "wing", "le": [a, b], "chord": [..], "twist": [rad, rad], "span": s,
//    "foils": [i, j], "inflate": m}
//   {"prim": "wingregion", same frame, "t": [t0, t1], "front": [..], "back": [..]}
//   {"prim": "revolve", "origin", "axis", "s": [...], "r": [...]}
//   {"prim": "capsule", "a", "b", "r"}  {"prim": "cylinder", "a", "b", "r", "round"}
//   {"prim": "box", "centre", "axes", "half", "round"}
//   {"prim": "ellipsoid", "centre", "axes", "radii"}  {"prim": "torus", "centre", "axis", "R", "r"}
//   {"prim": "slab", "origin": [x0, y0], "step": [dx, dy], "size": [nx, ny], "z": [...], "lo", "hi",
//    "rect": [x0, y0, x1, y1], "round"}    the layer lo..hi (along the normal) over a height field

#include <cstdint>

#if defined(_WIN32)
#define MK_API extern "C" __declspec(dllexport)
#else
#define MK_API extern "C" __attribute__((visibility("default")))
#endif

struct mk_mesh {
    int vertex_count;
    int triangle_count;
    float* positions;         ///< 3 per vertex
    float* normals;           ///< 3 per vertex, split at sharp edges
    std::uint32_t* indices;   ///< 3 per triangle
    std::uint16_t* materials; ///< 1 per triangle
    // what the checks read, measured on the simplified mesh
    int boundary_edges;       ///< edges with one triangle: cracks
    int nonmanifold_edges;    ///< edges with three or more triangles
    int misoriented_edges;    ///< edges whose two triangles run the same way
    int components;           ///< pieces not joined by an edge
    int degenerate_triangles; ///< area below 1e-10 m2
    int raw_triangles;        ///< before simplification
    int fragments_removed;    ///< specks smaller than the scene's "fragment" size (m), dropped
    double volume;            ///< m3, by the divergence theorem (positive: outward normals)
    double area;              ///< m2
    double seconds;
    double seconds_polygonize; ///< of which sampling and marching
    double seconds_decimate;   ///< of which simplifying
    char error[512];
};

/// Builds the mesh of a scene; returns 0, or 1 with the reason in error.
MK_API int mk_build(const char* scene_json, mk_mesh* out);
/// Frees what mk_build allocated.
MK_API void mk_release(mk_mesh* mesh);
/// Signed distances and materials of a scene at n points (3 doubles each).
MK_API int mk_eval(const char* scene_json, const double* points, int n, double* distance, int* material, char* error,
                   int error_size);

/// A scene built once for many evaluations (a probe of the airframe): null,
/// with the reason in error, if the description is wrong.
struct mk_scene;
MK_API mk_scene* mk_scene_new(const char* scene_json, char* error, int error_size);
MK_API void mk_scene_eval(const mk_scene* scene, const double* points, int n, double* distance, int* material);
MK_API void mk_scene_free(mk_scene* scene);
