/* fsim vision C ABI (design 8.4, 9.8): cameras on vehicles for any language
 * with a C FFI. Layered on fsim_c.h: a camera set is created for a world
 * handle (fsim_world_create or fsim_vecenv_world), cameras are mounted on
 * vehicle ids, fsim_vision_render() draws them all and the images are read
 * from library-owned buffers. Errors: return codes + fsim_last_error(). */
#ifndef FSIM_VISION_C_H
#define FSIM_VISION_C_H

#include "fsim/Export.h"
#include "fsim/fsim_c.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fsim_vision fsim_vision;

typedef struct fsim_vision_options {
    uint32_t struct_size;
    int32_t earth;               /* 1: the globe (0: sky and vehicles only) */
    int32_t imagery;             /* 1: satellite imagery */
    int32_t elevation;           /* 1: terrain relief */
    const char* imagery_url;     /* NULL = Esri World Imagery */
    const char* elevation_url;   /* NULL = AWS Terrarium */
    uint32_t max_level;          /* deepest tile level (15) */
    int32_t sky;                 /* 1: sky dome + sun from the world's clock */
    uint32_t max_vehicles;       /* vehicles drawn (64) */
    const char* asset_dir;       /* NULL, or an extra asset directory (models/<type>.glb) */
    int32_t debug_layer;         /* 1: Vulkan validation */
} fsim_vision_options;

typedef struct fsim_camera_spec {
    uint32_t struct_size;
    uint32_t width, height;      /* 128 x 128 */
    double fov_deg;              /* vertical field of view (60) */
    double offset_body_m[3];     /* mount position, body frame (x forward, y right, z down) */
    double yaw_deg, pitch_deg, roll_deg; /* mount attitude relative to the body (yaw right, pitch up, roll right) */
    int32_t hide_own_vehicle;    /* 1 (default): the carrier's own model is not drawn in this camera */
    int32_t depth;               /* 1: also deliver depth (metres) through fsim_vision_depth */
} fsim_camera_spec;

FSIM_VISION_API void fsim_vision_options_init(fsim_vision_options* options);
FSIM_VISION_API void fsim_camera_spec_init(fsim_camera_spec* spec);

/* Opens the process's Vulkan device (no window). */
FSIM_VISION_API int fsim_vision_create(fsim_world* world, const fsim_vision_options* options, fsim_vision** out);
FSIM_VISION_API void fsim_vision_destroy(fsim_vision* vision);

/* Mount a camera on a vehicle (at any time); remove one (its index stays valid, images become NULL). */
FSIM_VISION_API int fsim_vision_add_camera(fsim_vision* vision, uint32_t vehicle_id, const fsim_camera_spec* spec, uint32_t* out_camera);
FSIM_VISION_API void fsim_vision_remove_camera(fsim_vision* vision, uint32_t camera);
FSIM_VISION_API uint32_t fsim_vision_camera_count(const fsim_vision* vision);

/* Draw every camera from the world's current state; blocks until the images are ready. */
FSIM_VISION_API int fsim_vision_render(fsim_vision* vision);
/* height x width x 3 bytes, top row first; valid until the next render. NULL for a bad camera. */
FSIM_VISION_API const uint8_t* fsim_vision_image(const fsim_vision* vision, uint32_t camera, uint32_t* width, uint32_t* height);
/* height x width floats, metres along the view axis; NULL unless the camera was added with depth. */
FSIM_VISION_API const float* fsim_vision_depth(const fsim_vision* vision, uint32_t camera, uint32_t* width, uint32_t* height);
FSIM_VISION_API int fsim_vision_save_png(const fsim_vision* vision, uint32_t camera, const char* path);
FSIM_VISION_API double fsim_vision_last_render_ms(const fsim_vision* vision);

#ifdef __cplusplus
}
#endif

#endif /* FSIM_VISION_C_H */
