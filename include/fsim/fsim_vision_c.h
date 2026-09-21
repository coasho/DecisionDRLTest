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
    int32_t publish;             /* 1 (default): viewers of a published world see the camera images */
    int32_t segmentation;        /* 1: build the id-coloured copy of the vehicles, so cameras may ask for segmentation */
} fsim_vision_options;

typedef struct fsim_camera_spec {
    uint32_t struct_size;
    uint32_t width, height;      /* 128 x 128 */
    double fov_deg;              /* vertical field of view (60) */
    double offset_body_m[3];     /* mount position, body frame (x forward, y right, z down) */
    double yaw_deg, pitch_deg, roll_deg; /* mount attitude relative to the body (yaw right, pitch up, roll right) */
    int32_t hide_own_vehicle;    /* 1 (default): the carrier's own model is not drawn in this camera */
    int32_t depth;               /* 1: also deliver depth (metres) through fsim_vision_depth */
    int32_t segmentation;        /* 1: also deliver vehicle ids through fsim_vision_segmentation (needs options->segmentation) */
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
/* height x width uint16 vehicle ids, 0 where no vehicle; NULL unless the camera was added with segmentation. */
FSIM_VISION_API const uint16_t* fsim_vision_segmentation(const fsim_vision* vision, uint32_t camera, uint32_t* width, uint32_t* height);
/* The id a vehicle is painted with in segmentation images, or 0 if it is not drawn. */
FSIM_VISION_API uint32_t fsim_vision_segmentation_id(const fsim_vision* vision, uint32_t vehicle_id);
FSIM_VISION_API int fsim_vision_save_png(const fsim_vision* vision, uint32_t camera, const char* path);
/* The ids as a PNG a human can read: a distinct colour per id, black for none. */
FSIM_VISION_API int fsim_vision_save_segmentation_png(const fsim_vision* vision, uint32_t camera, const char* path);
/* Run `frames` frames without reading back so terrain tiles stream in (after a jump to a new region). */
FSIM_VISION_API int fsim_vision_settle(fsim_vision* vision, uint32_t frames);
FSIM_VISION_API double fsim_vision_last_render_ms(const fsim_vision* vision);

/* One camera per batch vehicle of a VecEnv (batch order), images packed as tensors:
 * rgb [N][H][W][3] bytes, depth [N][H][W] floats (when spec->depth),
 * segmentation [N][H][W] uint16 (when spec->segmentation). */
typedef struct fsim_vision_batch fsim_vision_batch;
FSIM_VISION_API int fsim_vision_batch_create(fsim_vecenv* env, const fsim_camera_spec* spec, const fsim_vision_options* options, fsim_vision_batch** out);
FSIM_VISION_API void fsim_vision_batch_destroy(fsim_vision_batch* batch);
FSIM_VISION_API int fsim_vision_batch_render(fsim_vision_batch* batch);
FSIM_VISION_API const uint8_t* fsim_vision_batch_rgb(const fsim_vision_batch* batch, size_t* length);
FSIM_VISION_API const float* fsim_vision_batch_depth(const fsim_vision_batch* batch, size_t* length);
FSIM_VISION_API const uint16_t* fsim_vision_batch_segmentation(const fsim_vision_batch* batch, size_t* length);
FSIM_VISION_API uint32_t fsim_vision_batch_count(const fsim_vision_batch* batch);
/* The per-camera handle underneath (save_png, extra cameras); owned by the batch. */
FSIM_VISION_API fsim_vision* fsim_vision_batch_sensors(fsim_vision_batch* batch);

#ifdef __cplusplus
}
#endif

#endif /* FSIM_VISION_C_H */
