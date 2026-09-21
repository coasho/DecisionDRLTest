/* The vision C ABI from plain C99: cameras on a world handle and on a VecEnv's world. */
#include "fsim/fsim_c.h"
#include "fsim/fsim_vision_c.h"

#include <stdio.h>
#include <string.h>

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            fprintf(stderr, "%s:%d: check failed: %s (%s)\n", __FILE__, __LINE__, #cond, fsim_last_error()); \
            return 1;                                                                                \
        }                                                                                            \
    } while (0)

int main(int argc, char** argv) {
    fsim_world_options wo;
    fsim_world* world = NULL;
    fsim_vehicle_spec spec;
    uint32_t id = 0, cam = 0, cam2 = 0, w = 0, h = 0;
    fsim_vision_options vo;
    fsim_camera_spec cs;
    fsim_vision* vision = NULL;
    const uint8_t* rgb;
    const float* depth;
    const uint16_t* ids;
    size_t i, sky, labelled;

    fsim_world_options_init(&wo);
    wo.name = "vision-c";
    wo.publish = 0;
    wo.workers = 1;
    wo.pin_workers = 0;
    wo.jsbsim_root = argc > 1 ? argv[1] : NULL;
    CHECK(fsim_world_create(&wo, &world) == FSIM_OK);
    fsim_vehicle_spec_init(&spec);
    spec.name = "cam";
    spec.altitude_msl_m = 2000.0;
    spec.airspeed_ms = 60.0;
    CHECK(fsim_world_create_vehicle(world, &spec, &id) == FSIM_OK);

    fsim_vision_options_init(&vo);
    CHECK(vo.struct_size == sizeof(fsim_vision_options));
    vo.earth = 0; /* sky and vehicles only: no tiles */
    vo.segmentation = 1;
    if (fsim_vision_create(world, &vo, &vision) != FSIM_OK) {
        fprintf(stderr, "skipping: %s" "\n", fsim_last_error());
        fsim_world_destroy(world);
        return 77; /* no Vulkan device here (CI runners) */
    }
    fsim_camera_spec_init(&cs);
    cs.width = 64;
    cs.height = 48;
    cs.pitch_deg = 60.0; /* sky */
    cs.depth = 1;
    CHECK(fsim_vision_add_camera(vision, id, &cs, &cam) == FSIM_OK);
    CHECK(fsim_vision_add_camera(vision, 999, &cs, &cam2) != FSIM_OK);
    CHECK(strstr(fsim_last_error(), "999") != NULL);
    cs.pitch_deg = -10.0;
    cs.offset_body_m[0] = -20.0;
    cs.offset_body_m[2] = -3.0;
    cs.hide_own_vehicle = 0;
    cs.depth = 0;
    cs.segmentation = 1; /* the chase camera labels the aircraft it is looking at */
    CHECK(fsim_vision_add_camera(vision, id, &cs, &cam2) == FSIM_OK);
    CHECK(fsim_vision_camera_count(vision) == 2);
    CHECK(fsim_world_step(world, 1) == FSIM_OK);
    CHECK(fsim_vision_render(vision) == FSIM_OK);
    rgb = fsim_vision_image(vision, cam, &w, &h);
    CHECK(rgb != NULL && w == 64 && h == 48);
    sky = 0;
    for (i = 0; i < (size_t)w * h; ++i) sky += (rgb[3 * i + 2] > rgb[3 * i] + 20) ? 1u : 0u;
    CHECK(sky * 10 >= (size_t)w * h * 9);
    depth = fsim_vision_depth(vision, cam, &w, &h);
    CHECK(depth != NULL && depth[0] > 1000.0f);
    CHECK(fsim_vision_depth(vision, cam2, &w, &h) == NULL);
    /* Segmentation: the chase camera's own aircraft carries id 1, the sky camera has no ids at all. */
    CHECK(fsim_vision_segmentation_id(vision, id) == 1);
    CHECK(fsim_vision_segmentation_id(vision, 999) == 0);
    ids = fsim_vision_segmentation(vision, cam2, &w, &h);
    CHECK(ids != NULL && w == 64 && h == 48);
    labelled = 0;
    for (i = 0; i < (size_t)w * h; ++i) {
        CHECK(ids[i] == 0 || ids[i] == 1);
        labelled += ids[i] == 1 ? 1u : 0u;
    }
    CHECK(labelled > 0);
    CHECK(fsim_vision_segmentation(vision, cam, &w, &h) == NULL);
    CHECK(fsim_vision_image(vision, 7, &w, &h) == NULL);
    CHECK(fsim_vision_last_render_ms(vision) > 0.0);
    fsim_vision_destroy(vision);
    fsim_vision_destroy(NULL);
    fsim_world_destroy(world);

    /* A VecEnv's world through the same handle type. */
    {
        fsim_options eo;
        fsim_vecenv* env = NULL;
        fsim_world* ew;
        fsim_options_init(&eo);
        eo.num_envs = 2;
        eo.workers = 1;
        eo.publish = 0;
        eo.world_name = "vision-c-batch";
        eo.jsbsim_root = argc > 1 ? argv[1] : NULL;
        CHECK(fsim_vecenv_create(&eo, &env) == FSIM_OK);
        ew = fsim_vecenv_world(env);
        CHECK(ew != NULL);
        CHECK(fsim_world_vehicle_count(ew) == 2);
        CHECK(fsim_world_find_vehicle(ew, "env1/0") != 0);
        CHECK(fsim_vision_create(ew, &vo, &vision) == FSIM_OK);
        fsim_camera_spec_init(&cs);
        CHECK(fsim_vision_add_camera(vision, fsim_world_find_vehicle(ew, "env0/0"), &cs, &cam) == FSIM_OK);
        CHECK(fsim_vecenv_reset(env, 1) == FSIM_OK);
        CHECK(fsim_vision_render(vision) == FSIM_OK);
        CHECK(fsim_vision_image(vision, cam, &w, &h) != NULL && w == 128);
        fsim_vision_destroy(vision);
        /* The batch API: tensors over every environment. */
        {
            fsim_vision_batch* batch = NULL;
            size_t n = 0;
            const uint8_t* t;
            const float* d;
            fsim_camera_spec_init(&cs);
            cs.width = 16;
            cs.height = 8;
            cs.pitch_deg = 60.0;
            cs.depth = 1;
            CHECK(fsim_vision_batch_create(env, &cs, &vo, &batch) == FSIM_OK);
            CHECK(fsim_vision_batch_count(batch) == 2);
            CHECK(fsim_vision_batch_render(batch) == FSIM_OK);
            t = fsim_vision_batch_rgb(batch, &n);
            CHECK(t != NULL && n == 2 * 16 * 8 * 3);
            d = fsim_vision_batch_depth(batch, &n);
            CHECK(d != NULL && n == 2 * 16 * 8 && d[0] > 1000.0f);
            CHECK(fsim_vision_camera_count(fsim_vision_batch_sensors(batch)) == 2);
            fsim_vision_batch_destroy(batch);
        }
        fsim_world_destroy(ew); /* borrowed: a no-op */
        fsim_vecenv_destroy(env);
    }
    printf("vision c abi ok\n");
    return 0;
}
