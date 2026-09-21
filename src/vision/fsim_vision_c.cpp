// C ABI over fsim::vision::Sensors.

#include "fsim/fsim_vision_c.h"

#include "fsim/Vision.h"

#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <string>

struct fsim_vision {
    fsim::World* world = nullptr; ///< owned by the world handle
    std::unique_ptr<fsim::vision::Sensors> sensors;
};

namespace {

int fail(int code, const std::string& message) {
    fsim_set_last_error(message.c_str());
    return code;
}

int guard(const char* what, const std::function<int()>& fn) noexcept {
    try {
        const int r = fn();
        if (r == FSIM_OK) fsim_set_last_error("");
        return r;
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string(what) + ": " + e.what());
    } catch (...) {
        return fail(FSIM_ERROR, std::string(what) + ": unknown error");
    }
}

} // namespace

extern "C" {

FSIM_VISION_API void fsim_vision_options_init(fsim_vision_options* o) {
    if (!o) return;
    std::memset(o, 0, sizeof *o);
    o->struct_size = sizeof *o;
    o->earth = 1;
    o->imagery = 1;
    o->elevation = 1;
    o->max_level = 15;
    o->sky = 1;
    o->max_vehicles = 64;
}

FSIM_VISION_API void fsim_camera_spec_init(fsim_camera_spec* s) {
    if (!s) return;
    std::memset(s, 0, sizeof *s);
    s->struct_size = sizeof *s;
    s->width = 128;
    s->height = 128;
    s->fov_deg = 60.0;
    s->hide_own_vehicle = 1;
}

FSIM_VISION_API int fsim_vision_create(fsim_world* world, const fsim_vision_options* options, fsim_vision** out) {
    if (!world || !options || !out || options->struct_size < sizeof(fsim_vision_options)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vision_create: bad arguments");
    *out = nullptr;
    return guard("fsim_vision_create", [&] {
        fsim::vision::Options o;
        o.earth = options->earth != 0;
        o.imagery = options->imagery != 0;
        o.elevation = options->elevation != 0;
        if (options->imagery_url) o.imageryUrl = options->imagery_url;
        if (options->elevation_url) o.elevationUrl = options->elevation_url;
        o.maxLevel = options->max_level ? options->max_level : 15u;
        o.sky = options->sky != 0;
        o.maxVehicles = options->max_vehicles ? options->max_vehicles : 64u;
        if (options->asset_dir) o.assetDir = options->asset_dir;
        o.debugLayer = options->debug_layer != 0;
        fsim::World* object = fsim_world_object(world);
        if (!object) return fail(FSIM_INVALID_ARGUMENT, "fsim_vision_create: bad world handle");
        auto v = new fsim_vision;
        v->world = object;
        v->sensors = std::make_unique<fsim::vision::Sensors>(*object, o);
        *out = v;
        return static_cast<int>(FSIM_OK);
    });
}

FSIM_VISION_API void fsim_vision_destroy(fsim_vision* vision) { delete vision; }

FSIM_VISION_API int fsim_vision_add_camera(fsim_vision* vision, uint32_t vehicle_id, const fsim_camera_spec* spec, uint32_t* out_camera) {
    if (!vision || !spec || !out_camera || spec->struct_size < sizeof(fsim_camera_spec)) return fail(FSIM_INVALID_ARGUMENT, "fsim_vision_add_camera: bad arguments");
    return guard("fsim_vision_add_camera", [&] {
        fsim::vision::CameraSpec c;
        c.width = spec->width;
        c.height = spec->height;
        c.fovDeg = spec->fov_deg;
        for (int i = 0; i < 3; ++i) c.offsetBodyM[i] = spec->offset_body_m[i];
        c.yawDeg = spec->yaw_deg;
        c.pitchDeg = spec->pitch_deg;
        c.rollDeg = spec->roll_deg;
        c.hideOwnVehicle = spec->hide_own_vehicle != 0;
        c.depth = spec->depth != 0;
        const fsim::Vehicle vehicle = vision->world->vehicle(vehicle_id);
        if (!vehicle.valid()) return fail(FSIM_INVALID_ARGUMENT, "fsim_vision_add_camera: no vehicle with id " + std::to_string(vehicle_id));
        *out_camera = vision->sensors->addCamera(vehicle, c);
        return static_cast<int>(FSIM_OK);
    });
}

FSIM_VISION_API void fsim_vision_remove_camera(fsim_vision* vision, uint32_t camera) {
    if (vision) vision->sensors->removeCamera(camera);
}

FSIM_VISION_API uint32_t fsim_vision_camera_count(const fsim_vision* vision) { return vision ? static_cast<uint32_t>(vision->sensors->cameraCount()) : 0u; }

FSIM_VISION_API int fsim_vision_render(fsim_vision* vision) {
    if (!vision) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vision_render", [&] {
        vision->sensors->render();
        return static_cast<int>(FSIM_OK);
    });
}

FSIM_VISION_API const uint8_t* fsim_vision_image(const fsim_vision* vision, uint32_t camera, uint32_t* width, uint32_t* height) {
    if (!vision) return nullptr;
    const auto img = vision->sensors->image(camera);
    if (width) *width = img.width;
    if (height) *height = img.height;
    return img.rgb;
}

FSIM_VISION_API const float* fsim_vision_depth(const fsim_vision* vision, uint32_t camera, uint32_t* width, uint32_t* height) {
    if (!vision) return nullptr;
    const auto img = vision->sensors->depth(camera);
    if (width) *width = img.width;
    if (height) *height = img.height;
    return img.metres;
}

FSIM_VISION_API int fsim_vision_save_png(const fsim_vision* vision, uint32_t camera, const char* path) {
    if (!vision || !path) return FSIM_INVALID_ARGUMENT;
    return vision->sensors->savePng(camera, path) ? FSIM_OK : fail(FSIM_ERROR, std::string("fsim_vision_save_png: cannot write ") + path);
}

FSIM_VISION_API double fsim_vision_last_render_ms(const fsim_vision* vision) { return vision ? vision->sensors->lastRenderMs() : 0.0; }

} // extern "C"
