/* fsim._vision: cameras for the Python SDK over fsim_vision_c.h.
 *
 * A module of its own because fsim_vision.dll brings Vulkan and the scene
 * graph with it: `import fsim` must work on a machine without a GPU driver,
 * and only `fsim.vision` needs this. Same rules as fsim._native - stable ABI,
 * fast calls, images handed out without a copy, the GIL released while
 * rendering. A render reads the world's state, so it marks the world busy
 * for its duration, as a step would. */
#include "fsim_py.h"

#include "fsim/fsim_vision_c.h"

static PyObject* Error;
static PyTypeObject* BufferType;
static PyTypeObject* VisionType;
static PyTypeObject* BatchType;

static PyObject* fail(void) {
    const char* text = fsim_last_error();
    PyErr_SetString(Error, text && *text ? text : "fsim vision call failed");
    return NULL;
}

static int check_args(Py_ssize_t n, Py_ssize_t lo, Py_ssize_t hi, const char* name) {
    if (n < lo || n > hi) {
        PyErr_Format(PyExc_TypeError, "%s() takes %zd to %zd arguments (%zd given)", name, lo, hi, n);
        return 0;
    }
    return 1;
}

static int as_u32(PyObject* o, uint32_t* out) {
    const unsigned long v = PyLong_AsUnsignedLong(o);
    if (v == (unsigned long)-1 && PyErr_Occurred()) return 0;
    *out = (uint32_t)v;
    return 1;
}

/* The fsim_py_ref inside a World or VecEnv of fsim._native. */
static fsim_py_ref* ref_of(PyObject* o, const char* capsule_name) {
    PyObject* capsule = PyObject_CallMethod(o, "_capsule", NULL);
    if (!capsule) return NULL;
    fsim_py_ref* ref = (fsim_py_ref*)PyCapsule_GetPointer(capsule, capsule_name);
    Py_DECREF(capsule);
    return ref;
}

#define OF(f, k) FSIM_PY_FIELD(fsim_vision_options, f, k)
static const fsim_py_field options_fields[] = {
    OF(earth, FSIM_PY_I32),        OF(imagery, FSIM_PY_I32),   OF(elevation, FSIM_PY_I32), OF(imagery_url, FSIM_PY_STR),
    OF(elevation_url, FSIM_PY_STR), OF(max_level, FSIM_PY_U32), OF(sky, FSIM_PY_I32),       OF(max_vehicles, FSIM_PY_U32),
    OF(asset_dir, FSIM_PY_STR),    OF(debug_layer, FSIM_PY_I32), OF(publish, FSIM_PY_I32),  OF(segmentation, FSIM_PY_I32)};
#undef OF

#define CF(f, k) FSIM_PY_FIELD(fsim_camera_spec, f, k)
static const fsim_py_field camera_fields[] = {
    CF(width, FSIM_PY_U32),      CF(height, FSIM_PY_U32),         CF(fov_deg, FSIM_PY_F64), CF(yaw_deg, FSIM_PY_F64),
    CF(pitch_deg, FSIM_PY_F64),  CF(roll_deg, FSIM_PY_F64),       CF(hide_own_vehicle, FSIM_PY_I32),
    CF(depth, FSIM_PY_I32),      CF(segmentation, FSIM_PY_I32)};
#undef CF
#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

/* offset_body_m is an array: read it from the dict's "offset_body_m" by hand. */
static int read_camera(PyObject* dict, fsim_camera_spec* spec) {
    fsim_camera_spec_init(spec);
    if (!dict || dict == Py_None) return 0;
    PyObject* copy = PyDict_Copy(dict);
    if (!copy) return -1;
    PyObject* offset = PyDict_GetItemString(copy, "offset_body_m"); /* borrowed */
    if (offset) {
        PyObject* seq = PySequence_Fast(offset, "offset_body_m must be three numbers");
        if (!seq || PySequence_Size(seq) != 3) {
            if (seq) PyErr_SetString(PyExc_ValueError, "offset_body_m must be three numbers");
            Py_XDECREF(seq);
            Py_DECREF(copy);
            return -1;
        }
        for (int i = 0; i < 3; ++i) {
            PyObject* item = PySequence_GetItem(seq, i);
            spec->offset_body_m[i] = item ? PyFloat_AsDouble(item) : 0.0;
            Py_XDECREF(item);
        }
        Py_DECREF(seq);
        if (PyErr_Occurred() || PyDict_DelItemString(copy, "offset_body_m") < 0) {
            Py_DECREF(copy);
            return -1;
        }
    }
    const int rc = fsim_py_set_fields(copy, spec, camera_fields, COUNT(camera_fields), "camera");
    Py_DECREF(copy); /* strings in spec point into the caller's dict, which still holds them */
    return rc;
}

/* ==========================================================================
 * Vision: cameras on the vehicles of one world
 * ======================================================================== */
typedef struct {
    PyObject_HEAD
    fsim_vision* vision;
    PyObject* owner; /* the World (or the VisionBatch for a batch's sensors) */
    int* busy;       /* the world's */
    int borrowed;
} VisionObject;

static PyObject* vision_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject *world, *options = NULL;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "Vision(world, options)");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "O|O:Vision", &world, &options)) return NULL;
    fsim_py_ref* ref = ref_of(world, FSIM_PY_WORLD_CAPSULE);
    if (!ref || !fsim_py_idle(ref->busy, "the world")) return NULL;
    fsim_vision_options o;
    fsim_vision_options_init(&o);
    if (fsim_py_set_fields(options, &o, options_fields, COUNT(options_fields), "vision") < 0) return NULL;
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    VisionObject* self = (VisionObject*)alloc(type, 0);
    if (!self) return NULL;
    self->owner = Py_NewRef(world);
    self->busy = ref->busy;
    fsim_vision* v = NULL;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vision_create((fsim_world*)ref->handle, &o, &v);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) {
        Py_DECREF(self);
        return fail();
    }
    self->vision = v;
    return (PyObject*)self;
}

static void vision_dealloc(PyObject* o) {
    VisionObject* self = (VisionObject*)o;
    PyTypeObject* tp = Py_TYPE(o);
    if (self->vision && !self->borrowed) fsim_vision_destroy(self->vision);
    Py_CLEAR(self->owner);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

static PyObject* vision_add_camera(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VisionObject* self = (VisionObject*)o;
    uint32_t vehicle, index = 0;
    if (!check_args(n, 1, 2, "add_camera") || !as_u32(args[0], &vehicle) || !fsim_py_idle(self->busy, "the world")) return NULL;
    fsim_camera_spec spec;
    if (read_camera(n == 2 ? args[1] : NULL, &spec) < 0) return NULL;
    if (fsim_vision_add_camera(self->vision, vehicle, &spec, &index) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(index);
}

static PyObject* vision_remove_camera(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VisionObject* self = (VisionObject*)o;
    uint32_t camera;
    if (!check_args(n, 1, 1, "remove_camera") || !as_u32(args[0], &camera) || !fsim_py_idle(self->busy, "the world")) return NULL;
    fsim_vision_remove_camera(self->vision, camera);
    Py_RETURN_NONE;
}

static PyObject* vision_camera_count(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "camera_count")) return NULL;
    return PyLong_FromUnsignedLong(fsim_vision_camera_count(((VisionObject*)o)->vision));
}

static PyObject* vision_render(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VisionObject* self = (VisionObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "render") || !fsim_py_idle(self->busy, "the world")) return NULL;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vision_render(self->vision);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* vision_settle(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VisionObject* self = (VisionObject*)o;
    uint32_t frames = 30;
    if (!check_args(n, 0, 1, "settle") || (n == 1 && !as_u32(args[0], &frames)) || !fsim_py_idle(self->busy, "the world")) return NULL;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vision_settle(self->vision, frames);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* image(camera) -> (buffer, height, width): valid until the next render. */
static PyObject* vision_image(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VisionObject* self = (VisionObject*)o;
    uint32_t camera, w = 0, h = 0;
    if (!check_args(n, 1, 1, "image") || !as_u32(args[0], &camera)) return NULL;
    const uint8_t* p = fsim_vision_image(self->vision, camera, &w, &h);
    return Py_BuildValue("(NII)", fsim_py_buffer_new(BufferType, o, p, (Py_ssize_t)w * h * 3, 1), h, w);
}

static PyObject* vision_depth(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VisionObject* self = (VisionObject*)o;
    uint32_t camera, w = 0, h = 0;
    if (!check_args(n, 1, 1, "depth") || !as_u32(args[0], &camera)) return NULL;
    const float* p = fsim_vision_depth(self->vision, camera, &w, &h);
    return Py_BuildValue("(NII)", fsim_py_buffer_new(BufferType, o, p, (Py_ssize_t)w * h * 4, 1), h, w);
}

static PyObject* vision_segmentation(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VisionObject* self = (VisionObject*)o;
    uint32_t camera, w = 0, h = 0;
    if (!check_args(n, 1, 1, "segmentation") || !as_u32(args[0], &camera)) return NULL;
    const uint16_t* p = fsim_vision_segmentation(self->vision, camera, &w, &h);
    return Py_BuildValue("(NII)", fsim_py_buffer_new(BufferType, o, p, (Py_ssize_t)w * h * 2, 1), h, w);
}

static PyObject* vision_segmentation_id(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t vehicle;
    if (!check_args(n, 1, 1, "segmentation_id") || !as_u32(args[0], &vehicle)) return NULL;
    return PyLong_FromUnsignedLong(fsim_vision_segmentation_id(((VisionObject*)o)->vision, vehicle));
}

static PyObject* vision_save(PyObject* o, PyObject* const* args, Py_ssize_t n, int segmentation) {
    VisionObject* self = (VisionObject*)o;
    uint32_t camera;
    if (!check_args(n, 2, 2, "save_png") || !as_u32(args[0], &camera)) return NULL;
    const char* path = PyUnicode_Check(args[1]) ? PyUnicode_AsUTF8AndSize(args[1], NULL) : NULL;
    if (!path) {
        if (!PyErr_Occurred()) PyErr_SetString(PyExc_TypeError, "path must be a string");
        return NULL;
    }
    const int rc = segmentation ? fsim_vision_save_segmentation_png(self->vision, camera, path) : fsim_vision_save_png(self->vision, camera, path);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}
static PyObject* vision_save_png(PyObject* o, PyObject* const* args, Py_ssize_t n) { return vision_save(o, args, n, 0); }
static PyObject* vision_save_segmentation_png(PyObject* o, PyObject* const* args, Py_ssize_t n) { return vision_save(o, args, n, 1); }

static PyObject* vision_last_render_ms(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "last_render_ms")) return NULL;
    return PyFloat_FromDouble(fsim_vision_last_render_ms(((VisionObject*)o)->vision));
}

#define FAST(name, fn, doc) {name, (PyCFunction)(void (*)(void))fn, METH_FASTCALL, doc}
static PyMethodDef vision_methods[] = {
    FAST("add_camera", vision_add_camera, "add_camera(vehicle_id, spec=None) -> camera index"),
    FAST("remove_camera", vision_remove_camera, "remove_camera(index)"),
    FAST("camera_count", vision_camera_count, "cameras added so far (removed ones included)"),
    FAST("render", vision_render, "draw every camera from the world's current state"),
    FAST("settle", vision_settle, "settle(frames=30): let terrain stream in without reading back"),
    FAST("image", vision_image, "image(camera) -> (rgb bytes, height, width)"),
    FAST("depth", vision_depth, "depth(camera) -> (float32 bytes, height, width)"),
    FAST("segmentation", vision_segmentation, "segmentation(camera) -> (uint16 bytes, height, width)"),
    FAST("segmentation_id", vision_segmentation_id, "segmentation_id(vehicle_id)"),
    FAST("save_png", vision_save_png, "save_png(camera, path)"),
    FAST("save_segmentation_png", vision_save_segmentation_png, "save_segmentation_png(camera, path)"),
    FAST("last_render_ms", vision_last_render_ms, "wall time of the last render"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot vision_slots[] = {
    {Py_tp_new, (void*)vision_new},
    {Py_tp_dealloc, (void*)vision_dealloc},
    {Py_tp_methods, (void*)vision_methods},
    {Py_tp_doc, (void*)"Vision(world, options) over fsim_vision (see fsim.vision.Vision)"},
    {0, NULL}};
static PyType_Spec vision_spec = {"fsim._vision.Vision", sizeof(VisionObject), 0, Py_TPFLAGS_DEFAULT, vision_slots};

/* ==========================================================================
 * VisionBatch: one camera per vehicle of a VecEnv, images as tensors
 * ======================================================================== */
typedef struct {
    PyObject_HEAD
    fsim_vision_batch* batch;
    PyObject* owner; /* the VecEnv */
    int* busy;
} BatchObject;

static PyObject* batch_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject *env, *camera = NULL, *options = NULL;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "VisionBatch(vecenv, camera, options)");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "O|OO:VisionBatch", &env, &camera, &options)) return NULL;
    fsim_py_ref* ref = ref_of(env, FSIM_PY_VECENV_CAPSULE);
    if (!ref || !fsim_py_idle(ref->busy, "the VecEnv")) return NULL;
    fsim_camera_spec spec;
    fsim_vision_options o;
    fsim_vision_options_init(&o);
    if (read_camera(camera, &spec) < 0 || fsim_py_set_fields(options, &o, options_fields, COUNT(options_fields), "vision") < 0) return NULL;
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    BatchObject* self = (BatchObject*)alloc(type, 0);
    if (!self) return NULL;
    self->owner = Py_NewRef(env);
    self->busy = ref->busy;
    fsim_vision_batch* b = NULL;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vision_batch_create((fsim_vecenv*)ref->handle, &spec, &o, &b);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) {
        Py_DECREF(self);
        return fail();
    }
    self->batch = b;
    return (PyObject*)self;
}

static void batch_dealloc(PyObject* o) {
    BatchObject* self = (BatchObject*)o;
    PyTypeObject* tp = Py_TYPE(o);
    if (self->batch) fsim_vision_batch_destroy(self->batch);
    Py_CLEAR(self->owner);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

static PyObject* batch_render(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    BatchObject* self = (BatchObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "render") || !fsim_py_idle(self->busy, "the VecEnv")) return NULL;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vision_batch_render(self->batch);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* tensors() -> (rgb, depth, segmentation) buffers; None for what was not asked for. */
static PyObject* batch_tensors(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    BatchObject* self = (BatchObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "tensors")) return NULL;
    size_t lr = 0, ld = 0, ls = 0;
    const uint8_t* rgb = fsim_vision_batch_rgb(self->batch, &lr);
    const float* depth = fsim_vision_batch_depth(self->batch, &ld);
    const uint16_t* seg = fsim_vision_batch_segmentation(self->batch, &ls);
    return Py_BuildValue("(NNN)", fsim_py_buffer_new(BufferType, o, rgb, (Py_ssize_t)lr, 1),
                         fsim_py_buffer_new(BufferType, o, depth, (Py_ssize_t)(ld * 4), 1),
                         fsim_py_buffer_new(BufferType, o, seg, (Py_ssize_t)(ls * 2), 1));
}

static PyObject* batch_count(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "count")) return NULL;
    return PyLong_FromUnsignedLong(fsim_vision_batch_count(((BatchObject*)o)->batch));
}

/* sensors(): the Vision underneath (save_png, extra cameras); keeps the batch alive. */
static PyObject* batch_sensors(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    BatchObject* self = (BatchObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "sensors")) return NULL;
    allocfunc alloc = (allocfunc)PyType_GetSlot(VisionType, Py_tp_alloc);
    VisionObject* v = (VisionObject*)alloc(VisionType, 0);
    if (!v) return NULL;
    v->vision = fsim_vision_batch_sensors(self->batch);
    v->owner = Py_NewRef(o);
    v->busy = self->busy;
    v->borrowed = 1;
    return (PyObject*)v;
}

static PyMethodDef batch_methods[] = {
    FAST("render", batch_render, "draw every batch camera"),
    FAST("tensors", batch_tensors, "(rgb, depth, segmentation) buffers, zero-copy"),
    FAST("count", batch_count, "cameras (one per batch vehicle)"),
    FAST("sensors", batch_sensors, "the Vision underneath"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot batch_slots[] = {
    {Py_tp_new, (void*)batch_new},
    {Py_tp_dealloc, (void*)batch_dealloc},
    {Py_tp_methods, (void*)batch_methods},
    {Py_tp_doc, (void*)"VisionBatch(vecenv, camera, options) over fsim_vision_batch"},
    {0, NULL}};
static PyType_Spec batch_spec = {"fsim._vision.VisionBatch", sizeof(BatchObject), 0, Py_TPFLAGS_DEFAULT, batch_slots};

static PyObject* mod_set_log_level(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    long level;
    (void)m;
    if (!check_args(n, 1, 1, "set_log_level")) return NULL;
    level = PyLong_AsLong(args[0]);
    if (level == -1 && PyErr_Occurred()) return NULL;
    fsim_vision_set_log_level((int)level);
    Py_RETURN_NONE;
}

static PyMethodDef module_methods[] = {
    FAST("set_log_level", mod_set_log_level, "set_log_level(level): this library's log and the scene graph's"),
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef module = {PyModuleDef_HEAD_INIT, "fsim._vision", "fsim cameras over the vision C ABI", -1,
                                    module_methods, NULL, NULL, NULL, NULL};

static int add_type(PyObject* m, PyType_Spec* spec, PyTypeObject** out, const char* name) {
    *out = (PyTypeObject*)PyType_FromSpec(spec);
    if (!*out) return -1;
    return PyModule_AddObjectRef(m, name, (PyObject*)*out);
}

PyMODINIT_FUNC PyInit__vision(void) {
    PyObject* native = PyImport_ImportModule("fsim._native");
    if (!native) return NULL;
    Error = PyObject_GetAttrString(native, "Error"); /* one fsim.Error for both modules */
    Py_DECREF(native);
    if (!Error) return NULL;
    PyObject* m = PyModule_Create(&module);
    if (!m) return NULL;
    if (add_type(m, &fsim_py_buffer_spec, &BufferType, "Buffer") < 0 || add_type(m, &vision_spec, &VisionType, "Vision") < 0 ||
        add_type(m, &batch_spec, &BatchType, "VisionBatch") < 0) {
        Py_DECREF(m);
        return NULL;
    }
    return m;
}
