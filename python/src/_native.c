/* fsim._native: the Python SDK's native core (docs/sdk/python.md).
 *
 * A thin layer over the C ABI (include/fsim/fsim_c.h), written for the least
 * cost per call rather than for convenience - the fsim package's Python code
 * provides that on top:
 *
 *   - METH_FASTCALL throughout, positional arguments only;
 *   - memory the platform owns (VecEnv buffers, vehicle states, recordings)
 *     is exported through the buffer protocol and viewed by numpy without a
 *     copy, once, and stays valid across steps;
 *   - actions, command rows and state gathers are read from and written to
 *     the caller's arrays in place;
 *   - the GIL is released for anything that simulates (steps, resets,
 *     loading vehicles), so other Python threads run meanwhile;
 *   - many-vehicle operations are one call (fsim_world_command_batch,
 *     fsim_world_gather_states).
 *
 * Stable ABI (abi3) for Python 3.11+: one binary for every interpreter from
 * 3.11 on. Built with the project's own GCC for the python.org / conda
 * CPython (MSVC) - the interface between them is C and both use the UCRT. */
#include "fsim_py.h"

#include "fsim/fsim_c.h"

#include <math.h>

static PyObject* Error;            /* fsim.Error */
static PyTypeObject* BufferType;
static PyTypeObject* VecEnvType;
static PyTypeObject* WorldType;
static PyTypeObject* ScenarioType;
static PyTypeObject* RecordingType;

static PyObject* fail(void) {
    const char* text = fsim_last_error();
    PyErr_SetString(Error, text && *text ? text : "fsim call failed");
    return NULL;
}

static int check_args(Py_ssize_t n, Py_ssize_t lo, Py_ssize_t hi, const char* name) {
    if (n < lo || n > hi) {
        if (lo == hi) PyErr_Format(PyExc_TypeError, "%s() takes %zd argument(s) (%zd given)", name, lo, n);
        else PyErr_Format(PyExc_TypeError, "%s() takes %zd to %zd arguments (%zd given)", name, lo, hi, n);
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

static int as_int(PyObject* o, int* out) {
    const long v = PyLong_AsLong(o);
    if (v == -1 && PyErr_Occurred()) return 0;
    *out = (int)v;
    return 1;
}

static const char* as_str(PyObject* o, const char* what) {
    const char* s = PyUnicode_Check(o) ? PyUnicode_AsUTF8AndSize(o, NULL) : NULL;
    if (!s && !PyErr_Occurred()) PyErr_Format(PyExc_TypeError, "%s must be a string", what);
    return s;
}

/* A C-contiguous buffer of exactly `count` items of one of the formats in
 * `formats` (single characters, optionally with a byte-order prefix). */
static int get_array(PyObject* o, Py_buffer* b, int writable, const char* formats, Py_ssize_t itemsize, const char* what) {
    if (PyObject_GetBuffer(o, b, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT | (writable ? PyBUF_WRITABLE : 0)) < 0) {
        PyErr_Clear(); /* numpy says ValueError for a strided array: here it means "convert it first" */
        PyErr_Format(PyExc_TypeError, "%s: expected a C-contiguous array of '%s' items", what, formats);
        return 0;
    }
    const char* f = b->format ? b->format : "B";
    if (*f == '<' || *f == '=' || *f == '@') ++f;
    if (b->itemsize != itemsize || f[0] == '\0' || f[1] != '\0' || !strchr(formats, f[0])) {
        PyErr_Format(PyExc_TypeError, "%s: expected a contiguous array of '%s' items of %zd bytes, got '%s'", what, formats,
                     itemsize, b->format ? b->format : "B");
        PyBuffer_Release(b);
        return 0;
    }
    return 1;
}

/* ==========================================================================
 * Option tables
 * ======================================================================== */
#define OF(f, k) FSIM_PY_FIELD(fsim_options, f, k)
static const fsim_py_field vecenv_fields[] = {
    OF(num_envs, FSIM_PY_U32), OF(vehicles_per_env, FSIM_PY_U32), OF(workers, FSIM_PY_U32), OF(seed, FSIM_PY_U64),
    OF(aircraft, FSIM_PY_STR), OF(jsbsim_root, FSIM_PY_STR), OF(task, FSIM_PY_STR), OF(observation, FSIM_PY_STR),
    OF(action, FSIM_PY_STR), OF(dt, FSIM_PY_F64), OF(frame_skip, FSIM_PY_I32), OF(max_episode_steps, FSIM_PY_U32),
    OF(latitude_deg, FSIM_PY_F64), OF(longitude_deg, FSIM_PY_F64), OF(altitude_m, FSIM_PY_F64), OF(heading_deg, FSIM_PY_F64),
    OF(airspeed_ms, FSIM_PY_F64), OF(latitude_jitter_deg, FSIM_PY_F64), OF(longitude_jitter_deg, FSIM_PY_F64),
    OF(altitude_jitter_m, FSIM_PY_F64), OF(heading_jitter_deg, FSIM_PY_F64), OF(airspeed_jitter_ms, FSIM_PY_F64),
    OF(target_altitude_delta_m, FSIM_PY_F64), OF(target_heading_delta_deg, FSIM_PY_F64), OF(world_name, FSIM_PY_STR),
    OF(publish, FSIM_PY_I32), OF(terrain, FSIM_PY_I32), OF(scenario_path, FSIM_PY_STR)};
#undef OF

#define WF(f, k) FSIM_PY_FIELD(fsim_world_options, f, k)
static const fsim_py_field world_fields[] = {
    WF(name, FSIM_PY_STR), WF(dt, FSIM_PY_F64), WF(frame_skip, FSIM_PY_I32), WF(workers, FSIM_PY_U32),
    WF(pin_workers, FSIM_PY_I32), WF(seed, FSIM_PY_U64), WF(capacity, FSIM_PY_U32), WF(publish, FSIM_PY_I32),
    WF(publish_interval_s, FSIM_PY_F64), WF(jsbsim_root, FSIM_PY_STR), WF(terrain, FSIM_PY_I32),
    WF(terrain_url, FSIM_PY_STR), WF(terrain_zoom, FSIM_PY_U32), WF(record_path, FSIM_PY_STR),
    WF(record_interval_s, FSIM_PY_F64)};
#undef WF

#define SF(f, k) FSIM_PY_FIELD(fsim_vehicle_spec, f, k)
static const fsim_py_field spec_fields[] = {
    SF(name, FSIM_PY_STR), SF(type, FSIM_PY_STR), SF(latitude_deg, FSIM_PY_F64), SF(longitude_deg, FSIM_PY_F64),
    SF(altitude_msl_m, FSIM_PY_F64), SF(heading_deg, FSIM_PY_F64), SF(pitch_deg, FSIM_PY_F64), SF(roll_deg, FSIM_PY_F64),
    SF(airspeed_ms, FSIM_PY_F64), SF(on_ground, FSIM_PY_I32), SF(model, FSIM_PY_STR), SF(control_divider, FSIM_PY_U32)};
#undef SF

#define EF(f) FSIM_PY_FIELD(fsim_environment, f, FSIM_PY_F64)
static const fsim_py_field environment_fields[] = {
    EF(epoch_utc_seconds), EF(time_factor), EF(temperature_sl_k), EF(pressure_sl_pa), EF(humidity),
    EF(wind_direction_deg), EF(wind_speed_ms), EF(wind_gust_ms), EF(turbulence), EF(visibility_m),
    EF(cloud_base_m), EF(cloud_cover), EF(precipitation)};
#undef EF

#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

/* ==========================================================================
 * VecEnv
 * ======================================================================== */
typedef struct {
    PyObject_HEAD
    fsim_vecenv* env;
    int busy;
    Py_ssize_t action_count; /* M*K*A floats per step */
    float* scratch;          /* float64 actions narrowed here */
    fsim_py_ref ref;
} VecEnvObject;

static PyObject* vecenv_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject* options = NULL;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "VecEnv(options): options is a dict");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "|O:VecEnv", &options)) return NULL;
    fsim_options o;
    fsim_options_init(&o);
    if (fsim_py_set_fields(options, &o, vecenv_fields, COUNT(vecenv_fields), "VecEnv") < 0) return NULL;

    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    VecEnvObject* self = (VecEnvObject*)alloc(type, 0);
    if (!self) return NULL;
    int rc;
    fsim_vecenv* env = NULL;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vecenv_create(&o, &env);
    Py_END_ALLOW_THREADS
    if (rc != FSIM_OK) {
        Py_DECREF(self);
        return fail();
    }
    self->env = env;
    fsim_buffers b;
    memset(&b, 0, sizeof b);
    b.struct_size = sizeof b;
    fsim_vecenv_buffers(env, &b);
    self->action_count = (Py_ssize_t)b.num_envs * b.vehicles_per_env * b.action_size;
    self->ref.handle = env;
    self->ref.busy = &self->busy;
    return (PyObject*)self;
}

static void vecenv_dealloc(PyObject* o) {
    VecEnvObject* self = (VecEnvObject*)o;
    PyTypeObject* tp = Py_TYPE(o);
    if (self->env) fsim_vecenv_destroy(self->env);
    PyMem_Free(self->scratch);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

/* step(actions): M*K*A float32 (read in place) or float64 (narrowed). */
static PyObject* vecenv_step(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    if (!check_args(n, 1, 1, "step") || !fsim_py_idle(&self->busy, "this VecEnv")) return NULL;
    Py_buffer b;
    if (PyObject_GetBuffer(args[0], &b, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT) < 0) {
        /* numpy says ValueError for a strided array, others BufferError or
         * TypeError: all mean "convert it first", which fsim.VecEnv does. */
        PyErr_Clear();
        PyErr_SetString(PyExc_TypeError, "step: actions must be a C-contiguous float32 or float64 array");
        return NULL;
    }
    const char* f = b.format ? b.format : "B";
    if (*f == '<' || *f == '=' || *f == '@') ++f;
    const float* actions = NULL;
    if (f[0] == 'f' && f[1] == '\0' && b.itemsize == 4) {
        actions = (const float*)b.buf;
    } else if (f[0] == 'd' && f[1] == '\0' && b.itemsize == 8) {
        if (!self->scratch) self->scratch = (float*)PyMem_Malloc(sizeof(float) * (size_t)(self->action_count ? self->action_count : 1));
        if (!self->scratch) {
            PyBuffer_Release(&b);
            return PyErr_NoMemory();
        }
        const Py_ssize_t count = b.len / 8 < self->action_count ? b.len / 8 : self->action_count;
        for (Py_ssize_t i = 0; i < count; ++i) self->scratch[i] = (float)((const double*)b.buf)[i];
        actions = self->scratch;
    } else {
        PyBuffer_Release(&b);
        PyErr_Format(PyExc_TypeError, "step: actions must be float32 or float64, got '%s'", b.format ? b.format : "B");
        return NULL;
    }
    if (b.len / b.itemsize != self->action_count) {
        PyErr_Format(PyExc_ValueError, "step: expected %zd action values (vehicles x action size), got %zd", self->action_count,
                     b.len / b.itemsize);
        PyBuffer_Release(&b);
        return NULL;
    }
    int rc;
    self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vecenv_step(self->env, actions, (size_t)self->action_count);
    Py_END_ALLOW_THREADS
    self->busy = 0;
    PyBuffer_Release(&b);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* vecenv_reset(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    if (!check_args(n, 0, 1, "reset") || !fsim_py_idle(&self->busy, "this VecEnv")) return NULL;
    unsigned long long seed = 0;
    if (n == 1 && args[0] != Py_None) {
        seed = PyLong_AsUnsignedLongLong(args[0]);
        if (seed == (unsigned long long)-1 && PyErr_Occurred()) return NULL;
    }
    int rc;
    self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vecenv_reset(self->env, (uint64_t)seed);
    Py_END_ALLOW_THREADS
    self->busy = 0;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* The environment's buffers, as read-only memory to view once: they are
 * allocated when the environment is and never move. */
static PyObject* vecenv_buffers(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "buffers")) return NULL;
    fsim_buffers b;
    memset(&b, 0, sizeof b);
    b.struct_size = sizeof b;
    if (fsim_vecenv_buffers(self->env, &b) != FSIM_OK) return fail();
    const Py_ssize_t nv = (Py_ssize_t)b.num_envs * b.vehicles_per_env, no = b.observation_size;
    return Py_BuildValue(
        "{s:N,s:N,s:N,s:N,s:N,s:N,s:I,s:I,s:I,s:I,s:d}",
        "observations", fsim_py_buffer_new(BufferType, o, b.observations, nv * no * 4, 1),
        "rewards", fsim_py_buffer_new(BufferType, o, b.rewards, nv * 4, 1),
        "terminated", fsim_py_buffer_new(BufferType, o, b.terminated, nv, 1),
        "truncated", fsim_py_buffer_new(BufferType, o, b.truncated, nv, 1),
        "final_observations", fsim_py_buffer_new(BufferType, o, b.final_observations, nv * no * 4, 1),
        "episode_steps", fsim_py_buffer_new(BufferType, o, b.episode_steps, (Py_ssize_t)b.num_envs * 4, 1),
        "num_envs", b.num_envs, "vehicles_per_env", b.vehicles_per_env, "observation_size", b.observation_size,
        "action_size", b.action_size, "agent_step_seconds", b.agent_step_seconds);
}

static PyObject* names_list(const char* (*get)(const fsim_vecenv*, uint32_t), const fsim_vecenv* env, uint32_t count) {
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        PyObject* s = PyUnicode_FromString(get(env, i));
        if (!s) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, s);
    }
    return list;
}

static PyObject* vecenv_set_action_ranges(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    if (!check_args(n, 1, 1, "set_action_ranges")) return NULL;
    const char* mode = as_str(args[0], "action ranges");
    if (!mode) return NULL;
    if (fsim_vecenv_set_action_ranges(self->env, mode) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* vecenv_names(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "names")) return NULL;
    fsim_buffers b;
    memset(&b, 0, sizeof b);
    b.struct_size = sizeof b;
    fsim_vecenv_buffers(self->env, &b);
    PyObject* obs = names_list(fsim_vecenv_observation_name, self->env, b.observation_size);
    PyObject* act = obs ? names_list(fsim_vecenv_action_name, self->env, b.action_size) : NULL;
    if (!act) {
        Py_XDECREF(obs);
        return NULL;
    }
    return Py_BuildValue("(NN)", obs, act);
}

static PyObject* vecenv_vehicle_steps(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "vehicle_steps")) return NULL;
    return PyLong_FromUnsignedLongLong(fsim_vecenv_vehicle_steps(((VecEnvObject*)o)->env));
}

static PyObject* vecenv_set_autoreset(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    int mode;
    if (!check_args(n, 1, 1, "set_autoreset") || !as_int(args[0], &mode) || !fsim_py_idle(&self->busy, "this VecEnv")) return NULL;
    if (fsim_vecenv_set_autoreset(self->env, mode) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* vecenv_autoreset(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "autoreset")) return NULL;
    return PyLong_FromLong(fsim_vecenv_autoreset(((VecEnvObject*)o)->env));
}

static PyObject* vecenv_vehicle_ids(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "vehicle_ids")) return NULL;
    const uint32_t count = fsim_vecenv_vehicle_ids(self->env, NULL, 0);
    PyObject* bytes = PyBytes_FromStringAndSize(NULL, (Py_ssize_t)count * 4);
    if (!bytes) return NULL;
    fsim_vecenv_vehicle_ids(self->env, (uint32_t*)PyBytes_AsString(bytes), count);
    return bytes; /* uint32 in batch order; numpy.frombuffer(..., np.uint32) */
}

static PyObject* world_borrowed(PyObject* owner, fsim_world* world, int* busy);

/* The batch's world through the object model; keeps this VecEnv alive. */
static PyObject* vecenv_world(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "world")) return NULL;
    fsim_world* w = fsim_vecenv_world(self->env);
    if (!w) return fail();
    return world_borrowed(o, w, &self->busy);
}

static PyObject* vecenv_capsule(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "_capsule")) return NULL;
    return PyCapsule_New(&((VecEnvObject*)o)->ref, FSIM_PY_VECENV_CAPSULE, NULL);
}

#define FAST(name, fn, doc) {name, (PyCFunction)(void (*)(void))fn, METH_FASTCALL, doc}
static PyMethodDef vecenv_methods[] = {
    FAST("step", vecenv_step, "step(actions): M*K*A float32 (or float64) values; results land in the buffers"),
    FAST("reset", vecenv_reset, "reset(seed=0)"),
    FAST("buffers", vecenv_buffers, "the environment's result buffers as read-only memory, and their sizes"),
    FAST("names", vecenv_names, "(observation names, action names)"),
    FAST("set_action_ranges", vecenv_set_action_ranges, "set_action_ranges(\"fixed\" | \"aircraft\")"),
    FAST("vehicle_steps", vecenv_vehicle_steps, "FDM vehicle-steps so far"),
    FAST("set_autoreset", vecenv_set_autoreset, "set_autoreset(AUTORESET_NEXT_STEP | AUTORESET_SAME_STEP)"),
    FAST("autoreset", vecenv_autoreset, "the current auto-reset mode"),
    FAST("vehicle_ids", vecenv_vehicle_ids, "world vehicle ids in batch order, as uint32 bytes"),
    FAST("world", vecenv_world, "the batch's world (a borrowed World)"),
    FAST("_capsule", vecenv_capsule, "for fsim._vision"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot vecenv_slots[] = {
    {Py_tp_new, (void*)vecenv_new},
    {Py_tp_dealloc, (void*)vecenv_dealloc},
    {Py_tp_methods, (void*)vecenv_methods},
    {Py_tp_doc, (void*)"VecEnv(options: dict) over fsim_vecenv (see fsim.VecEnv)"},
    {0, NULL}};
static PyType_Spec vecenv_spec = {"fsim._native.VecEnv", sizeof(VecEnvObject), 0, Py_TPFLAGS_DEFAULT, vecenv_slots};

/* ==========================================================================
 * World
 * ======================================================================== */
typedef struct {
    PyObject_HEAD
    fsim_world* world;
    PyObject* owner; /* the VecEnv a borrowed world belongs to */
    int own_busy;
    int* busy;
    fsim_py_ref ref;
} WorldObject;

#define WORLD_IDLE(self) fsim_py_idle((self)->busy, "this World")

static PyObject* world_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject* options = NULL;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "World(options): options is a dict");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "|O:World", &options)) return NULL;
    fsim_world_options o;
    fsim_world_options_init(&o);
    if (fsim_py_set_fields(options, &o, world_fields, COUNT(world_fields), "World") < 0) return NULL;
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    WorldObject* self = (WorldObject*)alloc(type, 0);
    if (!self) return NULL;
    self->busy = &self->own_busy;
    int rc;
    fsim_world* w = NULL;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_world_create(&o, &w);
    Py_END_ALLOW_THREADS
    if (rc != FSIM_OK) {
        Py_DECREF(self);
        return fail();
    }
    self->world = w;
    self->ref.handle = w;
    self->ref.busy = self->busy;
    return (PyObject*)self;
}

static PyObject* world_borrowed(PyObject* owner, fsim_world* world, int* busy) {
    allocfunc alloc = (allocfunc)PyType_GetSlot(WorldType, Py_tp_alloc);
    WorldObject* self = (WorldObject*)alloc(WorldType, 0);
    if (!self) return NULL;
    self->world = world;
    self->owner = Py_NewRef(owner);
    self->busy = busy;
    self->ref.handle = world;
    self->ref.busy = busy;
    return (PyObject*)self;
}

static void world_dealloc(PyObject* o) {
    WorldObject* self = (WorldObject*)o;
    PyTypeObject* tp = Py_TYPE(o);
    if (self->world && !self->owner) fsim_world_destroy(self->world);
    Py_CLEAR(self->owner);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

static PyObject* world_step(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t steps = 1;
    if (!check_args(n, 0, 1, "step") || (n == 1 && !as_u32(args[0], &steps)) || !WORLD_IDLE(self)) return NULL;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_world_step(self->world, steps);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_info(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "info")) return NULL;
    return Py_BuildValue("{s:d,s:d,s:K,s:O,s:I}", "time", fsim_world_time(self->world), "step_seconds",
                         fsim_world_step_seconds(self->world), "vehicle_steps",
                         (unsigned long long)fsim_world_vehicle_steps(self->world), "published",
                         fsim_world_published(self->world) ? Py_True : Py_False, "vehicle_count",
                         fsim_world_vehicle_count(self->world));
}

static PyObject* world_time(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "time")) return NULL;
    return PyFloat_FromDouble(fsim_world_time(((WorldObject*)o)->world));
}

static PyObject* world_create_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 1, 1, "create_vehicle") || !WORLD_IDLE(self)) return NULL;
    fsim_vehicle_spec spec;
    fsim_vehicle_spec_init(&spec);
    if (fsim_py_set_fields(args[0], &spec, spec_fields, COUNT(spec_fields), "vehicle") < 0) return NULL;
    uint32_t id = 0;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_world_create_vehicle(self->world, &spec, &id);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(id);
}

static PyObject* world_remove_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "remove_vehicle") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    if (fsim_world_remove_vehicle(self->world, id) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* reset_vehicle(id, spec=None): spec's initial-state fields, or the vehicle's own. */
static PyObject* world_reset_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 2, "reset_vehicle") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    fsim_vehicle_spec spec;
    const fsim_vehicle_spec* p = NULL;
    if (n == 2 && args[1] != Py_None) {
        fsim_vehicle_spec_init(&spec);
        if (fsim_py_set_fields(args[1], &spec, spec_fields, COUNT(spec_fields), "vehicle") < 0) return NULL;
        p = &spec;
    }
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_world_reset_vehicle(self->world, id, p);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_find_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    if (!check_args(n, 1, 1, "find_vehicle")) return NULL;
    const char* name = as_str(args[0], "name");
    if (!name) return NULL;
    return PyLong_FromUnsignedLong(fsim_world_find_vehicle(((WorldObject*)o)->world, name));
}

static PyObject* world_vehicle_ids(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "vehicle_ids")) return NULL;
    const uint32_t count = fsim_world_vehicle_ids(self->world, NULL, 0);
    PyObject* bytes = PyBytes_FromStringAndSize(NULL, (Py_ssize_t)count * 4);
    if (!bytes) return NULL;
    fsim_world_vehicle_ids(self->world, (uint32_t*)PyBytes_AsString(bytes), count);
    return bytes;
}

static PyObject* world_vehicle_info(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "vehicle_info") || !as_u32(args[0], &id)) return NULL;
    return Py_BuildValue("(ss)", fsim_vehicle_name(self->world, id), fsim_vehicle_type(self->world, id));
}

/* state_buffer(id, sensed): the vehicle's live state struct, without a copy.
 * Writable only so that ctypes can map a struct over it; it is the
 * platform's snapshot, rewritten every step. */
static PyObject* world_state_buffer(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int sensed = 0;
    if (!check_args(n, 1, 2, "state_buffer") || !as_u32(args[0], &id) || (n == 2 && !as_int(args[1], &sensed))) return NULL;
    const fsim_vehicle_state* s = sensed ? fsim_vehicle_sensed_ptr(self->world, id) : fsim_vehicle_state_ptr(self->world, id);
    if (!s) {
        PyErr_Format(PyExc_KeyError, "no vehicle with id %u", (unsigned)id);
        return NULL;
    }
    return fsim_py_buffer_new(BufferType, o, s, (Py_ssize_t)sizeof(fsim_vehicle_state), 0);
}

/* gather_states(ids uint32[n], out bytes[n * sizeof(state)], sensed) */
static PyObject* world_gather_states(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    int sensed = 0;
    if (!check_args(n, 2, 3, "gather_states") || (n == 3 && !as_int(args[2], &sensed)) || !WORLD_IDLE(self)) return NULL;
    Py_buffer ids, out;
    if (!get_array(args[0], &ids, 0, "IL", 4, "gather_states ids")) return NULL;
    if (PyObject_GetBuffer(args[1], &out, PyBUF_C_CONTIGUOUS | PyBUF_WRITABLE) < 0) {
        PyBuffer_Release(&ids);
        PyErr_Clear();
        PyErr_SetString(PyExc_TypeError, "gather_states: out must be a writable C-contiguous array");
        return NULL;
    }
    const Py_ssize_t count = ids.len / 4;
    if (out.len != count * (Py_ssize_t)sizeof(fsim_vehicle_state)) {
        PyErr_Format(PyExc_ValueError, "gather_states: out holds %zd bytes, %zd vehicles need %zd", out.len, count,
                     count * (Py_ssize_t)sizeof(fsim_vehicle_state));
        PyBuffer_Release(&ids);
        PyBuffer_Release(&out);
        return NULL;
    }
    const int rc = fsim_world_gather_states(self->world, (const uint32_t*)ids.buf, (uint32_t)count, sensed, (fsim_vehicle_state*)out.buf);
    PyBuffer_Release(&ids);
    PyBuffer_Release(&out);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* command(id, level, values): one vehicle, the level's fields as a sequence of floats. */
static PyObject* world_command(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    if (!check_args(n, 3, 3, "command") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self)) return NULL;
    const uint32_t fields = fsim_command_field_count(level);
    double row[8];
    PyObject* seq = PySequence_Fast(args[2], "command: values must be a sequence of numbers");
    if (!seq) return NULL;
    if (!fields || PySequence_Size(seq) != (Py_ssize_t)fields) {
        PyErr_Format(PyExc_ValueError, "command: level %d takes %u values", level, (unsigned)fields);
        Py_DECREF(seq);
        return NULL;
    }
    for (uint32_t i = 0; i < fields; ++i) {
        PyObject* item = PySequence_GetItem(seq, (Py_ssize_t)i);
        row[i] = item ? PyFloat_AsDouble(item) : -1.0;
        Py_XDECREF(item);
        if (PyErr_Occurred()) {
            Py_DECREF(seq);
            return NULL;
        }
    }
    Py_DECREF(seq);
    if (fsim_world_command_batch(self->world, level, &id, 1, row, 0) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* command_batch(level, ids uint32[n], values float64[n][fields]) */
static PyObject* world_command_batch(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    int level;
    if (!check_args(n, 3, 3, "command_batch") || !as_int(args[0], &level) || !WORLD_IDLE(self)) return NULL;
    const uint32_t fields = fsim_command_field_count(level);
    if (!fields) {
        PyErr_Format(PyExc_ValueError, "command_batch: level %d cannot be batched", level);
        return NULL;
    }
    Py_buffer ids, values;
    if (!get_array(args[1], &ids, 0, "IL", 4, "command_batch ids")) return NULL;
    if (!get_array(args[2], &values, 0, "d", 8, "command_batch values")) {
        PyBuffer_Release(&ids);
        return NULL;
    }
    const Py_ssize_t count = ids.len / 4;
    if (values.len != count * (Py_ssize_t)fields * 8) {
        PyErr_Format(PyExc_ValueError, "command_batch: %zd vehicles at level %d need %zd values, got %zd", count, level,
                     count * (Py_ssize_t)fields, values.len / 8);
        PyBuffer_Release(&ids);
        PyBuffer_Release(&values);
        return NULL;
    }
    const int rc = fsim_world_command_batch(self->world, level, (const uint32_t*)ids.buf, (uint32_t)count, (const double*)values.buf, fields);
    PyBuffer_Release(&ids);
    PyBuffer_Release(&values);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* A route: rows of (latitude_rad, longitude_rad, altitude_msl_m, airspeed_ms,
 * capture_radius_m). *out is PyMem-allocated (free it); 0, or -1 with an error set. */
static int read_points(PyObject* o, fsim_position_command** out, uint32_t* count) {
    *out = NULL;
    *count = 0;
    PyObject* seq = PySequence_Fast(o, "points must be a sequence of 5-number rows");
    if (!seq) return -1;
    const uint32_t point_count = (uint32_t)PySequence_Size(seq);
    fsim_position_command* points = (fsim_position_command*)PyMem_Malloc(sizeof(fsim_position_command) * (point_count ? point_count : 1));
    for (uint32_t i = 0; points && i < point_count; ++i) {
        PyObject* row = PySequence_GetItem(seq, (Py_ssize_t)i);
        double v[5] = {0, 0, 0, 0, 0};
        PyObject* r = row ? PySequence_Fast(row, "each point must be 5 numbers") : NULL;
        if (r && PySequence_Size(r) == 5)
            for (int k = 0; k < 5; ++k) {
                PyObject* item = PySequence_GetItem(r, k);
                v[k] = item ? PyFloat_AsDouble(item) : 0.0;
                Py_XDECREF(item);
            }
        else if (r) PyErr_SetString(PyExc_ValueError, "each point must be (latitude_rad, longitude_rad, altitude_msl_m, airspeed_ms, capture_radius_m)");
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
        points[i].latitude_rad = v[0];
        points[i].longitude_rad = v[1];
        points[i].altitude_msl_m = v[2];
        points[i].airspeed_ms = v[3];
        points[i].capture_radius_m = v[4];
    }
    Py_DECREF(seq);
    if (!points) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(points);
        return -1;
    }
    *out = points;
    *count = point_count;
    return 0;
}

/* command_behavior(id, behavior_id, target, params dict, points) - points: rows of 5 floats */
static PyObject* world_command_behavior(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, target = 0;
    if (!check_args(n, 2, 5, "command_behavior") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* bid = as_str(args[1], "behaviour id");
    if (!bid || (n >= 3 && args[2] != Py_None && !as_u32(args[2], &target))) return NULL;
    fsim_py_params p;
    if (fsim_py_params_read(n >= 4 ? args[3] : NULL, &p) < 0) {
        fsim_py_params_free(&p);
        return NULL;
    }
    fsim_position_command* points = NULL;
    uint32_t point_count = 0;
    if (n >= 5 && args[4] != Py_None && read_points(args[4], &points, &point_count) < 0) {
        fsim_py_params_free(&p);
        return NULL;
    }
    fsim_behavior_command c;
    memset(&c, 0, sizeof c);
    c.id = bid;
    c.target = target;
    c.param_names = p.names;
    c.param_values = p.values;
    c.param_count = p.count;
    c.points = points;
    c.point_count = point_count;
    const int rc = fsim_vehicle_command_behavior(self->world, id, &c);
    PyMem_Free(points);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* --- Capabilities and activities (ABI 1.4) ------------------------------------------
 * Results are tuples (status, reason, activity, other, clamped); activity
 * infos (id, vehicle, capability, source, axes, state, reason, by,
 * constraints, constraints_seen, start_time, end_time). fsim/world.py wraps
 * them. */

static PyObject* result_tuple(const fsim_command_result* r) {
    return Py_BuildValue("(iiKKO)", r->status, r->reason, (unsigned long long)r->activity, (unsigned long long)r->other,
                         (r->flags & 1u) ? Py_True : Py_False);
}

static PyObject* info_tuple(const fsim_activity_info* a) {
    return Py_BuildValue("(KIIiIiiKIIdd)", (unsigned long long)a->id, a->vehicle, a->capability, a->source, a->axes, a->state, a->reason,
                         (unsigned long long)a->by, a->constraints, a->constraints_seen, a->start_time, a->end_time);
}

static int as_u64(PyObject* o, uint64_t* out) {
    const unsigned long long v = PyLong_AsUnsignedLongLong(o);
    if (v == (unsigned long long)-1 && PyErr_Occurred()) return 0;
    *out = (uint64_t)v;
    return 1;
}

/* source, axes, range, min_version: optional, None = the default */
static int read_options(PyObject* const* args, Py_ssize_t n, Py_ssize_t first, fsim_command_options* o) {
    int v;
    uint32_t u;
    fsim_command_options_init(o);
    if (n > first && args[first] != Py_None) {
        if (!as_int(args[first], &v)) return 0;
        o->source = v;
    }
    if (n > first + 1 && args[first + 1] != Py_None) {
        if (!as_u32(args[first + 1], &u)) return 0;
        o->axes = u;
    }
    if (n > first + 2 && args[first + 2] != Py_None) {
        if (!as_int(args[first + 2], &v)) return 0;
        o->range = v;
    }
    if (n > first + 3 && args[first + 3] != Py_None) {
        if (!as_u32(args[first + 3], &u)) return 0;
        o->min_version = u;
    }
    return 1;
}

/* A sequence of at most 8 numbers into row; returns how many, or -1. */
static Py_ssize_t read_values(PyObject* o, double* row, const char* what) {
    PyObject* seq = PySequence_Fast(o, "values must be a sequence of numbers");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    if (count > 8) {
        PyErr_Format(PyExc_ValueError, "%s: at most 8 values", what);
        Py_DECREF(seq);
        return -1;
    }
    for (Py_ssize_t i = 0; i < count; ++i) {
        PyObject* item = PySequence_GetItem(seq, i);
        row[i] = item ? PyFloat_AsDouble(item) : -1.0;
        Py_XDECREF(item);
        if (PyErr_Occurred()) {
            Py_DECREF(seq);
            return -1;
        }
    }
    Py_DECREF(seq);
    return count;
}

/* submit(id, level, values, source=None, axes=None, range=None, min_version=None) -> result */
static PyObject* world_submit(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    double row[8];
    fsim_command_options opt;
    fsim_command_result r;
    if (!check_args(n, 3, 7, "submit") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[2], row, "submit");
    if (count < 0 || !read_options(args, n, 3, &opt)) return NULL;
    if (fsim_vehicle_submit(self->world, id, level, row, (uint32_t)count, &opt, &r) != FSIM_OK) return fail();
    return result_tuple(&r);
}

/* submit_behavior(id, behavior, target=0, params=None, points=None, source=None, axes=None, range=None, min_version=None) */
static PyObject* world_submit_behavior(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, target = 0;
    fsim_command_options opt;
    fsim_command_result r;
    if (!check_args(n, 2, 9, "submit_behavior") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* bid = as_str(args[1], "behaviour id");
    if (!bid || (n >= 3 && args[2] != Py_None && !as_u32(args[2], &target)) || !read_options(args, n, 5, &opt)) return NULL;
    fsim_py_params p;
    if (fsim_py_params_read(n >= 4 ? args[3] : NULL, &p) < 0) {
        fsim_py_params_free(&p);
        return NULL;
    }
    fsim_position_command* points = NULL;
    uint32_t point_count = 0;
    if (n >= 5 && args[4] != Py_None && read_points(args[4], &points, &point_count) < 0) {
        fsim_py_params_free(&p);
        return NULL;
    }
    fsim_behavior_command c;
    memset(&c, 0, sizeof c);
    c.id = bid;
    c.target = target;
    c.param_names = p.names;
    c.param_values = p.values;
    c.param_count = p.count;
    c.points = points;
    c.point_count = point_count;
    const int rc = fsim_vehicle_submit_behavior(self->world, id, &c, &opt, &r);
    PyMem_Free(points);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    return result_tuple(&r);
}

/* submit_support(id, kind, values, source=None, axes=None, range=None, min_version=None) -> result */
static PyObject* world_submit_support(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int kind;
    double row[8];
    fsim_command_options opt;
    fsim_command_result r;
    if (!check_args(n, 3, 7, "submit_support") || !as_u32(args[0], &id) || !as_int(args[1], &kind) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[2], row, "submit_support");
    if (count < 0 || !read_options(args, n, 3, &opt)) return NULL;
    if (fsim_vehicle_submit_support(self->world, id, kind, row, (uint32_t)count, &opt, &r) != FSIM_OK) return fail();
    return result_tuple(&r);
}

/* activity_update(activity, values) -> result */
static PyObject* world_activity_update(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    double row[8];
    fsim_command_result r;
    if (!check_args(n, 2, 2, "activity_update") || !as_u64(args[0], &activity) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[1], row, "activity_update");
    if (count < 0) return NULL;
    if (fsim_activity_update(self->world, activity, row, (uint32_t)count, &r) != FSIM_OK) return fail();
    return result_tuple(&r);
}

/* activity_update_batch(activities uint64[n], values float64[n][stride], stride) */
static PyObject* world_activity_update_batch(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t stride;
    Py_buffer acts, values;
    if (!check_args(n, 3, 3, "activity_update_batch") || !as_u32(args[2], &stride) || !WORLD_IDLE(self)) return NULL;
    if (!get_array(args[0], &acts, 0, "Q", 8, "activity_update_batch activities")) return NULL;
    if (!get_array(args[1], &values, 0, "d", 8, "activity_update_batch values")) {
        PyBuffer_Release(&acts);
        return NULL;
    }
    const Py_ssize_t count = acts.len / 8;
    if (stride == 0 || values.len != count * (Py_ssize_t)stride * 8) {
        PyErr_Format(PyExc_ValueError, "activity_update_batch: %zd activities at a stride of %u need %zd values, got %zd", count,
                     (unsigned)stride, count * (Py_ssize_t)stride, values.len / 8);
        PyBuffer_Release(&acts);
        PyBuffer_Release(&values);
        return NULL;
    }
    const int rc = fsim_activity_update_batch(self->world, (const fsim_activity_id*)acts.buf, (uint32_t)count, (const double*)values.buf, stride);
    PyBuffer_Release(&acts);
    PyBuffer_Release(&values);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* activity_cancel(activity) -> result */
static PyObject* world_activity_cancel(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    fsim_command_result r;
    if (!check_args(n, 1, 1, "activity_cancel") || !as_u64(args[0], &activity) || !WORLD_IDLE(self)) return NULL;
    if (fsim_activity_cancel(self->world, activity, &r) != FSIM_OK) return fail();
    return result_tuple(&r);
}

/* activity_info(activity) -> info tuple, or None if the vehicle does not remember it */
static PyObject* world_activity_info(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    fsim_activity_info a;
    if (!check_args(n, 1, 1, "activity_info") || !as_u64(args[0], &activity)) return NULL;
    if (fsim_activity_get(self->world, activity, &a) != FSIM_OK) Py_RETURN_NONE;
    return info_tuple(&a);
}

/* vehicle_activities(id) -> [info tuple]: live, then ended (newest first) */
static PyObject* world_vehicle_activities(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_activity_info a;
    if (!check_args(n, 1, 1, "vehicle_activities") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_activity_count(self->world, id);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        if (fsim_vehicle_activity(self->world, id, i, &a) != FSIM_OK) continue;
        PyObject* t = info_tuple(&a);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* capabilities(id) -> [(id, version, kind, interactions, level, axes, terminating, needs_target, behavior,
 *                       [(name, unit, min, max, default, optional)], axis_groups)] */
static PyObject* world_capabilities(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_capability_info c;
    fsim_parameter_info p;
    if (!check_args(n, 1, 1, "capabilities") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_capability_count(self->world, id);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        if (fsim_vehicle_capability(self->world, id, i, &c) != FSIM_OK) continue;
        PyObject* params = PyList_New(0);
        for (uint32_t k = 0; params && k < c.parameter_count; ++k) {
            if (fsim_vehicle_capability_parameter(self->world, id, i, k, &p) != FSIM_OK) continue;
            PyObject* t = Py_BuildValue("(ssdddO)", p.name, p.unit, p.min, p.max, p.default_value, p.optional ? Py_True : Py_False);
            if (!t || PyList_Append(params, t) < 0) {
                Py_XDECREF(t);
                Py_CLEAR(params);
                break;
            }
            Py_DECREF(t);
        }
        PyObject* t = params ? Py_BuildValue("(sIiIiIOOsNI)", c.id, c.version, c.kind, c.interactions, c.level, c.axes,
                                             c.terminating ? Py_True : Py_False, c.needs_target ? Py_True : Py_False, c.behavior, params,
                                             c.axis_groups)
                             : NULL;
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* capability_status(id, capability) -> (availability, reason) */
static PyObject* world_capability_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int32_t availability = 0, reason = 0;
    if (!check_args(n, 2, 2, "capability_status") || !as_u32(args[0], &id)) return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_capability_status(self->world, id, cap, &availability, &reason) != FSIM_OK) return fail();
    return Py_BuildValue("(ii)", availability, reason);
}

/* profile_value(id, path) -> float (NaN if unknown) */
static PyObject* world_profile_value(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    double value = 0.0;
    if (!check_args(n, 2, 2, "profile_value") || !as_u32(args[0], &id)) return NULL;
    const char* path = as_str(args[1], "profile path");
    if (!path) return NULL;
    if (fsim_vehicle_profile_value(self->world, id, path, &value) != FSIM_OK) return fail();
    return PyFloat_FromDouble(value);
}

/* profile_section(id, section) -> (version, provenance) */
static PyObject* world_profile_section(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, version = 0;
    int32_t provenance = 0;
    if (!check_args(n, 2, 2, "profile_section") || !as_u32(args[0], &id)) return NULL;
    const char* section = as_str(args[1], "section name");
    if (!section) return NULL;
    if (fsim_vehicle_profile_section(self->world, id, section, &version, &provenance) != FSIM_OK) {
        PyErr_Format(PyExc_KeyError, "no vehicle %u or no profile section %s", (unsigned)id, section);
        return NULL;
    }
    return Py_BuildValue("(Ii)", version, provenance);
}

/* set_vehicle_default(id, mode) -> reason (0: set) */
static PyObject* world_set_vehicle_default(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int mode;
    int32_t reason = 0;
    if (!check_args(n, 2, 2, "set_vehicle_default") || !as_u32(args[0], &id) || !as_int(args[1], &mode) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_set_default(self->world, id, mode, &reason) != FSIM_OK && reason == 0) return fail();
    return PyLong_FromLong(reason);
}

/* vehicle_default(id) -> mode */
static PyObject* world_vehicle_default(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id;
    int32_t mode = 0;
    if (!check_args(n, 1, 1, "vehicle_default") || !as_u32(args[0], &id)) return NULL;
    if (fsim_vehicle_get_default(((WorldObject*)o)->world, id, &mode) != FSIM_OK) return fail();
    return PyLong_FromLong(mode);
}

/* set_protection(id, mode) */
static PyObject* world_set_protection(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int mode;
    if (!check_args(n, 2, 2, "set_protection") || !as_u32(args[0], &id) || !as_int(args[1], &mode) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_set_protection(self->world, id, mode) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* protection(id) -> mode */
static PyObject* world_protection(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id;
    int32_t mode = 0;
    if (!check_args(n, 1, 1, "protection") || !as_u32(args[0], &id)) return NULL;
    if (fsim_vehicle_get_protection(((WorldObject*)o)->world, id, &mode) != FSIM_OK) return fail();
    return PyLong_FromLong(mode);
}

/* envelope(id) -> (mode, [(limited_updates, exceeded_updates, exceeded_s, worst_excess)] per limit) */
static PyObject* world_envelope(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_envelope_status s;
    if (!check_args(n, 1, 1, "envelope") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_envelope(self->world, id, &s) != FSIM_OK) return fail();
    PyObject* rows = PyList_New(FSIM_LIMIT_COUNT);
    for (int l = 0; rows && l < FSIM_LIMIT_COUNT; ++l) {
        PyObject* t = Py_BuildValue("(IIdd)", s.limited_updates[l], s.exceeded_updates[l], s.exceeded_s[l], s.worst_excess[l]);
        if (!t || PyList_SetItem(rows, l, t) < 0) { /* SetItem takes t, even when it fails */
            Py_CLEAR(rows);
            break;
        }
    }
    return rows ? Py_BuildValue("(iN)", s.mode, rows) : NULL;
}

static PyObject* world_active_level(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id;
    if (!check_args(n, 1, 1, "active_level") || !as_u32(args[0], &id)) return NULL;
    return PyLong_FromLong(fsim_vehicle_active_level(((WorldObject*)o)->world, id));
}

static PyObject* world_behavior_finished(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id;
    if (!check_args(n, 1, 1, "behavior_finished") || !as_u32(args[0], &id)) return NULL;
    return PyBool_FromLong(fsim_vehicle_behavior_finished(((WorldObject*)o)->world, id) == 1);
}

static PyObject* world_use_controller(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    if (!check_args(n, 3, 3, "use_controller") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self)) return NULL;
    const char* cid = as_str(args[2], "controller id");
    if (!cid) return NULL;
    if (fsim_vehicle_use_controller(self->world, id, level, cid) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_set_controller_parameter(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    if (!check_args(n, 4, 4, "set_controller_parameter") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self))
        return NULL;
    const char* name = as_str(args[2], "parameter name");
    const double value = name ? PyFloat_AsDouble(args[3]) : 0.0;
    if (!name || (value == -1.0 && PyErr_Occurred())) return NULL;
    if (fsim_vehicle_set_controller_parameter(self->world, id, level, name, value) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_controller_parameter(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    if (!check_args(n, 3, 3, "controller_parameter") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self))
        return NULL;
    const char* name = as_str(args[2], "parameter name");
    double v = 0.0;
    if (!name) return NULL;
    if (fsim_vehicle_controller_parameter(self->world, id, level, name, &v) != FSIM_OK) return fail();
    return PyFloat_FromDouble(v);
}

static PyObject* world_get_property(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 2, 2, "get_property") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* path = as_str(args[1], "property path");
    double v = 0.0;
    if (!path) return NULL;
    if (fsim_vehicle_get_property(self->world, id, path, &v) != FSIM_OK) return fail();
    return PyFloat_FromDouble(v);
}

static PyObject* world_set_property(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 3, 3, "set_property") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* path = as_str(args[1], "property path");
    const double v = path ? PyFloat_AsDouble(args[2]) : 0.0;
    if (!path || (v == -1.0 && PyErr_Occurred())) return NULL;
    if (fsim_vehicle_set_property(self->world, id, path, v) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_get_environment(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "get_environment")) return NULL;
    fsim_environment e;
    fsim_environment_init(&e);
    if (fsim_world_get_environment(((WorldObject*)o)->world, &e) != FSIM_OK) return fail();
    return fsim_py_get_fields(&e, environment_fields, COUNT(environment_fields));
}

/* set_environment(changes): the current environment with `changes` applied. */
static PyObject* world_set_environment(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 1, 1, "set_environment") || !WORLD_IDLE(self)) return NULL;
    fsim_environment e;
    fsim_environment_init(&e);
    if (fsim_world_get_environment(self->world, &e) != FSIM_OK) return fail();
    if (fsim_py_set_fields(args[0], &e, environment_fields, COUNT(environment_fields), "environment") < 0) return NULL;
    if (fsim_world_set_environment(self->world, &e) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_add_effect(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 2, 3, "add_effect") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* effect = as_str(args[1], "effect id");
    fsim_py_params p;
    if (!effect || fsim_py_params_read(n == 3 ? args[2] : NULL, &p) < 0) {
        if (effect) fsim_py_params_free(&p);
        return NULL;
    }
    const int rc = fsim_vehicle_add_effect(self->world, id, effect, p.names, p.values, p.count);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_clear_effects(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "clear_effects") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_clear_effects(self->world, id) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_comm_create_node(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t address;
    if (!check_args(n, 1, 1, "comm_create_node") || !as_u32(args[0], &address) || !WORLD_IDLE(self)) return NULL;
    if (fsim_comm_create_node(self->world, address) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* comm_send(from, to, channel, format, data: bytes-like) */
static PyObject* world_comm_send(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t from, to, channel, format;
    if (!check_args(n, 5, 5, "comm_send") || !as_u32(args[0], &from) || !as_u32(args[1], &to) || !as_u32(args[2], &channel) ||
        !as_u32(args[3], &format) || !WORLD_IDLE(self))
        return NULL;
    Py_buffer data;
    if (PyObject_GetBuffer(args[4], &data, PyBUF_C_CONTIGUOUS) < 0) return NULL;
    const int rc = fsim_comm_send(self->world, from, to, channel, format, data.buf, (size_t)data.len);
    PyBuffer_Release(&data);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* comm_inbox(node) -> [(from, to, channel, format, time_sent, time_delivered, bytes)] */
static PyObject* world_comm_inbox(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t node;
    if (!check_args(n, 1, 1, "comm_inbox") || !as_u32(args[0], &node)) return NULL;
    const uint32_t count = fsim_comm_inbox_count(self->world, node);
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        fsim_message m;
        memset(&m, 0, sizeof m);
        if (fsim_comm_inbox_get(self->world, node, i, &m) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyObject* t = Py_BuildValue("(IIIIddy#)", m.from, m.to, m.channel, m.format, m.time_sent, m.time_delivered,
                                    (const char*)m.bytes, (Py_ssize_t)m.length);
        if (!t) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, t);
    }
    return list;
}

static PyObject* world_comm_set_medium(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 1, 2, "comm_set_medium") || !WORLD_IDLE(self)) return NULL;
    const char* medium = as_str(args[0], "medium id");
    fsim_py_params p;
    if (!medium || fsim_py_params_read(n == 2 ? args[1] : NULL, &p) < 0) {
        if (medium) fsim_py_params_free(&p);
        return NULL;
    }
    const int rc = fsim_comm_set_medium(self->world, medium, p.names, p.values, p.count);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_comm_attach_protocol(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t node;
    if (!check_args(n, 2, 3, "comm_attach_protocol") || !as_u32(args[0], &node) || !WORLD_IDLE(self)) return NULL;
    const char* protocol = as_str(args[1], "protocol id");
    fsim_py_params p;
    if (!protocol || fsim_py_params_read(n == 3 ? args[2] : NULL, &p) < 0) {
        if (protocol) fsim_py_params_free(&p);
        return NULL;
    }
    const int rc = fsim_comm_attach_protocol(self->world, node, protocol, p.names, p.values, p.count);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* world_comm_attach_udp_bridge(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t node, local, remote;
    if (!check_args(n, 4, 4, "comm_attach_udp_bridge") || !as_u32(args[0], &node) || !as_u32(args[1], &local) ||
        !as_u32(args[3], &remote) || !WORLD_IDLE(self))
        return NULL;
    const char* host = as_str(args[2], "remote host");
    if (!host) return NULL;
    if (fsim_comm_attach_udp_bridge(self->world, node, (uint16_t)local, host, (uint16_t)remote) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

typedef struct {
    PyObject_HEAD
    fsim_scenario* scenario;
} ScenarioObject;

static PyObject* world_apply_scenario(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 1, 1, "apply_scenario") || !WORLD_IDLE(self)) return NULL;
    if (!PyObject_TypeCheck(args[0], ScenarioType)) {
        PyErr_SetString(PyExc_TypeError, "apply_scenario: expected a Scenario");
        return NULL;
    }
    const fsim_scenario* sc = ((ScenarioObject*)args[0])->scenario;
    const uint32_t capacity = fsim_scenario_vehicle_count(sc);
    uint32_t* ids = (uint32_t*)PyMem_Malloc(sizeof(uint32_t) * (capacity ? capacity : 1));
    if (!ids) return PyErr_NoMemory();
    size_t count = 0;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_scenario_apply(self->world, sc, ids, capacity, &count);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) {
        PyMem_Free(ids);
        return fail();
    }
    PyObject* bytes = PyBytes_FromStringAndSize((const char*)ids, (Py_ssize_t)(count * 4));
    PyMem_Free(ids);
    return bytes;
}

static PyObject* world_capsule(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "_capsule")) return NULL;
    return PyCapsule_New(&((WorldObject*)o)->ref, FSIM_PY_WORLD_CAPSULE, NULL);
}

static PyObject* world_borrowed_flag(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "borrowed")) return NULL;
    return PyBool_FromLong(((WorldObject*)o)->owner != NULL);
}

static PyMethodDef world_methods[] = {
    FAST("step", world_step, "step(n=1): advance every vehicle by n world steps"),
    FAST("info", world_info, "time, step seconds, vehicle steps, published, vehicle count"),
    FAST("time", world_time, "simulation seconds since creation"),
    FAST("create_vehicle", world_create_vehicle, "create_vehicle(spec: dict) -> id"),
    FAST("remove_vehicle", world_remove_vehicle, "remove_vehicle(id)"),
    FAST("reset_vehicle", world_reset_vehicle, "reset_vehicle(id, spec=None)"),
    FAST("find_vehicle", world_find_vehicle, "find_vehicle(name) -> id, 0 if none"),
    FAST("vehicle_ids", world_vehicle_ids, "every vehicle id, as uint32 bytes"),
    FAST("vehicle_info", world_vehicle_info, "vehicle_info(id) -> (name, type)"),
    FAST("state_buffer", world_state_buffer, "state_buffer(id, sensed=0): the live state struct, zero-copy"),
    FAST("gather_states", world_gather_states, "gather_states(ids, out, sensed=0)"),
    FAST("command", world_command, "command(id, level, values)"),
    FAST("command_batch", world_command_batch, "command_batch(level, ids, values)"),
    FAST("command_behavior", world_command_behavior, "command_behavior(id, behavior, target=0, params=None, points=None)"),
    FAST("submit", world_submit, "submit(id, level, values, source, axes, range, min_version) -> result"),
    FAST("submit_behavior", world_submit_behavior, "submit_behavior(id, behavior, target, params, points, source, axes, range, min_version) -> result"),
    FAST("submit_support", world_submit_support, "submit_support(id, kind, values, source, axes, range, min_version) -> result"),
    FAST("activity_update", world_activity_update, "activity_update(activity, values) -> result"),
    FAST("activity_update_batch", world_activity_update_batch, "activity_update_batch(activities uint64, values float64, stride)"),
    FAST("activity_cancel", world_activity_cancel, "activity_cancel(activity) -> result"),
    FAST("activity_info", world_activity_info, "activity_info(activity) -> info or None"),
    FAST("vehicle_activities", world_vehicle_activities, "vehicle_activities(id) -> [info]"),
    FAST("capabilities", world_capabilities, "capabilities(id) -> [capability]"),
    FAST("capability_status", world_capability_status, "capability_status(id, capability) -> (availability, reason)"),
    FAST("profile_value", world_profile_value, "profile_value(id, path) -> float, NaN if unknown"),
    FAST("profile_section", world_profile_section, "profile_section(id, section) -> (version, provenance)"),
    FAST("set_vehicle_default", world_set_vehicle_default, "set_vehicle_default(id, mode) -> reason, 0 if set"),
    FAST("set_protection", world_set_protection, "set_protection(id, mode)"),
    FAST("protection", world_protection, "protection(id) -> mode"),
    FAST("envelope", world_envelope, "envelope(id) -> (mode, [(limited_updates, exceeded_updates, exceeded_s, worst_excess)])"),
    FAST("vehicle_default", world_vehicle_default, "vehicle_default(id) -> mode"),
    FAST("active_level", world_active_level, "active_level(id)"),
    FAST("behavior_finished", world_behavior_finished, "behavior_finished(id)"),
    FAST("use_controller", world_use_controller, "use_controller(id, level, controller_id)"),
    FAST("set_controller_parameter", world_set_controller_parameter, "set_controller_parameter(id, level, name, value)"),
    FAST("controller_parameter", world_controller_parameter, "controller_parameter(id, level, name) -> float"),
    FAST("get_property", world_get_property, "get_property(id, path)"),
    FAST("set_property", world_set_property, "set_property(id, path, value)"),
    FAST("get_environment", world_get_environment, "the environment as a dict"),
    FAST("set_environment", world_set_environment, "set_environment(changes: dict)"),
    FAST("add_effect", world_add_effect, "add_effect(id or 0 for all, effect_id, params=None)"),
    FAST("clear_effects", world_clear_effects, "clear_effects(id)"),
    FAST("comm_create_node", world_comm_create_node, "comm_create_node(address)"),
    FAST("comm_send", world_comm_send, "comm_send(from, to, channel, format, data)"),
    FAST("comm_inbox", world_comm_inbox, "comm_inbox(node) -> messages"),
    FAST("comm_set_medium", world_comm_set_medium, "comm_set_medium(medium_id, params=None)"),
    FAST("comm_attach_protocol", world_comm_attach_protocol, "comm_attach_protocol(node, protocol_id, params=None)"),
    FAST("comm_attach_udp_bridge", world_comm_attach_udp_bridge, "comm_attach_udp_bridge(node, local_port, host, port)"),
    FAST("apply_scenario", world_apply_scenario, "apply_scenario(scenario) -> ids as uint32 bytes"),
    FAST("borrowed", world_borrowed_flag, "True for a VecEnv's world"),
    FAST("_capsule", world_capsule, "for fsim._vision"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot world_slots[] = {
    {Py_tp_new, (void*)world_new},
    {Py_tp_dealloc, (void*)world_dealloc},
    {Py_tp_methods, (void*)world_methods},
    {Py_tp_doc, (void*)"World(options: dict) over fsim_world (see fsim.World)"},
    {0, NULL}};
static PyType_Spec world_spec = {"fsim._native.World", sizeof(WorldObject), 0, Py_TPFLAGS_DEFAULT, world_slots};

/* ==========================================================================
 * Scenario
 * ======================================================================== */
/* Scenario(path) or Scenario(None, json, source_name) */
static PyObject* scenario_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject *path = Py_None, *json = Py_None;
    const char* source = "inline";
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "Scenario(path) or Scenario(None, json, source)");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "|OOs:Scenario", &path, &json, &source)) return NULL;
    fsim_scenario* sc = NULL;
    int rc;
    if (path != Py_None) {
        const char* p = as_str(path, "path");
        if (!p) return NULL;
        rc = fsim_scenario_load(p, &sc);
    } else {
        const char* text = as_str(json, "json");
        if (!text) return NULL;
        rc = fsim_scenario_parse(text, source, &sc);
    }
    if (rc != FSIM_OK) return fail();
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    ScenarioObject* self = (ScenarioObject*)alloc(type, 0);
    if (!self) {
        fsim_scenario_destroy(sc);
        return NULL;
    }
    self->scenario = sc;
    return (PyObject*)self;
}

static void scenario_dealloc(PyObject* o) {
    PyTypeObject* tp = Py_TYPE(o);
    fsim_scenario_destroy(((ScenarioObject*)o)->scenario);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

static PyObject* scenario_world_options(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "world_options")) return NULL;
    fsim_world_options wo;
    fsim_world_options_init(&wo);
    if (fsim_scenario_world_options(((ScenarioObject*)o)->scenario, &wo) != FSIM_OK) return fail();
    return fsim_py_get_fields(&wo, world_fields, COUNT(world_fields));
}

static PyObject* scenario_vehicle_count(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "vehicle_count")) return NULL;
    return PyLong_FromUnsignedLong(fsim_scenario_vehicle_count(((ScenarioObject*)o)->scenario));
}

static PyMethodDef scenario_methods[] = {
    FAST("world_options", scenario_world_options, "the scenario's world section as World options"),
    FAST("vehicle_count", scenario_vehicle_count, "vehicle instances, counting 'count'"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot scenario_slots[] = {
    {Py_tp_new, (void*)scenario_new},
    {Py_tp_dealloc, (void*)scenario_dealloc},
    {Py_tp_methods, (void*)scenario_methods},
    {Py_tp_doc, (void*)"Scenario(path) or Scenario(None, json, source) over fsim_scenario"},
    {0, NULL}};
static PyType_Spec scenario_spec = {"fsim._native.Scenario", sizeof(ScenarioObject), 0, Py_TPFLAGS_DEFAULT, scenario_slots};

/* ==========================================================================
 * Recording
 * ======================================================================== */
typedef struct {
    PyObject_HEAD
    fsim_recording* recording;
} RecordingObject;

static PyObject* recording_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    const char* path;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "Recording(path)");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "s:Recording", &path)) return NULL;
    fsim_recording* r = NULL;
    int rc;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_recording_load(path, &r);
    Py_END_ALLOW_THREADS
    if (rc != FSIM_OK) return fail();
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    RecordingObject* self = (RecordingObject*)alloc(type, 0);
    if (!self) {
        fsim_recording_destroy(r);
        return NULL;
    }
    self->recording = r;
    return (PyObject*)self;
}

static void recording_dealloc(PyObject* o) {
    PyTypeObject* tp = Py_TYPE(o);
    fsim_recording_destroy(((RecordingObject*)o)->recording);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

static PyObject* recording_info(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    const fsim_recording* r = ((RecordingObject*)o)->recording;
    (void)args;
    if (!check_args(n, 0, 0, "info")) return NULL;
    return Py_BuildValue("{s:I,s:s,s:d,s:i}", "frame_count", fsim_recording_frame_count(r), "world_name",
                         fsim_recording_world_name(r), "dt", fsim_recording_dt(r), "frame_skip", (int)fsim_recording_frame_skip(r));
}

static PyObject* recording_frame_time(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t frame;
    if (!check_args(n, 1, 1, "frame_time") || !as_u32(args[0], &frame)) return NULL;
    return PyFloat_FromDouble(fsim_recording_frame_time(((RecordingObject*)o)->recording, frame));
}

/* samples(frame): the frame's fsim_recorded_sample array, zero-copy. */
static PyObject* recording_samples(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t frame, count = 0;
    if (!check_args(n, 1, 1, "samples") || !as_u32(args[0], &frame)) return NULL;
    const fsim_recorded_sample* s = fsim_recording_samples(((RecordingObject*)o)->recording, frame, &count);
    if (!s || !count) return PyBytes_FromStringAndSize(NULL, 0);
    return fsim_py_buffer_new(BufferType, o, s, (Py_ssize_t)count * (Py_ssize_t)sizeof(fsim_recorded_sample), 1);
}

static PyObject* recording_events(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t frame, count = 0;
    if (!check_args(n, 1, 1, "events") || !as_u32(args[0], &frame)) return NULL;
    const fsim_recorded_event* e = fsim_recording_events(((RecordingObject*)o)->recording, frame, &count);
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        PyObject* d = Py_BuildValue(
            "{s:I,s:I,s:K,s:O,s:i,s:s,s:s,s:s,s:d,s:d,s:d,s:d}", "slot", e[i].slot, "id", e[i].id, "generation",
            (unsigned long long)e[i].generation, "alive", e[i].alive ? Py_True : Py_False, "control_level", (int)e[i].control_level,
            "name", e[i].name ? e[i].name : "", "type", e[i].type ? e[i].type : "", "model", e[i].model ? e[i].model : "",
            "initial_latitude_deg", e[i].initial_latitude_deg, "initial_longitude_deg", e[i].initial_longitude_deg,
            "initial_altitude_msl_m", e[i].initial_altitude_msl_m, "initial_heading_deg", e[i].initial_heading_deg);
        if (!d) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, d);
    }
    return list;
}

static PyMethodDef recording_methods[] = {
    FAST("info", recording_info, "frame count, world name, dt, frame skip"),
    FAST("frame_time", recording_frame_time, "frame_time(frame) -> seconds"),
    FAST("samples", recording_samples, "samples(frame): fsim_recorded_sample array, zero-copy"),
    FAST("events", recording_events, "events(frame) -> list of dicts"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot recording_slots[] = {
    {Py_tp_new, (void*)recording_new},
    {Py_tp_dealloc, (void*)recording_dealloc},
    {Py_tp_methods, (void*)recording_methods},
    {Py_tp_doc, (void*)"Recording(path) over fsim_recording"},
    {0, NULL}};
static PyType_Spec recording_spec = {"fsim._native.Recording", sizeof(RecordingObject), 0, Py_TPFLAGS_DEFAULT, recording_slots};

/* ==========================================================================
 * Module
 * ======================================================================== */
static PyObject* mod_version(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    (void)m;
    (void)args;
    if (!check_args(n, 0, 0, "version")) return NULL;
    return Py_BuildValue("(sI)", fsim_version(), fsim_abi_version());
}

static PyObject* mod_registered_ids(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int registry;
    (void)m;
    if (!check_args(n, 1, 1, "registered_ids") || !as_int(args[0], &registry)) return NULL;
    PyObject* list = PyList_New(0);
    if (!list) return NULL;
    for (uint32_t i = 0;; ++i) {
        const char* id = fsim_registered_id(registry, i);
        if (!id || !*id) break;
        PyObject* s = PyUnicode_FromString(id);
        if (!s || PyList_Append(list, s) < 0) {
            Py_XDECREF(s);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(s);
    }
    return list;
}

static PyObject* mod_reason_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "reason_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_reason_name(code));
}

static PyObject* mod_activity_state_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "activity_state_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_activity_state_name(code));
}

static PyObject* mod_limit_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "limit_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_limit_name(code));
}

static PyObject* mod_command_field_count(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int level;
    (void)m;
    if (!check_args(n, 1, 1, "command_field_count") || !as_int(args[0], &level)) return NULL;
    return PyLong_FromUnsignedLong(fsim_command_field_count(level));
}

/* layout(): the C structs' sizes and field offsets, so the Python mirrors can
 * be checked against the library they are about to read. */
#define OFS(T, f) #f, (unsigned long long)offsetof(T, f)
static PyObject* mod_layout(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    (void)m;
    (void)args;
    if (!check_args(n, 0, 0, "layout")) return NULL;
    PyObject* state = Py_BuildValue(
        "{s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K}", "size", (unsigned long long)sizeof(fsim_vehicle_state),
        OFS(fsim_vehicle_state, position_ecef), OFS(fsim_vehicle_state, latitude_rad), OFS(fsim_vehicle_state, euler_rad),
        OFS(fsim_vehicle_state, velocity_ned_ms), OFS(fsim_vehicle_state, airspeed_true_ms), OFS(fsim_vehicle_state, load_factor),
        OFS(fsim_vehicle_state, gear_position), OFS(fsim_vehicle_state, engine_count), OFS(fsim_vehicle_state, throttle_position),
        OFS(fsim_vehicle_state, fuel_kg), OFS(fsim_vehicle_state, step_count), OFS(fsim_vehicle_state, on_ground),
        OFS(fsim_vehicle_state, rotation_body_to_ecef), OFS(fsim_vehicle_state, engine_rpm),
        OFS(fsim_vehicle_state, afterburner), OFS(fsim_vehicle_state, leading_edge_flap_rad),
        OFS(fsim_vehicle_state, wheel_count), OFS(fsim_vehicle_state, wheel_speed_ms));
    PyObject* inputs = Py_BuildValue("{s:K,s:K,s:K}", "size", (unsigned long long)sizeof(fsim_control_inputs),
                                     OFS(fsim_control_inputs, throttle), OFS(fsim_control_inputs, brake_right));
    PyObject* sample = Py_BuildValue("{s:K,s:K,s:K}", "size", (unsigned long long)sizeof(fsim_recorded_sample),
                                     OFS(fsim_recorded_sample, state), OFS(fsim_recorded_sample, inputs));
    if (!state || !inputs || !sample) {
        Py_XDECREF(state);
        Py_XDECREF(inputs);
        Py_XDECREF(sample);
        return NULL;
    }
    return Py_BuildValue("{s:N,s:N,s:N}", "vehicle_state", state, "control_inputs", inputs, "recorded_sample", sample);
}
#undef OFS

static PyObject* mod_set_log_level(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int level;
    (void)m;
    if (!check_args(n, 1, 1, "set_log_level") || !as_int(args[0], &level)) return NULL;
    fsim_set_log_level(level);
    Py_RETURN_NONE;
}

static PyObject* mod_log_level(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    (void)m;
    (void)args;
    if (!check_args(n, 0, 0, "log_level")) return NULL;
    return PyLong_FromLong(fsim_log_level());
}

static PyMethodDef module_methods[] = {
    FAST("version", mod_version, "(library version, ABI version)"),
    FAST("set_log_level", mod_set_log_level, "set_log_level(0 trace .. 4 error, 5 off)"),
    FAST("log_level", mod_log_level, "the platform log's level"),
    FAST("registered_ids", mod_registered_ids, "registered_ids(REGISTRY_TASK | REGISTRY_OBSERVATION | REGISTRY_ACTION)"),
    FAST("command_field_count", mod_command_field_count, "command_field_count(level)"),
    FAST("reason_name", mod_reason_name, "reason_name(code): why a command was refused or an activity ended"),
    FAST("activity_state_name", mod_activity_state_name, "activity_state_name(code)"),
    FAST("limit_name", mod_limit_name, "limit_name(i): an envelope limit, \"load_factor_max\" ..."),
    FAST("layout", mod_layout, "C struct sizes and offsets"),
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef module = {PyModuleDef_HEAD_INIT, "fsim._native", "fsim's native core over the C ABI", -1,
                                    module_methods, NULL, NULL, NULL, NULL};

static int add_type(PyObject* m, PyType_Spec* spec, PyTypeObject** out, const char* name) {
    *out = (PyTypeObject*)PyType_FromSpec(spec);
    if (!*out) return -1;
    return PyModule_AddObjectRef(m, name, (PyObject*)*out);
}

PyMODINIT_FUNC PyInit__native(void) {
    PyObject* m = PyModule_Create(&module);
    if (!m) return NULL;
    Error = PyErr_NewExceptionWithDoc("fsim.Error", "A call into the platform failed; the message says why.", PyExc_RuntimeError, NULL);
    if (!Error || PyModule_AddObjectRef(m, "Error", Error) < 0 ||
        add_type(m, &fsim_py_buffer_spec, &BufferType, "Buffer") < 0 || add_type(m, &vecenv_spec, &VecEnvType, "VecEnv") < 0 ||
        add_type(m, &world_spec, &WorldType, "World") < 0 || add_type(m, &scenario_spec, &ScenarioType, "Scenario") < 0 ||
        add_type(m, &recording_spec, &RecordingType, "Recording") < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_ACTUATOR", FSIM_LEVEL_ACTUATOR) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_ATTITUDE", FSIM_LEVEL_ATTITUDE) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_ACCELERATION", FSIM_LEVEL_ACCELERATION) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_VELOCITY", FSIM_LEVEL_VELOCITY) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_POSITION", FSIM_LEVEL_POSITION) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_BEHAVIOR", FSIM_LEVEL_BEHAVIOR) < 0 ||
        PyModule_AddIntConstant(m, "AUTORESET_NEXT_STEP", FSIM_AUTORESET_NEXT_STEP) < 0 ||
        PyModule_AddIntConstant(m, "AUTORESET_SAME_STEP", FSIM_AUTORESET_SAME_STEP) < 0 ||
        PyModule_AddIntConstant(m, "REGISTRY_TASK", FSIM_REGISTRY_TASK) < 0 ||
        PyModule_AddIntConstant(m, "REGISTRY_OBSERVATION", FSIM_REGISTRY_OBSERVATION) < 0 ||
        PyModule_AddIntConstant(m, "REGISTRY_ACTION", FSIM_REGISTRY_ACTION) < 0) {
        Py_DECREF(m);
        return NULL;
    }
    PyObject* hold = PyFloat_FromDouble(fsim_hold());
    if (!hold || PyModule_AddObjectRef(m, "HOLD", hold) < 0) {
        Py_XDECREF(hold);
        Py_DECREF(m);
        return NULL;
    }
    Py_DECREF(hold);
    return m;
}
