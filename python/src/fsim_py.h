/* Shared by the Python SDK's native modules (fsim._native, fsim._vision).
 *
 * Both are built against CPython's stable ABI (abi3, Python 3.11 and later)
 * so one binary serves every interpreter from 3.11 on, and both are plain C
 * over the C ABI: no C++ crosses into Python, and no numpy headers are
 * needed - memory is handed out through the buffer protocol and viewed by
 * numpy without a copy. */
#ifndef FSIM_PY_H
#define FSIM_PY_H

#ifndef Py_LIMITED_API
#    define Py_LIMITED_API 0x030B0000
#endif
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* What fsim._native hands fsim._vision through a capsule: the handle, and the
 * flag that says a call on it is running without the GIL. A camera render
 * and a world step on two threads must not overlap, so each marks the flag
 * of the world it touches. */
typedef struct fsim_py_ref {
    void* handle; /* fsim_world* or fsim_vecenv* */
    int* busy;
} fsim_py_ref;
#define FSIM_PY_WORLD_CAPSULE "fsim._native.world"
#define FSIM_PY_VECENV_CAPSULE "fsim._native.vecenv"

/* A call that runs without the GIL marks its object busy for the duration;
 * any other call on that object meanwhile - from another Python thread -
 * gets an error instead of a data race. Checked with the GIL held, so the
 * flag itself needs no lock. */
static inline int fsim_py_idle(const int* busy, const char* what) {
    if (busy && *busy) {
        PyErr_Format(PyExc_RuntimeError, "%s is in use by another thread (one call at a time per object)", what);
        return 0;
    }
    return 1;
}

/* --------------------------------------------------------------------------
 * Buffer: memory owned by the platform, exported through the buffer
 * protocol. It holds a reference to the object that owns the memory, and
 * every numpy array viewing it holds a reference to it, so no view can
 * outlive the memory it looks at.
 * ------------------------------------------------------------------------ */
typedef struct {
    PyObject_HEAD
    PyObject* owner;
    void* data;
    Py_ssize_t len;
    int readonly;
} fsim_py_buffer;

static inline int fsim_py_buffer_get(PyObject* self, Py_buffer* view, int flags) {
    fsim_py_buffer* b = (fsim_py_buffer*)self;
    if ((flags & PyBUF_WRITABLE) && b->readonly) {
        PyErr_SetString(PyExc_BufferError, "this memory is read-only");
        view->obj = NULL;
        return -1;
    }
    view->obj = Py_NewRef(self);
    view->buf = b->data;
    view->len = b->len;
    view->readonly = b->readonly;
    view->itemsize = 1;
    view->format = (flags & PyBUF_FORMAT) ? (char*)"B" : NULL;
    view->ndim = 1;
    view->shape = (flags & PyBUF_ND) ? &b->len : NULL;
    view->strides = (flags & PyBUF_STRIDES) ? &view->itemsize : NULL;
    view->suboffsets = NULL;
    view->internal = NULL;
    return 0;
}

static inline void fsim_py_buffer_release(PyObject* self, Py_buffer* view) {
    (void)self;
    (void)view;
}

static inline void fsim_py_buffer_dealloc(PyObject* self) {
    PyTypeObject* tp = Py_TYPE(self);
    Py_CLEAR(((fsim_py_buffer*)self)->owner);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(self);
    Py_DECREF(tp);
}

static PyType_Slot fsim_py_buffer_slots[] = {
    {Py_tp_dealloc, (void*)fsim_py_buffer_dealloc},
    {Py_bf_getbuffer, (void*)fsim_py_buffer_get},
    {Py_bf_releasebuffer, (void*)fsim_py_buffer_release},
    {Py_tp_doc, (void*)"Platform-owned memory, exported without a copy; keeps its owner alive."},
    {0, NULL}};

static PyType_Spec fsim_py_buffer_spec = {"fsim._Buffer", sizeof(fsim_py_buffer), 0, Py_TPFLAGS_DEFAULT, fsim_py_buffer_slots};

/* A new buffer over [data, data + len) owned by `owner`; NULL data gives None. */
static inline PyObject* fsim_py_buffer_new(PyTypeObject* type, PyObject* owner, const void* data, Py_ssize_t len, int readonly) {
    if (!data) Py_RETURN_NONE;
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    fsim_py_buffer* b = (fsim_py_buffer*)alloc(type, 0);
    if (!b) return NULL;
    b->owner = Py_NewRef(owner);
    b->data = (void*)data;
    b->len = len;
    b->readonly = readonly;
    return (PyObject*)b;
}

/* --------------------------------------------------------------------------
 * Options: a Python dict onto one of the C ABI's option structs, field by
 * field from a table, so a misspelt key is an error rather than a default.
 * ------------------------------------------------------------------------ */
typedef enum { FSIM_PY_U32, FSIM_PY_I32, FSIM_PY_U64, FSIM_PY_F64, FSIM_PY_STR } fsim_py_kind;
typedef struct {
    const char* name;
    fsim_py_kind kind;
    size_t offset;
} fsim_py_field;
#define FSIM_PY_FIELD(T, f, k) {#f, k, offsetof(T, f)}

/* Strings point into the dict's str objects: valid while the dict lives. */
static inline int fsim_py_set_fields(PyObject* dict, void* target, const fsim_py_field* fields, size_t count, const char* what) {
    if (!dict || dict == Py_None) return 0;
    if (!PyDict_Check(dict)) {
        PyErr_Format(PyExc_TypeError, "%s options must be a dict", what);
        return -1;
    }
    PyObject *key, *value;
    Py_ssize_t pos = 0;
    while (PyDict_Next(dict, &pos, &key, &value)) {
        const char* name = PyUnicode_Check(key) ? PyUnicode_AsUTF8AndSize(key, NULL) : NULL;
        if (!name) {
            PyErr_Format(PyExc_TypeError, "%s option names must be strings", what);
            return -1;
        }
        const fsim_py_field* f = NULL;
        for (size_t i = 0; i < count; ++i)
            if (strcmp(fields[i].name, name) == 0) f = &fields[i];
        if (!f) {
            PyErr_Format(PyExc_TypeError, "unknown %s option '%s'", what, name);
            return -1;
        }
        char* at = (char*)target + f->offset;
        switch (f->kind) {
        case FSIM_PY_U32: {
            const unsigned long v = PyLong_AsUnsignedLong(value);
            if (v == (unsigned long)-1 && PyErr_Occurred()) return -1;
            *(uint32_t*)at = (uint32_t)v;
            break;
        }
        case FSIM_PY_I32: {
            const long v = PyLong_AsLong(value);
            if (v == -1 && PyErr_Occurred()) return -1;
            *(int32_t*)at = (int32_t)v;
            break;
        }
        case FSIM_PY_U64: {
            const unsigned long long v = PyLong_AsUnsignedLongLong(value);
            if (v == (unsigned long long)-1 && PyErr_Occurred()) return -1;
            *(uint64_t*)at = (uint64_t)v;
            break;
        }
        case FSIM_PY_F64: {
            const double v = PyFloat_AsDouble(value);
            if (v == -1.0 && PyErr_Occurred()) return -1;
            *(double*)at = v;
            break;
        }
        case FSIM_PY_STR:
            if (value == Py_None) {
                *(const char**)at = NULL;
            } else {
                const char* s = PyUnicode_Check(value) ? PyUnicode_AsUTF8AndSize(value, NULL) : NULL;
                if (!s) {
                    PyErr_Format(PyExc_TypeError, "%s option '%s' must be a string", what, name);
                    return -1;
                }
                *(const char**)at = s;
            }
            break;
        }
    }
    return 0;
}

static inline PyObject* fsim_py_get_fields(const void* source, const fsim_py_field* fields, size_t count) {
    PyObject* d = PyDict_New();
    if (!d) return NULL;
    for (size_t i = 0; i < count; ++i) {
        const char* at = (const char*)source + fields[i].offset;
        PyObject* v = NULL;
        switch (fields[i].kind) {
        case FSIM_PY_U32: v = PyLong_FromUnsignedLong(*(const uint32_t*)at); break;
        case FSIM_PY_I32: v = PyLong_FromLong(*(const int32_t*)at); break;
        case FSIM_PY_U64: v = PyLong_FromUnsignedLongLong(*(const uint64_t*)at); break;
        case FSIM_PY_F64: v = PyFloat_FromDouble(*(const double*)at); break;
        case FSIM_PY_STR: {
            const char* s = *(const char* const*)at;
            v = s ? PyUnicode_FromString(s) : Py_NewRef(Py_None);
            break;
        }
        }
        if (!v || PyDict_SetItemString(d, fields[i].name, v) < 0) {
            Py_XDECREF(v);
            Py_DECREF(d);
            return NULL;
        }
        Py_DECREF(v);
    }
    return d;
}

/* Named parameters (effects, media, protocols, behaviours): a dict of
 * str -> float as the C ABI's parallel arrays. Freed with fsim_py_params_free. */
typedef struct {
    const char** names;
    double* values;
    uint32_t count;
} fsim_py_params;

static inline int fsim_py_params_read(PyObject* dict, fsim_py_params* out) {
    out->names = NULL;
    out->values = NULL;
    out->count = 0;
    if (!dict || dict == Py_None) return 0;
    if (!PyDict_Check(dict)) {
        PyErr_SetString(PyExc_TypeError, "parameters must be a dict of name -> number");
        return -1;
    }
    const Py_ssize_t n = PyDict_Size(dict);
    if (n == 0) return 0;
    out->names = (const char**)PyMem_Malloc(sizeof(const char*) * (size_t)n);
    out->values = (double*)PyMem_Malloc(sizeof(double) * (size_t)n);
    if (!out->names || !out->values) {
        PyErr_NoMemory();
        return -1;
    }
    PyObject *key, *value;
    Py_ssize_t pos = 0;
    while (PyDict_Next(dict, &pos, &key, &value)) {
        const char* name = PyUnicode_Check(key) ? PyUnicode_AsUTF8AndSize(key, NULL) : NULL;
        if (!name) {
            PyErr_SetString(PyExc_TypeError, "parameter names must be strings");
            return -1;
        }
        const double v = PyFloat_AsDouble(value);
        if (v == -1.0 && PyErr_Occurred()) return -1;
        out->names[out->count] = name;
        out->values[out->count] = v;
        ++out->count;
    }
    return 0;
}

static inline void fsim_py_params_free(fsim_py_params* p) {
    PyMem_Free((void*)p->names);
    PyMem_Free(p->values);
    p->names = NULL;
    p->values = NULL;
}

#endif /* FSIM_PY_H */
