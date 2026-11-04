// SPDX-License-Identifier: LGPL-3.0-or-later
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "bridge.h"
static PyObject *decode(PyObject *self, PyObject *args) {
    (void)self;
    Py_buffer input;
    int kind, a, b, c;
    if (!PyArg_ParseTuple(args, "y*iiii", &input, &kind, &a, &b, &c)) return NULL;
    if (input.len < 4 || input.len > 16384 * 4 || input.len % 4) {
        PyBuffer_Release(&input);
        PyErr_SetString(PyExc_ValueError, "expected 1..16384 packed uint32 IQ samples");
        return NULL;
    }
    uint32_t *iq = PyMem_Malloc(input.len);
    void *work = PyMem_Malloc(epd_workspace_size());
    if (!iq || !work) {
        PyMem_Free(iq); PyMem_Free(work); PyBuffer_Release(&input);
        return PyErr_NoMemory();
    }
    memcpy(iq, input.buf, input.len);
    unsigned words = input.len / 4;
    PyBuffer_Release(&input);
    uint32_t result[EPD_RESULT_WORDS];
    int valid;
    Py_BEGIN_ALLOW_THREADS
    valid = kind == 0 ? epd_decode_wifi(iq, words, a, b, work, result)
                     : epd_decode_ble(iq, words, a, b, c, work, result);
    Py_END_ALLOW_THREADS
    PyMem_Free(iq); PyMem_Free(work);
    if (!valid) Py_RETURN_NONE;
    return PyBytes_FromStringAndSize((const char *)result, sizeof(result));
}
static PyMethodDef methods[] = {{"decode", decode, METH_VARARGS, "Decode packed IQ using the portable C core."}, {NULL, NULL, 0, NULL}};
static struct PyModuleDef module = {PyModuleDef_HEAD_INIT, "_native", NULL, -1, methods};
PyMODINIT_FUNC PyInit__native(void) { return PyModule_Create(&module); }
