// SPDX-License-Identifier: LGPL-3.0-or-later
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <math.h>
#include "bridge.h"
#include "epd/wifi.h"
#include "epd/spatial.h"
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
static PyObject *dsss_symbols(PyObject *self, PyObject *args) {
    (void)self;
    Py_buffer input;
    unsigned kind, hint, length;
    int cfo;
    if (!PyArg_ParseTuple(args, "y*IIIi", &input, &kind, &hint, &length, &cfo)) return NULL;
    if (input.len < 4 || input.len > 16384*4 || input.len % 4) {
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
    unsigned words = input.len / 4, count, rate = 0;
    PyBuffer_Release(&input);
    float values[2048];
    Py_BEGIN_ALLOW_THREADS
    count = epd_wifi_dsss_symbols(iq, words, kind, hint, length, cfo, work, values, 1024, &rate);
    Py_END_ALLOW_THREADS
    PyMem_Free(iq); PyMem_Free(work);
    return Py_BuildValue("Iy#", count ? rate : 0, (char *)values, (Py_ssize_t)(count * 2 * sizeof(float)));
}
static PyObject *spatial_layout(PyObject *self, PyObject *args) {
  int phy, paths, ltf, gi;
  epd_spatial_plan p;
  if (!PyArg_ParseTuple(args, "iiii", &phy, &paths, &ltf, &gi))
    return NULL;
  if (epd_spatial_layout(phy, paths, ltf, gi, &p))
    return PyErr_Format(PyExc_ValueError,
                        "Unsupported spatial training configuration");
  return Py_BuildValue("(iiiiiii)", p.ltf_count, p.training_fft,
                       p.training_offset, p.training_stride, p.data_fft,
                       p.data_offset, p.data_stride);
}

static PyObject *spatial_decode(PyObject *self, PyObject *args) {
  int phy, paths, ltf, gi, stbc, nsym, stats[2];
  Py_buffer noise, spectra;
  epd_spatial_plan p;
  if (!PyArg_ParseTuple(args, "iiiiiiy*y*", &phy, &paths, &ltf, &gi, &stbc,
                        &nsym, &noise, &spectra))
    return NULL;
  PyObject *result = NULL, *channels = NULL, *symbols = NULL;
  Py_ssize_t nr = noise.len / sizeof(double);
  if (epd_spatial_layout(phy, paths, ltf, gi, &p) || nsym < 0 || nsym > 8 ||
      (stbc && (phy != 2 || paths != 2)) || nr < 1 || nr > INT_MAX ||
      noise.len % sizeof(double)) {
    PyErr_SetString(PyExc_ValueError, "Invalid spatial decoder dimensions");
    goto done;
  }
  Py_ssize_t row =
      sizeof(epd_complex) * (p.ltf_count * p.training_fft + nsym * p.data_fft);
  Py_ssize_t hrow = sizeof(epd_complex) * p.data_fft * paths;
  if (nr > PY_SSIZE_T_MAX / row || nr > PY_SSIZE_T_MAX / hrow ||
      spectra.len != nr * row || (uintptr_t)spectra.buf % sizeof(double) ||
      (uintptr_t)noise.buf % sizeof(double)) {
    PyErr_SetString(
        PyExc_ValueError,
        "Expected aligned, contiguous double-precision FFT and noise buffers");
    goto done;
  }
  for (Py_ssize_t i = 0; i < nr; i++) {
    double value = ((const double *)noise.buf)[i];
    if (!(value > 0) || !isfinite(value)) {
      PyErr_SetString(PyExc_ValueError, "Receiver noise power must be finite and positive");
      goto done;
    }
  }
  channels = PyBytes_FromStringAndSize(NULL, nr * hrow);
  symbols = PyBytes_FromStringAndSize(
      NULL, nsym * p.data_fft * (stbc ? 1 : paths) * sizeof(epd_complex));
  if (channels && symbols) {
    epd_complex *h = (epd_complex *)PyBytes_AS_STRING(channels);
    epd_complex *y = (epd_complex *)PyBytes_AS_STRING(symbols);
    int error;
    Py_BEGIN_ALLOW_THREADS
    error = epd_spatial_decode(
        &p, (int)nr, nsym, stbc, noise.buf, spectra.buf, h, y, stats);
    Py_END_ALLOW_THREADS
    if (error)
      PyErr_NoMemory();
    else
      result = Py_BuildValue("OOii", channels, symbols, stats[0], stats[1]);
  }
done:
  Py_XDECREF(channels);
  Py_XDECREF(symbols);
  PyBuffer_Release(&noise);
  PyBuffer_Release(&spectra);
  return result;
}
static PyMethodDef methods[] = {
    {"spatial_layout", spatial_layout, METH_VARARGS,
     "Training and payload FFT windows."},
    {"spatial_decode", spatial_decode, METH_VARARGS,
     "Estimate measured spatial channels and recover payload symbols."},
    {"dsss_symbols", dsss_symbols, METH_VARARGS,
     "Measured 11b constellation samples."},
    {"decode", decode, METH_VARARGS,
     "Decode packed IQ using the portable C core."},
    {NULL, NULL, 0, NULL}};
static struct PyModuleDef module = {PyModuleDef_HEAD_INIT, "_native", NULL, -1, methods};
PyMODINIT_FUNC PyInit__native(void) { return PyModule_Create(&module); }
