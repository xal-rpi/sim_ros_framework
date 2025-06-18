// test_wheel_speed.c
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <math.h>

static PyObject *compute_test_wheel_speed(PyObject *self, PyObject *args) {
  PyObject *sensor_data;
  double control_rate, max_latency;
  if (!PyArg_ParseTuple(args, "Odd", &sensor_data, &control_rate, &max_latency))
    return NULL;

  /* --- extract simtime --- */
  PyObject *simtime_obj = PyDict_GetItemString(sensor_data, "simtime");
  if (!simtime_obj) {
    PyErr_SetString(PyExc_KeyError, "\"simtime\" not found");
    return NULL;
  }
  double simtime = PyFloat_AsDouble(simtime_obj);
  if (PyErr_Occurred())
    return NULL;

  /* --- build a 60 s repeating test waveform of wheel_speed (rad/s) --- */
  const double PI = 3.141592653589793;
  const double base_max_speed = 80.0; /* 80 rad/s */
  double cycle = fmod(simtime, 60.0), speed_frac = 0.0;

  if (cycle < 15.0) {
    /* step to 70% */
    speed_frac = 0.7;
  } else if (cycle < 30.0) {
    /* linear ramp 0→100% */
    speed_frac = (cycle - 15.0) / 15.0;
  } else if (cycle < 45.0) {
    /* low‐freq sine 0.1 Hz mapped into [0,1] */
    double phi = 2.0 * PI * 0.1 * (cycle - 30.0);
    speed_frac = 0.3 * (1.0 + sin(phi));
  } else {
    /* 5-step staircase: 10,20,30,40,50 rad/s over 15 s */
    double tc = cycle - 45.0;
    const int steps = 5;
    double step_dur = 15.0 / steps; /* =3 s/step */
    int idx = (int)(tc / step_dur);
    if (idx < 0)
      idx = 0;
    if (idx >= steps)
      idx = steps - 1;
    double speed_val = 10.0 + 10.0 * idx; /* rad/s */
    /* frac in [0,1]: */
    speed_frac = speed_val / base_max_speed;
  }

  /* clamp into [0,1] */
  if (speed_frac < 0.0)
    speed_frac = 0.0;
  if (speed_frac > 1.0)
    speed_frac = 1.0;

  double wheel_speed = speed_frac * base_max_speed;

  /* --- build targets list (unchanged) --- */
  PyObject *targets_list = PyList_New(0);
  if (!targets_list)
    return NULL;

  double s_values[] = {0.0, 20.0, 40.0};
  int num_targets = sizeof(s_values) / sizeof(s_values[0]);

  /* compute & clamp latency */
  double latency = max_latency + 0.005;
  if (latency > 0.1)
    latency = 0.1;
  double time_val = simtime + control_rate + latency;

  for (int i = 0; i < num_targets; ++i) {
    PyObject *t = PyDict_New();
    if (!t) {
      Py_DECREF(targets_list);
      return NULL;
    }
    PyDict_SetItemString(t, "s", PyFloat_FromDouble(s_values[i]));
    PyDict_SetItemString(t, "wheel_speed", PyFloat_FromDouble(wheel_speed));
    PyDict_SetItemString(t, "x", PyFloat_FromDouble(s_values[i]));
    PyDict_SetItemString(t, "y", PyFloat_FromDouble(0.0));
    PyDict_SetItemString(t, "z", PyFloat_FromDouble(0.0));
    PyDict_SetItemString(t, "tx", PyFloat_FromDouble(1.0));
    PyDict_SetItemString(t, "ty", PyFloat_FromDouble(0.0));
    PyDict_SetItemString(t, "tz", PyFloat_FromDouble(0.0));
    if (PyList_Append(targets_list, t) < 0) {
      Py_DECREF(t);
      Py_DECREF(targets_list);
      return NULL;
    }
    Py_DECREF(t);
  }

  PyObject *result = PyDict_New();
  if (!result) {
    Py_DECREF(targets_list);
    return NULL;
  }
  PyDict_SetItemString(result, "targets", targets_list);
  PyDict_SetItemString(result, "time", PyFloat_FromDouble(time_val));

  return result;
}

static PyMethodDef TestPatternsMethods[] = {
    {"compute_test_wheel_speed", compute_test_wheel_speed, METH_VARARGS,
     "Generates a multi-test waveform based on input parameters."},
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef test_patterns_module = {
    PyModuleDef_HEAD_INIT, "bng_controller.core.test_wheel_speed",
    "High‐level test controllers in C", -1, TestPatternsMethods};

PyMODINIT_FUNC PyInit_test_wheel_speed(void) {
  return PyModule_Create(&test_patterns_module);
}
