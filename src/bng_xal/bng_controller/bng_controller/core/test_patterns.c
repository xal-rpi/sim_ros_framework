// controller_core_c.c
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <math.h>

static PyObject *compute_control_multi_test(PyObject *self, PyObject *args) {
  PyObject *sensor_data;
  double control_rate, max_latency;
  if (!PyArg_ParseTuple(args, "Odd", &sensor_data, &control_rate, &max_latency))
    return NULL;

  /* --- extract simtime and vel.x --- */
  PyObject *simtime_obj = PyDict_GetItemString(sensor_data, "simtime");
  if (!simtime_obj) {
    PyErr_SetString(PyExc_KeyError, "\"simtime\" not found");
    return NULL;
  }
  double simtime = PyFloat_AsDouble(simtime_obj);
  if (PyErr_Occurred())
    return NULL;

  PyObject *vel_dict = PyDict_GetItemString(sensor_data, "velocity");
  if (!vel_dict || !PyDict_Check(vel_dict)) {
    PyErr_SetString(PyExc_KeyError, "\"velocity\" missing or not a dict");
    return NULL;
  }
  PyObject *velx_obj = PyDict_GetItemString(vel_dict, "x");
  if (!velx_obj) {
    PyErr_SetString(PyExc_KeyError, "\"x\" not found in velocity dict");
    return NULL;
  }
  double velx = PyFloat_AsDouble(velx_obj);
  if (PyErr_Occurred())
    return NULL;

  /* --- fade max torque with speed --- */
  const double PI = 3.141592653589793;
  const double base_maxT = 2000.0;
  const double max_speed = 30.0;
  double frac = 1.0 - (velx / max_speed);
  if (frac < 0.0)
    frac = 0.0;
  double availT = base_maxT * frac;

  /* --- build a 60s repeating test waveform --- */
  double cycle = fmod(simtime, 60.0);
  double rawT;

  if (cycle < 15.0) {
    /* Step to 70% of available torque */
    rawT = 0.7 * availT;
  } else if (cycle < 30.0) {
    /* Linear ramp from 0 → +100% over 15s */
    double t = (cycle - 15.0) / 15.0;
    rawT = t * availT;
  } else if (cycle < 45.0) {
    /* Low‐frequency sine (0.2 Hz) */
    rawT = availT * sin(2.0 * PI * 0.2 * (cycle - 30.0));
  } else {
    /* Chirp: 0.1→1.0 Hz sweep over 15s */
    double tc = cycle - 45.0;
    const double Tdur = 15.0;
    const double f0 = 0.1, f1 = 1.0;
    double k = (f1 - f0) / Tdur;
    double phase = 2.0 * PI * (f0 * tc + 0.5 * k * tc * tc);
    rawT = availT * sin(phase);
  }

  /* Clamp into ±availT */
  if (rawT > availT)
    rawT = availT;
  if (rawT < -availT)
    rawT = -availT;

  /* Split into drive vs. brake */
  double wheel_torque = rawT > 0.0 ? rawT : 0.0;
  double brake_torque = rawT < 0.0 ? -rawT : 0.0;

  /* Build targets list */
  PyObject *targets_list = PyList_New(0);
  if (!targets_list)
    return NULL;

  double s_values[] = {0.0, 20.0, 40.0};
  int num_targets = sizeof(s_values) / sizeof(s_values[0]);

  /* Compute & clamp latency */
  double latency = max_latency + 0.005;
  if (latency > 0.1)
    latency = 0.1;
  double time_val = simtime + control_rate + latency;

  for (int i = 0; i < num_targets; ++i) {
    PyObject *target_dict = PyDict_New();
    if (!target_dict) {
      Py_DECREF(targets_list);
      return NULL;
    }
    PyDict_SetItemString(target_dict, "s", PyFloat_FromDouble(s_values[i]));
    PyDict_SetItemString(target_dict, "d", PyFloat_FromDouble(0.0));
    PyDict_SetItemString(target_dict, "phi", PyFloat_FromDouble(0.0));
    PyDict_SetItemString(target_dict, "wheel_torque",
                         PyFloat_FromDouble(wheel_torque));
    PyDict_SetItemString(target_dict, "brake_torque",
                         PyFloat_FromDouble(brake_torque));
    PyDict_SetItemString(target_dict, "road_wheel_angle",
                         PyFloat_FromDouble(0.0));

    if (PyList_Append(targets_list, target_dict) < 0) {
      Py_DECREF(target_dict);
      Py_DECREF(targets_list);
      return NULL;
    }
    Py_DECREF(target_dict); // PyList_Append increments ref count
  }

  /* Build result dict */
  PyObject *result = PyDict_New();
  if (!result) {
    Py_DECREF(targets_list);
    return NULL;
  }
  PyDict_SetItemString(result, "targets", targets_list);
  PyDict_SetItemString(result, "time", PyFloat_FromDouble(time_val));
  // Py_DECREF(targets_list); // PyDict_SetItemString does not steal a reference
  // here, but it's good practice to DECREF if it did. However, targets_list is
  // part of the result, so it should not be DECREF'd until result is.

  return result;
}

// Empty testing controller
static PyObject *compute_control_empty(PyObject *self, PyObject *args) {
  PyObject *sensor_data;
  double control_rate, max_latency;

  /* parse (PyObject *sensor_data, double control_rate, double max_latency) */
  if (!PyArg_ParseTuple(args, "Odd", &sensor_data, &control_rate, &max_latency))
    return NULL;

  /* extract simtime = sensor_data["simtime"] */
  PyObject *simtime_obj = PyDict_GetItemString(sensor_data, "simtime");
  if (!simtime_obj) {
    PyErr_SetString(PyExc_KeyError, "\"simtime\" not found in sensor_data");
    return NULL;
  }
  double simtime = PyFloat_AsDouble(simtime_obj);
  if (PyErr_Occurred())
    return NULL;

  /* build result dict */
  PyObject *result = PyDict_New();
  if (!result)
    return NULL;

  double latency = max_latency + 0.005;
  if (latency > 0.1)
    latency = 0.1;
  double time_val = simtime + control_rate + latency;

  PyDict_SetItemString(result, "time", PyFloat_FromDouble(time_val));

  return result;
}

static PyMethodDef TestPatternsMethods[] = {
    {"compute_control_multi_test", compute_control_multi_test, METH_VARARGS,
     "Generates a multi-test waveform based on input parameters."},
    {"compute_control_empty", compute_control_empty, METH_VARARGS,
     "Empty testing control"},
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef test_patterns_module = {
    PyModuleDef_HEAD_INIT, "bng_controller.core.test_patterns",
    "High‐level test controllers in C", -1, TestPatternsMethods};

PyMODINIT_FUNC PyInit_test_patterns(void) {
  return PyModule_Create(&test_patterns_module);
}
