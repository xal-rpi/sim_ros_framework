// controller_core.c
#define PY_SSIZE_T_CLEAN
#include <Python.h>

// PID controller
static PyObject *compute_control_pid(PyObject *self, PyObject *args) {
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

  /* … PID logic … */
  double engine_torque = 100.0;
  double road_wheel_angle = 2.0;
  double brake_torque = 0.0;

  /* build result dict */
  PyObject *result = PyDict_New();
  if (!result)
    return NULL;

  PyDict_SetItemString(result, "engine_torque",
                       PyFloat_FromDouble(engine_torque));
  PyDict_SetItemString(result, "road_wheel_angle",
                       PyFloat_FromDouble(road_wheel_angle));
  PyDict_SetItemString(result, "brake_torque",
                       PyFloat_FromDouble(brake_torque));

  double latency = max_latency + 0.005;
  if (latency > 0.1)
    latency = 0.1;
  double time_val = simtime + control_rate + latency;

  PyDict_SetItemString(result, "time", PyFloat_FromDouble(time_val));

  return result;
}

// MPC controller
static PyObject *compute_control_mpc(PyObject *self, PyObject *args) {
  PyObject *sensor_data;
  if (!PyArg_ParseTuple(args, "O", &sensor_data))
    return NULL;

  // … mpc logic …

  double engine_torque = 100.0;
  double road_wheel_angle = 2.0;
  double brake_torque = 0.0;

  PyObject *result = PyDict_New();
  if (!result)
    return NULL;
  PyDict_SetItemString(result, "engine_torque",
                       PyFloat_FromDouble(engine_torque));
  PyDict_SetItemString(result, "road_wheel_angle",
                       PyFloat_FromDouble(road_wheel_angle));
  PyDict_SetItemString(result, "brake_torque",
                       PyFloat_FromDouble(brake_torque));
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


static PyMethodDef ControllerCoreMethods[] = {
    {"compute_control_pid", compute_control_pid, METH_VARARGS,
     "PID‐based control"},
    {"compute_control_mpc", compute_control_mpc, METH_VARARGS,
     "MPC‐based control"},
    {"compute_control_empty", compute_control_empty, METH_VARARGS,
     "Empty testing control"},
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef controller_core_module = {
    PyModuleDef_HEAD_INIT, "bng_controller.core.controller_core",
    "High‐level controllers in C", -1, ControllerCoreMethods};

PyMODINIT_FUNC PyInit_controller_core(void) {
  return PyModule_Create(&controller_core_module);
}
