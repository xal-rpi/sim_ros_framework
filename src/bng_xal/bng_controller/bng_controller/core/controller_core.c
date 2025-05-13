// controller_core.c
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <math.h>

static PyObject *compute_control_pid(PyObject *self, PyObject *args) {
  PyObject *sensor_data;
  double control_rate, max_latency;

  /* parse args */
  if (!PyArg_ParseTuple(args, "Odd", &sensor_data, &control_rate, &max_latency))
    return NULL;

  /* extract simtime */
  PyObject *simtime_obj = PyDict_GetItemString(sensor_data, "simtime");
  if (!simtime_obj) {
    PyErr_SetString(PyExc_KeyError, "\"simtime\" not found");
    return NULL;
  }
  double simtime = PyFloat_AsDouble(simtime_obj);
  if (PyErr_Occurred())
    return NULL;

  /* extract velocity.x */
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

  /* --- generate a sine‐wave wheel torque whose amplitude
        falls to zero at max_speed --- */
  const double PI = 3.141592653589793;
  const double freq = 0.05;        /* Hz of sine wave */
  const double base_maxT = 2000.0; /* max wheel torque at 0 m/s  */
  const double max_speed = 30.0;   /* torque→0 by 30 m/s */
  const double dc_off = 0.5;       /* +0.5 shifts up 50% of availT */

  /* 1) linearly fade max torque with speed */
  double frac = 1.0 - (velx / max_speed);
  if (frac < 0.0)
    frac = 0.0;
  double availT = base_maxT * frac;

  /* 2) raw shifted sine */
  double S = sin(2.0 * PI * freq * simtime);
  double rawT = availT * (S + dc_off);

  /* 3) clamp into ±availT */
  if (rawT > availT)
    rawT = availT;
  if (rawT < -availT)
    rawT = -availT;

  /* 4) split into drive vs. brake */
  double wheel_torque = rawT > 0.0 ? rawT : 0.0;
  double brake_torque = rawT < 0.0 ? -rawT : 0.0;

  /* you can still compute road_wheel_angle, etc. */
  double road_wheel_angle = 0.0;

  /* build result dict */
  PyObject *result = PyDict_New();
  if (!result)
    return NULL;

  PyDict_SetItemString(result, "wheel_torque",
                       PyFloat_FromDouble(wheel_torque));
  PyDict_SetItemString(result, "road_wheel_angle",
                       PyFloat_FromDouble(road_wheel_angle));
  PyDict_SetItemString(result, "brake_torque",
                       PyFloat_FromDouble(brake_torque));

  /* compute and clamp latency */
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

  double wheel_torque = 100.0;
  double road_wheel_angle = 2.0;
  double brake_torque = 0.0;

  PyObject *result = PyDict_New();
  if (!result)
    return NULL;
  PyDict_SetItemString(result, "wheel_torque",
                       PyFloat_FromDouble(wheel_torque));
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
