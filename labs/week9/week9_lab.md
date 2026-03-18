# Week 9 — Frenet Frame Geometry

**Theme:** use the 2D Frenet coordinate frame from class to build reference paths from curvature, recover the corresponding 2D geometry, and validate Frenet coordinates against live vehicle data.

**Due Date:** 5 PM Today.

**Submission:** turn in your code and figures in one pdf/word document.

---

## Learning goals

By the end of this lab, you should be able to:

1. Define a reference path through a curvature description.
2. Recover a planar path from the Frenet-frame differential equations discussed in class.
3. Interpret the meaning of path progress, lateral offset, and heading error.
4. Compare a live vehicle pose against a reference path and decide whether your Frenet conversion is behaving correctly.
5. Reconstruct Frenet quantities from sparse waypoints

---


## Setup

```bash
cd ~/ros2_ws/src/sim_ros_framework
python -m pip install -r requirements.txt

git checkout hpa-s26
git pull origin hpa-s26

./luamod/build.bash --win --all
./xal_bng_ws_build.bash -w ~/ros2_ws --clean
source ~/ros2_ws/install/setup.bash
```

---

## Helper API You May Use

If you want to use the provided Week 9 helper file, the main functions and classes are in [labs/week9/frenet_helper.py](frenet_helper.py).

For Parts 1 and 2, you may use:
- `sample_curvature_function(s, kappa_of_s)`
- `integrate_frenet_path(s=..., kappa=..., x0=..., y0=..., psi0=...)`

For Part 3, you may use:
- `FrenetHelper(...)`
- `helper.get_latest_pose()`
- `helper.closest_reference_state(...)`
- `helper.report_frenet(...)`
- `helper.publish_reference_path_points(...)`

These helpers are optional. You may write your own code if you prefer.

---

## Review from class

Use the Frenet-frame path equations derived in lecture. In this lab, the independent variable is arc length.

You should integrate:

$$
\frac{dx}{ds} = \cos(\psi), \qquad
\frac{dy}{ds} = \sin(\psi), \qquad
\frac{d\psi}{ds} = \kappa(s)
$$

where:
- $s$ is arc length along the path
- $\psi$ is the path heading
- $\kappa(s)$ is the curvature profile you define from the geometry

Use the initial condition:

$$
(x(0), y(0), \psi(0)) = \left(0, 0, \frac{\pi}{2}\right)
$$

Interpret this initial pose carefully before you begin plotting.

### Integration helper

You may use the helper in [frenet_helper.py](frenet_helper.py) to perform the numerical integration step.

The helper does **not** define the path for you. You still need to:
- choose the total path length
- define the arc-length grid `s`
- define the curvature profile as a Python function of `s`
- evaluate that function on your `s` grid to obtain the curvature vector

**Recommended workflow**:

1. Build an arc-length vector `s` and specify the total path length and number of samples.
2. Write a scalar Python function `kappa_of_s(si)`.
3. Evaluate `kappa_of_s` at every sample in `s`.
4. Pass the resulting curvature vector to the integrator.

Before you show any integration results, save a screenshot of your `kappa_of_s(si)` function. That screenshot is part of the submission.

Helper import:

```python
from frenet_helper import integrate_frenet_path, sample_curvature_function
```

Generic usage pattern:

```python
import numpy as np

s_final = ... # Estimate the total path length from design geometry

def kappa_of_s(si: float) -> float:
  # return the curvature for one scalar value of arc length
  ...

s = np.linspace(0.0, s_final, num_samples)
kappa = sample_curvature_function(s, kappa_of_s)

path = integrate_frenet_path(
  s=s,
  kappa=kappa,
  x0=...,
  y0=...,
  psi0=...,
)

x = path.x
y = path.y
psi = path.yaw
```

If you do not want to use `sample_curvature_function`, the equivalent pattern is:

```python
kappa = np.array([kappa_of_s(si) for si in s], dtype=float)
```

You can add arrows to your 2D path plot to show the direction of travel. For example, using Matplotlib:

```python
def add_direction_arrows(ax, path, num_arrows=6, color="tab:blue"):
    if len(path.x) < 2:
        return

    arrow_indices = np.linspace(0, len(path.x) - 2, num_arrows, dtype=int)
    arrow_indices = np.unique(arrow_indices)

    for idx in arrow_indices:
        ax.annotate(
            "",
            xy=(path.x[idx + 1], path.y[idx + 1]),
            xytext=(path.x[idx], path.y[idx]),
            arrowprops={
                "arrowstyle": "->",
                "color": color,
                "lw": 1.8,
                "shrinkA": 0.0,
                "shrinkB": 0.0,
            },
        )
```

## Part 1 — Straight + Two Full Circles

### Path specification

Construct a path with the following geometry:

- straight segment length: 50 m
- turning radius: 30 m
- start pose: (0,0,π/2)

The path should contain:

1. a straight segment
2. two full circle turns of the specified radius in the same direction
3. another straight segment

### Your tasks

1. Clearly state the total path length and the number of samples you used.
2. Define an appropriate piecewise curvature function from the geometry above.
3. Integrate the Frenet-frame equations numerically to recover the path.
4. Plot the path in 2D.
5. Plot your curvature profile versus arc length.

### What to show on the plot

On your 2D path plot, mark:
- the start point
- the direction of travel at the start
- the end point

### Deliverables

Submit:
- a screenshot of your `kappa_of_s(si)` function for this path
- one 2D path plot and one plot of curvature versus arc length
---

## Part 2 — "Oval" Track

### Path specification

Use the same values:

- straight segment length: 50 m
- turning radius: 30 m

Now build an **"oval"** made from:

1. one straight segment
2. one half-circle turn
3. one straight segment
4. one half-circle turn

Start again from:

$$
(x(0), y(0), \psi(0)) = \left(0, 0, \frac{\pi}{2}\right)
$$

### Your tasks

1. Clearly state the total path length and the number of samples you used.
2. Define an appropriate piecewise curvature function from the geometry above.
3. Integrate the Frenet equations to recover the full 2D path.
4. Plot the "oval" in 2D.
5. Check whether your final pose is close to the initial pose, and comment on any mismatch.

### What to show on the plot

Show:
- the start point
- the end point
- the direction of travel at the start

### Deliverables

Submit:
- a screenshot of your `kappa_of_s(si)` function for the "oval" path
- one 2D "oval" plot and kappa versus arc length plot
- a short comment on whether your path closes as expected

---

## Part 3 — Live BeamNG Frenet Validation

### Goal

Use live vehicle pose data from BeamNG to test whether you know how to convert from world pose to Frenet quantities.

For this part, use the **"oval"** path geometry from Part 2.

### What you are validating

Using the notation from class, compute in real time:

- progress along the reference path
- lateral deviation from the reference path
- heading error relative to the reference path tangent

### BeamNG setup

Launch a scenario with enough open space. Replace the IPs as needed.

```bash
ros2 launch bng_bringup basic.launch.py \
  config:=gridworld.yaml \
  host:=<WINDOWS_BEAMNG_IP> \
  remote:=<WSL_IP>
```

Confirm that reduced state is available:

```bash
ros2 topic echo /EGO/reduced_state --once
```

This topic is the live source of vehicle pose for Part 3. Your code must either:
- subscribe to `/EGO/reduced_state` directly, or
- use the helper below, which already subscribes and caches the latest state

### Recommended workflow

1. Generate the "oval" reference so that it starts from the vehicle's initial pose.
2. Read the vehicle's current position and heading from `/EGO/reduced_state` and use it to regen the reference path if needed.
3. From the current vehicle position `(x, y)`, find the closest reference state
   $(s^*, x(s^*), y(s^*), \psi(s^*))$.
4. Use the formulas from class to compute lateral error and heading error.
5. Visualize the reference path in 2D while the vehicle is moving.
6. Drive around the reference path manually.
7. Display or log the live Frenet quantities in a way that lets you judge whether the conversion is correct.

### Utility helper provided

To reduce the geometry bookkeeping, you may use a helper that gives you the
closest reference state, but not the final Frenet conversion.

Import:

```python
from frenet_helper import FrenetHelper
```

Use the same in-memory path object that you built in Part 2. For example, if Part 2 produced:

```python
path = integrate_frenet_path(...)
```

then you can initialize the helper directly from that result:

```python
helper = FrenetHelper(
  vehicle_name="EGO",
  path=path,
  closed=True,
  spin_in_thread=True,
)
```

If you prefer to pass the arrays explicitly, that also works:

```python
helper = FrenetHelper(
  vehicle_name="EGO",
  s=path.s,
  x=path.x,
  y=path.y,
  yaw=path.yaw,
  kappa=path.kappa,
  closed=True,
  spin_in_thread=True,
)
```

The helper can then return:
- `s`
- `x_ref = x(s*)`
- `y_ref = y(s*)`
- `psi_ref = psi(s*)`
- projection distance

You should then compute your own:
- lateral error
- heading error

using the formulas from class.

### Live ROS + PlotJuggler helper provided

We provide a single helper that:
- subscribes to `/EGO/reduced_state`
- caches the latest pose
- publishes the reference path for PlotJuggler
- lets you publish your own computed `s`, `e`, and `dphi`
- accepts the arrays from your Part 1/2 path directly

So you can focus on the Frenet conversion and not the ROS plumbing.

```python
from frenet_helper import FrenetHelper
```

Typical setup:

```python
helper = FrenetHelper(
  vehicle_name="EGO",
  path=path,
  closed=True,
  spin_in_thread=True,
)
```

Then, inside your loop:

```python
pose = helper.get_latest_pose()
if pose is None:
    continue
```

and after you compute your own Frenet values:

This publishes a stamped `FrenetStateMsg` on `/<vehicle>/week9/frenet/state`, which you can inspect in PlotJuggler.

```python
helper.report_frenet(
    s=s_star,
    e=e,
    dphi=delta_psi,
    proj_dist=ref_state.proj_dist,
    x_ref=ref_state.x_ref,
    y_ref=ref_state.y_ref,
    psi_ref=ref_state.psi_ref,
)
```

### High-level loop skeleton

The following is the intended structure of your live computation loop. Fill in the missing pieces yourself.

```python
from frenet_helper import FrenetHelper

if not rclpy.ok():
    rclpy.init()

helper = FrenetHelper(
  vehicle_name="EGO",
  path=path,
  closed=True,
  spin_in_thread=True,
)

last_s = None

while True:
  pose = helper.get_latest_pose()
  if pose is None:
    continue

  ref_state = helper.closest_reference_state(
    x=pose.x,
    y=pose.y,
    last_s=last_s, # Help to narrow the search window (optional)
  )
  last_s = ref_state.s

  s_star = ref_state.s
  x_ref = ref_state.x_ref
  y_ref = ref_state.y_ref
  psi_ref = ref_state.psi_ref

  # TODO: compute e using the formula from class
  e = ...

  # TODO: compute Delta Heading using the formula from class
  delta_psi = ...

  # TODO: publish, print, plot, or log
  helper.report_frenet(
    ...
  )

  time.sleep(0.05) # Adjust the sleep time as needed to balance responsiveness and CPU load
```

### Suggested visualization choices


At minimum, you should be able to see:
- the reference path in the plane
- the vehicle trajectory in the plane
- the time history of your Frenet quantities

### Using PlotJuggler for the reference trajectory

PlotJuggler can be used to show the reference path geometry.

If you are unsure how to configure an XY plot in PlotJuggler, look at the PlotJuggler `Help` or `Cheatsheet` panel first. That is the fastest way to check how to make plots such as:
- `x` versus `y`
- `s` versus `x`
- `s` versus `y`
- `s` versus `yaw`
- `s` versus `kappa`

`FrenetHelper` publishes the reference path **point by point** as a stamped ROS message stream.

Order matters here. The reference path is not continuously latched in the background. You must actively publish the reference samples before PlotJuggler can draw them.

Use this sequence:
1. Start your ROS nodes and create `helper = FrenetHelper(...)`.
2. Open PlotJuggler and subscribe to the ROS topics.
3. Configure the XY plot using the fields from `/<vehicle>/week9/ref/sample`.
4. Then call `helper.publish_reference_path_points(...)`.

If you publish the reference path **before** PlotJuggler is connected, PlotJuggler may show nothing because it missed the streamed samples. In that case, call `helper.publish_reference_path_points(...)` again.

If you use:

```python
helper.publish_reference_path_points(sample_period=0.01, repeat=2)
```

then PlotJuggler can build an **XY plot** from the fields of:
- `/<vehicle>/week9/ref/sample`

This topic uses `PathSampleMsg`, which contains:
- `sample_index`
- `s`
- `x`
- `y`
- `yaw`
- `kappa`

You may also inspect:
- `x` versus `y`
- `x` versus `s`
- `y` versus `s`
- `yaw` versus `s`
- `kappa` versus `s`

Run that point-by-point publisher after PlotJuggler is already subscribed, so the samples are visible in the XY plot.

You can also overlay the driven vehicle path using the fields of:
 - `/<vehicle>/week9/frenet/state`

### Using PlotJuggler to plot versus arc length

PlotJuggler can plot signals against arc length if you use an **XY plot** instead of a time plot.

If you publish live Frenet state with:
- `helper.report_frenet(...)`

then you can create XY plots such as:
- `e` versus `s` from `/<vehicle>/week9/frenet/state`
- `dphi` versus `s` from `/<vehicle>/week9/frenet/state`

This is often more useful than a time plot when you want to compare behavior at different locations along the path rather than at different times.

### Questions to answer

While you drive, observe and answer:

1. When is your lateral deviation positive?
2. When is your heading error positive?
3. Does your progress variable evolve the way you expect when the car moves forward, stops, or reverses direction on the path?
4. Where does your conversion appear to break down or become noisy?
5. Explain in 2-3 sentences how helper.closest_reference_state works and how it uses `last_s` to speed up the search.

### Deliverables

Submit:
- one figure or screenshot showing the reference path and the driven path together
- one figure showing your live Frenet quantities over time
- a short qualitative discussion of whether the conversion matches what you learned in class

---

## Part 4 — Generate Smooth Trajectory and Reconstruct Frenet Path Offline

### Goal

Create a smooth trajectory of your own, record it, and then reconstruct a curvature-based Frenet path from that recording **offline**.

The main idea is to start from a driven XY trajectory, convert it into an arc-length description, estimate curvature from a smooth spline approximation, and then integrate that curvature profile to see whether you can recover the original path geometry.

### Required workflow

1. Drive a smooth trajectory in BeamNG.
2. Record the run.
3. Load the recorded trajectory offline.
4. Choose a starting point on that trajectory.
5. Downsample the trajectory to a fixed number of waypoints.
6. Build an arc-length coordinate `s` from the downsampled XY data.
7. Fit a smooth spline approximation to `x(s)` and `y(s)`.
8. Compute a curvature profile from the spline derivatives.
9. Integrate the recovered curvature profile to reconstruct the path.
10. Compare the reconstructed path against the original recorded trajectory.

### Logging

Start logging with:

```bash
ros2 run bng_simulator start_logs
```

### Expectations

Use one trajectory that is clearly smooth, for example:
- a large S-curve
- a smooth loop

Keep the path simple enough that you can interpret the geometry. Avoid stop-and-go motion and avoid sharp corners.

### Detailed offline procedure

After recording, your Part 4 pipeline should look like this:

1. Load the logged vehicle positions `(x_i, y_i)`.
2. Select one continuous portion of the run that you want to analyze.
3. Pick a starting index and reorder or crop the data so your reconstructed path begins there.
4. Downsample the selected path to a fixed number of waypoints.

Recommended: use something like 100-200 waypoints spaced along the path, not uniformly in time. The goal is to make the offline reconstruction easier to interpret and less noisy.

5. Build the cumulative arc-length coordinate from the downsampled points:

$$
s_0 = 0, \qquad
s_i = s_{i-1} + \sqrt{(x_i - x_{i-1})^2 + (y_i - y_{i-1})^2}
$$

6. Treat the downsampled path as a smooth planar curve `x(s), y(s)` and fit splines to both coordinates.
7. Evaluate the spline derivatives with respect to `s`.
8. Compute heading from the tangent direction:

$$
\psi(s) = \operatorname{atan2}\left(\frac{dy}{ds}, \frac{dx}{ds}\right)
$$

9. Compute curvature from the spline approximation using (general formula without assuming unit speed):

$$
\kappa(s) = \frac{x'(s) y''(s) - y'(s) x''(s)}{\left(x'(s)^2 + y'(s)^2\right)^{3/2}}
$$

Here, `x'(s)` and `y'(s)` are first derivatives of the spline, and `x''(s)` and `y''(s)` are second derivatives.

Use a spline tool directly rather than a Week 9 helper function. For example, SciPy gives you the right building blocks:

```python
from scipy.interpolate import CubicSpline

x_spline = CubicSpline(s_waypoints, x_waypoints)
y_spline = CubicSpline(s_waypoints, y_waypoints)

x_smooth = x_spline(s_dense)
y_smooth = y_spline(s_dense)

dx_ds = x_spline(s_dense, 1) # First derivative of x with respect to s

ddx_ds2 = x_spline(s_dense, 2) # Second derivative of x with respect to s

# Use these derivatives to compute psi(s) and kappa(s).
```

That is intentionally only a scaffold. You still need to:
- choose the waypoint set
- choose the evaluation grid `s_dense`
- compute `\psi(s)` from the first derivatives
- compute `\kappa(s)` from the curvature formula above
- integrate the recovered curvature profile yourself and compare it against the recorded path

10. Use the recovered curvature profile together with the selected starting pose

$$
(x(0), y(0), \psi(0))
$$

from your recorded trajectory to integrate the Frenet equations again.

11. Compare:
- the original recorded trajectory
- the downsampled trajectory
- the trajectory reconstructed by integrating the recovered curvature profile

If your spline and curvature calculation are working well, the reconstructed path should follow the original geometry closely, although it will not be exact.

### What to discuss

In your write-up, explain:
- how you chose the starting point
- how many waypoints you kept after downsampling
- whether your recovered curvature profile looks smooth or noisy
- where the reconstructed path matches the original path well
- where the reconstruction error becomes noticeable

### Deliverables

Submit:
- one XY plot showing all three of the following on the same axes:
  - the original recorded trajectory
  - the downsampled trajectory
  - the trajectory reconstructed from the recovered curvature profile
- one plot of curvature versus arc length, `\kappa(s)`
- a short paragraph describing your starting-point choice, your downsampling choice, and how well the reconstructed path matches the original one

---
