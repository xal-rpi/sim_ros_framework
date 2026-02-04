# Lab 4 — Pure Pursuit (Week 4)

**Theme:** use Week 3 data + tooling to build a simple geometric path follower (pure pursuit), then validate it offline and live.

**Due next week:** everything from today’s lab (code + plots + short write-up).

---

## Check — Update + rebuild pipeline (WSL)

This is the same workflow as Week 3.

### 1) Checkout the correct branch and pull

```bash
cd ~/ros2_ws/src/sim_ros_framework

git checkout .
# If the course branch is hpa-s26:
git checkout hpa-s26
git pull origin hpa-s26
```

### 2) Install additional Python dependencies (for Week 4 GUI helpers)

```bash
python3 -m pip install PySide6 pyqtgraph
```

If your environment needs it, you can add `--user` (or use your venv/conda env).

### 3) Update the BeamNG Lua mod

```bash
cd ~/ros2_ws/src/sim_ros_framework
./luamod/build.bash --win --all
```

### 4) Rebuild the ROS2 overlay (clean)

```bash
cd ~/ros2_ws/src/sim_ros_framework
./xal_bng_ws_build.bash -w ~/ros2_ws -r jazzy --clean

# For this terminal only (new terminals usually source automatically):
source ~/ros2_ws/install/setup.bash
```

---

## Check — Launch the simulator (the `remote:=` fix)

Last week’s most common failure mode was: **BeamNG was running, but data/control were flaky or missing** because the in-sim controller/telemetry tried to send UDP packets to the wrong IP.

In this stack:
- `host:=...` = **BeamNG (Windows) IP** that WSL connects *to*.
- `remote:=...` = **WSL/Linux IP** that BeamNG should send packets *back to* (`sendIp`).

### 1) Get the two IP addresses

- **Windows / BeamNG IP**: from Windows `ipconfig` (the adapter on the same network as WSL can reach).

- **WSL IP** (run in WSL): ifconfig or:

```bash
ip route get 1.1.1.1 | awk '{print $7; exit}'
```

### 2) Launch (same as Week 3, but include `remote:=`)

Example (replace the correct IPs):

```bash
ros2 launch bng_bringup basic.launch.py \
	host:=<WINDOWS_BEAMNG_IP> \
	remote:=<WSL_IP>
```


### 3) Quick sanity checks

In another WSL terminal:

```bash
ros2 topic list | grep EGO
```

Optional live plotting:

```bash
ros2 run plotjuggler plotjuggler
```

Plot at least:
- `/EGO/reduced_state/*`

---

## Q3 — Calibrate steering: `steeringInput` ↔ road-wheel angle `delta`

Before you implement pure pursuit, you need a conversion between:
- **`steeringInput`** (dimensionless command, typically in [-1, 1])
- **front road-wheel steer angle** $\delta$ in **radians** (what the bicycle model uses)

You will estimate a simple linear model:

$$\texttt{steeringInput} \approx m\,\delta + b$$

and the inverse mapping:

$$\delta \approx \frac{\texttt{steeringInput} - b}{m}$$

### 1) Record a calibration run

Goal: capture a time segment where you **turn the steering wheel back and forth several times** (multi-turn sweep), preferably while moving slowly (or even nearly stopped) so the steering angle spans most of its range.

1) Start BeamNG + launch ROS2 as in Q2
2) Start the interactive logger:

```bash
ros2 run bng_simulator start_logs
```

3) During the run: do several smooth left-right sweeps of steering (avoid crashing)
4) Stop logging and note the run number (e.g. `run_006`)

### 2) Load the run and access the fields

Using the same analysis workflow as last week (your Week 3 notebook / script that loads run logs), load the run with `load_run_data(...)` and access `/EGO/gtstate`.

Fields you must use:
- `/EGO/gtstate['steeringInput']`  → steering input
- `/EGO/gtstate['wheelFR_angleAtan2']` and `/EGO/gtstate['wheelFL_angleAtan2']` → front wheel steer angles (radians)

Define the **road-wheel steer angle** as the average of left/right front wheels:

$$\delta = \tfrac{1}{2}(\delta_{FR} + \delta_{FL})$$

### 3) Fit a line (polyfit degree 1)

You must extract these arrays (exact field names):

```python
steering_input = np.asarray(gt["steeringInput"], dtype=float)
delta_fr = np.asarray(gt["wheelFR_angleAtan2"], dtype=float)
delta_fl = np.asarray(gt["wheelFL_angleAtan2"], dtype=float)
```

Then compute $\delta = 0.5(\delta_{FR}+\delta_{FL})$ and fit a linear model using `np.polyfit(..., 1)`:

- Fit: `steeringInput ≈ m * delta + b`
- Inverse: `delta ≈ (steeringInput - b) / m`

Deliverable for this part:
- A plot showing the regression quality:
	- Scatter: `steeringInput` vs `delta` with the best-fit line overlaid
	- And/or a prediction check: plot `steeringInput_measured` vs `steeringInput_predicted` (should lie close to the 45° line)
- Report the fitted values `m` and `b` and the final conversion you will use.

---

## Q4 — Start `gridworld.yaml` and validate the pure-rolling kinematics

In class we derived (under a **pure rolling / no lateral slip** bicycle model):

$$r \approx \frac{v_x}{a+b}\tan(\delta)$$

and the lateral-velocity relationships:

$$v_y \approx b * r$$

and (front-wheel point velocity):

$$v_y + a\,r \approx v_x\tan(\delta)$$

Your job is to generate a simple speed + steering excitation, log the data, and check how well these approximations hold.

### 1) Launch the scenario

```bash
ros2 launch bng_bringup basic.launch.py config:=gridworld.yaml host:=172.19.208.1 remote:=172.19.215.209
```

### 2) Drive a repeatable excitation (student controller script)

Write a small ROS2 Python script (same pattern as Week 3) that:
- Runs a fixed-rate loop (recommend **50 Hz**)
- Commands a **velocity profile** $v_x^{cmd}(t)$ with a few plateaus (or constant speed). Pick values that are safe in `gridworld.yaml`.
- Commands a **steering profile** (e.g., sinusoid) that sweeps left/right for multiple periods

Guidance (skeleton only — choose the numerical values yourself):

```python
def velocity_profile(t: float) -> float:
	# TODO: piecewise constant schedule or constant speed
	...

def steering_profile(t: float) -> float:
	# TODO: sinusoid within allowed steeringInput range
	# Example structure: amp * sin(2*pi*t/T)
	...

HZ = ...
DT = 1.0 / HZ
T_END = ...

elapsed = 0.0
while rclpy.ok() and elapsed <= T_END:
	target_speed = velocity_profile(elapsed)
	steering_input = steering_profile(elapsed)

	# IMPORTANT: send both commands (don’t leave steering=None)
	ctl.send_command(vehicle_speed=target_speed, steering=steering_input)

	elapsed += DT
	time.sleep(DT)
```

### 3) Record the run

Use the interactive logger and keep track of the run number:

```bash
ros2 run bng_simulator start_logs
```

### 4) Offline validation (what to compute and plot)

From the logged `/EGO/gtstate`, extract:

```python
vy = np.array(gt_data['vel_y'])  # m/s
vx = np.array(gt_data['vel_x'])  # m/s
r  = np.array(gt_data['angVel_z'])  # rad/s
```

You also need the road-wheel steer angle $\delta$ (in radians). Use your calibration from Q3 to convert logged `steeringInput` to $\delta$.

Determine the vehicle geometry parameters for the vehicle you are running:

- $a$ = CG-to-front-axle distance (m)
- $b$ = CG-to-rear-axle distance (m)
- $L = a + b$ = wheelbase (m)

Where to find them (typical):

- Vehicle config YAML in `src/bng_xal/bng_bringup/config/vehicles/` (look for keys like `cogToFrontAxle`, `cogToRearAxle`, and/or `distLR`).

Deliverable: state the file path + the values you used for $a$ and $b$.

Now form the kinematic predictions:
- $r_{est} = \frac{v_x}{a+b}\tan(\delta)$
- $v_{y,est} = b*r$
- $v_{fy,est} = v_x\tan(\delta)$ and compare it to $v_{fy} = v_y + a\,r$

Deliverables for this part:
- Plot 1: $r(t)$ and $r_{est}(t)$ over the same time axis
- Plot 2: $v_y(t)$ and $b\,r(t)$ over the same time axis
- Plot 3 (optional but recommended): $(v_y + a r)(t)$ and $(v_x\tan\delta)(t)$
- Repeat Plot 1 (or report a summary error metric) for **at least two different speed targets** (e.g., a “slow” run and a “faster” run) and briefly compare the mismatch.

If you observe mismatches, you must:

1) **Explain why** (3–6 sentences). Typical causes: tire slip at higher $|\delta|$ and/or higher speed, transient effects during fast steering, or any understeer/oversteer dynamics not captured by the kinematic model.

2) **Estimate the maximum steering angle magnitude** $\delta_{max}$ such that the pure-rolling approximation is “good enough”.

Deliverable: report your chosen $\delta_{max}$ (in radians and degrees) and show a plot that supports it.

---

## Q5 — Pure Pursuit controller (online)

Now you will use the reference trajectory + lookahead target point to compute a steering command and drive the car along the path.

### 1) Launch the pure pursuit scenario

For this question, run the scenario config:

```bash
ros2 launch bng_bringup basic.launch.py \
	config:=pure_pursuit.yaml \
	host:=<WINDOWS_BEAMNG_IP> \
	remote:=<WSL_IP>
```

### What is given to you (conceptually)

Assume you have a helper object that:

1) Loads a **reference trajectory** from a CSV file (example: `labs/week4/ref.csv`). The CSV contains at least `x,y` (and may include `s,yaw,curvature`).
2) Subscribes to the live vehicle state and continuously computes a **lookahead target** on the reference path.
3) Lets you query a single “latest sample” that includes:

- `x, y, yaw`: current vehicle pose in the map/world frame
- `V`: current speed
- `s`: current progress along the reference path (arc-length)
- `s_target`: lookahead progress (typically `s + Ld`)
- `x_target, y_target, yaw_target`: lookahead target point on the reference

And it provides a method to send commands to the vehicle:

- send `V_target` (speed target)
- send `delta_target` (road-wheel steering angle target, radians)

### Your task

Design the **pure pursuit steering** part (from class) that computes `delta_target` from:
- current pose `(x, y, yaw)`
- lookahead target point `(x_target, y_target)`
- lookahead distance `Ld`
- wheelbase `L = a + b`

Do **not** use “magic numbers”: use your calibrated steering conversion from Q3 and the geometry `a,b` you identified in Q4.

You must **experiment with different speed targets** (at least 2) and describe how tracking quality changes with speed.

### Pseudocode (Python-shaped skeleton, algorithm masked)

This is the *shape* of the script you should write, but with the pure-pursuit math intentionally left as `TODO`.

```python
import time
import threading

import numpy as np
import rclpy

# Provided helper
from trajectory_ref_qt_streamer import TrajectoryStreamNode


def wrap_angle_pi(angle_rad: float) -> float:
	"""Wrap angle to [-pi, pi)."""
	return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


if not rclpy.ok():
	rclpy.init()


# -----------------
# TODO: parameters
# -----------------
vehicle_name = "EGO"
csv_path = "ref.csv"      # reference trajectory CSV
Ld0 = ...                 # lookahead distance [m]
steering_scale = ...      # from Q3 calibration (delta [rad] -> steeringInput [-])

a = ...
b = ...
wheelbase_L = a + b

vel_target = ...          # speed target [m/s]
HZ = ...                  # control loop rate
T_END = ...               # total run time [s]


traj_server = TrajectoryStreamNode(
	vehicle_name=vehicle_name,
	csv_path=csv_path,
	Ld=Ld0,
	steering_scale=steering_scale,
	enable_display=True,
)


def control_loop() -> None:
	DT = 1.0 / HZ
	elapsed = 0.0
	while rclpy.ok() and elapsed <= T_END:
		latest_state = traj_server.get_state_on_ref_path()
		if latest_state is None:
			time.sleep(DT)
			continue

		# Current state (estimated on the reference)
		x_curr, y_curr, yaw = latest_state.x, latest_state.y, latest_state.yaw
		V_curr = latest_state.V

		# Lookahead target point on the reference
		x_target, y_target = latest_state.x_target, latest_state.y_target

		# -----------------------------------------
		# TODO: Pure Pursuit from class
		# -----------------------------------------
		# Use the geometry between (x_curr,y_curr,yaw) and (x_target,y_target)
		# to compute a road-wheel steering angle command delta_target [rad].
		#
		# The following names match the lecture notation:
		heading_to_target = ...   # e.g., atan2( y_target - y_curr, x_target - x_curr )
		alpha = ...               # wrap_angle_pi( heading_to_target - yaw )
		kappa = ...               # curvature command from pure pursuit
		delta_target = ...        # steering angle command using wheelbase

		traj_server.send_vehicle_speed(
			V_target=vel_target,
			delta_target=delta_target,
		)

		elapsed += DT
		time.sleep(DT)


control_thread = threading.Thread(target=control_loop, daemon=True)
control_thread.start()

# Optional: UI display (lets you see target vs measured signals)
traj_server.run_display(blocking=True)
```

### Deliverables

- Evidence of successful path following (short video or a few screenshots).
- A short comparison across **at least two** `V_target` values (e.g., “slow” vs “faster”): what breaks first, what stays stable, and whether you had to retune `Ld`.
- Plots (from your live display or PlotJuggler) showing:
	- `V_target` vs `V`
	- your `delta_target` signal over time
- 3–6 sentences explaining what happens when `Ld` is:
	- too small (aggressive / oscillatory)
	- too large (cuts corners / slow response)
