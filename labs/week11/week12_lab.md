# Week 12 - Closed-Loop BeamNG Tracking with MPC

**Theme:** move from the Week 11 local scaffold to the live BeamNG interface and build a closed-loop controller around the Frenet tracking MPC.

**This lab has no formal deliverables.** Use it to assemble and tune the straight-line closed-loop controller that you will build on next.

---

## Learning goals

By the end of this lab, you should be able to:

1. Generate a straight reference with a clear steady-speed condition.
2. Explain where the desired wheel-speed and rear-torque profiles come from in the offline reference.
3. Start BeamNG from the ROS stack, connect to the live reduced-state stream, and spawn the vehicle at the beginning of the reference.
4. Build a wheel-speed based low-level longitudinal loop around inverse-throttle feedforward plus PI feedback.
5. Run a closed-loop MPC tracking controller in BeamNG with separate actuation and planning workers.
6. Organize the controller as separate actuation and planning workers that you can tune incrementally.

---

## Setup

Run the same update and rebuild steps as in the previous BeamNG labs.

```bash
cd ~/ros2_ws/src/sim_ros_framework

git checkout hpa-s26
git pull origin hpa-s26

python -m pip install -r requirements.txt

./luamod/build.bash --win --all
./xal_bng_ws_build.bash -w ~/ros2_ws -r jazzy --clean
source ~/ros2_ws/install/setup.bash
```

The Lua build step matters here. If you skip it, the in-sim controller and telemetry may not match the Python side you are running.

All work for this lab takes place in `labs/week11/`, even though this is the Week 12 assignment.

---

## BeamNG launch reminder

You already saw this workflow in earlier labs. Start the simulator stack from WSL with the same scenario config you used before:

```bash
ros2 launch bng_bringup basic.launch.py \
  config:=techground_sbr.yaml \
  host:=<WINDOWS_BEAMNG_IP> \
  remote:=<WSL_IP>
```

Meaning:

- `host:=...` is the Windows / BeamNG IP that WSL connects to.
- `remote:=...` is the WSL IP that BeamNG sends packets back to.

If you need the WSL IP from WSL, a convenient command is:

```bash
ip route get 1.1.1.1 | awk '{print $7; exit}'
```

On the Windows side, if you want faster turnover while debugging the controller, it is reasonable to run BeamNG with no graphics, for example with `-headless` or `-gfx null`. You can always turn rendering back on later once the controller is working.

The Windows-side BeamNG command should look like this:

```powershell
.\BeamNG.tech.x64.exe -tcom -colorStdOutLog [-disable-sandbox] [-nosteam] -console -headless -gfx null
```

The two additions to notice are:

- `-headless` to run without the normal rendered window.
- `-gfx null` to disable the graphics backend.

This shortened command is only meant to remind you about the headless and no-GPU flags. It does not show the full IP-related syntax. Refer back to the old BeamNG lab command line for the complete startup command with the networking arguments.

---

## Your file for this lab

Create your own Python file in `labs/week11/`, for example `your_week12_controller.py`.

---

## Imports you should start from

Use this import block at the top of your script:

```python
from __future__ import annotations

import importlib
import sys
import threading
import time
from pathlib import Path

import casadi as ca
import matplotlib.pyplot as plt
import numpy as np
import rclpy


_THIS_DIR = Path(__file__).resolve().parent
_WEEK10_DIR = _THIS_DIR.parent / "week10"
if str(_WEEK10_DIR) not in sys.path:
	sys.path.insert(0, str(_WEEK10_DIR))

from bng_controller.torque_speed_controller import TorqueSpeedController
from bng_simulator.utils.math_op import convert_euler_to_quaternion
from bng_simulator.utils.services_utils import send_request
from mpc_helper import FrenetTrackingMPC, TrackingMPCConfig
from tracking_helper import TrackingReference
from week12_helper import (
	ControllerRuntime,
	Week12Logger,
	build_plan,
	publish_live_visualizer,
	sample_plan,
	start_live_visualizer,
	stop_live_visualizer,
)


_week10_traj = importlib.import_module("traj_opt_helper")
_week10_dyn = importlib.import_module("casadi_dynamics")
TrackDefinition = _week10_traj.TrackDefinition
TrajOptConfig = _week10_traj.TrajOptConfig
TrajOptProblem = _week10_traj.TrajOptProblem
FialaBicycleCasADi = _week10_dyn.FialaBicycleCasADi
fiala_params = _week10_dyn.fiala_params
```

These imports are enough to build the reference, connect to BeamNG, run the workers, publish ROS logging topics, and keep the Qt visualizer.

---

## Helper files you should inspect

Do not blindly use the helpers. Inspect them.

The most relevant files are:

| File | What to inspect |
|---|---|
| `week10/traj_opt_helper.py` | `TrajOptConfig`, `TrajOptProblem`, `TrajOptResult` |
| `week10/casadi_dynamics.py` | local torque-estimation and inverse-throttle surrogate |
| `week11/tracking_helper.py` | `TrackingReference.from_traj_result(...)`, `cartesian_to_frenet(...)`, `get_ref_traj(...)` |
| `week11/mpc_helper.py` | `TrackingMPCConfig`, `FrenetTrackingMPC.solve(...)`, `_build_problem()` |
| `week11/week12_helper.py` | `ControllerRuntime`, `Week12Logger`, `build_plan(...)`, `sample_plan(...)` |

That is where you should look if you are unsure:

- how the straight reference is built
- where the steady-speed condition is encoded
- how the Frenet preview window is sampled
- where the local torque-estimation and inverse-throttle surrogate lives
- which weights and bounds the MPC uses
- what gets published to ROS for logging

---

## Review: the live control structure

The overall closed-loop structure is still the same as in Week 11, but now the state comes from BeamNG through ROS.

At a high level, your script will:

1. build the offline reference
2. connect to BeamNG and wait for live reduced-state messages
3. teleport the vehicle to the start of the reference
4. run a fast actuation worker
5. run a slower MPC worker
6. log the relevant signals through ROS
7. inspect the run while the controller is executing


---

## Part 1 - Build and inspect a straight reference

Start with a straight-line reference. The point of this part is to make the offline reference explicit before any BeamNG control is involved.

### Code prototype

Use this as the shape of your reference builder:

```python
vehicle_name = "EGO"
reference_mode = "trajectory"


def build_straight_reference(mode: str = "trajectory") -> tuple[object, TrackingReference]:
	kappa_fn = lambda _s: 0.0
	track = TrackDefinition.from_kappa_function(
		kappa_fn,
		s_start=0.0,
		s_end=500.0,
		ds=2.0,
		e_half_width=4.0,
	)
	cfg = TrajOptConfig(
		num_nodes=200,
		start_velocity=(3.0, 5.0),
		initial_state_bounds={
			"e": (-0.05, 0.05),
			"beta": (-0.05, 0.05),
			"delta": (-0.02, 0.02),
			"r": (-0.01, 0.01),
			"rear_wheel_torque": (0.0, 400.0),
		},
		target_steady_velocity=(100.0, 15.0), # Change this if you want a different steady condition.
		cost_control_rate_weight=1.0,
		ipopt_print_level=0,
		ipopt_max_iter=1200,
	)
	result = TrajOptProblem(track, cfg).solve()
	reference = TrackingReference.from_traj_result(result, reference_mode=mode)
	return result, reference
```

### What to notice

The steady-speed condition is added in:

```python
target_steady_velocity=(100.0, 15.0)
```

That means the optimizer is being asked to approach a steady operating condition near the later part of the path. The exact implementation is in `week10/traj_opt_helper.py`, so inspect that file and verify how this condition is encoded.

### What to plot

Before you touch BeamNG, visualize at least:

- the reference trajectory in the plane
- the curvature profile
- the reference bounds
- the desired rear wheel speed profile
- the desired rear wheel torque profile

The last two matter because they are the signals your low-level longitudinal loop will try to realize later.

### Suggested plotting prototype

```python
def plot_reference_summary(reference: TrackingReference):
	x_left, y_left, _ = reference.frenet_to_cartesian(reference.s, reference.e_max, np.zeros_like(reference.s))
	x_right, y_right, _ = reference.frenet_to_cartesian(reference.s, reference.e_min, np.zeros_like(reference.s))
	fig, axs = plt.subplots(1, 3, figsize=(14, 4.5))

	axs[0].plot(reference.x, reference.y, lw=2.0)
	axs[0].plot(x_left, y_left, "k--", lw=1.2, alpha=0.8, label="left bound")
	axs[0].plot(x_right, y_right, "k--", lw=1.2, alpha=0.8, label="right bound")
	axs[0].set_aspect("equal")
	axs[0].grid(True, alpha=0.3)
	axs[0].legend()

	axs[1].plot(reference.s, reference.kappa, lw=2.0)
	axs[1].set_xlabel("s [m]")
	axs[1].set_ylabel("kappa [1/m]")
	axs[1].grid(True, alpha=0.3)

	axs[2].plot(reference.s, reference.state_profiles["wr"], lw=2.0, label="wr_ref")
	axs[2].plot(reference.s, reference.state_profiles["rear_wheel_torque"], lw=2.0, label="rear torque ref")
	axs[2].set_xlabel("s [m]")
	axs[2].grid(True, alpha=0.3)
	axs[2].legend()

	fig.tight_layout()
	return fig
```

### Tasks

1. Build the straight reference.
2. Explain where the steady-speed condition is added.
3. Plot the reference trajectory, bounds, desired rear wheel speed, and desired rear wheel torque.
4. Briefly comment on whether the wheel-speed and torque profiles look near steady by the end of the path.

---

## Part 2 - Start BeamNG and spawn at the reference start

Now attach the script to the live BeamNG simulator.

### Build the torque helper functions

Before connecting to BeamNG, instantiate the local model and ask it for the reusable powertrain functions:

```python
ca_drift = FialaBicycleCasADi(fiala_params)
powertrain = ca_drift.build_powertrain_functions()
torque_est_fn = powertrain["torque_est_fn"]
inv_throttle_fn = powertrain["inv_throttle_fn"]
```

That keeps the surrogate definition inside `week10/casadi_dynamics.py` instead of duplicating a local helper in every script.

These two objects are CasADi functions.

- `torque_est_fn(engine_speed_rads, boost_pressure, throttle, rear_wheelspeed_ms)` returns the estimated rear-wheel torque generated by the surrogate.
- `inv_throttle_fn(engine_speed_rads, boost_pressure, rear_wheelspeed_ms, desired_rear_wheel_torque)` returns the feedforward throttle value that should produce the requested rear-wheel torque.

In this lab you will mostly call them numerically inside the low-level loop, but they can also be used symbolically.

### Initialize ROS and the live controller interface

Add this next:

```python
ca_drift = FialaBicycleCasADi(fiala_params)
powertrain = ca_drift.build_powertrain_functions()
torque_est_fn = powertrain["torque_est_fn"]
inv_throttle_fn = powertrain["inv_throttle_fn"]

if not rclpy.ok():
	rclpy.init()

ctl = TorqueSpeedController(vehicle_name=vehicle_name, spin_in_thread=True)
time.sleep(2.0) # pause to let the subscription come up
```

That pause after creating `TorqueSpeedController` is intentional. The controller subscribes to the live reduced-state topic in a background thread, and the short pause gives the ROS subscription path time to come up before you try to use the state stream or teleport the vehicle.

If you skip the pause and immediately start using the controller, the first few messages may not have arrived yet.

### Teleport helper prototype

Copy this function into your own script:

```python
def spawn_at_reference(reference: TrackingReference) -> None:
	x0 = float(reference.x[0])
	y0 = float(reference.y[0])
	yaw0_deg = float(np.degrees(reference.yaw[0]))
	z0 = ... # You an read the initial z or so from ros2 topic echo
	rot_quat = convert_euler_to_quaternion((0.0, 0.0, np.radians(yaw0_deg + 90.0)))
	send_request(
		"vehicle.teleport",
		{
			"vehicle_name": vehicle_name,
			"pos": [x0, y0, z0],
			"rot_quat": [float(q) for q in rot_quat],
			"reset": False,
		},
	)
	time.sleep(3.0)
```

What this does:

- uses the first reference position as the BeamNG spawn point
- converts the reference yaw into BeamNG’s spawn convention with the `+90 deg` offset
- waits a few seconds so the teleport is fully applied before continuing

After you use it:

```python
spawn_at_reference(tracking_ref)
```

This helper is especially useful because it lets you reposition the vehicle at the start of the reference without doing `ros2 launch` again and again.

Right after `spawn_at_reference(tracking_ref)`, add a pause if you want to inspect the spawn before the rest of the script runs. For example:

```python
spawn_at_reference(tracking_ref)
input("Press Enter to continue once you have checked the spawn in BeamNG...")
```

change the initial position in your reference slightly and confirm that the spawn location changes with it. That is a quick sanity check that your script really is using the reference geometry you built.

### Tasks

1. Start BeamNG with the ROS launch command above.
2. Initialize `TorqueSpeedController` and explain why the pause after initialization is useful.
3. Add and call `spawn_at_reference(tracking_ref)`.
4. Verify that changing the initial reference pose changes the BeamNG spawn position.

---

## Part 3 - Low-level wheel-speed loop and closed-loop MPC on the straight

At this point you have:

- an offline straight reference
- a live BeamNG state stream
- a vehicle teleported to the start of the reference
- feedforward torque helpers

Now you can build the real closed-loop controller.

For now, focus only on the actuation side. The goal of this section is to make the fast worker path explicit before you fill in the MPC worker.

### Part 3.a - Build the main controller scaffold

Start by adding the MPC builder itself. Use this exact template first and tune the weights later:

```python
def build_mpc() -> FrenetTrackingMPC:
    # Will need to play with all of this for best perf
    # The values here are far from tuned.
	return FrenetTrackingMPC(
		TrackingMPCConfig(
			horizon_steps=15,
			prediction_ds=2.0,
			weight_speed=0.1,
			weight_beta=0.1,
			weight_wr=0.1,
			weight_e=0.1,
			weight_dphi=0.1,
			terminal_e=0.1,
			terminal_dphi=0.1,
			ipopt_print_level=0,
			ipopt_max_iter=500,
			max_lateral_error=6.0,
			weight_steer=0.01,
			weight_torque=0.1,
			weight_steer_increment=0.1,
			weight_torque_increment=0.1,
			warm_start_duals=True,
		),
		state_bounds={"e": (-6.0, 6.0)},
	)
```

This is only a starting point. You should expect to come back and tune these parameters later once the controller is running.

After that, build the shared runtime and keep the Qt visualizer enabled:

```python
runtime = ControllerRuntime()
ros_logger = Week12Logger(vehicle_name) if publish_ros else None
live_visualizer = True
if live_visualizer:
	plt.close("all")
visualizer = start_live_visualizer(tracking_ref) if live_visualizer else None

mpc = build_mpc()
preview_horizon = mpc.cfg.horizon_steps * float(mpc.cfg.prediction_ds or 1.0)
s_stop = float(tracking_ref.s[-1] - preview_horizon)
solve_log: list[dict[str, object]] = []
```

This is a good place to start when tuning in headless and no-GPU mode: BeamNG runs without graphics, while the Qt visualizer still shows the controller behavior and the planned trajectory snapshots.

`ControllerRuntime` is the small shared-memory object between the two workers. It stores:

- the latest raw vehicle state
- the latest projected Frenet state
- the latest MPC plan
- the last applied control
- a stop flag and the first worker exception

That is the minimum bookkeeping needed for the two-worker structure from Week 11.

Also keep this steering conversion coefficient near the top of your file:

```python
STEER_TO_ROADWHEEL_ANGLE = -0.5814544122705007
```

The planned steering coming out of the MPC is a road-wheel angle in radians. The BeamNG command you actually send is the normalized `steering` input. In this setup,

$$
\mathrm{roadwheel\_angle} \approx \mathrm{STEER\_TO\_ROADWHEEL\_ANGLE} \cdot \mathrm{steering}
$$

so in code you will typically compute:

```python
steering_cmd = roadwheel_target / STEER_TO_ROADWHEEL_ANGLE
```

and then clip that value before sending it.

### Connect the live state callback

Copy this directly:

```python
def on_state_msg(msg) -> None:
	runtime.publish_raw_state(ctl.state_msg_to_dict(msg))


ctl.add_state_listener(on_state_msg)
```

What this does is simple: every time the live reduced-state message arrives from BeamNG, you convert it to the controller dictionary format and publish it into `ControllerRuntime`. After that, the actuation worker can wait on new state data without talking to ROS directly.

### Add the worker placeholders first

Before implementing the details of the controller law, put the worker structure in place.

```python
def actuation_worker() -> None:
	"""Read live state, project to Frenet, and send the next low-level command."""
	pass
```

This worker will eventually:

- wait for the newest raw vehicle state
- project it into Frenet coordinates
- sample the latest stored plan
- apply the wheel-speed / inverse-throttle / PI logic
- send the steering and throttle command to BeamNG

Also add an incomplete MPC worker placeholder for now:

```python
def mpc_worker() -> None:
	"""Project the latest state, solve the tracking problem, and store the new plan."""
	pass
```

This worker will eventually:

- wait for the newest projected state
- build the preview window from the reference
- solve the Frenet tracking MPC
- store the plan in `ControllerRuntime`

### End-of-file thread structure

The end of your file should have the same high-level structure as below:

```python
actuation_thread = threading.Thread(target=lambda: run_worker("actuation", actuation_worker), daemon=True)
mpc_thread = threading.Thread(target=lambda: run_worker("mpc", mpc_worker), daemon=True)

actuation_thread.start()
mpc_thread.start()

try:
	while not runtime.should_stop():
		if visualizer is not None and not visualizer["process"].is_alive():
			runtime.stop()
			break
		time.sleep(0.05)
except KeyboardInterrupt:
	runtime.stop()
finally:
	actuation_thread.join(timeout=2.0)
	mpc_thread.join(timeout=2.0)
	ctl.remove_state_listener(on_state_msg)
	ctl.clear_targets()
	ctl.close()
	if ros_logger is not None:
		ros_logger.destroy_node()

thread_error = runtime.get_first_error()
visualizer_alive = visualizer is not None and visualizer["process"].is_alive()

if thread_error is not None and visualizer_alive:
	name, exc = thread_error
	print(f"{name} thread failed: {exc}")
	print("controller stopped; close the Qt visualizer to exit")
	try:
		while visualizer["process"].is_alive():
			time.sleep(0.1)
	except KeyboardInterrupt:
		stop_live_visualizer(visualizer)
elif visualizer_alive:
	print("simulation finished; close the Qt visualizer to exit")
	try:
		while visualizer["process"].is_alive():
			time.sleep(0.1)
	except KeyboardInterrupt:
		stop_live_visualizer(visualizer)

if thread_error is not None:
	name, exc = thread_error
	raise RuntimeError(f"{name} thread failed") from exc

plt.show()
```

You do not need to memorize this. Copy it, keep it in place, and then fill in the worker bodies.

### Copy-paste scaffold for 3.a

Below is a full sample block that you can copy into your file. The actuation path is mostly filled in structurally, but the actual feedback law is intentionally left incomplete. You should define your own `WR_FEEDBACK_KP` and `WR_FEEDBACK_KI` and decide how to update the wheel-speed integrator.

```python
def compute_low_level_command(
	state: dict[str, float],
	projected: dict[str, object],
	plan: dict[str, object],
	integ_e_wr: float,
	dt: float,
) -> tuple[dict[str, float], float]:
	state_now = projected["current_state"]
	s_now = float(projected["s"])
    # This functioon finds where you are in the plan and returns the target values at that point.
	plan_sample = sample_plan(plan, s_now + 0.5) # Exlain why you need an offset here.

	roadwheel_target = float(plan_sample["roadwheel_angle"])
	rear_wheelspeed_target = float(plan_sample["rear_wheelspeed_ms"])
	rear_wheel_torque_target = float(plan_sample["rear_wheel_torque"])

	# Convert the planned road-wheel angle into the normalized BeamNG steering command.
	steering_cmd = ...

	# Feedforward throttle from the inverse-throttle surrogate.
	# Arguments: engine speed [rad/s], boost pressure, rear wheel speed [m/s], desired rear-wheel torque [Nm].
	ff_throttle = float(
		inv_throttle_fn(
			state["we"],
			state["pb"],
			state_now["wr"],
			rear_wheel_torque_target,
		)
	)

	# Wheel-speed tracking error for your PI feedback term.
	wr_error = rear_wheelspeed_target - float(state_now["wr"])

	# TODO: choose your own Kp and Ki.
	# TODO: update integ_e_wr using dt and clip it if you want anti-windup.
	# TODO: define fb_throttle from wr_error and integ_e_wr.
	# Example intent only:
	# integ_e_wr = ...
	# fb_throttle = ...

	# Start from the feedforward term, then add your feedback correction.
	throttle_cmd = ff_throttle
	# throttle_cmd = ff_throttle + fb_throttle
	throttle_cmd = float(np.clip(throttle_cmd, 0.0, 1.0))

	command = {
		"throttle": throttle_cmd,
		"steering": steering_cmd,
		"roadwheel_angle": roadwheel_target,
		"rear_wheel_torque": rear_wheel_torque_target,
		"rear_wheelspeed_target": rear_wheelspeed_target,
	}
	return command, integ_e_wr


def actuation_worker() -> None:
	last_seq = 0
	last_s = None
	init_time = None
	last_sim_time = None
	integ_e_wr = 0.0

	while not runtime.should_stop():
		item = runtime.wait_for_raw_state(last_seq)
		if item is None:
			break
		seq, state = item
		last_seq = seq

		if init_time is None:
			init_time = float(state["t"])

		projected, last_s = build_projected_state(tracking_ref, seq, state, last_s, init_time)
		runtime.publish_projected_state(projected)

		if projected["s"] >= s_stop:
			runtime.stop()
			break

		plan = runtime.get_plan()
		current_sim_time = float(state["t"])
		dt = 0.0 if last_sim_time is None else float(np.clip(current_sim_time - last_sim_time, 0.0, 0.1))
		last_sim_time = current_sim_time

		if plan is None:
			continue

		command, integ_e_wr = compute_low_level_command(state, projected, plan, integ_e_wr, dt)
		ctl.send_command(
			throttle=command["throttle"],
			brake=0.0,
			steering=command["steering"],
		)
		runtime.set_applied_control(command)


def mpc_worker() -> None:
	"""Fill this in next. It should wait for projected state, solve MPC, and store the plan."""
	raise NotImplementedError("Implement the Week 12 MPC worker next")


def run_worker(name: str, worker) -> None:
	try:
		worker()
	except Exception as exc:
		runtime.record_error(name, exc)


actuation_thread = threading.Thread(target=lambda: run_worker("actuation", actuation_worker), daemon=True)
mpc_thread = threading.Thread(target=lambda: run_worker("mpc", mpc_worker), daemon=True)

actuation_thread.start()
mpc_thread.start()

try:
	while not runtime.should_stop():
		if visualizer is not None and not visualizer["process"].is_alive():
			runtime.stop()
			break
		time.sleep(0.05)
except KeyboardInterrupt:
	runtime.stop()
finally:
	actuation_thread.join(timeout=2.0)
	mpc_thread.join(timeout=2.0)
	ctl.remove_state_listener(on_state_msg)
	ctl.clear_targets()
	ctl.close()
	if ros_logger is not None:
		ros_logger.destroy_node()

thread_error = runtime.get_first_error()
visualizer_alive = visualizer is not None and visualizer["process"].is_alive()

if thread_error is not None and visualizer_alive:
	name, exc = thread_error
	print(f"{name} thread failed: {exc}")
	print("controller stopped; close the Qt visualizer to exit")
	try:
		while visualizer["process"].is_alive():
			time.sleep(0.1)
	except KeyboardInterrupt:
		stop_live_visualizer(visualizer)
elif visualizer_alive:
	print("simulation finished; close the Qt visualizer to exit")
	try:
		while visualizer["process"].is_alive():
			time.sleep(0.1)
	except KeyboardInterrupt:
		stop_live_visualizer(visualizer)

if thread_error is not None:
	name, exc = thread_error
	raise RuntimeError(f"{name} thread failed") from exc

plt.show()
```

The point of this scaffold is:

- `compute_low_level_command(...)` is where you will add the wheel-speed PI feedback later
- `actuation_worker()` already shows the full state-to-command flow
- `mpc_worker()` is left intentionally incomplete so you can fill it in during Part `3.b`

### Copy-paste scaffold for 3.b

Once the actuation path is in place, fill in the MPC side with the same approach: keep the structure explicit and isolate the solve step in its own helper.

```python
# Same iplementation as in Week 11, but now the state comes from the live projected stream instead of the raw state stream.
def solve_tracking_problem(
	mpc: FrenetTrackingMPC,
	reference: TrackingReference,
	projected: dict[str, object],
	applied_control: dict[str, float],
) -> tuple[dict[str, object], float]:
	# Build the preview window starting at the current Frenet position.
	ref_window = reference.get_ref_traj(
		s_start=float(projected["s"]),
		horizon_steps=mpc.cfg.horizon_steps,
		ds=float(mpc.cfg.prediction_ds or 1.0),
	)

	# Solve one receding-horizon problem from the current Frenet state.
	solve_start = time.perf_counter()
	solution = mpc.solve(
		projected["current_state"],
		ref_window,
		prev_control=applied_control,
	)
	solve_time = time.perf_counter() - solve_start
	return solution, solve_time


def mpc_worker() -> None:
	last_seq = 0
	solve_idx = 0

	while not runtime.should_stop():
		projected = runtime.wait_for_projected_state(last_seq)
		if projected is None:
			break
		last_seq = int(projected["seq"])

		if projected["s"] >= s_stop:
			runtime.stop()
			break

		applied_control = runtime.get_applied_control()
		solution, solve_time = solve_tracking_problem(mpc, tracking_ref, projected, applied_control)

		# Convert the raw MPC output into the plan format used by sample_plan(...)
		# and by the actuation worker.
		plan = build_plan(tracking_ref, projected, solution, applied_control, solve_time)
		runtime.set_plan(plan)
		solve_log.append(plan["record"])

		# Keep the live visualizer in sync with the newest planned rollout.
		publish_live_visualizer(visualizer, plan["record"])

		print(
			f"solve={solve_idx:03d} "
			f"s={projected['s']:7.2f} "
			f"e={projected['e']: .3f} "
			f"dphi={projected['dphi']: .3f} "
			f"V={projected['raw_state']['V']: .2f} "
			f"solve_t={solve_time * 1e3:6.1f}ms"
		)
		solve_idx += 1
```

What this code is doing:

- `solve_tracking_problem(...)` extracts the Frenet preview window and times one solve
- `mpc_worker()` waits for the latest projected state rather than the raw state stream
- `build_plan(...)` converts the MPC output into the sampled plan representation used by the actuation worker
- `runtime.set_plan(plan)` makes that latest plan available to the fast low-level loop

### Tasks

1. Copy the scaffold above into your own file.
2. Fill in `compute_low_level_command(...)` with your own wheel-speed PI feedback law.
3. Verify that the steering command uses the road-wheel to steering conversion correctly.
4. Add `solve_tracking_problem(...)` and the `mpc_worker()` structure above.
5. Verify that the actuation worker is sampling the latest plan produced by the MPC worker.

---

## Part 4 - Generate oval references with different aggressiveness levels

Once the straight-line controller is working, keep the same controller structure and move to an oval reference.

The goal here is not just to switch from straight to oval. The goal is to generate several oval references with increasing aggressiveness and see where the same controller begins to struggle.

In this part, “more aggressive” should mean a combination of:

- tighter curvature or less forgiving geometry
- a more ambitious start-speed range and resulting optimized speed profile
- less smoothing in the offline optimized input profile

### Step 1 - Define explicit oval presets

Use a small preset dictionary so the aggressiveness levels are visible in one place:

```python
OVAL_REFERENCE_PRESETS = {
	"mild": {
		"track": {
			"straight_length": 25.0,
			"radius": 18.0,
			"n_laps": 1,
			"e_half_width": 4.0,
			"lead_in": 60.0,
			"lead_out": 60.0,
		},
		"trajopt": {
			"num_nodes": 180,
			"start_velocity": (3.0, 4.5),
			"cost_control_rate_weight": 2.0,
			"ipopt_max_iter": 1800,
		},
	},
	"medium": {
		"track": {
			"straight_length": 20.0,
			"radius": 15.0,
			"n_laps": 2,
			"e_half_width": 4.0,
			"lead_in": 50.0,
			"lead_out": 50.0,
		},
		"trajopt": {
			"num_nodes": 200,
			"start_velocity": (3.0, 5.0),
			"cost_control_rate_weight": 1.0,
			"ipopt_max_iter": 2200,
		},
	},
	"aggressive": {
		"track": {
			"straight_length": 15.0,
			"radius": 12.0,
			"n_laps": 2,
			"e_half_width": 3.5,
			"lead_in": 40.0,
			"lead_out": 40.0,
		},
		"trajopt": {
			"num_nodes": 220,
			"start_velocity": (4.0, 6.0),
			"cost_control_rate_weight": 0.35,
			"ipopt_max_iter": 2600,
		},
	},
}
```

These values are only a starting point. The important part is that the aggressiveness is encoded explicitly and can be tuned.

### Step 2 - Build one oval reference from a selected level

Use a builder like this:

```python
def build_oval_reference(level: str = "mild", mode: str = "trajectory") -> tuple[object, TrackingReference]:
	if level not in OVAL_REFERENCE_PRESETS:
		raise ValueError(f"Unknown oval aggressiveness '{level}'")

	preset = OVAL_REFERENCE_PRESETS[level]
	track = TrackDefinition.oval(**preset["track"])
	cfg = TrajOptConfig(
		num_nodes=preset["trajopt"]["num_nodes"],
		start_velocity=preset["trajopt"]["start_velocity"],
		initial_state_bounds={
			"e": (-0.05, 0.05),
			"beta": (-0.05, 0.05),
			"delta": (-0.02, 0.02),
			"r": (-0.01, 0.01),
			"rear_wheel_torque": (0.0, 400.0),
		},
		cost_control_rate_weight=preset["trajopt"]["cost_control_rate_weight"],
		ipopt_print_level=0,
		ipopt_max_iter=preset["trajopt"]["ipopt_max_iter"],
	)
	result = TrajOptProblem(track, cfg).solve()
	return result, TrackingReference.from_traj_result(result, reference_mode=mode)
```

### Step 3 - Generate all levels and compare them offline first

Before tracking anything in BeamNG, generate all three references and compare them. For example, compare:

- geometry in the plane
- speed profile `V(s)`
- rear wheel speed `wr(s)`
- rear wheel torque profile

This step matters because it tells you whether the “aggressive” reference is actually aggressive in a meaningful way or whether you only changed labels.

### Step 4 - Try to track them in increasing order

Add a simple selector near the top of your controller file:

```python
reference_shape = "oval"
oval_aggressiveness = "mild"
```

and then switch the reference build logic to:

```python
if reference_shape == "oval":
	ref_traj, tracking_ref = build_oval_reference(level=oval_aggressiveness, mode=reference_mode)
else:
	ref_traj, tracking_ref = build_straight_reference(reference_mode)
```

Now try the same closed-loop controller on:

1. `mild`
2. `medium`
3. `aggressive`

Do not retune everything immediately. First see what breaks when you reuse the straight-line tuning.
