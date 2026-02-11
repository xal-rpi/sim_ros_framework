# Lab 5–6 — Throttle Control + Pure Pursuit + Bicycle Lateral Force Inversion (Weeks 5 & 6)

**Theme:** build a practical speed controller (feedforward + PI feedback) that commands **throttle**, then reuse it inside your **pure pursuit** controller, and finally use such a controller to obtain proper data to **invert the bicycle model** and estimate lateral tire forces vs slip angles.

**Due:** **Wednesday 02/25 (full lab)**. There is no class next week.

**What you submit (single docx including plots and/or code screenshots):**
- A short report (figures + key numbers + 1–2 paragraphs per part)
- Your controller scripts (or notebook) used to run the experiments
- Plots exported as PNG/SVG (and optionally PlotJuggler layout screenshot when tuning)

---

## Pre-lab — repo + pipeline sanity check (same as Week 3/4)

### 1) Pull the course branch + rebuild

```bash
cd ~/ros2_ws/src/sim_ros_framework

# Make sure none of these commands fail (if they do, ask for help before proceeding):
git checkout . 
git checkout hpa-s26
git pull origin hpa-s26

python -m pip install -r requirements.txt

# ALWAYS DO THIS WITHOUT THE GAME RUNNING.
./luamod/build.bash --win --all

./xal_bng_ws_build.bash -w ~/ros2_ws -r jazzy --clean
source ~/ros2_ws/install/setup.bash
```

### 2) Launch + confirm data flow

Launch gridworld with correct `host:=...` and `remote:=...` exactly like Week 4.

Quick checks:

```bash
ros2 topic list | grep EGO
ros2 topic echo /EGO/reduced_state --once
```

Recommended PlotJuggler signals for this lab:
- `/EGO/reduced_state/vx`
- `/EGO/reduced_state/r`
- `/EGO/reduced_state/delta`
- `/EGO/reduced_state/throttle`
- `/EGO/llc_cmd/vehicle_speed` (your speed target. will only show up when sending targets)
- `/EGO/llc_cmd/steering` (your steering target; will only show up when sending commands)

Important note on logging + offline analysis:
- You should be comfortable using **`/EGO/gtstate`** in your logs. It contains the richest set of fields at the highest frequency.
- `/EGO/reduced_state` is a convenient reduced subset intended for lightweight streaming (great for PlotJuggler), but your report plots should primarily come from **`/EGO/gtstate`** unless stated otherwise.

Common `/EGO/gtstate` fields used in this lab:
- `throttle`, `accel_x`, `accel_y` (longitudinal control + acceleration)
- `vel_x`, `vel_y` (body-frame velocities)
- `angVel_z` (yaw rate $r$)
- `wheelFL_angle`, `wheelFR_angle` (front road-wheel steer angles; average them to get $\delta$)

---

## Part 1 (Week 5) — Design a speed controller (feedforward + PI) using throttle

### Goal

You will design a longitudinal speed controller that makes the vehicle track a desired speed profile $v_{ref}(t)$ by commanding **throttle**.

This lab has **two implementation tracks**:

**Undergraduate track (use the in-sim controller):**
- You identify a feedforward model and tune PI gains, then **send the parameters** to the low-level controller using `send_calibration(...)`.
- You then command target speed using `command_vehicle_speed(...)` and the low-level controller applies throttle internally.

**Graduate track (implement the controller in Python):**
- You identify the same feedforward model, but you implement the PI loop in Python and send the resulting throttle command using `send_command(throttle=...)`.
- In this track, do not rely on `command_vehicle_speed(...)` for control (it is okay to log a reference speed profile).

In both tracks, the controller structure is:

$$\text{throttle}(t) = \underbrace{\text{FF}(v_{ref}, a_{ref})}_{\text{model}} + \underbrace{K_p (v_{ref}-v) + K_i \int (v_{ref}-v)dt}_{\text{PI feedback}}$$

You will:
1) collect open-loop throttle data,
2) identify a simple feedforward model,
3) test **feedforward-only** tracking,
4) tune **PI** gains using PlotJuggler, and
5) log and report your best tuning.

### Choose your vehicle (pick ONE)

Pick one scenario config (this selects the vehicle):

- [src/bng_xal/bng_bringup/config/scenarios/techground_sbr.yaml](../../src/bng_xal/bng_bringup/config/scenarios/techground_sbr.yaml)
- [src/bng_xal/bng_bringup/config/scenarios/techground_sunburst2.yaml](../../src/bng_xal/bng_bringup/config/scenarios/techground_sunburst2.yaml)
- [src/bng_xal/bng_bringup/config/scenarios/techground_etk800.yaml](../../src/bng_xal/bng_bringup/config/scenarios/techground_etk800.yaml)

These scenarios start on `tech_ground` with plenty of flat space for steady-state testing.

**Where to find physical parameters** (you will need them in Part 3):

Vehicle parameter YAMLs are in:
- [src/bng_xal/bng_bringup/config/vehicles/](../../src/bng_xal/bng_bringup/config/vehicles/)

For example:
- SBR: [src/bng_xal/bng_bringup/config/vehicles/sbr_track.yaml](../../src/bng_xal/bng_bringup/config/vehicles/sbr_track.yaml)
- Sunburst: [src/bng_xal/bng_bringup/config/vehicles/sunburst2_drift_pro.yaml](../../src/bng_xal/bng_bringup/config/vehicles/sunburst2_drift_pro.yaml)
- ETK800: [src/bng_xal/bng_bringup/config/vehicles/etk800_844_track_M.yaml](../../src/bng_xal/bng_bringup/config/vehicles/etk800_844_track_M.yaml)

You will specifically use:
- `totalMass` (mass $m$)
- `inertia.zz` (yaw inertia $I_z$)
- `cogToFrontAxle` (distance $a$)
- `cogToRearAxle` (distance $b$)

### A. Select, launch + record one short “pipeline check” run

Launch your selected scenario:

```bash
ros2 launch bng_bringup basic.launch.py \
  config:=techground_sbr.yaml \
  host:=<WINDOWS_BEAMNG_IP> remote:=<WSL_IP>
```

(Replace `techground_sbr.yaml` with your chosen file.)

Start logs:

```bash
ros2 run bng_simulator start_logs
```

Drive ~10 s by hand to confirm you are logging `/EGO/gtstate` and `/EGO/reduced_state` correctly.

### B. Open-loop throttle experiment (you design the throttle profile)

Goal: create data segments where the vehicle reaches **approximately steady speed** for each throttle plateau.

Constraints:
- steering must stay at 0
- no braking
- duration: 30–60 s
- do not crash or go out of the drivable area (you can reset if needed)



Skeleton (fill the TODOs; choose your own numbers):

```python
import time
import rclpy
from bng_controller.torque_speed_controller import TorqueSpeedController

if not rclpy.ok():
    rclpy.init()

HZ = 30.0 # Adjust if needed
DT = 1.0 / HZ
T_END = # Adjust total duration (30–60 s)

def throttle_profile(t: float) -> float:
    # TODO: piecewise-constant plateaus in [0, 1]
    # Include at least 4 nonzero plateaus + a final return to 0.
    return 0.0

ctl = TorqueSpeedController(vehicle_name="EGO", spin_in_thread=True)

t0 = time.time()
next_tick = t0

while True:
    t = time.time() - t0
    if t >= T_END:
        break

    u = float(throttle_profile(t))
    u = max(0.0, min(1.0, u))

    ctl.send_command(throttle=u, steering=0.0, brake=0.0)

    next_tick += DT
    time.sleep(max(0.0, next_tick - time.time()))
```

**During the run**, use PlotJuggler to confirm:
- throttle plateaus are correct (`/EGO/llc_cmd/throttle` if you send throttle directly)
- $v_x$ reaches steady-ish segments (`/EGO/reduced_state/vx`)

### C. Identify a feedforward throttle model (from your logged data)

The in-sim speed controller uses this feedforward form:

$$\text{FF}(v_{ref}, a_{ref}) = c_0 + c_1 v_{ref} + c_2 v_{ref}^2 + c_3 a_{ref} + c_4 v_{ref} a_{ref}$$

For **steady-state plateaus**, you can start with $a_{ref} \approx 0$ and identify only:

$$\text{throttle} \approx c_0 + c_1 v + c_2 v^2$$

Deliverables for this subsection:
- A scatter plot of **measured steady-state** $(v,\text{throttle})$ points
- Your fitted coefficients ($c_0, c_1, c_2, c_3, c_4$) for the full model.

Notes:
- Use `/EGO/gtstate/vel_x` for speed and `/EGO/gtstate/throttle` for the applied throttle (not the target).
- Pick data points where acceleration is not too high (you can use `/EGO/gtstate/accel_x`) when fitting.

For this lab, fit the **full** feedforward model (including $c_3$ and $c_4$), like in `test_speed_controller.ipynb`.

#### Quick example: multiple runs + regime mask + fit

The goal here is to show the *pattern*:
1) loop over multiple runs,
2) build a boolean mask to keep only the regime you want,
3) fit $c_0..c_4$ with least squares.

```python
import numpy as np

from bng_simulator.utils.logger_utils import (
    load_metadata,
    load_consolidated_data,
    load_run_data,
)

run_numbers = [12, 13, 14]  # TODO: pick runs that include varied speeds/accelerations

v_all = []
a_all = []
u_all = []

for run in run_numbers:
    gt = ...  # TODO: load the run and have it as `gt` (see past weeks / examples to remember how)

    v = np.asarray(gt["vel_x"])      # m/s
    a = np.asarray(gt["accel_x"])    # m/s^2 (or compute from v)
    u = np.asarray(gt["throttle"])   # 0..1 (or commanded throttle if you sent it)
    br = np.asarray(gt.get("brake", 0.0 * u))

    # Small steering is a common choice for FF identification
    delta = 0.5 * (np.asarray(gt["wheelFL_angle"]) + np.asarray(gt["wheelFR_angle"]))

    # TODO: pick your own mask conditions and justify them in your writeup.
    # Below is only an EXAMPLE template — you should modify/extend it.
    mask = np.ones_like(v, dtype=bool)
    mask &= (v > 2.0)            # example: moving
    mask &= (u > 0.05)           # example: actively on throttle
    mask &= (br < 1e-3)          # example: not braking
    # TODO: add at least 2 more conditions you choose (e.g., limit accel, low slip, small steering, etc.)
    mask &= (np.abs(delta) < np.deg2rad(10))  # example: small steering (adjust/replace)

    v_all.append(v[mask])
    a_all.append(a[mask])
    u_all.append(u[mask])

v = np.concatenate(v_all)
a = np.concatenate(a_all)
u = np.concatenate(u_all)

# Full model: u ≈ c0 + c1*v + c2*v^2 + c3*a + c4*v*a
X = np.column_stack([
    np.ones_like(v),
    v,
    v**2,
    a,
    v * a,
])

c, *_ = np.linalg.lstsq(X, u, rcond=None)
c0, c1, c2, c3, c4 = c

# Minimal fit-quality check (report something like this in your writeup)
# If problem with any of this, let me know.
u_hat = X @ c
rmse = float(np.sqrt(np.mean((u_hat - u) ** 2)))
ss_res = float(np.sum((u_hat - u) ** 2))
ss_tot = float(np.sum((u - float(np.mean(u))) ** 2))
r2 = float("nan") if ss_tot < 1e-12 else (1.0 - ss_res / ss_tot)
print({"c0": c0, "c1": c1, "c2": c2, "c3": c3, "c4": c4, "rmse": rmse, "R^2": r2})
```

Deliverables for the full model:
- Report $(c_0, c_1, c_2, c_3, c_4)$ and at least one plot that validates the fit (e.g., predicted vs measured throttle).
- Use these full-model coefficients when calling `send_calibration(speed_c0..speed_c4, ...)`.

### D. Upload the feedforward + PI gains to the simulator (`send_calibration`)

The Python helper exposes a tuning packet (no driving target fields) via `send_calibration(...)`.

Skeleton:

```python
import rclpy
from bng_controller.torque_speed_controller import TorqueSpeedController

rclpy.init()
ctl = TorqueSpeedController(vehicle_name="EGO", spin_in_thread=True)

ctl.send_calibration(
    speed_c0=0.0,  # TODO
    speed_c1=0.0,  # TODO
    speed_c2=0.0,  # TODO
    speed_c3=0.0,
    speed_c4=0.0,
    speedKp=0.0,   # start with Kp=0
    speedKi=0.0,   # start with Ki=0
)

ctl.close()
rclpy.shutdown()
```

### E. Test feedforward-only speed tracking (Kp = Ki = 0)

Now command a desired speed profile using `command_vehicle_speed(...)`.

For clean tuning runs, it is best to drive approximately straight. However, you do **not** need to explicitly force steering to zero in your speed command unless you are running the dedicated straight-line identification experiment.

Important: if you use sharp steps, the internal $a_{ref}$ inferred from target changes can spike. Prefer **ramps + plateaus**.

Skeleton:

```python
import time
import rclpy
from bng_controller.torque_speed_controller import TorqueSpeedController

HZ = 50.0
DT = 1.0 / HZ
T_END = 40.0

def speed_profile(t: float) -> float:
    # TODO: choose a safe speed profile with plateaus and gentle ramps
    return 0.0

rclpy.init()
ctl = TorqueSpeedController(vehicle_name="EGO", spin_in_thread=True)

t0 = time.time()
next_tick = t0

while True:
    t = time.time() - t0
    if t >= T_END:
        break

    vref = float(speed_profile(t))
    ctl.command_vehicle_speed(vref, brake=0.0)

    next_tick += DT
    time.sleep(max(0.0, next_tick - time.time()))
```

If you prefer to keep steering fixed for this specific test, you may pass `steering=0.0`, but I would encourage not providing it such that ater when collecting lateral data, ou can easily drive around while the controller handles speed.

PlotJuggler plots to use while tuning:
- `/EGO/llc_cmd/vehicle_speed` (your target)
- `/EGO/reduced_state/vx` (measured)
- `/EGO/reduced_state/throttle` (what the sim applied)

Deliverables:
- A plot of $v_{ref}(t)$ and $v_x(t)$ showing **feedforward-only** tracking
- A short discussion: where does FF help, and where does it fail?

### F. Add PI feedback and tune $(K_p, K_i)$

Set your identified $(c_0..c_4)$ and then tune gains:
- start with small positive $K_p$
- add small $K_i$ to reduce steady-state error
- avoid oscillation (speed hunting) and excessive throttle chatter

Deliverables:
- Your final $(c_0..c_4, K_p, K_i)$
- A “best” tracking plot (target vs measured speed) and a throttle plot
- A short paragraph explaining how you tuned (what you changed and why)

Additionally, analyze the impact of $K_p$ and $K_i$:
- Show at least one comparison plot: FF-only vs +$K_p$ vs +$K_p$+$K_i$
- Comment on rise time, overshoot, steady-state error, and any oscillations / throttle chatter

Also analyze **what happens in a turn**:
- Do a short run where you intentionally steer (e.g., a gentle constant-radius turn or a few left/right turns) while holding a constant $v_{ref}$.
- Plot $v_x(t)$ and throttle during the turning segments.
- Discuss whether the controller maintains speed in the turn; if not, explain why (e.g., throttle saturation, traction limits, tire slip, or the fact that longitudinal and lateral dynamics are coupled).

### G (Graduate track only). Implement the controller in Python (send throttle)

In the graduate track, you implement:

$$u = \mathrm{sat}_{[0,1]}\Big(\underbrace{c_0 + c_1 v_{ref} + c_2 v_{ref}^2}_{\text{FF (steady-state)}} + K_p (v_{ref}-v) + K_i \int (v_{ref}-v) dt\Big)$$

Minimal skeleton (use whatever state feedback you already used in Week 4 / the examples):

```python
import time
import rclpy
from bng_controller.torque_speed_controller import TorqueSpeedController

HZ = 50.0
DT = 1.0 / HZ
T_END = 40.0

def vref_profile(t: float) -> float:
    return 0.0  # TODO

def ff_throttle(vref: float) -> float:
    # TODO: use your fitted c0,c1,c2 (and optionally c3,c4)
    return 0.0

rclpy.init()
ctl = TorqueSpeedController(vehicle_name="EGO", spin_in_thread=True)

Kp = 0.0  # TODO
Ki = 0.0  # TODO
err_int = 0.0

t0 = time.time()
next_tick = t0

while True:
    t = time.time() - t0
    if t >= T_END:
        break

    state = ctl.get_latest_state_dict()
    if state is None:
        time.sleep(0.01)
        continue

    v = float(state["vx"])
    vref = float(vref_profile(t))

    err = vref - v
    err_int = err_int + err * DT

    u = ff_throttle(vref) + Kp * err + Ki * err_int
    u = max(0.0, min(1.0, u))

    ctl.send_command(throttle=u, brake=0.0)

    next_tick += DT
    time.sleep(max(0.0, next_tick - time.time()))

```

---

## Part 2 (Week 6) — Pure pursuit again, now with your speed controller

Last week, many cars “looked bad” because the **wheels were not spinning correctly** under the old speed control. This was intentional, it was not a proper speed controller.

Goal: rerun the Week 4 scenario and compare behavior before/after your new speed controller.

### Tasks

1) Use the pre-made pure pursuit scenario config that matches your Part 1 vehicle:
    - SBR: [src/bng_xal/bng_bringup/config/scenarios/pure_pursuit_sbr.yaml](../../src/bng_xal/bng_bringup/config/scenarios/pure_pursuit_sbr.yaml)
    - Sunburst2: [src/bng_xal/bng_bringup/config/scenarios/pure_pursuit_sunburst2.yaml](../../src/bng_xal/bng_bringup/config/scenarios/pure_pursuit_sunburst2.yaml)
    - ETK800: [src/bng_xal/bng_bringup/config/scenarios/pure_pursuit_etk800.yaml](../../src/bng_xal/bng_bringup/config/scenarios/pure_pursuit_etk800.yaml)

2) Launch it (same `host:=...` and `remote:=...` workflow as Week 4):

```bash
ros2 launch bng_bringup basic.launch.py \
  config:=pure_pursuit_sbr.yaml \
  host:=<WINDOWS_BEAMNG_IP> remote:=<WSL_IP>
```

(Replace `pure_pursuit_sbr.yaml` with the config you selected.)

3) Run your pure pursuit code from Week 4, but use your **newly calibrated speed controller**:
        - In your pure pursuit script, **upload the calibration you found in Part 1** *before* starting the control loop:
            call `send_calibration(speed_c0..speed_c4, speedKp, speedKi)` with your fitted coefficients and tuned PI gains.
        - Then, during pure pursuit, keep sending **speed targets** (and steering targets) exactly like Week 4.
            For example, if your Week 4 code already uses `send_command(vehicle_speed=..., steering=...)` (or a wrapper like `send_vehicle_speed(...)`), continue using that — but now it will use your calibrated FF+PI to produce better actuation.
4) Record a run with logging enabled.

### What to report

- A trajectory plot (reference path + driven path)
- A speed plot ($v_{ref}(t)$ and $v_x(t)$)
- A comparison to last week: overlay (or side-by-side) **Week 4 vs Week 5–6** for both trajectory and speed tracking.
- A short discussion: what changed vs Week 4? (wheel spin, cornering behavior, stability, tracking quality)

---

## Part 3 (Week 6) — Invert bicycle dynamics to estimate lateral tire forces vs slip angles

### Goal

Using your working speed controller, you will collect low-to-moderate slip data at several speeds, then estimate front/rear lateral forces and plot **tire curves**:

- $F_{yf}$ vs $\alpha_f$
- $F_{yr}$ vs $\alpha_r$

This is not a “perfect tire model” lab. The goal is a clean, defensible workflow and curves that make physical sense.

### A. Data collection experiment design

Use a flat/open area (your `tech_ground` scenario is fine) and perform a **repeatable steering maneuver** at several constant speeds.

Requirements:
- Choose at least **3 speeds** (e.g., low/medium/high within safe limits)
- For each speed, run at least **15–25 s** of steering excitation
- Keep slip moderate (avoid drifting/spinning)

Steering input choice:
- You may automate steering (sinusoid/chirp), or you may **steer manually** while your speed controller holds a steady speed.
- Either approach is acceptable, but you must explain your choice and show that you covered multiple speeds with moderate slip.

Suggested input design:
- Hold $v_{ref}$ constant per run (your speed controller should maintain it)
- Apply a steering input that is **large enough to reveal the nonlinear “S-shape”** in the tire curve, but still **avoid sustained sliding/drifting**.
    (In practice: increase steering until slip angles are clearly nonzero and the curve bends, then back off if you start to spin / drift.)

Log at a reasonable rate (50 Hz recommended).

### B. Compute slip angles

Use the standard bicycle slip-angle definitions (body-frame velocities at axles):

$$\alpha_f = \delta - \arctan2(v_y + a r,\ v_x)$$
$$\alpha_r = -\arctan2(v_y - b r,\ v_x)$$

Where:
- $v_x, v_y$ are CG velocities in the body frame
- $r$ is yaw rate
- $\delta$ is road-wheel steer angle (front)
- $a,b$ are CG-to-axle distances from your chosen vehicle YAML

Use signals from:
- Use `/EGO/gtstate` for offline analysis: `vel_x`, `vel_y`, `angVel_z`, and the front wheel angles (average `wheelFL_angle` and `wheelFR_angle` to compute $\delta$).

### C. Invert the lateral dynamics to estimate $F_{yf}$ and $F_{yr}$

Use the planar bicycle equations (lateral force balance + yaw moment balance), keeping the steering geometry term:

$$m(\dot{v}_y + v_x\,r) = F_{yf}\cos(\delta) + F_{yr}$$
$$I_z \dot{r} = a F_{yf}\cos(\delta) - b F_{yr}$$

Practical notes:
- You will need physical parameters from your vehicle YAML:
    - `totalMass` ($m$)
    - `inertia.zz` ($I_z$)
    - `cogToFrontAxle` ($a$)
    - `cogToRearAxle` ($b$)
    See: [src/bng_xal/bng_bringup/config/vehicles/](../../src/bng_xal/bng_bringup/config/vehicles/)
- If you want to stay fully within `/EGO/gtstate`, compute
    $$a_y = \dot{v}_y + v_x\,r$$
    numerically from `vel_y`, `vel_x`, `angVel_z`.
- You still need $\dot{v}_y$ and $\dot{r}$; compute them numerically **with smoothing** to avoid amplifying noise.

Example derivative estimation with a Savitzky–Golay filter (pattern only):

```python
import numpy as np
from scipy.signal import savgol_filter

# Inputs (from your logs)
t = np.asarray(time_s)          # seconds, uniform-ish
vy = np.asarray(vel_y)          # m/s
r  = np.asarray(yaw_rate_r)     # rad/s

dt = float(np.median(np.diff(t)))
win = 21         # must be odd; tune based on sample rate and noise
poly = 3

vy_dot = savgol_filter(vy, window_length=win, polyorder=poly, deriv=1, delta=dt)
r_dot  = savgol_filter(r,  window_length=win, polyorder=poly, deriv=1, delta=dt)
```

Closed-form inversion (no matrix solve):

**Do this by hand once:** Starting from the two equations above, derive explicit formulas for $F_{yf}$ and $F_{yr}$ (as functions of $m,I_z,a,b,\delta,\dot{v}_y,\dot{r},v_x,r$).

Deliverable: include your algebra in the report, and then use your closed-form expressions to compute $F_{yf}(t)$ and $F_{yr}(t)$ **for every valid time sample**.

### Required: mask your samples (like Part 1)

Do **not** invert forces on “bad” samples. Build a boolean mask (same idea as the feedforward identification mask) and justify your thresholds.

At minimum, include gates like:
- moving: $v_x > v_{min}$ (avoid divide-by-near-zero issues)
- low longitudinal transients: $|a_x|$ small (use `/EGO/gtstate/accel_x`)
- low longitudinal slip: $|\kappa_f|$ and $|\kappa_r|$ small (avoid combined-slip / traction-limited samples)
- No braking: brake command near zero

For $\kappa_f,\kappa_r$, compute them similarly to Week 3 using wheel linear speeds and axle longitudinal speeds. (Typical pattern: average `wheelFL_speed`/`wheelFR_speed` to get front wheel linear speed, average `wheelRL_speed`/`wheelRR_speed` for rear.)

Mask template (pattern only — pick your own thresholds and justify them):

```python
mask = np.ones_like(vx, dtype=bool)

mask &= (vx > 2.0)                       # moving
mask &= (np.abs(accel_x) < 1.0)          # low longitudinal transients
mask &= (np.abs(kappa_f) < 0.03)         # near-zero longitudinal slip (front)
mask &= (np.abs(kappa_r) < 0.03)         # near-zero longitudinal slip (rear)
mask &= (np.abs(delta) < np.deg2rad(25)) # optional: avoid extreme steering
```

### D. Plot the tire curves

Concatenate all your valid $(\alpha_f, F_{yf})$ and $(\alpha_r, F_{yr})$ points across all speeds, then make scatter plots:
- plot $F_{yf}$ vs $\alpha_f$ (scatter)
- plot $F_{yr}$ vs $\alpha_r$ (scatter)


Deliverables:
- Your computed $(\alpha_f, F_{yf})$ and $(\alpha_r, F_{yr})$ plots
- A short discussion of limitations (noise, load transfer not modeled, nonlinearity at higher slip, etc.)

---

## Final Submission checklist

Your PDF must include:
- Part 1: throttle plateau design, FF fit plot + coefficients, feedforward-only tracking plot, final PI tuning plots and final gains
- Part 2: pure pursuit results with the new speed controller (path + speed plots + discussion)
- Part 3: clear definition of slip angles, equations used for inversion, and final tire curves
