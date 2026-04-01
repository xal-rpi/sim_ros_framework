# Week 10 — Trajectory Generation via Nonlinear Optimisation

**Theme:** generate minimum-time race lines on an oval track by formulating and solving a nonlinear program (NLP) using the bicycle dynamics model from previous weeks.

**Due Date:** 5 PM Today.

**Submission:** turn in your code and figures in one pdf/word document.

---

## Learning goals

By the end of this lab, you should be able to:

1. Formulate a trajectory-generation problem as a nonlinear programme (NLP).
2. Define a reference track from a curvature profile and set lateral bounds (track edges).
3. Understand the change of independent variable from time to path progress $s$.
4. Impose physically meaningful constraints: actuator limits, track boundaries, acceleration bounds.
5. Interpret the effect of each constraint on the resulting trajectory.
6. Visualise and analyse the generated race line — speed profile, lateral position, tire utilisation.

---

## Setup

```bash
cd ~/ros2_ws/src/sim_ros_framework

git checkout hpa-s26
git pull origin hpa-s26

python -m pip install -r requirements.txt
```

Make sure CasADi and IPOPT are available:

```bash
python -c "import casadi; print(casadi.__version__)"
```

All work for this lab takes place in `labs/week10/`.

---

## Helper API You Will Use

The helper for this lab is in [traj_opt_helper.py](traj_opt_helper.py). The vehicle dynamics model is in [casadi_dynamics.py](casadi_dynamics.py).

In particular, the parameters collected in `fiala_params` inside [casadi_dynamics.py](casadi_dynamics.py) are the identified tire and vehicle parameters for the SBR vehicle. You should think of these as the fitted parameters from the previous labs on tire-force modelling, now packaged into a dynamics model that you can optimise over.

The main classes are:

| Class / Function | What it does |
|---|---|
| `TrackDefinition` | Defines a track: curvature profile, lateral bounds, initial pose |
| `TrackDefinition.oval(...)` | Builds an oval track with lead-in/lead-out straights |
| `TrackDefinition.plot()` | Preview the track geometry |
| `TrajOptConfig` | Solver settings: node count, start velocity, initial-state bounds, IPOPT verbosity |
| `TrajOptProblem` | Sets up the NLP from a track definition |
| `TrajOptProblem.add_accel_bounds(...)` | Add acceleration constraints on a range of $s$ |
| `TrajOptProblem.set_e_bounds(...)` | Override lateral bounds on a range of $s$ |
| `TrajOptProblem.set_state_range(...)` | Override global range for any state/control |
| `TrajOptProblem.solve()` | Solve the NLP; returns a `TrajOptResult` |
| `TrajOptResult` | Contains the optimal trajectory with plotting methods |

Import:

```python
from traj_opt_helper import (
    TrackDefinition,
    TrajOptProblem,
    TrajOptConfig,
    TrajOptResult,
    StateIdx,
    ControlIdx,
)
```

You are expected to inspect [traj_opt_helper.py](traj_opt_helper.py) while doing the lab. In particular:
- look at `TrajOptResult` to see which signals are already exposed as properties
- look at `plot_states()` to see how state and control columns are mapped
- use `result.x[:, StateIdx....]` and `result.u[:, ControlIdx....]` when you want direct access to the raw arrays

---

## Review from lecture

### Change of variables: time → path distance

The vehicle dynamics in time are $\dot{x} = f(x, u)$. To work along the path we use $s$ as the independent variable:

$$
\frac{dx}{ds} = \frac{f(x, u)}{\dot{s}}, \qquad \dot{s} = \frac{V \cos(\Delta\psi)}{1 - e\,\kappa(s)}
$$

Time becomes a state:

$$
\frac{dt}{ds} = \frac{1}{\dot{s}}
$$

The minimum-time cost is simply $J = t(s_{\text{final}}) - t(0)$.

### State and control vectors

The dynamics model in [casadi_dynamics.py](casadi_dynamics.py) is a rear-wheel-drive bicycle with Fiala tires. The state and control layout is:

**States** ($x \in \mathbb{R}^{10}$):

| Index | Name | Description | Units |
|:---:|---|---|---|
| 0 | `s` | path progress | m |
| 1 | `t` | time | s |
| 2 | `r` | yaw rate | rad/s |
| 3 | `V` | speed | m/s |
| 4 | `beta` | side-slip angle | rad |
| 5 | `wr` | rear wheel speed | m/s |
| 6 | `e` | lateral deviation from centre-line | m |
| 7 | `dphi` | heading error | rad |
| 8 | `delta` | road-wheel (steering) angle | rad |
| 9 | `rear_wheel_torque` | rear wheel torque | Nm |

**Controls** ($u \in \mathbb{R}^{2}$):

| Index | Name | Description | Units |
|:---:|---|---|---|
| 0 | `delta_r` | steering rate | rad/s |
| 1 | `rear_wheel_torque_r` | rear wheel torque rate | Nm/s |

The controls are **rates**. Steering angle and rear wheel torque are states that evolve according to the rate commands. This makes the control smooth by construction.

### Scaling

All variables are scaled internally by a characteristic magnitude so that $\hat{x}_i \approx O(1)$. You work in **physical units** — the helper handles scaling.

### Constraints

The NLP enforces:
- **Dynamics** as equality constraints (implicit Euler collocation)
- **Box constraints** on every state and control (actuator limits, speed, etc.)
- **Track boundaries**: $e_{\min}(s) \le e \le e_{\max}(s)$

You will add constraints on top of these.

---

## Part 1 — Visualise the Track (Warm-up)

### Goal

Build the default oval track, visualise it, and understand the geometry.

### Tasks

1. Create a `TrackDefinition` using the `oval(...)` factory:

```python
track = TrackDefinition.oval(
    straight_length=20.0,
    radius=15.0,
    n_laps=2,
    e_half_width=4.0,
    lead_in=50.0,
    lead_out=50.0,
)
```

2. Plot the track:

```python
track.plot()
```

3. On a separate figure, plot the curvature profile $\kappa(s)$ and the lateral bounds $e_{\min}(s)$, $e_{\max}(s)$ versus arc length $s$.

Hint: inspect the `TrackDefinition` fields and helpers directly. Useful entries are `track.s_start`, `track.s_end`, `track.s`, `track.kappa`, `track.e_min`, `track.e_max`, plus interpolation helpers `track.kappa_at(...)` and `track.e_bounds_at(...)`.

### Questions

- What is the total path length?
- What does the curvature look like in the turns? In the straights?
- What do the lateral bounds represent physically?

### Deliverables

- One 2D track plot
- One plot of $\kappa(s)$ versus $s$
- One plot of $e_{\min}(s)$, $e_{\max}(s)$ versus $s$
- Answers to the three questions above

---

## Part 2 — Generate a Baseline Race Line

### Goal

Solve the trajectory optimisation with default settings and analyse the result.

### Tasks

1. Create the problem and solve with default configuration:

```python
config = TrajOptConfig(
    num_nodes=200,
    start_velocity=(3.0, 5.0),
    initial_state_bounds={
        "e": (-0.05, 0.05),
        "beta": (-0.05, 0.05),
        "delta": (-0.02, 0.02),
        "r": (-0.01, 0.01),
        "rear_wheel_torque": (0.0, 400.0),
    },
)
problem = TrajOptProblem(track, config)
result = problem.solve()
```

Choose your initial conditions deliberately. In particular, keep the initial speed in the 3–5 m/s range or above; if you start too close to zero speed then $\dot{s}$ becomes too small and the path-domain dynamics become numerically fragile.

2. Plot the state and control profiles:

```python
result.plot_states()
```

3. Plot the trajectory on the track with vehicle overlays:

```python
result.plot_with_vehicles(
    s_range=(track.s_start, track.s_end),
    step=8,
    color_state="V",
    vehicle_scale=2.0,
)
```

Interpretation of arguments:
- `s_range`: path interval to display
- `step`: draw one vehicle every `step` samples (smaller means denser overlays)
- `color_state`: any `TrajOptResult` property used to color the trajectory (`"V"`, `"beta"`, `"rear_wheel_torque"`, `"wr"`, etc.)
- `vehicle_scale`: visual scale for the drawn car outline

4. Report:
   - the total time $t(s_{\text{final}})$
   - the maximum speed
   - the maximum lateral deviation $e$

Clue: you can read these directly from the solved object (not only from plots). For example:

```python
total_time = result.t[-1]
max_speed = result.V.max()
max_abs_e = np.abs(result.e).max()

print(total_time, max_speed, max_abs_e)
```

If you need any other signal, inspect `result.x[:,StateIdx.xx]` with `StateIdx` (for states) and `result.u[:,ControlIdx.xx]` with `ControlIdx` (for controls).

### Questions

- Does the optimizer use the full track width?
- Where does the car go fast? Where does it slow down?
- Look at the side-slip angle $\beta$. Is the car drifting?

For this lab, call it drifting only if the body is noticeably misaligned with the direction of travel for a sustained portion of the turn, meaning $\beta$ is clearly nonzero and not just a small transient cornering slip.

### Deliverables

- State/control profile plots
- `plot_with_vehicles(...)` trajectory plot
- Answers to the questions above

---

## Part 3 — Understanding and Adding Constraints

You will **modify the problem** by adding constraints one at a time and observing the effect.

### 3A — Acceleration bounds on the lead-in straight

The lead-in straight (from $s = 0$ to $s = 50$ m) is where the car accelerates from rest. Without constraints, the optimizer may request unrealistic acceleration.

**Task:**

1. Add an acceleration bound on the lead-in straight:

```python
problem = TrajOptProblem(track, config)
problem.add_accel_bounds(a_min=0.0, a_max=5.0, s_range=(5.0, 50.0))
result = problem.solve()
```

2. Compare the speed profile $V(s)$ with and without the bound.

**Question:** In physical terms, what does `a_max = 5.0` correspond to? How many g's is that?

### 3B — Narrower track boundaries

**Task:**

1. Tighten the track to $\pm 2\,\text{m}$ on one of the turn segments:

```python
problem = TrajOptProblem(track, config)
problem.set_e_bounds(e_min=-2.0, e_max=2.0, s_range=(60.0, 120.0))
result = problem.solve()
```

2. Plot the 2D trajectory and states, and observe the difference in the constrained section.

**Question:** Does the optimizer still cut corners? How does the speed profile change?

### 3C — Turn engine braking off: non-negative rear wheel torque

The default bounds already allow negative rear wheel torque, which acts like engine braking. In this part, turn engine braking off by requiring non-negative rear wheel torque:

```python
problem = TrajOptProblem(track, config)
problem.set_state_range("rear_wheel_torque", lb=0.0, ub=3500.0)
result = problem.solve()
```

**Questions:**
- Does the car still complete the track?
- Where does the speed peak? Where is it lowest?
- What happens to the lateral position through the turns — does the optimizer use the full width?

### 3D — Combine constraints (your design)

Now combine at least **two** of the constraints above (acceleration bounds + narrow track, or engine-braking-off + narrow track, etc.) and solve again.

**Task:**

Create your own constrained problem. You may also try:
- limiting steering rate: `problem.set_state_range("delta_r", lb=-0.3, ub=0.3)`
- limiting maximum speed: `problem.set_state_range("V", lb=0.5, ub=15.0)`

Solve and compare.

**Deliverables for Part 3:**
- For each sub-part (3A, 3B, 3C): one 2D plot and one speed and beta, and e-profile plot comparing constrained vs. unconstrained
- For 3D: one 2D plot and a short description of what you chose and what happened
- Answers to all questions

The sample code below  shows how to plot multiple states together for comparison. You can adapt this to plot the constrained vs. unconstrained trajectories on the same axes.
```Python
    s = result.s
    fig, axs = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    axs = axs.flatten()

    axs[0].plot(s, result.V, lw=2)
    axs[0].set_ylabel("V (m/s)")
    axs[0].set_title("Speed")
    axs[0].grid(True, alpha=0.4)

    e_lo, e_hi = result.track.e_bounds_at(s)
    axs[1].plot(s, result.e, lw=2, label="e")
    axs[1].fill_between(s, e_lo, e_hi, color="red", alpha=0.15, label="bounds")
    axs[1].set_ylabel("e (m)")
    axs[1].set_title("Lateral Deviation")
    axs[1].legend()
    axs[1].grid(True, alpha=0.4)

    axs[2].plot(s, result.beta, lw=2)
    axs[2].set_ylabel("beta (rad)")
    axs[2].set_xlabel("s (m)")
    axs[2].set_title("Side-slip")
    axs[2].grid(True, alpha=0.4)

    axs[3].plot(s, result.rear_wheel_torque, lw=2)
    axs[3].set_ylabel("rear_wheel_torque (Nm)")
    axs[3].set_xlabel("s (m)")
    axs[3].set_title("Rear Wheel Torque")
    axs[3].grid(True, alpha=0.4)

    fig.suptitle(title)
    plt.tight_layout()
```
---

## Part 4 — Force Analysis and GG / Friction-Circle Plots

### Goal

Use force-level diagnostics to understand whether the optimized trajectory is near tire limits.

### Tasks

1. You already used `plot_with_vehicles(...)` in Part 2. Keep using it when needed for geometry checks.

2. Evaluate tire-force quantities using the helper utility:

```python
force_data = result.compute_tire_forces()
```

This returns arrays such as:
- `force_data["s"]`
- `force_data["Fxf"]`, `force_data["Fyf"]`, `force_data["Fxr"]`, `force_data["Fyr"]`
- `force_data["Fzf"]`, `force_data["Fzr"]`
- `force_data["alpha_f"]`, `force_data["alpha_r"]` (slip angles)
- `force_data["kappa_x_front"]`, `force_data["kappa_x_rear"]` (longitudinal slip ratios)
- `force_data["front_util"]`, `force_data["rear_util"]` (normalized friction usage)
- `force_data["gg_long"]`, `force_data["gg_lat"]`

3. Plot the normalized friction-circle usage:

```python
result.plot_friction_circle(force_data)
```

4. Plot the GG diagram:

```python
result.plot_gg_diagram(force_data)
```

5. Plot force versus slip-angle and slip-ratio maps:

```python
result.plot_force_vs_alpha_slip(force_data)
```

A GG diagram is a scatter plot of longitudinal and lateral acceleration in units of gravity, i.e. $a_x/g$ versus $a_y/g$. Each point is one operating condition of the car along the trajectory. It tells you how the vehicle trades braking/traction (x-axis) against cornering (y-axis), and where the trajectory is pushing close to the combined grip limits.

6. (Optional) If you still want an additional geometry plot for this part, use:

```python
result.plot_with_vehicles(
    s_range=(50.0, 200.0),
    step=8,
    color_state="V",
    vehicle_scale=2.0,
)
```

7. Interpret the plots:
- Where is rear tire usage highest?
- Are front or rear points touching or exceeding the unit friction circle?
- What parts of the path correspond to extreme `a_x/g` or `a_y/g` in the GG plot?
- In the force-vs-slip plots, where do you see nonlinear/saturated behavior?

### Deliverables

- One friction-circle figure
- One GG figure
- One force-vs-slip/alpha figure
- A short paragraph (4–6 sentences) interpreting where and why force usage is highest

---

## Part 5 — Exploration (Optional)

Choose **one** of the following and write a short analysis (3–5 sentences):

### Option A: How does node count affect the solution?

Run the same problem with `num_nodes` = 100, 200, and 400. Compare:
- Solve time
- Trajectory smoothness
- Total time $t(s_{\text{final}})$

Can you create a situation where the otpimization would struggle to find a solution by enforcing constraints that may not be physically feasible or not feasible at low node counts?


### Option B: Custom track shape

The map/setup used so far likely does not produce strong operation at the tire limits over most of the trajectory. One way to improve that is to extend or redesign the map so the optimizer sees longer/faster sections and tighter transitions.

In later extensions of this lab, we will also add braking and drifting scenarios, which should produce much clearer operation near the limits.

Build a non-oval track using `TrackDefinition.from_kappa_function(...)` with your own curvature function. Solve and show the result.

```python
def my_kappa(s):
    # Your curvature profile here
    ...

track = TrackDefinition.from_kappa_function(
    my_kappa, s_start=0.0, s_end=300.0,
    e_half_width=5.0,
)
```

### Deliverables

- Plots or a table for the option you chose
- 3–5 sentence analysis

---
