# Scenario and config overrides

How to change spawn, sensors, LLC, and simulator settings **without editing Python or Lua**.

Config lives under `bng_bringup/config/`. After a symlink install of `bng_bringup`, edits take effect on relaunch — no rebuild.

See also:

- [compose_reference.yaml](runs/compose_reference.yaml) — run-file knobs and launch examples
- [bng_bringup README](../README.md) — layout, presets, WSL networking
- Root [README](../../../../README.md) — build workflow and architecture

---

## How composition works

A thin **run file** (`config/runs/*.yaml`) is merged with **fragments** by `scenario_compose`:

```
run YAML  →  preset (optional)  →  launch args (shorthand only)
                ↓
         scenario_compose
                ↓
    sim + level + vehicles + defaults
                ↓
         overrides (deep-merge)
```

### CREATE profile (default)

Spawns vehicles with the full xlab stack:

| Fragment | File | Contents |
|----------|------|----------|
| Simulator | `defaults/sim.yaml` (or `compose.sim`) | BeamNG connection, timing |
| Map | `levels/<level>.yaml` | Spawn presets, `extra_objects` |
| Vehicle model | `vehicle_catalog.yaml` | Model, `steering_to_input`, `torque_map` |
| LLC + sensors | `defaults/llc.yaml` | `gtstate`, `roof_imu`, `LowLevelController` |
| ROS poll | `defaults/ros_poll.yaml` | GE/TCP poll timers (not UDP) |
| UDP slots | `defaults/udp_io.yaml` | `port_stride`; launch sets base ports |

### ATTACH profile

Grafts minimal sensors onto an **existing in-game vehicle id** — no spawn, **no LLC**, no UDP golden path.

| Fragment | File |
|----------|------|
| Sensors | `defaults/attach_sensors.yaml` |

Use `config:=attach.yaml` and set `compose.vehicle_id` (or `compose.vehicles[]`) to the BeamNG id (e.g. `thePlayer`).

---

## Merge priority (later wins)

For each CREATE vehicle, settings are layered in this order:

1. `defaults/llc.yaml` — global baseline
2. `vehicle_catalog.yaml` — per-model `steering_to_input`, `torque_map`, optional `sensors` / `controllers` / `llc_overrides`
3. `compose.vehicles[].llc_overrides` — **controllers only**
4. Run `overrides` — deep-merge onto the full composed config (see below)

Launch CLI args (`level`, `spawn`, `yaw`, `pos`, …) only patch **compose shorthand** (single-vehicle runs). They do **not** change LLC or sensor params.

User presets merge **before** launch args: `base run → preset → launch args`.

---

## Recommended: run-level `overrides`

Best for experiment-specific tweaks. Add an `overrides:` block to your run YAML (or to a user preset).

### Sensor tuning (gtState filters, attitude, debug)

Field names match [defaults/llc.yaml](defaults/llc.yaml) / [defaults/attach_sensors.yaml](defaults/attach_sensors.yaml) under `sensors.gtstate`.

| Key | Role |
|-----|------|
| `accel_tau_s` / `gyro_tau_s` / `vel_tau_s` / `wheel_angvel_tau_s` | EMA time constants [s] |
| `attitude_mode` | `triangle` (raw attach axes) or `integrate` (curl ω + slow pull to triangle; kills HF flex on `vy`/quat) |
| `attitude_tau_s` | Absolute pull to triangle [s]; used only when `attitude_mode: integrate` |
| `attach_z_offset` | [m] shift attach search along vehicle up (usually `0.0`; non-zero enables report-point transport) |
| `debug_raw` | Extra Lua fields (`velRaw`, `velTri`, `angVelUncorr`, …) for offline plots |
| `physics_update_time` | Physics-side filter step [s] |

```yaml
# config/runs/my_experiment.yaml
compose:
  vehicle: utv_wild
  vehicle_id: EGO
  level: tech_ground
  spawn: origin

scenario_mode: create

overrides:
  vehicles:
    EGO:
      sensors:
        gtstate:
          accel_tau_s: 0.01
          gyro_tau_s: 0.005
          vel_tau_s: 0.005
          physics_update_time: 0.005
          attitude_mode: integrate   # or triangle for legacy axes
          attitude_tau_s: 0.3
          attach_z_offset: 0.0
          debug_raw: true            # then: ros2 run bng_controller plot_gtstate_debug
```

Multi-vehicle: use one block per id (`EGO`, `AGENT2`, …). See [multi_agent.yaml](runs/multi_agent.yaml).

### LLC controller tuning

```yaml
overrides:
  vehicles:
    EGO:
      controllers:
        LowLevelController:
          controllerRate: 0.01
          calibration:
            control_mode: full
            command_timeout: 4.0
            verbose: false
            gains:
              kp: 4027.68
              ki: 12.95
              steer_pi_enable: 0.0
```

Field names match [defaults/llc.yaml](defaults/llc.yaml) under `controllers.LowLevelController`.

### Simulator / scenario knobs

```yaml
overrides:
  beamng:
    setup_funcs:
      beamng.settings.set_deterministic:
        steps_per_second: 60
```

### ROS poll (GE/TCP path only)

Does **not** affect BeamNG LLC or UDP observations. Overrides poll rate and whether `sim_manager_node` publishes ROS topics:

```yaml
overrides:
  ros_poll:
    EGO:
      gtstate:
        poll_time: 0.1
        publish: 1
```

Flat keys (`gtstate: …` without a vehicle id) work only when the run has a **single** vehicle.

---

## User preset with overrides

Keep spawn/level in a preset; add LLC or sensor overrides there instead of a new run file.

```yaml
# config/presets/user/derby_fast_gyro.yaml
level: derby
spawn: grid_lane

overrides:
  vehicles:
    EGO:
      sensors:
        gtstate:
          gyro_tau_s: 0.002
```

```bash
ros2 launch bng_bringup simulator.launch.py preset:=derby_fast_gyro.yaml
```

Preset search: `config/presets/user/` → `~/.config/bng_bringup/presets/` → absolute path.

Presets cannot define `compose.vehicles[]` (multi-vehicle runs must edit the run YAML).

---

## Per-vehicle `llc_overrides` (controllers only)

In the run file, under `compose.vehicles[]`:

```yaml
compose:
  vehicles:
    - id: EGO
      vehicle: utv_wild
      spawn: origin
      llc_overrides:
        LowLevelController:
          calibration:
            control_mode: torque
```

This merges into `controllers` only. For **sensor** params use `overrides.vehicles.<id>.sensors` (above).

---

## Vehicle catalog (all runs using that model)

Put calibration artifacts and model-wide defaults in [vehicle_catalog.yaml](vehicle_catalog.yaml):

```yaml
catalog:
  utv_wild:
    model: utv
    part_config: /vehicles/utv/wild.pc
    steering_to_input: -0.5948683325943701   # → LLC gains.steering_to_input
    torque_map: utv_wild_drivetrain          # → LLC + gtstate torque_map
    llc_overrides:
      LowLevelController:
        calibration:
          verbose: true
    sensors:
      gtstate:
        debug_raw: false
```

Use the catalog when the change belongs to the **vehicle model**, not a single experiment.

---

## Edit `defaults/` in place (last resort)

Change the global baseline when every CREATE (or ATTACH) run should pick up the same value.

| File | Affects |
|------|---------|
| [defaults/llc.yaml](defaults/llc.yaml) | All CREATE runs — sensors + LLC |
| [defaults/attach_sensors.yaml](defaults/attach_sensors.yaml) | All ATTACH runs |
| [defaults/ros_poll.yaml](defaults/ros_poll.yaml) | Auto ROS poll config |
| [defaults/sim.yaml](defaults/sim.yaml) | Simulator connection |
| [defaults/udp_io.yaml](defaults/udp_io.yaml) | UDP port stride |

Prefer run `overrides` or catalog entries so `defaults/` stays the shared golden path.

---

## Launch CLI (spawn / vehicle only)

Single-vehicle shorthand runs only (`gridworld.yaml`, `derby_utv.yaml`, …):

```bash
ros2 launch bng_bringup simulator.launch.py \
  config:=gridworld.yaml \
  level:=derby spawn:=grid_lane yaw:=45 \
  pos:="44.05,127.22,79.3"
```

| Arg | Effect |
|-----|--------|
| `config` | Run file under `config/runs/` |
| `preset` | User preset overlay |
| `vehicle`, `vehicle_id`, `level`, `spawn`, `yaw`, `pos` | Patch `compose` shorthand |

Multi-vehicle runs (`compose.vehicles[]`): edit the run YAML; launch overrides are **rejected**.

Infrastructure args (`host`, `remote`, `bng_listen_port`, …) are not scenario content — see [bng_bringup README](../README.md).

---

## Quick decision guide

| Goal | Where to edit |
|------|----------------|
| New map / spawn for one experiment | Run `compose.level` / `compose.spawn`, or `preset:=` |
| Tune gtState filters / attitude / debug for one run | `overrides.vehicles.<id>.sensors.gtstate` in run or preset |
| Tune LLC gains / control_mode for one run | `overrides.vehicles.<id>.controllers.LowLevelController` |
| Calibrated `steering_to_input` for a model | `vehicle_catalog.yaml` |
| Change default LLC for all CREATE runs | `defaults/llc.yaml` |
| Attach to freeroam vehicle, tweak gtState | `overrides` or `defaults/attach_sensors.yaml` |
| ROS topic poll rate (not UDP) | `overrides.ros_poll` or `defaults/ros_poll.yaml` |

---

## Verify composed config

On launch, `sim_manager_node` logs a scenario manifest (vehicle ids, spawn, LLC summary, ROS topics). For offline inspection:

```python
from bng_simulator.utils.scenario_compose import compose_scenario, summarize_config
cfg = compose_scenario("gridworld.yaml")
print(summarize_config(cfg, config_path="gridworld.yaml"))
```

Inspect `cfg["vehicles"]["EGO"]["sensors"]` and `cfg["vehicles"]["EGO"]["controllers"]` to confirm overrides landed.

---

## Common pitfalls

- **ATTACH has no LLC** — `defaults/llc.yaml` is not used; only `attach_sensors.yaml`.
- **`llc_overrides` is controllers-only** — use `overrides.vehicles.<id>.sensors` for gtState.
- **`overrides.ros_poll` ≠ sensor physics** — it only controls GE/TCP polling and optional ROS publishing.
- **Config edits ignored** — ensure `install/share/bng_bringup/config` is a symlink (root README step 4).
- **Multi-vehicle + flat `ros_poll`** — use `overrides.ros_poll.<vehicle_id>.<sensor>`.
