# bng_bringup

Launch files and **all scenario configuration** for the xlab research stack.

## Quick start

```bash
ros2 launch bng_bringup simulator.launch.py
```

Omitting `config:=` selects `runs/single_vehicle.yaml`: Covet Turbo Rally on Tech
Ground at `origin`, named `EGO`. This starts `sim_manager_node` + `sensor_dispatcher`;
it does not start recording. The default profile connects to an already-running game.
Windowed/headless/no-GPU startup and the
independent physics-pacing choices are described in
[simulator operations](../../../docs/simulator_operations.md#game-modes-and-physics-pacing).

## Config layout

```
config/
  OVERRIDES.md       # How to tune LLC, sensors, scenarios (start here)
  runs/              # Thin run files (what you pass to config:=)
  levels/            # Per-map spawn presets (derby, tech_ground, …)
  defaults/          # sim, llc, udp_io, ros_poll fragments
  presets/user/      # Opt-in spawn and controller overlays (preset:=)
  vehicle_catalog.yaml   # Model, yaw_offset, steering_calibration, torque_map
  vehicles/          # Generated BeamNG vehicle exports (attach / reference)
  garage/            # Opt-in support-experiment placement profiles
```

### Run file (minimal)

```yaml
# config/runs/single_vehicle.yaml
compose:
  vehicle: covet_turbo_rally
  vehicle_id: EGO
  level: tech_ground
  spawn: origin
scenario_mode: create
```

The default `single_vehicle.yaml` selects Covet Turbo Rally on Tech Ground.
Use `vehicle:=`, `level:=`, and `spawn:=` to change those choices.

### User preset (no new run YAML)

```yaml
# config/presets/user/derby_grid_lane.yaml
level: derby
spawn: grid_lane
```

```bash
ros2 launch bng_bringup simulator.launch.py preset:=derby_grid_lane.yaml
```

Search order for `preset:=`: absolute path → `config/presets/user/` → `~/.config/bng_bringup/presets/`.

### Roll-filter presets

Presets apply only when explicitly selected with `preset:=`; none is automatic.
The Easylift catalog entry supplies the native export and defaults to shadow mode.
Other current catalog vehicles do not configure a roll-filter model.

| Preset | Effect on Easylift |
|---|---|
| `roll_filter_active.yaml` | Enables intervention; preserves all catalog bounds and tuning |
| `roll_filter_shadow.yaml` | Evaluates without intervention; preserves all catalog bounds and tuning |
| `vehicle_environment.yaml` | Receives environment context with the roll adapter removed |

```bash
ros2 launch bng_bringup simulator.launch.py \
  vehicle:=utv_canam_x3_r322_easylift preset:=roll_filter_active.yaml
```

Both mode presets use the catalog's 35° roll limit and recovery setting. Select a
different limit explicitly in an experiment override; switching mode alone does not
change it. Model/export configuration remains in the vehicle catalog.

Easylift's `calibration.autonomous_rate_limits` explicitly limits autonomous
post-PI targets to 0.9 rad/s steering and 6000 Nm/s torque, independently of filter
mode. The roll filter has separate candidate bounds, initially set to the same
rates, with an assisted steering range of ±0.65 rad inside the measured locks.
These are command bounds, not hard limits on wheel motion or free driver requests.
Accepted manual continuation passes through; assistance re-enters the angle range
at bounded rate and hands back only when the retained driver target is reachable.
Unavailable filtering or bypass releases manual assistance. Service brake enters
the model; parking brake makes this filter path unavailable.
See the [manual command contract](../../../luamod/FEEDBACK_CONTRACT.md#roll-filter-command-contract).

## Record a run

Keep the simulator launch running. In another terminal with the workspace sourced:

```bash
ros2 run bng_simulator start_logs
```

Confirm the displayed destination. The logger records full GTState data, including
available `llc_*` and `rf_*` diagnostics, plus all ROS topics in an MCAP bag by
default. Type `stop` or press Ctrl-C in the logger terminal, enter the requested
metadata, and confirm that the run should be kept. Stop recording before closing
the simulator launch.

Runs normally go to `~/beamng_log_data/<model_config>/run_XXX/`, containing
`data.pkl`, `metadata.yaml` (including the resolved simulator configuration), and
`rosbag_<timestamp>/` when bag recording is enabled. For GTState-only recording:

```bash
ros2 run bng_simulator start_logs --no_rosbag
```

A running high-level controller can additionally supply its configuration and
verified reference snapshot; manual recording does not require one. See
[simulator logging options](../bng_simulator/README.md#utility-scripts) for topic
selection, bag format, and controller/reference capture details.

## Add a vehicle and export its parameters

Normal LLC operation requires a measured steering map. For a first export, a
catalog entry can set `calibration_only: true` with just `model` and `part_config`.
This spawns through the normal level/spawn presets with attach sensors and no
LLC, torque map, or rollover filter. For example, after marking an entry for
calibration, launch it normally:

```bash
ros2 launch bng_bringup simulator.launch.py config:=single_vehicle.yaml vehicle:=utv_canam_x3_r322_easylift
```

Keep the launch running and use the `vehicle-config` command below with
`--vehicle EGO`. After export, remove `calibration_only`, set
`steering_calibration: vehicles/<export>.yaml`, and add the desired control
settings. Rebuild `bng_bringup` before relaunching with the LLC.

Alternatively, spawn an uncalibrated vehicle manually in BeamNG, then attach:

```bash
ros2 launch bng_bringup simulator.launch.py config:=attach.yaml
```

Run the `vehicle-config` command below with `--vehicle thePlayer` (or its actual
in-game ID). After exporting, add `model`, exact `part_config`, and
`steering_calibration: vehicles/<export>.yaml` under `catalog:`. Preserve filename
case. Set torque-map and model-based filter settings only when available.

With BeamNG open and the ROS workspace sourced, launch one catalog entry:

```bash
ros2 launch bng_bringup simulator.launch.py \
  config:=single_vehicle.yaml vehicle:=racetruck_tt1_unlimited
```

Other available keys: `covet_turbo_rally`, `covet_dunes`. Stop the launch with
Ctrl-C before switching vehicles. The run spawns the selected vehicle as `EGO`;
`level:=` and `spawn:=` select another map/spawn preset.

These three entries use `drivetrain.mode: 2` (arcade) and `startGear: null` for
automatic clutching/shifting. A forced gear command switches BeamNG back to
realistic mode, so clear the inherited UTV `startGear` when using arcade.

Keep the launch running. From another sourced terminal, export to the source
folder containing the other vehicle YAMLs:

```bash
ros2 run bng_simulator sim_control --timeout 10 vehicle-config \
  --vehicle EGO \
  --output ~/ros2_ws/src/sim_ros_framework/src/bng_xal/bng_bringup/config/vehicles/
```

This settles the vehicle, measures body-frame mass/CoG/inertia, axle/wheel geometry
and steering calibration/response, then queries engine, gearbox, powertrain and
controllers. It sweeps steering and cycles gears; keep the vehicle stationary and
let it finish. The filename comes from the actual model/config, e.g.
`racetruck_tt1_unlimited.yaml`, `covet_turbo_rally.yaml`, or `covet_Dunes.yaml`.
Use `--output /path/to/name.yaml` for an explicit filename; existing files are
replaced.

The export does not update the catalog. Its `front_steering_samples` supply the
LLC input/angle map; point `steering_calibration` at this file instead of copying
the diagnostic mid-range scalar slope. These exports precede dynamics
identification and do not supply a fitted model or torque map. Rebuild
`bng_bringup` after adding a new export so it is installed for ROS launches.

## Launch arguments

| Arg | Default | Purpose |
|-----|---------|---------|
| `config` | `single_vehicle.yaml` | Run file under `config/runs/` |
| `preset` | — | User preset YAML overlay |
| `host` | `127.0.0.1` | BeamNG IP |
| `remote` | — | Companion bind IP (WSL → Windows); falls back to `host` |
| `bng_listen_port` | `64257` | Base `control_listen` (port_index 0) |
| `bng_send_port` | `64258` | Base `control_state_send` |
| `bng_sensor_port` | `64259` | Base `sensor_send` |
| `vehicle`, `level`, `spawn`, `yaw`, `pos` | — | Single-vehicle shorthand overrides |

Multi-vehicle runs (`compose.vehicles[]`) — edit YAML; launch overrides are rejected.

## WSL2 networking

BeamNG on Windows, ROS in WSL:

```bash
# Windows host IP from WSL:
export BNG_HOST=$(ip route show | grep -i default | awk '{print $3}')
ros2 launch bng_bringup simulator.launch.py host:=$BNG_HOST remote:=$BNG_HOST
```

Or use [mirrored networking](https://learn.microsoft.com/en-us/windows/wsl/wsl-config#configuration-settings-for-wslconfig) and keep defaults.

## Reference

- **Overrides guide:** [config/OVERRIDES.md](config/OVERRIDES.md) — LLC, gtState, sensors, merge priority
- Run-file knobs: `config/runs/compose_reference.yaml`
- Multi-agent: `config/runs/multi_agent.yaml`
- Attach mode: `config/runs/attach.yaml`

## Build

See root README **step 4** for the full split-build workflow. Short version:

**First time:**

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash

colcon build --packages-select bng_msgs bng_simulator bng_controller
colcon build --symlink-install --packages-select bng_bringup
source install/setup.bash
ls -la install/share/bng_bringup/config
```

**Config YAML** — no rebuild after symlink is verified; edit `config/` in `src/` and relaunch.

**Rebuild `bng_bringup` only** (launch file changes — keep `--symlink-install`):

```bash
colcon build --symlink-install --packages-select bng_bringup
source install/setup.bash
```

Do not mix normal and symlink builds on `bng_bringup`. Do not use `--symlink-install` on the other packages.

### Measured steering calibration

Catalog entries require `steering_calibration: vehicles/<export>.yaml`. Compose
copies `front_steering_samples` (`u`, `delta` in mean road-wheel radians) into
`LowLevelController.calibration.steering_map`; it validates finite, strictly
monotone samples covering normalized inputs -1 and +1. Invalid/missing data
fails before spawn. The manifest prints the selected map's sample count.

The LLC interpolates angle → normalized input and the inverse using those same
samples. Measured endpoints supply asymmetric physical limits for feedback and
roll filtering. The single `steering_to_input` coefficient is no longer accepted.
Supported catalog vehicles are Covet Dunes, Covet Turbo Rally and Racetruck.
Easylift now uses its measured steering export with the LLC, existing torque map,
and native roll filter in shadow mode.
Recreating the controller or respawning loads the selected map again; reset
retains it. Rebuild/reload the Lua mod after updating these controller sources.
