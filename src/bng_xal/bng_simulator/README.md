# bng_simulator

ROS 2 package that **manages BeamNG scenarios and vehicles** — connects to BeamNG.tech, spawns vehicles with the xlab LLC, and exposes sim services.

## Quick start

Launched via bringup (not standalone):

```bash
ros2 launch bng_bringup simulator.launch.py config:=single_vehicle.yaml
```

Node: `sim_manager_node` — loads composed config, starts `SimulationManager`, injects UDP ports.

## Architecture

```
config/runs/single_vehicle.yaml
        │
        ▼
scenario_compose.compose_scenario()   ← catalog + levels + defaults
        │
        ▼
SimulationManager  →  ScenarioBuilder  →  beamngpy
        │
        ▼
BeamNG (vehicles, sensors, LLC via luamod)
```

| Module | Role |
|--------|------|
| `sim_manager_node.py` | ROS node, launch overrides, port injection |
| `utils/scenario_compose.py` | Compose pipeline, presets, path resolution |
| `core/simulation_manager.py` | Sim lifecycle, attach/create modes |
| `core/scenario_builder.py` | beamngpy scenario from composed dict |
| `utils/vehicle_io_config.py` | Per-vehicle UDP port assignment |
| `utils/math_op.py` | xlab ↔ BeamNG yaw at spawn/teleport |

Config lives in **`bng_bringup`** — this package does not ship `config/`.

## Dependencies

**Once** (from workspace root, after sourcing ROS):

```bash
rosdep install --from-paths src/sim_ros_framework/src/bng_xal --ignore-src -r -y
pip install -r src/sim_ros_framework/requirements.txt
```

- `package.xml` — apt/ROS (`rclpy`, `python3-yaml`, …)
- `requirements.txt` — pip (`beamngpy`, …)

**Build / reinstall** after code changes:

```bash
colcon build --packages-select bng_msgs bng_simulator bng_controller bng_bringup
source install/setup.bash
```

## Services

| Service | Purpose |
|---------|---------|
| `/execute_request` | Teleport, get_manager_config, xlab API calls |
| `/start_logger`, `/stop_logger` | GtState logging to disk |

Example:

```bash
ros2 service call /execute_request bng_msgs/srv/ExecuteRequest \
  "{function_name: 'get_manager_config', arguments: ''}"
```

## Utility scripts

| Command | Purpose |
|---------|---------|
| `ros2 run bng_simulator sim_shell` | Interactive sim commands |
| `ros2 run bng_simulator sim_control` | CLI teleport / inputs |
| `ros2 run bng_simulator start_logs` | Log GtState to pickle and ROS topics to MCAP |
| `ros2 run bng_simulator find_ema` | Tune gtState EMA filters |
| `ros2 run bng_simulator sim_control garage-plan` / `garage-run` | [Stationary support experiments, plans and recovery](../../../docs/support_garage.md) |

`start_logs` records all ROS topics by default, including the retained controller descriptor, MPC horizons,
surface state, GT path and solve diagnostics. At recording start it
also checks `/drive_high_level_ros/get_ref_traj_info` (`xal_drive_msgs/srv/GetRefTrajInfo`)
and copies the loaded controller configuration to `<run>/tracking.yaml`. Metadata
contains `tracking_yaml: tracking.yaml` for replay and `mpc_config` with the
controller's session, native clock, reference identity and channel layout.
The service returns this descriptor under `response.info`; MPC and offline
feedback share it. Reference
artifacts are copied to `<run>/reference/` (CSV, metadata and surface), checked
against the controller's SHA-256 identity. The saved tracking YAML uses that
relative snapshot; replay does not depend on a subsequently regenerated trajectory.
Reference files must be available locally: use scp, Git or shared storage for a
remote controller. Files are never transferred through ROS. Missing/mismatched
files produce `REFERENCE SNAPSHOT FAILED` and an explicit metadata error while
sensor recording continues. The same `snapshot_reference(response.info, run_folder)`
helper in `drive_trajopt.traj.artifact` is available to external PX4 loggers. The controller's YAML must be accessible on the logging
machine; unavailable messages, service, or file do not prevent sensor recording.
Source the workspace containing `xal_drive_msgs` to enable configuration capture.
The background check also supports a controller started after recording begins.

For a renamed or namespaced controller, set the service explicitly:

```bash
ros2 run bng_simulator start_logs \
  --mpc_info_service /my_controller/get_ref_traj_info
```

`--record_topics default` selects the five current `/drive_high_level_ros/` controller
topics, including `controller_info`; `--record_topics all` remains the default, and `--no_rosbag` disables
bag recording. MPC capture uses the current `GetRefTrajInfo` API, rather than the
legacy `/mpc_high_level/get_config` API.

Dev entry (same as node, explicit config path):

```bash
ros2 run bng_simulator run_simulator -- --config single_vehicle.yaml
```

## Tests

```bash
pytest src/bng_xal/bng_simulator/test -q
```

## Examples

Service API notebooks (require running `sim_manager_node`):

| Notebook | Purpose |
|----------|---------|
| `examples/check_service_requests.ipynb` | Teleport, ABS/ESC, gearbox, vehicle properties via `execute_request` |
| `examples/create_vehicle_config.ipynb` | Query sim/vehicle config for attach / catalog workflows |

For control patterns use `bng_controller/examples/`.

## See also

- [bng_bringup README](../bng_bringup/README.md) — config & launch
- [luamod README](../../../luamod/README.md) — LLC Lua, torque map FFI
