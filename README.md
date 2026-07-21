# xlab — BeamNG ↔ ROS2 research framework

High-fidelity vehicle simulation (BeamNG.tech) with a **compose-based** config stack, per-vehicle UDP I/O, and a thin ROS bridge for observations and low-level control.

## Quick start

1. **Prerequisites:** ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04), BeamNG.tech, Python 3.10+, [luamod](luamod/README.md) built and installed in BeamNG.
2. **Clone into a colcon workspace:**
   ```bash
   cd ~/ros2_ws/src && git clone <repo-url> sim_ros_framework
   ```
3. **Install dependencies (once per machine / when `package.xml` or `requirements.txt` change):**
   ```bash
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash    # or jazzy on Ubuntu 24.04

   # apt + ROS keys from package.xml (may use sudo)
   rosdep install --from-paths src/sim_ros_framework/src/bng_xal --ignore-src -r -y

   # pure Python (beamngpy, etc.) — use your venv first if you have one
   pip install -r src/sim_ros_framework/requirements.txt
   ```
   Optional notebooks / plotting / MPC tools: `pip install -r src/sim_ros_framework/requirements-extra.txt`

4. **Build workspace**

   **First time** — run these three commands in order (symlink **only** `bng_bringup` so config YAML is edited live in `src/`):

   ```bash
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   ```

   ```bash
   colcon build --packages-select bng_msgs bng_simulator bng_controller
   ```

   ```bash
   colcon build --symlink-install --packages-select bng_bringup
   source install/setup.bash
   ```

   **Check** that config is symlinked (not copied):

   ```bash
   ls -la install/share/bng_bringup/config
   ```

   The path should end with `-> .../src/bng_xal/bng_bringup/config`. If it is a normal directory, re-run the `bng_bringup` symlink build above.

   **Config YAML** (`src/bng_xal/bng_bringup/config/…`) — after the check passes, save and relaunch (step 6). No rebuild for edits or new run/preset/default files.

   **Later rebuilds** — only when you change code (always `source install/setup.bash` after):

   ```bash
   # Python in bng_simulator or bng_controller
   colcon build --packages-select bng_simulator bng_controller
   ```

   ```bash
   # .msg / .srv in bng_msgs, then dependents
   colcon build --packages-select bng_msgs
   colcon build --packages-select bng_simulator bng_controller
   ```

   ```bash
   # launch/simulator.launch.py in bng_bringup (keep --symlink-install)
   colcon build --symlink-install --packages-select bng_bringup
   ```

   **Rules** — do not mix normal and symlink builds on the same package:
   - Never add `--symlink-install` to `bng_msgs`, `bng_simulator`, or `bng_controller`.
   - Always add `--symlink-install` when rebuilding `bng_bringup` (a plain build copies stale config into `install/`).
   - If `bng_bringup` symlink build fails after a mode switch:

   ```bash
   rm -rf build/bng_bringup install/bng_bringup
   colcon build --symlink-install --packages-select bng_bringup
   source install/setup.bash
   ```

5. **Start BeamNG** (with xlab mod loaded):
   ```bash
   ./BinLinux/BeamNG.tech.x64 -tcom -colorStdOutLog -disable-sandbox
   # -tcom -colorStdOutLog [-disable-sandbox] [-nosteam] -console -headless -gfx null
   ```
6. **Launch sim + bridge:**
   ```bash
   ros2 launch bng_bringup simulator.launch.py config:=gridworld.yaml
   ```

You should see `sim_manager_node` spawn the scenario and `sensor_dispatcher` publish `/EGO/sensors/...`.

**WSL2 networking:** set `host:=` / `remote:=` on launch if BeamNG runs on Windows — see [bng_bringup README](src/bng_xal/bng_bringup/README.md).

---

## Architecture (golden path)

```
┌─────────────────────────────────────────────────────────────────┐
│  ros2 launch bng_bringup simulator.launch.py                    │
└────────────┬───────────────────────────────┬────────────────────┘
             │                               │
     sim_manager_node                  sensor_dispatcher
     (scenario, vehicles, LLC)        (UDP ↔ ROS bridge)
             │                               │
             └─────────── BeamNG ────────────┘
                    xlab LLC (luamod)
                    control_listen  ← commands
                    sensor_send     → observations
```

| Layer | Package | Role |
|-------|---------|------|
| **Launch + config** | `bng_bringup` | Run YAML, catalog, levels, presets, launch file |
| **Simulation** | `bng_simulator` | `sim_manager_node`, scenario compose, BeamNG API |
| **Companion I/O** | `bng_controller` | `sensor_dispatcher`, `VehicleSession`, UDP client |
| **Messages** | `bng_msgs` | `BngControlCmd`, `BngVehicleStateMsg`, services |
| **In-sim logic** | `luamod` | LLC Lua, gtState, torque map FFI |

**Config flow:** thin `config/runs/*.yaml` → `scenario_compose` → fragments (`defaults/`, `levels/`, `vehicle_catalog.yaml`).

**Changing scenarios and LLC/sensor settings:** see [bng_bringup/config/OVERRIDES.md](src/bng_xal/bng_bringup/config/OVERRIDES.md) for merge priority, run `overrides`, presets, catalog, and when to edit `defaults/` in place.

---

## Control: three ways

### 1. ROS topic (recommended for controllers)

Publish `bng_msgs/BngControlCmd` to `/<vehicle>/control/cmd`.  
`sensor_dispatcher` forwards to the vehicle's `control_listen` UDP port.

```python
# See bng_controller/examples/control_via_ros.py
cmd.valid_fields = BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING
cmd.torque = 50.0
cmd.steering = 0.05   # roadwheel [rad], uses catalog steering_to_input
```

### 2. Python `VehicleSession` (scripts, calibration, notebooks)

```python
from bng_controller.vehicle_session import VehicleSession
from bng_msgs.msg import BngControlCmd

with VehicleSession.from_vehicle_name("EGO", recreate=True) as session:
    session.send_control_cmd(cmd)
```

See [bng_controller README](src/bng_xal/bng_controller/README.md) and `examples/`.

### 3. Raw UDP (advanced)

`VehicleIoClient.send_command({"torque": 50, "steering_input": 0.2})` — same JSON envelope as LLC `loadScalar`.

**Steering axes:**

| Field | Use when |
|-------|----------|
| `steering_input` [-1,1] | Calibrating — unknown `steering_to_input` |
| `steering` [rad] | Closed-loop — scale in `vehicle_catalog.yaml` |

Details: [bng_msgs/MESSAGES.md](src/bng_xal/bng_msgs/MESSAGES.md).

---

## Observations

After launch, typical topics (vehicle `EGO`):

| Topic | Message |
|-------|---------|
| `/EGO/sensors/control_state/control_state` | `BngVehicleStateMsg` |
| `/EGO/sensors/roof_imu/imu` | `BngImuMsg` |

```bash
ros2 topic echo /EGO/sensors/control_state/control_state --field yaw,v,rear_wheel_torque_est
```

---

## Configuration at launch

```bash
# Default gridworld (tech_ground, utv at origin)
ros2 launch bng_bringup simulator.launch.py

# Derby preset (no new YAML file)
ros2 launch bng_bringup simulator.launch.py preset:=derby_grid_lane.yaml

# Spawn / vehicle shorthand (single-vehicle runs only)
ros2 launch bng_bringup simulator.launch.py level:=derby spawn:=grid_lane yaw:=0
```

| Doc | Use when |
|-----|----------|
| **[OVERRIDES.md](src/bng_xal/bng_bringup/config/OVERRIDES.md)** | Tune LLC, gtState, sensors, or defaults — run `overrides`, presets, catalog |
| [compose_reference.yaml](src/bng_xal/bng_bringup/config/runs/compose_reference.yaml) | Run-file knobs, launch examples, fragment stack |
| [bng_bringup README](src/bng_xal/bng_bringup/README.md) | Config layout, presets, WSL networking |

Config YAML edits need no rebuild if `bng_bringup` is symlink-installed (step 4).

---

## Tests

```bash
source ~/ros2_ws/install/setup.bash
pytest src/sim_ros_framework/src/bng_xal/bng_simulator/test \
       src/sim_ros_framework/src/bng_xal/bng_controller/test -q
```

After `.msg` changes: `colcon build --packages-select bng_msgs` then rebuild dependents. After Python edits: `colcon build --packages-select bng_simulator bng_controller`. Config YAML: no rebuild if `bng_bringup` was symlink-installed (see step 4).

---

## Package documentation

| Package | README |
|---------|--------|
| Launch & config | [bng_bringup](src/bng_xal/bng_bringup/README.md) |
| Simulation manager | [bng_simulator](src/bng_xal/bng_simulator/README.md) |
| UDP / ROS bridge | [bng_controller](src/bng_xal/bng_controller/README.md) |
| ROS messages | [bng_msgs](src/bng_xal/bng_msgs/README.md) · [MESSAGES.md](src/bng_xal/bng_msgs/MESSAGES.md) |
| BeamNG Lua mod | [luamod](luamod/README.md) |

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| BeamNG needs focus | Keep window focused when loading scenarios |
| WSL2 can't reach BeamNG | `host:=<windows-ip>` on launch; see bringup README |
| Vehicle falls / wrong spawn on derby | Use `preset:=derby_grid_lane.yaml` (origin is void on derby) |
| Mod changes ignored | Re-run `./luamod/build.bash` and restart BeamNG |
| `xlab extension not found` (headless) | `-headless -noui -nosteam` |
| Torque units wrong | Metric units in BeamNG settings |
| Config edits ignored after launch | `ls -la install/share/bng_bringup/config` — re-run `colcon build --symlink-install --packages-select bng_bringup` (see step 4) |
| `bng_msgs` / `bng_simulator` symlink build errors | Use split build: symlink **only** `bng_bringup`; never mix `--symlink-install` on other packages |

---

## Optional: Nix dev shell

```bash
nix develop   # ROS 2 + Python deps from flake.nix
```

---

*License: Apache-2.0*
