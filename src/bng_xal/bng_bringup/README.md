# bng_bringup

Launch files and **all scenario configuration** for the xlab research stack.

## Quick start

```bash
ros2 launch bng_bringup simulator.launch.py config:=gridworld.yaml
```

This starts `sim_manager_node` + `sensor_dispatcher` (see root README).

## Config layout

```
config/
  runs/              # Thin run files (what you pass to config:=)
  levels/            # Per-map spawn presets (derby, tech_ground, …)
  defaults/          # sim, llc, udp_io, ros_poll fragments
  presets/user/      # User spawn overlays (preset:=)
  vehicle_catalog.yaml   # Model, yaw_offset, steering_to_input, torque_map
  vehicles/          # Generated BeamNG vehicle exports (attach / reference)
```

### Run file (minimal)

```yaml
# config/runs/gridworld.yaml
compose:
  vehicle: utv_wild
  vehicle_id: EGO
  level: tech_ground
  spawn: origin
scenario_mode: create
```

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

## Launch arguments

| Arg | Default | Purpose |
|-----|---------|---------|
| `config` | `gridworld.yaml` | Run file under `config/runs/` |
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

- All knobs: `config/runs/compose_reference.yaml`
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
