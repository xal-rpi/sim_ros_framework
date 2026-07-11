# bng_controller

**Companion I/O bridge** between ROS (or Python) and xlab LLC UDP ports — not a high-level planner.

## Quick start

Started automatically with the sim:

```bash
ros2 launch bng_bringup simulator.launch.py config:=gridworld.yaml
```

`sensor_dispatcher`:
- **binds** each vehicle's `sensor_send` → publishes `/<vehicle>/sensors/<stream>/<type>`
- **subscribes** to `/<vehicle>/control/cmd` (`BngControlCmd`) → forwards to `control_listen`

## Control paths

### A. ROS topic (controllers, MPC, RL nodes)

```python
from bng_msgs.msg import BngControlCmd

msg = BngControlCmd()
msg.valid_fields = BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING
msg.torque = 50.0
msg.steering = 0.05          # roadwheel [rad]
publisher.publish(msg)       # topic: /EGO/control/cmd
```

Full example: [`examples/control_via_ros.py`](examples/control_via_ros.py)

### B. VehicleSession (scripts, calibration)

```python
from bng_controller.vehicle_session import VehicleSession
from bng_msgs.msg import BngControlCmd

with VehicleSession.from_vehicle_name("EGO", recreate=True) as session:
    session.send_control_cmd(cmd)
    session.calibrate({"gains": {"kp": 4000.0}})  # LLC tune envelope
```

- `use_cached=True` (default) — reuse dispatcher's session (infrequent tune OK)
- `recreate=True` — dedicated UDP client (calibration sweeps, don't steal binds)

Full example: [`examples/control_via_session.py`](examples/control_via_session.py)

### C. Steering calibration (`steering_input`)

When `steering_to_input` is unknown, command BeamNG input directly:

```python
msg.valid_fields = BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING_INPUT
msg.steering_input = 0.3   # [-1, 1], bypasses catalog scale
```

Sweep at ±1 and estimate scale: `python3 examples/steering_input_sweep.py` (sim running). Prints `steering_to_input` → paste into `vehicle_catalog.yaml`, then use `FIELD_STEERING`.

## BngControlCmd field guide

| Bit | Field | LLC meaning |
|-----|-------|-------------|
| `FIELD_THROTTLE` | throttle | Direct [0,1], bypasses torque map |
| `FIELD_TORQUE` | torque | Feedforward rear torque [N·m] |
| `FIELD_WHEEL_SPEED` | wheel_speed | Rear speed target [m/s] + PI |
| `FIELD_STEERING` | steering | Roadwheel [rad] |
| `FIELD_STEERING_INPUT` | steering_input | Direct BeamNG input |
| `FIELD_BRAKE` | brake | Brake [0,1] |

**Replace semantics:** only set bits you intend active — omitted axes are cleared each cmd.

**Torque map:** utv has `torque_map` in catalog; without it, `torque` / `wheel_speed` do not actuate (throttle and steering still work).

Details: [bng_msgs/MESSAGES.md](../bng_msgs/MESSAGES.md)

## Smoke tests (live sim required)

```bash
ros2 run bng_controller test_vehicle_udp -- --command-port 64257 --state-port 64258
ros2 run bng_controller test_vehicle_sensor_udp -- --sensor-port 64259
ros2 run bng_controller test_llc_wr_torque -- --command-port 64257 --state-port 64258 --torque 50
```

## Core modules

| Module | Role |
|--------|------|
| `sensor_dispatcher.py` | ROS node — multi-vehicle bridge |
| `vehicle_session.py` | High-level companion API |
| `vehicle_config.py` | Resolve LLC/UDP from sim or composed YAML |
| `vehicle_io.py` | UDP client |
| `sensor_converters.py` | Observation batch → `bng_msgs` |
| `llc_scalar_commands.py` | Command envelope helpers + test cases |

## Dependencies

- ROS 2: `rclpy`, `bng_msgs`, `bng_simulator`
- Python: `setuptools`

```bash
colcon build --packages-select bng_controller
```

## Tests

```bash
pytest src/bng_xal/bng_controller/test -q
```

## Research controllers

MPC / RL / planning code belongs in your own package or `policies/` — publish `BngControlCmd` or use `VehicleSession`, not an in-tree HLC.
