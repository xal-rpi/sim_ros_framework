# bng_msgs

ROS 2 message and service definitions for the xlab BeamNG interface.

## Quick reference

**Commands:** `BngControlCmd` on `/<vehicle>/control/cmd`  
**State:** `BngVehicleStateMsg` on `/<vehicle>/sensors/control_state/control_state`  
**Services:** `ExecuteRequest`, `StartLogger`, `StopLogger`

Full field tables and control semantics: **[MESSAGES.md](MESSAGES.md)** ← start here for control integration.

## Build

Must be built before `bng_simulator` / `bng_controller`:

```bash
colcon build --packages-select bng_msgs
source install/setup.bash
```

After editing `.msg` files, rebuild `bng_msgs` and any dependent package.

## Messages (golden path)

| Message | Source | Purpose |
|---------|--------|---------|
| `BngControlCmd` | Your controller → dispatcher | Scalar LLC commands |
| `BngVehicleStateMsg` | `control_state` UDP stream | Pose, velocities, actuation, torque est. |
| `BngImuMsg` | `imu` UDP stream | Roof IMU |
| `BngGpsMsg` | `gps` (if enabled) | Position |
| `GtStateMsg` | GE poll path (legacy) | Full gtState via ros_poll |

## Services

| Service | Provider | Purpose |
|---------|----------|---------|
| `ExecuteRequest` | `sim_manager_node` | Teleport, config query, xlab API |
| `StartLogger` / `StopLogger` | `sim_manager_node` | Disk logging |

## Dependencies

- `std_msgs`, `geometry_msgs`, `rosidl_default_generators`

## See also

- [bng_controller README](../bng_controller/README.md) — how commands are forwarded
- [Root README](../../../README.md) — setup and architecture
