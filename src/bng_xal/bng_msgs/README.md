# bng_msgs

BeamNG bridge sensors, scalar commands and simulator/logger services.
Shared controller localization, MPC horizons and reference setup live in
[`xal_drive_msgs`](../../../../xal_drive_msgs/README.md), for both BeamNG and PX4.

| Interface | Purpose |
|---|---|
| `BasicStateMsg` | Basic pose/direction/velocity sensor |
| `BngVehicleStateMsg` | Control-state UDP observation: pose, velocities and actuation |
| `BngImuMsg`, `BngGpsMsg` | IMU and GPS streams |
| `GtStateMsg` | Full ground-truth observation |
| `BngControlCmd` | Scalar LLC command through the ROS bridge |
| `ExecuteRequest` | Simulator/vehicle API requests |
| `StartLogger`, `StopLogger` | Recording lifecycle |

[Field notes](MESSAGES.md) explain clocks and command validity bits.
Build before the simulator/controller packages:

```bash
cd ~/ros2_ws
colcon build --packages-select bng_msgs bng_simulator bng_controller bng_bringup
source install/setup.bash
```

Removed tracking/HLC definitions have no compatibility aliases. After an interface
removal, clean this package's generated build/install outputs before rebuilding so
obsolete types are not left discoverable. Dependencies: `std_msgs`, `geometry_msgs`
and the standard ROS interface generators/runtime.
