# bng_msgs — xlab interface messages

Focused messages for the **gridworld** golden path.

## Types

| Choice | Rationale |
|--------|-----------|
| `float64` scalars | Matches `geometry_msgs` (Vector3/Point/Quaternion use `float64`). Standard for sim/control. |
| `geometry_msgs` vectors | `BngVehicleStateMsg` uses `Point` / `Vector3` / `Quaternion` for pose, velocity, rates, and accel for RViz and standard ROS tooling. Scalars (`yaw`, `beta`, `v`, wheel speeds, actuation) stay flat where controllers expect them. |
| `BngImuMsg` | `geometry_msgs/Vector3` for pos/accel/gyro/ang_accel. |

## Timing convention (observations only)

| Field | Meaning |
|-------|---------|
| `header.stamp` | ROS time when the bridge node received or published the message |
| `sim_time` | BeamNG simulation time `[s]` from the UDP observation |

Commands use `header.stamp` only (no `sim_time`).

---

## Observations (sensor_dispatcher → ROS)

### `BngVehicleStateMsg`

Source: `sensor_broadcast.control_state` (same fields as Lua `controlStateOut`).

| Lua field | Message field |
|-----------|---------------|
| `t` | `sim_time` |
| `x,y,z` | `position` |
| `yaw,pitch,roll,Phi,beta` | `yaw,pitch,roll,phi,beta` |
| `quat` | `orientation` |
| `vx,vy,vz,V` | `linear_velocity`, `v` |
| `p,q,r` | `angular_velocity` |
| `accel_x/y/z` | `linear_acceleration` |
| `w_fl/fr/rl/rr` | `w_fl/fr/rl/rr` |
| `delta_l/r` | `delta_l/r` |
| `throttle,brake,pbrake` | same |
| `gear_index,gear_ratio` | same |
| `we,pb` | same |
| `rear_wheel_torque_est` | same |
| `torque_min,max` | rear torque at throttle=0 / throttle=1 |

Default topic: `/<vehicle>/sensors/control_state/bng_vehicle_state`

### `BngImuMsg`

Source: `sensor_broadcast` stream with `sensor: imu` (e.g. `roof_imu`).

| Lua `data` | Message |
|------------|---------|
| `time` / observation `t` | `sim_time` |
| `pos` | `position` |
| `accel` | `accel` |
| `gyro` | `gyro` |
| `ang_accel` | `ang_accel` |

Default topic: `/<vehicle>/sensors/<stream>/bng_imu`

### `BngGpsMsg`

Source: `sensor_broadcast` stream with `sensor: gps`. BeamNG tech GPS provides
`x, y, lon, lat` only (no altitude).

| Lua `data` | Message |
|------------|---------|
| `x,y` | `x,y` |
| `lat,lon` | `latitude,longitude` |

Default topic: `/<vehicle>/sensors/<stream>/bng_gps`

---

## Commands (ROS → control_listen)

Infrequent LLC calibration — ``VehicleSession.from_vehicle_name("EGO").calibrate({...})``
from Python; see ``vehicle_session.py``. No ROS tune topic.

### `BngControlCmd`

Scalar LLC commands. OR ``valid_fields`` bits; ``sensor_dispatcher`` builds the
JSON ``data`` payload and sendto's ``control_listen``.

| Bit | Constant | JSON key | LLC setpoint |
|-----|----------|----------|--------------|
| `0x01` | `FIELD_THROTTLE` | `throttle` | `throttle_override` |
| `0x02` | `FIELD_BRAKE` | `brake` | `brake_des` |
| `0x04` | `FIELD_STEERING` | `steering` | `steer_des_rad` (needs `steering_to_input`) |
| `0x08` | `FIELD_TORQUE` | `torque` | `torque_des` |
| `0x10` | `FIELD_WHEEL_SPEED` | `wheel_speed` | `omega_des` |
| `0x20` | `FIELD_PARKING_BRAKE` | `pbrake` | reserved (LLC TBD) |
| `0x40` | `FIELD_GEAR` | `gear_index` | reserved (LLC TBD) |
| `0x80` | `FIELD_STEERING_INPUT` | `steering_input` | direct `electrics.steering_input` (calibration) |

Use `steering_input` when `steering_to_input` is unknown (fit roadwheel vs input).
Use `steering` [rad] once catalog scale is calibrated. If both are sent, LLC uses
`steering_input` and ignores `steering`.

Example: `valid_fields = FIELD_TORQUE | FIELD_STEERING_INPUT`

`valid_fields == 0` → bridge skips send (no-op).

Topic: `/<vehicle>/control/cmd` (`sensor_dispatcher` subscriber).

**Note:** Only one process should own high-rate `control_listen` sends per vehicle.
