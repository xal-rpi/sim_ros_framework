"""Convert xlab UDP sensor_send observations to bng_msgs.

Wire format (controller_manager.lua)
------------------------------------
Batch: ``{ "sent_t": <sim_t>, "<stream>": { "sensor", "name", "t", "data" }, ... }``

Observation ``sensor`` tag selects the converter (see ``ObservationSensor`` /
``SENSOR_REGISTRY``). Lua ``sensor_broadcast.sensor`` must match the enum value.

**control_state** ``data`` (= controlStateOut):
  t, x, y, z, quat, yaw, pitch, roll, Phi, beta, vx, vy, vz, V, p, q, r,
  accel_x/y/z, w_fl/fr/rl/rr, delta_l/r, throttle, brake, pbrake,
  gear_index, gear_ratio, we, pb, rear_wheel_torque_est, torque_min, torque_max

**imu** ``data``: pos, accel, gyro, ang_accel

**gps** ``data``: x, y, lon, lat
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, Tuple, Type, Union

from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import Header

from bng_msgs.msg import BngGpsMsg, BngImuMsg, BngVehicleStateMsg

RosMsg = Union[BngVehicleStateMsg, BngImuMsg, BngGpsMsg]
ConvertFn = Callable[[Dict[str, Any], str], RosMsg]


class ObservationSensor(str, Enum):
    """UDP observation ``sensor`` tag (YAML ``sensor_broadcast.sensor``)."""

    CONTROL_STATE = "control_state"
    IMU = "imu"
    GPS = "gps"


@dataclass(frozen=True)
class SensorSpec:
    """One sensor type: ROS message class + observation converter."""

    msg_type: Type[RosMsg]
    convert: ConvertFn


def _vec3(value: Any) -> Tuple[float, float, float]:
    return float(value[0]), float(value[1]), float(value[2])


def _quat(value: Any) -> Tuple[float, float, float, float]:
    return float(value[0]), float(value[1]), float(value[2]), float(value[3])


def _header(frame_id: str) -> Header:
    header = Header()
    header.frame_id = frame_id
    return header


def control_state_observation_to_msg(
    observation: Dict[str, Any],
    frame_id: str,
) -> BngVehicleStateMsg:
    data = observation["data"]
    msg = BngVehicleStateMsg()
    msg.header = _header(frame_id)
    msg.sim_time = float(observation["t"])

    msg.position = Point(
        x=float(data["x"]),
        y=float(data["y"]),
        z=float(data["z"]),
    )
    msg.yaw = float(data["yaw"])
    msg.pitch = float(data["pitch"])
    msg.roll = float(data["roll"])
    msg.phi = float(data["Phi"])
    msg.beta = float(data["beta"])

    qx, qy, qz, qw = _quat(data["quat"])
    msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

    msg.linear_velocity = Vector3(
        x=float(data["vx"]),
        y=float(data["vy"]),
        z=float(data["vz"]),
    )
    msg.v = float(data["V"])

    msg.angular_velocity = Vector3(
        x=float(data["p"]),
        y=float(data["q"]),
        z=float(data["r"]),
    )
    msg.linear_acceleration = Vector3(
        x=float(data["accel_x"]),
        y=float(data["accel_y"]),
        z=float(data["accel_z"]),
    )

    msg.w_fl = float(data["w_fl"])
    msg.w_fr = float(data["w_fr"])
    msg.w_rl = float(data["w_rl"])
    msg.w_rr = float(data["w_rr"])
    msg.delta_l = float(data["delta_l"])
    msg.delta_r = float(data["delta_r"])

    msg.throttle = float(data["throttle"])
    msg.brake = float(data["brake"])
    msg.pbrake = float(data["pbrake"])
    msg.gear_index = int(data["gear_index"])
    msg.gear_ratio = float(data["gear_ratio"])
    msg.we = float(data["we"])
    msg.pb = float(data["pb"])

    msg.rear_wheel_torque_est = float(data["rear_wheel_torque_est"])
    msg.torque_min = float(data["torque_min"])
    msg.torque_max = float(data["torque_max"])
    return msg


def imu_observation_to_msg(
    observation: Dict[str, Any],
    frame_id: str,
) -> BngImuMsg:
    data = observation["data"]
    msg = BngImuMsg()
    msg.header = _header(frame_id)
    msg.sim_time = float(observation["t"])

    px, py, pz = _vec3(data["pos"])
    msg.position = Vector3(x=px, y=py, z=pz)

    ax, ay, az = _vec3(data["accel"])
    msg.accel = Vector3(x=ax, y=ay, z=az)

    gx, gy, gz = _vec3(data["gyro"])
    msg.gyro = Vector3(x=gx, y=gy, z=gz)

    aax, aay, aaz = _vec3(data["ang_accel"])
    msg.ang_accel = Vector3(x=aax, y=aay, z=aaz)
    return msg


def gps_observation_to_msg(
    observation: Dict[str, Any],
    frame_id: str,
) -> BngGpsMsg:
    data = observation["data"]
    msg = BngGpsMsg()
    msg.header = _header(frame_id)
    msg.sim_time = float(observation["t"])

    msg.x = float(data["x"])
    msg.y = float(data["y"])
    msg.latitude = float(data["lat"])
    msg.longitude = float(data["lon"])
    return msg


SENSOR_REGISTRY: Dict[ObservationSensor, SensorSpec] = {
    ObservationSensor.CONTROL_STATE: SensorSpec(
        BngVehicleStateMsg, control_state_observation_to_msg
    ),
    ObservationSensor.IMU: SensorSpec(BngImuMsg, imu_observation_to_msg),
    ObservationSensor.GPS: SensorSpec(BngGpsMsg, gps_observation_to_msg),
}


def ros_msg_type_for(sensor: ObservationSensor) -> Type[RosMsg]:
    return SENSOR_REGISTRY[sensor].msg_type


def observation_to_ros_msg(
    observation: Dict[str, Any],
    sensor_type: Union[str, ObservationSensor],
    frame_id: str,
) -> RosMsg:
    kind = (
        sensor_type
        if isinstance(sensor_type, ObservationSensor)
        else ObservationSensor(sensor_type)
    )
    return SENSOR_REGISTRY[kind].convert(observation, frame_id)
