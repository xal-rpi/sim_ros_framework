"""Unit tests for xlab UDP observation → bng_msgs converters."""

from __future__ import annotations

from typing import Any, Dict

from bng_controller.sensor_converters import (
    ObservationSensor,
    control_state_observation_to_msg,
    observation_to_ros_msg,
)


def test_control_state_observation_to_msg(
    sample_control_state_observation: Dict[str, Any],
) -> None:
    msg = control_state_observation_to_msg(sample_control_state_observation, "map")
    data = sample_control_state_observation["data"]
    assert msg.header.frame_id == "map"
    assert msg.sim_time == 1.25
    assert msg.position.x == data["x"]
    assert msg.position.y == data["y"]
    assert msg.yaw == data["yaw"]
    assert msg.rear_wheel_torque_est == data["rear_wheel_torque_est"]


def test_observation_to_ros_msg_registry(
    sample_control_state_observation: Dict[str, Any],
) -> None:
    msg = observation_to_ros_msg(
        sample_control_state_observation,
        ObservationSensor.CONTROL_STATE,
        "EGO",
    )
    assert msg.header.frame_id == "EGO"
