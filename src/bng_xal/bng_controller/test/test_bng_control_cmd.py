"""Unit tests for BngControlCmd → LLC JSON payload mapping."""

from __future__ import annotations

from bng_controller.vehicle_session import bng_control_cmd_payload
from bng_msgs.msg import BngControlCmd


def _cmd(**kwargs) -> BngControlCmd:
    msg = BngControlCmd()
    for key, value in kwargs.items():
        setattr(msg, key, value)
    return msg


def test_steering_input_only() -> None:
    payload = bng_control_cmd_payload(
        _cmd(valid_fields=BngControlCmd.FIELD_STEERING_INPUT, steering_input=0.35)
    )
    assert payload == {"steering_input": 0.35}


def test_steering_input_wins_when_both_steering_axes_set() -> None:
    payload = bng_control_cmd_payload(
        _cmd(
            valid_fields=(
                BngControlCmd.FIELD_STEERING
                | BngControlCmd.FIELD_STEERING_INPUT
            ),
            steering=0.1,
            steering_input=-0.2,
        )
    )
    assert payload == {"steering": 0.1, "steering_input": -0.2}


def test_torque_and_steering_input_for_calibration() -> None:
    payload = bng_control_cmd_payload(
        _cmd(
            valid_fields=(
                BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING_INPUT
            ),
            torque=60.0,
            steering_input=0.5,
        )
    )
    assert payload == {"torque": 60.0, "steering_input": 0.5}


def test_valid_fields_zero_is_noop() -> None:
    assert bng_control_cmd_payload(_cmd(valid_fields=0, steering_input=1.0)) is None
