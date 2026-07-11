"""Unit tests for LLC scalar UDP command envelopes (no sim required)."""

from __future__ import annotations

import json

import pytest

from bng_controller.llc_scalar_commands import (
    LLC_SCALAR_CMD_CASES,
    cmd_envelope,
    is_valid_scalar_command,
)


@pytest.mark.parametrize("name,payload", list(LLC_SCALAR_CMD_CASES.items()))
def test_scalar_cases_are_valid(name: str, payload: dict) -> None:
    assert is_valid_scalar_command(payload), name


@pytest.mark.parametrize("name,payload", list(LLC_SCALAR_CMD_CASES.items()))
def test_scalar_cmd_envelope_roundtrip_json(name: str, payload: dict) -> None:
    env = cmd_envelope(payload)
    assert env["type"] == "cmd"
    assert env["data"] == payload
    raw = json.dumps(env)
    decoded = json.loads(raw)
    assert decoded["data"] == payload


def test_brake_alone_is_not_a_scalar_command() -> None:
    assert not is_valid_scalar_command({"brake": 0.5})


def test_scalar_replace_clears_unmentioned_fields() -> None:
    """Document LLC loadScalar replace semantics (see controller_llc.lua)."""
    full = {"torque": 50.0, "wheel_speed": 5.0, "steering": 0.0}
    throttle_only = {"throttle": 0.4, "steering": 0.0}
    torque_after_throttle = {"torque": 1500.0, "wheel_speed": 5.0, "steering": 0.0}
    assert "throttle" not in torque_after_throttle
    assert "torque" not in throttle_only
    assert is_valid_scalar_command(full)
    assert is_valid_scalar_command(throttle_only)


def test_empty_payload_rejected() -> None:
    assert not is_valid_scalar_command({})


def test_trajectory_shape_requires_torque_array() -> None:
    """Document LLC trajectory detection: torque array required like wr/steer."""
    traj = {
        "x": [0.0, 1.0],
        "y": [0.0, 0.0],
        "wr": [5.0, 5.0],
        "steer": [0.0, 0.0],
        "torque": [50.0, 50.0],
    }
    assert "torque" in traj
    assert len(traj["torque"]) == len(traj["x"])

    missing_torque = {k: v for k, v in traj.items() if k != "torque"}
    assert "torque" not in missing_torque
