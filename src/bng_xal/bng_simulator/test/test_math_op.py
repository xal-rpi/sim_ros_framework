"""Unit tests for xlab ↔ BeamNG yaw / euler helpers."""

from __future__ import annotations

import numpy as np

from bng_simulator.utils.math_op import (
    apply_xlab_yaw_to_beamng,
    convert_euler_to_quaternion,
    process_euler_to_quat,
)
from bng_simulator.utils.scenario_compose import beamng_yaw_from_xlab


def test_beamng_yaw_from_xlab_utv_offset() -> None:
    assert beamng_yaw_from_xlab(0.0, 90.0) == -90.0
    assert beamng_yaw_from_xlab(45.0, 90.0) == -45.0


def test_apply_xlab_yaw_to_beamng() -> None:
    args = {"xlab_yaw_deg": 0.0}
    apply_xlab_yaw_to_beamng(args, yaw_offset_deg=90.0)
    assert args["yaw_angle"] == -90.0
    assert args["xlab_yaw_deg"] == 0.0


def test_apply_xlab_yaw_to_beamng_skips_when_quat_present() -> None:
    args = {"rot_quat": (0.0, 0.0, 0.0, 1.0), "xlab_yaw_deg": 10.0}
    apply_xlab_yaw_to_beamng(args, yaw_offset_deg=90.0)
    assert "yaw_angle" not in args
    assert args["xlab_yaw_deg"] == 10.0


def test_process_euler_to_quat_removes_yaw_keys() -> None:
    args = {"yaw_angle": 90.0, "pitch_angle": 0.0, "roll_angle": 0.0}
    process_euler_to_quat(args)
    assert "rot_quat" in args
    assert "yaw_angle" not in args
    assert len(args["rot_quat"]) == 4


def test_convert_euler_to_quaternion_identity_yaw() -> None:
    quat = convert_euler_to_quaternion((0.0, 0.0, 0.0))
    assert np.allclose(quat, [0.0, 0.0, 0.0, 1.0], atol=1e-6)
