"""Shared fixtures for bng_controller unit tests."""

from __future__ import annotations

from typing import Any, Dict

import pytest


@pytest.fixture
def sample_llc_config() -> Dict[str, Any]:
    return {
        "control_listen_ip": "127.0.0.1",
        "control_listen": 64257,
        "control_state_send_ip": "127.0.0.1",
        "control_state_send": 64258,
        "sensor_send_ip": "127.0.0.1",
        "sensor_send": 64259,
        "sensor_broadcast": {"sensor": "control_state"},
    }


@pytest.fixture
def sample_control_state_observation() -> Dict[str, Any]:
    return {
        "sensor": "control_state",
        "name": "control_state",
        "t": 1.25,
        "data": {
            "x": 1.0,
            "y": 2.0,
            "z": 3.0,
            "quat": [0.0, 0.0, 0.0, 1.0],
            "yaw": 0.1,
            "pitch": 0.0,
            "roll": 0.0,
            "Phi": 0.0,
            "beta": 0.0,
            "vx": 5.0,
            "vy": 0.0,
            "vz": 0.0,
            "V": 5.0,
            "p": 0.0,
            "q": 0.0,
            "r": 0.0,
            "accel_x": 0.0,
            "accel_y": 0.0,
            "accel_z": 0.0,
            "w_fl": 10.0,
            "w_fr": 10.0,
            "w_rl": 10.0,
            "w_rr": 10.0,
            "delta_l": 0.0,
            "delta_r": 0.0,
            "throttle": 0.2,
            "brake": 0.0,
            "pbrake": 0.0,
            "gear_index": 2,
            "gear_ratio": 3.5,
            "we": 100.0,
            "pb": 0.0,
            "rear_wheel_torque_est": 80.0,
            "torque_min": -200.0,
            "torque_max": 200.0,
        },
    }
