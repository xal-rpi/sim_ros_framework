"""Unit tests for VehicleIoEndpoints parsing."""

from __future__ import annotations

from typing import Any, Dict

import pytest

from bng_controller.vehicle_io import VehicleIoEndpoints


def test_from_low_level_controller_config(sample_llc_config: Dict[str, Any]) -> None:
    endpoints = VehicleIoEndpoints.from_low_level_controller_config(sample_llc_config)
    assert endpoints.command_addr == ("127.0.0.1", 64257)
    assert endpoints.control_state_bind_addr == ("127.0.0.1", 64258)
    assert endpoints.sensor_bind_addr == ("127.0.0.1", 64259)


def test_from_low_level_controller_config_without_sensor() -> None:
    llc = {
        "control_listen_ip": "10.0.0.1",
        "control_listen": 1,
        "control_state_send_ip": "10.0.0.1",
        "control_state_send": 2,
    }
    endpoints = VehicleIoEndpoints.from_low_level_controller_config(llc)
    assert endpoints.sensor_bind_addr is None


def test_from_low_level_controller_config_missing_listen_raises() -> None:
    with pytest.raises(KeyError):
        VehicleIoEndpoints.from_low_level_controller_config({})
