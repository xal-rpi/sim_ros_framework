"""Unit tests for offline vehicle I/O resolution from composed runs."""

from __future__ import annotations

from bng_controller.vehicle_config import LaunchIoParams, resolve_vehicle_io


def test_resolve_vehicle_io_from_gridworld() -> None:
    resolution = resolve_vehicle_io(
        "EGO",
        "gridworld.yaml",
        source="config",
        launch=LaunchIoParams(
            beamng_host="127.0.0.1",
            listen_port=64257,
            send_port=64258,
            sensor_port=64259,
        ),
    )
    assert resolution.vehicle_name == "EGO"
    assert resolution.endpoints.command_addr == ("127.0.0.1", 64257)
    assert resolution.endpoints.sensor_bind_addr == ("127.0.0.1", 64259)
    assert "LowLevelController" in resolution.llc_config or resolution.llc_config


def test_resolve_vehicle_io_with_preset() -> None:
    resolution = resolve_vehicle_io(
        "EGO",
        "gridworld.yaml",
        source="config",
        launch_overrides={"preset": "derby_grid_lane.yaml"},
    )
    assert resolution.endpoints.command_addr[1] == 64257
