"""Resolve per-vehicle xlab UDP I/O from live sim_manager or composed YAML."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Dict, Literal, Optional

from bng_controller.vehicle_io import VehicleIoEndpoints
from bng_simulator.utils.scenario_compose import compose_scenario
from bng_simulator.utils.vehicle_io_config import vehicles_with_io

IoSource = Literal["config", "manager"]

__all__ = [
    "LaunchIoParams",
    "VehicleIoResolution",
    "fetch_manager_config",
    "resolve_vehicle_io",
    "resolve_vehicles_io",
]


@dataclass(frozen=True)
class LaunchIoParams:
    """Base UDP triplet for port_index 0 — only needed for source='config' fallback.

    Live sim_manager already injects per-vehicle ports; external scripts should
    prefer source='manager' (default) to match the running simulation.
    """

    beamng_host: str = "127.0.0.1"
    remote_host: str = ""
    listen_port: int = 64257
    send_port: int = 64258
    sensor_port: int = 64259


@dataclass(frozen=True)
class VehicleIoResolution:
    """Resolved companion-side UDP endpoints for one vehicle."""

    vehicle_name: str
    endpoints: VehicleIoEndpoints
    llc_config: Dict[str, Any]
    vehicle_config: Dict[str, Any]


def fetch_manager_config(
    node_ros=None,
    *,
    timeout_sec: float = 5.0,
) -> Dict[str, Any]:
    """Call sim_manager ``get_manager_config`` (requires ROS discovery)."""
    from bng_simulator.utils.services_utils import send_request

    config = send_request(
        function_name="get_manager_config",
        function_args={},
        timeout_sec=timeout_sec,
        node_ros=node_ros,
    )
    if config is None:
        raise RuntimeError("get_manager_config failed (is sim_manager_node running?)")
    return config


def _resolution_from_vehicle(
    vehicle_name: str,
    vehicle_cfg: Dict[str, Any],
) -> Optional[VehicleIoResolution]:
    llc_cfg = vehicle_cfg.get("controllers", {}).get("LowLevelController")
    if not llc_cfg:
        return None
    return VehicleIoResolution(
        vehicle_name=vehicle_name,
        endpoints=VehicleIoEndpoints.from_low_level_controller_config(llc_cfg),
        llc_config=deepcopy(llc_cfg),
        vehicle_config=deepcopy(vehicle_cfg),
    )


def resolve_vehicles_io(
    config_path: Optional[str] = None,
    *,
    source: IoSource = "manager",
    launch: Optional[LaunchIoParams] = None,
    launch_overrides: Optional[Dict[str, Any]] = None,
    node_ros=None,
    timeout_sec: float = 5.0,
    fallback_to_config: bool = False,
) -> Dict[str, VehicleIoResolution]:
    """
    Resolve companion UDP endpoints for every vehicle that has an LLC.

    source=manager (default) — live get_manager_config; matches running sim.
    source=config — compose a run YAML (offline / no sim); requires config_path.
    """
    launch = launch or LaunchIoParams()

    def from_config() -> Dict[str, VehicleIoResolution]:
        if not config_path:
            raise ValueError(
                "config_path required when source='config' or fallback_to_config=True"
            )
        scenario = compose_scenario(config_path, launch_overrides)
        vehicles = vehicles_with_io(
            scenario,
            beamng_host=launch.beamng_host,
            remote_host=launch.remote_host,
            listen_port=launch.listen_port,
            send_port=launch.send_port,
            sensor_port=launch.sensor_port,
        )
        resolved: Dict[str, VehicleIoResolution] = {}
        for vehicle_name, vehicle_cfg in vehicles.items():
            entry = _resolution_from_vehicle(vehicle_name, vehicle_cfg)
            if entry is not None:
                resolved[vehicle_name] = entry
        return resolved

    if source == "config":
        return from_config()

    try:
        manager_config = fetch_manager_config(node_ros, timeout_sec=timeout_sec)
    except RuntimeError:
        if fallback_to_config:
            return from_config()
        raise

    resolved: Dict[str, VehicleIoResolution] = {}
    for vehicle_name, vehicle_cfg in manager_config.get("vehicles", {}).items():
        entry = _resolution_from_vehicle(vehicle_name, vehicle_cfg)
        if entry is not None:
            resolved[vehicle_name] = entry
    return resolved


def resolve_vehicle_io(
    vehicle_name: str,
    config_path: Optional[str] = None,
    *,
    source: IoSource = "manager",
    launch: Optional[LaunchIoParams] = None,
    launch_overrides: Optional[Dict[str, Any]] = None,
    node_ros=None,
    timeout_sec: float = 5.0,
    fallback_to_config: bool = False,
) -> VehicleIoResolution:
    """
    Resolve companion UDP endpoints for one vehicle.

    Default: live sim_manager (no config file)::

        from bng_controller.vehicle_session import VehicleSession

        session = VehicleSession.from_vehicle_name("EGO", recreate=True)

    Offline / pre-launch (requires config_path)::

        from bng_controller.vehicle_config import resolve_vehicle_io

        res = resolve_vehicle_io(
            "EGO", "gridworld.yaml", source="config",
        )
    """
    all_resolved = resolve_vehicles_io(
        config_path,
        source=source,
        launch=launch,
        launch_overrides=launch_overrides,
        node_ros=node_ros,
        timeout_sec=timeout_sec,
        fallback_to_config=fallback_to_config,
    )
    if vehicle_name not in all_resolved:
        known = ", ".join(sorted(all_resolved.keys())) or "(none with LLC)"
        raise KeyError(
            f"vehicle '{vehicle_name}' has no resolvable LLC I/O. Known: {known}"
        )
    return all_resolved[vehicle_name]
