"""Per-vehicle xlab UDP port injection for resolved scenario configs."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Tuple


def resolved_udp_ports(
    scenario_config: Dict[str, Any],
    vehicle_cfg: Dict[str, Any],
    *,
    listen_port: int,
    send_port: int,
    sensor_port: int,
) -> Tuple[int, int, int]:
    """Compute per-vehicle UDP ports from udp_io defaults and io.port_index."""
    udp_io = scenario_config.get("udp_io", {})
    stride = int(udp_io.get("port_stride", 10))
    base_listen = int(udp_io.get("control_listen", listen_port))
    base_send = int(udp_io.get("control_state_send", send_port))
    base_sensor = int(udp_io.get("sensor_send", sensor_port))
    port_index = int(vehicle_cfg.get("io", {}).get("port_index", 0))
    offset = port_index * stride
    return (
        base_listen + offset,
        base_send + offset,
        base_sensor + offset,
    )


def inject_llc_io_ports(
    llc_cfg: Dict[str, Any],
    *,
    beamng_host: str,
    remote_host: str,
    listen_port: int,
    send_port: int,
    sensor_port: int,
) -> None:
    """Inject xlab UDP endpoints into a LowLevelController dict."""
    egress_ip = remote_host or beamng_host
    if "control_listen_ip" not in llc_cfg:
        llc_cfg["control_listen_ip"] = beamng_host
    if "control_listen" not in llc_cfg:
        llc_cfg["control_listen"] = int(listen_port)
    if "control_state_send_ip" not in llc_cfg:
        llc_cfg["control_state_send_ip"] = egress_ip
    if "control_state_send" not in llc_cfg:
        llc_cfg["control_state_send"] = int(send_port)
    if "sensor_send_ip" not in llc_cfg:
        llc_cfg["sensor_send_ip"] = egress_ip
    if "sensor_send" not in llc_cfg:
        llc_cfg["sensor_send"] = int(sensor_port)


def inject_vehicle_io_ports(
    scenario_config: Dict[str, Any],
    *,
    beamng_host: str,
    remote_host: str,
    listen_port: int,
    send_port: int,
    sensor_port: int,
) -> None:
    """Inject per-vehicle UDP ports into all LLC controllers in-place.

    ``listen_port`` / ``send_port`` / ``sensor_port`` are the BASE triplet for
    ``io.port_index == 0``. Each vehicle uses base + port_index * port_stride.
    Persisted on ``scenario_config['udp_io']`` so get_manager_config reflects
    the live port model.
    """
    udp_io = scenario_config.setdefault("udp_io", {})
    udp_io["control_listen"] = int(listen_port)
    udp_io["control_state_send"] = int(send_port)
    udp_io["sensor_send"] = int(sensor_port)

    for vehicle_cfg in scenario_config.get("vehicles", {}).values():
        ports = resolved_udp_ports(
            scenario_config,
            vehicle_cfg,
            listen_port=listen_port,
            send_port=send_port,
            sensor_port=sensor_port,
        )
        for llc_cfg in vehicle_cfg.get("controllers", {}).values():
            inject_llc_io_ports(
                llc_cfg,
                beamng_host=beamng_host,
                remote_host=remote_host,
                listen_port=ports[0],
                send_port=ports[1],
                sensor_port=ports[2],
            )


def vehicles_with_io(
    scenario_config: Dict[str, Any],
    *,
    beamng_host: str,
    remote_host: str,
    listen_port: int,
    send_port: int,
    sensor_port: int,
) -> Dict[str, Any]:
    """Return vehicles dict copy with per-vehicle LLC I/O ports injected."""
    vehicles = deepcopy(scenario_config.get("vehicles", {}))
    wrapped = deepcopy(scenario_config)
    wrapped["vehicles"] = vehicles
    inject_vehicle_io_ports(
        wrapped,
        beamng_host=beamng_host,
        remote_host=remote_host,
        listen_port=listen_port,
        send_port=send_port,
        sensor_port=sensor_port,
    )
    return vehicles
