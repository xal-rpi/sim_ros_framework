"""Unit tests for per-vehicle UDP port injection."""

from __future__ import annotations

from bng_simulator.utils.scenario_compose import compose_scenario
from bng_simulator.utils.vehicle_io_config import (
    inject_vehicle_io_ports,
    resolved_udp_ports,
)


def _sample_scenario() -> dict:
    return compose_scenario("multi_agent.yaml")


def test_resolved_udp_ports_stride() -> None:
    scenario = _sample_scenario()
    ego_cfg = scenario["vehicles"]["EGO"]
    agent2_cfg = scenario["vehicles"]["AGENT2"]
    base_listen, base_send, base_sensor = resolved_udp_ports(
        scenario,
        ego_cfg,
        listen_port=64257,
        send_port=64258,
        sensor_port=64259,
    )
    agent2_ports = resolved_udp_ports(
        scenario,
        agent2_cfg,
        listen_port=64257,
        send_port=64258,
        sensor_port=64259,
    )
    stride = scenario["udp_io"]["port_stride"]
    assert agent2_ports[0] == base_listen + stride
    assert agent2_ports[1] == base_send + stride
    assert agent2_ports[2] == base_sensor + stride


def test_inject_vehicle_io_ports_into_llc() -> None:
    scenario = _sample_scenario()
    inject_vehicle_io_ports(
        scenario,
        beamng_host="10.0.0.1",
        remote_host="10.0.0.2",
        listen_port=64000,
        send_port=64001,
        sensor_port=64002,
    )
    ego_llc = scenario["vehicles"]["EGO"]["controllers"]["LowLevelController"]
    agent2_llc = scenario["vehicles"]["AGENT2"]["controllers"]["LowLevelController"]
    assert ego_llc["control_listen"] == 64000
    assert ego_llc["control_listen_ip"] == "10.0.0.1"
    assert ego_llc["sensor_send_ip"] == "10.0.0.2"
    assert agent2_llc["control_listen"] == 64010
