#!/usr/bin/env python3
"""UDP companion client for the per-vehicle xlab I/O ports (external/Python side).

Processes **outside** BeamNG (sensor_dispatcher, control bridge, smoke tests).
Vehicle Lua (`controller_manager.lua`) owns the simulator side.

All YAML keys are **vehicle/simulator-centric** — they describe what BeamNG does
on that socket:

| YAML key (vehicle view)              | Vehicle (Lua)              | Companion (this client)        |
|--------------------------------------|----------------------------|--------------------------------|
| ``control_listen_ip`` / ``control_listen`` | **BIND** (recv commands)   | **sendto** only (``command_addr``) |
| ``control_state_send_ip`` / ``control_state_send`` | **sendto** (egress state)  | **BIND** (``control_state_bind_addr``) |
| ``sensor_send_ip`` / ``sensor_send`` | **sendto** (observations)  | **BIND** (``sensor_bind_addr``) |

``*_ip`` is the destination IP the vehicle uses when sending (and the IP the
companion binds on for egress ports). Commands are always **sendto** the
vehicle's bound ``control_listen`` address.

Bind rules (per vehicle, per IP:port)
-------------------------------------
- ``control_listen``: one binder (vehicle). Many senders OK.
- ``control_state_send``: one binder on a given IP:port (companion). Vehicle only sendto's.
- ``sensor_send``: one binder on a given IP:port (companion). Vehicle only sendto's.

Gridworld uses **sensor_dispatcher** on ``sensor_send`` only; plant state comes
from the ``control_state`` stream inside sensor batches (not a separate
``control_state_send`` bind in the dispatcher).

Examples
--------

Control bridge / MPC (sendto commands, optionally bind live state)::

    from bng_controller.vehicle_io import VehicleIoClient

    io = VehicleIoClient(
        command_addr=("127.0.0.1", 64257),
        control_state_bind_addr=("127.0.0.1", 64258),
    )
    io.send_command({"torque": 50.0, "steering": 0.0})
    state = io.recv_control_state()
    io.close()

Sensor dispatcher (bind observations; control via ROS on same node)::

    # sensor_dispatcher subscribes to /<vehicle>/control/cmd automatically.
    # For tune/calibrate from a notebook (infrequent), use VehicleSession:

    from bng_controller.vehicle_session import VehicleSession

    session = VehicleSession.from_llc_config(llc_cfg)
    session.calibrate({"gains": {"kp": 4000.0, "ki": 13.0}})
    session.close()
"""

from __future__ import annotations

import json
import select
import socket
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

Addr = Tuple[str, int]

_SENSOR_BATCH_META_KEYS = frozenset({"sent_t"})


@dataclass(frozen=True)
class VehicleIoEndpoints:
    """Companion-side endpoints derived from LowLevelController YAML."""

    command_addr: Addr
    control_state_bind_addr: Addr
    sensor_bind_addr: Optional[Addr] = None

    @classmethod
    def from_low_level_controller_config(cls, llc_cfg: Dict[str, Any]) -> "VehicleIoEndpoints":
        listen_ip = llc_cfg["control_listen_ip"]
        listen_port = llc_cfg["control_listen"]
        control_state_send_ip = llc_cfg["control_state_send_ip"]
        control_state_send_port = llc_cfg["control_state_send"]

        sensor_bind = None
        if "sensor_send" in llc_cfg:
            sensor_ip = llc_cfg["sensor_send_ip"]
            sensor_port = llc_cfg["sensor_send"]
            sensor_bind = (str(sensor_ip), int(sensor_port))

        return cls(
            command_addr=(str(listen_ip), int(listen_port)),
            control_state_bind_addr=(
                str(control_state_send_ip),
                int(control_state_send_port),
            ),
            sensor_bind_addr=sensor_bind,
        )


class VehicleIoClient:
    """Companion UDP client: sendto vehicle commands, optionally bind egress ports."""

    def __init__(
        self,
        command_addr: Addr,
        control_state_bind_addr: Optional[Addr] = None,
        sensor_bind_addr: Optional[Addr] = None,
        recv_timeout: float = 0.2,
        bufsize: int = 8192,
    ) -> None:
        self.command_addr = command_addr
        self.control_state_bind_addr = control_state_bind_addr
        self.sensor_bind_addr = sensor_bind_addr
        self.recv_timeout = recv_timeout
        self.bufsize = bufsize

        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._control_state_recv_sock: Optional[socket.socket] = None
        if control_state_bind_addr is not None:
            self._control_state_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._control_state_recv_sock.bind(control_state_bind_addr)
            self._control_state_recv_sock.settimeout(recv_timeout)

        self._sensor_recv_sock: Optional[socket.socket] = None
        if sensor_bind_addr is not None:
            self._sensor_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sensor_recv_sock.bind(sensor_bind_addr)
            self._sensor_recv_sock.settimeout(recv_timeout)

    @classmethod
    def from_endpoints(cls, endpoints: VehicleIoEndpoints, **kwargs: Any) -> "VehicleIoClient":
        return cls(
            endpoints.command_addr,
            control_state_bind_addr=endpoints.control_state_bind_addr,
            sensor_bind_addr=endpoints.sensor_bind_addr,
            **kwargs,
        )

    def close(self) -> None:
        for sock in (self._send_sock, self._control_state_recv_sock, self._sensor_recv_sock):
            if sock is None:
                continue
            try:
                sock.close()
            except OSError:
                pass

    def __enter__(self) -> "VehicleIoClient":
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def _send_envelope(self, msg_type: str, data: Optional[Dict[str, Any]] = None) -> None:
        payload: Dict[str, Any] = {"type": msg_type}
        if data is not None:
            payload["data"] = data
        packet = json.dumps(payload).encode("utf-8")
        self._send_sock.sendto(packet, self.command_addr)

    def send_raw(self, payload: Dict[str, Any]) -> None:
        """Send a JSON object without the cmd/tune envelope."""
        packet = json.dumps(payload).encode("utf-8")
        self._send_sock.sendto(packet, self.command_addr)

    def send_command(self, data: Dict[str, Any]) -> None:
        self._send_envelope("cmd", data)

    def send_tune(self, data: Dict[str, Any]) -> None:
        self._send_envelope("tune", data)

    def send_bypass(self, enabled: bool) -> None:
        self._send_envelope("bypass", {"enabled": bool(enabled)})

    def send_reset(self) -> None:
        self._send_envelope("reset")

    @staticmethod
    def _recv(sock: socket.socket, timeout: Optional[float], bufsize: int) -> List[bytes]:
        ready, _, _ = select.select([sock], [], [], timeout)
        if not ready:
            return []
        packets: List[bytes] = []
        data, _ = sock.recvfrom(bufsize)
        packets.append(data)
        sock.setblocking(False)
        try:
            while True:
                data, _ = sock.recvfrom(bufsize)
                packets.append(data)
        except BlockingIOError:
            pass
        finally:
            sock.setblocking(True)
        return packets

    @staticmethod
    def _recv_last(sock: socket.socket, timeout: Optional[float], bufsize: int) -> Optional[bytes]:
        packets = VehicleIoClient._recv(sock, timeout, bufsize)
        if not packets:
            return None
        return packets[-1]

    @staticmethod
    def _expand_sensor_packet(packet: Dict[str, Any]) -> List[Dict[str, Any]]:
        observations: List[Dict[str, Any]] = []
        for key, value in packet.items():
            if key in _SENSOR_BATCH_META_KEYS:
                continue
            if isinstance(value, dict) and value.get("sensor"):
                observations.append(value)
        if not observations:
            raise ValueError(
                "expected map-batch sensor packet "
                f"(top-level keys: {sorted(packet.keys())})"
            )
        return observations

    @staticmethod
    def _decode_sensor_packets(packets: List[bytes]) -> List[Dict[str, Any]]:
        return [json.loads(raw.decode("utf-8")) for raw in packets]

    def recv_control_state(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """Return the latest controlStateOut JSON dict, or None on timeout."""
        if self._control_state_recv_sock is None:
            raise RuntimeError("control_state_bind_addr was not configured on this client")
        raw = self._recv_last(
            self._control_state_recv_sock,
            timeout if timeout is not None else self.recv_timeout,
            self.bufsize,
        )
        if raw is None:
            return None
        return json.loads(raw.decode("utf-8"))

    def recv_sensor(self, timeout: Optional[float] = None) -> List[Dict[str, Any]]:
        """Return observations flattened from every map-batch datagram in this poll."""
        if self._sensor_recv_sock is None:
            raise RuntimeError("sensor_bind_addr was not configured on this client")
        packets = self._recv(
            self._sensor_recv_sock,
            timeout if timeout is not None else self.recv_timeout,
            self.bufsize,
        )
        observations: List[Dict[str, Any]] = []
        for packet in self._decode_sensor_packets(packets):
            observations.extend(self._expand_sensor_packet(packet))
        return observations

    def recv_sensor_last(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """Return the last observation from the latest datagram in this poll."""
        if self._sensor_recv_sock is None:
            raise RuntimeError("sensor_bind_addr was not configured on this client")
        raw = self._recv_last(
            self._sensor_recv_sock,
            timeout if timeout is not None else self.recv_timeout,
            self.bufsize,
        )
        if raw is None:
            return None
        expanded = self._expand_sensor_packet(json.loads(raw.decode("utf-8")))
        return expanded[-1] if expanded else None
