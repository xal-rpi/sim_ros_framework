#!/usr/bin/env python3
"""bng_controller.torque_speed_controller

Notebook-friendly torque / wheel-speed command client.

This is intentionally *not* an executable ROS node. It's a small class extending
`rclpy.node.Node` so it can:
- query the simulation manager (`execute_request`) to discover UDP endpoints
- send low-level commands (torque / wheel_speed / vehicle_speed / steering / brake)
  via UDP to the BeamNG Lua controller_manager (socketIn bound on listenIp:listenPort)
- optionally subscribe to `/<vehicle_name>/reduced_state` (published by `gt_state_bridge`)
  and keep the latest state locally for control decisions

Lua command JSON format (see `controller_nn_torque.lua`):
{
  "torque": <number>,        # wheel torque target (Nm)
  "wheel_speed": <number>,  # rear wheel speed target (m/s)
  "vehicle_speed": <number>,# vehicle speed target (m/s)
  "steering": <number>,     # steering input [-1, 1]
  "brake": <number>         # brake [0, 1]
}

Important:
- BeamNG sends reduced GT state over UDP to sendIp:sendPort. The provided
  `gt_state_bridge.py` binds that UDP port and republishes to ROS.
  If you enable ROS state subscription here, make sure `gt_state_bridge` is running.

Example (Jupyter):

    import rclpy
    from bng_controller.torque_speed_controller import TorqueSpeedController

    rclpy.init()
    ctl = TorqueSpeedController(vehicle_name="ego", spin_in_thread=True)

    ctl.command_wheel_speed(12.0)     # m/s
    state = ctl.get_latest_state_dict()

    ctl.stop_spin()
    ctl.close()
    rclpy.shutdown()

"""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float64

from bng_simulator.utils.services_utils import send_request

try:
    from bng_msgs.msg import ReducedGtStateMsg
except Exception:  # pragma: no cover
    ReducedGtStateMsg = None  # type: ignore


@dataclass(frozen=True)
class LowLevelControllerEndpoints:
    """UDP endpoints configured for one vehicle."""

    listen_ip: str
    listen_port: int
    send_ip: Optional[str] = None
    send_port: Optional[int] = None

    @property
    def command_addr(self) -> Tuple[str, int]:
        return (self.listen_ip, self.listen_port)


class TorqueSpeedController(Node):
    """Send torque/speed commands to BeamNG LLC and keep latest state."""

    def __init__(
        self,
        vehicle_name: str,
        *,
        node_name: Optional[str] = None,
        subscribe_reduced_state: bool = True,
        publish_commands: bool = True,
        command_topic_prefix: Optional[str] = None,
        spin_in_thread: bool = False,
        state_topic: Optional[str] = None,
        manager_config_timeout_sec: float = 5.0,
    ):
        super().__init__(node_name or f"torque_speed_controller_{vehicle_name}")

        self.vehicle_name = vehicle_name
        self._lock = threading.Lock()

        self._endpoints = self._discover_endpoints(timeout_sec=manager_config_timeout_sec)

        # UDP socket used to send commands to Lua socketIn (listenIp:listenPort)
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.settimeout(0.2)
        self._last_command_wall_time: float = 0.0
        self._last_command_payload: Dict[str, Any] = {}

        # Optional ROS publishing of commanded targets (useful for PlotJuggler)
        self._publish_commands = bool(publish_commands)
        self._command_topic_prefix = command_topic_prefix or f"/{vehicle_name}/llc_cmd"
        self._cmd_pubs: Dict[str, Any] = {}
        if self._publish_commands:
            self._cmd_pubs = {
                "torque": self.create_publisher(
                    Float64, f"{self._command_topic_prefix}/torque", 10
                ),
                "wheel_speed": self.create_publisher(
                    Float64, f"{self._command_topic_prefix}/wheel_speed", 10
                ),
                "vehicle_speed": self.create_publisher(
                    Float64, f"{self._command_topic_prefix}/vehicle_speed", 10
                ),
                "steering": self.create_publisher(
                    Float64, f"{self._command_topic_prefix}/steering", 10
                ),
                "brake": self.create_publisher(
                    Float64, f"{self._command_topic_prefix}/brake", 10
                ),
            }

        # Latest state cache (ROS subscription)
        self._latest_state_msg = None
        self._latest_state_wall_time: float = 0.0

        self._state_sub = None
        self._state_topic = state_topic or f"/{vehicle_name}/reduced_state"

        if subscribe_reduced_state:
            if ReducedGtStateMsg is None:
                raise RuntimeError(
                    "ReducedGtStateMsg is not importable; "
                    "did you source the workspace / build bng_msgs?"
                )
            self._state_sub = self.create_subscription(
                ReducedGtStateMsg,
                self._state_topic,
                self._on_reduced_state,
                10,
            )
            self.get_logger().info(f"Subscribed to {self._state_topic}")

        # Optional background spinning (useful in notebooks)
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spinning = False

        if spin_in_thread:
            self.start_spin()

        self.get_logger().info(
            f"Ready for vehicle='{vehicle_name}', command_udp={self._endpoints.command_addr}"
        )

    # -----------------
    # Endpoint discovery
    # -----------------

    def _discover_endpoints(self, timeout_sec: float) -> LowLevelControllerEndpoints:
        manager_config = send_request(
            function_name="get_manager_config",
            function_args={},
            timeout_sec=timeout_sec,
            node_ros=self,
        )
        if not manager_config:
            raise RuntimeError("Failed to fetch manager config (get_manager_config)")

        vehicles_cfg = manager_config.get("vehicles", {})
        if self.vehicle_name not in vehicles_cfg:
            available = list(vehicles_cfg.keys())
            raise KeyError(
                f"Vehicle '{self.vehicle_name}' not found in manager config. "
                f"Available: {available}"
            )

        vehicle_cfg = vehicles_cfg[self.vehicle_name]
        llc_cfg = (vehicle_cfg.get("controllers") or {}).get("LowLevelController") or {}
        if not llc_cfg:
            raise KeyError(
                f"No LowLevelController config for vehicle '{self.vehicle_name}'"
            )

        listen_ip = llc_cfg.get("listenIp")
        listen_port = llc_cfg.get("listenPort")
        if not listen_ip or listen_port is None:
            raise KeyError(
                f"LowLevelController missing listenIp/listenPort for '{self.vehicle_name}'"
            )

        send_ip = llc_cfg.get("sendIp")
        send_port = llc_cfg.get("sendPort")

        return LowLevelControllerEndpoints(
            listen_ip=str(listen_ip),
            listen_port=int(listen_port),
            send_ip=str(send_ip) if send_ip else None,
            send_port=int(send_port) if send_port is not None else None,
        )

    @property
    def endpoints(self) -> LowLevelControllerEndpoints:
        """Discovered UDP endpoints for this vehicle."""

        return self._endpoints

    # -----------------
    # ROS reduced state
    # -----------------

    def _on_reduced_state(self, msg) -> None:
        with self._lock:
            self._latest_state_msg = msg
            self._latest_state_wall_time = time.time()

    def get_latest_state_msg(self):
        """Return the latest `ReducedGtStateMsg` (or None if not received)."""

        with self._lock:
            return self._latest_state_msg

    def get_latest_state_dict(self) -> Optional[Dict[str, float]]:
        """Return the latest reduced state as a plain dict (or None)."""

        msg = self.get_latest_state_msg()
        if msg is None:
            return None

        # Keep this explicit (stable) rather than `msg.__dict__`.
        return {
            "t": float(getattr(msg, "time", 0.0)),
            "x": float(getattr(msg, "x", 0.0)),
            "y": float(getattr(msg, "y", 0.0)),
            "yaw": float(getattr(msg, "yaw", 0.0)),
            "V": float(getattr(msg, "vel_mag", 0.0)),
            "vx": float(getattr(msg, "vx", 0.0)),
            "vy": float(getattr(msg, "vy", 0.0)),
            "beta": float(getattr(msg, "beta", 0.0)),
            "r": float(getattr(msg, "r", 0.0)),
            "delta": float(getattr(msg, "delta", 0.0)),
            "wr": float(getattr(msg, "wr", 0.0)),
            "wf": float(getattr(msg, "wf", 0.0)),
            "we": float(getattr(msg, "we", 0.0)),
            "pb": float(getattr(msg, "pb", 0.0)),
            "throttle": float(getattr(msg, "throttle", 0.0)),
            "brake": float(getattr(msg, "brake", 0.0)),
            "accel_x": float(getattr(msg, "accel_x", 0.0)),
            "accel_y": float(getattr(msg, "accel_y", 0.0)),
        }

    def get_state_age_sec(self) -> Optional[float]:
        """Age (wall time) of the latest received reduced_state."""

        with self._lock:
            if self._latest_state_wall_time <= 0:
                return None
            return time.time() - self._latest_state_wall_time

    # ------------
    # UDP commands
    # ------------

    def send_command(
        self,
        *,
        torque: Optional[float] = None,
        wheel_speed: Optional[float] = None,
        vehicle_speed: Optional[float] = None,
        steering: Optional[float] = None,
        brake: Optional[float] = None,
    ) -> None:
        """Send a JSON command over UDP.

        Any field set to None is omitted. On the Lua side, missing fields become nil.
        Sending an empty command (all None) effectively clears all targets.
        """

        payload: Dict[str, Any] = {}
        if torque is not None:
            payload["torque"] = float(torque)
        if wheel_speed is not None:
            payload["wheel_speed"] = float(wheel_speed)
        if vehicle_speed is not None:
            payload["vehicle_speed"] = float(vehicle_speed)
        if steering is not None:
            payload["steering"] = float(steering)
        if brake is not None:
            payload["brake"] = float(brake)

        data = json.dumps(payload).encode("utf-8")
        self._cmd_sock.sendto(data, self._endpoints.command_addr)

        self._last_command_wall_time = time.time()
        self._last_command_payload = payload

        if self._publish_commands and self._cmd_pubs:
            # Publish only the values the caller set (None means "leave unchanged").
            if torque is not None:
                self._cmd_pubs["torque"].publish(Float64(data=float(torque)))
            if wheel_speed is not None:
                self._cmd_pubs["wheel_speed"].publish(Float64(data=float(wheel_speed)))
            if vehicle_speed is not None:
                self._cmd_pubs["wheel_speed"].publish(Float64(data=float(vehicle_speed)))
            if steering is not None:
                self._cmd_pubs["steering"].publish(Float64(data=float(steering)))
            if brake is not None:
                self._cmd_pubs["brake"].publish(Float64(data=float(brake)))

    def command_torque(self, torque_nm: float, *, steering: Optional[float] = None, brake: Optional[float] = None) -> None:
        self.send_command(torque=float(torque_nm), steering=steering, brake=brake)

    def command_wheel_speed(self, wheel_speed_ms: float, *, steering: Optional[float] = None, brake: Optional[float] = None) -> None:
        self.send_command(wheel_speed=float(wheel_speed_ms), steering=steering, brake=brake)

    def command_vehicle_speed(self, vehicle_speed_ms: float, *, steering: Optional[float] = None, brake: Optional[float] = None) -> None:
        self.send_command(vehicle_speed=float(vehicle_speed_ms), steering=steering, brake=brake)

    def clear_targets(self) -> None:
        """Clear all controller targets by sending `{}`."""

        self.send_command()

    def get_last_command(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "wall_time": float(self._last_command_wall_time),
                "payload": dict(self._last_command_payload),
            }

    # ----------------
    # Notebook helpers
    # ----------------

    def start_spin(self) -> None:
        """Start a background thread that spins this node.

        This is convenient in notebooks so subscriptions keep updating without
        the user managing an executor.
        """

        if self._spinning:
            return

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spinning = True

        def _spin_loop() -> None:
            try:
                while self._spinning and rclpy.ok():
                    self._executor.spin_once(timeout_sec=0.05)
            finally:
                try:
                    self._executor.remove_node(self)
                except Exception:
                    pass

        self._spin_thread = threading.Thread(target=_spin_loop, daemon=True)
        self._spin_thread.start()

    def stop_spin(self) -> None:
        if not self._spinning:
            return
        self._spinning = False
        if self._spin_thread:
            self._spin_thread.join(timeout=1.0)
        self._spin_thread = None
        self._executor = None

    def close(self) -> None:
        """Close sockets and stop spinning.

        Note: does not call `rclpy.shutdown()`.
        """

        self.stop_spin()
        try:
            self._cmd_sock.close()
        except Exception:
            pass

    def destroy_node(self):  # type: ignore[override]
        try:
            self.close()
        finally:
            super().destroy_node()
