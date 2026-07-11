#!/usr/bin/env python3
"""Vehicle bridge: sensor_send → ROS topics + BngControlCmd → control_listen.

Binds UDP at node startup from the same run YAML as sim_manager (source=config).
External scripts should use ``VehicleSession.from_vehicle_name`` with the default
source=manager to match the live simulation after startup.

See sensor_converters.py for UDP wire formats.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from bng_msgs.msg import BngControlCmd

from bng_controller.sensor_converters import (
    ObservationSensor,
    observation_to_ros_msg,
    ros_msg_type_for,
)
from bng_controller.vehicle_config import (
    LaunchIoParams,
    VehicleIoResolution,
    resolve_vehicles_io,
)
from bng_controller.vehicle_io import VehicleIoClient
from bng_simulator.utils.scenario_compose import launch_overrides_from_ros
from bng_controller.vehicle_session import (
    VehicleSession,
    clear_vehicle_sessions,
    register_vehicle_session,
    unregister_vehicle_session,
)


@dataclass(frozen=True)
class StreamPublisher:
    sensor_type: str
    topic: str
    publisher: Publisher


class VehicleSensorReceiver:
    """UDP receiver and ROS publishers for one vehicle's sensor_send stream."""

    def __init__(
        self,
        vehicle_name: str,
        io_client: VehicleIoClient,
        stream_publishers: Dict[str, StreamPublisher],
        logger,
        frame_id: str,
        ros_time_fn,
    ) -> None:
        self.vehicle_name = vehicle_name
        self._io = io_client
        self._streams = stream_publishers
        self._logger = logger
        self._frame_id = frame_id
        self._ros_time_fn = ros_time_fn
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.message_count = 0
        self.last_receive_time = 0.0

    def start(self) -> bool:
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self) -> None:
        if not self.running:
            return
        self.running = False
        if self.thread:
            self.thread.join(timeout=0.5)
        self._io.close()
        self._logger.info(
            f"[{self.vehicle_name}] Sensor receiver stopped "
            f"({self.message_count} observations published)"
        )

    def _receive_loop(self) -> None:
        self._logger.info(f"[{self.vehicle_name}] Sensor receiver thread started")
        consecutive_timeouts = 0
        max_consecutive_timeouts = 50

        while self.running:
            try:
                observations = self._io.recv_sensor(timeout=0.2)
                if not observations:
                    consecutive_timeouts += 1
                    if consecutive_timeouts >= max_consecutive_timeouts:
                        self._logger.warn(
                            f"[{self.vehicle_name}] No sensor data for "
                            f"{max_consecutive_timeouts * 0.2:.0f}s"
                        )
                        consecutive_timeouts = 0
                    continue

                self.last_receive_time = time.time()
                consecutive_timeouts = 0

                for observation in observations:
                    stream_name = observation.get("name")
                    if not stream_name or stream_name not in self._streams:
                        continue

                    stream = self._streams[stream_name]
                    msg = observation_to_ros_msg(
                        observation,
                        stream.sensor_type,
                        self._frame_id,
                    )
                    msg.header.stamp = self._ros_time_fn()
                    stream.publisher.publish(msg)
                    self.message_count += 1

            except Exception as exc:
                self._logger.error(f"[{self.vehicle_name}] Sensor receive error: {exc}")
                break

        self._logger.info(f"[{self.vehicle_name}] Sensor receiver thread exiting")


class SensorDispatcher(Node):
    """Multi-vehicle sensor_send → ROS + BngControlCmd → control_listen bridge."""

    vehicle_sessions: Dict[str, VehicleSession] = {}

    @classmethod
    def get_vehicle_session(cls, vehicle_name: str) -> Optional[VehicleSession]:
        return cls.vehicle_sessions.get(vehicle_name)

    def __init__(self) -> None:
        super().__init__("sensor_dispatcher")

        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("config", "")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("remote", "")
        self.declare_parameter("bng_listen_port", 64257)
        self.declare_parameter("bng_send_port", 64258)
        self.declare_parameter("bng_sensor_port", 64259)
        self.declare_parameter("vehicle", "")
        self.declare_parameter("vehicle_id", "")
        self.declare_parameter("level", "")
        self.declare_parameter("spawn", "")
        self.declare_parameter("yaw", "")
        self.declare_parameter("pos", "")
        self.declare_parameter("preset", "")

        log_level_str = self.get_parameter("log_level").value.upper()
        lvl_map = {
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        severity = lvl_map.get(log_level_str, rclpy.logging.LoggingSeverity.INFO)
        rclpy.logging.set_logger_level(self.get_logger().name, severity)

        self.frame_id = self.get_parameter("frame_id").value
        self.receivers: Dict[str, VehicleSensorReceiver] = {}
        self._control_subscriptions = []

        self._setup_vehicle_receivers()

        self.get_logger().info(
            f"Vehicle bridge initialized with {len(self.receivers)} vehicle(s)"
        )

    def _on_control_cmd(self, session: VehicleSession, vehicle_name: str, msg: BngControlCmd) -> None:
        if msg.valid_fields == 0:
            return
        session.send_control_cmd(msg)

    @staticmethod
    def _resolve_topic(
        vehicle_name: str,
        stream_name: str,
        sensor_type: str,
        entry: Dict[str, Any],
    ) -> str:
        topic = entry.get("topic")
        if topic:
            return str(topic)
        return f"/{vehicle_name}/sensors/{stream_name}/{sensor_type}"

    def _setup_vehicle_receivers(self) -> None:
        config_path = str(self.get_parameter("config").value)
        if not config_path:
            raise RuntimeError("sensor_dispatcher requires 'config' launch parameter")

        launch = LaunchIoParams(
            beamng_host=str(self.get_parameter("host").value),
            remote_host=str(self.get_parameter("remote").value),
            listen_port=int(self.get_parameter("bng_listen_port").value),
            send_port=int(self.get_parameter("bng_send_port").value),
            sensor_port=int(self.get_parameter("bng_sensor_port").value),
        )

        self.get_logger().info(f"Loading scenario config: {config_path}")
        resolved = resolve_vehicles_io(
            config_path,
            source="config",
            launch=launch,
            launch_overrides=launch_overrides_from_ros(self),
        )

        if not resolved:
            self.get_logger().warn("No vehicles with LLC in scenario config")
            return

        bound_ports: Dict[Tuple[str, int], str] = {}
        for vehicle_name, resolution in resolved.items():
            self._create_vehicle_receiver(vehicle_name, resolution, bound_ports)

    def _create_vehicle_receiver(
        self,
        vehicle_name: str,
        resolution: VehicleIoResolution,
        bound_ports: Dict[Tuple[str, int], str],
    ) -> None:
        llc_cfg = resolution.llc_config
        endpoints = resolution.endpoints

        broadcast_cfg = llc_cfg.get("sensor_broadcast")
        if not broadcast_cfg:
            self.get_logger().info(
                f"[{vehicle_name}] No sensor_broadcast configured, skipping"
            )
            return

        if endpoints.sensor_bind_addr is None:
            self.get_logger().warn(
                f"[{vehicle_name}] sensor_send not configured, skipping"
            )
            return

        bind_addr = endpoints.sensor_bind_addr
        if bind_addr in bound_ports:
            self.get_logger().error(
                f"[{vehicle_name}] sensor_send {bind_addr[0]}:{bind_addr[1]} "
                f"already bound by {bound_ports[bind_addr]} — "
                "assign unique io.port_index per vehicle (see udp_io.yaml port_stride)"
            )
            return
        bound_ports[bind_addr] = vehicle_name

        stream_publishers: Dict[str, StreamPublisher] = {}
        for stream_name, entry in broadcast_cfg.items():
            if not isinstance(entry, dict):
                continue
            sensor_type_raw = str(entry["sensor"])
            try:
                sensor_type = ObservationSensor(sensor_type_raw)
            except ValueError:
                self.get_logger().warn(
                    f"[{vehicle_name}] Unsupported sensor type '{sensor_type_raw}' "
                    f"for stream '{stream_name}', skipping publisher"
                )
                continue
            msg_type = ros_msg_type_for(sensor_type)

            topic = self._resolve_topic(vehicle_name, stream_name, sensor_type.value, entry)
            publisher = self.create_publisher(msg_type, topic, 10)
            stream_publishers[stream_name] = StreamPublisher(
                sensor_type=sensor_type.value,
                topic=topic,
                publisher=publisher,
            )
            self.get_logger().info(
                f"[{vehicle_name}] {stream_name} ({sensor_type.value}) -> {topic}"
            )

        if not stream_publishers:
            self.get_logger().warn(
                f"[{vehicle_name}] No publishable sensor_broadcast streams"
            )
            return

        io_client = VehicleIoClient(
            command_addr=endpoints.command_addr,
            control_state_bind_addr=None,
            sensor_bind_addr=endpoints.sensor_bind_addr,
        )
        session = VehicleSession(io_client)
        register_vehicle_session(vehicle_name, session)
        SensorDispatcher.vehicle_sessions[vehicle_name] = session

        control_topic = f"/{vehicle_name}/control/cmd"
        sub = self.create_subscription(
            BngControlCmd,
            control_topic,
            lambda msg, s=session, v=vehicle_name: self._on_control_cmd(s, v, msg),
            10,
        )
        self._control_subscriptions.append(sub)
        self.get_logger().info(f"[{vehicle_name}] BngControlCmd <- {control_topic}")

        receiver = VehicleSensorReceiver(
            vehicle_name=vehicle_name,
            io_client=io_client,
            stream_publishers=stream_publishers,
            logger=self.get_logger(),
            frame_id=self.frame_id,
            ros_time_fn=lambda: self.get_clock().now().to_msg(),
        )

        self.get_logger().info(
            f"[{vehicle_name}] Binding sensor_send on "
            f"{bind_addr[0]}:{bind_addr[1]}"
        )
        if receiver.start():
            self.receivers[vehicle_name] = receiver
        else:
            io_client.close()

    def destroy_node(self) -> bool:
        self.get_logger().info("Shutting down sensor dispatcher...")
        for receiver in self.receivers.values():
            receiver.stop()
        for vehicle_name in list(self.receivers.keys()):
            unregister_vehicle_session(vehicle_name)
            SensorDispatcher.vehicle_sessions.pop(vehicle_name, None)
        clear_vehicle_sessions()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0

    try:
        node = SensorDispatcher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, cleaning up sensor dispatcher...")
    except Exception as exc:
        print(f"Uncaught exception: {exc}")
        import traceback

        traceback.print_exc()
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
