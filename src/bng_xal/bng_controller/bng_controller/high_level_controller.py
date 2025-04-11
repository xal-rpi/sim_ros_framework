#!/usr/bin/env python3

import socket
import json
import threading
from rclpy.node import Node
from typing import Dict, Any, Optional, Tuple

# Import C extension for expensive computations
try:
    from bng_controller.core import controller_core
except ImportError:
    # Fallback if C extension not built
    import logging

    logging.warning("C extension not found, using Python implementation (empty)")

    # Python implementation of controller_core
    class controller_core:
        @staticmethod
        def compute_control_targets(sensor_data):
            # Python fallback implementation
            return (0.0, 0.0, 0.0)  # engine_torque, road_wheel_angle, brake_torque


class HighLevelController(Node):
    def __init__(
        self,
        listen_ip: str = "127.0.0.1",
        listen_port: int = 64257,
        send_ip: str = "127.0.0.1",
        send_port: int = 64258,
    ):
        super().__init__("high_level_controller")

        # Communication settings
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.send_ip = send_ip
        self.send_port = send_port

        # Log parameters
        self.get_logger().info(
            f"Initializing with listen={listen_ip}:{listen_port}, send={send_ip}:{send_port}"
        )

        # Initialize UDP sockets
        self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_socket.bind((self.listen_ip, self.listen_port))
        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # State variables
        self.latest_sensor_data = {}
        self.running = False
        self.control_rate = 0.01  # 100Hz

        # Thread for receiving data
        self.receive_thread = None

        # ROS timer for control loop (if needed)
        self.timer = self.create_timer(self.control_rate, self._control_callback)
        self.timer.cancel()  # Don't start right away

    def start(self):
        """Start the controller threads"""
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_sensor_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        # Start control timer
        self.timer.reset()

        self.get_logger().info("Controller started")

    def stop(self):
        """Stop the controller threads"""
        self.running = False

        # Stop timer
        self.timer.cancel()

        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)

        self.listen_socket.close()
        self.send_socket.close()

        self.get_logger().info("Controller stopped")

    def _receive_sensor_data(self):
        """Thread function to receive sensor data from BeamNG"""
        self.listen_socket.settimeout(0.5)  # 500ms timeout

        while self.running:
            try:
                data, addr = self.listen_socket.recvfrom(8192)  # Buffer size
                # Parse JSON data
                self.latest_sensor_data = json.loads(data.decode("utf-8"))
                self.get_logger().debug(f"Received sensor data from {addr}")
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Error receiving sensor data: {e}")

    def _control_callback(self):
        """Control callback function executed at 100Hz by ROS timer"""
        if not self.running or not self.latest_sensor_data:
            return

        try:
            # Compute control targets using C extension
            engine_torque, road_wheel_angle, brake_torque = (
                controller_core.compute_control_targets(self.latest_sensor_data)
            )

            # Prepare control message
            control_msg = {
                "engine_torque": engine_torque,
                "road_wheel_angle": road_wheel_angle,
                "brake_torque": brake_torque,
                "timestamp": self.get_clock().now().to_msg().sec,
            }

            # Send control targets to BeamNG
            self._send_control_targets(control_msg)

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")

    def _send_control_targets(self, control_msg: Dict[str, Any]):
        """Send control targets to BeamNG low-level controller"""
        try:
            # Convert to JSON and send
            msg_bytes = json.dumps(control_msg).encode("utf-8")
            self.send_socket.sendto(msg_bytes, (self.send_ip, self.send_port))
            self.get_logger().debug("Sent control targets")
        except Exception as e:
            self.get_logger().error(f"Error sending control targets: {e}")
