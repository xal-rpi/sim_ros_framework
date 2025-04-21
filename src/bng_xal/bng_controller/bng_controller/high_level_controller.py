#!/usr/bin/env python3

import socket
import json
import threading
import time
from rclpy import logging
from rclpy.node import Node
from typing import Dict, Any

# Import C extension for expensive computations
try:
    from bng_controller.core import controller_core
except ImportError:
    # Fallback if C extension not built
    logging.get_logger("controller").error(
        "C extension not found, using Python implementation (empty)"
    )

    # Python implementation of controller_core
    class controller_core:
        @staticmethod
        def compute_control_targets(sensor_data):
            # Python fallback implementation - simple controls for testing
            # Just input a constant small steering angle and some throttle
            engine_torque = 100.0  # Small constant torque for testing
            road_wheel_angle = 0.1  # Small constant steering angle for testing
            brake_torque = 0.0  # No braking
            return (engine_torque, road_wheel_angle, brake_torque)


class HighLevelController(Node):
    def __init__(
        self,
        listen_ip: str = "0.0.0.0",  # Listen on all interfaces
        listen_port: int = 64258,  # This is the port that the Lua controller sends to
        send_ip: str = "172.26.32.1",  # The BeamNG host IP
        send_port: int = 64257,  # This is the port that the Lua controller listens on
        logger=None,
    ):
        super().__init__("high_level_controller")

        # Communication settings
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.send_ip = send_ip
        self.send_port = send_port

        self.logger = logger or logging.get_logger(self.__class__.__name__)
        # Set log level to debug
        self.logger.set_level(logging.LoggingSeverity.DEBUG)

        # Initialize UDP sockets with better error handling
        try:
            self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.listen_socket.bind((self.listen_ip, self.listen_port))
            self.logger.info(
                f"Successfully bound to {self.listen_ip}:{self.listen_port}"
            )
        except Exception as e:
            self.logger.error(f"Failed to bind listen socket: {e}")
            raise

        try:
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.logger.info(
                f"Created send socket targeting {self.send_ip}:{self.send_port}"
            )
        except Exception as e:
            self.logger.error(f"Failed to create send socket: {e}")
            raise

        # State variables
        self.latest_sensor_data = {}
        self.running = False
        self.exit_event = threading.Event()
        self.control_rate = 0.1  # 10Hz update rate
        self.message_counter = 0
        self.last_receive_time = 0

        # Thread for receiving data
        self.receive_thread = None

        # ROS timer for control loop
        self.timer = self.create_timer(self.control_rate, self._control_callback)
        self.timer.cancel()  # Don't start right away

    def start(self):
        """Start the controller threads"""
        try:
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_sensor_data)
            self.receive_thread.daemon = True
            self.receive_thread.start()

            # Start control timer
            self.timer.reset()

            self.logger.info("Controller started")
            return True
        except Exception as e:
            self.logger.error(f"Failed to start controller: {e}")
            self.running = False
            return False

    def stop(self):
        """Stop the high-level controller."""
        if not hasattr(self, 'is_running') or not self.is_running:
            self.logger.debug("Controller already stopped, skipping")
            return

        self.is_running = False

        if hasattr(self, 'controller') and self.controller:
            try:
                self.controller.stop()
            except Exception as e:
                self.logger.error(f"Error stopping controller: {e}")

        self.logger.info("Controller stopped")

    def _receive_sensor_data(self):
        """Thread function to receive sensor data from BeamNG"""
        self.listen_socket.settimeout(0.2)  # 200ms timeout

        while self.running and not self.exit_event.is_set():
            try:
                data, addr = self.listen_socket.recvfrom(8192)
                self.last_receive_time = time.time()

                try:
                    self.latest_sensor_data = json.loads(data.decode("utf-8"))
                    self.message_counter += 1
                    self.logger.info(
                        f"Received message #{self.message_counter} from {addr}",
                        throttle_duration_sec=2,
                    )
                    self.logger.debug(
                        f"Parsed data: {str(self.latest_sensor_data)[:200]}...",
                        throttle_duration_sec=2,
                    )
                except json.JSONDecodeError as je:
                    self.logger.error(f"Failed to parse JSON: {je}")

            except socket.timeout:
                # Check exit flag more frequently
                if not self.running or self.exit_event.is_set():
                    break
                continue
            except Exception as e:
                if self.running and not self.exit_event.is_set():
                    self.logger.error(f"Error receiving sensor data: {e}")
                break  # Exit the loop on any other exception

        self.logger.info("Receive thread terminating")

    def _control_callback(self):
        """Control callback function executed at the control rate by ROS timer"""
        if not self.running:
            return

        try:
            # Even without sensor data, send control commands for testing
            if not self.latest_sensor_data and self.message_counter == 0:
                self.logger.warn(
                    "No sensor data received yet, using default controls for testing"
                )
                engine_torque = 50.0  # Small torque
                road_wheel_angle = 0.1  # Small steering angle
                brake_torque = 0.0  # No braking
            else:
                # Compute control targets using C extension or fallback
                engine_torque, road_wheel_angle, brake_torque = (
                    controller_core.compute_control_targets(self.latest_sensor_data)
                )

            # Prepare control message
            control_msg = {
                "engine_torque": engine_torque,
                "road_wheel_angle": road_wheel_angle,
                "brake_torque": brake_torque,
                "timestamp": int(time.time()),
            }

            # Send control targets to BeamNG
            self._send_control_targets(control_msg)

        except Exception as e:
            self.logger.error(f"Error in control loop: {e}")

    def _send_control_targets(self, control_msg: Dict[str, Any]):
        """Send control targets to BeamNG low-level controller"""
        try:
            # Convert to JSON and send
            msg_bytes = json.dumps(control_msg).encode("utf-8")
            self.send_socket.sendto(msg_bytes, (self.send_ip, self.send_port))
            self.logger.debug(f"Sent control targets: {control_msg}")
        except Exception as e:
            self.logger.error(f"Error sending control targets: {e}")
