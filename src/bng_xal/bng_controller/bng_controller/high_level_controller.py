#!/usr/bin/env python3

import socket
import json
import threading
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Import C extension for expensive computations
try:
    from bng_controller.core import controller_core
except ImportError:
    rclpy.logging.get_logger("controller").error(
        "C extension not found, using Python implementation (empty)"
    )

    # Fallback if C extension not built
    class controller_core:
        @staticmethod
        def compute_control_targets(sensor_data):
            # Python fallback implementation - simple controls for testing
            engine_torque = 100.0  # Small constant torque for testing
            road_wheel_angle = 0.1  # Small constant steering angle for testing
            brake_torque = 0.0  # No braking
            return (engine_torque, road_wheel_angle, brake_torque)


class HighLevelController(Node):
    def __init__(self):
        """Initialize the high-level controller node."""
        super().__init__("high_level_controller")

        # Declare parameters
        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 64258)
        self.declare_parameter("send_ip", "172.26.32.1")
        self.declare_parameter("send_port", 64257)
        self.declare_parameter("control_rate", 0.1)

        # Get parameters
        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = self.get_parameter("listen_port").value
        self.send_ip = self.get_parameter("send_ip").value
        self.send_port = self.get_parameter("send_port").value
        self.control_rate = self.get_parameter("control_rate").value

        # State variables
        self.latest_sensor_data = {}
        self.running = False
        self.exit_event = threading.Event()
        self.message_counter = 0
        self.last_receive_time = 0

        # Initialize sockets
        self._initialize_sockets()

        # Create control timer
        self.timer = self.create_timer(self.control_rate, self._control_callback)
        # Initially pause the timer
        self.timer.cancel()

        # Thread for receiving data
        self.receive_thread = None

        self.get_logger().info("High-level controller node initialized")

    def _initialize_sockets(self):
        """Initialize UDP sockets for communication."""
        try:
            # Socket for receiving vehicle state data
            self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.listen_socket.bind((self.listen_ip, self.listen_port))
            self.listen_socket.settimeout(0.2)  # 200ms timeout

            # Socket for sending control commands
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.get_logger().info(
                f"Sockets initialized - Listening on {self.listen_ip}:{self.listen_port}, "
                f"Sending to {self.send_ip}:{self.send_port}"
            )

            self.get_logger().debug("Sockets initialized")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize sockets: {e}")
            raise

    def start(self):
        """Start the controller threads and timer."""
        if self.running:
            self.get_logger().warn("Controller already running")
            return True

        try:
            self.running = True

            # Start receive thread
            if not self.receive_thread or not self.receive_thread.is_alive():
                self.receive_thread = threading.Thread(target=self._receive_sensor_data)
                self.receive_thread.daemon = True
                self.receive_thread.start()

            # Start control timer
            self.timer.reset()
            self.get_logger().info("Control timer started")

            self.get_logger().debug("Controller started")

            return True
        except Exception as e:
            self.get_logger().error(f"Failed to start controller: {e}")
            self.running = False
            return False

    def stop(self):
        """Stop the controller threads and timer."""
        if not self.running:
            self.get_logger().debug("Controller already stopped")
            return

        self.get_logger().info("Stopping controller...")

        # Signal threads to exit
        self.running = False
        self.exit_event.set()

        # Cancel the timer
        self.timer.cancel()

        # Wait for threads to terminate
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)

        self.get_logger().info("Controller stopped")

    def _receive_sensor_data(self):
        """Thread function to receive sensor data from BeamNG."""
        self.get_logger().info("Receive thread started")

        while self.running and not self.exit_event.is_set():
            try:
                data, addr = self.listen_socket.recvfrom(8192)
                self.last_receive_time = time.time()

                try:
                    self.latest_sensor_data = json.loads(data.decode("utf-8"))
                    self.message_counter += 1

                    # Log every 10th message to reduce log spam
                    if self.message_counter % 10 == 0:
                        self.get_logger().info(
                            f"Received message #{self.message_counter} from {addr}"
                        )

                except json.JSONDecodeError as je:
                    self.get_logger().error(f"Failed to parse JSON: {je}")

            except socket.timeout:
                # This is normal, continue
                continue
            except Exception as e:
                if self.running and not self.exit_event.is_set():
                    self.get_logger().error(f"Error receiving sensor data: {e}")
                break

        self.get_logger().info("Receive thread terminating")

    def _control_callback(self):
        """Control callback function executed at the control rate by ROS timer."""
        callback_time = time.time()
        self.get_logger().info(
            f"Control callback executing at {callback_time:.3f}",
            throttle_duration_sec=1,
        )

        # Skip if controller is stopped
        if not self.running:
            self.get_logger().warn("Control callback running but controller is stopped")
            return

        try:
            # Compute control targets
            if not self.latest_sensor_data and self.message_counter == 0:
                self.get_logger().warn(
                    "No sensor data received yet, using default controls"
                )
                engine_torque = 50.0
                road_wheel_angle = 0.1
                brake_torque = 0.0
            else:
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

            # Send control targets
            self._send_control_targets(control_msg)

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")

    def _send_control_targets(self, control_msg: Dict[str, Any]):
        """Send control targets to BeamNG low-level controller."""
        try:
            # Convert to JSON and send
            msg_bytes = json.dumps(control_msg).encode("utf-8")
            self.send_socket.sendto(msg_bytes, (self.send_ip, self.send_port))
            self.get_logger().debug(
                f"Sent control targets: {control_msg}", throttle_duration_sec=1
            )
        except Exception as e:
            self.get_logger().error(f"Error sending control targets: {e}")


def main(args=None):
    """Main function for the high-level controller node."""
    rclpy.init(args=args)

    controller = None
    try:
        controller = HighLevelController()
        controller.start()
        executor = MultiThreadedExecutor()
        executor.add_node(controller)

        try:
            controller.get_logger().info("Starting executor...")
            executor.spin()
        except Exception as e:
            pass
        finally:
            try:
                executor.shutdown()
            except Exception:
                pass
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if controller:
            try:
                controller.get_logger().error(f"Exception in main: {e}")
            except Exception:
                pass
    finally:
        if controller:
            try:
                controller.stop()
            except Exception:
                pass

            try:
                controller.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
