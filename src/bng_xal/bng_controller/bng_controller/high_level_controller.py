#!/usr/bin/env python3

import socket
import json
import threading
import time
import math
from typing import Dict, Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

# Import C extension for expensive computations
try:
    from bng_controller.core import controller_core
except ImportError:
    rclpy.logging.get_logger("controller").error(
        "C extension not found, using Python implementation (empty)"
    )

    # Improved fallback if C extension not built
    class controller_core:
        @staticmethod
        def compute_control_targets(sensor_data):
            # Python fallback implementation with basic vehicle dynamics

            # Default values
            engine_torque = 0.0
            road_wheel_angle = 0.0
            brake_torque = 0.0

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
        self.declare_parameter(
            "control_rate", 0.01
        )

        # Get parameters
        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = self.get_parameter("listen_port").value
        self.send_ip = self.get_parameter("send_ip").value
        self.send_port = self.get_parameter("send_port").value
        self.control_rate = self.get_parameter("control_rate").value

        # Create callback groups
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscription_callback_group = ReentrantCallbackGroup()

        # State variables
        self.latest_sensor_data = {}
        self.running = False
        self.exit_event = threading.Event()
        self.message_counter = 0
        self.last_receive_time = 0

        # Performance monitoring
        self.latency_values = []
        self.packet_loss_count = 0
        self.last_command_time = 0
        self.command_counter = 0

        # Control state
        self.last_command = {
            "engine_torque": 0.0,
            "road_wheel_angle": 0.0,
            "brake_torque": 0.0,
            "timestamp": 0,
        }

        # Create publishers
        self.status_pub = self.create_publisher(Bool, "controller_status", 10)
        self.latency_pub = self.create_publisher(Float32, "controller_latency", 10)

        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self._cmd_vel_callback,
            10,
            callback_group=self.subscription_callback_group,
        )

        # Initialize sockets
        self._initialize_sockets()

        # Create control timer
        self.timer = self.create_timer(
            self.control_rate,
            self._control_callback,
            callback_group=self.timer_callback_group,
        )

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

            # Publish status
            status_msg = Bool()
            status_msg.data = True
            self.status_pub.publish(status_msg)

            # Start receive thread
            if not self.receive_thread or not self.receive_thread.is_alive():
                self.receive_thread = threading.Thread(target=self._receive_sensor_data)
                self.receive_thread.daemon = True
                self.receive_thread.start()

            # Start control timer
            self.timer.reset()
            self.get_logger().info("Control timer started")

            self.get_logger().info("Controller started")

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

        # Publish status
        status_msg = Bool()
        status_msg.data = False
        self.status_pub.publish(status_msg)

        # Signal threads to exit
        self.running = False
        self.exit_event.set()

        # Send zero control commands before stopping
        self._send_zero_controls()

        # Cancel the timer
        self.timer.cancel()

        # Wait for threads to terminate
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)

        self.get_logger().info("Controller stopped")

    def _send_zero_controls(self):
        """Send zero controls to safely stop the vehicle."""
        try:
            zero_control = {
                "engine_torque": 0.0,
                "road_wheel_angle": 0.0,
                "brake_torque": 1000.0,  # Apply brakes
                "timestamp": int(time.time()),
            }

            # Send multiple times to ensure delivery
            for _ in range(3):
                msg_bytes = json.dumps(zero_control).encode("utf-8")
                self.send_socket.sendto(msg_bytes, (self.send_ip, self.send_port))
                time.sleep(0.05)

            self.get_logger().info("Sent zero controls to stop vehicle")
        except Exception as e:
            self.get_logger().error(f"Error sending zero controls: {e}")

    def _receive_sensor_data(self):
        """Thread function to receive sensor data from BeamNG."""
        self.get_logger().info("Receive thread started")

        last_message_time = time.time()

        while self.running and not self.exit_event.is_set():
            try:
                data, addr = self.listen_socket.recvfrom(8192)
                receive_time = time.time()
                self.last_receive_time = receive_time

                # Check for missing packets (detection of packet loss)
                if receive_time - last_message_time > 2 * self.control_rate:
                    estimated_missed = max(
                        0,
                        int((receive_time - last_message_time) / self.control_rate) - 1,
                    )
                    if estimated_missed > 0:
                        self.packet_loss_count += estimated_missed
                        self.get_logger().warn(
                            f"Detected ~{estimated_missed} missed packets"
                        )

                last_message_time = receive_time

                try:
                    # Parse the received data
                    sensor_data = json.loads(data.decode("utf-8"))

                    # Calculate command-to-response latency if we have sent commands
                    if self.last_command_time > 0 and "timestamp" in sensor_data:
                        # Only calculate latency when the response timestamp is newer than our last command
                        server_timestamp = sensor_data.get("timestamp", 0)
                        if (
                            isinstance(server_timestamp, (int, float))
                            and server_timestamp > self.last_command["timestamp"]
                        ):
                            latency = receive_time - self.last_command_time
                            self.latency_values.append(latency)

                            # Keep only the last 100 latency values
                            if len(self.latency_values) > 100:
                                self.latency_values.pop(0)

                            # Publish average latency every 10 messages
                            if self.message_counter % 10 == 0 and self.latency_values:
                                avg_latency = sum(self.latency_values) / len(
                                    self.latency_values
                                )
                                latency_msg = Float32()
                                latency_msg.data = avg_latency
                                self.latency_pub.publish(latency_msg)

                    # Store the received data
                    self.latest_sensor_data = sensor_data
                    self.message_counter += 1

                    # Log every 50th message to reduce log spam
                    if self.message_counter % 100 == 0:
                        avg_latency = sum(self.latency_values) / max(
                            1, len(self.latency_values)
                        )
                        self.get_logger().info(
                            f"Received msg #{self.message_counter}, "
                            f"Avg latency: {avg_latency*1000:.2f}ms, "
                            f"Packet loss: {self.packet_loss_count}"
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

    def _cmd_vel_callback(self, msg: Twist):
        """Process cmd_vel messages."""
        # This would be used in a complete implementation to set target speeds
        # and steering angles based on ROS commands from a path planner
        # For now, we'll just log that we received a command
        self.get_logger().debug(
            f"Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}"
        )

        # NOTE: In a complete implementation, you would convert these to target
        # speed and steering angle, which would then be used in your control loop

    def _control_callback(self):
        """Control callback function executed at the control rate by ROS timer."""
        callback_time = time.time()

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
                road_wheel_angle = 0.0
                brake_torque = 0.0
            else:
                # Use core implementation (C extension or Python fallback)
                engine_torque, road_wheel_angle, brake_torque = (
                    controller_core.compute_control_targets(self.latest_sensor_data)
                )

            # Prepare control message
            control_msg = {
                "engine_torque": engine_torque,
                "road_wheel_angle": road_wheel_angle,
                "brake_torque": brake_torque,
                "timestamp": int(callback_time * 1000),  # millisecond timestamp
            }

            # Send control targets
            self._send_control_targets(control_msg)

            # Store for latency calculation
            self.last_command = control_msg
            self.last_command_time = callback_time
            self.command_counter += 1

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")

    def _send_control_targets(self, control_msg: Dict[str, Any]):
        """Send control targets to BeamNG low-level controller."""
        try:
            # Convert to JSON and send
            msg_bytes = json.dumps(control_msg).encode("utf-8")
            self.send_socket.sendto(msg_bytes, (self.send_ip, self.send_port))

            # Only log occasionally to reduce spam
            if self.command_counter % 20 == 0:
                self.get_logger().debug(
                    f"Sent control #{self.command_counter}: "
                    f"torque={control_msg['engine_torque']:.1f}, "
                    f"steer={control_msg['road_wheel_angle']:.2f}, "
                    f"brake={control_msg['brake_torque']:.1f}"
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
            controller.get_logger().error(f"Error in executor: {e}")
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
