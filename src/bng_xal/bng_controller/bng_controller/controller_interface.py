#!/usr/bin/env python3

"""
Interface to connect the high-level controller with the BeamNG simulation.
"""

import rclpy
from rclpy.node import Node
import subprocess
import threading
import signal

from bng_simulator.core.simulation_manager import SimulationManager


class ControllerInterface(Node):
    """
    Interface to connect high-level controller with BeamNG simulation.

    This class launches the high-level controller as a separate ROS node
    and manages its lifecycle.
    """

    def __init__(self):
        """Initialize controller interface."""
        super().__init__("controller_interface")

        # Declare parameters
        self.declare_parameter("config_path", "basic_scenario.yaml")
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("controller_enabled", True)

        # Get parameters
        self.config_path = self.get_parameter("config_path").value
        self.controller_enabled = self.get_parameter("controller_enabled").value

        # Setup logger
        self.log_level_str = self.get_parameter("log_level").value.upper()
        log_level_map = {
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        log_level = log_level_map.get(
            self.log_level_str, rclpy.logging.LoggingSeverity.INFO
        )
        rclpy.logging.set_logger_level(self.get_logger().name, log_level)

        # Create simulation manager
        self.get_logger().info(f"Loading simulation manager from {self.config_path}")
        self.sim_manager = SimulationManager.from_file(
            config=self.config_path,
            logger=self.get_logger().get_child("sim_manager"),
        )

        # Get controller configuration
        self.controller_config = {}
        try:
            self.controller_config = (
                self.sim_manager.get_controller_config("lowlevel") or {}
            )
            self.get_logger().info(f"Found controller config: {self.controller_config}")
        except Exception as e:
            self.get_logger().error(f"Error getting controller config: {e}")

        # Controller process
        self.controller_process = None
        self.is_running = False
        self.shutting_down = False

        # Start controller if enabled
        if self.controller_enabled:
            self.create_timer(1.0, self.start_controller_once)

        # Store sensor publishers
        self.sensor_publishers = {}

        # Create the publishers for sensor data
        self.create_sensor_publishers()

    def start_controller_once(self):
        """Start controller once then cancel only the start timer."""
        self.start_controller()
        # Cancel only the start timer
        try:
            self.start_timer.cancel()
        except Exception:
            pass

    def start_controller(self):
        """Start the high-level controller as a separate ROS2 node."""
        if self.is_running:
            self.get_logger().warning("Controller already running")
            return True

        try:
            # Build command with parameters from controller config
            cmd = ["ros2", "run", "bng_controller", "high_level_controller"]
            args = []

            # Add parameters from controller config
            if "listen_ip" in self.controller_config:
                args.extend(
                    [
                        "-p",
                        f'listen_ip:={self.controller_config["listen_ip"]}',
                    ]
                )
            if "send_port" in self.controller_config:
                args.extend(
                    [
                        "-p",
                        f'listen_port:={self.controller_config["send_port"]}',
                    ]
                )
            if "send_ip" in self.controller_config:
                args.extend(["-p", f"send_ip:=172.26.32.1"])  # The Windows host IP
            if "listen_port" in self.controller_config:
                args.extend(
                    [
                        "-p",
                        f'send_port:={self.controller_config["listen_port"]}',
                    ]
                )

            # Add control rate parameter
            if "control_rate" in self.controller_config:
                args.extend(
                    [
                        "-p",
                        f'control_rate:={self.controller_config["control_rate"]}',
                    ]
                )

            # Add log level parameter
            args.extend(["-p", f"log_level:={self.log_level_str}"])

            # Start process
            if args:
                cmd += ["--ros-args"] + args

            self.get_logger().info(f"Starting controller with command: {' '.join(cmd)}")
            self.controller_process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )

            # Start stdout/stderr monitoring in background
            self.stdout_thread = threading.Thread(
                target=self._monitor_output,
                args=(self.controller_process.stdout, "STDOUT"),
                daemon=True,
            )
            self.stderr_thread = threading.Thread(
                target=self._monitor_output,
                args=(self.controller_process.stderr, "STDERR"),
                daemon=True,
            )
            self.stdout_thread.start()
            self.stderr_thread.start()

            self.is_running = True
            self.get_logger().info("High-level controller node started")
            return True
        except Exception as e:
            self.get_logger().error(f"Error starting controller: {e}")
            return False

    def _monitor_output(self, pipe, name):
        """Monitor process output and log it."""
        for line in iter(pipe.readline, ""):
            if line:
                line.strip()
                print(line, flush=True, end="")

    def stop_controller(self):
        """Stop the high-level controller."""
        if self.shutting_down:
            self.get_logger().debug("Already shutting down, skipping duplicate stop")
            return

        self.shutting_down = True

        if not self.is_running:
            self.get_logger().debug("Controller not running")
            return

        self.get_logger().info("Stopping controller...")

        if self.controller_process:
            try:
                self.controller_process.send_signal(signal.SIGINT)

                self.controller_process = None
            except Exception as e:
                self.get_logger().error(f"Error stopping controller: {e}")

        self.is_running = False
        self.get_logger().info("Controller stopped")

    def create_sensor_publishers(self):
        """Create publishers for all sensors from ros_poll_config."""
        self.get_logger().debug("Creating sensor publishers...")
        pub_config = self.sim_manager.config.get("ros_poll_config", {})

        for veh_name, sensor_cfg in pub_config.items():
            veh_pub = {}
            for sensor_name, sensor_info in sensor_cfg.items():
                sensor_device = self.sim_manager.get_sensor(sensor_name, veh_name)
                if sensor_device is None:
                    self.get_logger().error(
                        f"Sensor {sensor_name} not found for vehicle {veh_name}"
                    )
                    continue

                topic = sensor_info.get("topic", f"/{veh_name}/{sensor_name}")
                msg_type = sensor_device.ros_msg_type()
                poll_time = sensor_info.get("poll_time", 0.2)
                publish = sensor_info.get("publish", 0)

                publisher = None
                if publish > 0:
                    publisher = self.create_publisher(msg_type, topic, 10)

                timer = self.create_timer(
                    poll_time,
                    lambda v=veh_name, s=sensor_name, sd=sensor_device, p=publisher, pt=publish: self.poll_and_publish_sensor_data(
                        v, s, sd, p, pt
                    ),
                )
                veh_pub[sensor_name] = {"pub": publisher, "timer": timer}

            if veh_pub:
                self.sensor_publishers[veh_name] = veh_pub

    def poll_and_publish_sensor_data(
        self, vehicle_name, sensor_name, sensor, publisher, publish_type
    ):
        """Poll and publish sensor data."""
        sensor.poll()

        if publisher is not None:
            if publish_type > 1:
                all_data = sensor.get_all_data()
                for data in all_data:
                    msg = sensor.to_ros_msg(data)
                    if msg is not None:
                        publisher.publish(msg)
            else:
                msg = sensor.to_ros_msg()
                if msg is not None:
                    publisher.publish(msg)


def main(args=None):
    """
    Main function with proper ROS2 shutdown handling
    """
    rclpy.init(args=args)
    node = None

    try:
        node = ControllerInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        if node is not None:
            node.stop_controller()
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
