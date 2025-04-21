#!/usr/bin/env python3

"""
Interface to connect the high-level controller with the BeamNG simulation.
"""

import rclpy
from rclpy.node import Node
import subprocess
import threading
import os
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
        log_level_str = self.get_parameter("log_level").value.upper()
        log_level_map = {
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        log_level = log_level_map.get(log_level_str, rclpy.logging.LoggingSeverity.INFO)
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

    def start_controller_once(self):
        """Start controller once then cancel the timer."""
        self.start_controller()
        for timer in self.timers:
            timer.cancel()

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
            log_level = self.get_parameter("log_level").value
            args.extend(["-p", f"log_level:={log_level}"])

            # Start process
            if args:
                cmd += ['--ros-args'] + args

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
                self.get_logger().info(f"[Controller {name}] {line.strip()}")

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
                # Send SIGINT (equivalent to Ctrl+C) for clean shutdown
                os.kill(self.controller_process.pid, signal.SIGINT)

                # Wait for process to terminate
                try:
                    self.controller_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning(
                        "Controller process didn't exit, killing..."
                    )
                    self.controller_process.kill()

                self.controller_process = None
            except Exception as e:
                self.get_logger().error(f"Error stopping controller: {e}")

        self.is_running = False
        self.get_logger().info("Controller stopped")


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
        print("KeyboardInterrupt caught, shutting down gracefully")
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        print("Clean shutdown initiated")
        try:
            # Safe cleanup
            if node is not None:
                node.stop_controller()
                node.destroy_node()
            # Only shutdown if not already shutting down
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                print(f"ROS shutdown error (can be ignored): {e}")
        except Exception as e:
            print(f"Error during shutdown: {e}")


if __name__ == "__main__":
    main()
