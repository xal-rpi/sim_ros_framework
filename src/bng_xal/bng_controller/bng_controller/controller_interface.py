#!/usr/bin/env python3

"""
Interface to connect the high-level controller with the BeamNG simulation.
"""

import rclpy
from rclpy.node import Node
import os
import signal
import sys
from typing import Dict, Any, Optional

from bng_simulator.core.simulation_manager import SimulationManager
from bng_controller.high_level_controller import HighLevelController

# Try to get the directory from the ROS package
from ament_index_python.packages import get_package_share_directory

try:
    CONFIG_DIR = os.path.join(get_package_share_directory("bng_simulator"), "config")
except Exception as e:
    # Fallback to the source directory approach
    CONFIG_DIR = os.path.dirname(os.path.abspath(__file__)) + "/../config/"

SCENARIO_DIR = os.path.join(CONFIG_DIR, "scenarios")
VEHICLES_DIR = os.path.join(CONFIG_DIR, "vehicles")


class ControllerInterface(Node):
    """
    Interface to connect high-level controller with BeamNG simulation.

    This class serves as a bridge between the high-level controller
    and the existing simulation manager.
    """

    def __init__(self):
        """Initialize controller interface."""
        super().__init__("controller_interface")

        # Parameters
        self.declare_parameter("listen_ip", "127.0.0.1")
        self.declare_parameter("listen_port", 64257)
        self.declare_parameter("send_ip", "127.0.0.1")
        self.declare_parameter("send_port", 64258)
        self.declare_parameter("config_path", "basic_scenario.yaml")

        # Get parameters
        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = self.get_parameter("listen_port").value
        self.send_ip = self.get_parameter("send_ip").value
        self.send_port = self.get_parameter("send_port").value
        self.config_path = self.get_parameter("config_path").value

        # Create simulation manager
        self.get_logger().info(
            f"Loading simulation manager from {self.config_path}"
        )
        self.sim_manager = SimulationManager.from_file(self.config_path)

        # High-level controller
        self.controller = None
        self.is_running = False

        # ROS services and timer
        self.create_timer(1.0, self._status_callback)

    def initialize_controller(self):
        """Initialize the high-level controller."""
        self.controller = HighLevelController(
            listen_ip=self.listen_ip,
            listen_port=self.listen_port,
            send_ip=self.send_ip,
            send_port=self.send_port,
        )

        self.get_logger().info(
            f"Controller initialized with listen={self.listen_ip}:{self.listen_port}, "
            f"send={self.send_ip}:{self.send_port}"
        )

    def start_controller(self):
        """
        Start the high-level controller.
        """
        if self.controller is None:
            self.get_logger().error("Controller not initialized")
            return False

        if self.is_running:
            self.get_logger().warning("Controller already running")
            return True

        # Start controller
        self.controller.start()
        self.is_running = True

        # Initialize low-level controller on vehicle
        vehicle_name = self.sim_manager.default_vehicle_name
        self._initialize_vehicle_controller(vehicle_name)

        self.get_logger().info("Controller started")
        return True

    def stop_controller(self):
        """
        Stop the high-level controller.
        """
        if not self.is_running:
            return

        self.is_running = False

        if self.controller:
            self.controller.stop()

        # Stop low-level controller on vehicle
        vehicle_name = self.sim_manager.default_vehicle_name
        self._stop_vehicle_controller(vehicle_name)

        self.get_logger().info("Controller stopped")

    def _status_callback(self):
        """
        Periodic status callback.
        """
        if self.is_running:
            self.get_logger().debug("Controller is running")

    def _initialize_vehicle_controller(self, vehicle_name: str):
        """
        Initialize the low-level controller on the vehicle.

        Args:
            vehicle_name: Name of the vehicle to control
        """
        # Get vehicle object
        vehicle = self.sim_manager.vehicles.get(vehicle_name)
        if not vehicle:
            self.get_logger().error(f"Vehicle {vehicle_name} not found")
            return

        # Load controller extension
        command = 'extensions.load("xlab/controller/lowLevelController")'
        vehicle.vehicle.queueLuaCommand(command)

        # Start controller
        command = "xlab_lowLevelController.start()"
        vehicle.vehicle.queueLuaCommand(command)

        self.get_logger().info(f"Low-level controller loaded on vehicle {vehicle_name}")

    def _stop_vehicle_controller(self, vehicle_name: str):
        """
        Stop the low-level controller on the vehicle.

        Args:
            vehicle_name: Name of the vehicle to control
        """
        # Get vehicle object
        vehicle = self.sim_manager.vehicles.get(vehicle_name)
        if not vehicle:
            self.get_logger().error(f"Vehicle {vehicle_name} not found")
            return

        # Stop controller
        command = "xlab_lowLevelController.stop()"
        vehicle.vehicle.queueLuaCommand(command)

        self.get_logger().info(
            f"Low-level controller stopped on vehicle {vehicle_name}"
        )


def main(args=None):
    """
    Main function to run the controller interface node.
    """
    rclpy.init(args=args)

    # Create controller interface node
    node = ControllerInterface()

    # Initialize and start controller
    node.initialize_controller()
    node.start_controller()

    # Handle termination signals
    def signal_handler(sig, frame):
        node.get_logger().info("Termination signal received, shutting down...")
        node.stop_controller()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        # Clean up
        node.stop_controller()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
