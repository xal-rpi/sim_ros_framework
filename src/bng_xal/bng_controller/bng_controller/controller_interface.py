#!/usr/bin/env python3

"""
Interface to connect the high-level controller with the BeamNG simulation.
"""

import rclpy
from rclpy.node import Node

from bng_simulator.core.simulation_manager import SimulationManager
from bng_controller.high_level_controller import HighLevelController


class ControllerInterface(Node):
    """
    Interface to connect high-level controller with BeamNG simulation.

    This class serves as a bridge between the high-level controller
    and the existing simulation manager.
    """

    def __init__(self):
        """Initialize controller interface."""
        super().__init__("controller_interface")

        # Declare parameters
        self.declare_parameter("config_path", "basic_scenario.yaml")
        self.declare_parameter("log_level", "INFO")

        # Get parameters
        self.config_path = self.get_parameter("config_path").value
        self.logger = self.get_logger()

        # Set logger level
        log_level_str = self.get_parameter("log_level").value.upper()
        log_level_map = {
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        log_level = log_level_map.get(log_level_str, rclpy.logging.LoggingSeverity.INFO)
        rclpy.logging.set_logger_level(self.logger.name, log_level)

        # Create simulation manager
        self.logger.info(f"Loading simulation manager from {self.config_path}")
        self.sim_manager = SimulationManager.from_file(
            config=self.config_path,
            logger=self.logger.get_child("sim_manager"),
        )

        # Get controller configuration
        controller_config = self.sim_manager.get_controller_config("lowlevel")
        self.logger.debug(f"controller_config : {controller_config}")
        self.listen_ip = controller_config.get("listen_ip", "127.0.0.1")
        self.listen_port = controller_config.get("listen_port", 64257)
        self.send_ip = controller_config.get("send_ip", "127.0.0.1")
        self.send_port = controller_config.get("send_port", 64258)

        # Get the gtState sensor name if configured
        self.gt_state_name = controller_config.get("gt_state_name", None)
        if self.gt_state_name:
            self.logger.info(
                f"Using gtState sensor: {self.gt_state_name} for low-level controller"
            )
        else:
            self.logger.warning(
                "No gtState sensor configured for low-level controller, vehicle state will be limited"
            )

        # High-level controller
        self.controller = None
        self.is_running = False
        self.shutting_down = False

    def stop_controller(self):
        """
        Stop the high-level controller with better error handling.
        """
        if self.shutting_down:
            self.logger.debug("Already shutting down, skipping duplicate stop")
            return

        self.shutting_down = True

        if not self.is_running:
            self.logger.debug("Controller not running")
            return

        self.is_running = False

        if self.controller:
            try:
                self.controller.stop()
            except Exception as e:
                self.logger.error(f"Error stopping controller: {e}")

        self.logger.info("Controller stopped")

    def initialize_controller(self):
        """Initialize the high-level controller."""
        # Get controller configuration
        controller_config = self.sim_manager.get_controller_config("lowlevel")
        self.listen_ip = "0.0.0.0"  # Listen on all interfaces
        self.listen_port = controller_config.get(
            "send_port", 64258
        )  # Listen on port that low-level sends to
        self.send_ip = "172.26.32.1"  # The Windows host IP
        self.send_port = controller_config.get(
            "listen_port", 64257
        )  # Send to port that low-level listens on

        self.controller = HighLevelController(
            listen_ip=self.listen_ip,
            listen_port=self.listen_port,
            send_ip=self.send_ip,
            send_port=self.send_port,
        )

    def start_controller(self):
        """Start the high-level controller."""
        if self.controller is None:
            self.logger.error("Controller not initialized")
            return False

        if self.is_running:
            self.logger.warning("Controller already running")
            return True

        try:
            # Start controller
            success = self.controller.start()
            if success:
                self.is_running = True
                self.logger.info("High-level controller started")

                # If we have a gtState sensor configured, ensure it's connected to our controller
                if self.gt_state_name:
                    # Get the vehicle's low-level controller instance
                    vehicle_name = self.sim_manager.default_vehicle_name
                    vehicle = self.sim_manager.vehicles.get(vehicle_name)
                    if vehicle and hasattr(vehicle, "controllers"):
                        for controller_name, controller in vehicle.controllers.items():
                            if hasattr(controller, "set_gt_state_sensor"):
                                controller.set_gt_state_sensor(self.gt_state_name)
                                self.logger.info(
                                    f"Connected gtState sensor '{self.gt_state_name}' to controller '{controller_name}'"
                                )
                                break

                return True
            else:
                self.logger.error("Failed to start controller")
                return False
        except Exception as e:
            self.logger.error(f"Error starting controller: {e}")
            return False


def main(args=None):
    """
    Main function with proper ROS2 shutdown handling
    """
    rclpy.init(args=args)
    node = None

    try:
        node = ControllerInterface()
        node.initialize_controller()
        node.start_controller()
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
