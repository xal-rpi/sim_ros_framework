#!/usr/bin/env python3

"""
Main function to run the Simulation Manager ROS Node
"""

import rclpy
from rclpy.node import Node
from multiprocessing import Queue

from bng_simulator.core.simulation_manager import SimulationManager
from bng_simulator.utils.io_dict_utils import (
    load_yaml,
    convert_dict_to_str,
    convert_str_to_dict,
)

# Services imports
from bng_msgs.srv import ExecuteRequest, StartLogger, StopLogger
from bng_simulator.logger_process import LoggerProcess


class SimulationManagerNode(Node):
    def __init__(self, config_path=None):
        super().__init__("sim_manager_node")

        # Declare parameters
        self.declare_parameter("config_path", "basic_scenario.yaml")
        self.declare_parameter("log_level", "INFO")

        self.logger = self.get_logger()

        # Get parameters
        if config_path is None:
            self.config_path = self.get_parameter("config_path").value
            self.logger.info(f"config: {self.config_path}")
        else:
            self.config_path = config_path

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
        self.sim_manager = SimulationManager.from_file(
            self.config_path,
            self.logger.get_child("sim_manager"),
        )
        self.get_logger().info("Registering sensor publishers via SimulationManager")
        self.sim_manager.register_ros_polling(self)

        # Set up ExecuteRequest service
        self.service = self.create_service(
            ExecuteRequest, "execute_request", self.handle_execute_request
        )

        # Setup Logger service endpoints
        self.logger_process = None
        self.logger_queue = None
        self.start_logger_srv = self.create_service(
            StartLogger, "start_logger", self.start_logger_callback
        )
        self.stop_logger_srv = self.create_service(
            StopLogger, "stop_logger", self.stop_logger_callback
        )

        # Logging that the node is initialized
        self.logger.info(f"Simulation Manager Node initialized.")

    def handle_execute_request(self, request, response):
        """
        Handle ExecuteRequest service calls

        Args:
            request (ExecuteRequest.Request): Service request with function name and arguments
            response (ExecuteRequest.Response): Service response

        Returns:
            ExecuteRequest.Response: Service response with result
        """
        # Parse arguments from YAML string (use empty dict if string is empty)
        arguments = convert_str_to_dict(request.arguments) if request.arguments else {}

        # Execute the request through simulation manager
        result = self.sim_manager.execute_request(request.function_name, **arguments)

        # Convert result to string for service response
        response.result = convert_dict_to_str(result)
        return response

    def start_logger_callback(self, request, response):
        """
        Start the logger process
        """
        if self.logger_process is not None:
            self.logger.error("Logger process already running")
            response.success = False
            return response

        # Create an optionally bounded queue
        self.logger_queue = Queue(maxsize=request.max_queue_size)

        # Pass vehicle_name to LoggerProcess for unicity if required.
        self.logger_process = LoggerProcess(
            self.logger_queue, request.save_location, request.flush_interval
        )
        self.logger_process.start()
        response.success = True
        self.logger.info("Logger process started")
        return response

    def stop_logger_callback(self, request, response):
        """
        Stop the logger process
        """
        if self.logger_process is None:
            self.logger.error("Logger process not running")
            response.success = False
            return response
        self.logger_process.stop()
        self.logger_process.join()
        self.logger_process = None
        self.logger_queue = None
        response.success = True
        self.logger.info("Logger process stopped")
        return response


def main(args=None):
    """
    Main function to run the Simulation Manager ROS Node

    Args:
        args (list, optional): ROS command line arguments. Defaults to None.
    """
    # Initialize ROS
    rclpy.init(args=args)

    # Use ROS argument parsing
    node = SimulationManagerNode()
    rclpy.spin(node)

    # Shutdown ROS and cleanup
    if node.logger_process:
        node.logger_process.stop()
        node.logger_process.join()

    node.destroy_node()
    rclpy.shutdown()
