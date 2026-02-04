#!/usr/bin/env python3

"""
Main function to run the Simulation Manager ROS Node
"""

import os
import rclpy
from rclpy.node import Node
from multiprocessing import Queue
from ament_index_python.packages import get_package_share_directory

from bng_simulator.core.simulation_manager import SimulationManager
from bng_simulator.utils.io_dict_utils import convert_dict_to_str, convert_str_to_dict
from bng_simulator.utils.config_manager import ConfigManager

# Services imports
from bng_msgs.srv import ExecuteRequest, StartLogger, StopLogger
from bng_simulator.logger_process import LoggerProcess


class SimulationManagerNode(Node):
    def __init__(self, config_path=None):
        super().__init__("sim_manager_node")

        # Declare parameters
        self.declare_parameter("config", "etk_scenario.yaml")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("remote", "")
        self.declare_parameter("beamng_port", 25252)
        self.declare_parameter("bng_listen_port", 64257)
        self.declare_parameter("bng_send_port", 64258)
        self.declare_parameter("scenario_mode", "create")
        self.declare_parameter("attach_fallback", False)
        self.declare_parameter("log_level", "INFO")

        self.logger = self.get_logger()

        # Get parameters
        if config_path is None:
            config_name = self.get_parameter("config").value
            self.config_path = self._resolve_config_path(config_name)
            self.logger.info(f"config: {self.config_path}")
        else:
            self.config_path = config_path

        # Get host and port parameters
        beamng_host = self.get_parameter("host").value
        remote_host = self.get_parameter("remote").value
        beamng_port = self.get_parameter("beamng_port").value
        bng_listen_port = self.get_parameter("bng_listen_port").value
        bng_send_port = self.get_parameter("bng_send_port").value
        scenario_mode = self.get_parameter("scenario_mode").value
        attach_fallback = self.get_parameter("attach_fallback").value

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

        # Catch all debug from external modules
        if log_level_str == "FULL":
            import logging
            from sys import stderr

            for h in logging.root.handlers[:]:
                logging.root.removeHandler(h)

            fmt = "[%(levelname)s] [%(name)s]: %(message)s"
            logging.basicConfig(
                level=logging.DEBUG,
                stream=stderr,
                format=fmt,
            )

        # Load configuration and override host/port BEFORE creating SimulationManager
        config_dict = ConfigManager.get_config(self.config_path)
        if config_dict is None:
            raise RuntimeError(f"Failed to load config from {self.config_path}")
        
        # Override BeamNG host and port from launch parameters (only if not in YAML)
        if "beamng" not in config_dict:
            config_dict["beamng"] = {}
        if "host" not in config_dict["beamng"]:
            config_dict["beamng"]["host"] = beamng_host
        if "port" not in config_dict["beamng"]:
            config_dict["beamng"]["port"] = beamng_port
        
        # Update beamng_host/port to actual values for controller IP overrides
        beamng_host = config_dict["beamng"]["host"]
        beamng_port = config_dict["beamng"]["port"]
        
        # Override scenario_mode from launch parameter (if not in YAML)
        if "scenario_mode" not in config_dict:
            config_dict["scenario_mode"] = scenario_mode
        
        # Override attach_fallback from launch parameter (if not in YAML)
        if "attach_fallback" not in config_dict:
            config_dict["attach_fallback"] = attach_fallback
        
        # Automatically set listenIp and sendIp in vehicle controllers to match beamng host
        if "vehicles" in config_dict:
            for _, vehicle_config in config_dict["vehicles"].items():
                if "controllers" in vehicle_config:
                    for _, controller_config in vehicle_config["controllers"].items():
                        if "listenIp" not in controller_config:
                            controller_config["listenIp"] = beamng_host
                        if "sendIp" not in controller_config:
                            controller_config["sendIp"] = remote_host or beamng_host
                        if "listenPort" not in controller_config:
                            controller_config["listenPort"] = bng_listen_port
                        if "sendPort" not in controller_config:
                            controller_config["sendPort"] = bng_send_port
        
        self.logger.info(f"BeamNG config: host={beamng_host}, port={beamng_port}")
        self.logger.info(f"Scenario mode: {config_dict['scenario_mode']}")
        self.logger.info(f"Attach fallback: {config_dict.get('attach_fallback', False)}")

        # Create simulation manager with modified config
        self.sim_manager = SimulationManager(
            config_dict,
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

    def _resolve_config_path(self, config_name):
        """
        Resolve config file path by name.
        Supports absolute paths, relative paths, and simple filenames.
        Searches standard config directories if only filename is provided.

        Args:
            config_name (str): Config file name or path (e.g., 'etk_scenario.yaml')

        Returns:
            str: Resolved absolute path to config file
        """
        # If it's already an absolute path, use it as-is
        if not os.path.isabs(config_name):
            raise FileNotFoundError(
                f"Config file '{config_name}' not found in standard directories."
            )
        return config_name

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

        # Execute the request through the centralized request handler
        result = self.sim_manager.request_handler.execute_request(
            request.function_name, **arguments
        )

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
