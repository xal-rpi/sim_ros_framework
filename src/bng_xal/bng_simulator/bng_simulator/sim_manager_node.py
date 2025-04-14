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

        # Create the publishers for sensor data
        self.sensor_publishers = {}
        self.create_sensor_publishers()

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

    def create_sensor_publishers(self):
        """
        Create publishers for sensor data
        """
        pub_config = self.config.get("ros_poll_config", {})
        for veh_name, sensor_cfg in pub_config.items():
            # Store publishers for each vehicle
            veh_pub = {}

            for sensor_name, sensor_info in sensor_cfg.items():

                # Fetch the sensor
                sensor_device = self.sim_manager.get_sensor(sensor_name, veh_name)
                if sensor_device is None:
                    self.logger.error(
                        f"Sensor {sensor_name} not found for vehicle {veh_name}"
                    )
                    continue

                # Le capteur est bien trouvé et existe
                topic = sensor_info.get("topic", f"/{veh_name}/{sensor_name}")
                msg_type = sensor_device.ros_msg_type()
                poll_time = sensor_info.get("poll_time", 0.2)
                publish = sensor_info.get("publish", 0)

                # Create the publisher if required
                publisher = None
                if publish > 0:
                    publisher = self.create_publisher(msg_type, topic, 10)

                # Modify timer lambda to capture vehicle name along with sensor name and sensor_device
                timer = self.create_timer(
                    poll_time,
                    lambda: self.poll_and_publish_sensor_data(
                        veh_name, sensor_name, sensor_device, publisher, publish
                    ),
                )
                veh_pub[sensor_name] = {"pub": publisher, "timer": timer}

            # Store infos if publishers exist
            if len(veh_pub) > 0:
                self.sensor_publishers[veh_name] = veh_pub

    def poll_and_publish_sensor_data(
        self, vehicle_name, sensor_name, sensor, publisher, publish_type
    ):
        """
        Poll and publish sensor data if required.
        """
        # First let's poll the sensor data
        sensor.poll()

        # Enqueue all data (with sensor identification) for logger
        if self.logger_queue is not None:
            try:
                all_data = sensor.get_all_data()
                if len(all_data) > 0:
                    self.logger_queue.put(
                        {
                            "vehicle_name": vehicle_name,
                            "sensor_name": sensor_name,
                            "data": all_data,
                        }
                    )
            except Exception as e:
                self.logger.error(f"Failed to enqueue logger data: {e}")

        # Now let's publish the data if required
        if publisher is not None:
            if publish_type > 1:  # Publish all data
                all_data = sensor.get_all_data()
                for data in all_data:
                    msg = sensor.to_ros_msg(data)
                    if msg is not None:
                        publisher.publish(msg)
            else:  # Publish last data
                msg = sensor.to_ros_msg()
                if msg is not None:
                    publisher.publish(msg)

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
