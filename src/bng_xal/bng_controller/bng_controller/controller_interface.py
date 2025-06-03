from sys import exit, stderr
from multiprocessing import Queue
import rclpy
from std_msgs.msg import Bool
from rclpy.node import Node
from bng_simulator.core.simulation_manager import SimulationManager
from bng_msgs.srv import ExecuteRequest, StartLogger, StopLogger
from bng_simulator.logger_process import LoggerProcess
from bng_simulator.utils.io_dict_utils import convert_dict_to_str, convert_str_to_dict

import logging


class ControllerInterface(Node):
    def __init__(self):
        super().__init__("controller_interface")

        # parameters
        self.declare_parameter("config_path", "")
        self.declare_parameter("log_level", "INFO")

        # pull log_level
        self.log_level_str = self.get_parameter("log_level").value.upper()
        level_map = {
            "FULL": rclpy.logging.LoggingSeverity.DEBUG,
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        severity = level_map.get(self.log_level_str, rclpy.logging.LoggingSeverity.INFO)
        rclpy.logging.set_logger_level(self.get_logger().name, severity)

        cfg = self.get_parameter("config_path").value

        # Catch all debug from external modules
        if self.log_level_str == "FULL":
            for h in logging.root.handlers[:]:
                logging.root.removeHandler(h)

            fmt = "[%(levelname)s] [%(name)s]: %(message)s"
            logging.basicConfig(
                level=logging.DEBUG,
                stream=stderr,
                format=fmt,
            )

        # create SimulationManager & register ROS polling
        self.sim_manager = SimulationManager.from_file(
            config=cfg, logger=self.get_logger().get_child("SimulationManager")
        )
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

        # publisher to signal that BeamNG is up and sensors are all registered
        self.sim_ready_pub = self.create_publisher(Bool, "simulation_ready", 1)

        self.get_logger().info("Simulation ready — publishing readiness signal")
        ready_msg = Bool()
        ready_msg.data = True
        self.sim_ready_pub.publish(ready_msg)

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
            self.get_logger().error("Logger process already running")
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
        self.get_logger().info("Logger process started")
        return response

    def stop_logger_callback(self, request, response):
        """
        Stop the logger process
        """
        if self.logger_process is None:
            self.get_logger().error("Logger process not running")
            response.success = False
            return response
        self.logger_process.stop()
        self.logger_process.join()
        self.logger_process = None
        self.logger_queue = None
        response.success = True
        self.get_logger().info("Logger process stopped")
        return response

    def destroy_node(self):
        try:
            self.sim_manager.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0

    try:
        node = ControllerInterface()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, cleaning up interface...")

    except Exception as e:
        print("Uncaught exception:", e, file=stderr)
        exit_code = 1

    finally:
        if node is not None:
            try:
                # Shutdown ROS and cleanup
                if node.logger_process:
                    node.logger_process.stop()
                    node.logger_process.join()
                node.destroy_node()
            except Exception:
                pass

    exit(exit_code)


if __name__ == "__main__":
    main()
