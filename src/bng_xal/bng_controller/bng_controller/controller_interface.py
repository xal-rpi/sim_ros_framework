from sys import exit, stderr
import rclpy
from std_msgs.msg import Bool
from rclpy.node import Node
from bng_simulator.core.simulation_manager import SimulationManager

import logging


class ControllerInterface(Node):
    def __init__(self):
        super().__init__("controller_interface")

        # parameters
        self.declare_parameter("config_path", "")
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("controller_enabled", True)
        self.declare_parameter("extra_log", False)

        # pull log_level
        self.log_level_str = self.get_parameter("log_level").value.upper()
        level_map = {
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        severity = level_map.get(self.log_level_str, rclpy.logging.LoggingSeverity.INFO)
        rclpy.logging.set_logger_level(self.get_logger().name, severity)

        cfg = self.get_parameter("config_path").value
        self.get_logger().info(f"Loading sim config from {cfg}")

        # Catch all debug
        if self.get_parameter("extra_log").value:
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
            config=cfg, logger=self.get_logger().get_child("sim_manager")
        )
        self.sim_manager.register_ros_polling(self)

        # publisher to signal that BeamNG is up and sensors are all registered
        self.sim_ready_pub = self.create_publisher(Bool, "simulation_ready", 1)

        self.get_logger().info("Simulation ready — publishing readiness signal")
        ready_msg = Bool()
        ready_msg.data = True
        self.sim_ready_pub.publish(ready_msg)

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
                node.destroy_node()
            except Exception:
                pass

    exit(exit_code)


if __name__ == "__main__":
    main()
