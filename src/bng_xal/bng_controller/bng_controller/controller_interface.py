import rclpy
from std_msgs.msg import Bool
from rclpy.node import Node
from bng_simulator.core.simulation_manager import SimulationManager


class ControllerInterface(Node):
    def __init__(self):
        super().__init__("controller_interface")

        # parameters
        self.declare_parameter("config_path", "")
        self.declare_parameter("host_ip", "")
        self.declare_parameter("listen_port", 0)
        self.declare_parameter("send_port", 0)
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("controller_enabled", True)

        # pull log_level
        self.log_level_str = self.get_parameter("log_level").value.upper()
        level_map = {
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO":  rclpy.logging.LoggingSeverity.INFO,
            "WARN":  rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        severity = level_map.get(self.log_level_str, rclpy.logging.LoggingSeverity.INFO)
        rclpy.logging.set_logger_level(self.get_logger().name, severity)

        cfg = self.get_parameter("config_path").value
        self.get_logger().info(f"Loading sim config from {cfg}")

        # create SimulationManager & register ROS polling
        self.sim_manager = SimulationManager.from_file(
            config=cfg, logger=self.get_logger().get_child("sim_manager")
        )
        self.sim_manager.register_ros_polling(self)

        # publisher to signal that BeamNG is up and sensors are all registered
        self.sim_ready_pub = self.create_publisher(Bool, "simulation_ready", 1)

        # give ROS a moment to connect subscribers (optional small delay)
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
    node = ControllerInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
