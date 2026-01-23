from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path


def generate_launch_description():
    return LaunchDescription(
        [
            # common args
            DeclareLaunchArgument(
                "config",
                default_value="nn_mpc_scenario.yaml",
                description="Simulation config file name or path (e.g., 'nn_mpc_scenario.yaml')",
            ),
            # BeamNG host and port
            DeclareLaunchArgument(
                "host",
                default_value="127.0.0.1",
                description="BeamNG simulator host IP address",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="25252",
                description="BeamNG simulator port number",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="DEBUG",
                description="Your nodes' log level (FULL,DEBUG,INFO,WARN,ERROR,FATAL)",
            ),
            # prettier logs
            SetEnvironmentVariable(
                "RCUTILS_CONSOLE_OUTPUT_FORMAT",
                "[{severity}] [{name}]: {message}",
            ),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            # 1) Controller interface node
            Node(
                package="bng_controller",
                executable="run_controller",
                name="controller_interface",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"config": LaunchConfiguration("config")},
                    {"host": LaunchConfiguration("host")},
                    {"port": LaunchConfiguration("port")},
                    {"log_level": LaunchConfiguration("log_level")},
                ],
            ),
        ]
    )
