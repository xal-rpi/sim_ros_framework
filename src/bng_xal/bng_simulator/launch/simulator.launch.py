from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from os import path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory("bng_simulator")

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "config",
                default_value="basic_scenario.yaml",
                description="Simulation config file name or path (e.g., 'basic_scenario.yaml')",
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
            # Better logging
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
            ),
            SetEnvironmentVariable(
                name="RCUTILS_CONSOLE_OUTPUT_FORMAT",
                value="[{severity}] [{name}]: {message}",
            ),
            SetEnvironmentVariable(
                name="RCUTILS_COLORIZED_OUTPUT",
                value="1",
            ),
            Node(
                package="bng_simulator",
                executable="sim_manager_node",
                name="sim_manager_node",
                output="screen",
                parameters=[
                    {
                        "config": LaunchConfiguration("config"),
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "log_level": LaunchConfiguration("log_level"),
                    }
                ],
            ),
        ]
    )
