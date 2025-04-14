from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory("bng_simulator")

    # Get default config path
    default_config_path = os.path.join(pkg_share, "config", "basic_scenario.yaml")

    # Define the default log format - more concise than the default
    default_log_format = "[{severity}] {message}"

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "config",
                default_value=default_config_path,
                description="Path to the simulation configuration file",
            ),
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
                        "log_level": LaunchConfiguration("log_level"),
                    }
                ],
            ),
        ]
    )
