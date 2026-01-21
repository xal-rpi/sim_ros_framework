from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from os import path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory("bng_simulator")

    # Get default config path
    default_config_path = path.join(pkg_share, "config", "etk_scenario.yaml")

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config_path,
                description="Path to the simulation configuration file",
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
                        "config_path": LaunchConfiguration("config_path"),
                        "log_level": LaunchConfiguration("log_level"),
                    }
                ],
            ),
        ]
    )
