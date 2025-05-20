from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path


def generate_launch_description():
    pkg_share = get_package_share_directory("bng_simulator")
    default_cfg = path.join(pkg_share, "config", "basic_scenario.yaml")

    return LaunchDescription(
        [
            # common args
            DeclareLaunchArgument(
                "config_path",
                default_value=default_cfg,
                description="Path to BeamNG simulation YAML",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Your nodes' log level (FULL,DEBUG,INFO,WARN,ERROR,FATAL)",
            ),
            DeclareLaunchArgument(
                "enable_path_viz",
                default_value="false",
                description="Whether to launch the path visualization node",
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
                    {"config_path": LaunchConfiguration("config_path")},
                    {"log_level": LaunchConfiguration("log_level")},
                ],
            ),
            # 2) High-level controller node
            Node(
                package="bng_controller",
                executable="high_level_controller",
                name="high_level_controller",
                output="screen",
                parameters=[
                    {"config_path": LaunchConfiguration("config_path")},
                    {"log_level": LaunchConfiguration("log_level")},
                ],
            ),
            # 3) Optional path visualization adapter
            Node(
                package="bng_controller",
                executable="path_viz",
                name="path_vis_adapter",
                output="screen",
                parameters=[
                    {"path_file": LaunchConfiguration("path_file_viz")},
                ],
                condition=IfCondition(LaunchConfiguration("enable_path_viz")),
            ),
        ]
    )
