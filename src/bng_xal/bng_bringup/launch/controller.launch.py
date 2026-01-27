from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    config_name = LaunchConfiguration("config")

    # Always produce an absolute path in the installed share space
    config_path = PathJoinSubstitution(
        [FindPackageShare("bng_bringup"), "config", "scenarios", config_name]
    )

    return LaunchDescription(
        [
            # common args
            DeclareLaunchArgument(
                "config",
                default_value="throttle_sweep.yaml",
                description="Simulation config file name or path (e.g., 'throttle_sweep.yaml')",
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
            # Scenario mode
            DeclareLaunchArgument(
                "scenario_mode",
                default_value="create",
                description="Scenario mode: 'create' (default), 'attach', or 'auto'",
            ),
            # Attach fallback
            DeclareLaunchArgument(
                "attach_fallback",
                default_value="False",
                description="If True, fallback to create mode when attach fails",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="DEBUG",
                description="Your nodes' log level (FULL,DEBUG,INFO,WARN,ERROR,FATAL)",
            ),
            DeclareLaunchArgument(
                "enable_path_viz",
                default_value="false",
                description="Whether to launch the path visualization node",
            ),
            DeclareLaunchArgument(
                "vehicle_name",
                default_value="ego",
                description="Vehicle name to control",
            ),
            # prettier logs
            SetEnvironmentVariable(
                "RCUTILS_CONSOLE_OUTPUT_FORMAT",
                "[{severity}] [{name}]: {message}",
            ),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            # 1) Simulation manager node (from bng_simulator package)
            Node(
                package="bng_simulator",
                executable="sim_manager_node",
                name="sim_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"config": config_path},
                    {"host": LaunchConfiguration("host")},
                    {"port": LaunchConfiguration("port")},
                    {"scenario_mode": LaunchConfiguration("scenario_mode")},
                    {"attach_fallback": LaunchConfiguration("attach_fallback")},
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
                    {"vehicle_name": LaunchConfiguration("vehicle_name")},
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
