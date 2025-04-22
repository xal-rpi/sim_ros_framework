from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path


def generate_launch_description():
    pkg_share   = get_package_share_directory("bng_simulator")
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
                description="Your nodes' log level (DEBUG,INFO,WARN,ERROR)",
            ),
            DeclareLaunchArgument(
                "listen_ip",
                default_value="0.0.0.0",
                description="HLC UDP listen IP",
            ),
            DeclareLaunchArgument(
                "listen_port",
                default_value="64258",
                description="HLC UDP listen port",
            ),
            DeclareLaunchArgument(
                "send_ip",
                default_value="172.26.32.1",
                description="HLC UDP send IP",
            ),
            DeclareLaunchArgument(
                "send_port",
                default_value="64257",
                description="HLC UDP send port",
            ),
            DeclareLaunchArgument(
                "control_rate",
                default_value="0.01",
                description="HLC control loop period (s)",
            ),
            DeclareLaunchArgument(
                "sim_start_delay",
                default_value="1.0",
                description="Delay after sim_ready before HLC starts (s)",
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
                parameters=[
                    {"config_path":  LaunchConfiguration("config_path")},
                    {"log_level":    LaunchConfiguration("log_level")},
                ],
            ),

            # 2) High-level controller node
            Node(
                package="bng_controller",
                executable="high_level_controller",
                name="high_level_controller",
                output="screen",
                parameters=[
                    {"log_level":        LaunchConfiguration("log_level")},
                    {"listen_ip":        LaunchConfiguration("listen_ip")},
                    {"listen_port":      LaunchConfiguration("listen_port")},
                    {"send_ip":          LaunchConfiguration("send_ip")},
                    {"send_port":        LaunchConfiguration("send_port")},
                    {"control_rate":     LaunchConfiguration("control_rate")},
                    {"sim_start_delay":  LaunchConfiguration("sim_start_delay")},
                ],
            ),
        ]
    )
