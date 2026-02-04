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
                "remote",
                default_value="",
                description="Remote/WSL IP address for controller sendIp",
            ),
            DeclareLaunchArgument(
                "beamng_port",
                default_value="25252",
                description="BeamNG simulator port number",
            ),
            DeclareLaunchArgument(
                "bng_listen_port",
                default_value="64257",
                description="BeamNG controller listen port",
            ),
            DeclareLaunchArgument(
                "bng_send_port",
                default_value="64258",
                description="BeamNG controller send port",
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
                    {"remote": LaunchConfiguration("remote")},
                    {"beamng_port": LaunchConfiguration("beamng_port")},
                    {"bng_listen_port": LaunchConfiguration("bng_listen_port")},
                    {"bng_send_port": LaunchConfiguration("bng_send_port")},
                    {"scenario_mode": LaunchConfiguration("scenario_mode")},
                    {"attach_fallback": LaunchConfiguration("attach_fallback")},
                    {"log_level": LaunchConfiguration("log_level")},
                ],
            ),
        ]
    )
