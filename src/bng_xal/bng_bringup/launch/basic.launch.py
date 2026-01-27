from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_name = LaunchConfiguration("config")

    # Always produce an absolute path in the installed share space
    config_path = PathJoinSubstitution(
        [FindPackageShare("bng_bringup"), "config/scenarios", config_name]
    )

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "config",
                default_value="etk_scenario.yaml",
                description="Config filename under bng_bringup/config/",
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
            # Scenario mode (NEW)
            DeclareLaunchArgument(
                "scenario_mode",
                default_value="create",
                description="Scenario mode: 'create' (default), 'attach', or 'auto'",
            ),
            # Attach fallback (NEW)
            DeclareLaunchArgument(
                "attach_fallback",
                default_value="False",
                description="If True, fallback to create mode when attach fails",
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
                        "config": config_path,
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "scenario_mode": LaunchConfiguration("scenario_mode"),
                        "attach_fallback": LaunchConfiguration("attach_fallback"),
                        "log_level": LaunchConfiguration("log_level"),
                    }
                ],
            ),
            Node(
                package="bng_controller",
                executable="gt_state_bridge",
                name="gt_state_bridge",
                output="screen",
                parameters=[
                    {
                        "log_level": LaunchConfiguration("log_level"),
                        "frame_id": "map",
                    }
                ],
            ),
        ]
    )
