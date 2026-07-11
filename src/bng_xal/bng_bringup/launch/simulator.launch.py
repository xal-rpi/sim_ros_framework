from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="gridworld.yaml",
                description="Run config filename — all scenario detail in YAML (config/runs/)",
            ),
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
                description="BASE control_listen port for io.port_index 0 only; vehicle N uses base + N*port_stride",
            ),
            DeclareLaunchArgument(
                "bng_send_port",
                default_value="64258",
                description="BASE control_state_send port for io.port_index 0",
            ),
            DeclareLaunchArgument(
                "bng_sensor_port",
                default_value="64259",
                description="BASE sensor_send port for io.port_index 0",
            ),
            DeclareLaunchArgument(
                "scenario_mode",
                default_value="create",
                description="Scenario mode: 'create' (default), 'attach', or 'auto'",
            ),
            DeclareLaunchArgument(
                "attach_fallback",
                default_value="False",
                description="If True, fallback to create mode when attach fails",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
            ),
            DeclareLaunchArgument(
                "vehicle",
                default_value="",
                description="Single-vehicle shorthand: override compose.vehicle (catalog id)",
            ),
            DeclareLaunchArgument(
                "vehicle_id",
                default_value="",
                description="Single-vehicle shorthand: override compose.vehicle_id",
            ),
            DeclareLaunchArgument(
                "level",
                default_value="",
                description="Single-vehicle shorthand: override compose.level",
            ),
            DeclareLaunchArgument(
                "spawn",
                default_value="",
                description="Single-vehicle shorthand: override compose.spawn preset",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="",
                description="Single-vehicle shorthand: override xlab spawn yaw [deg]",
            ),
            DeclareLaunchArgument(
                "pos",
                default_value="",
                description="Single-vehicle shorthand: absolute spawn pos as 'x,y,z' [m]",
            ),
            DeclareLaunchArgument(
                "preset",
                default_value="",
                description=(
                    "User preset YAML overlay (name, relative path, or absolute path). "
                    "Searched in config/presets/user/ and ~/.config/bng_bringup/presets/"
                ),
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
                        "remote": LaunchConfiguration("remote"),
                        "beamng_port": LaunchConfiguration("beamng_port"),
                        "bng_listen_port": LaunchConfiguration("bng_listen_port"),
                        "bng_send_port": LaunchConfiguration("bng_send_port"),
                        "bng_sensor_port": LaunchConfiguration("bng_sensor_port"),
                        "scenario_mode": LaunchConfiguration("scenario_mode"),
                        "attach_fallback": LaunchConfiguration("attach_fallback"),
                        "log_level": LaunchConfiguration("log_level"),
                        "vehicle": LaunchConfiguration("vehicle"),
                        "vehicle_id": LaunchConfiguration("vehicle_id"),
                        "level": LaunchConfiguration("level"),
                        "spawn": LaunchConfiguration("spawn"),
                        "yaw": LaunchConfiguration("yaw"),
                        "pos": LaunchConfiguration("pos"),
                        "preset": LaunchConfiguration("preset"),
                    }
                ],
            ),
            Node(
                package="bng_controller",
                executable="sensor_dispatcher",
                name="sensor_dispatcher",
                output="screen",
                parameters=[
                    {
                        "config": LaunchConfiguration("config"),
                        "host": LaunchConfiguration("host"),
                        "remote": LaunchConfiguration("remote"),
                        "bng_listen_port": LaunchConfiguration("bng_listen_port"),
                        "bng_send_port": LaunchConfiguration("bng_send_port"),
                        "bng_sensor_port": LaunchConfiguration("bng_sensor_port"),
                        "log_level": LaunchConfiguration("log_level"),
                        "frame_id": "map",
                        "vehicle": LaunchConfiguration("vehicle"),
                        "vehicle_id": LaunchConfiguration("vehicle_id"),
                        "level": LaunchConfiguration("level"),
                        "spawn": LaunchConfiguration("spawn"),
                        "yaw": LaunchConfiguration("yaw"),
                        "pos": LaunchConfiguration("pos"),
                        "preset": LaunchConfiguration("preset"),
                    }
                ],
            ),
        ]
    )
