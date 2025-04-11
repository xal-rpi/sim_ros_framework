from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "listen_ip",
                default_value="172.26.32.1",
                description="IP address to listen for sensor data",
            ),
            DeclareLaunchArgument(
                "listen_port",
                default_value="64257",
                description="Port to listen for sensor data",
            ),
            DeclareLaunchArgument(
                "send_ip",
                default_value="172.26.32.1",
                description="IP address to send control targets",
            ),
            DeclareLaunchArgument(
                "send_port",
                default_value="64258",
                description="Port to send control targets",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value="basic_scenario.yaml",
                description="Path to BeamNG simulation configuration file",
            ),
            # Controller interface node
            Node(
                package="bng_controller",
                executable="run_controller",
                name="controller",
                output="screen",
                parameters=[
                    {
                        "listen_ip": LaunchConfiguration("listen_ip"),
                        "listen_port": LaunchConfiguration("listen_port"),
                        "send_ip": LaunchConfiguration("send_ip"),
                        "send_port": LaunchConfiguration("send_port"),
                        "config_path": LaunchConfiguration("config_path"),
                    }
                ],
            ),
        ]
    )
