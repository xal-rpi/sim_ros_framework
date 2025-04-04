from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bng_simulator',
            executable='sim_manager_node',
            name='sim_manager',
            output='screen'
        )
    ])
