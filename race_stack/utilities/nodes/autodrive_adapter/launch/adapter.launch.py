from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autodrive_adapter',
            executable='adapter_node',
            name='autodrive_adapter',
            output='screen',
            emulate_tty=True,
        ),
    ])
