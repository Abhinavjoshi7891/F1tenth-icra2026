import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('frenet_planner')
    default_config = os.path.join(pkg_share, 'config', 'frenet_planner_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'frenet_config',
            default_value=default_config,
            description='Path to frenet planner params YAML'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/car_state/odom',
            description='Odometry topic'),

        Node(
            package='frenet_planner',
            executable='frenet_planner_node',
            name='frenet_planner',
            output='screen',
            parameters=[LaunchConfiguration('frenet_config')],
            remappings=[
                ('/odom', LaunchConfiguration('odom_topic')),
            ],
        ),
    ])
