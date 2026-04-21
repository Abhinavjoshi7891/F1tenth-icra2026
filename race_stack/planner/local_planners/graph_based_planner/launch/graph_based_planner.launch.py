import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('graph_based_planner')
    default_cfg = os.path.join(pkg_share, 'config', 'graph_based_planner_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('graph_based_config', default_value=default_cfg,
                              description='Path to graph_based_planner params YAML'),
        DeclareLaunchArgument('map_name',        default_value=''),
        DeclareLaunchArgument('display_graph',   default_value='True'),
        DeclareLaunchArgument('follow_mode',     default_value='0'),
        DeclareLaunchArgument('racecar_version', default_value='NUC2'),
        DeclareLaunchArgument('measure',         default_value='False'),
        DeclareLaunchArgument('profile',         default_value='False'),

        Node(
            package='graph_based_planner',
            executable='graph_planner_node',
            name='GraphPlanner',
            output='screen',
            parameters=[
                LaunchConfiguration('graph_based_config'),
                {
                    'map_name':        LaunchConfiguration('map_name'),
                    'display_graph':   LaunchConfiguration('display_graph'),
                    'follow_mode':     LaunchConfiguration('follow_mode'),
                    'racecar_version': LaunchConfiguration('racecar_version'),
                    'measure':         LaunchConfiguration('measure'),
                    'profile':         LaunchConfiguration('profile'),
                },
            ],
        ),
    ])
