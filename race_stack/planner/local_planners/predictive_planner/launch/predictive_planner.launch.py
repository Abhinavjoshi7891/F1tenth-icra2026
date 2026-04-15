import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('predictive_planner')
    
    return LaunchDescription([
        # SQP Avoidance Node
        Node(
            package='predictive_planner',
            executable='sqp_node',
            name='planner_sqp',
            output='screen',
            parameters=[{
                'evasion_dist': 0.65,
                'lookahead_dist': 15.0,
                'avoidance_resolution': 20
            }]
        ),
        
        # Waypoint Updater
        Node(
            package='predictive_planner',
            executable='waypoint_updater',
            name='waypoint_updater',
            output='screen'
        ),
        
        # Collision Predictor
        Node(
            package='predictive_planner',
            executable='collision_predictor',
            name='collision_predictor',
            output='screen'
        ),
        
        # GP Node (Trajectory Prediction)
        Node(
            package='predictive_planner',
            executable='gp_node',
            name='gp_node',
            output='screen',
            parameters=[{
                'num_opponents': 1,
                'measure': False
            }]
        )
    ])
