# local_map_pp_cartographer.launch.py
# Drop this in autodrive_devkit_workspace/autodrive_roboracer/launch/

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # AutoDRIVE bridge — from baked-in autodrive_roboracer package in container
        Node(
            package    = 'autodrive_roboracer',
            executable = 'autodrive_bridge',
            name       = 'autodrive_bridge',
            output     = 'screen',
            remappings = [('/tf', '/tf_devkit_unused')],
        ),
        # Racing controller — from mounted autodrive_adapter package
        Node(
            package    = 'autodrive_adapter',
            executable = 'local_map_pp_cartographer',
            name       = 'local_map_pp_cartographer',
            output     = 'screen',
        ),
    ])