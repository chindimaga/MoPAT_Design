from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mopat_pkg",
            node_namespace="/testbed",
            node_executable="camera_node",
            node_name="camera_node",
            output="screen"
        ),
        Node(
            package="mopat_pkg",
            node_namespace="/tracking",
            node_executable="occ_map_node",
            node_name="occ_map_node",
            output="screen"
        ),
        Node(
            package="mopat_pkg",
            node_namespace="/control",
            node_executable="config_space_node",
            node_name="cconfig_space_node",
            output="screen"
        ),
    ])
