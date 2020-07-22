from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mopat_pkg",
            node_namespace="/simulator",
            node_executable="simulator_node",
            node_name="simulator_node",
            output="screen",
        ),
        Node(
            package="mopat_pkg",
            node_namespace="/tracking",
            node_executable="occ_map_node",
            node_name="occ_map_node",
            output="screen",
        ),
        Node(
            package="mopat_pkg",
            node_namespace="/control",
            node_executable="config_space_node",
            node_name="config_space_node",
            output="screen",
        ),
        Node(
            package="mopat_pkg",
            node_namespace="/control",
            node_executable="discretization_node",
            node_name="discretization_node",
            output="screen",
            parameters=[
                {"samp_size" : "2"}
            ]
        ),
    ])
