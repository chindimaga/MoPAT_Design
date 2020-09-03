from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mopat_pkg",
            node_namespace="/testbed",
            node_executable="camera_node",
            node_name="camera_node",
            output="screen",
        ),
        Node(
            package="mopat_pkg",
            node_namespace="/tracking",
            node_executable="localization_node",
            node_name="localization_node",
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
        Node(
            package="mopat_pkg",
            node_namespace="/control",
            node_executable="motion_planning_node",
            node_name="motion_planning_node",
            output="screen",
        ),
    ])
