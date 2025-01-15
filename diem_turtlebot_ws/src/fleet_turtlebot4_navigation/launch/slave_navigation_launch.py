#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates a launch description that starts the RealRobotNavigationNode
    (or whichever node you like) with arguments for both robot_namespace and graph_path.
    """
    return LaunchDescription([
        # Launch argument for the robot namespace
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot1',
            description='Namespace for the robot. e.g. "robot1", "robot2", etc.'
        ),
        
        # Launch argument for the path to the navigation graph
        DeclareLaunchArgument(
            'graph_path',
            default_value='/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map_transformation_phase/result_graph_original.json',
            description='Path to the navigation graph JSON file.'
        ),

        # Definition of the Node to launch
        Node(
            package='fleet_turtlebot4_navigation',
            executable='slave_navigation_node',  # match your setup.py entry point
            name='slave_navigation_node',         # how you want it to appear in ROS
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',

            # Pass the command-line arguments to the node
            arguments=[
                '--robot_namespace', LaunchConfiguration('robot_namespace'),
                '--graph_path', LaunchConfiguration('graph_path')
            ]
        )
    ])
