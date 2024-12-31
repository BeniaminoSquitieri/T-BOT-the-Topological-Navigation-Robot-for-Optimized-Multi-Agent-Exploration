#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import launch.substitutions


def generate_launch_description():
    return LaunchDescription([
        # Dichiarazione dei parametri di lancio
        launch.actions.DeclareLaunchArgument(
            'robot_namespace',
            default_value='turtlebot1',
            description='Namespace del robot'
        ),

        # Definizione del nodo slave_navigation_node
        Node(
            package='fleet_turtlebot4_navigation',
            executable='slave_navigation_node',  # Deve corrispondere all'entry point nel setup.py
            name='slave_navigation_node',  # Nome del nodo ROS corrispondente al nome nel script Python
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            output='screen',
            arguments=[
                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
            ]
        )
    ])
