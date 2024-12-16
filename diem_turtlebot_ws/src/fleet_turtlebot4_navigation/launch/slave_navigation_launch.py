#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import launch
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
        launch.actions.DeclareLaunchArgument(
            'initial_node_label',
            default_value='node_5',  # Nome del nodo iniziale del grafo
            description='Nome del nodo iniziale dal quale estrarre proosizione X e Y'
        ),
        launch.actions.DeclareLaunchArgument(
            'initial_orientation',
            default_value='NORTH',  # Orientamento come stringa
            description='Orientamento iniziale (NORTH, EAST, SOUTH, WEST)'
        ),

        # Definizione del nodo slave_navigation_node
        Node(
            package='fleet_turtlebot4_navigation',
            executable='slave_navigation_node',
            name='slave_navigation_node',
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            output='screen',
            arguments=[
                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '--initial_node_label', launch.substitutions.LaunchConfiguration('initial_node_label'),
                '--initial_orientation', launch.substitutions.LaunchConfiguration('initial_orientation'),
            ]
        )
    ])
