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
            default_value='robot_111',
            description='Namespace del robot'
        ), #node 5
        launch.actions.DeclareLaunchArgument(
            'initial_x',
            default_value='-2.307755',
            description='Posizione iniziale X'
        ),
        launch.actions.DeclareLaunchArgument(
            'initial_y',
            default_value='-0.523547',
            description='Posizione iniziale Y'
        ),
        launch.actions.DeclareLaunchArgument(
            'initial_orientation',
            default_value='NORTH',  # Passa l'orientamento come stringa
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
                '--initial_x', launch.substitutions.LaunchConfiguration('initial_x'),
                '--initial_y', launch.substitutions.LaunchConfiguration('initial_y'),
                '--initial_orientation', launch.substitutions.LaunchConfiguration('initial_orientation'),
            ]
        )
    ])




