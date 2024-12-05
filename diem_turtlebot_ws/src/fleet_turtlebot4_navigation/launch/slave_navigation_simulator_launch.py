#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Dichiarazione degli argomenti di lancio
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot_simulator',
            description='Namespace del robot'
        ),
        DeclareLaunchArgument(
            'initial_x',
            default_value='0.0',
            description='Posizione iniziale X'
        ),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.0',
            description='Posizione iniziale Y'
        ),
        DeclareLaunchArgument(
            'initial_orientation',
            default_value='NORTH',
            description='Orientamento iniziale (NORTH, EAST, SOUTH, WEST)'
        ),

        # Definizione del nodo SlaveNavigationSimulator
        Node(
            package='fleet_turtlebot4_navigation',
            executable='simulated_slave_navigation_node',  
            name='simulated_slave_navigation_node',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            arguments=[
                '--robot_namespace', LaunchConfiguration('robot_namespace'),
                '--initial_x', LaunchConfiguration('initial_x'),
                '--initial_y', LaunchConfiguration('initial_y'),
                '--initial_orientation', LaunchConfiguration('initial_orientation'),
            ]
        )
    ])
