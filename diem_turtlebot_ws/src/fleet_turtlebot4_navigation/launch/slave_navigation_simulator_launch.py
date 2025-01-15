#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Argomento per il namespace del robot
        DeclareLaunchArgument(
            'robot_namespace',
            description='Namespace del robot'
        ),
        # Argomento per il percorso del grafo
        DeclareLaunchArgument(
            'graph_path',
            default_value='/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map_transformation_phase/graph/result_graph_reduced.json',
            description='Percorso al file del grafo di navigazione in formato JSON'
        ),
        # Nodo del robot simulato
        Node(
            package='fleet_turtlebot4_navigation',
            executable='simulated_slave_navigation_node',
            name='simulated_slave_navigation_node',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            arguments=[
                '--robot_namespace', LaunchConfiguration('robot_namespace'),
                '--graph_path', LaunchConfiguration('graph_path')
            ]
        )
    ])
