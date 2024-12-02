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
        ),
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
        launch.actions.DeclareLaunchArgument(
            'robot_id',
            default_value='111',
            description='ID del robot'
        ),

        # Definizione del nodo slave_navigation_node
        Node(
            package='fleet_turtlebot4_navigation',
            executable='slave_navigation_node',
            name='slave_navigation_node',
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[{
                'robot_namespace': launch.substitutions.LaunchConfiguration('robot_namespace'),
                'initial_x': launch.substitutions.LaunchConfiguration('initial_x'),
                'initial_y': launch.substitutions.LaunchConfiguration('initial_y'),
                'initial_orientation': launch.substitutions.LaunchConfiguration('initial_orientation'),
                'robot_id': launch.substitutions.LaunchConfiguration('robot_id')
            }]
        )
    ])




#ros2 launch fleet_turtlebot4_navigation slave_navigation_launch.py     robot_namespace:=robot_111     initial_x:=-2.3077550000000002     initial_y:=-0.5235470000000007     initial_orientation:=SOUTH     robot_id:=111
# ros2 launch fleet_turtlebot4_navigation slave_navigation_launch.py \
#     robot_namespace:=robot_111 \
#     initial_x:=-2.307755 \
#     initial_y:=-0.523547 \
#     initial_orientation:=NORTH \
#     robot_id:=111
