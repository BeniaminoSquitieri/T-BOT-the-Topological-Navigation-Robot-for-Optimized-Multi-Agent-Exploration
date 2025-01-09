from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Dichiarazione dell'argomento di lancio per 'graph_path'
    graph_path_arg = DeclareLaunchArgument(
        'graph_path',
        default_value='/home/beniamino/turtlebot4/diem_turtlebot_ws/src/fleet_turtlebot4_navigation/map_transformation_phase/result_graph_original.json',
        description='Path to the navigation graph JSON file'
    )

    # Dichiarazione dell'argomento di lancio per 'check_interval' (opzionale)
    check_interval_arg = DeclareLaunchArgument(
        'check_interval',
        default_value='2.0',
        description='Interval in seconds to check slave statuses'
    )

    # Dichiarazione dell'argomento di lancio per 'timeout' (opzionale)
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='150.0',
        description='Timeout in seconds to consider a slave inactive'
    )

    # Definizione del nodo master con parametri passati dagli argomenti di lancio
    master_node = Node(
        package='fleet_turtlebot4_navigation',
        executable='master_navigation_node',
        name='master_navigation_node',
        output='screen',
        parameters=[{
            'graph_path': LaunchConfiguration('graph_path'),
            'check_interval': LaunchConfiguration('check_interval'),
            'timeout': LaunchConfiguration('timeout')
        }]
    )

    return LaunchDescription([
        graph_path_arg,
        check_interval_arg,
        timeout_arg,
        master_node
    ])
