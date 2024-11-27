# multi_nav_pkg/launch/backup_master_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Dichiarazione degli argomenti di lancio
    graph_path_arg = DeclareLaunchArgument(
        'graph_path',
        description='Path to the full graph JSON file'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp',
        description='Directory to save subgraphs'
    )

    # Creazione della descrizione di lancio
    ld = LaunchDescription([
        graph_path_arg,
        output_dir_arg,
    ])

    def launch_setup(context, *args, **kwargs):
        graph_path = LaunchConfiguration('graph_path').perform(context)
        output_dir = LaunchConfiguration('output_dir').perform(context)

        # Lancio del nodo Backup Master
        backup_master_node = Node(
            package='multi_nav_pkg',
            executable='backup_master_navigation_node',
            name='backup_master_navigation_node',
            output='screen',
            parameters=[],
            arguments=[
                '--graph_path', graph_path,
                '--output_dir', output_dir
            ],
        )

        return [backup_master_node]

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
