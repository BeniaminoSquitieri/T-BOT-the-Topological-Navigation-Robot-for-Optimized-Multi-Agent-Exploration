# multi_nav_pkg/launch/master_launch.py

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

    # Capacità dei robot (facoltativo, separati da virgola)
    capacities_arg = DeclareLaunchArgument(
        'capacities',
        default_value='',
        description='Comma-separated list of robot capacities (optional)'
    )

    # Posizioni iniziali dei robot (facoltativo, formato x1:y1,x2:y2,...)
    start_positions_arg = DeclareLaunchArgument(
        'start_positions',
        default_value='',
        description='Comma-separated list of starting positions in the format x1:y1,x2:y2,... (optional)'
    )

    # Creazione della descrizione di lancio
    ld = LaunchDescription([
        graph_path_arg,
        output_dir_arg,
        capacities_arg,
        start_positions_arg,
    ])

    def launch_setup(context, *args, **kwargs):
        graph_path = LaunchConfiguration('graph_path').perform(context)
        output_dir = LaunchConfiguration('output_dir').perform(context)
        capacities_str = LaunchConfiguration('capacities').perform(context)
        start_positions_str = LaunchConfiguration('start_positions').perform(context)

        # Parsing delle capacità
        if capacities_str:
            capacities = [int(cap.strip()) for cap in capacities_str.split(',')]
        else:
            capacities = None

        # Parsing delle posizioni iniziali
        if start_positions_str:
            start_positions = []
            for pos_str in start_positions_str.split(','):
                x, y = pos_str.strip().split(':')
                start_positions.append({'x': float(x), 'y': float(y)})
        else:
            start_positions = None

        # Lancio del nodo Master
        master_node = Node(
            package='multi_nav_pkg',
            executable='master_navigation_node',
            name='master_navigation_node',
            output='screen',
            parameters=[],
            arguments=[
                '--graph_path', graph_path,
                '--output_dir', output_dir
            ],
        )

        return [master_node]

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
