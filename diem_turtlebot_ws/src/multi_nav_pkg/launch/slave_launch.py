# multi_nav_pkg/launch/slave_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Dichiarazione degli argomenti di lancio
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        description='Namespace of the robot (e.g., robot_108)'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp',
        description='Directory to save subgraphs'
    )

    # Creazione della descrizione di lancio
    ld = LaunchDescription([
        robot_namespace_arg,
        output_dir_arg,
    ])

    def launch_setup(context, *args, **kwargs):
        robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
        output_dir = LaunchConfiguration('output_dir').perform(context)

        # Lancio del nodo Slave
        slave_node = Node(
            package='multi_nav_pkg',
            executable='slave_navigation_node',
            namespace=robot_namespace,
            name=f'{robot_namespace}_slave_navigation_node',
            output='screen',
            parameters=[],
            arguments=[
                '--robot_namespace', robot_namespace,
                '--output_dir', output_dir
            ],
        )

        return [slave_node]

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
