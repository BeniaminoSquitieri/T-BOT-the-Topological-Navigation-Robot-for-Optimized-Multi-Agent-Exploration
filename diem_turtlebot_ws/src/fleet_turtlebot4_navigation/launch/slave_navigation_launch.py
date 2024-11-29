# launch/slave_navigation_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Dichiarazione degli argomenti di lancio
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        description='Unique namespace of the robot (e.g., robot_1)'
    )

    initial_x_arg = DeclareLaunchArgument(
        'initial_x',
        description='Initial x coordinate'
    )

    initial_y_arg = DeclareLaunchArgument(
        'initial_y',
        description='Initial y coordinate'
    )

    initial_orientation_arg = DeclareLaunchArgument(
        'initial_orientation',
        description='Initial orientation (NORTH, EAST, SOUTH, WEST)'
    )

    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='ID of the robot'
    )

    # Creazione della descrizione del lancio
    return LaunchDescription([
        robot_namespace_arg,
        initial_x_arg,
        initial_y_arg,
        initial_orientation_arg,
        robot_id_arg,
        # Creazione del nodo
        Node(
            package='fleet_turtlebot4_navigation',  # Sostituisci con il nome del tuo pacchetto
            executable='slave_navigation_node',     # Nome dell'eseguibile del tuo nodo
            name='slave_navigation_node',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            arguments=[
                '--robot_namespace', LaunchConfiguration('robot_namespace'),
                '--initial_x', LaunchConfiguration('initial_x'),
                '--initial_y', LaunchConfiguration('initial_y'),
                '--initial_orientation', LaunchConfiguration('initial_orientation'),
                '--robot_id', LaunchConfiguration('robot_id')
            ],
        ),
    ])

#ros2 launch fleet_turtlebot4_navigation slave_navigation_launch.py     robot_namespace:=robot_111     initial_x:=-2.3077550000000002     initial_y:=-0.5235470000000007     initial_orientation:=SOUTH     robot_id:=111