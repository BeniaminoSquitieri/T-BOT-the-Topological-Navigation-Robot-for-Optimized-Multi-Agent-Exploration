from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch.actions
import launch.substitutions

def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot_111',
            description='Namespace of the robot'
        ),
        launch.actions.DeclareLaunchArgument(
            'initial_x',
            default_value='-2.307755',
            description='Initial X position'
        ),
        launch.actions.DeclareLaunchArgument(
            'initial_y',
            default_value='-0.523547',
            description='Initial Y position'
        ),
        launch.actions.DeclareLaunchArgument(
            'initial_orientation',
            default_value='NORTH',
            description='Initial orientation in radians'
        ),
        launch.actions.DeclareLaunchArgument(
            'robot_id',
            default_value='111',
            description='Robot ID'
        ),
        Node(
            package='fleet_turtlebot4_navigation',
            executable='slave_navigation_node',
            name='slave_navigation_node',
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[{
                'robot_namespace': launch.substitutions.LaunchConfiguration('robot_namespace'),
                'initial_x': float(launch.substitutions.LaunchConfiguration('initial_x')),
                'initial_y': float(launch.substitutions.LaunchConfiguration('initial_y')),
                'initial_orientation': launch.substitutions.LaunchConfiguration('initial_orientation'),
                'robot_id': int(launch.substitutions.LaunchConfiguration('robot_id'))
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
