from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=['/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/map_server_params.yaml']
        )
    ])
