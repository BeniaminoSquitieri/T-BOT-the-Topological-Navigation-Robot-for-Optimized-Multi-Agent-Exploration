from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fleet_turtlebot4_navigation',
            executable='master_navigation_node',
            name='master_navigation_node',
            output='screen',
            parameters=[{
                'graph_path': '/home/beniamino/turtlebot4/diem_turtlebot_ws/src/fleet_turtlebot4_navigation/map/navigation_hardware_limitation.json',
                'check_interval': 2.0,
                'timeout': 150.0  # Timeout increased to 150 seconds
            }]
        )
    ])

#ros2 launch fleet_turtlebot4_navigation master_navigation_launch.py
