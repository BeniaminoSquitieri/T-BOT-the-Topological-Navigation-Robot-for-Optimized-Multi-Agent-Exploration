# multi_robot_navigation_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def generate_launch_description():
    # Declare launch arguments
    graph_path_arg = DeclareLaunchArgument(
        'graph_path',
        description='Path to the full graph JSON file'
    )

    robot_namespaces_arg = DeclareLaunchArgument(
        'robot_namespaces',
        description='Comma-separated list of robot namespaces'
    )

    start_positions_arg = DeclareLaunchArgument(
        'start_positions',
        description='Comma-separated list of starting positions in the format x1:y1,x2:y2,...'
    )

    ld = LaunchDescription([
        graph_path_arg,
        robot_namespaces_arg,
        start_positions_arg,
    ])

    def add_robot_nodes(context):
        graph_path = context.launch_configurations['graph_path']
        robot_ns_list = context.launch_configurations['robot_namespaces'].split(',')
        start_positions_list = context.launch_configurations['start_positions'].split(',')
        num_robots_value = len(robot_ns_list)

        nodes = []
        for idx, (robot_ns, start_pos) in enumerate(zip(robot_ns_list, start_positions_list)):
            robot_ns = robot_ns.strip()
            start_x, start_y = start_pos.strip().split(':')
            navigation_node = Node(
                package='your_package_name',  # Replace with your package name
                executable='robot_navigation_node',  # Name of the script without .py extension
                namespace=robot_ns,
                name=robot_ns + '_navigation_node',
                output='screen',
                parameters=[],
                arguments=[
                    '--robot_namespace', robot_ns,
                    '--graph_path', graph_path,
                    '--robot_id', str(idx),
                    '--num_robots', str(num_robots_value),
                    '--start_x', start_x,
                    '--start_y', start_y,
                ],
            )
            nodes.append(navigation_node)
        return nodes

    ld.add_action(OpaqueFunction(function=add_robot_nodes))

    return ld

#ros2 launch your_package_name multi_robot_navigation_launch.py \graph_path:=/path/to/your/graph/full_graph.json \robot_namespaces:=robot_1,robot_2,robot_3 \start_positions:=75.79224500000001:2.5764530000000008,-2.3077550000000002:-0.5235470000000007,-23.607755000000004:-0.22354699999999994
