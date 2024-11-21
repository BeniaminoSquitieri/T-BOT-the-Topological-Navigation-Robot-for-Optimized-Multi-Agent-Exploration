# multi_robot_navigation_launch.py

# Import necessary classes and functions from the ROS 2 launch system
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def generate_launch_description():
    """
    Generates the launch description for the multi-robot navigation system.
    """

    # Declare a launch argument for the path to the graph JSON file
    graph_path_arg = DeclareLaunchArgument(
        'graph_path',  # Argument name
        description='Path to the full graph JSON file'  # Description for the argument
    )

    # Declare a launch argument for the list of robot namespaces
    robot_namespaces_arg = DeclareLaunchArgument(
        'robot_namespaces',  # Argument name
        description='Comma-separated list of robot namespaces'  # Description for the argument
    )

    # Declare a launch argument for the list of robot starting positions
    start_positions_arg = DeclareLaunchArgument(
        'start_positions',  # Argument name
        description='Comma-separated list of starting positions in the format x1:y1,x2:y2,...'  # Description
    )

    # Create a LaunchDescription object and add the declared arguments
    ld = LaunchDescription([
        graph_path_arg,
        robot_namespaces_arg,
        start_positions_arg,
    ])

    def add_robot_nodes(context):
        """
        Dynamically adds robot nodes based on the launch arguments.

        Args:
            context: The launch context containing the argument values.

        Returns:
            list: A list of Node objects for each robot.
        """
        # Retrieve the graph path from the context
        graph_path = context.launch_configurations['graph_path']

        # Retrieve the list of robot namespaces and starting positions
        robot_ns_list = context.launch_configurations['robot_namespaces'].split(',')
        start_positions_list = context.launch_configurations['start_positions'].split(',')

        # Calculate the total number of robots based on the namespaces provided
        num_robots_value = len(robot_ns_list)

        # Initialize an empty list to hold the nodes for each robot
        nodes = []

        # Iterate over each robot's namespace and starting position
        for idx, (robot_ns, start_pos) in enumerate(zip(robot_ns_list, start_positions_list)):
            robot_ns = robot_ns.strip()  # Strip whitespace from the namespace
            start_x, start_y = start_pos.strip().split(':')  # Parse the starting position into x and y

            # Create a Node object for the robot's navigation node
            navigation_node = Node(
                package='nav_pkg',  # Replace with the name of your package
                executable='robot_navigation_node',  # Name of the executable (without .py)
                namespace=robot_ns,  # Assign the namespace for the robot
                name=robot_ns + '_navigation_node',  # Unique name for the node
                output='screen',  # Output the logs to the screen
                parameters=[],  # Empty parameters list (can add more if needed)
                arguments=[
                    '--robot_namespace', robot_ns,  # Pass the robot's namespace as an argument
                    '--graph_path', graph_path,  # Pass the path to the graph JSON file
                    '--robot_id', str(idx),  # Pass a unique ID for the robot
                    '--num_robots', str(num_robots_value),  # Pass the total number of robots
                    '--start_x', start_x,  # Pass the starting x-coordinate
                    '--start_y', start_y,  # Pass the starting y-coordinate
                ],
            )

            # Append the node to the list of nodes
            nodes.append(navigation_node)

        # Return the list of created nodes
        return nodes

    # Add an OpaqueFunction to dynamically add nodes to the launch description
    ld.add_action(OpaqueFunction(function=add_robot_nodes))

    # Return the final launch description
    return ld

# Example command to launch:
# ros2 launch your_package_name multi_robot_navigation_launch.py graph_path:=/path/to/your/graph/full_graph.json robot_namespaces:=robot_1,robot_2,robot_3 start_positions:=75.79224500000001:2.5764530000000008,-2.3077550000000002:-0.5235470000000007,-23.607755000000004:-0.22354699999999994
