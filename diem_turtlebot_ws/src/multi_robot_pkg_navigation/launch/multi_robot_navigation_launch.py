# launch/multi_robot_navigation_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import sys
import json
import networkx as nx

# Add the path to the graph partitioning module so it can be imported
sys.path.append('/home/beniamino/turtlebot4/diem_turtlebot_ws/src/multi_robot_pkg_navigation/map/map_transformation_phase/graph')  

# Import the graph partitioning module for graph loading, partitioning, and saving
import graph_partitioning

def generate_launch_description():
    """
    Generates the launch description for the multi-robot navigation system.
    This includes:
    - Declaring required input arguments for the graph path, robot namespaces, start positions, and orientations.
    - Using the graph partitioning module to divide the graph dynamically based on the number of robots.
    - Launching navigation nodes for each robot with its respective subgraph, starting position, and orientation.
    """

    # Declare the argument for the full graph JSON file path
    graph_path_arg = DeclareLaunchArgument(
        'graph_path',  # Argument name
        description='Path to the full graph JSON file'  # Description of the argument
    )

    # Declare the argument for the robot namespaces
    robot_namespaces_arg = DeclareLaunchArgument(
        'robot_namespaces',  # Argument name
        description='Comma-separated list of robot namespaces'  # Description of the argument
    )

    # Declare the argument for the starting positions of robots
    start_positions_arg = DeclareLaunchArgument(
        'start_positions',  # Argument name
        description='Comma-separated list of starting positions in the format x1:y1,x2:y2,...'  # Description
    )

    # Declare the argument for the orientations of robots
    orientations_arg = DeclareLaunchArgument(
        'orientations',  # Argument name
        description='Comma-separated list of orientations for each robot (e.g., NORTH,SOUTH,EAST,WEST)'  # Description
    )

    # Declare the argument for the output directory to save subgraph files
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',  # Argument name
        default_value='/tmp',  # Default value for the output directory
        description='Directory where subgraph files are saved'  # Description of the argument
    )

    # Create the launch description and add the declared arguments
    ld = LaunchDescription([
        graph_path_arg,          # Add the graph path argument to the launch description
        robot_namespaces_arg,    # Add the robot namespaces argument
        start_positions_arg,     # Add the starting positions argument
        orientations_arg,        # Add the orientations argument
        output_dir_arg,          # Add the output directory argument
    ])

    def launch_setup(context, *args, **kwargs):
        # Retrieve the graph path from the launch context
        graph_path = LaunchConfiguration('graph_path').perform(context)
        # Retrieve the list of robot namespaces and split them into a Python list
        robot_namespaces = LaunchConfiguration('robot_namespaces').perform(context).split(',')
        # Retrieve the list of starting positions and split them into a Python list
        start_positions = LaunchConfiguration('start_positions').perform(context).split(',')
        # Retrieve the list of orientations and split them into a Python list
        orientations = LaunchConfiguration('orientations').perform(context).split(',')
        # Retrieve the output directory where subgraph files will be saved
        output_dir = LaunchConfiguration('output_dir').perform(context)
        # Determine the number of robots based on the provided namespaces
        num_robots = len(robot_namespaces)

        # Ensure that the number of starting positions matches the number of robots
        if len(start_positions) != num_robots:
            print("Error: The number of starting positions must match the number of robot namespaces.")
            sys.exit(1)

        # Ensure that the number of orientations matches the number of robots
        if len(orientations) != num_robots:
            print("Error: The number of orientations must match the number of robot namespaces.")
            sys.exit(1)

        # Load the full graph from the provided file path using the graph_partitioning module
        full_graph = graph_partitioning.load_full_graph(graph_path)

        # Ensure the graph is connected before proceeding
        if not nx.is_strongly_connected(full_graph):
            print("The graph is not strongly connected. Please provide a strongly connected graph.")  # Error message
            sys.exit(1)  # Exit the program if the graph is not connected

        # Partition the graph into subgraphs for each robot
        subgraphs = graph_partitioning.partition_graph(full_graph, num_robots)

        # Save each subgraph to the output directory and get the file paths
        subgraph_paths = graph_partitioning.save_subgraphs(subgraphs, output_dir)

        # Initialize an empty list to hold the robot navigation nodes
        nodes = []
        # Iterate over each robot's namespace, starting position, and orientation
        for idx, (robot_ns, start_pos, orientation) in enumerate(zip(robot_namespaces, start_positions, orientations)):
            # Strip whitespace from the robot namespace
            robot_ns = robot_ns.strip()
            # Split the starting position into x and y coordinates
            start_x, start_y = start_pos.strip().split(':')
            # Strip whitespace from the orientation
            orientation = orientation.strip().upper()

            # Validate the orientation input
            valid_orientations = ['NORTH', 'EAST', 'SOUTH', 'WEST']
            if orientation not in valid_orientations:
                print(f"Error: Invalid orientation '{orientation}' for robot '{robot_ns}'. Valid options are {valid_orientations}.")
                sys.exit(1)

            # Get the path to the subgraph file for the current robot
            subgraph_file_path = subgraph_paths[idx]

            # Create a Node object for the robot's navigation node
            navigation_node = Node(
                package='nav_pkg',  # Ensure this matches the actual package name
                executable='robot_navigation_node',  # Name of the executable (without .py)
                namespace=robot_ns,  # Namespace for the robot
                name=robot_ns + '_navigation_node',  # Unique name for the node
                output='screen',  # Output logs to the screen
                parameters=[],  # No additional parameters
                arguments=[
                    '--robot_namespace', robot_ns,  # Pass the robot namespace as an argument
                    '--graph_path', subgraph_file_path,  # Pass the path to the subgraph file
                    '--robot_id', str(idx),  # Pass the unique robot ID (index) as an argument
                    '--num_robots', str(num_robots),  # Pass the number of robots as an argument
                    '--start_x', start_x,  # Pass the x-coordinate of the starting position
                    '--start_y', start_y,  # Pass the y-coordinate of the starting position
                    '--orientation', orientation  # Pass the orientation as an argument
                ],
            )

            # Add the created navigation node to the list
            nodes.append(navigation_node)

        # Return the list of generated nodes
        return nodes

    # Add the launch setup function as an OpaqueFunction to the launch description
    ld.add_action(OpaqueFunction(function=launch_setup))

    # Return the completed launch description
    return ld


#ros2 launch turtlebot4_navigation localization.launch.py map:=diem_map.yaml namespace:=/robot_108 
# ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot_108
# ros2 launch turtlebot4_viz view_robot.launch.py  namespace:=/robot_108

# ros2 launch turtlebot4_navigation localization.launch.py map:=diem_map.yaml namespace:=/robot_111
# ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot_111
# ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot_111

# ros2 launch multi_robot_pkg_navigation multi_robot_navigation_launch.py graph_path:=/home/beniamino/turtlebot4/diem_turtlebot_ws/src/multi_robot_pkg_navigation/nav_pkg/nav_pkg/navigation_hardware_limitation.json robot_namespaces:=robot_108,robot_111 start_positions:=17.292245:-1.1235469999999985,17.192245:-8.623547000000002
