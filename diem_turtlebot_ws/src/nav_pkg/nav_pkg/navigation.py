import os  # Module for interacting with the operating system
import argparse  # Module for parsing command-line arguments
import time  # Module for adding delays in code execution
import rclpy  # ROS2 client library for Python
from rclpy.node import Node  # Base class for creating ROS2 nodes
from std_msgs.msg import String  # Standard ROS2 message type for strings
from nav_pkg.graph import Graph  # Import the Graph class (ensure it supports JSON format)
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator  # Assumed custom navigation library for TurtleBot4
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # ROS2 QoS settings
from irobot_create_msgs.msg import KidnapStatus  # Message type for reporting if the robot is kidnapped
import json  # Module for working with JSON files

class NavigationNode(Node):
    """
    ROS2 node for navigation using a graph structure.
    """
    def __init__(self, graph_file_path, starting_point, orientation=None):
        """
        Initialize the navigation node.

        Args:
            graph_file_path (str): Path to the JSON file containing the graph structure.
            starting_point (dict): Starting point with 'x' and 'y' keys representing coordinates.
            orientation (str, optional): Starting orientation ('NORTH', 'EAST', 'SOUTH', 'WEST').
        """
        super().__init__('navigation_node')  # Initialize the ROS2 node with the name 'navigation_node'

        # Define Quality of Service (QoS) profile for communication reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Choose the best-effort reliability for non-critical updates
            history=HistoryPolicy.KEEP_LAST,  # Keep only the latest message in the buffer
            depth=1  # Set the buffer depth to 1 message
        )

        # Load the graph structure from the specified JSON file
        self.graph = Graph(graph_file_path, file_format="json")  # Load the graph, assuming Graph class supports JSON format

        # Initialize the TurtleBot4 navigator, which handles movement commands
        self.navigator = TurtleBot4Navigator()

        # Set initial position and orientation states for the robot
        self.current_node = starting_point  # Starting point coordinates
        self.current_orientation = orientation  # Starting orientation
        self.current_direction = "STRAIGHTON"  # Default direction to move forward

        # Variables to store the robot's previous state in case of kidnapping
        self.previous_node = None
        self.previous_orientation = None
        self.previous_direction = None
        self.kidnap_status = False  # Flag to check if the robot has been kidnapped

        # Set the robot's initial pose (starting position and orientation)
        self.set_initial_pose(self.current_node, self.current_orientation)

        # Subscribe to the 'command' topic to receive movement commands
        self.command_subscription = self.create_subscription(
            String,  # Message type for the command topic
            'command',  # Name of the topic to subscribe to
            self.direction_callback,  # Callback function to handle received commands
            1  # Queue size of the subscription
        )

        # Subscribe to the 'kidnap_status' topic to monitor if the robot has been kidnapped
        self.kidnap_subscription = self.create_subscription(
            KidnapStatus,  # Message type for kidnap status
            'kidnap_status',  # Name of the topic to subscribe to
            self.kidnap_callback,  # Callback function to handle kidnapping status updates
            qos_profile  # Apply the QoS profile defined above
        )

        # Publisher to send discovery messages (used for exploration or updates)
        self.publisher = self.create_publisher(String, 'discovery', 1)

        # Start navigating from the initial position, orientation, and direction
        self.perform_navigation(self.current_node, self.current_orientation, self.current_direction)

    def perform_navigation(self, current_point, orientation, direction):
        """
        Perform navigation to the next node based on the current point, orientation, and direction.

        Args:
            current_point (dict): Current position coordinates (x, y).
            orientation (str): Current orientation (e.g., 'NORTH', 'EAST').
            direction (str): Direction to move in (e.g., 'STRAIGHTON').
        """
        # Find the nearest node in the graph to the current point
        nearest_node = self.graph.find_nearest_point(current_point)
        self.get_logger().info(f"Starting from node: {nearest_node}")  # Log the starting node

        # Get the next node and orientation based on current state and direction
        next_node, next_orientation = self.graph.get_next_node_and_orientation(
            orientation, direction, nearest_node
        )
        self.get_logger().info(f"Navigating to: {next_node}")  # Log the destination node
        self.get_logger().info(f"With direction: {direction}")  # Log the movement direction

        # Navigate to the next node using the desired orientation
        self.navigate_to_node(next_node, self.orientation_conversion(next_orientation))

        # Update the robot's current state with the new node and orientation
        self.current_node = next_node
        self.current_orientation = next_orientation
        # Save previous state information in case of kidnapping
        self.previous_node = current_point
        self.previous_orientation = orientation
        self.previous_direction = direction

    # Additional methods (e.g., direction_callback, kidnap_callback) would be implemented here

def main(args=None):
    """
    Main function to initialize and run the navigation node.
    """
    # Initialize ROS2 with any provided command-line arguments
    rclpy.init(args=args)

    # Set up command-line argument parser for starting position and orientation
    parser = argparse.ArgumentParser(description='Navigation Node')
    parser.add_argument('--x', type=float, required=True, help='Starting x coordinate')  # X coordinate of start position
    parser.add_argument('--y', type=float, required=True, help='Starting y coordinate')  # Y coordinate of start position
    parser.add_argument('--orientation', type=str, required=True, choices=['NORTH', 'EAST', 'SOUTH', 'WEST'], help='Starting orientation')  # Orientation
    args = parser.parse_args()  # Parse command-line arguments

    # Define the starting point and orientation based on parsed arguments
    starting_point = {'x': args.x, 'y': args.y}
    orientation = args.orientation

    # Determine the absolute path to the current script's directory
    package_path = os.path.dirname(os.path.abspath(__file__))

    # Define the path to the JSON graph file within the project structure
    graph_path = os.path.join(package_path, 'map', 'diem_map_topological', 'skeletonized_map_graph.json')

    # Instantiate the navigation node with the specified graph file, starting point, and orientation
    navigation_node = NavigationNode(graph_path, starting_point, orientation)

    # Run the navigation node
    rclpy.spin(navigation_node)

    # Clean up and shut down the node once stopped
    navigation_node.destroy_node()
    rclpy.shutdown()

# Entry point to run the script
if __name__ == '__main__':
    main()
