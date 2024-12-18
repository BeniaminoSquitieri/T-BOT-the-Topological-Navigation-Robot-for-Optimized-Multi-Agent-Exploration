# master_navigation_node.py

#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating a ROS 2 node in Python
from std_msgs.msg import String  # Standard ROS message type for transmitting string data
import json  # For encoding/decoding JSON messages (graph data, statuses, positions)
import math  # For mathematical operations (e.g., pi, trigonometric functions)
import os  # For filesystem operations (e.g., checking if a file exists)
import networkx as nx  # For creating and handling complex graphs (nodes, edges, attributes)

# Import custom callback functions from the separate module
from .master_callbacks import MasterCallbacks

class MasterNavigationNode(Node, MasterCallbacks):
    """
    A ROS 2 node that acts as the master in a multi-robot navigation system.
    This node manages a fleet of robots, assigns navigation tasks, and monitors their status.

    Responsibilities:
    - Load a navigation graph and distribute it to slaves.
    - Register slaves and track their states.
    - Assign waypoints to robots by partitioning the graph.
    - Handle navigation feedback from robots.
    - Periodically check and manage the state of slaves, removing inactive ones.
    """

    def __init__(self):
        """
        Initialize the MasterNavigationNode.
        This includes setting up parameters, loading the navigation graph, and creating ROS publishers, subscribers, and timers.
        """
        # Initialize the ROS node with the name 'master_navigation_node'
        super().__init__('master_navigation_node')

        # Declare ROS parameters with default values
        self.declare_parameter(
            'graph_path',
            '/home/beniamino/turtlebot4/diem_turtlebot_ws/src/fleet_turtlebot4_navigation/map/navigation_hardware_limitation.json'
        )
        self.declare_parameter('check_interval', 2.0)  # Default interval for slave status checks
        self.declare_parameter('timeout', 150.0)  # Default timeout for detecting inactive slaves

        # Retrieve parameter values
        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Validate the existence of the navigation graph file
        if not os.path.exists(self.graph_path):
            self.get_logger().error(f"Graph file not found at {self.graph_path}")
            raise FileNotFoundError(f"Graph file not found at {self.graph_path}")

        # # Load the navigation graph from the specified file
        # self.full_graph = load_full_graph(self.graph_path)

        # Create publishers, subscribers, and timers
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)
        self.graph_timer = self.create_timer(5.0, self.publish_navigation_graph)

        # Subscribers to manage communication with slaves
        self.slave_registration_subscriber = self.create_subscription(
            String, '/slave_registration', self.slave_registration_callback, 10
        )
        self.initial_position_subscriber = self.create_subscription(
            String, '/slave_initial_positions', self.initial_position_callback, 10
        )
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )

        # Publisher for sending master heartbeat
        self.heartbeat_publisher = self.create_publisher(String, '/master_heartbeat', 1)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Timer for periodic slave checks and managing waiting tasks
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Initialize state tracking
        self.slaves = {}  # Dictionary to track active slaves and their states
        self.partitioning_done = False  # Flag for graph partitioning status
        self.occupied_edges = set()  # Set to track occupied edges in the graph

        # Log initialization
        self.get_logger().info("Master node initialized with edge-based occupation control.")

    # All callback methods have been moved to MasterCallbacks, so no need to redefine them here.


def main(args=None):
    """
    Main entry point for the node.

    This function initializes the ROS 2 framework, creates an instance of the `MasterNavigationNode`,
    and enters a spinning loop to handle callbacks until the node is terminated.
    """

    # Step 1: Initialize the ROS 2 client library with optional command-line arguments.
    # This sets up communication with the ROS 2 network.
    rclpy.init(args=args)

    # Step 2: Create an instance of the MasterNavigationNode.
    # This node is responsible for coordinating the navigation tasks across a fleet of robots.
    node = MasterNavigationNode()

    try:
        # Step 3: Start spinning the node.
        # Spinning keeps the node active and continuously listens for incoming messages
        # and executes callbacks as needed.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Step 4: Handle keyboard interrupt gracefully.
        # This ensures that the node exits cleanly when the user presses Ctrl+C.
        pass

    # Step 5: Perform cleanup before shutting down.
    # Destroy the node instance and shut down the ROS 2 framework.
    node.destroy_node()
    rclpy.shutdown()

# The main function is executed when the script is run directly.
# This ensures the program is run as a standalone script and not imported as a module.
if __name__ == '__main__':
    main()
