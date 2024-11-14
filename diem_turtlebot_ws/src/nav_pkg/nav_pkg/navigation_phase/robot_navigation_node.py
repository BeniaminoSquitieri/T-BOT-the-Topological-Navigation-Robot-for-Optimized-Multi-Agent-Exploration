# robot_navigation_node.py

import os
import argparse
import time
import threading
import json
import math
import networkx as nx
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class RobotNavigationNode(Node):
    """
    ROS2 Node for multi-robot navigation using internal graph partitioning and node target communication.
    Each robot partitions the full graph and navigates its assigned subgraph, communicating target nodes to avoid conflicts.

    Attributes:
        robot_namespace (str): Unique namespace for the robot.
        robot_id (int): Unique identifier for the robot (e.g., 0, 1, 2).
        num_robots (int): Total number of robots in the system.
        navigator (TurtleBot4Navigator): Instance for controlling the robot.
        nx_graph (networkx.Graph): Undirected graph representation using NetworkX.
        subgraph_nodes (list): List of nodes assigned to this robot after partitioning.
        visited_nodes (set): Set of nodes that have been visited.
        targeted_nodes (set): Set of nodes currently targeted by any robot.
        node_lock (threading.Lock): Lock to synchronize access to node sets.
        current_node_label (str): Label of the current node where the robot is located.
        navigation_thread (threading.Thread): Thread running the navigation logic.
    """

    def __init__(self, robot_namespace, graph_file_path, robot_id, num_robots, starting_point):
        """
        Initializes the navigation node for a single robot.

        Args:
            robot_namespace (str): Unique namespace for the robot.
            graph_file_path (str): Path to the JSON file containing the full graph.
            robot_id (int): Unique identifier for the robot.
            num_robots (int): Total number of robots.
            starting_point (dict): Starting point with 'x' and 'y' keys representing coordinates.
        """
        # Initialize the Node with a unique name based on the robot namespace
        super().__init__(robot_namespace + '_navigation_node')

        self.robot_namespace = robot_namespace
        self.robot_id = robot_id
        self.num_robots = num_robots

        # Create an instance of the navigator for the robot, specifying the namespace
        self.navigator = TurtleBot4Navigator(namespace='/' + self.robot_namespace)

        # Load the full graph from the JSON file
        self.load_graph(graph_file_path)

        # Partition the graph and get the nodes assigned to this robot
        self.partition_graph()

        # Initialize sets for visited and targeted nodes
        self.visited_nodes = set()
        self.targeted_nodes = set()
        self.node_lock = threading.Lock()  # Lock to ensure thread-safe operations on node sets

        # Subscribe to the shared topic to receive updates on targeted nodes
        self.target_subscriber = self.create_subscription(
            String,
            'target_nodes',  # Topic name
            self.target_callback,  # Callback function
            10  # QoS depth
        )

        # Publisher to announce targeted nodes
        self.target_publisher = self.create_publisher(
            String,
            'target_nodes',  # Topic name
            10  # QoS depth
        )

        # Find the nearest node to the starting point within the assigned subgraph
        self.current_node_label = self.find_nearest_node_in_subgraph(starting_point)
        if self.current_node_label is None:
            self.get_logger().error(f"[{self.robot_namespace}] No starting node found in assigned subgraph.")
            return

        self.get_logger().info(f"[{self.robot_namespace}] Starting at node: {self.current_node_label}")

        # Set the robot's initial pose to the starting point
        self.set_initial_pose(starting_point)

        # Start the navigation in a separate thread to allow ROS2 callbacks to be processed
        self.navigation_thread = threading.Thread(target=self.navigate)
        self.navigation_thread.start()

    def load_graph(self, graph_file_path):
        """
        Loads the full graph from a JSON file and builds a NetworkX Graph.

        Args:
            graph_file_path (str): Path to the JSON file containing the graph structure.
        """
        # Open and read the JSON file containing the graph data
        with open(graph_file_path, 'r') as f:
            data = json.load(f)

        # Create an undirected graph using NetworkX
        self.nx_graph = nx.Graph()

        # Add nodes to the graph with their coordinates as attributes
        for node in data['nodes']:
            label = node['label']  # Node label
            x = node['x']  # X-coordinate
            y = node['y']  # Y-coordinate
            self.nx_graph.add_node(label, x=x, y=y)  # Add node with position attributes

        # Add edges to the graph with calculated weights (Euclidean distance)
        for edge in data['edges']:
            u = edge['from']  # Source node
            v = edge['to']    # Destination node
            x1, y1 = self.nx_graph.nodes[u]['x'], self.nx_graph.nodes[u]['y']  # Coordinates of u
            x2, y2 = self.nx_graph.nodes[v]['x'], self.nx_graph.nodes[v]['y']  # Coordinates of v
            weight = math.hypot(x2 - x1, y2 - y1)  # Calculate Euclidean distance
            self.nx_graph.add_edge(u, v, weight=weight)  # Add edge with weight

    def partition_graph(self):
        """
        Partitions the full graph into subgraphs, assigning one to each robot using Kernighan-Lin algorithm.
        """
        # Initialize partitions with all nodes
        partitions = [set(self.nx_graph.nodes())]

        # Perform recursive bisection until we have the desired number of partitions
        while len(partitions) < self.num_robots:
            new_partitions = []
            for part in partitions:
                if len(new_partitions) + len(partitions) - len(new_partitions) >= self.num_robots:
                    new_partitions.append(part)
                    continue
                # Create subgraph
                subgraph = self.nx_graph.subgraph(part)
                # Partition using Kernighan-Lin algorithm
                parts = nx.algorithms.community.kernighan_lin_bisection(subgraph, weight='weight')
                new_partitions.extend(parts)
            partitions = new_partitions

        # Assign nodes to partitions
        self.node_partitions = partitions

        # Ensure we have the correct number of partitions
        if len(partitions) != self.num_robots:
            self.get_logger().error(f"Partitioning resulted in {len(partitions)} partitions, expected {self.num_robots}.")

        # Get the nodes assigned to this robot
        self.subgraph_nodes = list(self.node_partitions[self.robot_id])
        self.get_logger().info(f"[{self.robot_namespace}] Assigned nodes: {self.subgraph_nodes}")

        # Create subgraph
        self.subgraph = self.nx_graph.subgraph(self.subgraph_nodes).copy()

    def target_callback(self, msg):
        """
        Callback function to receive updates on targeted nodes from other robots.

        Args:
            msg (String): Message containing the target node in the format "robot_namespace,node_label".
        """
        robot_ns, node_label = msg.data.split(',')
        if robot_ns != self.robot_namespace:
            with self.node_lock:
                self.targeted_nodes.add(node_label)
                self.get_logger().info(f"[{self.robot_namespace}] Node {node_label} is targeted by {robot_ns}.")

    def announce_target_node(self, node_label):
        """
        Publishes a message announcing the target node.

        Args:
            node_label (str): Label of the node to target.
        """
        msg = String()
        msg.data = f"{self.robot_namespace},{node_label}"  # Format the message as "robot_namespace,node_label"
        self.target_publisher.publish(msg)  # Publish the message
        self.get_logger().info(f"[{self.robot_namespace}] Announced target node {node_label}.")

    def find_nearest_node_in_subgraph(self, point):
        """
        Finds the nearest node in the assigned subgraph to the given point.

        Args:
            point (dict): A point with 'x' and 'y' keys.

        Returns:
            str: The label of the nearest node, or None if not found.
        """
        min_distance = float('inf')
        nearest_node_label = None
        x0, y0 = point['x'], point['y']  # Coordinates of the starting point
        for node_label in self.subgraph.nodes():
            data = self.nx_graph.nodes[node_label]
            x1, y1 = data['x'], data['y']  # Coordinates of the graph node
            distance = math.hypot(x1 - x0, y1 - y0)  # Calculate Euclidean distance
            if distance < min_distance:
                min_distance = distance
                nearest_node_label = node_label  # Update nearest node
        return nearest_node_label

    def set_initial_pose(self, point):
        """
        Sets the initial pose of the robot to the given point.

        Args:
            point (dict): A point with 'x' and 'y' keys.
        """
        x, y = point['x'], point['y']  # Coordinates for initial pose
        initial_pose = self.navigator.getPoseStamped([x, y], 0.0)  # Create PoseStamped message
        self.navigator.setInitialPose(initial_pose)  # Set initial pose in the navigator
        time.sleep(1.0)  # Wait to ensure the pose is set

    def navigate(self):
        """
        Main navigation loop where the robot selects nodes to visit while communicating targets.
        """
        current_node_label = self.current_node_label  # Start from the initial node

        while rclpy.ok():
            # Select the next node to move to
            next_node = self.select_next_node(current_node_label)
            if not next_node:
                # No available nodes, navigation is complete
                self.get_logger().info(f"[{self.robot_namespace}] No more nodes to visit. Navigation completed.")
                break

            # Announce the target node
            self.announce_target_node(next_node)

            # Navigate to the next node
            self.navigate_to_node(next_node)

            # Update the current node
            current_node_label = next_node

            # Remove the node from targeted nodes
            with self.node_lock:
                self.targeted_nodes.discard(next_node)

        self.get_logger().info(f"[{self.robot_namespace}] Exiting navigation loop.")

    def select_next_node(self, current_node):
        """
        Selects the next node to move to from the current node within the assigned subgraph.

        Args:
            current_node (str): Label of the current node.

        Returns:
            str: The next node to move to, or None if no nodes are available.
        """
        with self.node_lock:
            # Get all adjacent nodes within the subgraph not yet visited and not targeted
            neighbors = [
                v for v in self.subgraph.neighbors(current_node)
                if v not in self.visited_nodes and v not in self.targeted_nodes
            ]
            if neighbors:
                # Select the neighbor with minimum weight (shortest distance)
                next_node = min(neighbors, key=lambda v: self.subgraph[current_node][v]['weight'])
                # Mark the node as visited and targeted
                self.visited_nodes.add(next_node)
                self.targeted_nodes.add(next_node)
                self.get_logger().info(f"[{self.robot_namespace}] Selected next node {next_node} to move to.")
                return next_node
            else:
                # No unvisited neighbors available
                return None

    def navigate_to_node(self, node_label):
        """
        Navigates to the specified node.

        Args:
            node_label (str): Identifier of the node to navigate to.
        """
        # Get the coordinates of the target node
        x, y = self.nx_graph.nodes[node_label]['x'], self.nx_graph.nodes[node_label]['y']

        # Get the current pose of the robot to calculate orientation
        current_pose = self.navigator.getCurrentPose()
        if current_pose:
            x0 = current_pose.pose.position.x
            y0 = current_pose.pose.position.y
            orientation = math.atan2(y - y0, x - x0)  # Calculate orientation towards the target
        else:
            orientation = 0.0  # Default orientation if current pose is unknown

        # Create a goal pose for the navigator
        goal_pose = self.navigator.getPoseStamped([x, y], orientation)

        # Start navigation to the goal pose
        self.navigator.startToPose(goal_pose)
        self.get_logger().info(f"[{self.robot_namespace}] Navigating to node {node_label} at ({x}, {y}).")

        # Wait until the navigation task is complete
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)  # Sleep to prevent busy-waiting

        time.sleep(1.0)  # Wait briefly before proceeding

    def destroy_node(self):
        """
        Overrides the destroy_node method to ensure the navigation thread is properly closed.
        """
        self.get_logger().info(f"[{self.robot_namespace}] Shutting down navigation node.")
        self.navigation_thread.join()  # Wait for the navigation thread to finish
        super().destroy_node()  # Call the superclass method to properly destroy the node

def main(args=None):
    """
    Main function to initialize and run the navigation node.
    """
    rclpy.init(args=args)  # Initialize the ROS2 Python client library

    parser = argparse.ArgumentParser(description='Robot Navigation Node with Internal Graph Partitioning')
    parser.add_argument('--robot_namespace', type=str, required=True, help='Unique namespace of the robot')
    parser.add_argument('--graph_path', type=str, required=True, help='Path to the full graph JSON file')
    parser.add_argument('--robot_id', type=int, required=True, help='Unique ID of the robot (e.g., 0, 1, 2)')
    parser.add_argument('--num_robots', type=int, required=True, help='Total number of robots')
    parser.add_argument('--start_x', type=float, required=True, help='Starting x coordinate')
    parser.add_argument('--start_y', type=float, required=True, help='Starting y coordinate')
    args = parser.parse_args()

    # Create a dictionary for the starting point coordinates
    starting_point = {'x': args.start_x, 'y': args.start_y}

    # Instantiate the navigation node with the provided arguments
    navigation_node = RobotNavigationNode(
        args.robot_namespace,
        args.graph_path,
        args.robot_id,
        args.num_robots,
        starting_point
    )

    try:
        # Keep the node running to handle callbacks
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        # Handle shutdown gracefully on Ctrl+C
        pass
    finally:
        # Destroy the node and shutdown rclpy
        navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
