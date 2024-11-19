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

        # Iterate over the nodes in the JSON data
        for node in data['nodes']:
            label = node['label']  # Extract the node label
            x = node['x']          # Extract the X-coordinate of the node
            y = node['y']          # Extract the Y-coordinate of the node
            # Add the node to the NetworkX graph with its position as attributes
            self.nx_graph.add_node(label, x=x, y=y)

        # Iterate over the edges in the JSON data
        for edge in data['edges']:
            u = edge['from']  # Source node of the edge
            v = edge['to']    # Destination node of the edge
            # Retrieve coordinates of the source and destination nodes
            x1, y1 = self.nx_graph.nodes[u]['x'], self.nx_graph.nodes[u]['y']
            x2, y2 = self.nx_graph.nodes[v]['x'], self.nx_graph.nodes[v]['y']
            # Calculate the Euclidean distance between the two nodes
            weight = math.hypot(x2 - x1, y2 - y1)
            # Add the edge to the graph with the calculated weight
            self.nx_graph.add_edge(u, v, weight=weight)


    def partition_graph(self):
        """
        Partitions the full graph into subgraphs, assigning one to each robot using Kernighan-Lin algorithm.
        """
        # Initialize partitions with all nodes as a single partition
        partitions = [set(self.nx_graph.nodes())]

        # Perform recursive bisection until we have the desired number of partitions
        while len(partitions) < self.num_robots:
            new_partitions = []  # Temporary list to store newly created partitions
            for part in partitions:
                # If enough partitions exist to meet the number of robots, skip further splitting
                if len(partitions) >= self.num_robots:
                    new_partitions.append(part)
                    continue
                # Create a subgraph for the current partition
                subgraph = self.nx_graph.subgraph(part)
                # Split the subgraph into two parts using the Kernighan-Lin algorithm
                parts = nx.algorithms.community.kernighan_lin_bisection(subgraph, weight='weight')
                # Add the resulting partitions to the new partitions list
                new_partitions.extend(parts)
            # Update the partitions list with the newly created partitions
            partitions = new_partitions

        # Assign nodes to the `node_partitions` attribute for future reference
        self.node_partitions = partitions

        # Log an error if the number of partitions doesn't match the number of robots
        if len(partitions) != self.num_robots:
            self.get_logger().error(f"Partitioning resulted in {len(partitions)} partitions, expected {self.num_robots}.")

        # Assign the nodes of the current robot's partition to `subgraph_nodes`
        self.subgraph_nodes = list(self.node_partitions[self.robot_id])
        self.get_logger().info(f"[{self.robot_namespace}] Assigned nodes: {self.subgraph_nodes}")

        # Create a subgraph containing only the nodes assigned to this robot
        self.subgraph = self.nx_graph.subgraph(self.subgraph_nodes).copy()


    def target_callback(self, msg):
        """
        Callback function to receive updates on targeted nodes from other robots.

        Args:
            msg (String): Message containing the target node in the format "robot_namespace,node_label".
        """
        # Parse the incoming message to extract the robot namespace and the node label.
        # The expected format of the message is "robot_namespace,node_label".
        robot_ns, node_label = msg.data.split(',')

        # Check if the message is from another robot.
        # Ignore messages originating from this robot's namespace to avoid self-conflicts.
        if robot_ns != self.robot_namespace:
            # Acquire the node lock to ensure thread-safe access to the 'targeted_nodes' set.
            with self.node_lock:
                # Add the node label to the set of targeted nodes.
                # This keeps track of nodes that are currently being targeted by other robots.
                self.targeted_nodes.add(node_label)

                # Log a message to indicate that another robot has targeted this node.
                # This helps in debugging and monitoring inter-robot coordination.
                self.get_logger().info(
                    f"[{self.robot_namespace}] Node {node_label} is targeted by {robot_ns}."
                )


    def announce_target_node(self, node_label):
        """
        Publishes a message announcing the target node.

        Args:
            node_label (str): Label of the node to target.
        """
        # Create a new message object of type std_msgs.msg.String
        msg = String()

        # Format the message data to include the robot's namespace and the target node's label
        # Example format: "robot_namespace,node_label" (e.g., "robot_1,node_5")
        msg.data = f"{self.robot_namespace},{node_label}"

        # Publish the message to the 'target_nodes' topic
        # This informs other robots about the node being targeted by this robot
        self.target_publisher.publish(msg)

        # Log the announcement for debugging and monitoring purposes
        self.get_logger().info(f"[{self.robot_namespace}] Announced target node {node_label}.")


    def find_nearest_node_in_subgraph(self, point):
        """
        Finds the nearest node in the assigned subgraph to the given point.

        Args:
            point (dict): A point with 'x' and 'y' keys representing coordinates.

        Returns:
            str: The label of the nearest node, or None if no nodes are found in the subgraph.
        """
        # Initialize the minimum distance to infinity
        # This acts as the comparison baseline to find the closest node
        min_distance = float('inf')

        # Variable to store the label of the nearest node
        nearest_node_label = None

        # Extract the 'x' and 'y' coordinates of the starting point
        x0, y0 = point['x'], point['y']

        # Iterate over all nodes in the assigned subgraph
        for node_label in self.subgraph.nodes():
            # Get the node's attributes (e.g., coordinates) from the main graph
            data = self.nx_graph.nodes[node_label]

            # Extract the 'x' and 'y' coordinates of the current node
            x1, y1 = data['x'], data['y']

            # Compute the Euclidean distance between the given point and the current node
            distance = math.hypot(x1 - x0, y1 - y0)

            # Check if this node is closer than the previously found nearest node
            if distance < min_distance:
                # Update the minimum distance
                min_distance = distance

                # Update the label of the nearest node
                nearest_node_label = node_label

        # Return the label of the nearest node, or None if no nodes were found
        return nearest_node_label


    def set_initial_pose(self, point):
        """
        Sets the initial pose of the robot to the given point.

        Args:
            point (dict): A point with 'x' and 'y' keys.
        """
        # Extract the 'x' and 'y' coordinates from the input dictionary `point`
        x, y = point['x'], point['y']  # Coordinates for the initial pose

        # Create a PoseStamped object for the robot's initial position and orientation
        # - `[x, y]`: The position of the robot in the world
        # - `0.0`: The initial orientation angle in radians (defaulted to facing forward)
        initial_pose = self.navigator.getPoseStamped([x, y], 0.0)

        # Set the robot's initial pose using the TurtleBot4 navigator
        # This ensures that the robot's starting location is correctly initialized
        self.navigator.setInitialPose(initial_pose)

        # Wait for a short period to allow the initial pose to be properly set
        # This ensures the pose is registered by the robot and the navigation system
        time.sleep(1.0)


    def navigate(self):
        """
        Main navigation loop where the robot selects nodes to visit while communicating targets.
        """
        # Initialize the navigation by setting the current node to the starting node
        current_node_label = self.current_node_label  # Start from the initial node

        # Main navigation loop; runs as long as the ROS2 system is operational
        while rclpy.ok():
            # Use the `select_next_node` method to determine the next node to visit
            next_node = self.select_next_node(current_node_label)

            # If no valid next node is found, terminate navigation
            if not next_node:
                # Log that the robot has completed its navigation task
                self.get_logger().info(
                    f"[{self.robot_namespace}] No more nodes to visit. Navigation completed."
                )
                break  # Exit the loop as there are no more nodes to visit

            # Announce the target node to other robots via a ROS2 topic
            # This prevents conflicts by informing others that this node is being visited
            self.announce_target_node(next_node)

            # Navigate to the selected node
            # This involves moving the robot to the coordinates of the `next_node`
            self.navigate_to_node(next_node)

            # Update the current node to the one the robot has just visited
            current_node_label = next_node

            # Once the robot reaches the node, remove it from the list of targeted nodes
            # This allows other robots to potentially visit this node in the future if needed
            with self.node_lock:
                self.targeted_nodes.discard(next_node)

        # After exiting the loop, log that the navigation process has ended
        self.get_logger().info(
            f"[{self.robot_namespace}] Exiting navigation loop."
        )


    def select_next_node(self, current_node):
        """
        Selects the next node to move to from the current node within the assigned subgraph.

        Args:
            current_node (str): Label of the current node.

        Returns:
            str: The next node to move to, or None if no nodes are available.
        """
        # Acquire the lock to ensure thread-safe access to the shared node sets
        # This prevents race conditions when modifying `visited_nodes` and `targeted_nodes`
        with self.node_lock:
            # Identify all neighboring nodes of the current node in the subgraph
            # Only include nodes that:
            # 1. Are not already visited (to avoid revisiting the same node)
            # 2. Are not currently targeted by other robots (to avoid conflicts)
            neighbors = [
                v for v in self.subgraph.neighbors(current_node)  # Iterate over adjacent nodes
                if v not in self.visited_nodes and v not in self.targeted_nodes
            ]

            # If there are unvisited and untargeted neighboring nodes
            if neighbors:
                # Select the neighbor with the smallest edge weight (shortest distance)
                # Use the `weight` attribute of the edge to determine the distance
                next_node = min(
                    neighbors, 
                    key=lambda v: self.subgraph[current_node][v]['weight']
                )

                # Mark the selected node as visited
                # This prevents it from being revisited by this robot
                self.visited_nodes.add(next_node)

                # Mark the selected node as targeted
                # This prevents it from being targeted by other robots
                self.targeted_nodes.add(next_node)

                # Log the selection of the next node for navigation
                self.get_logger().info(
                    f"[{self.robot_namespace}] Selected next node {next_node} to move to."
                )

                # Return the label of the selected next node
                return next_node
            else:
                # If there are no valid neighbors to visit
                # This implies the robot has completed all accessible nodes
                return None


    def navigate_to_node(self, node_label):
        """
        Navigates to the specified node.

        Args:
            node_label (str): Identifier of the node to navigate to.
        """
        # Get the coordinates of the target node from the graph
        # The node's label serves as a key to access its stored x and y positions
        x, y = self.nx_graph.nodes[node_label]['x'], self.nx_graph.nodes[node_label]['y']

        # Retrieve the current pose of the robot from the navigator
        # This is used to calculate the orientation towards the target node
        current_pose = self.navigator.getCurrentPose()

        if current_pose:
            # Extract the robot's current x and y coordinates
            x0 = current_pose.pose.position.x
            y0 = current_pose.pose.position.y
            # Calculate the orientation (angle) from the current position to the target node
            orientation = math.atan2(y - y0, x - x0)
        else:
            # If the current pose is unavailable, set a default orientation to 0 radians
            orientation = 0.0

        # Create a goal pose for the robot
        # This includes the target coordinates and the calculated orientation
        goal_pose = self.navigator.getPoseStamped([x, y], orientation)

        # Instruct the navigator to begin moving towards the goal pose
        self.navigator.startToPose(goal_pose)
        # Log the navigation action, specifying the target node and its coordinates
        self.get_logger().info(f"[{self.robot_namespace}] Navigating to node {node_label} at ({x}, {y}).")

        # Wait until the navigation task is complete
        # Continuously check the navigator's task status in a loop
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)  # Pause briefly to avoid excessive CPU usage in the loop

        # Wait an additional second after task completion
        # This provides a buffer before proceeding to the next task
        time.sleep(1.0)


    def destroy_node(self):
        """
        Overrides the destroy_node method to ensure the navigation thread is properly closed.
        """
        # Log the shutdown process for the current navigation node
        self.get_logger().info(f"[{self.robot_namespace}] Shutting down navigation node.")

        # Wait for the navigation thread to complete execution
        # This ensures no unfinished navigation logic persists after shutdown
        self.navigation_thread.join()

        # Call the superclass method to perform standard node destruction
        # This releases resources and stops all ongoing ROS2 processes associated with the node
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize and run the navigation node.
    """
    # Initialize the ROS2 Python client library
    # This sets up ROS2 communication infrastructure for the node
    rclpy.init(args=args)

    # Create an argument parser to handle command-line inputs
    parser = argparse.ArgumentParser(description='Robot Navigation Node with Internal Graph Partitioning')

    # Define the required arguments for the robot
    parser.add_argument('--robot_namespace', type=str, required=True, 
                        help='Unique namespace of the robot (e.g., "robot_1"). This distinguishes each robot.')
    parser.add_argument('--graph_path', type=str, required=True, 
                        help='Path to the JSON file containing the full graph structure.')
    parser.add_argument('--robot_id', type=int, required=True, 
                        help='Unique ID of the robot (e.g., 0 for the first robot, 1 for the second, etc.).')
    parser.add_argument('--num_robots', type=int, required=True, 
                        help='Total number of robots in the system.')
    parser.add_argument('--start_x', type=float, required=True, 
                        help='Starting x coordinate for the robot. This determines its initial position on the graph.')
    parser.add_argument('--start_y', type=float, required=True, 
                        help='Starting y coordinate for the robot. This also determines its initial position on the graph.')

    # Parse the command-line arguments into a structured object
    args = parser.parse_args()

    # Create a dictionary to represent the starting point of the robot
    # This contains the x and y coordinates passed from the command line
    starting_point = {'x': args.start_x, 'y': args.start_y}

    # Instantiate the navigation node for the robot
    # Pass all the required arguments, including the robot's namespace, graph path, ID, total robots, and starting point
    navigation_node = RobotNavigationNode(
        args.robot_namespace,  # Unique namespace for the robot
        args.graph_path,       # Path to the JSON graph file
        args.robot_id,         # Robot's unique ID
        args.num_robots,       # Total number of robots
        starting_point         # Starting point coordinates
    )

    try:
        # Keep the ROS2 node active to process incoming and outgoing messages
        # This allows the node to handle callbacks, such as receiving target updates or processing navigation commands
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        # Gracefully handle a keyboard interrupt (Ctrl+C) to stop the program
        pass
    finally:
        # Clean up resources before shutting down
        # Destroy the navigation node to ensure all threads and processes terminate properly
        navigation_node.destroy_node()

        # Shut down the ROS2 client library
        # This ensures the ROS2 communication infrastructure is properly closed
        rclpy.shutdown()

# Entry point for the script
# This ensures the main function is executed only when the script is run directly
if __name__ == '__main__':
    main()
