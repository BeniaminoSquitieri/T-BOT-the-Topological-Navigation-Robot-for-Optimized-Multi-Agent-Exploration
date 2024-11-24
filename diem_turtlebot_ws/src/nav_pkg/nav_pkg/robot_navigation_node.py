import os
import argparse
import time
import json
import math
import random
import networkx as nx
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class RobotNavigationNode(Node):
    """
    ROS2 Node for robot navigation using the Chinese Postman Problem (CPP) over a shared graph.
    Implements deadlock handling through timeouts and retries, ensuring safe and coordinated navigation.
    """

    def __init__(self, robot_namespace, graph_file_path, starting_point):
        """
        Initializes the navigation node for a single robot.

        Args:
            robot_namespace (str): Unique namespace for the robot.
            graph_file_path (str): Path to the JSON file containing the graph.
            starting_point (dict): Starting coordinates with 'x' and 'y' keys.
        """
        super().__init__(robot_namespace + '_navigation_node')  # Initialize ROS2 Node with a unique name

        self.robot_namespace = robot_namespace  # Store the robot's namespace

        self.navigator = TurtleBot4Navigator()  # Create a navigator for controlling the robot's movements

        # Load the graph structure from the specified JSON file
        self.load_graph(graph_file_path)

        # Print the details of the assigned subgraph for debugging
        self.print_subgraph()

        # Find the nearest graph node to the starting point
        self.current_node_label = self.find_nearest_node(starting_point)
        if self.current_node_label is None:
            self.get_logger().error(f"[{self.robot_namespace}] No starting node found in graph.")
            return

        self.get_logger().info(f"[{self.robot_namespace}] Starting at node: {self.current_node_label}")

        # Set the robot's initial pose in the environment
        self.set_initial_pose(starting_point)

        # Compute the Chinese Postman Problem (CPP) route for the robot
        self.compute_cpp_route()

        if not self.cpp_route:  # If no route could be computed, log an error
            self.get_logger().error(f"[{self.robot_namespace}] Unable to compute CPP route.")
            return

        self.route_index = 0  # Initialize the index for the CPP route
        self.navigation_in_progress = False  # Flag to indicate if navigation is currently in progress

        self.locked_nodes = set()  # Set to keep track of locked (targeted) nodes

        # Subscription to a shared topic for managing node locks
        self.lock_subscriber = self.create_subscription(
            String, 'node_locks', self.lock_callback, 10
        )

        # Publisher to announce node locks or unlocks
        self.lock_publisher = self.create_publisher(
            String, 'node_locks', 10
        )

        # Timer to periodically check and manage navigation tasks
        self.timer_period = 1.0  # Time interval in seconds
        self.timer = self.create_timer(self.timer_period, self.navigate)

    def load_graph(self, graph_file_path):
        """
        Loads the graph from a JSON file and creates a directed NetworkX graph.

        Args:
            graph_file_path (str): Path to the JSON file containing the graph structure.
        """
        with open(graph_file_path, 'r') as f:
            data = json.load(f)

        self.nx_graph = nx.DiGraph()  # Initialize a directed graph

        # Add nodes with their attributes (coordinates)
        for node in data['nodes']:
            label = node['label']
            x = node['x']
            y = node['y']
            self.nx_graph.add_node(label, x=x, y=y)

        # Add edges with their weights (distances)
        for edge in data['edges']:
            u = edge['from']
            v = edge['to']
            weight = edge.get('weight', 1.0)  # Default weight is 1.0
            self.nx_graph.add_edge(u, v, weight=weight)

    def print_subgraph(self):
        """
        Logs the nodes and edges of the assigned subgraph for debugging.
        """
        self.get_logger().info(f"[{self.robot_namespace}] Assigned subgraph nodes:")
        for node in self.nx_graph.nodes(data=True):
            self.get_logger().info(f"Node {node[0]}: {node[1]}")

        self.get_logger().info(f"[{self.robot_namespace}] Assigned subgraph edges:")
        for edge in self.nx_graph.edges(data=True):
            self.get_logger().info(f"Edge from {edge[0]} to {edge[1]}: {edge[2]}")

    def find_nearest_node(self, point):
        """
        Finds the nearest graph node to the given point.

        Args:
            point (dict): Coordinates with 'x' and 'y' keys.

        Returns:
            str: Label of the nearest node.
        """
        min_distance = float('inf')  # Start with an infinite minimum distance
        nearest_node_label = None
        x0, y0 = point['x'], point['y']  # Extract coordinates of the point
        for node_label in self.nx_graph.nodes():
            data = self.nx_graph.nodes[node_label]
            x1, y1 = data['x'], data['y']  # Coordinates of the current node
            distance = math.hypot(x1 - x0, y1 - y0)  # Calculate Euclidean distance
            if distance < min_distance:  # Update nearest node if closer
                min_distance = distance
                nearest_node_label = node_label
        return nearest_node_label

    def set_initial_pose(self, point):
        """
        Sets the robot's initial pose in the environment.

        Args:
            point (dict): Coordinates with 'x' and 'y' keys.
        """
        x, y = point['x'], point['y']  # Extract coordinates
        initial_pose = self.navigator.getPoseStamped([x, y], 0.0)  # Create initial pose
        self.navigator.setInitialPose(initial_pose)  # Apply the initial pose
        time.sleep(1.0)  # Wait for the pose to be set

    def compute_cpp_route(self):
        """
        Computes the Chinese Postman Problem (CPP) route for the robot.
        """
        # Create an Eulerian version of the graph
        self.eulerian_graph = self.eulerize(self.nx_graph)

        # Attempt to compute the Eulerian circuit from the current node
        self.cpp_route = []
        try:
            euler_circuit = list(nx.eulerian_circuit(self.eulerian_graph, source=self.current_node_label))
            for u, v in euler_circuit:
                self.cpp_route.append((u, v))  # Store each edge in the CPP route
            self.get_logger().info(f"[{self.robot_namespace}] Computed CPP route: {self.cpp_route}")
        except Exception as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to compute Eulerian circuit: {str(e)}")
            self.cpp_route = None

    def eulerize(self, G):
        """
        Ensures the graph is Eulerian by adding edges between odd-degree nodes.

        Args:
            G (nx.DiGraph): The original graph.

        Returns:
            nx.MultiDiGraph: An Eulerian version of the graph.
        """
        if not nx.is_connected(G.to_undirected()):
            raise nx.NetworkXError("Graph is not connected.")

        # Find nodes with odd degree
        odd_degree_nodes = [v for v, d in G.degree() if d % 2 == 1]

        # Calculate shortest paths between odd-degree nodes
        import itertools
        odd_node_pairs = list(itertools.combinations(odd_degree_nodes, 2))
        distances = dict(nx.all_pairs_dijkstra_path_length(G, weight='weight'))

        # Create a complete graph of odd nodes with weights as distances
        complete_graph = nx.Graph()
        for u, v in odd_node_pairs:
            weight = distances[u][v]
            complete_graph.add_edge(u, v, weight=weight)

        # Find minimum weight matching
        matching = nx.algorithms.matching.min_weight_matching(complete_graph, weight='weight')

        # Add matching edges to make the graph Eulerian
        G_eulerized = nx.MultiDiGraph(G)
        for u, v in matching:
            path = nx.shortest_path(G, source=u, target=v, weight='weight')
            for i in range(len(path) - 1):
                edge_data = G.get_edge_data(path[i], path[i + 1])
                weight = edge_data['weight']
                G_eulerized.add_edge(path[i], path[i + 1], weight=weight)

        return G_eulerized

    def navigate(self):
        """
        Manages the robot's navigation through the computed CPP route.
        Handles node locking and deadlock prevention.
        """
        if self.navigation_in_progress:  # Skip if already navigating
            return

        # Check if the route is complete
        if self.route_index >= len(self.cpp_route):
            self.get_logger().info(f"[{self.robot_namespace}] Completed CPP route. Restarting...")
            self.route_index = 0  # Restart the route
            return

        current_node, next_node = self.cpp_route[self.route_index]

        if next_node in self.locked_nodes:
            self.get_logger().info(f"[{self.robot_namespace}] Node {next_node} is locked. Retrying...")
            time.sleep(random.uniform(1.0, 3.0))  # Wait before retrying
            return

        self.locked_nodes.add(next_node)
        self.announce_node_lock(next_node, 'lock')

        if self.route_index > 0:
            prev_node = self.cpp_route[self.route_index - 1][1]
            self.locked_nodes.discard(prev_node)
            self.announce_node_lock(prev_node, 'unlock')

        self.navigation_in_progress = True
        self.navigate_to_node(current_node, next_node)
        self.current_node_label = next_node
        self.route_index += 1
        self.navigation_in_progress = False

    def lock_callback(self, msg):
        """
        Handles messages about node locks from other robots.

        Args:
            msg (String): Message in the format "robot_namespace,node_label,action".
        """
        robot_ns, node_label, action = msg.data.split(',')

        if robot_ns != self.robot_namespace:  # Ignore messages from itself
            if action == 'lock':
                self.locked_nodes.add(node_label)
                self.get_logger().info(f"[{self.robot_namespace}] Node {node_label} locked by {robot_ns}.")
            elif action == 'unlock':
                self.locked_nodes.discard(node_label)
                self.get_logger().info(f"[{self.robot_namespace}] Node {node_label} unlocked by {robot_ns}.")

    def announce_node_lock(self, node_label, action):
        """
        Publishes a message announcing the lock or unlock of a node.

        Args:
            node_label (str): The label of the node.
            action (str): The action performed ('lock' or 'unlock').
        """
        msg = String()
        msg.data = f"{self.robot_namespace},{node_label},{action}"
        self.lock_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Announced {action} of node {node_label}.")

    def navigate_to_node(self, current_node, target_node):
        """
        Navigates the robot to the target node.

        Args:
            current_node (str): Current node label.
            target_node (str): Target node label.
        """
        x, y = self.nx_graph.nodes[target_node]['x'], self.nx_graph.nodes[target_node]['y']
        orientation = self.calculate_orientation(current_node, target_node)
        goal_pose = self.navigator.getPoseStamped([x, y], orientation)

        self.get_logger().info(f"[{self.robot_namespace}] Navigating to node {target_node} at ({x}, {y}).")

        result = self.navigator.startToPose(goal_pose)

        if result is None:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to send goal to {target_node}. Exiting.")
            return

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"[{self.robot_namespace}] Reached node {target_node}.")

    def calculate_orientation(self, current_node, target_node):
        """
        Calculates the orientation between two nodes.

        Args:
            current_node (str): Current node label.
            target_node (str): Target node label.

        Returns:
            float: Orientation angle in radians.
        """
        x0, y0 = self.nx_graph.nodes[current_node]['x'], self.nx_graph.nodes[current_node]['y']
        x1, y1 = self.nx_graph.nodes[target_node]['x'], self.nx_graph.nodes[target_node]['y']
        return math.atan2(y1 - y0, x1 - x0)

    def destroy_node(self):
        """
        Ensures proper resource cleanup when shutting down the node.
        """
        self.get_logger().info(f"[{self.robot_namespace}] Shutting down navigation node.")
        super().destroy_node()

def main(args=None):
    """
    Main function to initialize and spin the navigation node.
    """
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Robot Navigation Node using Chinese Postman Algorithm with Deadlock Handling')
    parser.add_argument('--robot_namespace', type=str, required=True, help='Unique namespace of the robot')
    parser.add_argument('--graph_path', type=str, required=True, help='Path to the graph JSON file')
    parser.add_argument('--start_x', type=float, required=True, help='Starting x coordinate')
    parser.add_argument('--start_y', type=float, required=True, help='Starting y coordinate')

    argv = rclpy.utilities.remove_ros_args(args)
    parsed_args = parser.parse_args(argv[1:])

    starting_point = {'x': parsed_args.start_x, 'y': parsed_args.start_y}

    # Create the navigation node instance
    navigation_node = RobotNavigationNode(
        parsed_args.robot_namespace,
        parsed_args.graph_path,
        starting_point
    )

    try:
        rclpy.spin(navigation_node)  # Spin the node to process callbacks
    except KeyboardInterrupt:
        pass
    finally:
        navigation_node.destroy_node()  # Clean up resources
        rclpy.shutdown()  # Shut down ROS2

if __name__ == '__main__':
    main()
