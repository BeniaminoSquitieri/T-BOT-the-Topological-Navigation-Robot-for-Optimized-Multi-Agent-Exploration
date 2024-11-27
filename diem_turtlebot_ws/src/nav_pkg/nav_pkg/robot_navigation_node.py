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
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

# Define the class for robot navigation
class RobotNavigationNode(Node):
    """
    ROS2 Node for robot navigation using the Chinese Postman Problem (CPP) over a shared graph.
    Implements deadlock handling through timeouts and retries, ensuring safe and coordinated navigation.
    """

    def __init__(self, robot_namespace, graph_file_path, starting_point, orientation=None, starting_node_label=None):
        """
        Initializes the navigation node for a single robot.

        Args:
            robot_namespace (str): Unique namespace for the robot.
            graph_file_path (str): Path to the JSON file containing the graph.
            starting_point (dict): Starting coordinates with 'x' and 'y' keys.
            orientation (str, optional): Starting orientation ('NORTH', 'EAST', 'SOUTH', 'WEST'). Defaults to None.
            starting_node_label (str, optional): Label of the starting node. Defaults to None.
        """
        # Call the parent class initializer and set a unique node name
        super().__init__(robot_namespace + '_navigation_node')

        self.robot_namespace = robot_namespace  # Store the robot's namespace

        # Create a navigator instance for robot control
        self.navigator = TurtleBot4Navigator()

        # Load the graph structure from the provided file path
        self.load_graph(graph_file_path)

        # Log the graph details for debugging purposes
        self.print_subgraph()

        # Determine the starting node
        if starting_node_label:
            if starting_node_label in self.nx_graph.nodes():
                self.current_node_label = starting_node_label
            else:
                # If the node doesn't exist, select a random node and notify the user
                self.current_node_label = random.choice(list(self.nx_graph.nodes()))
                self.get_logger().warn(f"[{self.robot_namespace}] Starting node '{starting_node_label}' not found. Using random node '{self.current_node_label}' instead.")
        else:
            # Find the nearest node to the starting point
            self.current_node_label = self.find_nearest_node(starting_point)
            if self.current_node_label is None:
                # If no node is found, log an error and exit initialization
                self.get_logger().error(f"[{self.robot_namespace}] No starting node found in graph.")
                return

            # Calculate orientation towards the nearest node
            orientation_radians = self.calculate_orientation_point_to_node(starting_point, self.current_node_label)
            # Set the initial pose with the calculated orientation
            self.set_initial_pose(starting_point, orientation_radians)

            # Navigate to the nearest node before starting the main navigation
            self.navigate_to_coordinates(starting_point, self.nx_graph.nodes[self.current_node_label])

        # Log the node where the robot is starting its navigation
        self.get_logger().info(f"[{self.robot_namespace}] Starting at node: {self.current_node_label}")

        # Set the robot's initial position in the simulated environment
        if not starting_node_label:
            # Use the orientation towards the first node in the CPP route
            if orientation is None:
                # If no orientation is provided, calculate it towards the next node
                if self.current_node_label:
                    next_node_label = self.get_next_node_in_route(self.current_node_label)
                    if next_node_label:
                        orientation_radians = self.calculate_orientation(self.current_node_label, next_node_label)
                    else:
                        orientation_radians = 0.0
                else:
                    orientation_radians = 0.0
            else:
                # Convert orientation string to radians
                orientation_radians = self.orientation_conversion(orientation)

            self.set_initial_pose({'x': self.nx_graph.nodes[self.current_node_label]['x'],
                                   'y': self.nx_graph.nodes[self.current_node_label]['y']},
                                  orientation_radians)

        # Calculate the CPP (Chinese Postman Problem) route for the robot
        self.compute_cpp_route()

        # If no CPP route is calculated, log an error and exit initialization
        if not self.cpp_route:
            self.get_logger().error(f"[{self.robot_namespace}] Unable to compute CPP route.")
            return

        # Initialize the route index to track the current position in the CPP route
        self.route_index = 0

        # Flag to indicate if the robot is currently navigating
        self.navigation_in_progress = False

        # A set to track nodes that are locked or targeted by this or other robots
        self.locked_nodes = set()

        # Subscribe to a shared topic to receive updates on node locks
        self.lock_subscriber = self.create_subscription(
            String, 'node_locks', self.lock_callback, 10
        )

        # Publisher to announce locks or unlocks for nodes
        self.lock_publisher = self.create_publisher(
            String, 'node_locks', 10
        )

        # Timer to periodically invoke the `navigate` method for navigation management
        self.timer_period = 1.0  # Timer interval in seconds
        self.timer = self.create_timer(self.timer_period, self.navigate)

    def load_graph(self, graph_file_path):
        """
        Loads the graph from a JSON file and creates a directed NetworkX graph.

        Args:
            graph_file_path (str): Path to the JSON file containing the graph structure.
        """
        # Open and parse the graph JSON file
        with open(graph_file_path, 'r') as f:
            data = json.load(f)

        # Initialize a directed graph (edges have directions)
        self.nx_graph = nx.DiGraph()

        # Add nodes to the graph, along with their attributes (like coordinates)
        for node in data['nodes']:
            label = node['label']
            x = node['x']
            y = node['y']
            self.nx_graph.add_node(label, x=x, y=y)

        # Add edges between nodes, with weights (distances)
        for edge in data['edges']:
            u = edge['from']
            v = edge['to']
            weight = edge.get('weight', 1.0)  # Use weight from data or default to 1.0
            self.nx_graph.add_edge(u, v, weight=weight)

    def print_subgraph(self):
        """
        Logs the nodes and edges of the assigned graph for debugging purposes.
        """
        # Log all nodes in the graph
        self.get_logger().info(f"[{self.robot_namespace}] Assigned subgraph nodes:")
        for node in self.nx_graph.nodes(data=True):
            self.get_logger().info(f"Node {node[0]}: {node[1]}")

    def find_nearest_node(self, point):
        """
        Finds the nearest graph node to the given point.

        Args:
            point (dict): Coordinates with 'x' and 'y' keys.

        Returns:
            str: Label of the nearest node.
        """
        min_distance = float('inf')  # Initialize the minimum distance to a large value
        nearest_node_label = None  # Placeholder for the closest node
        x0, y0 = point['x'], point['y']  # Extract the point's coordinates

        # Iterate through all nodes to find the one closest to the given point
        for node_label in self.nx_graph.nodes():
            data = self.nx_graph.nodes[node_label]
            x1, y1 = data['x'], data['y']  # Node coordinates
            distance = math.hypot(x1 - x0, y1 - y0)  # Calculate Euclidean distance
            if distance < min_distance:  # Update if a closer node is found
                min_distance = distance
                nearest_node_label = node_label

        return nearest_node_label  # Return the label of the closest node

    def set_initial_pose(self, point, orientation):
        """
        Sets the robot's initial pose in the environment.

        Args:
            point (dict): Coordinates with 'x' and 'y' keys.
            orientation (float): Orientation angle in radians.
        """
        # Extract the coordinates
        x, y = point['x'], point['y']

        # Create a pose object with position and specified orientation
        initial_pose = self.navigator.getPoseStamped([x, y], orientation)

        # Set the robot's initial pose
        self.navigator.setInitialPose(initial_pose)

        # Wait for a short duration to ensure the pose is applied
        time.sleep(1.0)

    def compute_cpp_route(self):
        """
        Computes the Chinese Postman Problem (CPP) route for the robot.
        """
        # Create an Eulerian version of the graph (all nodes have even degrees)
        self.eulerian_graph = self.eulerize(self.nx_graph)

        # Try to compute the Eulerian circuit (path that visits all edges exactly once)
        self.cpp_route = []
        try:
            euler_circuit = list(nx.eulerian_circuit(self.eulerian_graph, source=self.current_node_label))
            for u, v in euler_circuit:
                self.cpp_route.append((u, v))  # Append each edge to the CPP route
            self.get_logger().info(f"[{self.robot_namespace}] Computed CPP route: {self.cpp_route}")
        except Exception as e:
            # If the Eulerian circuit fails, log an error and set the route to None
            self.get_logger().error(f"[{self.robot_namespace}] Failed to compute Eulerian circuit: {str(e)}")
            self.cpp_route = None

    def eulerize(self, G):
        """
        Ensures the graph is Eulerian by adding edges between odd-degree nodes.

        Args:
            G (nx.DiGraph): The original directed graph.

        Returns:
            nx.MultiDiGraph: An Eulerian version of the graph with added edges.
        """
        # Convert the directed graph to an undirected version and check connectivity
        if not nx.is_connected(G.to_undirected()):
            raise nx.NetworkXError("Graph is not connected.")

        # Find nodes with odd degrees (degree is the number of edges connected to a node)
        odd_degree_nodes = [v for v, d in G.degree() if d % 2 == 1]

        # Compute shortest paths between all pairs of odd-degree nodes
        import itertools
        odd_node_pairs = list(itertools.combinations(odd_degree_nodes, 2))
        distances = dict(nx.all_pairs_dijkstra_path_length(G, weight='weight'))

        # Create a complete graph where odd nodes are connected, weighted by shortest paths
        complete_graph = nx.Graph()
        for u, v in odd_node_pairs:
            weight = distances[u][v]  # Use the shortest path length as the weight
            complete_graph.add_edge(u, v, weight=weight)

        # Find the minimum weight matching for odd-degree nodes
        matching = nx.algorithms.matching.min_weight_matching(complete_graph, weight='weight')

        # Add edges from the matching to the original graph to make it Eulerian
        G_eulerized = nx.MultiDiGraph(G)
        for u, v in matching:
            path = nx.shortest_path(G, source=u, target=v, weight='weight')  # Get shortest path
            for i in range(len(path) - 1):  # Add each edge in the path
                edge_data = G.get_edge_data(path[i], path[i + 1])
                weight = edge_data['weight']
                G_eulerized.add_edge(path[i], path[i + 1], weight=weight)

        return G_eulerized  # Return the Eulerian graph

    def navigate(self):
        """
        Manages the robot's navigation through the computed CPP route.
        Handles node locking and retries to prevent deadlocks.
        """
        # Skip navigation if another navigation task is already in progress
        if self.navigation_in_progress:
            return

        # Check if the robot has completed the CPP route
        if self.route_index >= len(self.cpp_route):
            self.get_logger().info(f"[{self.robot_namespace}] Completed CPP route. Restarting...")
            self.route_index = 0  # Reset the route index to restart navigation
            return

        # Get the current and next nodes in the route
        current_node, next_node = self.cpp_route[self.route_index]

        # Check if the next node is locked (by this or another robot)
        if next_node in self.locked_nodes:
            self.get_logger().info(f"[{self.robot_namespace}] Node {next_node} is locked. Retrying...")
            time.sleep(random.uniform(1.0, 3.0))  # Wait a random amount of time before retrying
            return

        # Lock the next node and announce the lock
        self.locked_nodes.add(next_node)
        self.announce_node_lock(next_node, 'lock')

        # Unlock the previous node (if applicable) and announce the unlock
        if self.route_index > 0:
            prev_node = self.cpp_route[self.route_index - 1][1]
            self.locked_nodes.discard(prev_node)
            self.announce_node_lock(prev_node, 'unlock')

        # Start navigation to the next node
        self.navigation_in_progress = True
        self.navigate_to_node(current_node, next_node)  # Perform the navigation
        self.current_node_label = next_node  # Update the current node label
        self.route_index += 1  # Move to the next step in the route
        self.navigation_in_progress = False  # Mark navigation as complete

    def lock_callback(self, msg):
        """
        Handles messages about node locks from other robots.

        Args:
            msg (String): Message in the format "robot_namespace,node_label,action".
        """
        # Parse the message to extract the namespace, node label, and action
        robot_ns, node_label, action = msg.data.split(',')

        # Ignore messages from the same robot (self)
        if robot_ns != self.robot_namespace:
            if action == 'lock':
                # If the action is "lock", add the node to the locked set
                self.locked_nodes.add(node_label)
                self.get_logger().info(f"[{self.robot_namespace}] Node {node_label} locked by {robot_ns}.")
            elif action == 'unlock':
                # If the action is "unlock", remove the node from the locked set
                self.locked_nodes.discard(node_label)
                self.get_logger().info(f"[{self.robot_namespace}] Node {node_label} unlocked by {robot_ns}.")

    def announce_node_lock(self, node_label, action):
        """
        Publishes a message announcing the lock or unlock of a node.

        Args:
            node_label (str): The label of the node.
            action (str): The action performed ('lock' or 'unlock').
        """
        # Format the lock or unlock message
        msg = String()
        msg.data = f"{self.robot_namespace},{node_label},{action}"

        # Publish the message to the shared "node_locks" topic
        self.lock_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Announced {action} of node {node_label}.")

    def navigate_to_node(self, current_node, target_node):
        """
        Navigates the robot to the target node.
        
        Args:
            current_node (str): Current node label.
            target_node (str): Target node label.
        """
        # Get the coordinates of the target node
        x, y = self.nx_graph.nodes[target_node]['x'], self.nx_graph.nodes[target_node]['y']

        # Calculate the orientation for the robot to face the target node
        orientation = self.calculate_orientation(current_node, target_node)
        
        # Convert l'orientamento da radianti a stringa
        orientation_str = self.get_orientation_str(orientation)

        # Create a goal pose for navigation
        goal_pose = self.navigator.getPoseStamped([x, y], orientation)

        # Log the navigation task with orientamento come stringa
        self.get_logger().info(f"[{self.robot_namespace}] Navigating to node {target_node} at ({x}, {y}) with orientation {orientation_str} ({orientation} radians).")

        # Start navigation and handle potential failures
        result = self.navigator.startToPose(goal_pose)
        if result is None:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to send goal to {target_node}. Exiting.")
            return

        # Wait until the navigation task is complete
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        # Log that the target node has been reached
        self.get_logger().info(f"[{self.robot_namespace}] Reached node {target_node} with orientation {orientation_str} ({orientation} radians).")



    def navigate_to_coordinates(self, start_point, target_node_data):
        """
        Navigates the robot to the coordinates of the nearest node before starting the main navigation.
s
        Args:
            start_point (dict): Starting coordinates with 'x' and 'y' keys.
            target_node_data (dict): Data of the target node.
        """
        # Extract coordinates
        x_start, y_start = start_point['x'], start_point['y']
        x_target, y_target = target_node_data['x'], target_node_data['y']

        # Calculate the orientation
        orientation = math.atan2(y_target - y_start, x_target - x_start)

        # Create a goal pose
        goal_pose = self.navigator.getPoseStamped([x_target, y_target], orientation)

        # Log the navigation
        self.get_logger().info(f"[{self.robot_namespace}] Navigating to nearest node at ({x_target}, {y_target}) with orientation {orientation} radians.")

        # Start navigation
        result = self.navigator.startToPose(goal_pose)
        if result is None:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to navigate to nearest node. Exiting.")
            return

        # Wait until navigation is complete
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        # Log that the node has been reached
        self.get_logger().info(f"[{self.robot_namespace}] Reached nearest node.")

    def calculate_orientation(self, current_node, target_node):
        """
        Calculates the orientation (angle) between two nodes.

        Args:
            current_node (str): Current node label.
            target_node (str): Target node label.

        Returns:
            float: Orientation angle in radians.
        """
        # Extract coordinates of the current and target nodes
        x0, y0 = self.nx_graph.nodes[current_node]['x'], self.nx_graph.nodes[current_node]['y']
        x1, y1 = self.nx_graph.nodes[target_node]['x'], self.nx_graph.nodes[target_node]['y']

        # Compute the angle between the two points
        return math.atan2(y1 - y0, x1 - x0)

    def calculate_orientation_point_to_node(self, point, target_node_label):
        """
        Calculates the orientation (angle) from a point to a node.

        Args:
            point (dict): Coordinates with 'x' and 'y' keys.
            target_node_label (str): Target node label.

        Returns:
            float: Orientation angle in radians.
        """
        # Extract coordinates
        x0, y0 = point['x'], point['y']
        x1, y1 = self.nx_graph.nodes[target_node_label]['x'], self.nx_graph.nodes[target_node_label]['y']

        # Compute the angle between the two points
        return math.atan2(y1 - y0, x1 - x0)

    def orientation_conversion(self, orientation_str):
        """
        Converts an orientation string to an angle in radians.

        Args:
            orientation_str (str): Orientation as a string ('NORTH', 'EAST', 'SOUTH', 'WEST').

        Returns:
            float: Orientation angle in radians.
        """
        orientation_map = {
            "NORTH": 0.0,
            "EAST": -math.pi / 2,
            "SOUTH": math.pi,
            "WEST": math.pi / 2
        }
        return orientation_map.get(orientation_str.upper(), 0.0)
    

    def get_orientation_str(self, orientation_radians):
        """
        Converte un angolo in radianti nella stringa di orientamento corrispondente.
        
        Args:
            orientation_radians (float): Angolo in radianti.
        
        Returns:
            str: Orientamento come stringa ('NORTH', 'EAST', 'SOUTH', 'WEST').
        """
        orientation_map = {
            0.0: 'NORTH',
            -math.pi / 2: 'EAST',
            math.pi: 'SOUTH',
            math.pi / 2: 'WEST'
        }
        
        # Trova l'orientamento più vicino basato sull'angolo
        closest_orientation = min(orientation_map.keys(), key=lambda k: abs(k - orientation_radians))
        return orientation_map[closest_orientation]


    def get_next_node_in_route(self, current_node_label):
        """
        Retrieves the next node in the CPP route after the current node.

        Args:
            current_node_label (str): The label of the current node.

        Returns:
            str or None: The label of the next node, or None if not found.
        """
        for idx, (u, v) in enumerate(self.cpp_route):
            if u == current_node_label:
                return v
        return None

    def destroy_node(self):
        """
        Ensures proper resource cleanup when shutting down the node.
        """
        # Log that the node is shutting down
        self.get_logger().info(f"[{self.robot_namespace}] Shutting down navigation node.")
        super().destroy_node()  # Call the parent class method for cleanup


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Robot Navigation Node using Chinese Postman Algorithm')
    parser.add_argument('--robot_namespace', type=str, required=True, help='Unique namespace of the robot')
    parser.add_argument('--graph_path', type=str, required=True, help='Path to the graph JSON file')
    parser.add_argument('--robot_id', type=int, required=True, help='Unique robot ID')
    parser.add_argument('--num_robots', type=int, required=True, help='Total number of robots')
    parser.add_argument('--start_x', type=float, required=True, help='Starting x coordinate')
    parser.add_argument('--start_y', type=float, required=True, help='Starting y coordinate')
    parser.add_argument('--orientation', type=str, help='Starting orientation (NORTH, EAST, SOUTH, WEST)')
    parser.add_argument('--start_node', type=str, help='Starting node label (optional)')

    argv = rclpy.utilities.remove_ros_args(args)
    parsed_args = parser.parse_args(argv[1:])

    starting_point = {'x': parsed_args.start_x, 'y': parsed_args.start_y}

    navigation_node = RobotNavigationNode(
        parsed_args.robot_namespace,
        parsed_args.graph_path,
        starting_point,
        starting_node_label=parsed_args.start_node,
        orientation=parsed_args.orientation
    )

    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_node.destroy_node()
        rclpy.shutdown()


# Ensure the script runs only if executed directly
if __name__ == '__main__':
    main()
