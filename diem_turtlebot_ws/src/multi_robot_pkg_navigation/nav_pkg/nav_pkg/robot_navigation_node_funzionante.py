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
import asyncio



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
        self.navigator = TurtleBot4Navigator()

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

        # # Start the navigation in a separate thread to allow ROS2 callbacks to be processed
        # self.navigation_thread = threading.Thread(target=self.navigate)
        # self.navigation_thread.start()

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

        # Stampare il sottografo assegnato
        self.get_logger().info(f"[{self.robot_namespace}] Assigned subgraph nodes: {self.subgraph_nodes}")

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
        self.navigator.setInitialPose(initial_pose)

        # Wait for a short period to allow the initial pose to be properly set
        # This ensures the pose is registered by the robot and the navigation system
        time.sleep(1.0)


    async def navigate(self):
        # Log the start of the navigation process, including the nodes assigned to this robot.
        # This provides an overview of the subgraph the robot will explore.
        self.get_logger().info(f"[{self.robot_namespace}] Starting navigation in assigned subgraph: {self.subgraph_nodes}")

        # Set the current node to the robot's starting position.
        # This ensures the navigation process begins from the correct initial node.
        current_node_label = self.current_node_label

        # Continue navigating while the ROS2 system is active.
        while rclpy.ok():
            # Select the next node to visit based on the current position.
            # This uses the graph structure and the robot's assigned subgraph to choose the next node.
            next_node = self.select_next_node(current_node_label)

            # If no valid next node is available, it means the robot has completed its navigation task.
            if not next_node:
                self.get_logger().info(f"[{self.robot_namespace}] Navigation completed.")
                break

            # Announce the target node to other robots.
            # This prevents conflicts by ensuring no two robots aim for the same node simultaneously.
            self.announce_target_node(next_node)

            # Navigate to the selected node asynchronously.
            # This ensures the robot can perform other tasks or process events while moving.
            await self.navigate_to_node(current_node_label, next_node)

            # Update the current node to the one just visited.
            # This prepares for the next iteration of the navigation loop.
            current_node_label = next_node

            # Safely remove the node from the list of targeted nodes.
            # This ensures it is no longer marked as "in progress" and prevents conflicts with other robots.
            with self.node_lock:
                self.targeted_nodes.discard(next_node)

        # Log the end of the navigation loop.
        # This indicates the robot is exiting the navigation process.
        self.get_logger().info(f"[{self.robot_namespace}] Exiting navigation loop.")





    def select_next_node(self, current_node):
        """
        Selects the next node to move to from the current node within the assigned subgraph.

        Args:
            current_node (str): Label of the current node.

        Returns:
            str: The next node to move to, or None if no nodes are available.
        """
        # Acquire the lock to ensure thread-safe access to the shared node sets
        with self.node_lock:
            # Identify all neighboring nodes of the current node in the subgraph
            # Only include nodes that:
            # 1. Are not already visited (to avoid revisiting the same node)
            # 2. Are not currently targeted by other robots (to avoid conflicts)
            # 3. Are within the subgraph assigned to the robot
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


    async def navigate_to_node(self, current_node, target_node):
        """
        Navigates to the specified node asynchronously.

        Args:
            current_node (str): The current node label.
            target_node (str): The target node label.
        """
        x, y = self.nx_graph.nodes[target_node]['x'], self.nx_graph.nodes[target_node]['y']
        orientation = self.calculate_orientation(current_node, target_node)

        # Crea il goal pose
        goal_pose = self.navigator.getPoseStamped([x, y], orientation)

        self.get_logger().info(f"[{self.robot_namespace}] Navigating to node {target_node} at ({x}, {y}).")

        # Avvia la navigazione verso il goal pose
        send_goal_future = self.navigator.startToPose(goal_pose)

        if send_goal_future is None:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to send goal to {target_node}. Exiting.")
            return

        # Aspetta il completamento
        try:
            await asyncio.wrap_future(send_goal_future)
            self.get_logger().info(f"[{self.robot_namespace}] Reached node {target_node}.")
        except Exception as e:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation to {target_node} failed: {str(e)}")


    def calculate_orientation(self, current_node, target_node):
        # Coordinate del nodo corrente
        x0, y0 = self.nx_graph.nodes[current_node]['x'], self.nx_graph.nodes[current_node]['y']
        # Coordinate del nodo target
        x1, y1 = self.nx_graph.nodes[target_node]['x'], self.nx_graph.nodes[target_node]['y']
        # Calcolo dell'angolo (orientamento) in radianti
        return math.atan2(y1 - y0, x1 - x0)




    def destroy_node(self):
        """
        Overrides the destroy_node method to ensure resources are properly released.
        """
        self.get_logger().info(f"[{self.robot_namespace}] Shutting down navigation node.")
        super().destroy_node()



def main(args=None):
    # Initialize the ROS2 system
    rclpy.init(args=args)

    # Create a parser to read command-line parameters.
    # Parsing arguments allows us to dynamically configure the robot's behavior at runtime.
    # For example, we can specify the robot's namespace, graph file, ID, number of robots, and starting position.
    parser = argparse.ArgumentParser(description='Robot Navigation Node with Internal Graph Partitioning')

    # Define the required parameters for the node.
    # These arguments allow the node to understand which robot it represents,
    # where it starts, how many robots exist, and the map it should use for navigation.
    parser.add_argument('--robot_namespace', type=str, required=True, help='Unique namespace of the robot')
    parser.add_argument('--graph_path', type=str, required=True, help='Path to the full graph JSON file')
    parser.add_argument('--robot_id', type=int, required=True, help='Unique ID of the robot (e.g., 0, 1, 2)')
    parser.add_argument('--num_robots', type=int, required=True, help='Total number of robots')
    parser.add_argument('--start_x', type=float, required=True, help='Starting x coordinate')
    parser.add_argument('--start_y', type=float, required=True, help='Starting y coordinate')

    # Remove any ROS2-specific arguments that might interfere and parse the user-defined arguments.
    # This ensures that only relevant arguments for this script are considered. ROS2 adds some default arguments (e.g., for remapping topics or namespaces) which are not relevant for this script.
    # Removing them ensures that our argument parser processes only the ones explicitly defined for the navigation node.
    argv = rclpy.utilities.remove_ros_args(args)
    parsed_args = parser.parse_args(argv[1:])

    # Create a dictionary with the robot's starting coordinates.
    # This helps encapsulate the position information in a simple, reusable format.
    starting_point = {'x': parsed_args.start_x, 'y': parsed_args.start_y}

    # Instantiate the navigation node using the provided parameters.
    # These parameters will determine the robot's behavior and assigned tasks.
    navigation_node = RobotNavigationNode(
        parsed_args.robot_namespace,  # Unique namespace for the robot
        parsed_args.graph_path,       # Path to the graph JSON file
        parsed_args.robot_id,         # Robot's unique identifier
        parsed_args.num_robots,       # Total number of robots
        starting_point                # Robot's starting point
    )

    try:
        # Get the asyncio event loop to execute the node.
        # Asynchronous execution allows for non-blocking operations, which are ideal for real-time systems like ROS2.
        loop = asyncio.get_event_loop()
        # Start the asynchronous navigation task.
        loop.run_until_complete(navigation_node.navigate())
    except KeyboardInterrupt:
        # Handle keyboard interrupt (Ctrl+C) gracefully.
        # This ensures the program exits cleanly without leaving processes hanging.
        pass
    finally:
        # Properly shut down the node and release ROS2 resources.
        # Ensures that any resources allocated by the node are cleaned up.
        navigation_node.destroy_node()
        rclpy.shutdown()

# Program entry point
# The `if __name__ == '__main__'` condition ensures this script runs only if executed directly,
# and not if it's imported as a module into another script.
if __name__ == '__main__':
    main()

