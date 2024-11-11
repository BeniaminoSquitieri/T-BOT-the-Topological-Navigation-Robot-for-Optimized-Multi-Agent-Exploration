import os
import argparse
import time
import rclpy
from rclpy.node import Node
import json
import math
import networkx as nx
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class NavigationNode(Node):
    """
    ROS2 Node for navigation using a graph structure optimized with the Directed Chinese Postman Problem (DCPP).

    Attributes:
        nx_graph (networkx.DiGraph): Directed graph representation using NetworkX.
        navigator (TurtleBot4Navigator): Instance of TurtleBot4Navigator for controlling the robot.
        dcpp_route (list): Route calculated by the DCPP as a list of node pairs (u, v).
        current_node_label (str): Label of the current node where the robot is located.
    """
    def __init__(self, graph_file_path, starting_point):
        """
        Initializes the navigation node.

        Args:
            graph_file_path (str): Path to the JSON file containing the graph structure.
            starting_point (dict): Starting point with 'x' and 'y' keys representing coordinates.
        """
        super().__init__('navigation_node')  # Base Node constructor with the name 'navigation_node'

        self.navigator = TurtleBot4Navigator()  # Creates an instance of the navigator to control the robot

        # Load the graph from the JSON file and build the directed graph
        self.load_graph(graph_file_path)

        # Compute the Directed Chinese Postman Problem (DCPP) route
        self.dcpp_route = self.compute_dcpp_route(self.nx_graph)

        if not self.dcpp_route:
            self.get_logger().error("Unable to calculate the DCPP route.")
            return

        # Find the nearest node to the starting point
        self.current_node_label = self.find_nearest_node(starting_point)
        self.get_logger().info(f"Starting point: {self.current_node_label}")

        # Set the robot's initial pose to the starting node
        self.set_initial_pose(self.current_node_label)

        # Navigate through the DCPP route starting from the current node
        self.navigate_dcpp_route()

    def load_graph(self, graph_file_path):
        """
        Loads the graph from a JSON file and builds a NetworkX DiGraph.

        Args:
            graph_file_path (str): Path to the JSON file containing the graph structure.
        """
        # Load the graph data from the JSON file
        with open(graph_file_path, 'r') as f:
            data = json.load(f)

        # Create a directed graph using NetworkX
        self.nx_graph = nx.DiGraph()  # Constructor for a directed graph

        # Add nodes to the graph with their coordinates as attributes
        for node in data['nodes']:
            label = node['label']  # Node label (e.g., 'node_1')
            x = node['x']          # Node's X coordinate
            y = node['y']          # Node's Y coordinate
            # Add the node to the graph with attributes
            self.nx_graph.add_node(label, x=x, y=y)

        # Add edges to the graph with calculated weights (Euclidean distance)
        for edge in data['edges']:
            u = edge['from']  # Edge's starting node
            v = edge['to']    # Edge's ending node
            x1, y1 = self.nx_graph.nodes[u]['x'], self.nx_graph.nodes[u]['y']  # Coordinates of node u
            x2, y2 = self.nx_graph.nodes[v]['x'], self.nx_graph.nodes[v]['y']  # Coordinates of node v
            weight = math.hypot(x2 - x1, y2 - y1)  # Calculate weight as Euclidean distance
            # Add the edge to the graph with the calculated weight
            self.nx_graph.add_edge(u, v, weight=weight)

    def compute_dcpp_route(self, G):
        """
        Calculates the DCPP route for the given directed graph G.

        Args:
            G (networkx.DiGraph): Directed graph on which to calculate the DCPP route.

        Returns:
            list: The DCPP route as a list of edges (u, v).
        """
        # Check if the graph is strongly connected
        if not nx.is_strongly_connected(G):
            self.get_logger().error("The directed graph is not strongly connected.")
            return None

        # Calculate imbalance for each node (out-degree - in-degree)
        imbalances = {v: G.out_degree(v) - G.in_degree(v) for v in G.nodes()}
        positive_nodes = [v for v, imbalance in imbalances.items() if imbalance > 0]  # Nodes with positive imbalance
        negative_nodes = [v for v, imbalance in imbalances.items() if imbalance < 0]  # Nodes with negative imbalance

        # Create a list of unbalanced node pairs with costs to balance the graph
        imbalance_pairs = []
        for u in positive_nodes:
            for v in negative_nodes:
                x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
                x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
                distance = math.hypot(x2 - x1, y2 - y1)  # Calculate cost as Euclidean distance
                imbalance_pairs.append((u, v, distance))

        # Create a new graph M to represent the minimum-cost flow
        M = nx.DiGraph()
        M.add_nodes_from(G.nodes(data=True))  # Add all nodes from G to M
        M.add_edges_from(G.edges(data=True))  # Add all existing edges from G to M

        # Add edges between unbalanced nodes with capacity and weight
        for u, v, cost in imbalance_pairs:
            capacity = min(imbalances[u], -imbalances[v])  # Maximum possible capacity between u and v
            if capacity > 0:
                M.add_edge(u, v, capacity=capacity, weight=cost)  # Add the edge with capacity and cost

        # Add a super source and super sink to graph M
        M.add_node('super_source')
        M.add_node('super_sink')

        # Connect the super source to nodes with positive imbalance
        for v in positive_nodes:
            M.add_edge('super_source', v, capacity=imbalances[v], weight=0)

        # Connect nodes with negative imbalance to the super sink
        for v in negative_nodes:
            M.add_edge(v, 'super_sink', capacity=-imbalances[v], weight=0)

        # Calculate the minimum-cost flow in the graph to balance the original graph
        flowDict = nx.algorithms.flow.min_cost_flow(M, 'super_source', 'super_sink')

        # Add the necessary edges to the original graph G based on the calculated flow
        for u in G.nodes():
            for v in G.nodes():
                flow = flowDict.get(u, {}).get(v, 0)
                if flow > 0:
                    for _ in range(flow):
                        # Add the edge with the same weight as the original edge
                        G.add_edge(u, v, weight=G[u][v]['weight'])

        # After balancing, check if the graph is now Eulerian
        if nx.is_eulerian(G):
            self.get_logger().info("The graph has been balanced and is now Eulerian.")
            circuit = list(nx.eulerian_circuit(G))  # Find the Eulerian circuit in the balanced graph
            return circuit
        else:
            self.get_logger().error("Unable to balance the graph to make it Eulerian.")
            return None

    def find_nearest_node(self, point):
        """
        Finds the nearest node in the graph to the given point.

        Args:
            point (dict): A point with 'x' and 'y' keys.

        Returns:
            str: The label of the nearest node.
        """
        min_distance = float('inf')
        nearest_node_label = None
        x0, y0 = point['x'], point['y']  # Coordinates of the starting point
        for node_label, data in self.nx_graph.nodes(data=True):
            x1, y1 = data['x'], data['y']  # Coordinates of the node
            distance = math.hypot(x1 - x0, y1 - y0)  # Calculate Euclidean distance
            if distance < min_distance:
                min_distance = distance
                nearest_node_label = node_label
        return nearest_node_label

    def set_initial_pose(self, node_label):
        """
        Sets the initial pose of the robot to the given node.

        Args:
            node_label (str): The label of the node.
        """
        x, y = self.nx_graph.nodes[node_label]['x'], self.nx_graph.nodes[node_label]['y']  # Node's coordinates
        initial_pose = self.navigator.getPoseStamped([x, y], 0.0)  # Create a PoseStamped with 0 radians orientation
        self.navigator.setInitialPose(initial_pose)  # Set initial pose using the navigator
        time.sleep(1.0)

    def navigate_dcpp_route(self):
        """
        Navigates through the calculated DCPP route starting from the current node.
        """
        # Reorder the circuit to start from the current node
        circuit = self.reorder_circuit(self.dcpp_route, self.current_node_label)
        self.get_logger().info("Starting navigation through the DCPP route.")

        for u, v in circuit:
            # Determine the next node based on the current node
            if u == self.current_node_label:
                next_node_label = v
            elif v == self.current_node_label:
                next_node_label = u
            else:
                continue  # Skip edges not connected to the current node

            # Get the coordinates of the next node
            x, y = self.nx_graph.nodes[next_node_label]['x'], self.nx_graph.nodes[next_node_label]['y']

            # Calculate orientation toward the next node
            orientation = self.compute_orientation(self.current_node_label, next_node_label)

            # Navigate to the next node
            self.navigate_to_node(next_node_label, orientation)

            # Update the current node
            self.current_node_label = next_node_label

        self.get_logger().info("Navigation through the DCPP route completed.")

    def reorder_circuit(self, circuit, start_node_label):
        """
        Reorders the circuit to start from the specified node.

        Args:
            circuit (list): The Eulerian circuit as a list of edges (u, v).
            start_node_label (str): Label of the starting node.

        Returns:
            list: The reordered circuit.
        """
        for i, (u, v) in enumerate(circuit):
            if u == start_node_label:
                return circuit[i:] + circuit[:i]  # Reorder the circuit to start from start_node_label
        return circuit  # If the start node is not found, return the circuit as is

    def compute_orientation(self, u_label, v_label):
        """
        Calculates the orientation angle from node u to node v.

        Args:
            u_label (str): Label of node u.
            v_label (str): Label of node v.

        Returns:
            float: Orientation angle in radians.
        """
        x1, y1 = self.nx_graph.nodes[u_label]['x'], self.nx_graph.nodes[u_label]['y']  # Coordinates of node u
        x2, y2 = self.nx_graph.nodes[v_label]['x'], self.nx_graph.nodes[v_label]['y']  # Coordinates of node v
        angle = math.atan2(y2 - y1, x2 - x1)  # Calculate the angle between the two nodes
        return angle

    def navigate_to_node(self, node_label, orientation_angle):
        """
        Navigates to the specified node.

        Args:
            node_label (str): Identifier of the node to navigate to.
            orientation_angle (float): Orientation angle in radians.
        """
        x, y = self.nx_graph.nodes[node_label]['x'], self.nx_graph.nodes[node_label]['y']  # Node's coordinates
        goal_pose = self.navigator.getPoseStamped([x, y], orientation_angle)  # Create the goal pose
        self.navigator.startToPose(goal_pose)  # Start navigation to the goal
        self.get_logger().info(f"Navigating to node {node_label} at ({x}, {y}) with orientation {orientation_angle:.2f} radians.")
        # Wait until the robot reaches the goal
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)
        time.sleep(1.0)  # Wait a bit before moving to the next node

def main(args=None):
    """
    Main function to initialize and run the navigation node.
    """
    rclpy.init(args=args)  

    # Parse command line arguments to get starting coordinates
    parser = argparse.ArgumentParser(description='Navigation Node using DCPP')
    parser.add_argument('--start_x', type=float, required=True, help='Starting x coordinate')
    parser.add_argument('--start_y', type=float, required=True, help='Starting y coordinate')
    args = parser.parse_args()

    starting_point = {'x': args.start_x, 'y': args.start_y}  # Create a dictionary for the starting point

    # Get the absolute path of the current script's directory
    script_path = os.path.dirname(os.path.abspath(__file__))

    # Path to the graph JSON file
    graph_path = os.path.join(script_path, '/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/map_transformation_phase/graph/navigation_graph_simplified.json')

    # Create an instance of the navigation node with the graph path and starting point
    navigation_node = NavigationNode(graph_path, starting_point)

    # No need to run rclpy.spin as there are no topic subscriptions
    # rclpy.spin(navigation_node)

    navigation_node.destroy_node()  # Destroy the node
    rclpy.shutdown()  # Shut down ROS2

if __name__ == '__main__':
    main()
