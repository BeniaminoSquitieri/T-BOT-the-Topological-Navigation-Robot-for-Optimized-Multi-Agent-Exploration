#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import argparse
import math
import networkx as nx
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult
from .graph_partitioning import load_full_graph, partition_graph
from .path_calculation import calculate_dcpp_route, orientation_conversion

class SlaveState:
    """
    Class to manage the state of each slave robot.
    """
    def __init__(self, slave_ns, publisher):
        self.slave_ns = slave_ns
        self.publisher = publisher
        self.assigned_waypoints = []  # List of assigned waypoints
        self.current_waypoint_index = 0  # Index of the next waypoint to assign
        self.last_seen_time = 0.0  # Last time the slave communicated
        self.initial_x = None  # Initial X position
        self.initial_y = None  # Initial Y position
        self.initial_orientation = None  # Initial orientation
        self.is_master = False  # Flag indicating if this slave has become the master

class SlaveNavigationNode(Node):
    def __init__(self, robot_namespace, initial_x, initial_y, initial_orientation_str):
        # Initialize the node without specifying the namespace
        super().__init__('slave_navigation_node')

        self.robot_namespace = robot_namespace
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.initial_orientation_str = initial_orientation_str
        self.initial_orientation = self.orientation_conversion(initial_orientation_str)

        # Publisher to register the slave with the master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)

        # Publisher to send the initial position to the master
        self.initial_position_publisher = self.create_publisher(String, '/slave_initial_positions', 10)

        # Subscriber to receive navigation commands from the master
        self.navigation_commands_subscriber = self.create_subscription(
            String,
            '/navigation_commands',
            self.navigation_commands_callback,
            10
        )

        # Publisher to send navigation status to the master
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Publisher to send heartbeat messages to indicate this slave is active
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)

        # Subscriber to receive heartbeat messages from the master
        self.master_heartbeat_subscriber = self.create_subscription(
            String,
            '/master_heartbeat',
            self.master_heartbeat_callback,
            10
        )

        # Subscriber to receive heartbeat messages from other slaves
        self.slave_heartbeat_subscriber = self.create_subscription(
            String,
            '/slave_heartbeat',
            self.slave_heartbeat_callback,
            10
        )

        # Subscriber to receive the navigation graph from the master
        self.graph_subscriber = self.create_subscription(
            String,
            '/navigation_graph',
            self.navigation_graph_callback,
            10
        )

        # Timer to regularly publish registration messages
        self.registration_timer = self.create_timer(1.0, self.publish_registration)

        # Timer to regularly publish heartbeat messages
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Publish the initial position once at startup
        self.publish_initial_position()

        # Initialize the TurtleBot4Navigator
        self.navigator = TurtleBot4Navigator()

        # Initialize variables for master election
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 5.0  # Seconds to wait before considering the master dead

        # List of active slaves (namespace and last heartbeat)
        self.active_slaves = {}  # Key: slave_ns, Value: last_seen_time

        # Initialize the navigation graph
        self.navigation_graph = None

        # Log the initialization of the slave
        self.get_logger().info(f"[{self.robot_namespace}] Slave node initialized at ({self.initial_x}, {self.initial_y}) with orientation {self.initial_orientation_str} ({self.initial_orientation} radians).")

        # Timer to check for master heartbeat timeout
        self.master_check_timer = self.create_timer(1.0, self.check_master_alive)

        # Timer to check for slave heartbeats and maintain the active_slaves list
        self.slave_check_timer = self.create_timer(2.0, self.check_slave_alive)

        # Initialize the master role flag
        self.is_master = False

        # Initialize the graph partitioning and waypoint assignment if this node becomes master
        self.master_graph_partitioned = False

    def publish_registration(self):
        """
        Publishes a registration message to the master to indicate that this slave is active.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published registration.")

    def publish_initial_position(self):
        """
        Publishes the initial position of the slave to the master.
        """
        initial_position = {
            'robot_namespace': self.robot_namespace,
            'x': self.initial_x,
            'y': self.initial_y,
            'orientation': self.initial_orientation_str  # Send orientation as a string
        }
        msg = String()
        msg.data = json.dumps(initial_position)
        self.initial_position_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Published initial position: {initial_position}")

    def publish_heartbeat(self):
        """
        Publishes a heartbeat message to indicate that this slave is active.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published heartbeat.")

    def master_heartbeat_callback(self, msg):
        """
        Callback function triggered when a master heartbeat message is received.
        Updates the master heartbeat timestamp.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received master heartbeat.")

    def slave_heartbeat_callback(self, msg):
        """
        Callback function triggered when a slave heartbeat message is received.
        Updates the active_slaves list.
        """
        slave_ns = msg.data.strip()
        current_time = time.time()
        if slave_ns != self.robot_namespace:
            self.active_slaves[slave_ns] = current_time
            self.get_logger().debug(f"[{self.robot_namespace}] Received heartbeat from slave {slave_ns}.")

    def check_master_alive(self):
        """
        Checks if the master is alive based on the last received heartbeat.
        If the master is considered dead, initiates master election.
        """
        current_time = time.time()
        if self.master_alive:
            # Reset the flag; it will be set again if a heartbeat is received
            self.master_alive = False
        else:
            # No heartbeat received since last check
            if current_time - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost. Initiating master election.")
                self.elect_new_master()

    def check_slave_alive(self):
        """
        Checks the heartbeats of other slaves and removes any that have timed out.
        """
        current_time = time.time()
        for slave_ns in list(self.active_slaves.keys()):
            if current_time - self.active_slaves[slave_ns] > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Slave {slave_ns} heartbeat lost. Removing from active slaves.")
                del self.active_slaves[slave_ns]

    def elect_new_master(self):
        """
        Elects a new master from the active slaves.
        The slave with the highest priority (e.g., earliest namespace) becomes the new master.
        """
        # Determine all candidates (including self)
        candidates = list(self.active_slaves.keys()) + [self.robot_namespace]

        if not candidates:
            self.get_logger().error(f"[{self.robot_namespace}] No candidates available for master election.")
            return

        # Sort candidates based on namespace (assuming lex order gives priority)
        candidates_sorted = sorted(candidates)

        # The first candidate in the sorted list becomes the new master
        new_master = candidates_sorted[0]

        if new_master == self.robot_namespace:
            self.get_logger().info(f"[{self.robot_namespace}] Elected as the new master.")
            self.become_master()
        else:
            self.get_logger().info(f"[{self.robot_namespace}] New master is {new_master}.")

    def become_master(self):
        """
        Transforms this slave into the master, performing all master duties.
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] Now acting as the master.")

        # Publish the navigation graph to the '/navigation_graph' topic
        if self.navigation_graph is not None:
            self.publish_navigation_graph()
            self.get_logger().info(f"[{self.robot_namespace}] Published navigation graph. Starting partitioning and waypoint assignment.")
            self.partition_and_assign_waypoints()
        else:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot become master.")

    def publish_navigation_graph(self):
        """
        Publishes the navigation graph on the '/navigation_graph' topic.
        """
        graph_msg = String()
        graph_data = {
            'nodes': [
                {'label': node, 'x': data['x'], 'y': data['y'], 'orientation': data.get('orientation', 0.0)}
                for node, data in self.navigation_graph.nodes(data=True)
            ],
            'edges': [
                {'from': u, 'to': v, 'weight': data.get('weight', 1.0)}
                for u, v, data in self.navigation_graph.edges(data=True)
            ]
        }
        graph_msg.data = json.dumps(graph_data)
        self.status_publisher.publish(graph_msg)
        self.get_logger().info("Published navigation graph as master.")

    def navigation_graph_callback(self, msg):
        """
        Callback function triggered when a navigation graph message is received.
        Stores the graph data.
        """
        try:
            graph_data = json.loads(msg.data)
            self.navigation_graph = load_full_graph_from_data(graph_data)
            self.get_logger().info(f"[{self.robot_namespace}] Received navigation graph.")
            if self.is_master and not self.master_graph_partitioned:
                self.partition_and_assign_waypoints()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation graph: {e}")

    def navigation_commands_callback(self, msg):
        """
        Callback function triggered when a new waypoint message is received from the master.
        """
        waypoint_data = json.loads(msg.data)
        self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")
        self.execute_navigation(waypoint_data)

    def execute_navigation(self, waypoint):
        """
        Uses the TurtleBot4Navigator to move the robot to the specified waypoint.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']
        orientation_str = waypoint['orientation']
        orientation = self.orientation_conversion(orientation_str)

        # Log the navigation task
        self.get_logger().info(f"[{self.robot_namespace}] Navigating to {label} at ({x}, {y}) with orientation {orientation_str} ({orientation} radians).")

        # Create a goal pose using the navigator
        goal_pose = self.navigator.getPoseStamped([x, y], orientation)

        # Start the timer for navigation
        self.start_time = time.time()

        try:
            # Check if the action server is available
            if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                error_message = f"Action server not available for {label}."
                self.get_logger().error(error_message)
                self.publish_status("error", error_message, 0.0, label)
                return

            # Start navigation towards the goal pose
            self.navigator.startToPose(goal_pose)
        except Exception as e:
            # If an exception occurs, log an error and publish the status to the master
            error_message = f"Exception occurred while sending goal to {label}: {e}"
            self.get_logger().error(error_message)
            self.publish_status("error", error_message, 0.0, label)
            return

        # Wait until navigation is complete
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            # Optionally, add checks for preemption or timeout here

        # Calculate the time taken for navigation
        time_taken = time.time() - self.start_time

        # Check the result of the navigation
        nav_result = self.navigator.getResult()

        if nav_result == TaskResult.SUCCEEDED:
            # Navigation succeeded
            self.get_logger().info(f"[{self.robot_namespace}] Reached {label} in {time_taken:.2f} seconds.")
            self.publish_status("reached", "", time_taken, label)
        else:
            # Navigation failed
            error_message = f"Navigation to {label} failed with result code {nav_result}."
            self.get_logger().error(error_message)
            self.publish_status("error", error_message, time_taken, label)

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        """
        Publishes the navigation status to the master.
        """
        status_data = {
            'robot_namespace': self.robot_namespace,
            'status': status,
            'error_message': error_message,
            'time_taken': time_taken,
            'current_waypoint': current_waypoint
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Published status: {status_data}")

    def orientation_conversion(self, orientation_str):
        """
        Converts an orientation string to radians.

        Args:
            orientation_str (str): Orientation as a string ('NORTH', 'EAST', 'SOUTH', 'WEST').

        Returns:
            float: Orientation in radians.
        """
        orientation_map = {
            "NORTH": 0.0,
            "EAST": -math.pi / 2,
            "SOUTH": math.pi,
            "WEST": math.pi / 2
        }
        return orientation_map.get(orientation_str.upper(), 0.0)

    def partition_and_assign_waypoints(self):
        """
        Partitions the navigation graph and assigns waypoints to slaves.
        This method is called if the slave assumes the master role.
        """
        if self.navigation_graph is None:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot partition and assign waypoints.")
            return

        # Load the graph from the received data
        full_graph = load_full_graph_from_data(self.navigation_graph)

        # Retrieve all active slaves, including self
        all_slaves = list(self.active_slaves.keys()) + [self.robot_namespace]
        all_slaves_sorted = sorted(all_slaves)  # Sort based on namespace for priority

        num_slaves = len(all_slaves_sorted)

        # Collect initial positions of slaves
        start_positions = []
        for slave_ns in all_slaves_sorted:
            if slave_ns == self.robot_namespace:
                start_positions.append({'x': self.initial_x, 'y': self.initial_y})
            else:
                # In a full implementation, retrieve other slaves' initial positions
                # For simplicity, assign the master its own initial position for now
                # You should implement a mechanism to gather other slaves' positions
                # e.g., via a shared parameter server or a dedicated topic
                start_positions.append({'x': self.initial_x, 'y': self.initial_y})

        # Partition the graph into subgraphs based on the number of slaves and their initial positions
        try:
            subgraphs = partition_graph(full_graph, num_slaves, start_positions=start_positions)
            self.get_logger().info(f"[{self.robot_namespace}] Partitioned the graph into {len(subgraphs)} subgraphs.")
        except ValueError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to partition graph: {e}")
            return

        # Assign each subgraph to a slave
        for idx, slave_ns in enumerate(all_slaves_sorted):
            subgraph = subgraphs[idx]
            waypoints = self.extract_waypoints(subgraph)

            # For simplicity, assuming that the master has all the necessary data to assign waypoints
            if slave_ns == self.robot_namespace:
                # Assign waypoints to self as master
                self.assign_route_to_master(waypoints)
            else:
                # Implement a method to send waypoints to other slaves
                # For demonstration, we'll skip this part
                pass

        self.master_graph_partitioned = True

    def assign_route_to_master(self, waypoints):
        """
        Assigns a waypoint route to itself as the new master.

        Args:
            waypoints (list of dict): List of waypoints.
        """
        # Calculate the DCPP (Eulerian Circuit) route
        dcpp_route = calculate_dcpp_route(waypoints, self.navigation_graph, self.get_logger())

        self.assigned_waypoints = dcpp_route.copy()
        self.current_waypoint_index = 0

        # Detailed logging of the assigned route
        self.get_logger().info(f"[{self.robot_namespace}] DCPP route assigned as master:")
        for wp in dcpp_route:
            self.get_logger().info(f" - {wp['label']} at ({wp['x']}, {wp['y']}) with orientation {wp['orientation']} radians")

        # Assign the first waypoint
        self.assign_next_waypoint_as_master()

        # Log the number of assigned waypoints
        self.get_logger().info(f"[{self.robot_namespace}] Assigned {len(dcpp_route)} waypoints as master.")

    def assign_next_waypoint_as_master(self):
        """
        Assigns the next waypoint in the queue to itself as the master.
        """
        if self.current_waypoint_index < len(self.assigned_waypoints):
            waypoint = self.assigned_waypoints[self.current_waypoint_index]
            waypoint_msg = {
                'label': waypoint['label'],
                'x': waypoint['x'],
                'y': waypoint['y'],
                'orientation': orientation_conversion(waypoint['orientation'])
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            self.status_publisher.publish(msg)
            self.get_logger().info(f"[{self.robot_namespace}] Assigned waypoint as master: {waypoint_msg}")
        else:
            # All waypoints have been assigned, restart from the first
            self.get_logger().info(f"[{self.robot_namespace}] All waypoints have been assigned. Restarting the route.")
            self.current_waypoint_index = 0
            self.assign_next_waypoint_as_master()

    def extract_waypoints(self, subgraph):
        """
        Extracts waypoints from a subgraph.

        Args:
            subgraph (nx.Graph): Subgraph from which to extract waypoints.

        Returns:
            list of dict: List of waypoints with 'label', 'x', 'y', and 'orientation'.
        """
        waypoints = []
        for node, data in subgraph.nodes(data=True):
            waypoint = {
                'label': node,
                'x': data['x'],
                'y': data['y'],
                'orientation': data.get('orientation', 0.0)
            }
            waypoints.append(waypoint)
        return waypoints

def load_full_graph_from_data(graph_data):
    """
    Loads a NetworkX graph from a dictionary containing nodes and edges.

    Args:
        graph_data (dict): Dictionary with 'nodes' and 'edges'.

    Returns:
        nx.DiGraph: The loaded directed graph.
    """
    G = nx.DiGraph()

    for node in graph_data['nodes']:
        label = node['label']
        x = node['x']
        y = node['y']
        orientation = node.get('orientation', 0.0)
        G.add_node(label, x=x, y=y, orientation=orientation)

    for edge in graph_data['edges']:
        u = edge['from']
        v = edge['to']
        weight = edge.get('weight', 1.0)
        G.add_edge(u, v, weight=weight)

    return G

def main(args=None):
    parser = argparse.ArgumentParser(description='Slave Navigation Node using TurtleBot4 with Master Replacement')
    parser.add_argument('--robot_namespace', type=str, required=True, help='Unique namespace of the robot (e.g., robot_1)')
    parser.add_argument('--initial_x', type=float, required=True, help='Initial x coordinate')
    parser.add_argument('--initial_y', type=float, required=True, help='Initial y coordinate')
    parser.add_argument('--initial_orientation', type=str, required=True, help='Initial orientation (NORTH, EAST, SOUTH, WEST)')

    # Parse command-line arguments
    parsed_args, unknown = parser.parse_known_args()

    # Initialize ROS 2
    rclpy.init(args=args)

    # Create an instance of the SlaveNavigationNode
    node = SlaveNavigationNode(
        robot_namespace=parsed_args.robot_namespace,
        initial_x=parsed_args.initial_x,
        initial_y=parsed_args.initial_y,
        initial_orientation_str=parsed_args.initial_orientation
    )

    try:
        # Keep the node active and listening for callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node and shut down ROS 2
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
