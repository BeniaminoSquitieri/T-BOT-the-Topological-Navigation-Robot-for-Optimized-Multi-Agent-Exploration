#!/usr/bin/env python3

import rclpy  # The ROS 2 Python client library
from rclpy.node import Node  # Base class for creating a ROS 2 node in Python
from std_msgs.msg import String  # Standard ROS message type for transmitting string data
import json  # For encoding/decoding JSON messages (graph data, statuses, positions)
import time  # For timestamps and measuring intervals (e.g., last heartbeat time)
import argparse  # For parsing command-line arguments passed at node startup
import math  # For mathematical operations (e.g., pi, trigonometric functions)
import threading  # For running certain tasks (like simulated navigation) in parallel threads
import networkx as nx  # For creating and handling complex graphs (nodes, edges, attributes)

# Importing custom helper functions from local modules
# These modules handle graph partitioning and route calculation (DCPP)
from .graph_partitioning import load_full_graph, partition_graph
from .path_calculation import calculate_dcpp_route, orientation_rad_to_str

class SlaveState:
    """
    This class stores the state of a single slave robot from the master's perspective.

    Attributes:
        slave_ns (str): The unique namespace of the slave (e.g., "robot_1").
        assigned_waypoints (list): List of waypoints assigned to this slave.
        current_waypoint_index (int): Indicates which waypoint the slave is currently following.
        last_seen_time (float): Timestamp of the last communication (e.g., heartbeat) from this slave.
        initial_x (float): Initial X coordinate of the slave (once known).
        initial_y (float): Initial Y coordinate of the slave (once known).
        initial_orientation (str): Initial orientation as a string (e.g., "NORTH").
        publisher: ROS Publisher to send navigation commands to this slave.
        waiting (bool): Flag indicating if this slave is waiting for an available waypoint.
    """
    def __init__(self, slave_ns):
        self.slave_ns = slave_ns
        self.assigned_waypoints = []
        self.current_waypoint_index = 0
        self.last_seen_time = 0.0
        self.initial_x = None
        self.initial_y = None
        self.initial_orientation = None
        self.publisher = None
        self.waiting = False

class SlaveNavigationSimulator(Node):
    """
    A simulated slave node that mimics the behavior of a real TurtleBot4 robot.

    This node:
    - Registers with the master to signal its presence.
    - Waits for the navigation graph from the master and determines its initial coordinates.
    - Publishes the initial position once coordinates are known.
    - Receives navigation commands (waypoints) and simulates navigation.
    - Publishes navigation status (reached, error) to the master.
    - Sends and receives heartbeat messages to monitor the presence of other slaves and the master.
    - If the master is lost, participates in an election to become the new master.
    - If it becomes master, partitions the navigation graph and assigns waypoints to all slaves.

    Additionally, it now implements continuous publishing of the initial position even after assuming the role of master.
    """

    def __init__(self, robot_namespace, initial_node_label, initial_orientation_str):
        # Store the provided parameters:
        # robot_namespace: unique name for this robot (e.g., "robot_simulator")
        # initial_node_label: label of the node in the navigation graph where the robot starts
        # initial_orientation_str: initial orientation as a string (NORTH, EAST, SOUTH, WEST)
        self.robot_namespace = robot_namespace
        self.initial_node_label = initial_node_label
        self.initial_orientation_str = initial_orientation_str

        # Convert the orientation string to radians for internal representation
        self.initial_orientation = self.orientation_conversion(initial_orientation_str)

        # Initially, we do not know the initial coordinates (x, y) since we only have the node label
        self.initial_x = None
        self.initial_y = None

        # Initialize the ROS node with the provided namespace
        super().__init__('slave_navigation_simulator_node', namespace=self.robot_namespace)

        # Create the necessary publishers and subscribers for communication with the master and other slaves:

        # Publisher to register with the master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)

        # Publisher to send the initial position once known
        self.initial_position_publisher = self.create_publisher(String, '/slave_initial_positions', 10)

        # Subscriber to receive navigation commands (waypoints) from the master
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.navigation_commands_callback, 10
        )

        # Publisher to send navigation status updates to the master
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Publisher to send heartbeat messages indicating that this slave is active
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)

        # Subscriber to receive heartbeat messages from the master
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 10
        )

        # Subscriber to receive heartbeat messages from other slaves
        self.slave_heartbeat_subscriber = self.create_subscription(
            String, '/slave_heartbeat', self.slave_heartbeat_callback, 10
        )

        # Subscriber to receive the navigation graph from the master
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.navigation_graph_callback, 10
        )

        # Publisher to send the navigation graph if this node becomes master
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)

        # Subscriber to receive the initial positions of the slaves (NEW FEATURE)
        self.slave_initial_position_subscriber = self.create_subscription(
            String, '/slave_initial_positions', self.slave_initial_position_callback, 10
        )

        # Create timers for periodic tasks:

        # 1. Periodically publish the registration
        self.registration_timer = self.create_timer(1.0, self.publish_registration)

        # 2. Periodically publish the heartbeat
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # 3. Attempt to publish the initial position every 2 seconds until successful
        self.initial_position_timer = self.create_timer(2.0, self.try_publish_initial_position)

        # 4. Periodically publish the initial position
        self.periodic_position_publisher_timer = self.create_timer(5.0, self.periodic_position_publish)

        # Variables to detect the presence of the master:
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 5.0  # After 5 seconds without master heartbeat, consider the master lost

        # Dictionary of active slaves. Key: slave namespace, Value: instance of SlaveState
        self.active_slaves = {}

        # The navigation graph will be set once received
        self.navigation_graph = None

        # Set of occupied nodes, used if assigning waypoints and considering nodes as "occupied"
        self.occupied_nodes = set()

        # Flag to indicate if we have already partitioned the graph and assigned waypoints
        self.partitioning_done = False

        # Current waypoint index for this slave (if acting as master or has an assigned route)
        self.current_waypoint_index = 0

        # Flag to track if the initial position has been published
        self.initial_position_published = False

        # Flag to indicate if this node is the master
        self.is_master = False

        # Lock to manage concurrent access to shared data structures like self.active_slaves
        self.lock = threading.Lock()

        # Subscriber for navigation status (necessary if becoming master)
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )

        # Publisher for master heartbeats (initialized to None; will be created when becoming master)
        self.master_heartbeat_publisher = None  # # CHANGES

        # Timer for publishing master heartbeats (initialized to None; will be created when becoming master)
        self.master_heartbeat_timer = None  # # CHANGES

        # Log the initialization information
        self.get_logger().info(
            f"[{self.robot_namespace}] Slave simulator initialized with initial node label '{self.initial_node_label}' "
            f"and orientation {self.initial_orientation_str} ({self.initial_orientation} radians)."
        )

        # Timer to check the health of the master and slaves:
        # Check if the master is still active every 10 seconds
        self.master_check_timer = self.create_timer(10.0, self.check_master_alive)

        # Check the heartbeats of other slaves every 2 seconds
        self.slave_check_timer = self.create_timer(2.0, self.check_slave_alive)

    def try_publish_initial_position(self):
        if self.initial_x is not None and self.initial_y is not None and not self.initial_position_published:
            # Now we know x and y, and have not yet published the initial position
            initial_position = {
                'robot_namespace': self.robot_namespace,
                'x': self.initial_x,
                'y': self.initial_y,
                'orientation': self.initial_orientation_str
            }
            msg = String()
            msg.data = json.dumps(initial_position)
            self.initial_position_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Initial position published: {initial_position}")
            self.initial_position_published = True
            # Cancel the timer to stop attempting
            self.initial_position_timer.cancel()

            # Confirmation log
            self.get_logger().info(f"[{self.robot_namespace}] Initial position published correctly.")
        else:
            self.get_logger().debug(f"[{self.robot_namespace}] Waiting for the initial position to be set.")

    def periodic_position_publish(self):
        """
        Periodically publishes the initial position on '/slave_initial_positions'.
        This ensures that the position is regularly updated, even after election as master.
        """
        if self.initial_x is not None and self.initial_y is not None:
            orientation_str = orientation_rad_to_str(self.initial_orientation)
            position = {
                'robot_namespace': self.robot_namespace,
                'x': self.initial_x,
                'y': self.initial_y,
                'orientation': orientation_str
            }
            msg = String()
            msg.data = json.dumps(position)
            self.initial_position_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Initial position published periodically: {position}")

    def publish_registration(self):
        """
        Periodically publishes a registration message to the master.
        This ensures that the master knows that this slave is active and available.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Registration published.")

    def publish_heartbeat(self):
        """
        Periodically publishes a heartbeat message to signal that this slave is alive.
        The master and other slaves track these messages to detect if a slave goes offline.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Heartbeat published.")

    def master_heartbeat_callback(self, msg):
        """
        Callback invoked when a heartbeat from the master is received.
        Marks the master as alive and updates the timestamp of the last heartbeat.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Master heartbeat received.")

    def slave_heartbeat_callback(self, msg):
        """
        Callback invoked when a heartbeat from another slave is received.
        Updates the record of active slaves and their last_seen_time.
        This is useful if we need to become master later.
        """
        slave_ns = msg.data.strip()
        current_time = time.time()
        if slave_ns != self.robot_namespace:
            with self.lock:
                if slave_ns not in self.active_slaves:
                    self.active_slaves[slave_ns] = SlaveState(slave_ns)
                    self.get_logger().info(f"[{self.robot_namespace}] Detected slave: {slave_ns}")
                self.active_slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"[{self.robot_namespace}] Heartbeat received from slave {slave_ns}.")

    def slave_initial_position_callback(self, msg):
        """
        Callback to receive the initial positions of the slaves.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            x = data['x']
            y = data['y']
            orientation = data['orientation']
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"[{self.robot_namespace}] Invalid slave initial position message: {e}")
            return

        # Ignore the initial position of the master if published on /slave_initial_positions
        if slave_ns == self.robot_namespace:
            return

        with self.lock:
            if slave_ns not in self.active_slaves:
                self.active_slaves[slave_ns] = SlaveState(slave_ns)
                self.get_logger().info(f"[{self.robot_namespace}] Detected slave: {slave_ns}")
            slave = self.active_slaves[slave_ns]
            slave.initial_x = x
            slave.initial_y = y
            slave.initial_orientation = orientation
            self.get_logger().debug(f"[{self.robot_namespace}] Updated initial position for {slave_ns}: ({x}, {y}, {orientation})")

    def check_master_alive(self):
        """
        Periodically checks if the master is still alive.
        If no master heartbeat is received within the timeout, initiates an election for a new master.
        """
        current_time = time.time()
        if self.master_alive:
            # Reset the flag; it will be set again if a new heartbeat arrives
            self.master_alive = False
        else:
            # No heartbeat since the last check
            if current_time - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost. Initiating master election.")
                self.elect_new_master()

    def check_slave_alive(self):
        """
        Periodically checks the heartbeats of other slaves.
        Removes those that have not communicated within the timeout.
        """
        current_time = time.time()
        with self.lock:
            for slave_ns in list(self.active_slaves.keys()):
                if current_time - self.active_slaves[slave_ns].last_seen_time > self.heartbeat_timeout:
                    self.get_logger().warn(f"[{self.robot_namespace}] Heartbeat lost from slave {slave_ns}. Removing from active slaves.")
                    del self.active_slaves[slave_ns]

    def elect_new_master(self):
        """
        If the master is lost, elects a new master from all active slaves and this node.
        The one with the lexicographically smallest namespace wins.
        """
        with self.lock:
            candidates = list(self.active_slaves.keys()) + [self.robot_namespace]

        if not candidates:
            self.get_logger().error(f"[{self.robot_namespace}] No candidates available for master election.")
            return

        candidates_sorted = sorted(candidates)
        new_master = candidates_sorted[0]

        if new_master == self.robot_namespace:
            self.get_logger().info(f"[{self.robot_namespace}] Elected as the new master.")
            self.become_master()
        else:
            self.get_logger().info(f"[{self.robot_namespace}] The new master is {new_master}.")

    def become_master(self):
        """
        Transforms this slave into the master.
        As master, it publishes the navigation graph, partitions the graph, and assigns waypoints to all slaves.
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] Now acting as master.")

        # Unsubscribe from master heartbeat
        if self.master_heartbeat_subscriber:
            self.destroy_subscription(self.master_heartbeat_subscriber)
            self.master_heartbeat_subscriber = None
            self.get_logger().info(f"[{self.robot_namespace}] Unsubscribed from /master_heartbeat.")

        # Create the publisher for master heartbeats
        self.master_heartbeat_publisher = self.create_publisher(String, '/master_heartbeat', 10)
        self.get_logger().info(f"[{self.robot_namespace}] Created publisher for /master_heartbeat.")

        # Create a timer to publish master heartbeats
        self.master_heartbeat_timer = self.create_timer(1.0, self.publish_master_heartbeat)
        self.get_logger().info(f"[{self.robot_namespace}] Started timer for master heartbeats.")

        # Publish the navigation graph immediately
        if self.navigation_graph is not None:
            self.publish_navigation_graph()
            self.get_logger().info(f"[{self.robot_namespace}] Published navigation graph. Starting partitioning and waypoint assignment.")
            self.partition_and_assign_waypoints()
        else:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot become master.")

    def publish_master_heartbeat(self):
        """
        Publishes heartbeat messages on /master_heartbeat to signal that this node is the master.
        """
        if self.is_master and self.master_heartbeat_publisher:
            heartbeat_msg = String()
            heartbeat_msg.data = self.robot_namespace  # Master identifier
            self.master_heartbeat_publisher.publish(heartbeat_msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Master heartbeat published.")

    def publish_navigation_graph(self):
        """
        If we are the master, publishes the navigation graph on '/navigation_graph'.
        This allows all slaves to know the layout of the environment.
        """
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
        msg = String()
        msg.data = json.dumps(graph_data)
        self.graph_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Navigation graph published.")

    def navigation_graph_callback(self, msg):
        """
        Callback when the navigation graph is received from the master.
        Once received, we can determine the initial coordinates from the initial node.
        """
        try:
            graph_data = json.loads(msg.data)
            self.navigation_graph = self.load_full_graph_from_data(graph_data)
            self.get_logger().debug(f"[{self.robot_namespace}] Navigation graph received.")

            # Find the coordinates of the initial node label
            if self.initial_node_label in self.navigation_graph.nodes:
                node_data = self.navigation_graph.nodes[self.initial_node_label]
                self.initial_x = node_data['x']
                self.initial_y = node_data['y']
                # Do not publish the initial position here; rely on the timer that periodically attempts
                self.get_logger().info(f"[{self.robot_namespace}] Initial coordinates determined: ({self.initial_x}, {self.initial_y})")

            # If already master and have not yet partitioned, partition now
            if self.is_master and not self.partitioning_done:
                self.partition_and_assign_waypoints()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph decoding failed: {e}")

    def navigation_commands_callback(self, msg):
        """
        Callback when a navigation command (waypoint) is received.
        Parses the waypoint and starts simulating navigation towards it.
        """
        self.get_logger().debug(f"[{self.robot_namespace}] Navigation command received: {msg.data}")
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation command decoding failed: {e}")
            return

        # Convert orientation if provided as a string
        if isinstance(waypoint_data.get('orientation'), str):
            waypoint_data['orientation'] = self.orientation_conversion(waypoint_data['orientation'])

        # Log to verify to whom the waypoint is destined
        if waypoint_data.get('robot_namespace') == self.robot_namespace:
            self.get_logger().debug(f"[{self.robot_namespace}] Waypoint destined for master itself: {waypoint_data}")
        else:
            self.get_logger().debug(f"[{self.robot_namespace}] Waypoint destined for slave {waypoint_data.get('robot_namespace')}: {waypoint_data}")

        # Start navigation simulation in a separate thread to avoid blocking the main loop
        threading.Thread(target=self.simulate_navigation, args=(waypoint_data,)).start()
        self.get_logger().debug(f"[{self.robot_namespace}] Started navigation thread for waypoint: {waypoint_data}")

    def simulate_navigation(self, waypoint):
        """
        Simulates navigation towards the given waypoint.
        For testing purposes, sleeps for a fixed time and then reports success.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']
        orientation_rad = waypoint['orientation']

        # Add a log to confirm receipt of the waypoint
        self.get_logger().debug(f"[{self.robot_namespace}] Starting navigation simulation towards {label}: ({x}, {y}, {orientation_rad})")

        self.get_logger().info(f"[{self.robot_namespace}] Simulating navigation towards {label} at ({x}, {y}) with orientation {orientation_rad} radians.")

        # Simulate travel time, e.g., 5 seconds
        simulated_navigation_time = 5.0
        self.get_logger().debug(f"[{self.robot_namespace}] Starting navigation simulation for {label} (duration {simulated_navigation_time} seconds).")
        time.sleep(simulated_navigation_time)

        # Assume success in navigation for simplicity
        nav_success = True

        if nav_success:
            self.get_logger().info(f"[{self.robot_namespace}] Reached {label} in {simulated_navigation_time} seconds.")
            self.publish_status("reached", "", simulated_navigation_time, label)
            # Publish the updated position after reaching the waypoint
            self.update_and_publish_position(x, y, orientation_rad)
            if self.is_master:
                # If we are master, handle assigning the next waypoint to ourselves
                with self.lock:
                    self.current_waypoint_index += 1
                with self.lock:
                    if label in self.occupied_nodes:
                        self.occupied_nodes.remove(label)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {label} is now free.")
                # Assign the next waypoint
                self.assign_next_waypoint(self.robot_namespace)
                # Assign waypoints to waiting slaves
                self.assign_waiting_slaves()
        else:
            # Simulate an error case if necessary
            error_message = f"Navigation simulation to {label} failed."
            self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
            self.publish_status("error", error_message, simulated_navigation_time, label)

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        """
        Publishes the navigation status to the master to inform it of the result of the last navigation attempt.
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
        self.get_logger().info(f"[{self.robot_namespace}] Status published: {status_data}")

    def update_and_publish_position(self, x, y, orientation_rad):
        """
        Updates the current position and publishes it on the '/slave_initial_positions' topic.
        """
        self.initial_x = x
        self.initial_y = y
        self.initial_orientation = orientation_rad

        orientation_str = orientation_rad_to_str(orientation_rad)

        position = {
            'robot_namespace': self.robot_namespace,
            'x': self.initial_x,
            'y': self.initial_y,
            'orientation': orientation_str
        }
        msg = String()
        msg.data = json.dumps(position)
        self.initial_position_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Updated position published: {position}")

    def navigation_status_callback(self, msg):
        """
        Handles navigation status feedback from slaves if we become master.
        Updates their states and possibly reassigns waypoints.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_waypoint = data['current_waypoint']
            time_taken = data['time_taken']
            error_message = data.get('error_message', '')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"[{self.robot_namespace}] Invalid navigation status message: {e}")
            return

        current_time = time.time()

        if self.is_master:
            with self.lock:
                if slave_ns in self.active_slaves:
                    slave = self.active_slaves[slave_ns]
                elif slave_ns == self.robot_namespace:
                    # It's ourselves (the master) reporting the status
                    slave = self
                else:
                    self.get_logger().warn(f"[{self.robot_namespace}] Status received from unknown slave {slave_ns}.")
                    return

                slave.last_seen_time = current_time

                if status == "reached":
                    # The slave has reached a waypoint, free the node and assign the next
                    if current_waypoint in self.occupied_nodes:
                        self.occupied_nodes.remove(current_waypoint)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {current_waypoint} is now free.")
                    else:
                        self.get_logger().warn(f"[{self.robot_namespace}] Node {current_waypoint} was not marked as occupied.")

                    self.get_logger().info(f"[{self.robot_namespace}] Slave {slave_ns} has reached waypoint {current_waypoint}.")
                    slave.waiting = False
                    self.assign_next_waypoint(slave_ns)
                    self.assign_waiting_slaves()

                elif status == "error":
                    # The slave encountered an error
                    self.get_logger().error(f"[{self.robot_namespace}] Slave {slave_ns} encountered an error: {error_message}")
                    if current_waypoint in self.occupied_nodes:
                        self.occupied_nodes.remove(current_waypoint)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {current_waypoint} is now free due to an error.")
                    if slave_ns in self.active_slaves:
                        del self.active_slaves[slave_ns]
                        self.get_logger().warn(f"[{self.robot_namespace}] Removed slave {slave_ns} due to error.")
                        self.partition_and_assign_waypoints()

    def partition_and_assign_waypoints(self):
        """
        Partitions the navigation graph among all active slaves (and this node if master).
        Assigns a DCPP route to each slave.
        This is done only if we are the master.
        """
        if self.navigation_graph is None:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot partition and assign waypoints.")
            return

        with self.lock:
            num_slaves = len(self.active_slaves) + 1  # Include self as master
            if num_slaves == 0:
                self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
                self.partitioning_done = False
                return

            # Gather the initial positions of all active slaves
            start_positions = []
            for slave in self.active_slaves.values():
                if slave.initial_x is not None and slave.initial_y is not None:
                    start_positions.append({'x': slave.initial_x, 'y': slave.initial_y})
                else:
                    self.get_logger().warn(f"Slave {slave.slave_ns} initial position not available.")

            # Add our own initial position
            if self.initial_x is None or self.initial_y is None:
                # If we haven't received the graph yet or something is wrong, cannot partition
                self.get_logger().error("Master's initial position not available. Cannot partition.")
                return

            start_positions.append({'x': self.initial_x, 'y': self.initial_y})

            if len(start_positions) != num_slaves:
                self.get_logger().error("Not all slaves have valid initial positions.")
                return

            # Partition the graph into balanced subgraphs based on the number of slaves
            try:
                subgraphs = partition_graph(self.navigation_graph, num_slaves, start_positions=start_positions)
                self.get_logger().info(f"Partitioned the graph into {len(subgraphs)} subgraphs.")
                self.print_subgraphs(subgraphs)
            except ValueError as e:
                self.get_logger().error(f"Graph partitioning failed: {e}")
                return

            # Create a sorted list of all slaves including the master
            all_slaves = list(self.active_slaves.keys()) + [self.robot_namespace]
            all_slaves_sorted = sorted(all_slaves)

            if len(subgraphs) != len(all_slaves_sorted):
                self.get_logger().error("Number of subgraphs does not match the number of active slaves.")
                return

            # Assign a DCPP route to each slave
            for idx, slave_ns in enumerate(all_slaves_sorted):
                subgraph = subgraphs[idx]
                waypoints = self.extract_waypoints(subgraph)
                dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.get_logger())
                ordered_route = dcpp_route

                self.get_logger().info(f"DCPP Route for {slave_ns}:")
                for wp in ordered_route:
                    self.get_logger().info(f"  {wp}")

                if slave_ns == self.robot_namespace:
                    # Assign the route to ourselves
                    self.assigned_waypoints = ordered_route
                    self.assign_next_waypoint(self.robot_namespace)
                else:
                    # Assign the route to other slaves
                    if slave_ns in self.active_slaves:
                        slave = self.active_slaves[slave_ns]
                        slave.assigned_waypoints = ordered_route
                        self.assign_next_waypoint(slave_ns)
                    else:
                        self.get_logger().warn(f"Slave {slave_ns} not found in active_slaves.")

            # Mark partitioning as done
            self.partitioning_done = True

    def assign_next_waypoint(self, slave_ns):
        """
        Assigns the next waypoint in the assigned route to the given slave.
        If the node is occupied, the slave waits until it is free.
        """
        with self.lock:
            if slave_ns == self.robot_namespace:
                # Assign to ourselves
                slave = self
            else:
                slave = self.active_slaves.get(slave_ns, None)

            if slave is None:
                self.get_logger().warn(f"Slave {slave_ns} not found.")
                return

            if len(slave.assigned_waypoints) == 0:
                self.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                return

            waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
            node_label = waypoint['label']

            if node_label in self.occupied_nodes:
                self.get_logger().warn(f"Node {node_label} is already occupied. Cannot assign to slave {slave_ns}.")
                slave.waiting = True
                return

            waypoint_msg = {
                'robot_namespace': slave_ns,  # Add the robot namespace
                'label': waypoint['label'],
                'x': waypoint['x'],
                'y': waypoint['y'],
                'orientation': orientation_rad_to_str(waypoint['orientation'])
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)

            if slave_ns == self.robot_namespace:
                # Assign to ourselves, simulate receiving the command directly
                self.navigation_commands_callback(msg)
                self.get_logger().info(f"[Master {self.robot_namespace}] Waypoint assigned to itself: {waypoint_msg}")
                # Add a log to confirm that the master has assigned the waypoint to itself
                self.get_logger().debug(f"[Master {self.robot_namespace}] Waypoint received: {waypoint_msg}")
            else:
                # Send the waypoint to the slave
                if slave.publisher is None:
                    slave.publisher = self.create_publisher(String, f'/{slave_ns}/navigation_commands', 10)
                slave.publisher.publish(msg)
                self.get_logger().info(f"[Master {self.robot_namespace}] Waypoint assigned to {slave_ns}: {waypoint_msg}")

            self.occupied_nodes.add(node_label)
            slave.current_waypoint_index += 1
            if slave.current_waypoint_index >= len(slave.assigned_waypoints):
                slave.current_waypoint_index = 0

    def assign_waiting_slaves(self):
        """
        Assigns waypoints to slaves that were waiting for a free node.
        Attempts to reassign if the node is now free.
        """
        with self.lock:
            candidates = list(self.active_slaves.keys()) + [self.robot_namespace]
            for slave_ns in sorted(candidates):
                if slave_ns == self.robot_namespace and self.is_master:
                    slave = self
                else:
                    slave = self.active_slaves.get(slave_ns, None)
                    if slave is None:
                        continue

                if slave.waiting:
                    if len(slave.assigned_waypoints) == 0:
                        self.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                        continue

                    waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
                    node_label = waypoint['label']

                    # Check if the node is now free
                    if node_label not in self.occupied_nodes:
                        # Occupy the node and send the waypoint to the slave
                        self.occupied_nodes.add(node_label)
                        slave.waiting = False
                        self.get_logger().info(f"Assigned node {node_label} to slave {slave_ns} (previously waiting).")

                        # Create the waypoint message
                        waypoint_msg = {
                            'robot_namespace': slave_ns,
                            'label': waypoint['label'],
                            'x': waypoint['x'],
                            'y': waypoint['y'],
                            'orientation': orientation_rad_to_str(waypoint['orientation'])
                        }
                        msg = String()
                        msg.data = json.dumps(waypoint_msg)

                        if slave_ns == self.robot_namespace:
                            # Assign to ourselves, simulate receiving the command directly
                            self.navigation_commands_callback(msg)
                            self.get_logger().info(f"[Master {self.robot_namespace}] Waypoint assigned to itself: {waypoint_msg}")
                            # Add a log to confirm that the master has assigned the waypoint to itself
                            self.get_logger().debug(f"[Master {self.robot_namespace}] Waypoint received: {waypoint_msg}")
                        else:
                            # Send the waypoint to the slave
                            if slave.publisher is None:
                                slave.publisher = self.create_publisher(String, f'/{slave_ns}/navigation_commands', 10)
                            slave.publisher.publish(msg)
                            self.get_logger().info(f"[Master {self.robot_namespace}] Waypoint assigned to {slave_ns}: {waypoint_msg}")

                        slave.current_waypoint_index += 1
                        if slave.current_waypoint_index >= len(slave.assigned_waypoints):
                            slave.current_waypoint_index = 0
                    else:
                        self.get_logger().warn(f"Node {node_label} is still occupied. Slave {slave_ns} remains waiting.")

    def print_subgraphs(self, subgraphs):
        """
        Logs details of each subgraph created during graph partitioning.

        This function provides a detailed view of the nodes and edges in each subgraph, which is useful for debugging.

        Args:
            subgraphs (list of nx.Graph): A list of subgraphs created during partitioning.
        """
        self.get_logger().info("----- Subgraphs After Partition -----")
        
        # Iterate over each subgraph and log its details
        for idx, subgraph in enumerate(subgraphs):
            self.get_logger().info(f"Subgraph {idx+1}:")
            self.get_logger().info(f"  Nodes ({len(subgraph.nodes())}):")
            
            # Log each node with its properties
            for node, data in subgraph.nodes(data=True):
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                orientation = data.get('orientation', 0.0)
                self.get_logger().info(f"    {node}: Position=({x}, {y}), Orientation={orientation} radians")
            
            self.get_logger().info(f"  Edges ({len(subgraph.edges())}):")
            
            # Log each edge with its properties
            for u, v, data in subgraph.edges(data=True):
                weight = data.get('weight', 1.0)
                self.get_logger().info(f"    From {u} to {v}, Weight: {weight}")
        
        self.get_logger().info("----- End of Subgraphs -----")

    def extract_waypoints(self, subgraph):
        """
        Extracts the waypoints (nodes) from a subgraph as a list of dictionaries.
        Each dictionary includes label, x, y, and orientation.
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

    def orientation_conversion(self, orientation_input):
        """
        Converts an orientation given as a string (NORTH, EAST, SOUTH, WEST) or float to radians.
        Defaults to 0 if unrecognized.
        """
        if isinstance(orientation_input, str):
            orientation_map = {
                "NORTH": 0.0,
                "EAST": -math.pi / 2,
                "SOUTH": math.pi,
                "WEST": math.pi / 2
            }
            return orientation_map.get(orientation_input.upper(), 0.0)
        elif isinstance(orientation_input, (float, int)):
            return float(orientation_input)
        else:
            return 0.0

    def load_full_graph_from_data(self, graph_data):
        """
        Loads a directed graph (DiGraph) from a dictionary with nodes and edges.
        Used when we receive the navigation graph as JSON.
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

    def assign_next_waypoint_to_self(self):
        """
        Timer callback to assign the next waypoint to the master itself.
        """
        if self.assigned_waypoints and not self.waiting:
            self.assign_next_waypoint(self.robot_namespace)

    
def main(args=None):
    """
    Main entry point.
    Parses arguments, initializes ROS, creates the node, and spins until interrupted.
    """
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Slave Navigation Simulator Node')
    parser.add_argument('--robot_namespace', type=str, default='robot_simulator', help='Robot namespace')
    parser.add_argument('--initial_node_label', type=str, default='node_1', help='Initial node label where the robot starts')
    parser.add_argument('--initial_orientation', type=str, default='NORTH', help='Initial orientation (NORTH, EAST, SOUTH, WEST)')

    # Ignore unknown ROS arguments
    args, unknown = parser.parse_known_args()

    # Create the node instance with the provided parameters
    node = SlaveNavigationSimulator(
        robot_namespace=args.robot_namespace,
        initial_node_label=args.initial_node_label,
        initial_orientation_str=args.initial_orientation
    )

    try:
        # Keep the node running and responding to callbacks until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up on exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
