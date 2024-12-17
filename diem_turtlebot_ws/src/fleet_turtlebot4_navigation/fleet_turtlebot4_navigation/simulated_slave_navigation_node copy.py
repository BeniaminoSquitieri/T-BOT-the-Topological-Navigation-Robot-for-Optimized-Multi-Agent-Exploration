#!/usr/bin/env python3

import rclpy  # The ROS 2 Python client library, providing the main APIs to create and handle nodes, publishers, etc.
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
    This class stores the state of a single slave robot from the perspective of the master.

    Attributes:
        slave_ns (str): The unique namespace of the slave (e.g., "robot_1").
        assigned_waypoints (list): List of waypoints (each a dict with x, y, label, orientation) assigned to this slave.
        current_waypoint_index (int): Tracks which waypoint the slave is currently going to.
        last_seen_time (float): Timestamp of the last communication (e.g., heartbeat) from this slave.
        initial_x (float): Initial X coordinate of the slave (once known).
        initial_y (float): Initial Y coordinate of the slave (once known).
        initial_orientation (float): Initial orientation in radians (once known).
        publisher: A ROS publisher to send navigation commands to this slave. Created when needed.
        waiting (bool): Flag indicating if this slave is waiting for a waypoint to become available (due to occupancy).
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
    A simulated slave node, mimicking the behavior of a real TurtleBot4-based navigation node.

    This node:
    - Registers itself with the master to signal its presence.
    - Waits for the navigation graph from the master, determines its initial (x,y) from the provided initial node label.
    - Publishes its initial position once the (x,y) are known.
    - Receives navigation commands (waypoints) and simulates navigation to them (no actual robot movement, just a timer).
    - Publishes navigation status (reached, error) to the master.
    - Sends and receives heartbeat messages to/from the master and other slaves to monitor their presence.
    - If the master is lost, can participate in an election process to become the new master.
    - If it becomes the master, partitions the navigation graph and assigns waypoints to all slaves.

    Unlike previous versions, we do not accept initial_x and initial_y directly.
    Instead, we take an initial_node_label and initial_orientation, and only after
    receiving the graph, we find the coordinates of that node.
    """

    def __init__(self, robot_namespace, initial_node_label, initial_orientation_str):
        # Store the given parameters:
        # robot_namespace: unique name for this robot (e.g., "robot_simulator")
        # initial_node_label: the label of the node in the graph where the robot starts
        # initial_orientation_str: initial orientation as a string (NORTH, EAST, SOUTH, WEST)
        self.robot_namespace = robot_namespace
        self.initial_node_label = initial_node_label
        self.initial_orientation_str = initial_orientation_str

        # Convert the orientation string to radians for internal representation.
        self.initial_orientation = self.orientation_conversion(initial_orientation_str)

        # At this stage, we do not know the initial (x,y) coordinates because we only have the node label.
        # We'll set them later once we have the graph.
        self.initial_x = None
        self.initial_y = None

        # Initialize the ROS node with the given namespace.
        # The node name is 'slave_navigation_simulator_node'.
        super().__init__('slave_navigation_simulator_node', namespace=self.robot_namespace)

        # Create publishers and subscribers needed for communication with the master and other slaves:

        # Publisher to register with the master (so the master knows this slave exists).
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)

        # Publisher to send initial position once known.
        self.initial_position_publisher = self.create_publisher(String, '/slave_initial_positions', 10)

        # Subscriber to receive navigation commands (waypoints) from the master.
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.navigation_commands_callback, 10
        )

        # Publisher to send navigation status updates to the master.
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Publisher to send heartbeat messages indicating this slave is alive.
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)

        # Subscriber to receive heartbeat messages from the master.
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 10
        )

        # Subscriber to receive heartbeat messages from other slaves.
        self.slave_heartbeat_subscriber = self.create_subscription(
            String, '/slave_heartbeat', self.slave_heartbeat_callback, 10
        )

        # Subscriber to receive the navigation graph from the master.
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.navigation_graph_callback, 10
        )

        # Publisher to send the navigation graph if this node becomes master.
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)

        # Create timers for periodic tasks:
        # 1. Publish registration periodically, so the master knows we are here.
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        # 2. Publish heartbeat periodically, so others know we are alive.
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        # 3. Try publishing initial position every 2 seconds until successful (once we know x,y).
        self.initial_position_timer = self.create_timer(2.0, self.try_publish_initial_position)

        # Variables for detecting master presence:
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 5.0  # After 5 seconds without master heartbeat, consider master lost.

        # Dictionary of active slaves. Key: slave namespace, Value: SlaveState instance.
        self.active_slaves = {}

        # The navigation graph will be set once we receive it.
        self.navigation_graph = None

        # Set of occupied nodes, used if we assign waypoints and consider nodes "occupied".
        self.occupied_nodes = set()

        # Flag to indicate if we have done graph partitioning and waypoint assignment.
        self.partitioning_done = False

        # Current waypoint index for this slave (if it ever acts as master or has assigned route).
        self.current_waypoint_index = 0

        # Flag to track if initial position was published.
        self.initial_position_published = False

        # Log the initialization details.
        self.get_logger().info(
            f"[{self.robot_namespace}] Slave simulator initialized with initial node label '{self.initial_node_label}' "
            f"and orientation {self.initial_orientation_str} ({self.initial_orientation} radians)."
        )

        # Timers to check the health of master and slaves:
        # Check if master is still alive every 10 seconds.
        self.master_check_timer = self.create_timer(10.0, self.check_master_alive)
        # Check other slaves' heartbeat every 2 seconds.
        self.slave_check_timer = self.create_timer(2.0, self.check_slave_alive)

        # Flag indicating if this node is the master.
        self.is_master = False

        # A lock to handle concurrent access to shared data structures like self.active_slaves.
        self.lock = threading.Lock()

        # Subscriber for navigation status (needed if we become master).
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )

    def try_publish_initial_position(self):
        """
        Attempts to publish the initial position to the master.
        We can only publish once we know the (x,y) coordinates of the initial node.
        Since we no longer take initial x,y from command line, we wait for the graph.
        Once we have the graph and thus know x,y, we publish once and stop this timer.
        """
        if self.initial_x is not None and self.initial_y is not None and not self.initial_position_published:
            # Now we know x and y, and we haven't published initial position yet.
            initial_position = {
                'robot_namespace': self.robot_namespace,
                'x': self.initial_x,
                'y': self.initial_y,
                'orientation': self.initial_orientation_str
            }
            msg = String()
            msg.data = json.dumps(initial_position)
            self.initial_position_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Published initial position: {initial_position}")
            self.initial_position_published = True
            # Cancel the timer so we don't keep trying.
            self.initial_position_timer.cancel()

    def publish_registration(self):
        """
        Periodically publish a registration message to the master.
        This ensures the master knows that this slave is active and available.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published registration.")

    def publish_heartbeat(self):
        """
        Periodically publish a heartbeat message to signal this slave is alive.
        The master and other slaves track these to detect if a slave goes offline.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published heartbeat.")

    def master_heartbeat_callback(self, msg):
        """
        Callback invoked when a heartbeat from the master is received.
        We mark the master as alive and update the timestamp of the last heartbeat.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received master heartbeat.")

    def slave_heartbeat_callback(self, msg):
        """
        Callback invoked when a heartbeat from another slave is received.
        We update our record of active slaves and their last_seen_time.
        This is useful if we become master later.
        """
        slave_ns = msg.data.strip()
        current_time = time.time()
        if slave_ns != self.robot_namespace:
            with self.lock:
                if slave_ns not in self.active_slaves:
                    self.active_slaves[slave_ns] = SlaveState(slave_ns)
                    self.get_logger().info(f"[{self.robot_namespace}] Detected new slave: {slave_ns}")
                self.active_slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"[{self.robot_namespace}] Received heartbeat from slave {slave_ns}.")

    def check_master_alive(self):
        """
        Periodically check if the master is alive.
        If no heartbeat from the master within the timeout, start a master election.
        """
        current_time = time.time()
        if self.master_alive:
            # Reset the flag; it will be set again if a new heartbeat arrives.
            self.master_alive = False
        else:
            # No heartbeat since the last check
            if current_time - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost. Initiating master election.")
                self.elect_new_master()

    def check_slave_alive(self):
        """
        Periodically check the heartbeats of other slaves.
        Remove those that have not communicated within the timeout.
        """
        current_time = time.time()
        with self.lock:
            for slave_ns in list(self.active_slaves.keys()):
                if current_time - self.active_slaves[slave_ns].last_seen_time > self.heartbeat_timeout:
                    self.get_logger().warn(f"[{self.robot_namespace}] Slave {slave_ns} heartbeat lost. Removing from active slaves.")
                    del self.active_slaves[slave_ns]

    def elect_new_master(self):
        """
        If the master is lost, elect a new master among all active slaves and this node.
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
            self.get_logger().info(f"[{self.robot_namespace}] New master is {new_master}.")

    def become_master(self):
        """
        Turn this slave into the master.
        As master, we will publish the graph, partition it, and assign waypoints to all slaves.
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] Now acting as the master.")

        if self.navigation_graph is not None:
            self.publish_navigation_graph()
            self.get_logger().info(f"[{self.robot_namespace}] Published navigation graph. Starting partitioning and waypoint assignment.")
            self.partition_and_assign_waypoints()
        else:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot become master.")

    def publish_navigation_graph(self):
        """
        If we are the master, publish the navigation graph on '/navigation_graph'.
        This allows all slaves to know the environment layout.
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
        self.get_logger().debug(f"[{self.robot_namespace}] Published navigation graph.")

    def navigation_graph_callback(self, msg):
        """
        Callback when we receive the navigation graph from the master.
        Once we have it, we can determine the initial (x,y) of our starting node.
        """
        try:
            graph_data = json.loads(msg.data)
            self.navigation_graph = self.load_full_graph_from_data(graph_data)
            self.get_logger().debug(f"[{self.robot_namespace}] Received navigation graph.")

            # Find the coordinates of the initial node label
            if self.initial_node_label in self.navigation_graph.nodes:
                node_data = self.navigation_graph.nodes[self.initial_node_label]
                self.initial_x = node_data['x']
                self.initial_y = node_data['y']
                # We don't publish initial position here, we rely on the timer that tries periodically.

            # If we become master before receiving the graph, we might partition now, but only if needed.
            if self.is_master and not self.partitioning_done:
                self.partition_and_assign_waypoints()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation graph: {e}")

    def navigation_commands_callback(self, msg):
        """
        Callback when we receive a navigation command (waypoint).
        We parse the waypoint and start simulating navigation to it.
        """
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation command: {e}")
            return

        # Convert orientation if provided as a string
        if isinstance(waypoint_data.get('orientation'), str):
            waypoint_data['orientation'] = self.orientation_conversion(waypoint_data['orientation'])

        # Start simulated navigation in a separate thread to avoid blocking the main loop.
        threading.Thread(target=self.simulate_navigation, args=(waypoint_data,)).start()

    def simulate_navigation(self, waypoint):
        """
        Simulate navigation to the given waypoint.
        For testing purposes, we just sleep for a fixed time and then report success or failure.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']
        orientation_rad = waypoint['orientation']

        self.get_logger().info(f"[{self.robot_namespace}] Simulating navigation to {label} at ({x}, {y}) with orientation {orientation_rad} radians.")

        # Simulate travel time of, say, 15 seconds.
        simulated_navigation_time = 15.0
        time.sleep(simulated_navigation_time)

        # Assume navigation success for simplicity.
        nav_success = True

        if nav_success:
            self.get_logger().info(f"[{self.robot_namespace}] Reached {label} in {simulated_navigation_time} seconds.")
            self.publish_status("reached", "", simulated_navigation_time, label)
            if self.is_master:
                # If master, handle waypoint assignment for itself.
                with self.lock:
                    self.current_waypoint_index += 1
                with self.lock:
                    if label in self.occupied_nodes:
                        self.occupied_nodes.remove(label)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {label} is now free.")
                # Assign the next waypoint
                self.assign_next_waypoint(self.robot_namespace)
                # Assign waiting slaves
                self.assign_waiting_slaves()
        else:
            # Simulate an error case if needed.
            error_message = f"Simulation of navigation to {label} failed."
            self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
            self.publish_status("error", error_message, simulated_navigation_time, label)

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        """
        Publish navigation status to the master so it knows the result of the last navigation attempt.
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

    def navigation_status_callback(self, msg):
        """
        Handle navigation status feedback from slaves if we become master.
        We update their states and possibly reassign waypoints.
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
                    # It's ourselves (the master) reporting status
                    slave = self
                else:
                    self.get_logger().warn(f"[{self.robot_namespace}] Received status from unknown slave {slave_ns}.")
                    return

                slave.last_seen_time = current_time

                if status == "reached":
                    # Slave reached a waypoint, free the node and assign the next one.
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
                    # Slave encountered an error
                    self.get_logger().error(f"[{self.robot_namespace}] Slave {slave_ns} encountered an error: {error_message}")
                    if current_waypoint in self.occupied_nodes:
                        self.occupied_nodes.remove(current_waypoint)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {current_waypoint} is now free due to error.")
                    if slave_ns in self.active_slaves:
                        del self.active_slaves[slave_ns]
                        self.get_logger().warn(f"[{self.robot_namespace}] Removing slave {slave_ns} due to error.")
                        self.partition_and_assign_waypoints()

    def partition_and_assign_waypoints(self):
        """
        Partition the navigation graph among all active slaves (and this node if master).
        Assign a DCPP route to each.
        This is only done if we are the master.
        """
        if self.navigation_graph is None:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot partition and assign waypoints.")
            return

        with self.lock:
            num_slaves = len(self.active_slaves) + 1  # Include ourself as master
            if num_slaves == 0:
                self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
                self.partitioning_done = False
                return

            # Collect initial positions. At this point, ideally we know initial_x, initial_y for all slaves.
            start_positions = []
            for slave_ns, slave in self.active_slaves.items():
                if slave.initial_x is not None and slave.initial_y is not None:
                    start_positions.append({'x': slave.initial_x, 'y': slave.initial_y})
                else:
                    self.get_logger().warn(f"Slave {slave_ns} initial position not available")

            # Add our own initial position
            if self.initial_x is None or self.initial_y is None:
                # If we haven't received the graph yet or something is off, can't partition
                self.get_logger().error("Master initial position not available, cannot partition.")
                return

            start_positions.append({'x': self.initial_x, 'y': self.initial_y})

            if len(start_positions) != num_slaves:
                self.get_logger().error("Not all slaves have valid initial positions.")
                return

            # Partition the graph
            try:
                subgraphs = partition_graph(self.navigation_graph, num_slaves, start_positions=start_positions)
                self.get_logger().info(f"Partitioned the graph into {len(subgraphs)} subgraphs.")
                self.print_subgraphs(subgraphs)
            except ValueError as e:
                self.get_logger().error(f"Failed to partition graph: {e}")
                return

            # Create a sorted list of all slaves including the master
            all_slaves = list(self.active_slaves.keys()) + [self.robot_namespace]
            all_slaves_sorted = sorted(all_slaves)

            if len(subgraphs) != len(all_slaves_sorted):
                self.get_logger().error("Number of subgraphs does not match number of active slaves.")
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
                    # Assign route to ourselves
                    self.assigned_waypoints = ordered_route
                    self.assign_next_waypoint(self.robot_namespace)
                else:
                    # Assign route to other slaves
                    if slave_ns in self.active_slaves:
                        slave = self.active_slaves[slave_ns]
                        slave.assigned_waypoints = ordered_route
                        self.assign_next_waypoint(slave_ns)
                    else:
                        self.get_logger().warn(f"Slave {slave_ns} not found in active_slaves.")

            self.partitioning_done = True

    def assign_next_waypoint(self, slave_ns):
        """
        Assign the next waypoint in the assigned route to the given slave.
        If the node is occupied, the slave waits until it's free.
        """
        with self.lock:
            if slave_ns == self.robot_namespace:
                # If assigning to ourselves
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
                'label': waypoint['label'],
                'x': waypoint['x'],
                'y': waypoint['y'],
                'orientation': orientation_rad_to_str(waypoint['orientation'])
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)

            if slave_ns == self.robot_namespace:
                # If assigning to ourselves, simulate receiving the command directly
                self.navigation_commands_callback(msg)
                self.get_logger().info(f"[Master {self.robot_namespace}] Assigned waypoint to itself: {waypoint_msg}")
            else:
                # Send the waypoint to the slave
                if slave.publisher is None:
                    slave.publisher = self.create_publisher(String, f'/{slave_ns}/navigation_commands', 10)
                slave.publisher.publish(msg)
                self.get_logger().info(f"[Master {self.robot_namespace}] Assigned waypoint to {slave_ns}: {waypoint_msg}")

            self.occupied_nodes.add(node_label)
            slave.current_waypoint_index += 1
            if slave.current_waypoint_index >= len(slave.assigned_waypoints):
                slave.current_waypoint_index = 0

    def assign_waiting_slaves(self):
        """
        Assign waypoints to slaves that were waiting for a node to become free.
        We attempt to assign again if the node is now free.
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

                    # Check if node is now free
                    if node_label not in self.occupied_nodes:
                        # Node is free, assign now
                        waypoint_msg = {
                            'label': waypoint['label'],
                            'x': waypoint['x'],
                            'y': waypoint['y'],
                            'orientation': orientation_rad_to_str(waypoint['orientation'])
                        }
                        msg = String()
                        msg.data = json.dumps(waypoint_msg)

                        if slave_ns == self.robot_namespace:
                            self.navigation_commands_callback(msg)
                            self.get_logger().info(f"[Master {self.robot_namespace}] Assigned waypoint to itself: {waypoint_msg}")
                        else:
                            if slave.publisher is None:
                                slave.publisher = self.create_publisher(String, f'/{slave_ns}/navigation_commands', 10)
                            slave.publisher.publish(msg)
                            self.get_logger().info(f"[Master {self.robot_namespace}] Assigned waypoint to {slave_ns}: {waypoint_msg}")

                        self.occupied_nodes.add(node_label)
                        slave.waiting = False
                        slave.current_waypoint_index += 1
                        if slave.current_waypoint_index >= len(slave.assigned_waypoints):
                            slave.current_waypoint_index = 0
                    else:
                        self.get_logger().warn(f"Node {node_label} is still occupied. Slave {slave_ns} remains in waiting state.")

    def print_subgraphs(self, subgraphs):
        """
        Print details of each subgraph after partitioning for debugging.
        This helps verify that partitioning worked as expected.
        """
        self.get_logger().info("----- Subgraphs After Partition -----")
        for idx, subgraph in enumerate(subgraphs):
            self.get_logger().info(f"Subgraph {idx+1}:")
            self.get_logger().info(f"  Nodes ({len(subgraph.nodes())}):")
            for node, data in subgraph.nodes(data=True):
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                orientation = data.get('orientation', 0.0)
                self.get_logger().info(f"    {node}: Position=({x}, {y}), Orientation={orientation} radians")
            self.get_logger().info(f"  Edges ({len(subgraph.edges())}):")
            for u, v, data in subgraph.edges(data=True):
                weight = data.get('weight', 1.0)
                self.get_logger().info(f"    From {u} to {v}, Weight: {weight}")
        self.get_logger().info("----- End of Subgraphs -----")

    def extract_waypoints(self, subgraph):
        """
        Extract waypoints (nodes) from a subgraph as a list of dictionaries.
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
        Convert an orientation given as a string (NORTH, EAST, SOUTH, WEST) or a float into radians.
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
        Load a directed graph (DiGraph) from a dictionary with nodes and edges.
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

def main(args=None):
    """
    Main entry point.
    Parse arguments, initialize ROS, create the node, and spin until interrupted.
    """
    rclpy.init(args=args)

    # Parse command-line arguments.
    parser = argparse.ArgumentParser(description='Slave Navigation Simulator Node')
    parser.add_argument('--robot_namespace', type=str, default='robot_simulator', help='Robot namespace')
    parser.add_argument('--initial_node_label', type=str, default='node_1', help='Initial node label where the robot starts')
    parser.add_argument('--initial_orientation', type=str, default='NORTH', help='Initial orientation (NORTH, EAST, SOUTH, WEST)')

    # Ignore unknown ROS arguments
    args, unknown = parser.parse_known_args()

    # Create the node instance with given parameters
    node = SlaveNavigationSimulator(
        robot_namespace=args.robot_namespace,
        initial_node_label=args.initial_node_label,
        initial_orientation_str=args.initial_orientation
    )

    try:
        # Keep the node spinning and responding to callbacks until Ctrl+C or another shutdown signal.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup on exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
