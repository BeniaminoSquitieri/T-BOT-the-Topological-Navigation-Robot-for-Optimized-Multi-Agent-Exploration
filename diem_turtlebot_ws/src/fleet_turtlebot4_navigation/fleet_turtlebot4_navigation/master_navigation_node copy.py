#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library, allows creation and handling of ROS nodes, publishers, subscribers, etc.
from rclpy.node import Node  # Base class for creating a ROS 2 node in Python
from std_msgs.msg import String  # Standard ROS message type for sending/receiving string data
import json  # For encoding and decoding JSON messages
import math  # For mathematical operations and comparisons (e.g., math.isclose)
from .graph_partitioning import load_full_graph, partition_graph  # Custom functions for loading and partitioning the graph
from .path_calculation import calculate_dcpp_route, orientation_rad_to_str  # Custom functions for path calculation and orientation conversion
import os  # For filesystem operations (e.g., checking if a file exists)
import networkx as nx  # For creating and handling complex graphs

class SlaveState:
    """
    Class to manage the state of each slave robot.
    This class is used by the master node to track and manage individual slaves in a fleet.

    Attributes:
        slave_ns (str): The unique namespace or identifier for the slave robot (e.g., "robot_1").
        publisher (rclpy.Publisher): A ROS publisher for sending navigation commands to this slave.
        assigned_waypoints (list): A list of waypoints (graph nodes) assigned to this slave.
        current_waypoint_index (int): Tracks the current waypoint index the slave is navigating to.
        last_seen_time (float): Timestamp (in seconds) of the last communication (e.g., heartbeat) from the slave.
        initial_x (float): Initial X-coordinate of the slave's starting position in the map.
        initial_y (float): Initial Y-coordinate of the slave's starting position in the map.
        initial_orientation (str): Initial orientation of the slave (e.g., "NORTH", "EAST", etc.).
        waiting (bool): Indicates whether the slave is waiting for an available edge to proceed.
        current_node (str): The label of the current graph node (vertex) where the slave is located.
    """
    def __init__(self, slave_ns, publisher):
        """
        Initialize a new instance of the SlaveState class.

        Args:
            slave_ns (str): Namespace or unique identifier for the slave robot.
            publisher (rclpy.Publisher): Publisher to send navigation commands to the slave.
        """
        # Store the slave namespace (identifier)
        self.slave_ns = slave_ns

        # Store the publisher for sending commands to this slave
        self.publisher = publisher

        # Initialize an empty list for the slave's assigned waypoints (navigation route)
        self.assigned_waypoints = []

        # Set the index of the next waypoint to navigate to 0 (start of the route)
        self.current_waypoint_index = 0

        # Initialize the last seen time as 0 (will be updated when the slave communicates)
        self.last_seen_time = 0.0

        # Initialize the slave's starting position (to be set when known)
        self.initial_x = None
        self.initial_y = None

        # Initialize the initial orientation of the slave
        self.initial_orientation = None

        # Set the waiting flag to False (default state; the slave is ready to navigate)
        self.waiting = False

        # Initialize the current node where the slave is located (to be determined later)
        self.current_node = None

class MasterNavigationNode(Node):
    """
    A ROS 2 node that acts as the master in a multi-robot navigation system.
    This node manages a fleet of robots, assigns navigation tasks, and monitors their status.

    Responsibilities:
    - Load a navigation graph and distribute it to slaves.
    - Register slaves and track their states.
    - Assign waypoints to robots by partitioning the graph.
    - Handle navigation feedback from robots.
    - Periodically check and manage the state of slaves, removing inactive ones.

    Attributes:
        graph_path (str): Path to the navigation graph JSON file.
        check_interval (float): Interval (in seconds) to check and update slave states.
        timeout (float): Time (in seconds) after which inactive slaves are considered timed out.
        full_graph (nx.DiGraph): The navigation graph loaded from the JSON file.
        graph_publisher (rclpy.Publisher): Publishes the navigation graph to slaves.
        graph_timer (rclpy.Timer): Periodically publishes the navigation graph.
        slave_registration_subscriber (rclpy.Subscription): Listens for slave registration messages.
        initial_position_subscriber (rclpy.Subscription): Listens for initial positions from slaves.
        navigation_status_subscriber (rclpy.Subscription): Listens for navigation status updates from slaves.
        heartbeat_publisher (rclpy.Publisher): Publishes heartbeat messages to notify slaves the master is active.
        heartbeat_timer (rclpy.Timer): Periodically sends heartbeat messages.
        timer (rclpy.Timer): Periodically checks the state of slaves and manages waiting tasks.
        slaves (dict): A dictionary mapping slave namespaces to their respective SlaveState objects.
        partitioning_done (bool): Flag to indicate whether graph partitioning has been completed.
        occupied_edges (set): A set of tuples representing occupied edges in the navigation graph.
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

        # Load the navigation graph from the specified file
        self.full_graph = load_full_graph(self.graph_path)

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

    def publish_navigation_graph(self):
        """
        Publish the navigation graph to the '/navigation_graph' topic.
        This allows slaves to know the topology of the environment.
        """
        graph_msg = String()  # Create a String message
        # Convert the graph nodes and edges into a JSON-serializable dictionary
        graph_data = {
            'nodes': [
                {
                    'label': node,
                    'x': data['x'],
                    'y': data['y'],
                    'orientation': data.get('orientation', 0.0)
                } for node, data in self.full_graph.nodes(data=True)
            ],
            'edges': [
                {
                    'from': u,
                    'to': v,
                    'weight': data.get('weight', 1.0)
                } for u, v, data in self.full_graph.edges(data=True)
            ]
        }
        # Serialize this dictionary into a JSON string
        graph_msg.data = json.dumps(graph_data)
        # Publish the graph message so that slaves can receive it
        self.graph_publisher.publish(graph_msg)
        # (Logging of the publication is commented out, can be added if needed.)

    def publish_heartbeat(self):
        """
        Periodically publish a heartbeat message to indicate that the master node is active.
        Slaves rely on this heartbeat to detect if the master goes offline.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = "alive"  # Simple message indicating the master is operational
        self.heartbeat_publisher.publish(heartbeat_msg)  # Publish the heartbeat message
        self.get_logger().debug("Published heartbeat.")  # Log the action for debugging

    def slave_registration_callback(self, msg):
        """
        Callback function triggered when a slave sends a registration message.
        This message informs the master about the slave's existence.

        Args:
            msg (String): ROS message containing the slave's namespace.
        """
        slave_ns = msg.data.strip()  # Extract and clean the slave namespace from the message
        current_time = self.get_clock().now().nanoseconds / 1e9  # Get current time in seconds

        if slave_ns not in self.slaves:
            # If this slave is new, register it
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)  # Create a SlaveState instance
            slave_state.last_seen_time = current_time  # Update the last seen time
            self.slaves[slave_ns] = slave_state  # Add the slave to the registry
            self.get_logger().info(f"Registered new slave: {slave_ns}")  # Log the registration
        else:
            # If the slave is already known, update its last seen time
            self.slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"Updated last seen time for slave: {slave_ns}")

    def initial_position_callback(self, msg):
        """
        Callback function triggered when a slave sends its initial position.
        This message contains the slave's coordinates and orientation.

        Args:
            msg (String): ROS message containing the initial position in JSON format.
        """
        try:
            # Write the raw message to a log file for debugging
            with open("/tmp/initial_position_messages.log", "a") as f:
                f.write(f"Received message: {msg.data}\n")
        except Exception as e:
            # Log an error if writing to the file fails
            self.get_logger().error(f"Failed to write received message to log: {e}")

        # Parse the JSON message
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']  # Extract the slave namespace
            initial_x = float(data['x'])  # Extract the X coordinate
            initial_y = float(data['y'])  # Extract the Y coordinate
            orientation = data.get('orientation', 'NORTH')  # Extract orientation (default to 'NORTH')
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            # Log an error if parsing the JSON fails
            self.get_logger().error(f"Invalid initial position message: {e}")
            return

        current_time = self.get_clock().now().nanoseconds / 1e9  # Get the current time in seconds

        if slave_ns in self.slaves:
            # If the slave is already registered, update its state
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time
            slave.initial_x = initial_x
            slave.initial_y = initial_y
            slave.initial_orientation = orientation
            slave.current_node = self.find_node_from_position(initial_x, initial_y)  # Find the corresponding graph node

            if slave.current_node is None:
                # Log an error if no matching node is found
                self.get_logger().error(f"Could not find a node matching the initial position of slave {slave_ns}")
            else:
                # Log the successfully updated position and node
                self.get_logger().info(
                    f"Received initial position from {slave_ns}: "
                    f"({initial_x}, {initial_y}) on node {slave.current_node} with orientation {orientation}"
                )
            # Attempt to partition the graph and assign waypoints
            self.repartition_and_assign_waypoints()
        else:
            # If the slave wasn't known, register it and assign waypoints
            self.get_logger().info(f"Initial position received for unregistered slave {slave_ns}, registering now.")
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            slave_state.initial_x = initial_x
            slave_state.initial_y = initial_y
            slave_state.initial_orientation = orientation
            slave_state.current_node = self.find_node_from_position(initial_x, initial_y)

            if slave_state.current_node is None:
                # Log an error if no matching node is found
                self.get_logger().error(f"Could not find a node matching the initial position of slave {slave_ns}")
            else:
                # Log the successful creation of a publisher for the new slave
                self.get_logger().info(f"Created publisher for slave: {slave_ns} at node {slave_state.current_node}")

            # Add the slave to the registry and partition the graph
            self.slaves[slave_ns] = slave_state
            self.repartition_and_assign_waypoints()

    def find_node_from_position(self, x, y):
        """
        Finds the graph node corresponding to the given coordinates (x, y).
        
        This function assumes that the position matches a node in the graph. It uses exact matching
        with a tolerance to account for small floating-point inaccuracies.

        Args:
            x (float): X-coordinate of the position.
            y (float): Y-coordinate of the position.

        Returns:
            str: The label of the node in the graph that matches the given coordinates.
            None: If no node matches the given coordinates.
        """
        for node, data in self.full_graph.nodes(data=True):
            # Check if the coordinates match within a tolerance using math.isclose
            if math.isclose(data['x'], x, abs_tol=1e-4) and math.isclose(data['y'], y, abs_tol=1e-4):
                return node  # Return the node label if a match is found
        return None  # Return None if no match is found


    def navigation_status_callback(self, msg):
        """
        Processes navigation status updates from slaves.

        This function is triggered whenever a slave sends a navigation status update. It parses the
        message to determine whether the slave successfully reached a waypoint or encountered an error.

        Args:
            msg (String): JSON-formatted string containing the navigation status of a slave.

        Expected JSON Structure:
            {
                "robot_namespace": <str>,  # Namespace of the slave
                "status": <str>,           # Status ("reached" or "error")
                "current_waypoint": <str>, # The waypoint label the slave interacted with
                "time_taken": <float>,     # Time taken to reach the waypoint
                "error_message": <str>     # Optional error message in case of failure
            }
        """
        self.get_logger().info(f"Received navigation status message: {msg.data}")

        # Parse the JSON data from the message
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_waypoint = data['current_waypoint']
            time_taken = data['time_taken']
            error_message = data.get('error_message', '')  # Optional error message
        except (json.JSONDecodeError, KeyError) as e:
            # Log an error if the message cannot be parsed
            self.get_logger().error(f"Invalid navigation status message: {e}")
            return

        self.get_logger().info(f"Decoded navigation status data: {data}")

        # Get the current time
        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns in self.slaves:
            # Retrieve the slave's state if it is known
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time  # Update the last seen time for the slave

            if status == "reached":
                # Handle successful navigation to a waypoint
                from_node = slave.current_node  # Current node the slave is on
                to_node = current_waypoint  # Target waypoint
                edge = (from_node, to_node)  # Edge traversed by the slave

                # Free the edge after the slave successfully traverses it
                if edge in self.occupied_edges:
                    self.occupied_edges.remove(edge)
                    self.get_logger().info(f"Edge {edge} is now free.")
                else:
                    self.get_logger().warn(f"Edge {edge} was not occupied. Check logic.")

                # Update the slave's current node and move to the next waypoint
                slave.current_node = current_waypoint
                slave.current_waypoint_index += 1
                slave.waiting = False  # The slave is no longer waiting

                # Assign the next waypoint to the slave
                self.assign_next_waypoint(slave_ns)

                # Check if there are any waiting slaves and attempt to assign their waypoints
                self.assign_waiting_slaves()

            elif status == "error":
                # Handle navigation errors
                self.get_logger().error(f"Slave {slave_ns} encountered an error: {error_message}")

                # Free the edge the slave was attempting to traverse
                from_node = slave.current_node
                to_node = current_waypoint
                edge = (from_node, to_node)
                if edge in self.occupied_edges:
                    self.occupied_edges.remove(edge)
                    self.get_logger().info(f"Edge {edge} is now free due to error.")

                # Remove the slave from the list of active slaves
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removed slave {slave_ns} due to error.")

                # Repartition the graph and assign waypoints to the remaining slaves
                self.repartition_and_assign_waypoints()
        else:
            # Log a warning if the slave is not recognized
            self.get_logger().warn(f"Received status from unknown slave {slave_ns}.")


    def check_all_waypoints_reached(self):
        """
        Checks if all slaves have completed their assigned waypoints.

        This function iterates through all active slaves and checks if they have reached the last
        waypoint in their assigned list.

        Returns:
            bool: True if all slaves have completed their waypoints, False otherwise.
        """
        for slave in self.slaves.values():
            # If any slave has not reached its last waypoint, return False
            if slave.current_waypoint_index < len(slave.assigned_waypoints):
                return False
        return True  # Return True if all waypoints are completed


    def repartition_and_assign_waypoints(self):
        """
        Partitions the graph and assigns waypoints to active slaves.

        This function divides the full navigation graph into subgraphs for each active slave. It calculates
        routes for each subgraph and assigns them to the respective slaves.
        """
        num_slaves = len(self.slaves)
        if num_slaves == 0:
            self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
            self.partitioning_done = False
            return

        # Collect the initial positions of all active slaves
        start_positions = []
        for slave in self.slaves.values():
            if slave.initial_x is not None and slave.initial_y is not None:
                start_positions.append({'x': slave.initial_x, 'y': slave.initial_y})
            else:
                self.get_logger().warn(f"Slave {slave.slave_ns} lacks valid initial position.")

        # Ensure all slaves have valid initial positions
        if len(start_positions) != num_slaves:
            self.get_logger().error("Not all slaves have valid initial positions.")
            return

        # Partition the graph into balanced subgraphs based on the number of slaves
        try:
            subgraphs = partition_graph(self.full_graph, num_slaves, start_positions=start_positions)
            self.get_logger().info(f"Partitioned the graph into {len(subgraphs)} subgraphs.")
        except ValueError as e:
            self.get_logger().error(f"Failed to partition graph: {e}")
            return

        # Sort the slave namespaces to ensure consistent assignment
        slaves_sorted = sorted(self.slaves.keys())

        # Ensure the number of subgraphs matches the number of slaves
        if len(subgraphs) != len(slaves_sorted):
            self.get_logger().error("Number of subgraphs does not match number of active slaves.")
            return

        # Assign each subgraph to a slave
        for idx, slave_ns in enumerate(slaves_sorted):
            subgraph = subgraphs[idx]

            # Extract waypoints from the subgraph
            waypoints = self.extract_waypoints(subgraph)

            # Calculate an Eulerian path through the waypoints
            dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.get_logger())

            # Log the calculated route for debugging
            self.get_logger().info(f"DCPP Route for {slave_ns}:")
            for wp in dcpp_route:
                self.get_logger().info(f"  {wp}")

            # Assign the calculated route to the slave
            slave = self.slaves[slave_ns]
            slave.assigned_waypoints = dcpp_route
            self.assign_next_waypoint(slave_ns)

        # Mark partitioning as done
        self.partitioning_done = True

    def assign_next_waypoint(self, slave_ns):
        """
        Assign the next waypoint from the slave's route to the slave for navigation.

        This function checks if the edge (path between two nodes) required for the next waypoint is free. 
        If it is free, the waypoint is sent to the slave. If not, the slave is marked as waiting.

        Args:
            slave_ns (str): The namespace of the slave robot.
        """
        # Retrieve the SlaveState object for the given slave
        slave = self.slaves[slave_ns]
        
        # Check if the slave has any waypoints assigned
        if len(slave.assigned_waypoints) == 0:
            self.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
            return

        # Get the next waypoint to assign
        # Use modulo operation to cycle through waypoints if needed (e.g., loop routes)
        waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
        from_node = slave.current_node  # Current position of the slave
        to_node = waypoint['label']  # Target node for the waypoint
        edge = (from_node, to_node)  # Define the edge between the nodes

        # Check if the edge is already occupied
        if edge in self.occupied_edges:
            # If occupied, the slave must wait for the edge to become free
            self.get_logger().warn(f"Edge {edge} is already occupied. Slave {slave_ns} must wait.")
            slave.waiting = True
            return
        else:
            # If the edge is free, occupy it for this slave
            self.occupied_edges.add(edge)
            self.get_logger().info(f"Assigned edge {edge} to {slave_ns}.")

            # Create the waypoint message
            waypoint_msg = {
                'label': waypoint['label'],  # Target node label
                'x': waypoint['x'],          # X-coordinate of the target node
                'y': waypoint['y'],          # Y-coordinate of the target node
                'orientation': orientation_rad_to_str(waypoint['orientation'])  # Orientation as a string
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)

            # Send the navigation command to the slave via its publisher
            slave.publisher.publish(msg)


    def assign_waiting_slaves(self):
        """
        Attempt to reassign waypoints to slaves that were previously marked as waiting.

        This function checks if the edge required by a waiting slave is now free. 
        If free, the waypoint is assigned, and the slave is removed from the waiting state.
        """
        # Process slaves in a sorted order to ensure consistency
        for slave_ns in sorted(self.slaves.keys()):
            slave = self.slaves[slave_ns]
            
            # Skip slaves that are not waiting
            if not slave.waiting:
                continue

            # Check if the slave has waypoints assigned
            if len(slave.assigned_waypoints) == 0:
                self.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                continue

            # Get the next waypoint for the slave
            waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
            from_node = slave.current_node
            to_node = waypoint['label']
            edge = (from_node, to_node)

            # Check if the edge is now free
            if edge not in self.occupied_edges:
                # Occupy the edge and send the waypoint to the slave
                self.occupied_edges.add(edge)
                slave.waiting = False
                self.get_logger().info(f"Assigned edge {edge} to {slave_ns} (previously waiting).")

                # Create the waypoint message
                waypoint_msg = {
                    'label': waypoint['label'],
                    'x': waypoint['x'],
                    'y': waypoint['y'],
                    'orientation': orientation_rad_to_str(waypoint['orientation'])
                }
                msg = String()
                msg.data = json.dumps(waypoint_msg)

                # Publish the navigation command to the slave
                slave.publisher.publish(msg)
            else:
                # If the edge is still occupied, the slave remains waiting
                self.get_logger().warn(f"Edge {edge} is still occupied. Slave {slave_ns} remains waiting.")


    def print_subgraphs(self, subgraphs):
        """
        Log details of each subgraph created during graph partitioning.

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
        Extract waypoints (nodes) from a subgraph.

        This function converts nodes in the subgraph into a list of dictionaries, where each dictionary
        contains the position and orientation of a node.

        Args:
            subgraph (nx.Graph): A subgraph containing nodes and edges.

        Returns:
            list: A list of dictionaries representing the waypoints.
        """
        waypoints = []
        
        # Iterate over each node in the subgraph
        for node, data in subgraph.nodes(data=True):
            waypoint = {
                'label': node,          # Node label (identifier)
                'x': data['x'],         # X-coordinate of the node
                'y': data['y'],         # Y-coordinate of the node
                'orientation': data.get('orientation', 0.0)  # Orientation of the node
            }
            waypoints.append(waypoint)  # Add the waypoint to the list
        
        return waypoints

    def timer_callback(self):
        """
        Periodic callback function triggered at intervals defined by `check_interval`.

        This method performs two primary tasks:
        1. Checks if any slaves have timed out due to inactivity (exceeding the timeout period).
        2. Attempts to reassign waypoints to slaves that are waiting for free edges.

        Purpose:
        - Ensures that inactive slaves are removed, and their resources (edges) are freed.
        - Improves efficiency by attempting to resolve waiting states for active slaves.

        This function is periodically called by a ROS timer.
        """
        # Log debug information for the timer execution
        self.get_logger().debug("Timer callback triggered.")
        
        # Check for and remove inactive slaves (exceeding timeout period)
        self.check_slaves_timeout()
        
        # Attempt to assign waypoints to slaves that are waiting for an edge to become free
        self.assign_waiting_slaves()


    def check_slaves_timeout(self):
        """
        Identify and remove slaves that have exceeded the communication timeout.

        For each slave:
        - Check if the last communication time (`last_seen_time`) exceeds the allowed `timeout`.
        - If so, free any occupied edge and remove the slave from the active list.

        After removing timed-out slaves:
        - Repartition the navigation graph to adjust for the updated number of slaves.
        - Reassign waypoints to the remaining active slaves.

        This ensures that resources (edges) are freed and the system remains responsive.
        """
        # Get the current time in seconds
        current_time = self.get_clock().now().nanoseconds / 1e9
        slaves_to_remove = []  # List to track slaves that need to be removed

        # Iterate through all active slaves to identify those that have timed out
        for slave_ns, slave in self.slaves.items():
            if current_time - slave.last_seen_time > self.timeout:
                # If the time since the last communication exceeds the timeout, mark the slave for removal
                self.get_logger().warn(f"Slave {slave_ns} has timed out. Removing from active slaves.")
                slaves_to_remove.append(slave_ns)

        # Remove all identified timed-out slaves
        for slave_ns in slaves_to_remove:
            if slave_ns in self.slaves:
                # Retrieve the slave object
                s = self.slaves[slave_ns]
                
                # Free any edge that the slave was occupying
                if s.current_waypoint_index < len(s.assigned_waypoints):
                    # Get the edge the slave was traveling on
                    waypoint = s.assigned_waypoints[s.current_waypoint_index % len(s.assigned_waypoints)]
                    from_node = s.current_node
                    to_node = waypoint['label']
                    edge = (from_node, to_node)
                    
                    # Remove the edge from the occupied edges set
                    if edge in self.occupied_edges:
                        self.occupied_edges.remove(edge)
                        self.get_logger().info(f"Edge {edge} is now free due to slave timeout.")

                # Remove the slave from the active slaves dictionary
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removed slave {slave_ns} due to timeout.")

        # If any slaves were removed, repartition the graph and reassign waypoints
        if slaves_to_remove:
            # Log the need for repartitioning
            self.get_logger().info("Repartitioning and reassigning waypoints after slave removal.")
            self.repartition_and_assign_waypoints()
            
def load_full_graph_from_data(graph_data):
    """
    Load a directed graph from a dictionary containing nodes and edges.

    This function is used to convert a JSON-like dictionary structure into a
    NetworkX directed graph (DiGraph) representation. The graph contains nodes
    with attributes (like position and orientation) and edges with weights.

    Args:
        graph_data (dict): A dictionary with two main keys:
            - 'nodes': A list of dictionaries, each containing:
                * 'label': A unique identifier for the node.
                * 'x': The x-coordinate of the node.
                * 'y': The y-coordinate of the node.
                * 'orientation' (optional): Orientation of the node (default is 0.0).
            - 'edges': A list of dictionaries, each containing:
                * 'from': The source node's label.
                * 'to': The target node's label.
                * 'weight' (optional): Weight of the edge (default is 1.0).

    Returns:
        nx.DiGraph: A NetworkX directed graph populated with the nodes and edges
                    described in the input dictionary.
    """
    # Initialize an empty directed graph
    G = nx.DiGraph()

    # Add nodes to the graph with their attributes
    for node in graph_data['nodes']:
        label = node['label']  # Unique identifier for the node
        x = node['x']  # x-coordinate of the node
        y = node['y']  # y-coordinate of the node
        orientation = node.get('orientation', 0.0)  # Orientation (default: 0.0 radians)
        
        # Add the node to the graph with its attributes
        G.add_node(label, x=x, y=y, orientation=orientation)

    # Add edges to the graph with their weights
    for edge in graph_data['edges']:
        u = edge['from']  # Source node label
        v = edge['to']    # Target node label
        weight = edge.get('weight', 1.0)  # Weight of the edge (default: 1.0)
        
        # Add the edge to the graph
        G.add_edge(u, v, weight=weight)

    # Return the constructed directed graph
    return G

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
