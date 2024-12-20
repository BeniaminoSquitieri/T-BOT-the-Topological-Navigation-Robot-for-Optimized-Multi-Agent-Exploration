#!/usr/bin/env python3

# ROS 2 and basic Python imports
import rclpy  # Python library to interface with ROS 2
from rclpy.node import Node  # Base class for creating ROS 2 nodes in Python
from std_msgs.msg import String  # Standard ROS message type for transmitting string data
import json  # For encoding and decoding JSON data
import time  # For timestamps, measuring time intervals, etc.
import argparse  # For parsing command-line arguments
import math  # For mathematical operations (sin, cos, pi, etc.)
import networkx as nx  # Library for creating and handling complex graphs
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult
# TurtleBot4Navigator is a custom class providing methods to command a TurtleBot4 robot and check navigation results.

# Importing custom functions for loading and partitioning the graph and for path calculations
from .graph_partitioning import  partition_graph
from .master.path_calculation import calculate_dcpp_route

def load_full_graph_from_data(graph_data):
    """
    Load a NetworkX graph from a dictionary containing nodes and edges.

    This function creates a directed graph (`nx.DiGraph`) using the `networkx` library.
    Nodes and edges are added to the graph using the information provided in the `graph_data` dictionary.

    Args:
        graph_data (dict): A dictionary with two main keys:
            - 'nodes': A list of dictionaries, where each dictionary contains:
                - 'label' (str): The identifier of the node.
                - 'x' (float): The X-coordinate of the node.
                - 'y' (float): The Y-coordinate of the node.
                - 'orientation' (optional, float): The orientation of the node in radians.
            - 'edges': A list of dictionaries, where each dictionary contains:
                - 'from' (str): The label of the starting node.
                - 'to' (str): The label of the ending node.
                - 'weight' (optional, float): The weight of the edge (default is 1.0).

    Returns:
        nx.DiGraph: The directed graph constructed from the provided data.
    """
    # Initialize a directed graph
    G = nx.DiGraph()

    # Iterate over all nodes in the input data and add them to the graph
    for node in graph_data['nodes']:
        # Extract node properties
        label = node['label']  # Node identifier
        x = node['x']          # X-coordinate
        y = node['y']          # Y-coordinate
        orientation = node.get('orientation', 0.0)  # Orientation (default: 0 radians)

        # Add the node to the graph with its properties
        G.add_node(label, x=x, y=y, orientation=orientation)

    # Iterate over all edges in the input data and add them to the graph
    for edge in graph_data['edges']:
        # Extract edge properties
        u = edge['from']       # Starting node label
        v = edge['to']         # Ending node label
        weight = edge.get('weight', 1.0)  # Edge weight (default: 1.0)

        # Add the edge to the graph with its weight
        G.add_edge(u, v, weight=weight)

    # Return the constructed graph
    return G


class SlaveState:
    """
    Class for managing the state of each slave robot.

    This class stores the state and properties of a slave robot, which is used for tracking
    the robot's position, assigned tasks, and communication status. It also includes 
    functionality to interact with the robot via ROS publishers.

    Attributes:
        slave_ns (str): The namespace or identifier for the slave (e.g., "robot_1").
        assigned_waypoints (list): A list of waypoints assigned to this slave.
        current_waypoint_index (int): The index of the next waypoint to be reached.
        last_seen_time (float): The timestamp of the last heartbeat received from this slave.
        initial_x (float): The X-coordinate of the slave's initial position.
        initial_y (float): The Y-coordinate of the slave's initial position.
        initial_orientation (float): The orientation of the slave in radians.
        publisher: A ROS publisher for sending messages to this slave.
        waiting (bool): A flag indicating whether the slave is waiting for a new waypoint.
    """
    def __init__(self, slave_ns):
        """
        Initialize the state of the slave robot.

        Args:
            slave_ns (str): The namespace or unique identifier for the slave.
        """
        self.slave_ns = slave_ns                   # Unique identifier or namespace for the slave
        self.assigned_waypoints = []               # Initialize the list of waypoints
        self.current_waypoint_index = 0            # Set the index of the next waypoint to 0
        self.last_seen_time = 0.0                  # Initialize the last seen time to 0
        self.initial_x = None                      # Placeholder for the initial X-coordinate
        self.initial_y = None                      # Placeholder for the initial Y-coordinate
        self.initial_orientation = None            # Placeholder for the initial orientation in radians
        self.publisher = None                      # Placeholder for the ROS publisher
        self.waiting = False                       # Set the waiting flag to False


class SlaveNavigationNode(Node):
    """
    ROS 2 node representing a slave robot.

    This node:
    - Waits for the navigation graph from the master to determine its initial position.
    - Registers itself to the master so the master knows it's available.
    - Receives waypoints from the master and executes them, publishing status updates.
    - Monitors the master's presence via heartbeat messages.
    - If the master goes offline, this node can participate in a re-election process to become the new master.

    Attributes:
        robot_namespace (str): Unique namespace identifying this slave (e.g., "robot_1").
        initial_node_label (str): The label of the initial map node where this robot is located.
        initial_orientation_str (str): Initial orientation of the robot as a string ("NORTH", "EAST", etc.).
        initial_orientation (float): Initial orientation of the robot in radians.
        slave_registration_publisher: Publishes registration messages so the master knows this slave exists.
        initial_position_publisher: Publishes this slave's initial position to the master once known.
        navigation_commands_subscriber: Subscribes to commands from the master to move to certain waypoints.
        status_publisher: Publishes navigation status updates to the master.
        navigation_status_subscriber: Subscribes to navigation status messages (from itself or master) if needed.
        heartbeat_publisher: Publishes heartbeat messages to indicate this slave is active.
        master_heartbeat_subscriber: Subscribes to master's heartbeat to monitor master's availability.
        slave_heartbeat_subscriber: Subscribes to other slaves' heartbeats (relevant if this becomes master).
        graph_subscriber: Subscribes to the navigation graph from the master to determine initial position.
        registration_timer: Timer to periodically re-send registration messages.
        heartbeat_timer: Timer to periodically send heartbeat messages.
        slave_check_timer: Timer to periodically check if other slaves are alive (when acting as master).
        navigator (TurtleBot4Navigator): A helper object to navigate the robot to given coordinates.
        master_alive (bool): Whether the master is currently considered alive.
        last_master_heartbeat (float): Timestamp of the last master heartbeat received.
        heartbeat_timeout (float): Time in seconds to wait before considering the master lost.
        active_slaves (dict): A dictionary of active slaves, used if this node becomes master.
        navigation_graph (nx.DiGraph): The received navigation graph.
        slave_command_publishers (dict): Publishers for sending commands to other slaves if this node is master.
        slave_initial_positions (dict): Stores initial positions of other slaves if this node becomes master.
        is_master (bool): Flag indicating if this node has become the master.
        master_graph_partitioned (bool): Whether the graph has been partitioned already (if master).
        assigned_waypoints (list): Waypoints assigned to this node if it becomes the master.
        current_waypoint_index (int): Current index in the assigned waypoints if this node is master.
        is_navigating (bool): Whether the robot is currently navigating to a waypoint.
        current_goal_label (str): Label of the current goal the robot is navigating to.
        current_goal_start_time (float): Start time of the current navigation task.
        navigation_check_timer: Timer to regularly check navigation status.
        initial_position_published (bool): Whether the initial position has been published to the master.
    """

    def __init__(self, robot_namespace, initial_node_label, initial_orientation_str):
        """
        Initialize the SlaveNavigationNode.

        Args:
            robot_namespace (str): Unique namespace identifying this slave (e.g., "robot_1").
            initial_node_label (str): Label of the initial node on the map where the robot starts.
            initial_orientation_str (str): Initial orientation of the robot as a string (e.g., "NORTH").
        """
        # Store the input parameters
        self.robot_namespace = robot_namespace
        self.initial_node_label = initial_node_label
        self.initial_orientation_str = initial_orientation_str

        # Convert the orientation from a string to radians
        self.initial_orientation = self.orientation_conversion(initial_orientation_str)

        # Initialize the ROS node with a specific name
        super().__init__('slave_navigation_node')

        # Create a publisher to register the slave with the master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)

        # Create a publisher to send the slave's initial position to the master
        self.initial_position_publisher = self.create_publisher(String, '/slave_initial_positions', 10)

        # Subscriber for receiving navigation commands from the master
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.navigation_commands_callback, 10
        )

        # Publisher to send navigation status updates to the master
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Subscriber for receiving navigation status messages (self or other slaves)
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )

        # Publisher for sending heartbeat messages to indicate the slave is alive
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)

        # Subscriber for monitoring heartbeat messages from the master
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 10
        )

        # Subscriber for monitoring heartbeat messages from other slaves
        self.slave_heartbeat_subscriber = self.create_subscription(
            String, '/slave_heartbeat', self.slave_heartbeat_callback, 10
        )

        # Subscriber for receiving the navigation graph from the master
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.navigation_graph_callback, 10
        )

        # Timer to periodically publish registration messages
        self.registration_timer = self.create_timer(1.0, self.publish_registration)

        # Timer to periodically publish heartbeat messages
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Timer to periodically check the status of other slaves
        self.slave_check_timer = self.create_timer(2.0, self.check_slave_alive)

        # Initialize the navigator for handling navigation tasks
        self.navigator = TurtleBot4Navigator()

        # Variables for monitoring the master
        self.master_alive = False  # Flag to track if the master is alive
        self.last_master_heartbeat = time.time()  # Timestamp of the last received heartbeat
        self.heartbeat_timeout = 150.0  # Timeout in seconds before considering the master lost

        # Dictionary for tracking other slaves if this node becomes master
        self.active_slaves = {}

        # Variable to store the navigation graph once received
        self.navigation_graph = None

        # Publishers for sending commands to other slaves if this node becomes master
        self.slave_command_publishers = {}
        self.slave_initial_positions = {}

        # Flags and variables related to the master's role
        self.is_master = False  # Flag to indicate if this node is acting as the master
        self.master_graph_partitioned = False  # Whether the graph has been partitioned as the master

        # Variables for managing waypoints if this node is master
        self.assigned_waypoints = []
        self.current_waypoint_index = 0

        # Flags for navigation state
        self.is_navigating = False  # Whether the robot is currently navigating
        self.current_goal_label = None  # Label of the current navigation goal
        self.current_goal_start_time = None  # Start time of the current navigation task

        # Timer to periodically check the navigation status
        self.navigation_check_timer = self.create_timer(0.5, self.check_navigation_status)

        # Flag to indicate if the initial position has been published
        self.initial_position_published = False

        # Log the initialization of the slave node
        self.get_logger().info(
            f"[{self.robot_namespace}] Slave node initialized with initial node label '{self.initial_node_label}' "
            f"and orientation '{self.initial_orientation_str}' ({self.initial_orientation} radians)."
        )


    def publish_registration(self):
        """
        Periodically publish a registration message so the master knows this slave is active.
        """
        # Create a ROS String message with the robot's namespace as the content
        msg = String()
        msg.data = self.robot_namespace

        # Publish the message to the /slave_registration topic
        self.slave_registration_publisher.publish(msg)

        # Log the publishing event for debugging purposes
        self.get_logger().debug(f"[{self.robot_namespace}] Published registration.")


    def publish_initial_position(self):
        """
        Once the coordinates of the initial node are known, publish them to the master.
        This lets the master understand where this slave is located in the graph.
        """
        # Check if the coordinates (x, y) are set before proceeding
        if self.initial_x is None or self.initial_y is None:
            # Log an error message if the coordinates are not available
            self.get_logger().error("Cannot publish initial position: coordinates not set.")
            return

        # Create a dictionary representing the initial position
        initial_position = {
            'robot_namespace': self.robot_namespace,  # Unique identifier for the robot
            'x': self.initial_x,                      # X-coordinate of the initial position
            'y': self.initial_y,                      # Y-coordinate of the initial position
            'orientation': self.initial_orientation_str  # Orientation as a string (e.g., "NORTH")
        }

        # Convert the dictionary to a JSON-formatted string
        msg = String()
        msg.data = json.dumps(initial_position)

        # Publish the initial position message to the /slave_initial_positions topic
        self.initial_position_publisher.publish(msg)

        # Log the successful publishing of the initial position for debugging
        self.get_logger().info(f"[{self.robot_namespace}] Published initial position: {initial_position}")

        # Mark that the initial position has been published to prevent re-publishing
        self.initial_position_published = True


    def publish_heartbeat(self):
        """
        Periodically publish a heartbeat message to signal that the slave is alive.
        The master relies on these heartbeats to detect if slaves go offline.
        """
        # Create a ROS String message with the robot's namespace as the content
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace

        # Publish the heartbeat message to the /slave_heartbeat topic
        self.heartbeat_publisher.publish(heartbeat_msg)

        # Log the publishing event for debugging purposes
        self.get_logger().debug(f"[{self.robot_namespace}] Published heartbeat.")


    def master_heartbeat_callback(self, msg):
        """
        When a heartbeat is received from the master, update the timestamp.
        This prevents considering the master as lost.
        """
        # Update the master_alive flag to indicate the master is active
        self.master_alive = True

        # Record the current time as the last received heartbeat time
        self.last_master_heartbeat = time.time()

        # Log the reception of the master's heartbeat for debugging
        self.get_logger().debug(f"[{self.robot_namespace}] Received master heartbeat.")


    def slave_heartbeat_callback(self, msg):
        """
        When a heartbeat from another slave is received, record the time.
        If this node becomes master later, it will use this info to track them.
        """
        # Extract the namespace of the sending slave from the message
        slave_ns = msg.data.strip()

        # Get the current time
        current_time = time.time()

        # Ignore heartbeats from itself
        if slave_ns != self.robot_namespace:
            # If the slave is new, add it to the active_slaves dictionary
            if slave_ns not in self.active_slaves:
                # Create a new SlaveState object for the newly detected slave
                self.active_slaves[slave_ns] = SlaveState(slave_ns)

                # Log the detection of a new slave
                self.get_logger().info(f"[{self.robot_namespace}] Detected new slave: {slave_ns}")

            # Update the last seen time for the slave
            self.active_slaves[slave_ns].last_seen_time = current_time

            # Log the reception of the slave's heartbeat for debugging
            self.get_logger().debug(f"[{self.robot_namespace}] Received heartbeat from slave {slave_ns}.")


    def check_master_alive(self):
        """
        Check if the master is still alive.
        If no heartbeat is received within the timeout, start a master election.
        """
        # Get the current time
        current_time = time.time()

        # If a heartbeat was received recently, reset the master_alive flag to check for the next cycle
        if self.master_alive:
            self.master_alive = False  # Reset flag for the next heartbeat
        else:
            # If the time since the last heartbeat exceeds the timeout, consider the master lost
            if current_time - self.last_master_heartbeat > self.heartbeat_timeout:
                # Log a warning about the master's loss and start the election process
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost. Initiating master election.")
                self.elect_new_master()  # Call the election process


    def check_slave_alive(self):
        """
        Periodically check if other slaves are still alive by comparing the current time with their last heartbeat.
        Remove slaves considered lost.
        """
        # Get the current time
        current_time = time.time()

        # Iterate through all active slaves' namespaces
        for slave_ns in list(self.active_slaves.keys()):
            # If the time since the last seen heartbeat exceeds the timeout, consider the slave lost
            if current_time - self.active_slaves[slave_ns].last_seen_time > self.heartbeat_timeout:
                # Log a warning about the lost slave
                self.get_logger().warn(f"[{self.robot_namespace}] Slave {slave_ns} heartbeat lost. Removing from active slaves.")

                # Remove the slave from the active list
                del self.active_slaves[slave_ns]


    def elect_new_master(self):
        """
        If the master is lost, elect a new master from the set of slaves and this node.
        The slave with the lexicographically smallest namespace becomes the new master.
        """
        # Combine the current node's namespace with the namespaces of all active slaves
        candidates = list(self.active_slaves.keys()) + [self.robot_namespace]

        # If no candidates are available, log an error and terminate the election process
        if not candidates:
            self.get_logger().error(f"[{self.robot_namespace}] No candidates available for master election.")
            return

        # Sort the candidates lexicographically (alphabetical order for strings)
        candidates_sorted = sorted(candidates)

        # The first candidate in the sorted list is the new master
        new_master = candidates_sorted[0]

        # If this node is the new master, take over as master
        if new_master == self.robot_namespace:
            self.get_logger().info(f"[{self.robot_namespace}] Elected as the new master.")
            self.become_master()  # Transition to master role
        else:
            # Otherwise, log the new master's identity
            self.get_logger().info(f"[{self.robot_namespace}] New master is {new_master}.")

    def become_master(self):
        """
        Transition this node into the role of the master.

        As master, the node will:
        - Publish the navigation graph to inform all slaves about the environment.
        - Partition the navigation graph to assign specific areas to each slave.
        - Assign waypoints to itself and the slaves for coverage.
        - Monitor the status of the slaves and manage their tasks.
        """
        # Set the master flag to True
        self.is_master = True

        # Log the transition to master role
        self.get_logger().info(f"[{self.robot_namespace}] Now acting as the master.")

        # Display available information about the graph and active slaves (for debugging)
        self.available_informations()

        # Check if the navigation graph is available
        if self.navigation_graph is not None:
            # If the graph is available, publish it to all slaves
            self.publish_navigation_graph()

            # Partition the graph and assign tasks (waypoints) to slaves and itself
            self.partition_and_assign_waypoints()
        else:
            # If the graph is not available, log an error and abort the transition
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot become master.")


    def available_informations(self):
        """
        Print out the current state of the navigation graph and list of active slaves.

        This function is primarily used for debugging when this node becomes the master,
        allowing the user to verify the environment and slave information.

        It logs:
        - A detailed description of the graph, including nodes and edges.
        - A list of all active slaves.
        """
        # Log the start of information printing
        self.get_logger().info("----- Available Information -----")

        # Log details of the navigation graph
        self.get_logger().info("Graph description:")
        if self.navigation_graph is not None:
            # Iterate through all nodes in the graph and log their details
            for node, data in self.navigation_graph.nodes(data=True):
                self.get_logger().info(
                    f"  Node {node}: Position=({data['x']}, {data['y']}), Orientation={data.get('orientation', 0.0)} radians"
                )
            # Iterate through all edges in the graph and log their details
            for u, v, data in self.navigation_graph.edges(data=True):
                self.get_logger().info(f"  Edge from {u} to {v}, Weight: {data.get('weight', 1.0)}")
        else:
            # If no graph is available, log this information
            self.get_logger().info("No navigation graph available.")

        # Log the list of active slaves
        self.get_logger().info("Available slaves:")
        for slave_ns in self.active_slaves.keys():
            self.get_logger().info(f"  - {slave_ns}")

        # Log the end of information printing
        self.get_logger().info("----- End of Available Information -----")


    def publish_navigation_graph(self):
        """
        Publish the navigation graph to inform all slaves about the environment.

        The graph contains:
        - Nodes: Representing points of interest in the environment (with position and orientation).
        - Edges: Representing connections between nodes with associated weights.

        This information is sent as a serialized JSON message.
        """
        # Prepare a dictionary to store the graph data
        graph_data = {
            'nodes': [
                {
                    'label': node,
                    'x': data['x'], 
                    'y': data['y'], 
                    'orientation': data.get('orientation', 0.0)
                }
                for node, data in self.navigation_graph.nodes(data=True)
            ],
            'edges': [
                {
                    'from': u, 
                    'to': v, 
                    'weight': data.get('weight', 1.0)
                }
                for u, v, data in self.navigation_graph.edges(data=True)
            ]
        }

        # Serialize the graph data into a JSON string
        graph_msg = String()
        graph_msg.data = json.dumps(graph_data)

        # Publish the serialized graph message to inform all slaves
        self.status_publisher.publish(graph_msg)

        # Log that the graph has been published
        self.get_logger().info("Published navigation graph as master.")


    def navigation_graph_callback(self, msg):
        """
        Callback triggered when the navigation graph is received.

        This graph defines the environment, including nodes and edges representing the map.
        Once the graph is received:
        - The robot's initial position is determined based on its starting node label.
        - If the initial position hasn't been published to the master, publish it.
        - If this node is the master and hasn't partitioned the graph yet, do so.

        Args:
            msg (String): A JSON string containing the navigation graph.
        """
        try:
            # Parse the JSON data into a Python dictionary
            graph_data = json.loads(msg.data)

            # Convert the dictionary into a NetworkX graph
            self.navigation_graph = load_full_graph_from_data(graph_data)

            # Find the coordinates of the initial node using its label
            if self.initial_node_label in self.navigation_graph.nodes:
                node_data = self.navigation_graph.nodes[self.initial_node_label]
                self.initial_x = node_data['x']
                self.initial_y = node_data['y']

                # Publish the initial position to the master if it hasn't been published yet
                if not self.initial_position_published:
                    self.publish_initial_position()

            # If this node is the master and hasn't partitioned the graph yet, do so
            if self.is_master and not self.master_graph_partitioned:
                self.partition_and_assign_waypoints()

        except json.JSONDecodeError as e:
            # Log an error if the graph data cannot be parsed
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation graph: {e}")


    def navigation_commands_callback(self, msg):
        """
        Callback triggered when a navigation command (waypoint) is received from the master.

        This command instructs the robot to navigate to a specific waypoint. The message is:
        - Decoded from JSON format.
        - Validated and processed (orientation converted to radians if needed).
        - Passed to the navigation execution function.

        Args:
            msg (String): A JSON string containing the navigation command (waypoint data).
        """
        try:
            # Parse the JSON data into a Python dictionary
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            # Log an error if the command data cannot be parsed
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation command: {e}")
            return

        # Convert orientation to radians if it's given as a string
        if isinstance(waypoint_data.get('orientation'), str):
            waypoint_data['orientation'] = self.orientation_conversion(waypoint_data['orientation'])

        # Log the received waypoint data
        self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")

        # Execute the navigation to the waypoint
        self.execute_navigation(waypoint_data)


    def execute_navigation(self, waypoint):
        """
        Navigate to a specified waypoint using the TurtleBot4Navigator.

        The function:
        - Starts navigation to the specified (x, y) coordinates with the given orientation.
        - Monitors the navigation process and waits for its completion.
        - Publishes a status message (success or failure) to the master after navigation.

        Args:
            waypoint (dict): A dictionary containing the waypoint data:
                - 'label': Label of the waypoint.
                - 'x': X-coordinate of the waypoint.
                - 'y': Y-coordinate of the waypoint.
                - 'orientation': Orientation at the waypoint (in radians).
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']
        orientation_rad = waypoint['orientation']

        # Log the navigation task
        self.get_logger().info(f"[{self.robot_namespace}] Navigating to {label} at ({x}, {y}) with orientation {orientation_rad} radians.")

        # Create a goal pose for the navigation
        goal_pose = self.navigator.getPoseStamped([x, y], orientation_rad)
        start_time = time.time()

        try:
            # Check if the navigation action server is available
            if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                error_message = f"Action server not available for {label}. Skipping this waypoint."
                self.get_logger().error(error_message)

                # Publish an error status
                self.publish_status("error", error_message, 0.0, label)

                # If this node is the master, skip this waypoint and move to the next one
                if self.is_master:
                    self.get_logger().warn(f"[{self.robot_namespace}] Skipping waypoint {label} and moving to next.")
                    self.current_waypoint_index += 1
                    self.assign_next_waypoint_as_master()

                return

            # Start the navigation task
            self.navigator.startToPose(goal_pose)
        except Exception as e:
            # Handle any exception that occurs while starting the navigation
            error_message = f"Exception occurred while sending goal to {label}: {e}"
            self.get_logger().error(error_message)

            # Publish an error status
            self.publish_status("error", error_message, 0.0, label)
            return

        # Wait for the navigation task to complete
        while not self.navigator.isTaskComplete():
            # Allow ROS to process other callbacks while waiting
            rclpy.spin_once(self, timeout_sec=0.1)

        # Calculate the time taken for navigation
        time_taken = time.time() - start_time

        # Get the result of the navigation task
        nav_result = self.navigator.getResult()

        # Handle the result of the navigation task
        if nav_result == TaskResult.SUCCEEDED:
            # Log success and publish a success status
            self.get_logger().info(f"[{self.robot_namespace}] Reached {label} in {time_taken:.2f} seconds.")
            self.publish_status("reached", "", time_taken, label)

            # Increment the waypoint index and assign the next waypoint if this node is the master
            self.current_waypoint_index += 1
            if self.is_master:
                self.assign_next_waypoint_as_master()
        else:
            # Log failure and publish an error status
            error_message = f"Navigation to {label} failed with result code {nav_result}."
            self.get_logger().error(error_message)
            self.publish_status("error", error_message, time_taken, label)

    def check_navigation_status(self):
        """
        Placeholder for additional navigation status checks.

        This function is designed to be expanded in the future if more detailed
        navigation monitoring is required. Currently, it does nothing.
        """
        pass

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        """
        Publish the navigation status to the master to inform it about the result of the last waypoint.

        This function constructs a status message with relevant details and sends it to the master.

        Args:
            status (str): The status of navigation ("reached", "error", etc.).
            error_message (str): Any error message associated with the navigation, if applicable.
            time_taken (float): Time taken to complete the waypoint navigation.
            current_waypoint (str): The label of the current waypoint.
        """
        # Create a dictionary with the status details
        status_data = {
            'robot_namespace': self.robot_namespace,
            'status': status,
            'error_message': error_message,
            'time_taken': time_taken,
            'current_waypoint': current_waypoint
        }
        # Convert the status dictionary to a JSON string
        msg = String()
        msg.data = json.dumps(status_data)

        # Publish the message
        self.status_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Published status: {status_data}")

    def navigation_status_callback(self, msg):
        """
        Callback to handle navigation status messages received from slaves.

        This function processes the navigation status of other slaves (or itself if it is the master).
        Based on the status, it may assign new waypoints or handle errors.

        Args:
            msg (String): A JSON string containing navigation status information.
        """
        try:
            # Parse the JSON data into a dictionary
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_waypoint = data['current_waypoint']
            time_taken = data['time_taken']
            error_message = data.get('error_message', '')
        except (json.JSONDecodeError, KeyError) as e:
            # Log an error if the message format is invalid
            self.get_logger().error(f"[{self.robot_namespace}] Invalid navigation status message: {e}")
            return

        # Update the current timestamp
        current_time = time.time()

        if self.is_master:
            # If this node is the master, handle states of slaves
            if slave_ns in self.active_slaves:
                # Get the slave's state if it is an active slave
                slave = self.active_slaves[slave_ns]
            elif slave_ns == self.robot_namespace:
                # If the message is about itself (the master)
                slave = self
            else:
                # Log a warning for unknown slaves
                self.get_logger().warn(f"[{self.robot_namespace}] Received status from unknown slave {slave_ns}.")
                return

            # Update the last seen time for the slave
            slave.last_seen_time = current_time

            # Handle status cases
            if status == "reached":
                # If the waypoint was reached, assign the next waypoint and handle waiting slaves
                slave.waiting = False  # Mark the slave as not waiting
                self.assign_next_waypoint(slave_ns)
                self.assign_waiting_slaves()
            elif status == "error":
                # Handle errors by removing the slave and reassigning tasks
                self.get_logger().error(f"[{self.robot_namespace}] Slave {slave_ns} encountered an error: {error_message}")
                if slave_ns in self.active_slaves:
                    # Remove the slave from active_slaves
                    del self.active_slaves[slave_ns]
                    self.get_logger().warn(f"[{self.robot_namespace}] Removing slave {slave_ns} due to error.")
                    # Repartition the graph and reassign tasks
                    self.partition_and_assign_waypoints()
        else:
            # If this node is not the master, no additional handling is required
            pass

    def print_subgraphs(self, subgraphs):
        """
        Utility function to print details about subgraphs after partitioning.

        This is useful for debugging and understanding how the graph was divided among the slaves.

        Args:
            subgraphs (list of nx.Graph): List of subgraphs resulting from the partitioning process.
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

    def orientation_conversion(self, orientation_input):
        """
        Convert orientation from a string or numeric input to radians.

        This utility helps translate compass directions like 'NORTH' or 'WEST'
        into numerical values in radians, which are required for navigation.

        Args:
            orientation_input (str or float): Orientation as a string ('NORTH', 'EAST', etc.)
                                            or as a numeric value (radians).

        Returns:
            float: Orientation in radians.
        """
        # Mapping of compass directions to radians
        orientations = {
            'NORTH': 0.0,
            'EAST': -math.pi / 2,
            'SOUTH': math.pi,
            'WEST': math.pi / 2
        }
        if isinstance(orientation_input, str):
            # Convert string orientation to radians
            return orientations.get(orientation_input.upper(), 0.0)
        elif isinstance(orientation_input, (float, int)):
            # If it's already a number, return it as a float
            return float(orientation_input)
        else:
            # Default to 0 radians for invalid inputs
            return 0.0



    def partition_and_assign_waypoints(self):
        """
        Partition the graph and assign waypoints to each slave if this node becomes the master.

        This method divides the graph into subgraphs for each slave and assigns waypoints
        to each one, including the master. It ensures that all slaves receive their 
        respective routes based on their assigned subgraph.

        Workflow:
        1. Check if the navigation graph is available.
        2. Gather all active slaves, including the master node.
        3. Determine the start positions for each slave.
        4. Partition the graph into subgraphs using the `partition_graph` function.
        5. Assign the resulting waypoints to each slave based on their subgraph.
        """
        if self.navigation_graph is None:
            # Log an error if no navigation graph is available and exit.
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot partition and assign waypoints.")
            return

        # Reference to the full navigation graph
        full_graph = self.navigation_graph

        # List of all slaves, including this node
        all_slaves = list(self.active_slaves.keys()) + [self.robot_namespace]

        # Sort the slave list for consistent ordering
        all_slaves_sorted = sorted(all_slaves)

        # Total number of slaves (including the master node)
        num_slaves = len(all_slaves_sorted)

        # Determine the starting positions for each slave
        start_positions = []
        for slave_ns in all_slaves_sorted:
            if slave_ns == self.robot_namespace:
                # For the master, use its initial position
                start_positions.append({'x': self.initial_x, 'y': self.initial_y})
            else:
                # For other slaves, use their recorded initial positions if available
                if slave_ns in self.slave_initial_positions:
                    pos = self.slave_initial_positions[slave_ns]
                    start_positions.append({'x': pos['x'], 'y': pos['y']})
                else:
                    # Fallback to the master's position if slave's position is unavailable
                    self.get_logger().warn(
                        f"[{self.robot_namespace}] Initial position for slave '{slave_ns}' not available. Using master position as fallback."
                    )
                    start_positions.append({'x': self.initial_x, 'y': self.initial_y})

        # Partition the graph into subgraphs for each slave
        try:
            subgraphs = partition_graph(full_graph, num_slaves, start_positions=start_positions)
            self.get_logger().info(f"[{self.robot_namespace}] Partitioned the graph into {len(subgraphs)} subgraphs.")
        except ValueError as e:
            # Handle errors during graph partitioning
            self.get_logger().error(f"[{self.robot_namespace}] Failed to partition graph: {e}")
            return

        # Assign waypoints from each subgraph to the respective slave
        for idx, slave_ns in enumerate(all_slaves_sorted):
            # Get the subgraph corresponding to the current slave
            subgraph = subgraphs[idx]

            # Extract waypoints from the subgraph
            waypoints = self.extract_waypoints(subgraph)

            if slave_ns == self.robot_namespace:
                # Assign waypoints to the master node itself
                self.assign_route_to_master(waypoints)
            else:
                # Assign waypoints to other slaves
                # Create a publisher for the slave if it doesn't already exist
                if slave_ns not in self.slave_command_publishers:
                    topic_name = f"/{slave_ns}/navigation_commands"
                    publisher = self.create_publisher(String, topic_name, 10)
                    self.slave_command_publishers[slave_ns] = publisher
                    self.get_logger().info(
                        f"[{self.robot_namespace}] Created publisher for slave '{slave_ns}' on topic '{topic_name}'."
                    )

                # Get the publisher for the slave
                publisher = self.slave_command_publishers[slave_ns]

                # Send each waypoint to the slave
                for waypoint in waypoints:
                    # Convert orientation from string to radians if necessary
                    if isinstance(waypoint['orientation'], str):
                        orientation_rad = self.orientation_conversion(waypoint['orientation'])
                    else:
                        orientation_rad = waypoint['orientation']

                    # Create a waypoint message
                    waypoint_msg = {
                        'label': waypoint['label'],
                        'x': waypoint['x'],
                        'y': waypoint['y'],
                        'orientation': orientation_rad
                    }
                    # Publish the waypoint to the slave
                    msg = String()
                    msg.data = json.dumps(waypoint_msg)
                    publisher.publish(msg)

                    # Log the assigned waypoint
                    self.get_logger().info(f"[{self.robot_namespace}] Assigned waypoint to {slave_ns}: {waypoint_msg}")

        # Mark the graph as partitioned
        self.master_graph_partitioned = True

    def assign_route_to_master(self, waypoints):
        """
        Assign a DCPP (Eulerian) route to the master robot.

        Steps:
        1. Use the waypoints and the navigation graph to calculate an Eulerian circuit.
        2. Store the route in the `assigned_waypoints` list for the master.
        3. Log the assigned waypoints for debugging purposes.
        4. Start executing the route by assigning the first waypoint.

        Args:
            waypoints (list): List of waypoints representing the master robot's subgraph.
        """
        # Calculate the DCPP route using the provided waypoints and graph
        dcpp_route = calculate_dcpp_route(waypoints, self.navigation_graph, self.get_logger())

        # Store the calculated route
        self.assigned_waypoints = dcpp_route.copy()

        # Initialize the waypoint index to start from the beginning of the route
        self.current_waypoint_index = 0

        # Log the assigned route for debugging
        self.get_logger().info(f"[{self.robot_namespace}] DCPP route assigned as master:")
        for wp in dcpp_route:
            self.get_logger().info(
                f" - {wp['label']} at ({wp['x']}, {wp['y']}) with orientation {wp['orientation']} radians"
            )

        # Begin execution of the route by assigning the first waypoint
        self.assign_next_waypoint_as_master()

        # Log the number of waypoints assigned
        self.get_logger().info(f"[{self.robot_namespace}] Assigned {len(dcpp_route)} waypoints as master.")

    def assign_next_waypoint_as_master(self):
        """
        Assign the next waypoint in the route to the master robot.

        Workflow:
        1. Check if there are waypoints assigned. If none, log a warning and exit.
        2. If the current index is valid:
            - Retrieve the waypoint at the current index.
            - Execute navigation to the waypoint.
        3. If all waypoints have been visited:
            - Log that the route will restart.
            - Reset the index to 0 and assign the first waypoint.

        Notes:
        - This method ensures the master continues navigation indefinitely by cycling through its route.
        """
        if not self.assigned_waypoints:
            # Log a warning if no waypoints are assigned
            self.get_logger().warn(f"[{self.robot_namespace}] No waypoints assigned. Cannot assign next waypoint.")
            return

        if self.current_waypoint_index < len(self.assigned_waypoints):
            # Retrieve and navigate to the current waypoint
            waypoint = self.assigned_waypoints[self.current_waypoint_index]
            self.execute_navigation(waypoint)
        else:
            # If all waypoints are completed, restart the route
            self.get_logger().info(f"[{self.robot_namespace}] All waypoints have been assigned. Restarting the route.")
            self.current_waypoint_index = 0
            waypoint = self.assigned_waypoints[self.current_waypoint_index]
            self.execute_navigation(waypoint)


def main(args=None):
    """
    Entry point of the program.
    - Parses command-line arguments to configure the simulated or real slave node.
    - Initializes ROS 2 and starts the slave node.
    
    Workflow:
    1. Parse input arguments to specify:
        - `robot_namespace`: Unique identifier for the robot (namespace).
        - `initial_node_label`: The label of the graph node where the robot starts.
        - `initial_orientation`: The robot's initial orientation (e.g., NORTH, EAST).
    2. Initialize ROS 2.
    3. Create and run the `SlaveNavigationNode`.
    4. Keep the node running until a termination signal (like Ctrl+C).
    """

    # Create an argument parser to handle input from the command line.
    parser = argparse.ArgumentParser(description='Slave Navigation Node using TurtleBot4 with Master Replacement')
    
    # Add command-line arguments:
    # --robot_namespace: Specifies the unique namespace for this robot.
    parser.add_argument(
        '--robot_namespace',
        type=str,
        required=True,  # This argument must be provided.
        help='Unique namespace of the robot (e.g., robot_1)'
    )
    
    # --initial_node_label: The label of the node where the robot starts in the navigation graph.
    parser.add_argument(
        '--initial_node_label',
        type=str,
        required=True,  # This argument must also be provided.
        help='Initial node label (e.g., node_5)'
    )
    
    # --initial_orientation: The initial orientation of the robot as a string.
    parser.add_argument(
        '--initial_orientation',
        type=str,
        required=True,  # Must specify the orientation (NORTH, EAST, etc.).
        help='Initial orientation (NORTH, EAST, SOUTH, WEST)'
    )

    # Parse the arguments passed to the script. 
    # `parsed_args` will contain the values of the defined arguments.
    # `unknown` captures any extra arguments that were not defined in the parser.
    parsed_args, unknown = parser.parse_known_args()

    # Initialize the ROS 2 client library.
    # This must be called before any ROS 2-related operation (e.g., creating nodes).
    rclpy.init(args=None)

    # Create an instance of the `SlaveNavigationNode` using the parsed arguments.
    # Pass the required parameters:
    # - `robot_namespace`: The namespace for the node (e.g., "robot_1").
    # - `initial_node_label`: The graph node label where the robot starts.
    # - `initial_orientation_str`: The string representing the robot's starting orientation.
    node = SlaveNavigationNode(
        robot_namespace=parsed_args.robot_namespace,
        initial_node_label=parsed_args.initial_node_label,
        initial_orientation_str=parsed_args.initial_orientation
    )

    try:
        # Start the node's main loop.
        # The `rclpy.spin()` function blocks execution, continuously listening for callbacks 
        # and processing them as events occur.
        # This keeps the node running indefinitely until a shutdown signal is received.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # If the user interrupts the program (e.g., presses Ctrl+C),
        # the exception is caught, and we gracefully shut down the node.
        pass
    finally:
        # Cleanup resources when the program exits:
        # 1. Destroy the node, releasing any resources it is using.
        node.destroy_node()
        # 2. Shutdown the ROS 2 client library, ensuring all resources are cleaned up.
        rclpy.shutdown()

# This block ensures that the `main` function is only executed when the script
# is run directly (not when it is imported as a module).
if __name__ == '__main__':
    main()

