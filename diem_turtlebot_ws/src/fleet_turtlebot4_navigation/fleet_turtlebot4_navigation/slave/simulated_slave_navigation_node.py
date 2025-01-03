#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math
import networkx as nx
from threading import Lock, Event
import random
import sys

# Master Tools
from fleet_turtlebot4_navigation.master.master_callbacks import MasterCallbacks
from fleet_turtlebot4_navigation.master.heartbeat_manager import HeartbeatManager
from fleet_turtlebot4_navigation.master.waypoint_manager import WaypointManager
from fleet_turtlebot4_navigation.master.graph_utils import load_full_graph_from_data
# from fleet_turtlebot4_navigation.master.slave_state import SlaveState


class SimulatedSlaveNavigationNode(Node, MasterCallbacks):
    """
    Simulated Slave Navigation Node that can elect itself as Master in case of Master failure.
    
    Questo nodo simula un robot slave in una flotta. In caso di crash o fallimento del nodo Master,
    partecipa a un processo di elezione per diventare il nuovo Master. Una volta eletto come Master, 
    assume responsabilità come l'assegnazione dei waypoints ad altri slaves. Quando opera come slave,
    naviga attraverso i waypoints sequenzialmente, simulando la navigazione con ritardi temporali.
    
    Implements the Bully Algorithm for leader election to ensure only one Master exists at any time.
    """

    def __init__(self, robot_namespace='robot1', initial_node_label='node_4'):
        """
        Initializes the SimulatedSlaveNavigationNode.
        
        Parameters:
            robot_namespace (str, optional):
                The unique namespace identifier for this slave robot. Defaults to 'robot1'.
            
            initial_node_label (str, optional):
                The label of the node where the slave starts navigating. Defaults to 'node_4'.
        """
        # Initialize the ROS2 node with the given namespace and name
        super().__init__('simulated_slave_navigation_node', namespace=robot_namespace)
        
        # Initialize MasterCallbacks to enable Master-specific functionalities
        MasterCallbacks.__init__(self)  # Enables functions like compute_global_cpp_route
        
        # Store the robot's namespace and initial node label
        self.robot_namespace = robot_namespace
        self.initial_node_label = initial_node_label
        
        # ------------------------------
        # Slave State Initialization
        # ------------------------------
        self.is_master = False  # Indicates if this node has become Master
        self.master_alive = True  # Tracks if the Master is alive
        self.last_master_heartbeat = time.time()  # Timestamp of the last Master heartbeat
        self.heartbeat_timeout = 100.0  # Time in seconds before considering Master dead
        
        # Declare and retrieve the 'timeout' parameter from ROS2 parameters
        self.declare_parameter('timeout', 5.0)  # Default timeout: 5 seconds
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # Initialize a thread-safe set to track active slaves
        self.active_slaves = set()
        self.active_slaves_lock = Lock()
        
        # ------------------------------
        # Navigation State Initialization
        # ------------------------------
        self.graph_received = False  # Indicates if the navigation graph has been received
        self.navigation_graph = None  # The navigation graph
        self.current_node = initial_node_label  # Current node label
        
        self.assigned_waypoints = []  # Queue of waypoints assigned to this slave
        self.is_navigating = False  # Flag indicating if the slave is currently navigating
        self.navigation_lock = Lock()  # Lock to protect navigation operations
        
        # ------------------------------
        # Master State Initialization (Inactive Initially)
        # ------------------------------
        self.slaves = {}  # Dictionary mapping slave_ns to SlaveState instances
        self.occupied_edges = set()  # Set of edges currently occupied by slaves
        self.edge_occupants = {}  # Mapping from edges to occupying slave_ns
        self.partitioning_done = False  # Flag indicating if partitioning is done
        self.heartbeat_manager = None  # HeartbeatManager instance (active if Master)
        self.waypoint_manager = None  # WaypointManager instance (active if Master)
        
        # Declare and retrieve the 'check_interval' parameter for Master timers
        self.declare_parameter('check_interval', 5.0)
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        
        # ------------------------------
        # Publisher and Subscriber Setup for Slave
        # ------------------------------
        # Publisher for slave registration messages
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)
        # Publisher for slave heartbeat messages
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)
        # Publisher for navigation status updates
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)
        
        # Publisher for Bully Algorithm messages
        self.election_publisher = self.create_publisher(String, '/election', 10)
        self.coordinator_publisher = self.create_publisher(String, '/coordinator', 10)
        
        # Subscriber for navigation commands (receives waypoints)
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.slave_navigation_commands_callback, 10
        )
        # Subscriber for receiving Master heartbeat messages
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 1
        )
        # Subscriber for receiving the navigation graph
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.slave_navigation_graph_callback, 10
        )
        # Subscriber for Election messages
        self.election_subscriber = self.create_subscription(
            String, '/election', self.election_callback, 10
        )
        # Subscriber for Coordinator messages
        self.coordinator_subscriber = self.create_subscription(
            String, '/coordinator', self.coordinator_callback, 10
        )
        
        # ------------------------------
        # Timer Setup for Slave Operations
        # ------------------------------
        # Timer to periodically publish slave registration messages
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        # Timer to periodically publish slave heartbeat messages
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        
        # Timer to periodically check if the Master is alive
        self.check_master_timer = self.create_timer(1.0, self.check_master_alive)
        
        # ------------------------------
        # Flags and Events for Election
        # ------------------------------
        self.election_in_progress = False  # Indicates if an election is ongoing
        self.election_event = Event()  # Event to signal election completion
        
        # ------------------------------
        # Randomized Start-up Delay to Reduce Election Conflicts
        # ------------------------------
        startup_delay = random.uniform(0, 2)  # Random delay between 0 and 2 seconds
        self.get_logger().info(f"[{self.robot_namespace}] Starting with a delay of {startup_delay:.2f} seconds to reduce election conflicts.")
        time.sleep(startup_delay)
        
        # Log the successful initialization of the slave node
        self.get_logger().info(
            f"[{self.robot_namespace}] Started as SLAVE simulator (initial node='{self.initial_node_label}')"
        )
    
    # =========================================================================
    #                           SLAVE LOGIC
    # =========================================================================
    
    def publish_registration(self):
        """
        Publishes a registration message to announce this node as a SLAVE.
        
        This method sends the robot's namespace to the '/slave_registration' topic, allowing the Master
        to recognize and track this slave.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published slave registration.")
    
    def publish_heartbeat(self):
        """
        Publishes a heartbeat message to indicate that this SLAVE is alive.
        
        This method sends the robot's namespace to the '/slave_heartbeat' topic, enabling the Master to monitor
        the activity and health of this slave.
        """
        hb = String()
        hb.data = self.robot_namespace
        self.heartbeat_publisher.publish(hb)
        self.get_logger().debug(f"[{self.robot_namespace}] Published slave heartbeat.")
    
    def master_heartbeat_callback(self, msg):
        """
        Callback to handle heartbeat messages received from the Master.
        
        Parameters:
            msg (String):
                The received heartbeat message containing the Master's namespace.
        """
        self.master_alive = True  # Reset the alive flag upon receiving a heartbeat
        self.last_master_heartbeat = time.time()  # Update the timestamp of the last heartbeat
        self.get_logger().debug(f"[{self.robot_namespace}] Received Master heartbeat.")
    
    def check_master_alive(self):
        """
        Periodically checks if the Master is alive based on heartbeat messages.
        
        If no heartbeat has been received within the 'heartbeat_timeout' duration, initiates an election.
        """
        if self.is_master:
            # If already Master, no need to check
            return
        
        if self.master_alive:
            # Reset the alive flag to check for future heartbeats
            self.master_alive = False
        else:
            # Calculate the time since the last heartbeat
            now = time.time()
            if now - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost => Initiating Election!")
                self.elect_new_master()
    
    def elect_new_master(self):
        """
        Initiates the election process to select a new Master using the Bully Algorithm.
        
        Steps:
            1. Set election_in_progress flag.
            2. Send ELECTION messages to all nodes with higher identifiers.
            3. Wait for OK responses within a timeout.
            4. If no OK responses, become Master and announce via COORDINATOR messages.
            5. If OK responses are received, wait for COORDINATOR message.
        """
        if self.election_in_progress:
            self.get_logger().info(f"[{self.robot_namespace}] Election already in progress. Waiting for coordinator.")
            return
        
        self.election_in_progress = True
        self.election_event.clear()
        
        # Identify nodes with higher namespace identifiers
        higher_nodes = [node for node in self.active_slaves if node > self.robot_namespace]
        
        if not higher_nodes:
            # No higher nodes, become Master
            self.get_logger().info(f"[{self.robot_namespace}] No higher nodes found. Becoming MASTER!")
            self.become_master()
            self.election_in_progress = False
            self.election_event.set()
            return
        
        # Send ELECTION messages to higher nodes
        for node in higher_nodes:
            election_msg = {
                "type": "ELECTION",
                "sender": self.robot_namespace
            }
            msg = String()
            msg.data = json.dumps(election_msg)
            self.election_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Sent ELECTION message to '{node}'.")
        
        # Wait for OK responses within a timeout
        wait_time = 5  # seconds
        self.get_logger().info(f"[{self.robot_namespace}] Waiting for OK responses for {wait_time} seconds.")
        self.election_event.wait(wait_time)
        
        if not self.election_in_progress:
            # Another node is taking over the election
            self.get_logger().info(f"[{self.robot_namespace}] Another node is handling the election.")
            return
        
        # If no OK responses, become Master
        self.get_logger().info(f"[{self.robot_namespace}] No OK responses received. Becoming MASTER!")
        self.become_master()
        self.election_in_progress = False
        self.election_event.set()
    
    # =========================================================================
    # Master Election Callbacks
    # =========================================================================
    
    def election_callback(self, msg):
        """
        Callback to handle incoming ELECTION messages from other nodes.
        
        Parameters:
            msg (String):
                The received ELECTION message in JSON format.
        """
        try:
            election_msg = json.loads(msg.data)
            if election_msg.get("type") != "ELECTION":
                return  # Ignore other message types
            sender = election_msg.get("sender")
            if sender and sender < self.robot_namespace:
                # Respond with OK message
                ok_msg = {
                    "type": "OK",
                    "sender": self.robot_namespace
                }
                response = String()
                response.data = json.dumps(ok_msg)
                self.election_publisher.publish(response)
                self.get_logger().info(f"[{self.robot_namespace}] Received ELECTION from '{sender}'. Sent OK.")
                
                # Start own election
                self.elect_new_master()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding ELECTION message: {e}")
    
    def coordinator_callback(self, msg):
        """
        Callback to handle incoming COORDINATOR messages from the new Master.
        
        Parameters:
            msg (String):
                The received COORDINATOR message in JSON format.
        """
        try:
            coord_msg = json.loads(msg.data)
            if coord_msg.get("type") != "COORDINATOR":
                return  # Ignore other message types
            sender = coord_msg.get("sender")
            if sender:
                self.get_logger().info(f"[{self.robot_namespace}] Received COORDINATOR message from '{sender}'. Recognizing as new Master.")
                self.is_master = False
                self.master_alive = True
                self.last_master_heartbeat = time.time()
                self.election_in_progress = False
                self.election_event.set()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding COORDINATOR message: {e}")
    
    # =========================================================================
    # Slave Navigation Graph Callback
    # =========================================================================
    
    def slave_navigation_graph_callback(self, msg):
        """
        Callback to handle the reception of the navigation graph from the Master.
        
        Parameters:
            msg (String):
                The received navigation graph message in JSON format.
        """
        if self.is_master:
            # If operating as Master, ignore incoming graph messages
            return
        
        # If the graph is already received and processed, ignore further messages
        if self.graph_received:
            self.get_logger().debug(f"[{self.robot_namespace}] Graph message ignored because the graph has already been received.")
            return
        
        try:
            data = json.loads(msg.data)  # Parse the JSON data
            self.navigation_graph = load_full_graph_from_data(data)  # Load the graph using utility function
            self.graph_received = True  # Mark that the graph has been received and processed

            # Check if the initial node exists in the navigation graph
            if self.initial_node_label in self.navigation_graph.nodes:
                self.current_node = self.initial_node_label
                self.get_logger().info(
                    f"[{self.robot_namespace}] Received graph => initial node='{self.initial_node_label}'"
                )
            else:
                # Log an error if the initial node is not present in the graph
                self.get_logger().error(
                    f"[{self.robot_namespace}] initial_node '{self.initial_node_label}' not found in the graph!"
                )
                return

            # Publish a 'ready' status message to inform the Master that the slave is ready
            ready_data = {
                'robot_namespace': self.robot_namespace,
                'status': 'ready',
                'error_message': '',
                'time_taken': 0.0,
                'current_waypoint': self.current_node,
                'traversed_edge': []
            }
            rmsg = String()
            rmsg.data = json.dumps(ready_data)
            self.status_publisher.publish(rmsg)
            self.get_logger().info(f"[{self.robot_namespace}] Published 'ready' status (SLAVE).")
        
        except json.JSONDecodeError as e:
            # Handle JSON parsing errors
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding graph as slave: {e}")
    
    # =========================================================================
    # Slave Navigation Commands Callback
    # =========================================================================
    
    def slave_navigation_commands_callback(self, msg):
        """
        Callback to handle navigation commands (waypoints) received when operating as SLAVE.
        
        Parameters:
            msg (String):
                The received navigation command message in JSON format.
        """
        if self.is_master:
            # If operating as Master, ignore navigation commands
            return
        
        try:
            waypoint_data = json.loads(msg.data)  # Parse the JSON data
        except json.JSONDecodeError as e:
            # Handle JSON parsing errors
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding waypoint: {e}")
            return

        if not self.graph_received or (self.navigation_graph is None):
            # If no navigation graph has been received, log a warning and publish an error status
            self.get_logger().warn(f"[{self.robot_namespace}] No graph available, cannot navigate as SLAVE.")
            self.publish_status("error", "NoGraph", 0.0, waypoint_data.get('label','?'), [])
            return

        with self.navigation_lock:
            # Add the received waypoint to the queue of assigned waypoints
            self.assigned_waypoints.append(waypoint_data)
            self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")

            if not self.is_navigating:
                # If not currently navigating, start executing waypoints
                self.is_navigating = True
                self.execute_waypoints_in_sequence()
    
    def execute_waypoints_in_sequence(self):
        """
        Executes assigned waypoints sequentially in a synchronous manner.
        
        This method processes the assigned_waypoints queue, navigating to each waypoint one by one.
        It simulates navigation by introducing time delays based on the distance to the waypoint.
        """
        while self.assigned_waypoints:
            wpt = self.assigned_waypoints.pop(0)  # Get the next waypoint
            self.simulate_navigation(wpt)  # Simulate navigation to the waypoint

        self.is_navigating = False  # Mark navigation as complete
    
    def simulate_navigation(self, waypoint):
        """
        Simulates navigation to a single waypoint with time delays.
        
        Parameters:
            waypoint (dict):
                The waypoint data containing 'label', 'x', and 'y' coordinates.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']

        if self.current_node == label:
            # If already at the target node, log the information and publish a 'reached' status
            self.get_logger().info(f"[{self.robot_namespace}] Already at '{label}'.")
            # self.publish_status("reached", "", 0.0, label, [label, label])
            return

        # Check if both current node and target node exist in the navigation graph
        if (self.current_node not in self.navigation_graph.nodes) or (label not in self.navigation_graph.nodes):
            # Log an error and publish an 'error' status if nodes are missing
            err_msg = f"Current node='{self.current_node}' or destination='{label}' not in the graph"
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])
            return

        # Calculate the distance between the current node and the target waypoint
        cx = self.navigation_graph.nodes[self.current_node]['x']
        cy = self.navigation_graph.nodes[self.current_node]['y']
        dist = math.hypot(x - cx, y - cy)  # Euclidean distance
        speed = 10.31  # Simulated speed (units per second)
        ttime = dist / speed  # Time to reach the waypoint

        trav_edge = [self.current_node, label]  # The edge being traversed
        self.publish_status("traversing", "", 0.0, label, trav_edge)  # Publish 'traversing' status
        self.get_logger().info(
            f"[{self.robot_namespace}] Navigating from '{self.current_node}' to '{label}' ~{ttime:.2f}s."
        )

        time.sleep(ttime)  # Simulate navigation delay
        # Update the current node to the waypoint after 'navigation'
        self.current_node = label
        self.get_logger().info(f"[{self.robot_namespace}] Reached '{label}' in {ttime:.2f}s.")
        self.publish_status("reached", "", ttime, label, trav_edge)  # Publish 'reached' status
    
    def publish_status(self, status, error_message, time_taken, current_waypoint, trav):
        """
        Publishes the navigation status of the SLAVE.
        
        Parameters:
            status (str):
                The current status of navigation (e.g., 'traversing', 'reached', 'error').
            
            error_message (str):
                Any error messages related to navigation. Empty string if no errors.
            
            time_taken (float):
                The time taken to reach the current waypoint.
            
            current_waypoint (str):
                The label of the current waypoint being navigated to.
            
            trav (list):
                The edge being traversed, represented as a list of node labels.
        """
        st = {
            'robot_namespace': self.robot_namespace,
            'status': status,
            'error_message': error_message,
            'time_taken': time_taken,
            'current_waypoint': current_waypoint,
            'traversed_edge': trav
        }
        s = String()
        s.data = json.dumps(st)  # Convert the status dictionary to JSON string
        self.status_publisher.publish(s)  # Publish the status message
        self.get_logger().debug(f"[{self.robot_namespace}] Published status: {st}")
    
    # =========================================================================
    # Diventare MASTER (Become Master)
    # =========================================================================
    
    def become_master(self):
        """
        Transitions this node from SLAVE to MASTER role.
        
        Upon becoming Master, the node performs the following actions:
            1. Initializes HeartbeatManager to start publishing Master heartbeats.
            2. Initializes WaypointManager to handle waypoint assignments.
            3. Resets Master-specific tracking structures.
            4. Subscribes to SLAVE registration and navigation status topics.
            5. Publishes the navigation graph to inform all slaves.
            6. Computes the global CPP route for waypoint distribution.
            7. Assigns waypoints to active slaves.
            8. Sets up Master-specific timers for periodic checks.
            9. Liberates le occupied_edges e ricomincia il ciclo.
            10. Interrompe la pubblicazione dei heartbeat come Slave.
            11. Rimuove se stesso dall'insieme degli active_slaves.
        """
        self.is_master = True  # Mark this node as Master
        self.get_logger().info(f"[{self.robot_namespace}] => MASTER MODE activated")
        
        # 1) Initialize HeartbeatManager to publish Master heartbeats immediately
        self.heartbeat_manager = HeartbeatManager(self)
        self.heartbeat_manager.start_publishing()  # Start publishing heartbeats immediately
        
        # 2) Initialize WaypointManager to manage waypoint assignments
        self.waypoint_manager = WaypointManager(self)
        
        # 3) Reset Master-specific tracking structures
        self.slaves = {}
        self.occupied_edges.clear()
        self.edge_occupants.clear()
        self.get_logger().debug(f"[{self.robot_namespace}] Cleared occupied_edges and edge_occupants.")
        
        # 4) Subscribe to SLAVE registration and navigation status topics
        self.slave_registration_subscriber = self.create_subscription(
            String, '/slave_registration', self.slave_registration_callback, 10
        )
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )
        
        # 5) If a navigation graph has been received, use it as the full_graph
        if self.navigation_graph is not None:
            self.full_graph = self.navigation_graph
            self.get_logger().info(f"[{self.robot_namespace}] (MASTER) Using navigation_graph as full_graph")

            # Initialize publisher to broadcast the navigation graph to all slaves
            self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)
            self.graph_timer = self.create_timer(1.0, self.publish_navigation_graph)

            # 6) Compute the global CPP route for waypoint distribution
            self.compute_global_cpp_route()
            self.get_logger().info(f"Global CPP route computed: {self.global_cpp_route}")

            # 7) Assign waypoints to active slaves using the WaypointManager
            self.waypoint_manager.repartition_and_assign_waypoints()
            
            # 9) Liberate le occupied_edges e ricomincia il ciclo
            # Assicurati che WaypointManager resetti il proprio stato se necessario
            if hasattr(self.waypoint_manager, 'reset_state'):
                self.waypoint_manager.reset_state()
                self.get_logger().debug(f"[{self.robot_namespace}] WaypointManager state reset.")
            
            self.get_logger().info(f"[{self.robot_namespace}] => Master has reset the cycle by freeing occupied_edges.")
        else:
            # Log a warning if no navigation graph is available to assign routes
            self.get_logger().warn(f"[{self.robot_namespace}] No navigation graph available as Master => cannot assign routes.")
        
        # 8) Initialize Master-specific timer for periodic checks (e.g., slave timeouts, waiting slaves)
        self.master_timer = self.create_timer(self.check_interval, self.master_timer_callback)
        
        # 10) Interrompe la pubblicazione dei heartbeat come Slave
        # Cancella il timer che pubblica i heartbeat sul topic '/slave_heartbeat'
        self.heartbeat_timer.cancel()
        self.get_logger().info(f"[{self.robot_namespace}] Stopped publishing slave heartbeat.")
        
        # 11) Rimuove se stesso dall'insieme degli active_slaves
        # Questo evita che il Master si assegni waypoint a se stesso
        with self.active_slaves_lock:
            if self.robot_namespace in self.active_slaves:
                self.active_slaves.remove(self.robot_namespace)
                self.get_logger().info(f"[{self.robot_namespace}] Removed itself from active_slaves.")
        
        # Log that Master setup is complete and ready to listen to slaves
        self.get_logger().info(f"[{self.robot_namespace}] => Master setup complete, ready to listen to slaves.")
    
    def master_timer_callback(self):
        """
        Periodic callback executed by the Master timer.
        
        Performs the following actions:
            - Checks for any slaves that have timed out and handles their removal.
            - Assigns waypoints to any slaves that are currently waiting for edge availability.
        """
        self.check_slaves_timeout()  # Check for slave timeouts
        self.waypoint_manager.assign_waiting_slaves()  # Assign waypoints to waiting slaves
    
    # =========================================================================
    # Override MasterCallbacks Methods (Only if is_master)
    # =========================================================================
    
    def slave_registration_callback(self, msg):
        """
        Callback to handle the registration of new slaves.
        
        Overrides the MasterCallbacks' method to ensure that only the Master processes slave registrations.
        
        Parameters:
            msg (String):
                The received slave registration message containing the slave's namespace.
        """
        if not self.is_master:
            # If not Master, ignore slave registration messages
            return
        # Call the superclass method to handle the registration logic
        super().slave_registration_callback(msg)
    
    def navigation_status_callback(self, msg):
        """
        Callback to handle navigation status updates from slaves.
        
        Overrides the MasterCallbacks' method to ensure that only the Master processes navigation statuses.
        
        Parameters:
            msg (String):
                The received navigation status message in JSON format.
        """
        if not self.is_master:
            # If not Master, ignore navigation status messages
            return
        # Call the superclass method to handle the navigation status logic
        super().navigation_status_callback(msg)
    
    # =========================================================================
    # Master Election - Bully Algorithm Specific Methods
    # =========================================================================
    
    def send_coordinator_message(self):
        """
        Sends a COORDINATOR message to announce this node as the new Master.
        """
        coord_msg = {
            "type": "COORDINATOR",
            "sender": self.robot_namespace
        }
        msg = String()
        msg.data = json.dumps(coord_msg)
        self.coordinator_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Sent COORDINATOR message.")
    
    # =========================================================================
    # ESECUZIONE (Execution)
    # =========================================================================
    
    def run(self):
        """
        Spins the ROS2 node to keep it active and processing callbacks.
        """
        rclpy.spin(self)
    
    def destroy_node(self):
        """
        Destroys the ROS2 node gracefully.
        """
        if self.is_master and self.heartbeat_manager:
            self.heartbeat_manager.stop_publishing()  # Stop publishing heartbeats if Master
        super().destroy_node()


def main(args=None):
    """
    The main entry point for the SimulatedSlaveNavigationNode.
    
    Parses command-line arguments to set the robot namespace and initial node label,
    initializes the node, and starts spinning it to process ROS2 callbacks.
    
    Parameters:
        args (list, optional):
            Command-line arguments passed to the node. Defaults to None.
    """
    rclpy.init(args=args)  # Initialize the ROS2 Python client library

    import argparse
    # Setup argument parser for command-line arguments
    parser = argparse.ArgumentParser(description='Simulated Slave that can become Master and truly assign waypoints.')
    parser.add_argument('--robot_namespace', type=str, default='robot1', help='Namespace of this slave')
    parser.add_argument('--initial_node_label', type=str, default='node_4', help='Initial node label for simulation.')
    parsed_args, unknown = parser.parse_known_args()

    # Instantiate the SimulatedSlaveNavigationNode with provided arguments
    node = SimulatedSlaveNavigationNode(
        robot_namespace=parsed_args.robot_namespace,
        initial_node_label=parsed_args.initial_node_label
    )
    try:
        node.run()  # Start spinning the node to process callbacks
    except KeyboardInterrupt:
        # Allow the node to be interrupted gracefully via keyboard (e.g., Ctrl+C)
        pass
    finally:
        node.destroy_node()  # Explicitly destroy the node (optional but recommended)
        rclpy.shutdown()  # Shutdown the ROS2 client library





if __name__ == '__main__':
    main()