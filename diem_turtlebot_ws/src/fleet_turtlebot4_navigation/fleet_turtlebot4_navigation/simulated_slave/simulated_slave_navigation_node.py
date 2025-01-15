#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math
import random
from threading import Lock, Event

# Required imports for QoS profile configuration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Master Tools (only import these if you need the master logic):
from fleet_turtlebot4_navigation.master.master_callbacks import MasterCallbacks
from fleet_turtlebot4_navigation.master.heartbeat_manager import HeartbeatManager
from fleet_turtlebot4_navigation.master.waypoint_manager import WaypointManager
from fleet_turtlebot4_navigation.master.graph_utils import load_full_graph_from_data

class SimulatedSlaveNavigationNode(Node, MasterCallbacks):
    """
    Simulated Slave Navigation Node that can become a Master in case of Master failure.

    The node:
      - Does not depend on any initial_node_label parameter.
      - Starts with self.current_node = None, meaning its initial position is unknown.
      - When it receives its first waypoint from the Master, it sets self.current_node to that destination if needed.
      - Can become Master using the Bully algorithm (election-based).
    """

    def __init__(self, robot_namespace='robot1', graph_path='/path/to/default_graph.json'):
        """
        Initialize the SimulatedSlaveNavigationNode in SLAVE mode.
        
        Args:
            robot_namespace (str): The namespace of this robot. Default 'robot1'.
            graph_path (str): The path to the navigation graph in JSON format.
        """
        # Call the Node constructor, naming the node 'simulated_slave_navigation_node'
        # and placing it under the specified namespace
        super().__init__('simulated_slave_navigation_node', namespace=robot_namespace)

        # Initialize MasterCallbacks (if you want to inherit the master's logic)
        MasterCallbacks.__init__(self)
        
        # Store the robot namespace
        self.robot_namespace = robot_namespace
        
        # Define and initialize a QoSProfile with specific settings
        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile.reliability = ReliabilityPolicy.RELIABLE
        self.qos_profile.durability = DurabilityPolicy.VOLATILE
        
        # Store the graph path used when the node becomes the Master
        self.graph_path = graph_path
        
        # Main flags and state variables
        self.is_master = False
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 1000.0
        
        # Declare a parameter 'timeout' (default: 5.0) for controlling slave inactivity
        self.declare_parameter('timeout', 5.0)
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # Thread-safe data structures for tracking active slaves
        self.active_slaves = set()
        self.active_slaves_lock = Lock()
        
        # Navigation state
        self.graph_received = False
        self.navigation_graph = None
        self.current_node = None  # Start with no known position
        self.assigned_waypoints = []
        self.is_navigating = False
        self.navigation_lock = Lock()
        
        # Data structures typical for the Master
        self.slaves = {}
        self.occupied_edges = set()
        self.edge_occupants = {}
        self.partitioning_done = False
        self.heartbeat_manager = None
        self.waypoint_manager = None
        
        # Parameter for the interval of Master checks
        self.declare_parameter('check_interval', 5.0)
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        
        # ------------------------- Publishers (SLAVE side) -------------------------
        # Publish the slave registration info to notify the Master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', self.qos_profile)
        # Publish heartbeat messages from the slave to the Master
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', self.qos_profile)
        # Publish the current navigation status
        self.status_publisher = self.create_publisher(String, '/navigation_status', self.qos_profile)
        
        # Publish election and coordinator messages (for the Bully algorithm)
        self.election_publisher = self.create_publisher(String, '/election', self.qos_profile)
        self.coordinator_publisher = self.create_publisher(String, '/coordinator', self.qos_profile)
        
        # ------------------------- Subscribers (SLAVE side) ------------------------
        # Listen for commands from the Master (waypoints)
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.slave_navigation_commands_callback, self.qos_profile
        )
        # Listen for heartbeats from the Master to detect Master failure
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, self.qos_profile
        )
        # Listen for the global navigation graph broadcast by the Master
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.slave_navigation_graph_callback, self.qos_profile
        )
        # Listen for election messages
        self.election_subscriber = self.create_subscription(
            String, '/election', self.election_callback, self.qos_profile
        )
        # Listen for coordinator messages indicating the new Master
        self.coordinator_subscriber = self.create_subscription(
            String, '/coordinator', self.coordinator_callback, self.qos_profile
        )
        
        # ------------------------- Timers (SLAVE side) ----------------------------
        # Timer that periodically publishes this slave's registration
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        # Timer that periodically publishes heartbeat messages to the Master
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        # Timer that checks periodically if the Master is alive
        self.check_master_timer = self.create_timer(1.0, self.check_master_alive)
        
        # Flags and threading events for election handling
        self.election_in_progress = False
        self.election_event = Event()
        
        # Random startup delay to reduce simultaneous election conflicts
        startup_delay = random.uniform(0, 2)
        self.get_logger().info(
            f"[{self.robot_namespace}] Starting with a delay of {startup_delay:.2f}s to reduce election conflicts."
        )
        time.sleep(startup_delay)
        
        # Log that the node started in SLAVE mode with no initial node
        self.get_logger().info(
            f"[{self.robot_namespace}] Started as SLAVE simulator with NO initial node."
        )
        
        # Track whether we've sent the "first waypoint reached" notification
        self.first_wp_notification_sent = False
        # Publisher for notifying that the first waypoint was reached
        self.first_wp_reached_pub = self.create_publisher(String, '/first_waypoint_reached', self.qos_profile)

    # =========================================================================
    # SLAVE methods: publishing and callbacks
    # =========================================================================
    
    def publish_registration(self):
        """
        Publishes the robot's namespace to the '/slave_registration' topic.
        Helps the Master discover or confirm this slave's existence.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published slave registration.")
    
    def publish_heartbeat(self):
        """
        Publishes a heartbeat signal indicating this slave is alive.
        Helps the Master keep track of active slaves.
        """
        hb = String()
        hb.data = self.robot_namespace
        self.heartbeat_publisher.publish(hb)
        self.get_logger().debug(f"[{self.robot_namespace}] Published slave heartbeat.")
    
    def master_heartbeat_callback(self, msg):
        """
        Callback triggered whenever a Master heartbeat is received.
        Resets the 'master_alive' flag, preventing election from starting.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received Master heartbeat.")
    
    def check_master_alive(self):
        """
        Timer-based check to confirm whether the Master is still alive.
        If the Master has not sent any heartbeat for too long, starts an election.
        """
        # If already Master, no need to check
        if self.is_master:
            return
        
        # The first time we fail to see a heartbeat, set master_alive to False
        if self.master_alive:
            self.master_alive = False
        else:
            # If no heartbeat for over 'heartbeat_timeout' seconds, do election
            now = time.time()
            if now - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost => Initiating Election!")
                self.elect_new_master()

    # =========================================================================
    # Bully Election Algorithm
    # =========================================================================
    
    def elect_new_master(self):
        """
        Initiates the Bully election algorithm.
        This node tries to become Master if it is the 'highest' node or if no higher nodes respond.
        """
        # If already running an election, skip
        if self.election_in_progress:
            self.get_logger().info(f"[{self.robot_namespace}] Election already in progress.")
            return
        
        self.election_in_progress = True
        self.election_event.clear()
        
        # Identify any active slaves with a 'higher' namespace (greater string comparison)
        with self.active_slaves_lock:
            higher_nodes = [node for node in self.active_slaves if node > self.robot_namespace]
        
        # If no higher nodes exist, become Master immediately
        if not higher_nodes:
            self.get_logger().info(f"[{self.robot_namespace}] No higher nodes found. Becoming MASTER!")
            self.become_master()
            self.election_in_progress = False
            self.election_event.set()
            return
        
        # Broadcast election message to higher nodes
        for node in higher_nodes:
            election_msg = {"type": "ELECTION", "sender": self.robot_namespace}
            msg = String()
            msg.data = json.dumps(election_msg)
            self.election_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Sent ELECTION to '{node}'.")
        
        # Wait for responses (OK messages)
        wait_time = 5
        self.get_logger().info(f"[{self.robot_namespace}] Waiting {wait_time}s for OK responses.")
        self.election_event.wait(wait_time)
        
        # If election_in_progress is cleared by an OK, some other node might be taking over
        if not self.election_in_progress:
            self.get_logger().info(f"[{self.robot_namespace}] Another node took over the election.")
            return
        
        # If no OK messages received, become Master
        self.get_logger().info(f"[{self.robot_namespace}] No OK received => Becoming MASTER!")
        self.become_master()
        self.election_in_progress = False
        self.election_event.set()

    def election_callback(self, msg):
        """
        Callback for handling election messages from other nodes.
        """
        try:
            election_msg = json.loads(msg.data)
            # Ignore messages that aren't of type 'ELECTION'
            if election_msg.get("type") != "ELECTION":
                return
            sender = election_msg.get("sender")
            # If the sender is 'lower' than self.robot_namespace, respond with 'OK' and start another election
            if sender and sender < self.robot_namespace:
                ok_msg = {"type": "OK", "sender": self.robot_namespace}
                response = String()
                response.data = json.dumps(ok_msg)
                self.election_publisher.publish(response)
                self.get_logger().info(f"[{self.robot_namespace}] Received ELECTION from '{sender}', sent OK.")
                self.elect_new_master()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding ELECTION message: {e}")

    def coordinator_callback(self, msg):
        """
        Callback for handling coordinator messages from the new Master.
        This means another node has won the election.
        """
        try:
            coord_msg = json.loads(msg.data)
            if coord_msg.get("type") != "COORDINATOR":
                return
            sender = coord_msg.get("sender")
            if sender:
                self.get_logger().info(f"[{self.robot_namespace}] COORDINATOR from '{sender}'. That is the new Master.")
                self.is_master = False
                self.master_alive = True
                self.last_master_heartbeat = time.time()
                self.election_in_progress = False
                self.election_event.set()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding COORDINATOR message: {e}")

    # =========================================================================
    # SLAVE: Receiving Graph
    # =========================================================================
    
    def slave_navigation_graph_callback(self, msg):
        """
        Callback for receiving the navigation graph from the Master.
        Marks the node as 'ready' once the graph is loaded.
        """
        # If we're already the Master, we shouldn't update the local graph from external data
        if self.is_master:
            return
        
        if self.graph_received:
            self.get_logger().debug(f"[{self.robot_namespace}] Graph already received, ignoring.")
            return
        
        try:
            data = json.loads(msg.data)
            # Load the graph structure from JSON data
            self.navigation_graph = load_full_graph_from_data(data)
            self.graph_received = True
            
            # Publish a "ready" status
            ready_data = {
                'robot_namespace': self.robot_namespace,
                'status': 'ready',
                'error_message': '',
                'time_taken': 0.0,
                'current_waypoint': str(self.current_node),  # store None as a string if current_node is None
                'traversed_edge': []
            }
            rmsg = String()
            rmsg.data = json.dumps(ready_data)
            self.status_publisher.publish(rmsg)
            self.get_logger().info(f"[{self.robot_namespace}] Graph received, published 'ready' status.")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding graph as slave: {e}")

    # =========================================================================
    # SLAVE: Receiving Waypoints
    # =========================================================================
    
    def slave_navigation_commands_callback(self, msg):
        """
        Callback for receiving navigation waypoints from the Master.
        Appends waypoints to a queue and starts navigation if not already navigating.
        """
        # If Master, ignore this
        if self.is_master:
            return
        
        # Parse incoming waypoint data
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding waypoint: {e}")
            return
        
        # If no graph is available, we can't navigate
        if not self.graph_received or self.navigation_graph is None:
            self.get_logger().warn(f"[{self.robot_namespace}] No graph available => cannot navigate.")
            self.publish_status("error", "NoGraph", 0.0, waypoint_data.get('label','?'), [])
            return

        # Add the waypoint to the assigned queue, and if not navigating, start
        with self.navigation_lock:
            self.assigned_waypoints.append(waypoint_data)
            self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")
            if not self.is_navigating:
                self.is_navigating = True
                self.execute_waypoints_in_sequence()

    def execute_waypoints_in_sequence(self):
        """
        Continuously execute waypoints in FIFO order until the queue is empty.
        """
        while self.assigned_waypoints:
            wpt = self.assigned_waypoints.pop(0)
            self.simulate_navigation(wpt)
        self.is_navigating = False

    def simulate_navigation(self, waypoint):
        """
        Simulate the robot traveling from the current node to the waypoint.
        Publishes relevant status updates and handles the first waypoint behavior.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']

        # If current_node is None, teleport to the new label
        if self.current_node is None:
            self.current_node = label
            self.publish_status("reached", "", 0.0, label, [label, label])

            # If we haven't notified about the first waypoint yet, do it now
            if not self.first_wp_notification_sent:
                notif = {"robot_namespace": self.robot_namespace}
                msg_notif = String()
                msg_notif.data = json.dumps(notif)
                self.first_wp_reached_pub.publish(msg_notif)
                self.first_wp_notification_sent = True
            return
            
        # If we are already at the target label, just log a message
        if self.current_node == label:
            self.get_logger().info(f"[{self.robot_namespace}] Already at '{label}'.")
            return
        
        # Check if current_node or destination are valid nodes in the graph
        if (self.current_node not in self.navigation_graph.nodes) or (label not in self.navigation_graph.nodes):
            err_msg = f"Current node='{self.current_node}' or destination='{label}' not in the graph"
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])
            return

        # Calculate distance and travel time
        cx = self.navigation_graph.nodes[self.current_node]['x']
        cy = self.navigation_graph.nodes[self.current_node]['y']
        dist = math.hypot(x - cx, y - cy)
        speed = 1.31
        ttime = dist / speed
        
        # Publish 'traversing' status
        trav_edge = [self.current_node, label]
        self.publish_status("traversing", "", 0.0, label, trav_edge)
        self.get_logger().info(
            f"[{self.robot_namespace}] Navigating from '{self.current_node}' to '{label}' ~{ttime:.2f}s."
        )
        
        # Simulate the travel time with a sleep
        time.sleep(ttime)
        self.current_node = label
        
        # Publish 'reached' status
        self.get_logger().info(f"[{self.robot_namespace}] Reached '{label}' in {ttime:.2f}s.")
        self.publish_status("reached", "", ttime, label, trav_edge)
        
        # If it's truly the first "real" waypoint we reach, notify
        if not self.first_wp_notification_sent:
            notif = {"robot_namespace": self.robot_namespace}
            msg_notif = String()
            msg_notif.data = json.dumps(notif)
            self.first_wp_reached_pub.publish(msg_notif)
            self.first_wp_notification_sent = True

    def publish_status(self, status, error_message, time_taken, current_waypoint, trav):
        """
        Publish navigation status updates, such as 'reached', 'traversing', or 'error'.
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
        s.data = json.dumps(st)
        self.status_publisher.publish(s)
        self.get_logger().debug(f"[{self.robot_namespace}] Published status: {st}")

    # =========================================================================
    # Becoming MASTER
    # =========================================================================
    
    def become_master(self):
        """
        Transitions this node to MASTER mode. Sets up Master data structures,
        starts HeartbeatManager, loads the global graph, and publishes it, etc.
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] => MASTER MODE activated")
        
        # Start publishing Master heartbeats with the shared QoS profile
        self.heartbeat_manager = HeartbeatManager(self, self.qos_profile)
        self.heartbeat_manager.start_publishing()
        
        # Create a WaypointManager for Master tasks
        self.waypoint_manager = WaypointManager(self)
        
        # Clear any existing slave data
        self.slaves = {}
        self.occupied_edges.clear()
        self.edge_occupants.clear()
        
        # Subscribe to slave_registration and navigation_status as Master
        self.slave_registration_subscriber = self.create_subscription(
            String, '/slave_registration', self.slave_registration_callback, self.qos_profile
        )
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, self.qos_profile
        )
        
        # If we have a valid graph path, try to load it
        if self.graph_path:
            try:
                with open(self.graph_path, 'r') as f:
                    graph_data = json.load(f)
                # Convert JSON data into a usable graph structure
                self.full_graph = load_full_graph_from_data(graph_data)
                self.get_logger().info(f"[{self.robot_namespace}] (MASTER) Loaded navigation_graph from '{self.graph_path}'.")
                
                # Publish the navigation graph so that other slaves can receive it
                self.publish_navigation_graph()
                
                # Compute the global CPP (Chinese Postman Problem) route
                self.compute_global_cpp_route()
                self.get_logger().info(f"Global CPP route computed: {self.global_cpp_route}")
                
                # Assign waypoints to slaves based on the new route
                self.waypoint_manager.repartition_and_assign_waypoints()
            except FileNotFoundError:
                self.get_logger().error(f"[{self.robot_namespace}] Graph file not found at '{self.graph_path}'. Cannot become Master.")
                return
            except json.JSONDecodeError as e:
                self.get_logger().error(f"[{self.robot_namespace}] Error decoding graph JSON: {e}. Cannot become Master.")
                return
            except Exception as e:
                self.get_logger().error(f"[{self.robot_namespace}] Unexpected error loading graph: {e}. Cannot become Master.")
                return
        else:
            self.get_logger().warn(f"[{self.robot_namespace}] No graph path provided => cannot assign routes as MASTER.")
        
        # Create a timer to periodically run Master tasks (e.g. checking for timeouts)
        self.master_timer = self.create_timer(self.check_interval, self.master_timer_callback)
        
        # Stop publishing slave heartbeats once we've become Master
        self.heartbeat_timer.cancel()
        self.get_logger().info(f"[{self.robot_namespace}] Stopped publishing slave heartbeat.")
        
        # Remove itself from active_slaves, as it is now the Master
        with self.active_slaves_lock:
            if self.robot_namespace in self.active_slaves:
                self.active_slaves.remove(self.robot_namespace)
                self.get_logger().info(f"[{self.robot_namespace}] Removed itself from active_slaves.")
        
        self.get_logger().info(f"[{self.robot_namespace}] => Master setup complete, ready to listen to slaves.")

    def master_timer_callback(self):
        """
        Called periodically when this node is Master.
        Performs slave timeout checks and tries to assign waiting slaves.
        """
        self.check_slaves_timeout()
        self.waypoint_manager.assign_waiting_slaves()

    # =========================================================================
    # Overrides of MasterCallbacks (active only if is_master == True)
    # =========================================================================
    
    def slave_registration_callback(self, msg):
        """
        Callback for /slave_registration messages. Only processed if is_master == True.
        """
        if not self.is_master:
            return
        super().slave_registration_callback(msg)

    def navigation_status_callback(self, msg):
        """
        Callback for /navigation_status messages. Only processed if is_master == True.
        """
        if not self.is_master:
            return
        super().navigation_status_callback(msg)

    def send_coordinator_message(self):
        """
        Sends a coordinator message indicating this node is the Master.
        """
        coord_msg = {"type": "COORDINATOR", "sender": self.robot_namespace}
        msg = String()
        msg.data = json.dumps(coord_msg)
        self.coordinator_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Sent COORDINATOR message.")

    # =========================================================================
    # Master-Specific Utility
    # =========================================================================
        
    def check_slaves_timeout(self):
        """
        Checks if slaves are still active based on their last heartbeat timestamp.
        If a slave has not sent a heartbeat within 'self.timeout', it is removed.
        """
        current_time = time.time()
        with self.active_slaves_lock:
            # Identify inactive slaves
            inactive_slaves = [
                slave_ns for slave_ns, slave in self.slaves.items()
                if (current_time - slave.last_seen_time) > self.timeout
            ]
            for slave_ns in inactive_slaves:
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Slave '{slave_ns}' timed out and has been removed.")

    # =========================================================================
    # Execution
    # =========================================================================
    
    def run(self):
        """
        Spins the node so it can respond to callbacks and timers indefinitely.
        """
        rclpy.spin(self)

    def destroy_node(self):
        """
        Cleans up resources, stops heartbeat publishing if Master, and then destroys this node.
        """
        if self.is_master and self.heartbeat_manager:
            self.heartbeat_manager.stop_publishing()
        super().destroy_node()


def main(args=None):
    """
    Entry point for running this node directly.
    """
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser(description='Simulated Slave Node (can become Master) without an initial node.')
    parser.add_argument('--robot_namespace', type=str, default='robot1', help='Namespace of this slave')
    parser.add_argument('--graph_path', type=str, default='', help='Path to the navigation graph JSON file')
    parsed_args, ros_args = parser.parse_known_args()

    # Create and run the SimulatedSlaveNavigationNode
    node = SimulatedSlaveNavigationNode(robot_namespace=parsed_args.robot_namespace, graph_path=parsed_args.graph_path)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
