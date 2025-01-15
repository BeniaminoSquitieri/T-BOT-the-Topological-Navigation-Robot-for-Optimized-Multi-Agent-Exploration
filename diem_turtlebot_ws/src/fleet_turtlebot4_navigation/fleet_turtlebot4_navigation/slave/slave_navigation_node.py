#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from threading import Lock, Event
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For real TurtleBot4 navigation
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult

# Possibly needed for graph logic (if you want to handle the graph in MASTER mode):
import networkx as nx

# Master Tools (if you use the same structure from your simulation)
# E.g., MasterCallbacks, HeartbeatManager, WaypointManager, load_full_graph_from_data
# Adjust the import paths as needed.
from fleet_turtlebot4_navigation.master.master_callbacks import MasterCallbacks
from fleet_turtlebot4_navigation.master.heartbeat_manager import HeartbeatManager
from fleet_turtlebot4_navigation.master.waypoint_manager import WaypointManager
from fleet_turtlebot4_navigation.master.graph_utils import load_full_graph_from_data


class RealRobotNavigationNode(Node, MasterCallbacks):
    """
    A unified node that can act as a SLAVE or become MASTER (Bully algorithm),
    and uses the real TurtleBot4Navigator for navigation.

    Differences compared to the 'simulated' node:
      - The 'simulate_navigation' logic is replaced with real TurtleBot4 navigation
        via 'TurtleBot4Navigator'.
    """

    def __init__(self, robot_namespace='robot1', graph_path=''):
        """
        Initialize the node in SLAVE mode.

        Args:
            robot_namespace (str): Unique namespace for this robot (e.g., 'robot1').
            graph_path (str): Path to the JSON navigation graph (used when becoming MASTER).
        """
        # Name the node 'real_robot_navigation_node' and place it under the robot_namespace
        super().__init__('real_robot_navigation_node', namespace=robot_namespace)
        
        # Initialize MasterCallbacks if you want the master logic
        MasterCallbacks.__init__(self)

        # Store basic parameters
        self.robot_namespace = robot_namespace
        self.graph_path = graph_path
        
        # QoS profile (similar to the simulation code)
        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile.reliability = ReliabilityPolicy.RELIABLE
        self.qos_profile.durability = DurabilityPolicy.VOLATILE

        # Flags and primary state
        self.is_master = False
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 1000.0

        # Declare and read some parameters
        self.declare_parameter('timeout', 5.0)
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        self.declare_parameter('check_interval', 5.0)
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value

        # For concurrency
        self.active_slaves = set()
        self.active_slaves_lock = Lock()

        # Graph / navigation state
        self.graph_received = False
        self.navigation_graph = None
        self.current_node = None  # start with no known location
        self.is_navigating = False
        self.assigned_waypoints = []
        self.navigation_lock = Lock()

        # Master data structures
        self.slaves = {}
        self.occupied_edges = set()
        self.edge_occupants = {}
        self.partitioning_done = False
        self.heartbeat_manager = None
        self.waypoint_manager = None

        # Timers, events for Bully election
        self.election_in_progress = False
        self.election_event = Event()

        # Initialize the real TurtleBot4Navigator for actual motion
        self.navigator = TurtleBot4Navigator()

        # Minor flags
        self.first_wp_notification_sent = False

        ############################
        # Publishers and Subscribers
        ############################
        # SLAVE side: (common topics)
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', self.qos_profile)
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', self.qos_profile)
        self.status_publisher = self.create_publisher(String, '/navigation_status', self.qos_profile)

        self.election_publisher = self.create_publisher(String, '/election', self.qos_profile)
        self.coordinator_publisher = self.create_publisher(String, '/coordinator', self.qos_profile)

        # Publish first waypoint reached if needed
        self.first_wp_reached_pub = self.create_publisher(String, '/first_waypoint_reached', self.qos_profile)

        # SUBSCRIBERS
        # Master heartbeat (to detect Master presence/failure)
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, self.qos_profile
        )
        # Navigation graph broadcast from Master
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.slave_navigation_graph_callback, self.qos_profile
        )
        # Election messages
        self.election_subscriber = self.create_subscription(
            String, '/election', self.election_callback, self.qos_profile
        )
        self.coordinator_subscriber = self.create_subscription(
            String, '/coordinator', self.coordinator_callback, self.qos_profile
        )

        # Listen for navigation commands on "/<robot_namespace>/navigation_commands"
        self.navigation_commands_subscriber = self.create_subscription(
            String,
            f'/{self.robot_namespace}/navigation_commands',
            self.slave_navigation_commands_callback,
            self.qos_profile
        )

        # Timers
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        self.check_master_timer = self.create_timer(1.0, self.check_master_alive)

        # Debug log: started as slave with no known node
        self.get_logger().info(
            f"[{self.robot_namespace}] Started (real) as SLAVE with NO initial node."
        )

    ###########################################################################
    # SLAVE-PART: Publishing & Basic Callbacks
    ###########################################################################

    def publish_registration(self):
        """
        Publishes the robot namespace so the Master knows we exist.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published slave registration.")

    def publish_heartbeat(self):
        """
        Publishes a heartbeat to inform the Master we're alive.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published slave heartbeat.")

    def master_heartbeat_callback(self, msg):
        """
        Resets 'master_alive' each time a Master heartbeat is received.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received master heartbeat.")

    def check_master_alive(self):
        """
        Periodic check to see if the Master is still sending heartbeats.
        If not, we start an election.
        """
        if self.is_master:
            return

        if self.master_alive:
            # Next loop we consider master_alive = False to see if another heartbeat arrives
            self.master_alive = False
        else:
            # If no heartbeat arrived for 'heartbeat_timeout', start election
            now = time.time()
            if (now - self.last_master_heartbeat) > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost => Initiating election!")
                self.elect_new_master()

    ###########################################################################
    # BULLY ALGORITHM (ELECTION)
    ###########################################################################

    def elect_new_master(self):
        """
        Initiates the Bully election. 
        If no 'higher' nodes respond, this node becomes Master.
        """
        if self.election_in_progress:
            self.get_logger().info(f"[{self.robot_namespace}] Election already in progress.")
            return

        self.election_in_progress = True
        self.election_event.clear()

        with self.active_slaves_lock:
            higher_nodes = [ns for ns in self.active_slaves if ns > self.robot_namespace]

        if not higher_nodes:
            self.get_logger().info(f"[{self.robot_namespace}] No higher nodes found. Becoming MASTER!")
            self.become_master()
            self.election_in_progress = False
            self.election_event.set()
            return

        # Send ELECTION messages to higher nodes
        for node_ns in higher_nodes:
            election_msg = {"type": "ELECTION", "sender": self.robot_namespace}
            msg = String()
            msg.data = json.dumps(election_msg)
            self.election_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Sent ELECTION to {node_ns}.")

        wait_time = 5
        self.get_logger().info(f"[{self.robot_namespace}] Waiting {wait_time}s for OK responses.")
        self.election_event.wait(wait_time)

        if not self.election_in_progress:
            # Another node responded, so we are not master
            self.get_logger().info(f"[{self.robot_namespace}] Another node took over the election.")
            return

        # If no one responded, become Master
        self.get_logger().info(f"[{self.robot_namespace}] No OK received => Becoming MASTER!")
        self.become_master()
        self.election_in_progress = False
        self.election_event.set()

    def election_callback(self, msg):
        """
        Callback for receiving ELECTION messages from other nodes.
        If we have a 'higher' name, we respond with 'OK' and start our own election.
        """
        try:
            data = json.loads(msg.data)
            if data.get("type") != "ELECTION":
                return
            sender = data.get("sender")
            if sender and sender < self.robot_namespace:
                # We are 'higher', so we say OK and start our own election
                ok_msg = {"type": "OK", "sender": self.robot_namespace}
                response = String()
                response.data = json.dumps(ok_msg)
                self.election_publisher.publish(response)
                self.get_logger().info(
                    f"[{self.robot_namespace}] Received ELECTION from '{sender}', sent OK. Launching our election."
                )
                self.elect_new_master()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding ELECTION message: {e}")

    def coordinator_callback(self, msg):
        """
        Callback for receiving COORDINATOR messages from the new Master.
        """
        try:
            data = json.loads(msg.data)
            if data.get("type") != "COORDINATOR":
                return
            sender = data.get("sender")
            if sender:
                self.get_logger().info(f"[{self.robot_namespace}] COORDINATOR from '{sender}'. That is the new Master.")
                self.is_master = False
                self.master_alive = True
                self.last_master_heartbeat = time.time()
                self.election_in_progress = False
                self.election_event.set()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding COORDINATOR message: {e}")

    ###########################################################################
    # SLAVE: Receiving Graph
    ###########################################################################

    def slave_navigation_graph_callback(self, msg):
        """
        Receives the navigation graph from the Master. 
        Once the graph is loaded, publish a 'ready' status if we haven't already.
        """
        if self.is_master:
            # If we are Master, ignore updates from external sources
            return

        if self.graph_received:
            self.get_logger().debug(f"[{self.robot_namespace}] Graph already received, ignoring.")
            return

        try:
            data = json.loads(msg.data)
            self.navigation_graph = load_full_graph_from_data(data)  # Provided by your master.graph_utils
            self.graph_received = True

            # Publish 'ready' status
            ready_data = {
                'robot_namespace': self.robot_namespace,
                'status': 'ready',
                'error_message': '',
                'time_taken': 0.0,
                'current_waypoint': str(self.current_node),
                'traversed_edge': []
            }
            rmsg = String()
            rmsg.data = json.dumps(ready_data)
            self.status_publisher.publish(rmsg)
            self.get_logger().info(f"[{self.robot_namespace}] Graph received, published 'ready' status.")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding graph as slave: {e}")

    ###########################################################################
    # SLAVE: Receiving Waypoints
    ###########################################################################

    def slave_navigation_commands_callback(self, msg):
        """
        Callback for receiving waypoint commands for this slave, e.g. from the Master.
        The waypoint is expected to have the form: { "label": str, "x": float, "y": float }
        """
        if self.is_master:
            return

        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding waypoint: {e}")
            return

        # If we don't have a graph, we can't properly navigate
        if not self.graph_received or self.navigation_graph is None:
            self.get_logger().warn(f"[{self.robot_namespace}] No graph available => cannot navigate.")
            self.publish_status("error", "NoGraph", 0.0, waypoint_data.get('label', '?'), [])
            return

        with self.navigation_lock:
            self.assigned_waypoints.append(waypoint_data)
            self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")
            # If not already navigating, start
            if not self.is_navigating:
                self.is_navigating = True
                # In the real code, we navigate one waypoint at a time:
                wpt = self.assigned_waypoints.pop(0)
                self.navigate_to_waypoint(wpt)
                # Potentially continue if multiple waypoints
                # but we can do a simple approach: once we reach the first, 
                # we check if there's more, etc.

    ###########################################################################
    # REAL Navigation using TurtleBot4Navigator
    ###########################################################################
    
    def navigate_to_waypoint(self, waypoint):
        """
        Replaces 'simulate_navigation' with actual TurtleBot4 navigation calls,
        using the startToPose(...) and getResult() approach (synchronous-like).
        """
        label = waypoint["label"]
        x = waypoint["x"]
        y = waypoint["y"]

        # If we have never set a current_node, "teleport" to the first destination.
        if self.current_node is None:
            self.current_node = label
            self.publish_status("reached", "", 0.0, label, [label, label])
            if not self.first_wp_notification_sent:
                self.publish_first_waypoint_notification()
            self.is_navigating = False
            return

        # If we are already at the destination
        if self.current_node == label:
            self.get_logger().info(f"[{self.robot_namespace}] Already at '{label}'.")
            self.is_navigating = False
            return

        # If the node doesn't exist in the graph, report an error
        if (self.current_node not in self.navigation_graph.nodes) or (label not in self.navigation_graph.nodes):
            err_msg = f"Current node='{self.current_node}' or destination='{label}' not in the graph."
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])
            self.is_navigating = False
            return

        # Build the pose (x, y, orientation=0.0)
        goal_pose = self.navigator.getPoseStamped([x, y], 0.0)

        start_time = time.time()
        self.publish_status("traversing", "", 0.0, label, [self.current_node, label])
        self.get_logger().info(
            f"[{self.robot_namespace}] Navigating from '{self.current_node}' to '{label}'..."
        )

        try:
            # (Optional) Check if the action server is ready.
            if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                err_message = f"Action server not available to navigate to '{label}'."
                self.get_logger().error(f"[{self.robot_namespace}] {err_message}")
                self.publish_status("error", err_message, 0.0, label, [])
                self.is_navigating = False
                return

            # Start blocking navigation
            self.navigator.startToPose(goal_pose)

            # Once navigation completes, measure the total time
            time_taken = time.time() - start_time

            # Retrieve the final navigation result (TaskResult.SUCCEEDED, etc.)
            nav_result = self.navigator.getResult()

            if nav_result == TaskResult.SUCCEEDED:
                # Success
                self.get_logger().info(
                    f"[{self.robot_namespace}] Reached '{label}' in {time_taken:.2f}s."
                )
                self.current_node = label
                self.publish_status("reached", "", time_taken, label, [self.current_node, label])
                
                if not self.first_wp_notification_sent:
                    self.publish_first_waypoint_notification()
            else:
                # Some failure code (ABORTED, CANCELLED, etc.)
                error_message = f"Navigation to '{label}' failed with code {nav_result}."
                self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
                self.publish_status("error", error_message, time_taken, label, [self.current_node, label])

        except Exception as e:
            error_message = f"Exception while navigating to '{label}': {e}"
            self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
            self.publish_status("error", error_message, 0.0, label, [self.current_node, label])

        # Finished processing this waypoint
        self.is_navigating = False
        
        # # If there are more queued waypoints, continue
        # with self.navigation_lock:
        #     if len(self.assigned_waypoints) > 0:
        #         next_wp = self.assigned_waypoints.pop(0)
        #         self.is_navigating = True
        #         self.navigate_to_waypoint(next_wp)




    def publish_first_waypoint_notification(self):
        """
        Publishes a one-time notification that the first waypoint has been reached.
        """
        if not self.first_wp_notification_sent and self.current_node is not None:
            data = {"robot_namespace": self.robot_namespace}
            msg = String()
            msg.data = json.dumps(data)
            self.first_wp_reached_pub.publish(msg)
            self.first_wp_notification_sent = True
            self.get_logger().info(f"[{self.robot_namespace}] Published first_waypoint_reached notification.")

    def publish_status(self, status, error_message, time_taken, current_waypoint, trav=None):
        """
        Publish a status update to '/navigation_status'.
        
        Args:
            status (str): e.g. "reached", "error", "traversing"
            error_message (str): If there's an error, store the message here
            time_taken (float): Time taken to reach waypoint
            current_waypoint (str): Label of the waypoint
            trav (list): e.g. [start_node, end_node] to indicate edge traveled
        """
        if trav is None:
            trav = []

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
        self.get_logger().info(f"[{self.robot_namespace}] Published status: {st}")

    ###########################################################################
    # BECOMING MASTER
    ###########################################################################
    
    def become_master(self):
        """
        Switch from SLAVE to MASTER mode. 
        Sets up Master data structures, loads graph from disk, broadcasts it, etc.
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] => MASTER MODE activated")

        # Heartbeat manager (master side)
        self.heartbeat_manager = HeartbeatManager(self, self.qos_profile)
        self.heartbeat_manager.start_publishing()

        # Waypoint manager for coordinating edges
        self.waypoint_manager = WaypointManager(self)

        # Clear old data
        self.slaves = {}
        self.occupied_edges.clear()
        self.edge_occupants.clear()

        # Subscribe to /slave_registration and /navigation_status as MASTER
        self.slave_registration_subscriber = self.create_subscription(
            String, '/slave_registration', self.slave_registration_callback, self.qos_profile
        )
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, self.qos_profile
        )

        if self.graph_path:
            try:
                with open(self.graph_path, 'r') as f:
                    graph_data = json.load(f)
                self.full_graph = load_full_graph_from_data(graph_data)
                self.get_logger().info(f"[{self.robot_namespace}] (MASTER) Loaded graph from '{self.graph_path}'.")

                # Publish the navigation graph so slaves can receive it
                self.publish_navigation_graph()

                self.compute_global_cpp_route()
                self.get_logger().info(f"Global CPP route computed: {self.global_cpp_route}")

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
            self.get_logger().warn(f"[{self.robot_namespace}] No graph path => cannot assign routes as MASTER.")

        self.master_timer = self.create_timer(self.check_interval, self.master_timer_callback)

        # Stop publishing slave heartbeat
        self.heartbeat_timer.cancel()
        self.get_logger().info(f"[{self.robot_namespace}] Stopped publishing slave heartbeat.")

        with self.active_slaves_lock:
            if self.robot_namespace in self.active_slaves:
                self.active_slaves.remove(self.robot_namespace)
                self.get_logger().info(f"[{self.robot_namespace}] Removed itself from active_slaves.")

        self.get_logger().info(f"[{self.robot_namespace}] => Master setup complete, ready to listen to slaves.")

    def master_timer_callback(self):
        """
        Periodically called when we are Master, 
        checks slave timeouts and tries to assign waiting slaves.
        """
        self.check_slaves_timeout()
        self.waypoint_manager.assign_waiting_slaves()

    ###########################################################################
    # MasterCallbacks overrides (only relevant if is_master == True)
    ###########################################################################

    def slave_registration_callback(self, msg):
        # Only if Master
        if not self.is_master:
            return
        super().slave_registration_callback(msg)

    def navigation_status_callback(self, msg):
        # Only if Master
        if not self.is_master:
            return
        super().navigation_status_callback(msg)

    def send_coordinator_message(self):
        # A method required by MasterCallbacks to broadcast we are the coordinator
        coord_msg = {"type": "COORDINATOR", "sender": self.robot_namespace}
        msg = String()
        msg.data = json.dumps(coord_msg)
        self.coordinator_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Sent COORDINATOR message.")

    ###########################################################################
    # Additional Master-specific method for timeouts
    ###########################################################################
    
    def check_slaves_timeout(self):
        """
        Check if slaves are still alive based on heartbeat timestamps.
        Remove them if they time out.
        """
        current_time = time.time()
        with self.active_slaves_lock:
            inactive_slaves = [
                ns for ns, slave_state in self.slaves.items()
                if (current_time - slave_state.last_seen_time) > self.timeout
            ]
            for ns in inactive_slaves:
                del self.slaves[ns]
                self.get_logger().warn(f"Slave '{ns}' timed out and has been removed.")

    ###########################################################################
    # Node Execution
    ###########################################################################

    def run(self):
        """
        Spin the node so it can respond to callbacks indefinitely.
        """
        rclpy.spin(self)

    def destroy_node(self):
        """
        Cleans up resources; if we're Master, stop publishing heartbeats.
        """
        if self.is_master and self.heartbeat_manager:
            self.heartbeat_manager.stop_publishing()
        super().destroy_node()


def main(args=None):
    """
    Entry point for running this node from the command line.
    """
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser(description='Real Robot Node that can become MASTER if needed.')
    parser.add_argument('--robot_namespace', type=str, default='robot1', help='Robot namespace.')
    parser.add_argument('--graph_path', type=str, default='', help='Path to the navigation graph JSON file.')
    parsed_args, ros_args = parser.parse_known_args()

    node = RealRobotNavigationNode(
        robot_namespace=parsed_args.robot_namespace,
        graph_path=parsed_args.graph_path
    )

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
