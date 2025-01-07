#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import networkx as nx
from threading import Lock, Event
import random

# Master Tools (callbacks, heartbeat, waypoints, graph utils)
from fleet_turtlebot4_navigation.master.master_callbacks import MasterCallbacks
from fleet_turtlebot4_navigation.master.heartbeat_manager import HeartbeatManager
from fleet_turtlebot4_navigation.master.waypoint_manager import WaypointManager
from fleet_turtlebot4_navigation.master.graph_utils import load_full_graph_from_data
# from fleet_turtlebot4_navigation.master.slave_state import SlaveState  # optional

# Real Navigation for TurtleBot4
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult


class RealSlaveNavigationNode(Node, MasterCallbacks):
    """
    Real Slave Navigation Node that can elect itself as Master in case of Master failure.
    
    This node starts as a SLAVE:
     - Periodically publishes registration and heartbeat to a Master node.
     - Receives navigation commands in JSON format and uses TurtleBot4Navigator to navigate.
     - If it detects that the Master is not alive (missing heartbeat), it initiates the
       Bully Algorithm. If it wins, it becomes MASTER and uses typical MasterCallbacks logic 
       to assign waypoints to other slaves.
    """

    def __init__(self, robot_namespace='robot1', initial_node_label='node_14'):
        """
        Constructor for the RealSlaveNavigationNode.
        
        Args:
            robot_namespace (str, optional): Namespace for this robot node. Defaults to 'robot1'.
            initial_node_label (str, optional): Label of the initial node for navigation. Defaults to 'node_14'.
        """
        # Initialize the Node with the given name and namespace
        super().__init__('real_slave_navigation_node', namespace=robot_namespace)
        
        # Initialize MasterCallbacks for master functionality if/when elected
        MasterCallbacks.__init__(self)
        
        # Basic attributes
        self.robot_namespace = robot_namespace
        self.initial_node_label = initial_node_label
        
        # --------------------------------------------------------------------
        # SLAVE State Initialization
        # --------------------------------------------------------------------
        self.is_master = False
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 15.0  # seconds before declaring Master "dead"

        # Configurable timeouts from parameters
        self.declare_parameter('timeout', 5.0)  # used in MasterCallbacks for check_slaves_timeout
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        self.declare_parameter('check_interval', 2.0)
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value

        # Thread-safe set for Bully election
        self.active_slaves = set()
        self.active_slaves_lock = Lock()

        # --------------------------------------------------------------------
        # NAVIGATION State Initialization
        # --------------------------------------------------------------------
        self.graph_received = False
        self.navigation_graph = None
        self.current_node = initial_node_label

        self.assigned_waypoints = []
        self.is_navigating = False
        self.navigation_lock = Lock()

        # --------------------------------------------------------------------
        # MASTER State Initialization (inactive at first)
        # --------------------------------------------------------------------
        self.slaves = {}
        self.occupied_edges = set()
        self.edge_occupants = {}
        self.partitioning_done = False
        self.heartbeat_manager = None
        self.waypoint_manager = None
        
        # --------------------------------------------------------------------
        # Setup Real Navigation with TurtleBot4Navigator
        # --------------------------------------------------------------------
        self.navigator = TurtleBot4Navigator()

        # --------------------------------------------------------------------
        # PUBLISHERS / SUBSCRIBERS - SLAVE side
        # --------------------------------------------------------------------
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Bully Algorithm messages
        self.election_publisher = self.create_publisher(String, '/election', 10)
        self.coordinator_publisher = self.create_publisher(String, '/coordinator', 10)

        # Subscribers for commands, heartbeats, graph, etc.
        self.navigation_commands_subscriber = self.create_subscription(
            String, '/navigation_commands', self.slave_navigation_commands_callback, 10
        )
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 10
        )
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.slave_navigation_graph_callback, 10
        )
        self.election_subscriber = self.create_subscription(
            String, '/election', self.election_callback, 10
        )
        self.coordinator_subscriber = self.create_subscription(
            String, '/coordinator', self.coordinator_callback, 10
        )

        # --------------------------------------------------------------------
        # Timers typical for SLAVE
        # --------------------------------------------------------------------
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        self.check_master_timer = self.create_timer(1.0, self.check_master_alive)

        # --------------------------------------------------------------------
        # Election flags / events
        # --------------------------------------------------------------------
        self.election_in_progress = False
        self.election_event = Event()

        # --------------------------------------------------------------------
        # Random Startup Delay to reduce election conflicts
        # (Manual one-shot approach: create a repeating timer, then self-destruct it.)
        # --------------------------------------------------------------------
        self.startup_delay = random.uniform(0, 2)
        self.get_logger().info(
            f"[{self.robot_namespace}] Starting with a delay of {self.startup_delay:.2f}s to reduce election conflicts."
        )

        # Create a standard repeating timer with the chosen delay;
        # We'll destroy it inside its own callback, achieving a one-shot effect.
        self.startup_timer = self.create_timer(self.startup_delay, self.startup_delay_callback)

        # Log node start
        # (Note: any other initialization can happen after the delay if desired.)
        self.get_logger().info(
            f"[{self.robot_namespace}] RealSlaveNavigationNode constructed as SLAVE (initial node='{self.initial_node_label}')."
        )

    # ------------------------------------------------------------------------
    # ONE-SHOT TIMER LOGIC
    # ------------------------------------------------------------------------
    def startup_delay_callback(self):
        """
        This callback fires once after 'startup_delay' seconds. We immediately
        destroy the timer so it doesn't repeat, making it effectively one-shot.
        """
        self.get_logger().info(
            f"[{self.robot_namespace}] Startup delay elapsed. Node is now fully active."
        )
        # If any special post-delay logic is needed, do it here:
        # e.g., checking initial conditions or publishing a special message.

        # Destroy this timer so it never triggers again.
        self.destroy_timer(self.startup_timer)

    # ------------------------------------------------------------------------
    # SLAVE LOGIC
    # ------------------------------------------------------------------------
    def publish_registration(self):
        """
        Publishes a registration message to /slave_registration, letting the Master track this SLAVE.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)

    def publish_heartbeat(self):
        """
        Publishes a heartbeat message indicating this SLAVE is alive.
        """
        hb = String()
        hb.data = self.robot_namespace
        self.heartbeat_publisher.publish(hb)

    def master_heartbeat_callback(self, msg):
        """
        Received heartbeat from Master => mark master_alive = True.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()

    def check_master_alive(self):
        """
        Called periodically. If no heartbeat is received for 'heartbeat_timeout' => elect new master.
        """
        if self.is_master:
            return
        
        if self.master_alive:
            self.master_alive = False
        else:
            now = time.time()
            if now - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(
                    f"[{self.robot_namespace}] Master heartbeat lost => Initiating Election!"
                )
                self.elect_new_master()

    # ------------------------------------------------------------------------
    # BULLY ELECTION
    # ------------------------------------------------------------------------
    def elect_new_master(self):
        """
        Bully Algorithm steps:
         - Mark election_in_progress
         - Identify slaves with bigger lexicographic namespace
         - Send ELECTION to them
         - If no OK => become Master
         - If OK => wait for COORDINATOR
        """
        if self.election_in_progress:
            self.get_logger().info(
                f"[{self.robot_namespace}] Election already in progress..."
            )
            return

        self.election_in_progress = True
        self.election_event.clear()

        with self.active_slaves_lock:
            bigger_slaves = [s for s in self.active_slaves if s > self.robot_namespace]

        if not bigger_slaves:
            self.get_logger().info(
                f"[{self.robot_namespace}] No bigger slaves => Becoming MASTER!"
            )
            self.become_master()
            self.election_in_progress = False
            self.election_event.set()
            return

        # Send ELECTION messages
        for node_ns in bigger_slaves:
            elect_msg = {"type": "ELECTION", "sender": self.robot_namespace}
            msg_s = String()
            msg_s.data = json.dumps(elect_msg)
            self.election_publisher.publish(msg_s)
            self.get_logger().debug(
                f"[{self.robot_namespace}] Sent ELECTION to '{node_ns}'"
            )

        # Wait for OK
        wait_s = 5.0
        self.get_logger().info(
            f"[{self.robot_namespace}] Waiting {wait_s}s for OK responses."
        )
        self.election_event.wait(wait_s)

        if not self.election_in_progress:
            self.get_logger().info(
                f"[{self.robot_namespace}] Another node took over the election."
            )
            return

        # No OK => become Master
        self.get_logger().info(
            f"[{self.robot_namespace}] No OK => I'm Master!"
        )
        self.become_master()
        self.election_in_progress = False
        self.election_event.set()

    def election_callback(self, msg):
        """
        Received ELECTION from another node.
        If we have smaller ID => we send OK. Then we do our own elect_new_master.
        """
        try:
            data = json.loads(msg.data)
            if data.get("type") != "ELECTION":
                return
            sender = data.get("sender")
            if sender and sender < self.robot_namespace:
                # respond with OK
                ok_msg = {"type": "OK", "sender": self.robot_namespace}
                s2 = String()
                s2.data = json.dumps(ok_msg)
                self.election_publisher.publish(s2)
                self.get_logger().info(
                    f"[{self.robot_namespace}] Got ELECTION from '{sender}'. Sent OK => start my own election."
                )
                self.elect_new_master()
        except Exception as e:
            self.get_logger().error(
                f"[{self.robot_namespace}] error in election_callback: {e}"
            )

    def coordinator_callback(self, msg):
        """
        Another node became Master => accept it, end election.
        """
        try:
            data = json.loads(msg.data)
            if data.get("type") != "COORDINATOR":
                return
            sender = data.get("sender")
            if sender:
                self.get_logger().info(
                    f"[{self.robot_namespace}] COORDINATOR from '{sender}' => they are new Master."
                )
                self.is_master = False
                self.master_alive = True
                self.last_master_heartbeat = time.time()
                self.election_in_progress = False
                self.election_event.set()
        except Exception as e:
            self.get_logger().error(
                f"[{self.robot_namespace}] error in coordinator_callback: {e}"
            )

    # ------------------------------------------------------------------------
    # SLAVE ~ NAVIGATION
    # ------------------------------------------------------------------------
    def slave_navigation_graph_callback(self, msg):
        """
        SLAVE receives graph from Master => store it if not already have.
        Publish 'ready' once processed.
        """
        if self.is_master:
            return
        
        if self.graph_received:
            self.get_logger().debug(
                f"[{self.robot_namespace}] Graph ignored (already have it)."
            )
            return
        
        try:
            data = json.loads(msg.data)
            self.navigation_graph = load_full_graph_from_data(data)
            self.graph_received = True

            if self.initial_node_label in self.navigation_graph.nodes:
                self.current_node = self.initial_node_label
                self.get_logger().info(
                    f"[{self.robot_namespace}] Received graph => initial node='{self.initial_node_label}'"
                )
            else:
                self.get_logger().error(
                    f"[{self.robot_namespace}] initial_node '{self.initial_node_label}' not in graph!"
                )
                return

            # Publish 'ready'
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
            self.get_logger().info(
                f"[{self.robot_namespace}] Published 'ready' (SLAVE)."
            )
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f"[{self.robot_namespace}] Error decoding graph as slave: {e}"
            )

    def slave_navigation_commands_callback(self, msg):
        """
        SLAVE receives navigation commands => queue them and navigate in sequence.
        """
        if self.is_master:
            return
        
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f"[{self.robot_namespace}] Error decoding waypoint: {e}"
            )
            return

        if not self.graph_received or (self.navigation_graph is None):
            self.get_logger().warn(
                f"[{self.robot_namespace}] No graph => cannot navigate as SLAVE."
            )
            self.publish_status("error", "NoGraph", 0.0, waypoint_data.get('label','?'), [])
            return

        with self.navigation_lock:
            self.assigned_waypoints.append(waypoint_data)
            self.get_logger().info(
                f"[{self.robot_namespace}] Received waypoint: {waypoint_data}"
            )
            if not self.is_navigating:
                self.is_navigating = True
                self.execute_waypoints_in_sequence()

    def execute_waypoints_in_sequence(self):
        """
        Synchronous processing of assigned_waypoints using real navigation.
        """
        while self.assigned_waypoints:
            wpt = self.assigned_waypoints.pop(0)
            self.real_navigate_to_waypoint(wpt)

        self.is_navigating = False

    def real_navigate_to_waypoint(self, waypoint):
        """
        Use TurtleBot4Navigator to navigate to a single waypoint.
        """
        label = waypoint.get('label', '???')
        x = waypoint.get('x')
        y = waypoint.get('y')

        if x is None or y is None:
            err = f"Waypoint '{label}' missing x or y."
            self.get_logger().error(f"[{self.robot_namespace}] {err}")
            self.publish_status("error", err, 0.0, label, [])
            return

        self.get_logger().info(
            f"[{self.robot_namespace}] [REALE] Navigating to '{label}' => ({x}, {y})."
        )

        # Build Pose
        goal_pose = self.navigator.getPoseStamped([x,y])

        # Wait for server
        if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            err_msg = f"Action server not available for '{label}'."
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])
            return

        # Start nav
        nav_future = self.navigator.startToPose(goal_pose)
        self.get_logger().debug(
            f"[{self.robot_namespace}] Navigation started => {label}..."
        )

        rclpy.spin_until_future_complete(self, nav_future)
        nav_result = nav_future.result()

        if nav_result is None:
            err_msg = f"No result => '{label}'."
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])
            return

        if nav_result == TaskResult.SUCCEEDED:
            self.get_logger().info(
                f"[{self.robot_namespace}] Reached '{label}' successfully!"
            )
            self.current_node = label
            self.publish_status("reached", "", 0.0, label, [])
        else:
            err_msg = f"Navigation failed => {label}, code={nav_result}"
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])

    def publish_status(self, status, error_message, time_taken, current_waypoint, trav=[]):
        """
        Publishes navigation status to /navigation_status.
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

    # ------------------------------------------------------------------------
    # BECOME MASTER
    # ------------------------------------------------------------------------
    def become_master(self):
        """
        Switch from SLAVE to MASTER role:
         1) Start HeartbeatManager => /master_heartbeat
         2) Start WaypointManager => route assignments
         3) Reset Master structures
         4) Subscribe to slave_registration, navigation_status
         5) If have navigation_graph => set as full_graph, publish, compute route, assign
         6) Master timer => check_slaves_timeout, assign_waiting_slaves
         7) Stop SLAVE heartbeat, remove self from active_slaves
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] => MASTER MODE activated")

        self.heartbeat_manager = HeartbeatManager(self)
        self.heartbeat_manager.start_publishing()

        self.waypoint_manager = WaypointManager(self)

        # Reset structures
        self.slaves = {}
        self.occupied_edges.clear()
        self.edge_occupants.clear()

        # Subscribe to SLAVE topics as Master
        self.slave_registration_subscriber = self.create_subscription(
            String, '/slave_registration', self.slave_registration_callback, 10
        )
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )

        if self.navigation_graph is not None:
            self.full_graph = self.navigation_graph
            self.get_logger().info(
                f"[{self.robot_namespace}] (MASTER) using navigation_graph as full_graph"
            )

            self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)
            self.graph_timer = self.create_timer(1.0, self.publish_navigation_graph)

            # compute route
            self.compute_global_cpp_route()
            self.get_logger().info(f"Global CPP route computed: {self.global_cpp_route}")

            # repartition
            self.waypoint_manager.repartition_and_assign_waypoints()
        else:
            self.get_logger().warn(
                f"[{self.robot_namespace}] No navigation graph => cannot assign routes as Master."
            )

        # Master timer
        self.master_timer = self.create_timer(self.check_interval, self.master_timer_callback)

        # stop SLAVE heartbeat
        self.heartbeat_timer.cancel()
        self.get_logger().info(
            f"[{self.robot_namespace}] Stopped publishing SLAVE heartbeat."
        )

        # remove self from active_slaves
        with self.active_slaves_lock:
            if self.robot_namespace in self.active_slaves:
                self.active_slaves.remove(self.robot_namespace)
                self.get_logger().info(
                    f"[{self.robot_namespace}] Removed itself from active_slaves."
                )

        self.get_logger().info(
            f"[{self.robot_namespace}] => Master setup complete, ready to listen to slaves."
        )

    def master_timer_callback(self):
        """
        Master Timer => check_slaves_timeout + assign_waiting_slaves
        """
        self.check_slaves_timeout()
        self.waypoint_manager.assign_waiting_slaves()

    # ------------------------------------------------------------------------
    # MASTER callback override if is_master
    # ------------------------------------------------------------------------
    def slave_registration_callback(self, msg):
        """
        Only handle if is_master => otherwise ignore
        """
        if not self.is_master:
            return
        super().slave_registration_callback(msg)

    def navigation_status_callback(self, msg):
        """
        Only handle if is_master => otherwise ignore
        """
        if not self.is_master:
            return
        super().navigation_status_callback(msg)

    # ------------------------------------------------------------------------
    # COORDINATOR if needed
    # ------------------------------------------------------------------------
    def send_coordinator_message(self):
        """
        Inform everyone I am the new MASTER
        """
        cmsg = {"type": "COORDINATOR", "sender": self.robot_namespace}
        s = String()
        s.data = json.dumps(cmsg)
        self.coordinator_publisher.publish(s)
        self.get_logger().info(
            f"[{self.robot_namespace}] Sent COORDINATOR message."
        )

    # ------------------------------------------------------------------------
    # RUN / DESTROY
    # ------------------------------------------------------------------------
    def run(self):
        """
        Spin the node
        """
        rclpy.spin(self)

    def destroy_node(self):
        """
        Graceful node shutdown
        """
        if self.is_master and self.heartbeat_manager:
            self.heartbeat_manager.stop_publishing()
        super().destroy_node()


def main(args=None):
    """
    Entry point: parse CLI, init node, spin
    """
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser(
        description='Real-based Slave Node that can become Master if needed (Bully).'
    )
    parser.add_argument('--robot_namespace', type=str, default='robot1',
                        help='Namespace for this node')
    parser.add_argument('--initial_node_label', type=str, default='node_14',
                        help='Initial node label for navigation.')
    parsed_args, _ = parser.parse_known_args()

    node = RealSlaveNavigationNode(
        robot_namespace=parsed_args.robot_namespace,
        initial_node_label=parsed_args.initial_node_label
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
