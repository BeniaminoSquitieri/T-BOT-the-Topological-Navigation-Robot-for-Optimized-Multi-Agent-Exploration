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
     - It periodically publishes registration and heartbeat to a Master node.
     - It receives navigation commands in JSON format and uses TurtleBot4Navigator to reach the destination.
     - If it detects that the Master is not alive (missing heartbeat), it starts an election using 
       the Bully Algorithm. If it wins, it becomes the new Master and starts assigning waypoints 
       to other slaves, using the typical MasterCallbacks logic.
    """

    def __init__(self, robot_namespace='robot1', initial_node_label='node_4'):
        """
        Constructor for the RealSlaveNavigationNode.
        
        Args:
            robot_namespace (str, optional):
                Unique namespace for this robot node. Defaults to 'robot1'.
            initial_node_label (str, optional):
                Label of the initial node for the robot's navigation. Defaults to 'node_4'.
        """
        # Initialize the Node base class with a specific node name and namespace
        super().__init__('real_slave_navigation_node', namespace=robot_namespace)
        
        # Initialize MasterCallbacks to enable master functionalities 
        MasterCallbacks.__init__(self)
        
        # Basic attributes
        self.robot_namespace = robot_namespace
        self.initial_node_label = initial_node_label
        
        # --------------------------------------------------
        # SLAVE state initialization
        # --------------------------------------------------
        self.is_master = False
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 15.0  # e.g. 15 seconds for Master to be considered "dead"

        # Declare and retrieve 'timeout' parameter (used in MasterCallbacks for check_slaves_timeout)
        self.declare_parameter('timeout', 5.0)
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # For the Master, a check_interval parameter (used by master timers)
        self.declare_parameter('check_interval', 2.0)
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value

        # Thread-safe set of active slaves for Bully election
        self.active_slaves = set()
        self.active_slaves_lock = Lock()

        # --------------------------------------------------
        # NAVIGATION state initialization
        # --------------------------------------------------
        self.graph_received = False
        self.navigation_graph = None
        self.current_node = initial_node_label

        self.assigned_waypoints = []
        self.is_navigating = False
        self.navigation_lock = Lock()

        # --------------------------------------------------
        # MASTER state initialization (inactive at start)
        # --------------------------------------------------
        self.slaves = {}              # { slave_ns : SlaveState(...) } 
        self.occupied_edges = set()
        self.edge_occupants = {}
        self.partitioning_done = False
        self.heartbeat_manager = None
        self.waypoint_manager = None
        
        # --------------------------------------------------
        # Real navigation setup with TurtleBot4Navigator
        # --------------------------------------------------
        self.navigator = TurtleBot4Navigator()

        # --------------------------------------------------
        # Publishers / Subscribers typical from SLAVE side
        # --------------------------------------------------
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Bully algorithm messages
        self.election_publisher = self.create_publisher(String, '/election', 10)
        self.coordinator_publisher = self.create_publisher(String, '/coordinator', 10)

        # Subscribers
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

        # --------------------------------------------------
        # Timers typical for SLAVE
        # --------------------------------------------------
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        self.check_master_timer = self.create_timer(1.0, self.check_master_alive)

        # --------------------------------------------------
        # Election flags / events
        # --------------------------------------------------
        self.election_in_progress = False
        self.election_event = Event()

        # --------------------------------------------------
        # Random startup delay to reduce election conflicts
        # --------------------------------------------------
        startup_delay = random.uniform(0, 2)
        self.get_logger().info(
            f"[{self.robot_namespace}] Starting with a delay of {startup_delay:.2f}s to reduce election conflicts."
        )
        time.sleep(startup_delay)

        # Log node start
        self.get_logger().info(
            f"[{self.robot_namespace}] RealSlaveNavigationNode started as SLAVE (initial node='{self.initial_node_label}')."
        )

    # =========================================================================
    #                          SLAVE LOGIC
    # =========================================================================

    def publish_registration(self):
        """
        Publishes a registration message to indicate that this node is a SLAVE.
        Sent on /slave_registration for the Master to detect new slaves.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)

    def publish_heartbeat(self):
        """
        Publishes a SLAVE heartbeat message on /slave_heartbeat.
        """
        hb = String()
        hb.data = self.robot_namespace
        self.heartbeat_publisher.publish(hb)

    def master_heartbeat_callback(self, msg):
        """
        Receives heartbeat from the Master; we mark master_alive as True.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()

    def check_master_alive(self):
        """
        Checks if the Master is still alive. If not => we do election.
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

    # =========================================================================
    #                           BULLY ELECTION
    # =========================================================================

    def elect_new_master(self):
        """
        Bully Algorithm election:
         - Mark election_in_progress
         - Identify nodes with a bigger lexicographic namespace
         - Send ELECTION messages
         - If no OK => become Master
         - If OK => wait for COORDINATOR
        """
        if self.election_in_progress:
            self.get_logger().info(
                f"[{self.robot_namespace}] Election already in progress."
            )
            return

        self.election_in_progress = True
        self.election_event.clear()

        # gather slaves with bigger ID
        with self.active_slaves_lock:
            bigger_slaves = [s for s in self.active_slaves if s > self.robot_namespace]

        if not bigger_slaves:
            self.get_logger().info(
                f"[{self.robot_namespace}] No bigger slaves found => becoming MASTER!"
            )
            self.become_master()
            self.election_in_progress = False
            self.election_event.set()
            return

        # send ELECTION to bigger
        for node_ns in bigger_slaves:
            elect_msg = {"type": "ELECTION", "sender": self.robot_namespace}
            msg_s = String()
            msg_s.data = json.dumps(elect_msg)
            self.election_publisher.publish(msg_s)
            self.get_logger().debug(
                f"[{self.robot_namespace}] Sent ELECTION to '{node_ns}'"
            )

        # wait for OK
        wait_s = 5.0
        self.get_logger().info(
            f"[{self.robot_namespace}] Waiting for OK responses for {wait_s} seconds."
        )
        self.election_event.wait(wait_s)

        if not self.election_in_progress:
            self.get_logger().info(
                f"[{self.robot_namespace}] Another node is handling election."
            )
            return

        self.get_logger().info(
            f"[{self.robot_namespace}] No OK => I'm Master!"
        )
        self.become_master()
        self.election_in_progress = False
        self.election_event.set()

    def election_callback(self, msg):
        """
        Receives ELECTION message from other nodes.
        If we have a smaller ID => we reply with OK and do our own election.
        """
        try:
            data = json.loads(msg.data)
            if data.get("type") != "ELECTION":
                return
            sender = data.get("sender")
            if sender and sender < self.robot_namespace:
                # send OK
                ok_msg = {"type": "OK", "sender": self.robot_namespace}
                s2 = String()
                s2.data = json.dumps(ok_msg)
                self.election_publisher.publish(s2)
                self.get_logger().info(
                    f"[{self.robot_namespace}] Received ELECTION from '{sender}', sent OK. => start election"
                )
                self.elect_new_master()
        except Exception as e:
            self.get_logger().error(
                f"[{self.robot_namespace}] error in election_callback: {e}"
            )

    def coordinator_callback(self, msg):
        """
        Receives COORDINATOR from the new Master => we accept that Master and end our election.
        """
        try:
            data = json.loads(msg.data)
            if data.get("type") != "COORDINATOR":
                return
            sender = data.get("sender")
            if sender:
                self.get_logger().info(
                    f"[{self.robot_namespace}] Received COORDINATOR from '{sender}'. Accepting new Master."
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

    # =========================================================================
    #                  SLAVE ~ Navigation Commands
    # =========================================================================

    def slave_navigation_graph_callback(self, msg):
        """
        If we are slave, we can store the graph. If we already have it => ignore.
        """
        if self.is_master:
            return
        if self.graph_received:
            self.get_logger().debug(
                f"[{self.robot_namespace}] Graph message ignored (already have it)."
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
            # publish 'ready'
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
                f"[{self.robot_namespace}] Published 'ready' status (SLAVE)."
            )
        except Exception as e:
            self.get_logger().error(
                f"[{self.robot_namespace}] Error decoding graph: {e}"
            )

    def slave_navigation_commands_callback(self, msg):
        """
        As SLAVE, we receive navigation commands as JSON:
        {
          "label": <str>,
          "x": <float>,
          "y": <float>
        }
        We use TurtleBot4Navigator to navigate there (like in the real code).
        """
        if self.is_master:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f"[{self.robot_namespace}] Error decoding waypoint: {e}"
            )
            return

        if not self.graph_received or (self.navigation_graph is None):
            self.get_logger().warn(
                f"[{self.robot_namespace}] No graph => cannot navigate as SLAVE."
            )
            self.publish_status("error", "NoGraph", 0.0, data.get('label','?'), [])
            return

        with self.navigation_lock:
            self.assigned_waypoints.append(data)
            self.get_logger().info(
                f"[{self.robot_namespace}] Received waypoint: {data}"
            )
            if not self.is_navigating:
                self.is_navigating = True
                self.execute_waypoints_in_sequence()

    def execute_waypoints_in_sequence(self):
        """
        Process assigned waypoints in a synchronous manner, using real navigation (TurtleBot4Navigator).
        """
        while self.assigned_waypoints:
            wpt = self.assigned_waypoints.pop(0)
            self.real_navigate_to_waypoint(wpt)

        self.is_navigating = False

    def real_navigate_to_waypoint(self, waypoint):
        """
        Uses TurtleBot4Navigator to go to (x,y) ignoring any starting node info.
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
            f"[{self.robot_namespace}] [REALE] Navigating to waypoint '{label}' => ({x}, {y})."
        )

        # Build a Pose from the coords
        goal_pose = self.navigator.getPoseStamped([x,y])

        # Wait for server
        if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            err_msg = f"Action server not available for waypoint '{label}'."
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])
            return

        # Start navigation 
        nav_future = self.navigator.startToPose(goal_pose)
        self.get_logger().debug(
            f"[{self.robot_namespace}] Navigation started => {label}, waiting for completion..."
        )

        # Wait
        rclpy.spin_until_future_complete(self, nav_future)
        nav_result = nav_future.result()

        if nav_result is None:
            err_msg = f"No result from navigation => '{label}'."
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])
            return

        if nav_result == TaskResult.SUCCEEDED:
            self.get_logger().info(
                f"[{self.robot_namespace}] Waypoint '{label}' reached successfully!"
            )
            self.current_node = label
            self.publish_status("reached", "", 0.0, label, [])
        else:
            err_msg = f"Navigation failed => {label}, code={nav_result}"
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])

    def publish_status(self, status, error_message, time_taken, current_waypoint, trav=[]):
        """
        Publishes status (like 'traversing', 'reached', or 'error') to /navigation_status.
        """
        st = {
            'robot_namespace': self.robot_namespace,
            'status': status,
            'error_message': error_message,
            'time_taken': time_taken,
            'current_waypoint': current_waypoint,
            'traversed_edge': trav
        }
        msg = String()
        msg.data = json.dumps(st)
        self.status_publisher.publish(msg)

    # =========================================================================
    #                          BECOME MASTER
    # =========================================================================

    def become_master(self):
        """
        Switch from SLAVE to MASTER mode:
         1) Start HeartbeatManager (publishes /master_heartbeat).
         2) Start WaypointManager (assigns routes).
         3) Reset Master structures (self.slaves, etc.).
         4) Subscribe to /slave_registration, /navigation_status.
         5) If we have a navigation_graph => set as full_graph, publish it, compute route, assign waypoints...
         6) Create master_timer => check_slaves_timeout + assign_waiting_slaves
         7) Stop publishing SLAVE heartbeat, remove self from active_slaves
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] => MASTER MODE activated")

        self.heartbeat_manager = HeartbeatManager(self)
        self.heartbeat_manager.start_publishing()

        self.waypoint_manager = WaypointManager(self)

        self.slaves = {}
        self.occupied_edges.clear()
        self.edge_occupants.clear()

        # Subscribe to slave-specific topics as Master
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

            # repartition waypoints
            self.waypoint_manager.repartition_and_assign_waypoints()
        else:
            self.get_logger().warn(
                f"[{self.robot_namespace}] No navigation graph => cannot assign routes as Master."
            )

        # Create master timer
        self.master_timer = self.create_timer(self.check_interval, self.master_timer_callback)

        # stop slave heartbeat 
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
            f"[{self.robot_namespace}] => Master setup complete, ready to receive slaves."
        )

    def master_timer_callback(self):
        """
        Master timer => check_slaves_timeout + assign_waiting_slaves
        """
        self.check_slaves_timeout()
        self.waypoint_manager.assign_waiting_slaves()

    # =========================================================================
    # MASTER callback override if is_master
    # =========================================================================

    def slave_registration_callback(self, msg):
        """
        Receives registration from new slaves => only if is_master we handle
        """
        if not self.is_master:
            return
        super().slave_registration_callback(msg)

    def navigation_status_callback(self, msg):
        """
        Receives status from slaves => only handle if is_master
        """
        if not self.is_master:
            return
        super().navigation_status_callback(msg)

    # =========================================================================
    # COORDINATOR message sending if needed
    # =========================================================================

    def send_coordinator_message(self):
        """
        Send a COORDINATOR message => I am new master
        """
        cmsg = {"type": "COORDINATOR", "sender": self.robot_namespace}
        s = String()
        s.data = json.dumps(cmsg)
        self.coordinator_publisher.publish(s)
        self.get_logger().info(
            f"[{self.robot_namespace}] Sent COORDINATOR message."
        )

    # =========================================================================
    # RUN / DESTROY
    # =========================================================================

    def run(self):
        """
        Spin the ROS2 node
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
    Entry point: parse CLI arguments, init node, spin
    """
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser(
        description='Real-based Slave that can become Master if needed (Bully).'
    )
    parser.add_argument('--robot_namespace', type=str, default='robot1',
                        help='Namespace for this node')
    parser.add_argument('--initial_node_label', type=str, default='node_4',
                        help='Initial node label for simulation')
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
