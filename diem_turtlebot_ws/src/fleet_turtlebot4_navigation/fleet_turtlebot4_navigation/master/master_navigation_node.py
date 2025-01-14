#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from .master_callbacks import MasterCallbacks
from .heartbeat_manager import HeartbeatManager
from .waypoint_manager import WaypointManager
from .graph_utils import load_full_graph
from .slave_state import SlaveState


class MasterNavigationNode(Node, MasterCallbacks):
    """
    MasterNavigationNode is a ROS2 node responsible for managing a fleet of slave robots.
    
    This node performs the following key functions:
      - Loads and publishes the navigation graph.
      - Registers and tracks slave robots.
      - Computes and assigns waypoints to slaves.
      - Monitors slave heartbeats to detect and handle inactive slaves.
      - Manages the occupancy of edges in the navigation graph to prevent overlapping tasks.
    
    The node inherits from both `Node` (a fundamental ROS2 class) and `MasterCallbacks` 
    (which provides callback methods for handling slave interactions and navigation tasks).
    """
    
    def __init__(self):
        """
        Initializes the MasterNavigationNode.
        """
        # Initialize the ROS2 node
        Node.__init__(self, 'master_navigation_node')
        
        # Initialize MasterCallbacks
        MasterCallbacks.__init__(self)

        # ---------------------------
        # Parameter Declaration and Retrieval
        # ---------------------------
        self.declare_parameter('graph_path', '/home/utente/percorso_al_file/result_graph.json')
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('timeout', 5.0)
        
        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # ---------------------------
        # Validate and Load Graph
        # ---------------------------
        if not os.path.exists(self.graph_path):
            self.get_logger().error(f"Graph file not found at {self.graph_path}")
            raise FileNotFoundError(f"Graph file not found at {self.graph_path}")

        self.full_graph = load_full_graph(self.graph_path)
        self.get_logger().info(f"Loaded undirected graph from {self.graph_path}.")

        # ---------------------------
        # QoS Configuration
        # ---------------------------
        # Create a custom QoSProfile for both publishers and subscribers
        # Adjust these policies as needed (BEST_EFFORT, VOLATILE, etc.)
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # ---------------------------
        # Publisher and Subscriber Setup
        # ---------------------------
        self.graph_publisher = self.create_publisher(
            String,
            '/navigation_graph',
            self.qos_profile
        )
        self.graph_timer = self.create_timer(1.0, self.publish_navigation_graph)

        self.slave_registration_subscriber = self.create_subscription(
            String,
            '/slave_registration',
            self.slave_registration_callback,
            self.qos_profile
        )

        self.navigation_status_subscriber = self.create_subscription(
            String,
            '/navigation_status',
            self.navigation_status_callback,
            self.qos_profile
        )

        # ---------------------------
        # Manager Initialization
        # ---------------------------
        # Pass the shared QoSProfile to HeartbeatManager
        self.heartbeat_manager = HeartbeatManager(self, self.qos_profile)
        self.heartbeat_manager.start_publishing()
        
        self.waypoint_manager = WaypointManager(self)

        # ---------------------------
        # Timer for Periodic Maintenance
        # ---------------------------
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # ---------------------------
        # Slave Tracking Structures
        # ---------------------------
        self.slaves = {}
        self.partitioning_done = False
        self.occupied_edges = set()
        self.edge_occupants = {}

        self.reset_occupied_edges()
        self.edge_last_visit_time = {}

        # For first waypoint reached logic
        self.first_wp_reached_subscriber = None
        self.slaves_that_reported_first_wp = set()
        self.waiting_for_first_waypoints = False

    def check_slaves_timeout(self):
        """
        Identifies and removes slaves that have exceeded the heartbeat timeout.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        slaves_to_remove = []

        for slave_ns, slave in self.slaves.items():
            if current_time - slave.last_seen_time > self.timeout:
                self.get_logger().warn(f"Slave {slave_ns} has timed out. Removing from active slaves.")
                slaves_to_remove.append(slave_ns)

        for slave_ns in slaves_to_remove:
            if slave_ns in self.slaves:
                s = self.slaves[slave_ns]
                if s.current_edge is not None and s.current_edge in self.occupied_edges:
                    self.occupied_edges.remove(s.current_edge)
                    occupant = self.edge_occupants.pop(s.current_edge, None)
                    self.get_logger().info(
                        f"Freed edge {s.current_edge} from occupant={occupant} due to slave timeout."
                    )
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removed slave {slave_ns} due to timeout.")

        if slaves_to_remove:
            self.get_logger().info("Repartitioning and reassigning waypoints after slave removal.")
            self.waypoint_manager.repartition_and_assign_waypoints()

    def timer_callback(self):
        """
        Periodic callback for maintenance tasks such as checking timeouts and assigning waiting waypoints.
        """
        self.get_logger().debug("Master timer callback triggered.")
        self.check_slaves_timeout()
        self.waypoint_manager.assign_waiting_slaves()

    def reset_occupied_edges(self):
        """
        Clears all tracking of occupied edges and their occupants.
        """
        self.occupied_edges.clear()
        self.edge_occupants.clear()

    def main_method(self):
        """
        A method you can call once your node is up, to start initial processes (optional).
        """
        self.waypoint_manager.repartition_and_assign_waypoints()


def main(args=None):
    rclpy.init(args=args)
    node = MasterNavigationNode()
    try:
        node.main_method()  # or do it inline if you prefer
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
