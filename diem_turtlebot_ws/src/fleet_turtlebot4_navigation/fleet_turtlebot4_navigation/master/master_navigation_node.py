#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

# Import custom modules
from fleet_turtlebot4_navigation.master.graph_utils import load_full_graph
from fleet_turtlebot4_navigation.master.master_callbacks import MasterCallbacks
from fleet_turtlebot4_navigation.master.heartbeat_manager import HeartbeatManager
from fleet_turtlebot4_navigation.master.waypoint_manager import WaypointManager

class MasterNavigationNode(Node, MasterCallbacks):
    def __init__(self):
        super().__init__('master_navigation_node')

        # Dichiarazione e recupero dei parametri
        self.declare_parameter(
            'graph_path',
            '/home/beniamino/turtlebot4/diem_turtlebot_ws/src/fleet_turtlebot4_navigation/map/navigation_hardware_limitation.json'
        )
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('timeout', 150.0)

        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Validazione dell'esistenza del file del grafo di navigazione
        if not os.path.exists(self.graph_path):
            self.get_logger().error(f"Graph file not found at {self.graph_path}")
            raise FileNotFoundError(f"Graph file not found at {self.graph_path}")

        # Caricamento del grafo di navigazione
        self.full_graph = load_full_graph(self.graph_path)

        # Publishers e subscribers
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)
        self.graph_timer = self.create_timer(5.0, self.publish_navigation_graph)

        self.slave_registration_subscriber = self.create_subscription(
            String, '/slave_registration', self.slave_registration_callback, 10
        )
        self.initial_position_subscriber = self.create_subscription(
            String, '/slave_initial_positions', self.initial_position_callback, 10
        )
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )

        # Inizializzazione di HeartbeatManager
        self.heartbeat_manager = HeartbeatManager(self)

        # Inizializzazione di WaypointManager
        self.waypoint_manager = WaypointManager(self)

        # Timer per controlli periodici e gestione delle task in attesa
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Tracciamento dello stato
        self.slaves = {}
        self.partitioning_done = False
        self.occupied_edges = set()

        self.get_logger().info("Master node initialized with edge-based occupation control.")

    def check_slaves_timeout(self):
        """
        Identifica e rimuove gli slave che hanno superato il periodo di timeout.
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

                if s.current_waypoint_index < len(s.assigned_waypoints):
                    waypoint = s.assigned_waypoints[s.current_waypoint_index % len(s.assigned_waypoints)]
                    from_node = s.current_node
                    to_node = waypoint['label']
                    edge = (from_node, to_node)

                    if edge in self.occupied_edges:
                        self.occupied_edges.remove(edge)
                        self.get_logger().info(f"Edge {edge} is now free due to slave timeout.")

                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removed slave {slave_ns} due to timeout.")

        if slaves_to_remove:
            self.get_logger().info("Repartitioning and reassigning waypoints after slave removal.")
            self.waypoint_manager.repartition_and_assign_waypoints()

def main(args=None):
    rclpy.init(args=args)
    node = MasterNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
