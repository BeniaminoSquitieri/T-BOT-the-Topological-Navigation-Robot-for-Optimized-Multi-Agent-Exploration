#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .master_callbacks import MasterCallbacks
from .heartbeat_manager import HeartbeatManager
from .waypoint_manager import WaypointManager
from .graph_utils import load_full_graph
from .slave_state import SlaveState


class MasterNavigationNode(Node, MasterCallbacks):
    def __init__(self):
        # Inizializza la classe Node
        Node.__init__(self, 'master_navigation_node')
        
        # Inizializza la classe MasterCallbacks
        MasterCallbacks.__init__(self)

        # Dichiarazione e recupero dei parametri
        self.declare_parameter(
            'graph_path',
            '/home/utente/percorso_al_file/result_graph.json'
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

        # Caricamento del grafo di navigazione (MultiGraph non diretto)
        self.full_graph = load_full_graph(self.graph_path)
        self.get_logger().info(f"Loaded undirected graph from {self.graph_path}.")

        # Publisher e subscriber
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)
        self.graph_timer = self.create_timer(1.0, self.publish_navigation_graph)

        self.slave_registration_subscriber = self.create_subscription(
            String, '/slave_registration', self.slave_registration_callback, 10
        )

        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )

        # Inizializzazione di HeartbeatManager
        self.heartbeat_manager = HeartbeatManager(self)

        # Inizializzazione di WaypointManager
        self.waypoint_manager = WaypointManager(self)

        # Timer per controlli periodici
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Tracciamento dello stato degli slave
        self.slaves = {}            # { namespace_slave: SlaveState(...) }
        self.partitioning_done = False

        # Per sapere quali archi sono occupati
        self.occupied_edges = set()
        # Per sapere chi occupa un certo arco
        self.edge_occupants = {}

        # Resetta occupied_edges e edge_occupants all'avvio
        self.reset_occupied_edges()

        # Per memorizzare l'ultimo tempo di visita di un edge (se servisse)
        self.edge_last_visit_time = {}

        self.get_logger().info("Master node initialized with undirected CPP approach.")


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

                # Libera l'edge occupato, se presente
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
            # Chiamata a WaypointManager
            self.waypoint_manager.repartition_and_assign_waypoints()


    def timer_callback(self):
        """
        Controlli periodici:
        - check_slaves_timeout
        - assign_waiting_slaves
        """
        self.get_logger().debug("Master timer callback triggered.")
        self.check_slaves_timeout()
        self.waypoint_manager.assign_waiting_slaves()


    def reset_occupied_edges(self):
        self.occupied_edges.clear()
        self.edge_occupants.clear()
        self.get_logger().info("Reset all occupied_edges and edge_occupants.")


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
