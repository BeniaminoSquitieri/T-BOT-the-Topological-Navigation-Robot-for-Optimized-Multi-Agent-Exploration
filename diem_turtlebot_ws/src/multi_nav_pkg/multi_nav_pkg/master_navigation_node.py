# multi_nav_pkg/master_navigation_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multi_nav_pkg.msg import Waypoint, NavigationStatus
import json
import networkx as nx
import os
import sys
import time
import logging
import socket
from graph_partitioning import load_full_graph, partition_graph, save_subgraphs

class MasterNavigationNode(Node):
    def __init__(self, graph_path, output_dir, check_interval=2.0, timeout=5.0):
        super().__init__('master_navigation_node')
        
        self.graph_path = graph_path
        self.output_dir = output_dir
        self.check_interval = check_interval  # Intervallo per il controllo dei slave
        self.timeout = timeout  # Tempo di timeout per considerare un slave inattivo

        # Carica il grafo completo
        self.full_graph = load_full_graph(self.graph_path)
        self.get_logger().info("Master node loaded the graph.")

        # Sottoscrizioni e Publisher
        self.slave_registration_subscriber = self.create_subscription(
            String,
            'slave_registration',
            self.slave_registration_callback,
            10
        )

        # Publisher per i comandi di navigazione per ogni slave
        self.navigation_command_publishers = {}

        # Dizionario per tracciare i slave attivi: {namespace: last_seen_time}
        self.active_slaves = {}

        # Dizionario per tracciare i waypoint assegnati: {namespace: [waypoints]}
        self.assigned_waypoints = {}

        # Dizionario per tracciare se lo slave sta navigando: {namespace: bool}
        self.slave_busy = {}

        # Set per tracciare i waypoint assegnati (per evitare duplicazioni)
        self.assigned_waypoints_set = set()

        # Timer per il controllo periodico dei slave
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Subscriber per ricevere feedback dai slave
        self.navigation_status_subscriber = self.create_subscription(
            NavigationStatus,
            'navigation_status',
            self.navigation_status_callback,
            10
        )

        # Publisher per heartbeat
        self.heartbeat_publisher = self.create_publisher(String, 'master_heartbeat', 10)

        # Timer per inviare heartbeat
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Configurazione del logger per inviare i log a Logstash
        self.logstash_host = '192.168.1.100'  # Indirizzo IP del server Logstash
        self.logstash_port = 5000            # Porta configurata in Logstash

        # Configura il logger
        self.logger = logging.getLogger('MasterNavigationNode')
        self.logger.setLevel(logging.DEBUG)

        # Crea un handler socket
        try:
            self.socket_handler = logging.handlers.SocketHandler(self.logstash_host, self.logstash_port)
            self.logger.addHandler(self.socket_handler)
            self.logger.debug("Socket handler for Logstash successfully added.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Logstash: {e}")

        # Override dei metodi di logging per inviare i log anche a Logstash
        self.get_logger().info = self.custom_info
        self.get_logger().warn = self.custom_warn
        self.get_logger().error = self.custom_error

    def custom_info(self, msg):
        self.logger.info(msg)
        super().get_logger().info(msg)

    def custom_warn(self, msg):
        self.logger.warning(msg)
        super().get_logger().warn(msg)

    def custom_error(self, msg):
        self.logger.error(msg)
        super().get_logger().error(msg)

    def publish_heartbeat(self):
        heartbeat_msg = String()
        heartbeat_msg.data = "alive"
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug("Published heartbeat.")

    def slave_registration_callback(self, msg):
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().nanoseconds / 1e9  # Tempo corrente in secondi
        self.active_slaves[slave_ns] = current_time
        self.get_logger().info(f"Received registration from slave: {slave_ns}")
        # Inizializza le strutture dati per il nuovo slave
        if slave_ns not in self.assigned_waypoints:
            self.assigned_waypoints[slave_ns] = []
            self.slave_busy[slave_ns] = False

    def timer_callback(self):
        # Tempo corrente
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Rimuove i slave che non hanno inviato una registrazione entro il timeout
        removed_slaves = []
        for slave_ns in list(self.active_slaves.keys()):
            last_seen = self.active_slaves[slave_ns]
            if current_time - last_seen > self.timeout:
                removed_slaves.append(slave_ns)
                del self.active_slaves[slave_ns]
                self.get_logger().warn(f"Slave {slave_ns} timed out and removed from active slaves.")
                # Chiudi il publisher se esiste
                if slave_ns in self.navigation_command_publishers:
                    del self.navigation_command_publishers[slave_ns]
                # Rimuovi waypoint assegnati
                if slave_ns in self.assigned_waypoints:
                    # Rilascia i waypoint assegnati
                    for wp in self.assigned_waypoints[slave_ns]:
                        wp_id = (wp['label'], wp['x'], wp['y'])
                        self.assigned_waypoints_set.discard(wp_id)
                    del self.assigned_waypoints[slave_ns]
                if slave_ns in self.slave_busy:
                    del self.slave_busy[slave_ns]
        
        # Se il numero di slave attivi è cambiato, ripartiziona il grafo e ridistribuisci i waypoint
        if removed_slaves or len(self.active_slaves) != len(self.navigation_command_publishers):
            self.get_logger().info("Active slaves changed. Repartitioning graph and reassigning waypoints.")
            self.repartition_and_assign_waypoints()

        # Assegna i waypoint agli slave che non sono occupati
        for slave_ns in self.active_slaves:
            if not self.slave_busy.get(slave_ns, False):
                if self.assigned_waypoints[slave_ns]:
                    next_waypoint = self.assigned_waypoints[slave_ns].pop(0)
                    # Rimuovi il waypoint dal set di assegnazione globale
                    wp_id = (next_waypoint['label'], next_waypoint['x'], next_waypoint['y'])
                    self.assigned_waypoints_set.discard(wp_id)
                    self.send_waypoint(slave_ns, next_waypoint)

    def repartition_and_assign_waypoints(self):
        num_slaves = len(self.active_slaves)
        if num_slaves == 0:
            self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
            return

        # Partiziona il grafo
        subgraphs = partition_graph(self.full_graph, num_slaves)
        self.get_logger().info(f"Partitioned the graph into {num_slaves} subgraphs.")

        # Salva i sottografi in file
        subgraph_paths = save_subgraphs(subgraphs, self.output_dir)
        self.get_logger().info(f"Saved subgraphs to {subgraph_paths}")

        # Assegna ogni sottografo a uno slave
        slaves = sorted(self.active_slaves.keys())  # Ordinare per assegnazione deterministica
        for idx, slave_ns in enumerate(slaves):
            subgraph_file = subgraph_paths[idx]
            waypoints = self.extract_waypoints(subgraph_file)
            # Filtra i waypoint già assegnati
            waypoints = [wp for wp in waypoints if (wp['label'], wp['x'], wp['y']) not in self.assigned_waypoints_set]
            self.assigned_waypoints[slave_ns] = waypoints.copy()
            # Aggiorna il set globale dei waypoint assegnati
            for wp in waypoints:
                wp_id = (wp['label'], wp['x'], wp['y'])
                self.assigned_waypoints_set.add(wp_id)
            self.slave_busy[slave_ns] = False  # Reset dello stato
            self.get_logger().info(f"Assigned {len(waypoints)} waypoints to {slave_ns}")

    def extract_waypoints(self, subgraph_file_path):
        # Carica il sottografo dal file
        with open(subgraph_file_path, 'r') as f:
            data = json.load(f)
        
        # Estrai i waypoint in ordine (puoi personalizzare l'ordine se necessario)
        waypoints = []
        for node in data['nodes']:
            waypoint = {
                'label': node['label'],
                'x': node['x'],
                'y': node['y'],
                'orientation': node.get('orientation', 0.0)  # Default orientation
            }
            waypoints.append(waypoint)
        return waypoints

    def send_waypoint(self, slave_ns, waypoint):
        # Crea o recupera il publisher per lo slave
        if slave_ns not in self.navigation_command_publishers:
            publisher = self.create_publisher(Waypoint, f"{slave_ns}/navigation_commands", 10)
            self.navigation_command_publishers[slave_ns] = publisher
            self.get_logger().info(f"Created publisher for slave: {slave_ns}")
        else:
            publisher = self.navigation_command_publishers[slave_ns]

        # Crea il messaggio Waypoint
        waypoint_msg = Waypoint()
        waypoint_msg.label = waypoint['label']
        waypoint_msg.x = waypoint['x']
        waypoint_msg.y = waypoint['y']
        waypoint_msg.orientation = waypoint['orientation']

        # Pubblica il waypoint
        publisher.publish(waypoint_msg)
        self.get_logger().info(f"Sent waypoint to {slave_ns}: {waypoint_msg}")
        
        # Segna lo slave come occupato
        self.slave_busy[slave_ns] = True

    def navigation_status_callback(self, msg):
        # Gestisci il feedback dai nodi slave
        self.get_logger().info(f"Received status from {msg.robot_namespace}: {msg.status}, Waypoint: {msg.current_waypoint}, Time Taken: {msg.time_taken}s, Error: {msg.error_message}")

        if msg.status == "reached":
            # Segna lo slave come disponibile per il prossimo waypoint
            self.slave_busy[msg.robot_namespace] = False
        elif msg.status == "error":
            # Gestisci gli errori, ad esempio riprogrammando i waypoint
            self.get_logger().error(f"Slave {msg.robot_namespace} encountered an error: {msg.error_message}")
            self.slave_busy[msg.robot_namespace] = False
            # Potresti riprogrammare i waypoint rimanenti
            self.repartition_and_assign_waypoints()

def main(args=None):
    rclpy.init(args=args)

    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('--graph_path', type=str, required=True, help='Path to the full graph JSON file')
    parser.add_argument('--output_dir', type=str, default='/tmp', help='Directory to save subgraphs')
    args, unknown = parser.parse_known_args()

    master_node = MasterNavigationNode(
        graph_path=args.graph_path,
        output_dir=args.output_dir
    )

    try:
        rclpy.spin(master_node)
    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
