#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import networkx as nx
import os
import math
from .graph_partitioning import load_full_graph, partition_graph, save_subgraphs

class MasterNavigationNode(Node):
    def __init__(self):
        super().__init__('master_navigation_node')

        # Dichiarazione dei parametri ROS 2 con valori predefiniti
        self.declare_parameter('graph_path', '')
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('timeout', 10.0)  # Timeout per slave inattivi

        # Recupero dei valori dei parametri
        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Caricamento del grafo completo dal file JSON specificato
        self.full_graph = load_full_graph(self.graph_path)
        self.get_logger().info("Master node loaded the graph.")

        # Stampa di tutti i nodi del grafo caricato
        self.print_all_graph_nodes()

        # Inizializzazione dei subscriber per la registrazione degli slave e le posizioni iniziali
        self.slave_registration_subscriber = self.create_subscription(
            String,
            'slave_registration',
            self.slave_registration_callback,
            10
        )

        self.initial_position_subscriber = self.create_subscription(
            String,
            'slave_initial_positions',
            self.initial_position_callback,
            10
        )

        # Subscriber per ricevere aggiornamenti sullo stato della navigazione dagli slave
        self.navigation_status_subscriber = self.create_subscription(
            String,
            'navigation_status',
            self.navigation_status_callback,
            10
        )

        # Publisher per inviare messaggi di heartbeat indicando che il master è attivo
        self.heartbeat_publisher = self.create_publisher(String, 'master_heartbeat', 10)

        # Creazione di un timer che pubblica messaggi di heartbeat ogni 1 secondo
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Timer per controllare lo stato dei slave e assegnare nuovi waypoints
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Dizionari per tracciare gli slave attivi e il loro ultimo aggiornamento
        self.active_slaves = {}            # {namespace: last_seen_time}
        self.assigned_subgraphs = {}       # {namespace: subgraph}
        self.navigation_command_publishers = {}  # {namespace: publisher}

        # Insieme per tenere traccia globalmente dei waypoint assegnati e prevenire duplicazioni
        self.assigned_waypoints_set = set()

        # Flag per indicare se il partizionamento del grafo e l'assegnazione dei waypoint sono stati completati
        self.partitioning_done = False

    def print_all_graph_nodes(self):
        """
        Stampa tutti i nodi del grafo con le loro coordinate.
        """
        self.get_logger().info("Printing all graph nodes:")
        for node, data in self.full_graph.nodes(data=True):
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            orientation = data.get('orientation', 0.0)
            self.get_logger().info(f"Node {node}: x={x}, y={y}, orientation={orientation} radians")

    def publish_heartbeat(self):
        """
        Pubblica un messaggio di heartbeat per indicare che il master è attivo.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = "alive"
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug("Published heartbeat.")

    def slave_registration_callback(self, msg):
        """
        Callback funzione attivata quando viene ricevuto un messaggio di registrazione di uno slave.
        Aggiorna il dizionario active_slaves con il namespace dello slave e l'ora corrente.
        Inizializza le strutture dati per nuovi slave.
        """
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns not in self.active_slaves:
            self.get_logger().info(f"New slave registered: {slave_ns}")
        else:
            self.get_logger().info(f"Slave re-registered: {slave_ns}")

        self.active_slaves[slave_ns] = current_time

        # Se non esiste già un publisher per questo slave, crealo
        if slave_ns not in self.navigation_command_publishers:
            publisher = self.create_publisher(String, f"{slave_ns}/navigation_commands", 10)
            self.navigation_command_publishers[slave_ns] = publisher
            self.get_logger().info(f"Created publisher for slave: {slave_ns}")

        # Ripartizione del grafo e assegnazione dei waypoint ogni volta che un nuovo slave viene registrato
        self.repartition_and_assign_waypoints()

    def initial_position_callback(self, msg):
        """
        Callback funzione attivata quando viene ricevuta la posizione iniziale di uno slave.
        Si aspetta una stringa JSON con 'robot_namespace', 'x', 'y', 'orientation'.
        """
        data = json.loads(msg.data)
        slave_ns = data['robot_namespace']
        initial_x = data['x']
        initial_y = data['y']
        initial_orientation = data['orientation']

        self.active_slaves[slave_ns] = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(
            f"Received initial position from {slave_ns}: "
            f"({initial_x}, {initial_y}, {initial_orientation} radians)"
        )

        # Ripartizione del grafo e assegnazione dei waypoint se non ancora fatto
        if not self.partitioning_done:
            self.get_logger().info(
                "Proceeding to partition the graph and assign waypoints."
            )
            self.repartition_and_assign_waypoints()

    def timer_callback(self):
        """
        Callback funzione periodica per:
        - Controllare se ci sono slave inattivi basati sul timeout.
        - Rimuovere gli slave che hanno superato il timeout.
        - Ripartizionare e ri-assegnare i waypoint se gli slave attivi sono cambiati.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        removed_slaves = []

        # Controllo dei timeout degli slave
        for slave_ns in list(self.active_slaves.keys()):
            last_seen = self.active_slaves[slave_ns]
            if current_time - last_seen > self.timeout:
                removed_slaves.append(slave_ns)
                del self.active_slaves[slave_ns]
                self.get_logger().warn(f"Slave {slave_ns} timed out and removed from active slaves.")

                # Rimuovi il publisher associato
                if slave_ns in self.navigation_command_publishers:
                    del self.navigation_command_publishers[slave_ns]

        if removed_slaves:
            self.get_logger().info("Active slaves changed. Repartitioning graph and reassigning waypoints.")
            self.repartition_and_assign_waypoints()

        # Stampa lo stato attuale degli slave
        self.print_current_status()

    def navigation_status_callback(self, msg):
        """
        Gestisce il feedback degli slave riguardo lo stato della navigazione.
        Si aspetta una stringa JSON con 'robot_namespace', 'status', 'current_waypoint', 'time_taken', 'error_message'.
        """
        data = json.loads(msg.data)
        slave_ns = data['robot_namespace']
        status = data['status']
        current_waypoint = data['current_waypoint']
        time_taken = data['time_taken']
        error_message = data['error_message']

        self.get_logger().info(
            f"Received status from {slave_ns}: {status}, "
            f"Waypoint: {current_waypoint}, Time Taken: {time_taken}s, Error: {error_message}"
        )

        # Aggiorna il tempo di ultimo aggiornamento dello slave
        if slave_ns in self.active_slaves:
            self.active_slaves[slave_ns] = self.get_clock().now().nanoseconds / 1e9

        if status == "reached":
            # Segnala che lo slave ha raggiunto il waypoint e necessita di nuovi waypoints
            self.get_logger().info(f"Slave {slave_ns} has reached waypoint {current_waypoint}. Repartitioning graph.")
            self.repartition_and_assign_waypoints()
        elif status == "error":
            self.get_logger().error(f"Slave {slave_ns} encountered an error: {error_message}")
            # Potresti implementare una logica di retry o di gestione degli errori qui
            self.repartition_and_assign_waypoints()

    def repartition_and_assign_waypoints(self):
        """
        Partiziona il grafo di navigazione in base al numero di slave attivi e assegna waypoint a ciascuno slave.
        Calcola il percorso DCPP (circuito Euleriano) per ogni sottografo.
        """
        num_slaves = len(self.active_slaves)
        if num_slaves == 0:
            self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
            return

        # Partiziona il grafo in sottografi
        try:
            subgraphs = partition_graph(self.full_graph, num_slaves)
            self.get_logger().info(f"Partitioned the graph into {len(subgraphs)} subgraphs.")
        except ValueError as e:
            self.get_logger().error(f"Failed to partition graph: {e}")
            return

        # Salva i sottografi
        subgraph_paths = save_subgraphs(subgraphs, self.output_dir)
        self.get_logger().info(f"Saved subgraphs to {subgraph_paths}")

        # Ordina gli slave per garantire una assegnazione coerente
        slaves = sorted(self.active_slaves.keys())

        # Assicurati che il numero di sottografi corrisponda al numero di slave
        if len(subgraph_paths) != len(slaves):
            self.get_logger().error("Number of subgraphs does not match number of active slaves.")
            return

        # Assegna ciascun sottografo a uno slave
        for idx, slave_ns in enumerate(slaves):
            subgraph_file = subgraph_paths[idx]
            waypoints = self.extract_waypoints(subgraph_file)
            # Calcola il percorso DCPP (circuito Euleriano) per il sottografo
            dcpp_route = self.calculate_dcpp_route(waypoints, subgraph_file)
            # Assegna il percorso ordinato allo slave
            self.assign_route_to_slave(slave_ns, dcpp_route)

        self.partitioning_done = True

    def extract_waypoints(self, subgraph_file_path):
        """
        Estrae i waypoint da un file JSON di un sottografo.

        Args:
            subgraph_file_path (str): Percorso al file JSON del sottografo.

        Returns:
            list of dict: Lista di waypoint con 'label', 'x', 'y', e 'orientation'.
        """
        with open(subgraph_file_path, 'r') as f:
            data = json.load(f)

        waypoints = []
        for node in data['nodes']:
            waypoint = {
                'label': node['label'],
                'x': node['x'],
                'y': node['y'],
                'orientation': node.get('orientation', 0.0)
            }
            waypoints.append(waypoint)
        return waypoints

    def calculate_dcpp_route(self, waypoints, subgraph_file_path):
        """
        Calcola il percorso DCPP (circuito Euleriano) per una lista di waypoint.

        Args:
            waypoints (list of dict): Lista di waypoint.
            subgraph_file_path (str): Percorso al file JSON del sottografo.

        Returns:
            list of dict: Lista ordinata di waypoint che costituiscono il percorso DCPP.
        """
        # Crea un grafo NetworkX a partire dai waypoint
        G = nx.DiGraph()
        for wp in waypoints:
            G.add_node(wp['label'], x=wp['x'], y=wp['y'], orientation=wp['orientation'])

        # Carica gli archi dal file subgraph JSON
        with open(subgraph_file_path, 'r') as f:
            data = json.load(f)

        for edge in data['edges']:
            u = edge['from']
            v = edge['to']
            weight = edge.get('weight', 1.0)
            G.add_edge(u, v, weight=weight)

        # Calcola il circuito Euleriano
        try:
            euler_circuit = list(nx.eulerian_circuit(G, source=waypoints[0]['label']))
            route_labels = [waypoints[0]['label']]
            for u, v in euler_circuit:
                route_labels.append(v)
        except nx.NetworkXError as e:
            self.get_logger().error(f"Errore nel calcolo del circuito Euleriano: {e}")
            # Fallback: sequenza lineare dei waypoint
            route_labels = [wp['label'] for wp in waypoints]

        # Mappa i label dei nodi al dizionario dei waypoint
        label_to_wp = {wp['label']: wp for wp in waypoints}
        ordered_route = [label_to_wp[label] for label in route_labels if label in label_to_wp]

        # Log del numero di nodi nel percorso
        self.get_logger().info(f"Calculated DCPP route with {len(ordered_route)} nodes.")

        return ordered_route

    def assign_route_to_slave(self, slave_ns, route):
        """
        Assegna un percorso di waypoint a uno slave specifico.

        Args:
            slave_ns (str): Namespace dello slave robot.
            route (list of dict): Lista ordinata di waypoint.
        """
        if slave_ns not in self.navigation_command_publishers:
            self.get_logger().error(f"No publisher found for slave {slave_ns}. Cannot assign route.")
            return

        publisher = self.navigation_command_publishers[slave_ns]

        for waypoint in route:
            waypoint_msg = {
                'label': waypoint['label'],
                'x': waypoint['x'],
                'y': waypoint['y'],
                'orientation': self.orientation_conversion(waypoint['orientation'])
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            publisher.publish(msg)
            self.get_logger().info(f"Assigned waypoint to {slave_ns}: {waypoint_msg}")

        # Log del numero di nodi assegnati
        self.get_logger().info(f"Assigned {len(route)} waypoints to {slave_ns}")

    def orientation_conversion(self, orientation_radians):
        """
        Converte un angolo in radianti nella stringa di orientamento corrispondente.

        Args:
            orientation_radians (float): Angolo in radianti.

        Returns:
            str: Orientamento come stringa ('NORTH', 'EAST', 'SOUTH', 'WEST').
        """
        orientation_map = {
            0.0: 'NORTH',
            -math.pi / 2: 'EAST',
            math.pi: 'SOUTH',
            math.pi / 2: 'WEST'
        }

        # Tolleranza per la corrispondenza degli angoli
        tolerance = 0.1  # Radianti (~5.7 gradi)

        for angle, direction in orientation_map.items():
            if abs(orientation_radians - angle) < tolerance:
                return direction

        # Se nessuna corrispondenza esatta, assegnare la direzione più vicina
        closest_angle = min(orientation_map.keys(), key=lambda k: abs(k - orientation_radians))
        return orientation_map[closest_angle]

    def print_current_status(self):
        """
        Stampa lo stato attuale degli slave attivi e dei waypoint assegnati.
        """
        num_active_slaves = len(self.active_slaves)
        self.get_logger().info(f"Current active slaves: {num_active_slaves}")
        for slave_ns in self.active_slaves:
            # Controlla se lo slave ha un publisher (indicando che ha ricevuto un percorso)
            if slave_ns in self.navigation_command_publishers:
                status = "Assigned"  # Potresti implementare uno stato più dettagliato
            else:
                status = "Available"
            self.get_logger().info(f"Slave {slave_ns}: Status: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = MasterNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
