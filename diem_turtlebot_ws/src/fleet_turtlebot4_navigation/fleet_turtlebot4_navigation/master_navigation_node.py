#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
from .graph_partitioning import load_full_graph, partition_graph
from .path_calculation import calculate_dcpp_route, orientation_rad_to_str
import os
import networkx as nx
import threading

class SlaveState:
    """
    Classe per gestire lo stato di ciascun slave robot.
    """
    def __init__(self, slave_ns, publisher):
        self.slave_ns = slave_ns
        self.publisher = publisher
        self.assigned_waypoints = []  # Lista di waypoint assegnati (DCPP route)
        self.current_waypoint_index = 0  # Indice del prossimo waypoint da assegnare
        self.last_seen_time = 0.0  # Ultima volta che lo slave ha comunicato
        self.initial_x = None  # Posizione iniziale X
        self.initial_y = None  # Posizione iniziale Y
        self.initial_orientation = None  # Orientamento iniziale
        self.waiting = False  # Flag per indicare se lo slave è in attesa di un waypoint

class MasterNavigationNode(Node):
    def __init__(self):
        super().__init__('master_navigation_node')

        # Dichiarazione dei parametri ROS 2 con valori predefiniti
        self.declare_parameter('graph_path', '/home/beniamino/turtlebot4/diem_turtlebot_ws/src/fleet_turtlebot4_navigation/map/navigation_hardware_limitation.json')
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('timeout', 150.0)  # Timeout per slave inattivi in secondi

        # Recupero dei valori dei parametri
        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Verifica l'esistenza del file del grafo
        if not os.path.exists(self.graph_path):
            self.get_logger().error(f"Graph file not found at {self.graph_path}")
            raise FileNotFoundError(f"Graph file not found at {self.graph_path}")

        # Caricamento del grafo completo dal file JSON specificato
        self.full_graph = load_full_graph(self.graph_path)

        # Publisher per inviare il grafo di navigazione
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)
        self.graph_timer = self.create_timer(5.0, self.publish_navigation_graph)

        # Inizializzazione dei subscriber per la registrazione degli slave e le posizioni iniziali
        self.slave_registration_subscriber = self.create_subscription(
            String,
            '/slave_registration',  # Topic assoluto
            self.slave_registration_callback,
            10
        )

        self.initial_position_subscriber = self.create_subscription(
            String,
            '/slave_initial_positions',  # Topic assoluto
            self.initial_position_callback,
            10
        )

        # Subscriber per ricevere aggiornamenti sullo stato della navigazione dagli slave
        self.navigation_status_subscriber = self.create_subscription(
            String,
            '/navigation_status',  # Topic assoluto
            self.navigation_status_callback,
            10
        )

        # Publisher per inviare messaggi di heartbeat indicando che il master è attivo
        self.heartbeat_publisher = self.create_publisher(String, '/master_heartbeat', 10)

        # Creazione di un timer che pubblica messaggi di heartbeat ogni 1 secondo
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Timer per controllare lo stato dei slave e rimuovere quelli inattivi
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Dizionario per tracciare gli slave attivi e il loro stato
        self.slaves = {}  # Chiave: slave_ns, Valore: SlaveState

        # Flag per indicare se il partizionamento del grafo e l'assegnazione dei waypoint sono stati completati
        self.partitioning_done = False

        # Aggiungi un set per tracciare i nodi attualmente occupati
        self.occupied_nodes = set()

        # Lock per gestire accessi concorrenti
        self.lock = threading.Lock()

        # Log dell'inizializzazione del master
        self.get_logger().info("Master node initialized.")

    def publish_navigation_graph(self):
        """
        Pubblica il grafo di navigazione sul topic '/navigation_graph'.
        """
        graph_msg = String()
        graph_data = {
            'nodes': [
                {'label': node, 'x': data['x'], 'y': data['y'], 'orientation': data.get('orientation', 0.0)}
                for node, data in self.full_graph.nodes(data=True)
            ],
            'edges': [
                {'from': u, 'to': v, 'weight': data.get('weight', 1.0)}
                for u, v, data in self.full_graph.edges(data=True)
            ]
        }
        graph_msg.data = json.dumps(graph_data)
        self.graph_publisher.publish(graph_msg)
        self.get_logger().info("Published navigation graph.")

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
        Callback per gestire i messaggi di registrazione degli slave.
        Aggiorna il dizionario degli slave con il namespace e l'ora corrente.
        Inizializza le strutture dati per i nuovi slave.

        Args:
            msg (String): Messaggio contenente il namespace dello slave.
        """
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns not in self.slaves:
            self.get_logger().info(f"New slave registered: {slave_ns}")

            # Crea un publisher per inviare comandi di navigazione allo slave
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            self.slaves[slave_ns] = slave_state

            # Non ripartizionare subito; attendi che lo slave invii la posizione iniziale
        else:
            # Aggiorna il tempo dell'ultimo aggiornamento dello slave
            self.slaves[slave_ns].last_seen_time = current_time

    def initial_position_callback(self, msg):
        """
        Callback per gestire i messaggi di posizione iniziale degli slave.
        Si aspetta una stringa JSON con 'robot_namespace', 'x', 'y', 'orientation'.

        Args:
            msg (String): Messaggio contenente la posizione iniziale dello slave.
        """
        # Salva il messaggio ricevuto per la diagnosi
        try:
            with open("/tmp/initial_position_messages.log", "a") as f:
                f.write(f"Received message: {msg.data}\n")
        except Exception as e:
            self.get_logger().error(f"Failed to write received message to log: {e}")

        # Log del messaggio ricevuto
        self.get_logger().info(f"Received initial position message: {msg.data}")

        try:
            # Parsing del messaggio JSON
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            initial_x = float(data['x'])
            initial_y = float(data['y'])
            orientation = data.get('orientation', 'NORTH')  # Assicurati di ricevere anche l'orientamento
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            # Stampa di errore nel caso il messaggio non sia valido
            self.get_logger().error(f"Invalid initial position message: {e}")
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time
            slave.initial_x = initial_x
            slave.initial_y = initial_y
            slave.initial_orientation = orientation
            self.get_logger().info(
                f"Received initial position from {slave_ns}: "
                f"({initial_x}, {initial_y}) with orientation {orientation}"
            )
            # Ripartizione del grafo e assegnazione dei waypoint poiché ora abbiamo la posizione iniziale
            self.repartition_and_assign_waypoints()
        else:
            # Slave non registrato, registrarlo ora
            self.get_logger().info(f"Initial position received for unregistered slave {slave_ns}, registering now.")
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            slave_state.initial_x = initial_x
            slave_state.initial_y = initial_y
            slave_state.initial_orientation = orientation
            self.slaves[slave_ns] = slave_state
            self.get_logger().info(f"Created publisher for slave: {slave_ns}")

            # Ripartizione del grafo e assegnazione dei waypoint poiché un nuovo slave è stato aggiunto
            self.repartition_and_assign_waypoints()

        # Stampa tutte le posizioni iniziali degli slave
        self.print_all_initial_positions()

    def print_all_initial_positions(self):
        """
        Stampa le posizioni iniziali di tutti gli slave registrati.
        """
        self.get_logger().info("Initial positions of all registered slaves:")
        for slave_ns, slave in self.slaves.items():
            if slave.initial_x is not None and slave.initial_y is not None:
                self.get_logger().info(
                    f"Slave {slave_ns}: Position ({slave.initial_x}, {slave.initial_y})"
                )
            else:
                self.get_logger().warn(
                    f"Slave {slave_ns} does not have a valid initial position."
                )

    def navigation_status_callback(self, msg):
        """
        Gestisce il feedback degli slave riguardo lo stato di navigazione.
        Si aspetta una stringa JSON con 'robot_namespace', 'status', 'current_waypoint', 'time_taken', 'error_message'.

        Args:
            msg (String): Messaggio contenente lo stato di navigazione dello slave.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_waypoint = data['current_waypoint']
            time_taken = data['time_taken']
            error_message = data.get('error_message', '')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid navigation status message: {e}")
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time

            if status == "reached":
                # Rimuovi il nodo dal set dei nodi occupati
                if current_waypoint in self.occupied_nodes:
                    self.occupied_nodes.remove(current_waypoint)
                    self.get_logger().info(f"Node {current_waypoint} is now free.")
                else:
                    self.get_logger().warn(f"Node {current_waypoint} was not marked as occupied.")

                self.get_logger().info(f"Slave {slave_ns} has reached waypoint {current_waypoint}.")
                slave.current_waypoint_index += 1
                slave.waiting = False  # Lo slave non è più in attesa
                self.assign_next_waypoint(slave_ns)

                # Dopo aver assegnato il prossimo waypoint, prova a assegnare waypoint agli slave in attesa
                self.assign_waiting_slaves()

                # Controlla se tutti i waypoint sono stati raggiunti per determinare il completamento del percorso
                if self.check_all_waypoints_reached():
                    self.get_logger().info("All DCPP routes have been completed. Fleet navigation is finished.")

            elif status == "error":
                # Gestione dello stato di errore
                self.get_logger().error(f"Slave {slave_ns} encountered an error: {error_message}")
                # Rimuovi il nodo dal set dei nodi occupati se necessario
                if current_waypoint in self.occupied_nodes:
                    self.occupied_nodes.remove(current_waypoint)
                    self.get_logger().info(f"Node {current_waypoint} is now free due to error.")
                # Rimuovi lo slave
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removing slave {slave_ns} due to error.")
                self.repartition_and_assign_waypoints()
        else:
            self.get_logger().warn(f"Received status from unknown slave {slave_ns}.")

    def check_all_waypoints_reached(self):
        """
        Controlla se tutti i waypoints assegnati agli slave sono stati raggiunti.

        Returns:
            bool: True se tutti i waypoints sono stati raggiunti, False altrimenti.
        """
        for slave in self.slaves.values():
            if slave.current_waypoint_index < len(slave.assigned_waypoints):
                return False
        return True

    def repartition_and_assign_waypoints(self):
        """
        Partiziona il grafo di navigazione in base al numero di slave attivi e assegna un percorso DCPP a ciascuno slave.
        Calcola il percorso DCPP (circuito Euleriano) per ogni sottografo.
        """
        num_slaves = len(self.slaves)
        if num_slaves == 0:
            self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
            self.partitioning_done = False
            return

        # Raccolta delle posizioni iniziali degli slave
        start_positions = []
        for slave in self.slaves.values():
            if slave.initial_x is not None and slave.initial_y is not None:
                start_positions.append({'x': slave.initial_x, 'y': slave.initial_y})
            else:
                self.get_logger().warn(f"Slave {slave.slave_ns} lacks valid initial position.")

        if len(start_positions) != num_slaves:
            self.get_logger().error("Not all slaves have valid initial positions.")
            return

        # Partiziona il grafo in sottografi basati sul numero di slave e le loro posizioni iniziali
        try:
            subgraphs = partition_graph(self.full_graph, num_slaves, start_positions=start_positions)
            self.get_logger().info(f"Partitioned the graph into {len(subgraphs)} subgraphs.")
            # Stampa i sottografi
            self.print_subgraphs(subgraphs)
        except ValueError as e:
            self.get_logger().error(f"Failed to partition graph: {e}")
            return

        # Ordina gli slave per garantire un'assegnazione coerente (es. ordine alfabetico)
        slaves_sorted = sorted(self.slaves.keys())

        # Assicurati che il numero di sottografi corrisponda al numero di slave
        if len(subgraphs) != len(slaves_sorted):
            self.get_logger().error("Number of subgraphs does not match number of active slaves.")
            return

        # Assegna un percorso DCPP a ciascuno slave
        for idx, slave_ns in enumerate(slaves_sorted):
            subgraph = subgraphs[idx]
            
            # Estrai i waypoint dal sottografo
            waypoints = self.extract_waypoints(subgraph)
            
            # Calcola il percorso DCPP passando waypoints, subgraph e logger
            dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.get_logger())
            ordered_route = dcpp_route  # Supponendo che calculate_dcpp_route ritorni il percorso ordinato

            # Log del percorso DCPP calcolato
            self.get_logger().info(f"DCPP Route for {slave_ns}:")
            for wp in ordered_route:
                self.get_logger().info(f"  {wp}")

            # Assegna i waypoints DCPP allo slave
            slave = self.slaves[slave_ns]
            slave.assigned_waypoints = ordered_route
            self.assign_next_waypoint(slave_ns)

        self.partitioning_done = True

    def assign_next_waypoint(self, slave_ns):
        """
        Assegna il prossimo waypoint nella coda allo slave.

        Args:
            slave_ns (str): Namespace dello slave robot.
        """
        slave = self.slaves[slave_ns]
        if slave.current_waypoint_index < len(slave.assigned_waypoints):
            waypoint = slave.assigned_waypoints[slave.current_waypoint_index]
            node_label = waypoint['label']

            if node_label in self.occupied_nodes:
                self.get_logger().warn(f"Node {node_label} is already occupied. Cannot assign to slave {slave_ns}.")
                slave.waiting = True  # Mette lo slave in attesa
                return

            waypoint_msg = {
                'label': waypoint['label'],
                'x': waypoint['x'],
                'y': waypoint['y'],
                'orientation': orientation_rad_to_str(waypoint['orientation'])  # Assicurati che questa funzione sia definita
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            slave.publisher.publish(msg)
            self.occupied_nodes.add(node_label)  # Segna il nodo come occupato
            self.get_logger().info(f"Assigned waypoint to {slave.slave_ns}: {waypoint_msg}")
        else:
            # Tutti i waypoint sono stati assegnati, fine del percorso
            self.get_logger().info(f"All waypoints have been assigned to {slave.slave_ns}. Route completed.")

    def assign_waiting_slaves(self):
        """
        Assegna i waypoint agli slave che sono in attesa.
        """
        for slave_ns in sorted(self.slaves.keys()):
            slave = self.slaves[slave_ns]
            if slave.waiting:
                if slave.current_waypoint_index < len(slave.assigned_waypoints):
                    waypoint = slave.assigned_waypoints[slave.current_waypoint_index]
                    node_label = waypoint['label']

                    if node_label not in self.occupied_nodes:
                        # Assegna il waypoint
                        waypoint_msg = {
                            'label': waypoint['label'],
                            'x': waypoint['x'],
                            'y': waypoint['y'],
                            'orientation': orientation_rad_to_str(waypoint['orientation'])  # Assicurati che questa funzione sia definita
                        }
                        msg = String()
                        msg.data = json.dumps(waypoint_msg)
                        slave.publisher.publish(msg)
                        self.occupied_nodes.add(node_label)  # Segna il nodo come occupato
                        self.get_logger().info(f"Assigned waypoint to {slave.slave_ns}: {waypoint_msg}")
                        slave.waiting = False  # Rimuove lo stato di attesa
                    else:
                        self.get_logger().warn(f"Node {node_label} is still occupied. Slave {slave.slave_ns} remains in waiting state.")
                else:
                    self.get_logger().info(f"All waypoints have been assigned to {slave.slave_ns}. Route completed.")
                    slave.waiting = False

    def print_subgraphs(self, subgraphs):
        """
        Stampa i dettagli di ciascun sottografo dopo la partizione.

        Args:
            subgraphs (list of nx.Graph): Lista di sottografi risultanti dalla partizione.
        """
        self.get_logger().info("----- Sottografi Dopo la Partizione -----")
        for idx, subgraph in enumerate(subgraphs):
            self.get_logger().info(f"Sottografo {idx+1}:")
            self.get_logger().info(f"  Nodi ({len(subgraph.nodes())}):")
            for node, data in subgraph.nodes(data=True):
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                orientation = data.get('orientation', 0.0)
                self.get_logger().info(f"    {node}: Posizione=({x}, {y}), Orientamento={orientation} radians")
            self.get_logger().info(f"  Archi ({len(subgraph.edges())}):")
            for u, v, data in subgraph.edges(data=True):
                weight = data.get('weight', 1.0)
                self.get_logger().info(f"    Da {u} a {v}, Peso: {weight}")
        self.get_logger().info("----- Fine dei Sottografi -----")

    def extract_waypoints(self, subgraph):
        """
        Estrae i waypoint da un sottografo.

        Args:
            subgraph (nx.Graph): Sottografo da cui estrarre i waypoint.

        Returns:
            list of dict: Lista di waypoint con 'label', 'x', 'y', e 'orientation'.
        """
        waypoints = []
        for node, data in subgraph.nodes(data=True):
            waypoint = {
                'label': node,
                'x': data['x'],
                'y': data['y'],
                'orientation': data.get('orientation', 0.0)
            }
            waypoints.append(waypoint)
        return waypoints

    def timer_callback(self):
        """
        Periodic callback to perform regular checks and manage the navigation fleet.
        """
        self.get_logger().debug("Timer callback triggered.")
        # Verifica se ci sono slave inattivi e rimuovili
        self.check_slaves_timeout()
        # Assegna waypoint agli slave in attesa, se possibile
        self.assign_waiting_slaves()

    def check_slaves_timeout(self):
        """
        Controlla se ci sono slave inattivi e li rimuove se superano il timeout.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        slaves_to_remove = []

        for slave_ns, slave in self.slaves.items():
            if current_time - slave.last_seen_time > self.timeout:
                self.get_logger().warn(f"Slave {slave_ns} has timed out. Removing from active slaves.")
                slaves_to_remove.append(slave_ns)

        for slave_ns in slaves_to_remove:
            # Rimuovi il nodo occupato se lo slave era assegnato a un waypoint
            if slave_ns in self.slaves:
                slave = self.slaves[slave_ns]
                if slave.current_waypoint_index < len(slave.assigned_waypoints):
                    waypoint = slave.assigned_waypoints[slave.current_waypoint_index]
                    node_label = waypoint['label']
                    if node_label in self.occupied_nodes:
                        self.occupied_nodes.remove(node_label)
                        self.get_logger().info(f"Node {node_label} is now free due to slave timeout.")

            del self.slaves[slave_ns]
            self.get_logger().warn(f"Removed slave {slave_ns} due to timeout.")

        if slaves_to_remove:
            # Ripartizione del grafo e assegnazione dei waypoint poiché il numero di slave è cambiato
            self.repartition_and_assign_waypoints()

def load_full_graph_from_data(graph_data):
    """
    Carica un grafo NetworkX da un dizionario contenente nodi e archi.

    Args:
        graph_data (dict): Dizionario con 'nodes' e 'edges'.

    Returns:
        nx.DiGraph: Il grafo diretto caricato.
    """
    G = nx.DiGraph()

    for node in graph_data['nodes']:
        label = node['label']
        x = node['x']
        y = node['y']
        orientation = node.get('orientation', 0.0)
        G.add_node(label, x=x, y=y, orientation=orientation)

    for edge in graph_data['edges']:
        u = edge['from']
        v = edge['to']
        weight = edge.get('weight', 1.0)
        G.add_edge(u, v, weight=weight)

    return G

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
