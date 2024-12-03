#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
from .graph_partitioning import load_full_graph, partition_graph
from .path_calculation import calculate_dcpp_route, orientation_rad_to_str  # Modificato
import os

class SlaveState:
    """
    Classe per gestire lo stato di ciascun slave robot.
    """
    def __init__(self, slave_ns, publisher):
        self.slave_ns = slave_ns
        self.publisher = publisher
        self.assigned_waypoints = []  # Lista di waypoint assegnati
        self.current_waypoint_index = 0  # Indice del prossimo waypoint da assegnare
        self.last_seen_time = 0.0  # Ultima volta che lo slave ha comunicato
        self.initial_x = None  # Posizione iniziale X
        self.initial_y = None  # Posizione iniziale Y
        self.initial_orientation = None  # Orientamento iniziale

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
        # self.get_logger().info("Master node loaded the graph.")

        # Stampa di tutti i nodi del grafo caricato
        # self.print_all_graph_nodes()

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

        # Timer per controllare lo stato dei slave e assegnare nuovi waypoints
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Dizionario per tracciare gli slave attivi e il loro stato
        self.slaves = {}  # Chiave: slave_ns, Valore: SlaveState

        # Flag per indicare se il partizionamento del grafo e l'assegnazione dei waypoint sono stati completati
        self.partitioning_done = False

    def print_all_graph_nodes(self):
        """
        Stampa tutti i nodi del grafo con le loro coordinate.
        """
        self.get_logger().info("----- Grafico Caricato -----")
        self.get_logger().info(f"Nodi ({len(self.full_graph.nodes())}):")
        for node, data in self.full_graph.nodes(data=True):
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            orientation = data.get('orientation', 0.0)
            self.get_logger().info(f"  Nodo: {node}, Posizione: ({x}, {y}), Orientamento: {orientation} radians")
        self.get_logger().info(f"Archi ({len(self.full_graph.edges())}):")
        for u, v, data in self.full_graph.edges(data=True):
            weight = data.get('weight', 1.0)
            self.get_logger().info(f"  Da {u} a {v}, Peso: {weight}")
        self.get_logger().info("----- Fine del Grafico -----")

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
        # self.get_logger().info("Published navigation graph.")

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
            # self.get_logger().info(f"Slave re-registered: {slave_ns}")
            # Aggiorna il tempo dell'ultimo aggiornamento dello slave
            self.slaves[slave_ns].last_seen_time = current_time

    def initial_position_callback(self, msg):
        """
        Callback per gestire i messaggi di posizione iniziale degli slave.
        Si aspetta una stringa JSON con 'robot_namespace', 'x', 'y'.

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
            self.get_logger().info(
                f"Received initial position from {slave_ns}: "
                f"({initial_x}, {initial_y})"
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

        # self.get_logger().info(
        #     f"Received status from {slave_ns}: {status}, "
        #     f"Waypoint: {current_waypoint}, Time Taken: {time_taken}s, Error: {error_message}"
        # )

        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time

            if status == "reached":
                # Slave ha raggiunto il waypoint corrente, assegna il successivo
                self.get_logger().info(f"Slave {slave_ns} has reached waypoint {current_waypoint}.")
                slave.current_waypoint_index += 1
                self.assign_next_waypoint(slave)
            elif status == "error":
                # Gestione dello stato di errore
                self.get_logger().error(f"Slave {slave_ns} encountered an error: {error_message}")
                # Implementa la logica di retry o gestione degli errori se necessario
                # Per ora, rimuoviamo lo slave
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removing slave {slave_ns} due to error.")
                self.repartition_and_assign_waypoints()
        else:
            self.get_logger().warn(f"Received status from unknown slave {slave_ns}.")

    def repartition_and_assign_waypoints(self):
        """
        Partiziona il grafo di navigazione in base al numero di slave attivi e assegna waypoint a ciascuno slave.
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

        # Assegna ogni sottografo a uno slave
        for idx, slave_ns in enumerate(slaves_sorted):
            subgraph = subgraphs[idx]
            waypoints = self.extract_waypoints(subgraph)

            # Verifica che lo slave abbia una posizione iniziale
            slave = self.slaves[slave_ns]
            if slave.initial_x is not None and slave.initial_y is not None:
                # Assicurati che il primo waypoint sia la posizione iniziale dello slave
                # Trova il waypoint più vicino alla posizione iniziale
                min_distance = float('inf')
                initial_wp = None
                for wp in waypoints:
                    distance = math.hypot(wp['x'] - slave.initial_x, wp['y'] - slave.initial_y)
                    if distance < min_distance:
                        min_distance = distance
                        initial_wp = wp
                if initial_wp:
                    # Riordina i waypoints in modo che initial_wp sia il primo
                    waypoints = [initial_wp] + [wp for wp in waypoints if wp != initial_wp]
            else:
                self.get_logger().warn(f"Slave {slave_ns} lacks initial position data.")

            # Calcola il percorso DCPP (circuito Euleriano) per il sottografo
            dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.get_logger())

            # Stampa del percorso DCPP calcolato
            # self.print_dcpp_route(slave_ns, dcpp_route)
  
            # Assegna il percorso allo slave
            self.assign_route_to_slave(slave_ns, dcpp_route)

        self.partitioning_done = True

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

    def print_dcpp_route(self, slave_ns, route):
        """
        Stampa il percorso DCPP calcolato per uno slave.

        Args:
            slave_ns (str): Namespace dello slave.
            route (list of dict): Percorso DCPP calcolato.
        """
        self.get_logger().info(f"----- DCPP Route for {slave_ns} -----")
        for idx, wp in enumerate(route):
            self.get_logger().info(f"  Waypoint {idx+1}: {wp['label']} at ({wp['x']}, {wp['y']})")
        self.get_logger().info(f"----- End of DCPP Route for {slave_ns} -----")

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

    def assign_route_to_slave(self, slave_ns, route):
        """
        Assegna un percorso di waypoint a uno slave specifico.

        Args:
            slave_ns (str): Namespace dello slave robot.
            route (list of dict): Lista ordinata di waypoint.
        """
        if slave_ns not in self.slaves:
            self.get_logger().error(f"No slave state found for {slave_ns}. Cannot assign route.")
            return

        slave = self.slaves[slave_ns]
        slave.assigned_waypoints = route.copy()
        slave.current_waypoint_index = 0

        # Log dettagliato del percorso assegnato
        # self.get_logger().info(f"Assigned DCPP route to {slave_ns}. Total waypoints: {len(route)}")

        # Assegna il primo waypoint 
        self.assign_next_waypoint(slave)

    def assign_next_waypoint(self, slave):
        """
        Assegna il prossimo waypoint nella coda allo slave.

        Args:
            slave (SlaveState): Lo slave a cui assegnare il waypoint.
        """
        if slave.current_waypoint_index < len(slave.assigned_waypoints):
            waypoint = slave.assigned_waypoints[slave.current_waypoint_index]
            waypoint_msg = {
                'label': waypoint['label'],
                'x': waypoint['x'],
                'y': waypoint['y'],
                'orientation': orientation_rad_to_str(waypoint['orientation'])  # Modificato
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            slave.publisher.publish(msg)
            self.get_logger().info(f"Assigned waypoint to {slave.slave_ns}: {waypoint_msg}")
        else:
            # Tutti i waypoint sono stati assegnati, fine del percorso
            self.get_logger().info(f"All waypoints have been assigned to {slave.slave_ns}. Route completed.")

    def print_current_status(self):
        """
        Stampa lo stato attuale degli slave attivi.
        """
        num_active_slaves = len(self.slaves)
        self.get_logger().info(f"Current active slaves: {num_active_slaves}")
        for slave_ns in sorted(self.slaves.keys()):
            slave = self.slaves[slave_ns]
            status = "Assigned" if slave.current_waypoint_index < len(slave.assigned_waypoints) else "Completed"
            self.get_logger().info(f"Slave {slave_ns}: Status: {status}")

    def timer_callback(self):
        """
        Callback del timer per monitorare lo stato degli slave e rimuovere quelli inattivi.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9  # Ottieni il tempo corrente in secondi
        inactive_slaves = []

        # Controlla se qualche slave è inattivo
        for slave_ns, slave in list(self.slaves.items()):  # Usa list() per evitare RuntimeError durante la modifica del dizionario
            if current_time - slave.last_seen_time > self.timeout:
                self.get_logger().warn(f"Slave {slave_ns} has been inactive for more than {self.timeout} seconds.")
                inactive_slaves.append(slave_ns)

        # Rimuovi gli slave inattivi
        for slave_ns in inactive_slaves:
            self.get_logger().warn(f"Removing inactive slave: {slave_ns}")
            del self.slaves[slave_ns]
            self.repartition_and_assign_waypoints()

        # Stampa lo stato corrente degli slave
        # self.print_current_status()

        # **Importante:** Non assegnare waypoint in questa callback.
        # L'assegnazione dei waypoint è gestita nella navigation_status_callback quando lo slave conferma il raggiungimento di un waypoint.

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
