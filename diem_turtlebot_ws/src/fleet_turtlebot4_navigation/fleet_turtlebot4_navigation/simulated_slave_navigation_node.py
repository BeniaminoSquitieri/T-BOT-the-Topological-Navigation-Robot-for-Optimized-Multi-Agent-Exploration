#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import argparse
import math
import threading
import networkx as nx  # Importazione corretta di networkx

class SlaveState:
    """
    Classe per gestire lo stato di ciascun slave robot.
    """
    def __init__(self, slave_ns):
        self.slave_ns = slave_ns
        self.assigned_waypoints = []  # Lista di waypoint assegnati
        self.current_waypoint_index = 0  # Indice del prossimo waypoint da assegnare
        self.last_seen_time = 0.0  # Ultima volta che lo slave ha comunicato
        self.initial_x = None  # Posizione iniziale X
        self.initial_y = None  # Posizione iniziale Y
        self.initial_orientation = None  # Orientamento iniziale
        self.publisher = None  # Publisher per inviare comandi di navigazione
        self.waiting = False  # Flag per indicare se lo slave è in attesa di un waypoint

class SlaveNavigationSimulator(Node):
    def __init__(self, robot_namespace, initial_x, initial_y, initial_orientation_str):
        # Inizializzazione delle variabili principali
        self.robot_namespace = robot_namespace
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.initial_orientation_str = initial_orientation_str
        self.initial_orientation = self.orientation_conversion(initial_orientation_str)
        super().__init__('slave_navigation_simulator_node', namespace=self.robot_namespace)

        # Publisher per registrare lo slave con il master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)

        # Publisher per inviare la posizione iniziale al master
        self.initial_position_publisher = self.create_publisher(String, '/slave_initial_positions', 10)

        # Subscriber per i comandi di navigazione
        self.navigation_commands_subscriber = self.create_subscription(
            String,
            'navigation_commands',  # Topic relativo
            self.navigation_commands_callback,
            10
        )

        # Publisher per inviare lo stato della navigazione al master
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)  # Topic assoluto

        # Publisher per inviare messaggi di heartbeat per indicare che questo slave è attivo
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)  # Topic assoluto

        # Subscriber per ricevere messaggi di heartbeat dal master
        self.master_heartbeat_subscriber = self.create_subscription(
            String,
            '/master_heartbeat',
            self.master_heartbeat_callback,
            10
        )

        # Subscriber per ricevere messaggi di heartbeat dagli altri slave
        self.slave_heartbeat_subscriber = self.create_subscription(
            String,
            '/slave_heartbeat',
            self.slave_heartbeat_callback,
            10
        )

        # Subscriber per ricevere il grafo di navigazione dal master
        self.graph_subscriber = self.create_subscription(
            String,
            '/navigation_graph',
            self.navigation_graph_callback,
            10
        )

        # Publisher per inviare il grafo di navigazione se diventa master (non utilizzato nel simulatore)
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)

        # Timer per pubblicare regolarmente i messaggi di registrazione
        self.registration_timer = self.create_timer(1.0, self.publish_registration)

        # Timer per pubblicare regolarmente i messaggi di heartbeat
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Timer per pubblicare regolarmente i messaggi di posizione iniziale
        self.initial_position_timer = self.create_timer(2.0, self.publish_initial_position)

        # Inizializza le variabili per l'elezione del master
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 5.0  # Secondi da attendere prima di considerare il master morto

        # Lista degli slave attivi (namespace e ultimo heartbeat)
        self.active_slaves = {}  # Chiave: slave_ns, Valore: last_seen_time

        # Inizializza il grafo di navigazione
        self.navigation_graph = None

        # Aggiungi un set per tracciare i nodi attualmente occupati
        self.occupied_nodes = set()

        # Flag per indicare se il partizionamento del grafo e l'assegnazione dei waypoint sono stati completati
        self.partitioning_done = False

        # Inizializza l'indice del waypoint corrente
        self.current_waypoint_index = 0  # Inizializzazione per evitare AttributeError

        # Flag per tracciare la pubblicazione della posizione iniziale
        self.initial_position_published = False

        # Log dell'inizializzazione dello slave
        self.get_logger().info(f"[{self.robot_namespace}] Slave simulator initialized at ({self.initial_x}, {self.initial_y}) with orientation {self.initial_orientation_str} ({self.initial_orientation} radians).")

        # Timer per controllare il timeout del heartbeat del master
        self.master_check_timer = self.create_timer(10.0, self.check_master_alive)

        # Timer per controllare gli heartbeat degli slave e mantenere la lista active_slaves
        self.slave_check_timer = self.create_timer(2.0, self.check_slave_alive)

        # Inizializza il flag del ruolo di master
        self.is_master = False

        # Lock per gestire accessi concorrenti
        self.lock = threading.Lock()

    def publish_registration(self):
        """
        Pubblica un messaggio di registrazione al master per indicare che questo slave è attivo.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published registration.")

    def publish_initial_position(self):
        """
        Pubblica la posizione iniziale dello slave al master.
        """
        if not self.initial_position_published:
            initial_position = {
                'robot_namespace': self.robot_namespace,
                'x': self.initial_x,
                'y': self.initial_y,
                'orientation': self.initial_orientation_str  # Invia l'orientamento come stringa
            }
            msg = String()
            msg.data = json.dumps(initial_position)
            self.initial_position_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Published initial position: {initial_position}")
            self.initial_position_published = True
            self.initial_position_timer.cancel()  # Ferma il timer dopo la prima pubblicazione

    def publish_heartbeat(self):
        """
        Pubblica un messaggio di heartbeat per indicare che questo slave è attivo.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published heartbeat.")

    def master_heartbeat_callback(self, msg):
        """
        Callback attivato quando viene ricevuto un messaggio di heartbeat dal master.
        Aggiorna il timestamp dell'heartbeat del master.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received master heartbeat.")

    def slave_heartbeat_callback(self, msg):
        """
        Callback attivato quando viene ricevuto un messaggio di heartbeat da un altro slave.
        Aggiorna la lista degli active_slaves.
        """
        slave_ns = msg.data.strip()
        current_time = time.time()
        if slave_ns != self.robot_namespace:
            with self.lock:
                self.active_slaves[slave_ns] = current_time
            self.get_logger().debug(f"[{self.robot_namespace}] Received heartbeat from slave {slave_ns}.")

    def check_master_alive(self):
        """
        Controlla se il master è attivo in base all'ultimo heartbeat ricevuto.
        Se il master è considerato morto, avvia l'elezione di un nuovo master.
        """
        current_time = time.time()
        if self.master_alive:
            # Reset del flag; verrà impostato di nuovo se un heartbeat viene ricevuto
            self.master_alive = False
        else:
            # Nessun heartbeat ricevuto dall'ultimo controllo
            if current_time - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost. Initiating master election.")
                self.elect_new_master()

    def check_slave_alive(self):
        """
        Controlla gli heartbeat degli altri slave e rimuove quelli che hanno superato il timeout.
        """
        current_time = time.time()
        with self.lock:
            for slave_ns in list(self.active_slaves.keys()):
                if current_time - self.active_slaves[slave_ns] > self.heartbeat_timeout:
                    self.get_logger().warn(f"[{self.robot_namespace}] Slave {slave_ns} heartbeat lost. Removing from active slaves.")
                    del self.active_slaves[slave_ns]

    def elect_new_master(self):
        """
        Elegge un nuovo master dagli slave attivi.
        Lo slave con la priorità più alta (ad esempio, il namespace più piccolo) diventa il nuovo master.
        """
        with self.lock:
            candidates = list(self.active_slaves.keys()) + [self.robot_namespace]

        if not candidates:
            self.get_logger().error(f"[{self.robot_namespace}] No candidates available for master election.")
            return

        # Ordina i candidati in base al namespace (assumendo che l'ordine lessicografico dia priorità)
        candidates_sorted = sorted(candidates)

        # Il primo candidato nella lista ordinata diventa il nuovo master
        new_master = candidates_sorted[0]

        if new_master == self.robot_namespace:
            self.get_logger().info(f"[{self.robot_namespace}] Elected as the new master.")
            self.become_master()
        else:
            self.get_logger().info(f"[{self.robot_namespace}] New master is {new_master}.")

    def become_master(self):
        """
        Trasforma questo slave nel master, eseguendo tutte le funzioni del master.
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] Now acting as the master.")

        # Pubblica il grafo di navigazione sul topic '/navigation_graph'
        if self.navigation_graph is not None:
            self.publish_navigation_graph()
            self.get_logger().info(f"[{self.robot_namespace}] Published navigation graph. Starting partitioning and waypoint assignment.")
            self.partition_and_assign_waypoints()
        else:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot become master.")

    def publish_navigation_graph(self):
        """
        Pubblica il grafo di navigazione sul topic '/navigation_graph'.
        """
        graph_data = {
            'nodes': [
                {'label': node, 'x': data['x'], 'y': data['y'], 'orientation': data.get('orientation', 0.0)}
                for node, data in self.navigation_graph.nodes(data=True)
            ],
            'edges': [
                {'from': u, 'to': v, 'weight': data.get('weight', 1.0)}
                for u, v, data in self.navigation_graph.edges(data=True)
            ]
        }
        msg = String()
        msg.data = json.dumps(graph_data)
        self.graph_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published navigation graph.")

    def navigation_graph_callback(self, msg):
        """
        Callback attivato quando viene ricevuto un messaggio di grafo di navigazione.
        Memorizza i dati del grafo.
        """
        try:
            graph_data = json.loads(msg.data)
            self.navigation_graph = self.load_full_graph_from_data(graph_data)
            self.get_logger().debug(f"[{self.robot_namespace}] Received navigation graph.")
            if self.is_master and not self.partitioning_done:
                self.partition_and_assign_waypoints()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation graph: {e}")

    def navigation_commands_callback(self, msg):
        """
        Callback attivato quando viene ricevuto un nuovo waypoint dal master.
        Simula la navigazione verso il waypoint specificato.
        """
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation command: {e}")
            return

        # Verifica e converte l'orientamento se necessario
        if isinstance(waypoint_data.get('orientation'), str):
            waypoint_data['orientation'] = self.orientation_conversion(waypoint_data['orientation'])

        self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")

        # Avvia la simulazione della navigazione in un thread separato per non bloccare il nodo
        threading.Thread(target=self.simulate_navigation, args=(waypoint_data,)).start()

    def simulate_navigation(self, waypoint):
        """
        Simula la navigazione verso un waypoint assegnato.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']
        orientation_rad = waypoint['orientation']  # Ora già in radianti

        # Log del compito di navigazione
        self.get_logger().info(f"[{self.robot_namespace}] Simulating navigation to {label} at ({x}, {y}) with orientation {orientation_rad} radians.")

        # Simula il tempo di navigazione (es. 5 secondi)
        simulated_navigation_time = 1.0
        time.sleep(simulated_navigation_time)

        # Simula il risultato della navigazione
        nav_success = True  # Cambia questo valore per simulare errori

        if nav_success:
            # Navigazione riuscita
            self.get_logger().info(f"[{self.robot_namespace}] Reached {label} in {simulated_navigation_time} seconds.")
            self.publish_status("reached", "", simulated_navigation_time, label)
            # Avanza al prossimo waypoint se presente
            with self.lock:
                self.current_waypoint_index += 1
            # Libera il nodo dopo aver raggiunto il waypoint
            with self.lock:
                if label in self.occupied_nodes:
                    self.occupied_nodes.remove(label)
                    self.get_logger().info(f"[{self.robot_namespace}] Node {label} is now free.")
        else:
            # Navigazione fallita
            error_message = f"Simulation of navigation to {label} failed."
            self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
            self.publish_status("error", error_message, simulated_navigation_time, label)

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        """
        Pubblica lo stato della navigazione al master.
        """
        status_data = {
            'robot_namespace': self.robot_namespace,
            'status': status,
            'error_message': error_message,
            'time_taken': time_taken,
            'current_waypoint': current_waypoint
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Published status: {status_data}")

    def orientation_conversion(self, orientation_input):
        """
        Converte un orientamento stringa o float in radianti.

        Args:
            orientation_input (str or float): Orientamento come stringa ('NORTH', 'EAST', 'SOUTH', 'WEST') o float in radianti.

        Returns:
            float: Orientamento in radianti.
        """
        if isinstance(orientation_input, str):
            orientation_map = {
                "NORTH": 0.0,
                "EAST": -math.pi / 2,
                "SOUTH": math.pi,
                "WEST": math.pi / 2
            }
            return orientation_map.get(orientation_input.upper(), 0.0)
        elif isinstance(orientation_input, (float, int)):
            # Se è già un float, lo ritorna direttamente
            return float(orientation_input)
        else:
            # Valore di default se il tipo non è riconosciuto
            return 0.0

    def load_full_graph_from_data(self, graph_data):
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

    def partition_and_assign_waypoints(self):
        """
        Partiziona il grafo di navigazione e assegna i waypoint agli slave.
        Questo metodo viene chiamato se lo slave assume il ruolo di master.
        """
        if self.navigation_graph is None:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot partition and assign waypoints.")
            return

        # Simulazione della partizione del grafo
        # In un vero scenario, utilizzeresti una funzione di partizione come `partition_graph`
        # Qui, per la simulazione, assegniamo waypoint predefiniti

        # Esempio di assegnazione di un waypoint predefinito a se stesso
        simulated_waypoints = [
            {'label': 'node_16', 'x': 17.192245, 'y': -8.623547000000002, 'orientation': self.initial_orientation_str},
            {'label': 'node_5', 'x': -2.3077550000000002, 'y': -0.5235470000000007, 'orientation': self.initial_orientation_str},
        ]

        self.assigned_waypoints = simulated_waypoints
        self.assign_next_waypoint()

    def assign_next_waypoint(self):
        """
        Assegna il prossimo waypoint nella coda allo slave.
        """
        if self.current_waypoint_index < len(self.assigned_waypoints):
            waypoint = self.assigned_waypoints[self.current_waypoint_index]
            node_label = waypoint['label']

            if node_label in self.occupied_nodes:
                self.get_logger().warn(f"[{self.robot_namespace}] Node {node_label} is already occupied. Cannot assign. Putting in waiting state.")
                # In un simulatore, non ci sono altri slave, quindi rimane in attesa
                return

            # Assegna il waypoint
            self.occupied_nodes.add(node_label)
            self.get_logger().info(f"[{self.robot_namespace}] Assigned waypoint: {waypoint}")
            self.publish_navigation_command(waypoint)
        else:
            self.get_logger().info(f"All waypoints have been assigned to {self.robot_namespace}. Route completed.")

    def publish_navigation_command(self, waypoint):
        """
        Pubblica un comando di navigazione allo slave (simulatore stesso).
        """
        msg = String()
        msg.data = json.dumps(waypoint)
        # Simula la ricezione del messaggio chiamando direttamente il callback
        self.navigation_commands_callback(msg)

    def print_subgraphs(self, subgraphs):
        """
        Stampa i dettagli di ciascun sottografo dopo la partizione.

        Args:
            subgraphs (list of nx.Graph): Lista di sottografi risultanti dalla partizione.
        """
        self.get_logger().info("----- Subgraphs After Partition -----")
        for idx, subgraph in enumerate(subgraphs):
            self.get_logger().info(f"Subgraph {idx+1}:")
            self.get_logger().info(f"  Nodes ({len(subgraph.nodes())}):")
            for node, data in subgraph.nodes(data=True):
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                orientation = data.get('orientation', 0.0)
                self.get_logger().info(f"    {node}: Position=({x}, {y}), Orientation={orientation} radians")
            self.get_logger().info(f"  Edges ({len(subgraph.edges())}):")
            for u, v, data in subgraph.edges(data=True):
                weight = data.get('weight', 1.0)
                self.get_logger().info(f"    From {u} to {v}, Weight: {weight}")
        self.get_logger().info("----- End of Subgraphs -----")

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

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Slave Navigation Simulator Node')
    parser.add_argument('--robot_namespace', type=str, default='robot_simulator', help='Robot namespace')
    parser.add_argument('--initial_x', type=float, default=0.0, help='Initial X position')
    parser.add_argument('--initial_y', type=float, default=0.0, help='Initial Y position')
    parser.add_argument('--initial_orientation', type=str, default='NORTH', help='Initial orientation (NORTH, EAST, SOUTH, WEST)')

    # ROS2 aggiunge argomenti propri; per evitare conflitti, dobbiamo ignorarli
    args, unknown = parser.parse_known_args()

    node = SlaveNavigationSimulator(
        robot_namespace=args.robot_namespace,
        initial_x=args.initial_x,
        initial_y=args.initial_y,
        initial_orientation_str=args.initial_orientation
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
