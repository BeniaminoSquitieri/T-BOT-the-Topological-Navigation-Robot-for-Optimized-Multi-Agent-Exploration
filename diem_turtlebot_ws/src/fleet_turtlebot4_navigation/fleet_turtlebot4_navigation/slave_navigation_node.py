#!/usr/bin/env python3

# Importazioni ROS 2 e Python di base
import rclpy  # Libreria Python per interfacciarsi con ROS 2
from rclpy.node import Node  # Classe base per creare nodi ROS 2 in Python
from std_msgs.msg import String  # Tipo di messaggio ROS standard per stringhe
import json  # Per la manipolazione di dati in formato JSON
import time  # Per ottenere timestamp, misurare intervalli di tempo, ecc.
import argparse  # Per la gestione degli argomenti da linea di comando
import math  # Funzioni matematiche (sin, cos, pi, etc.)
import networkx as nx  # Libreria per la gestione di grafi complessi (creazione, navigazione)
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult
# TurtleBot4Navigator è una classe custom che fornisce metodi per controllare il TurtleBot4
# e per verificare lo stato della navigazione.

# Importazione di funzioni custom per caricare e partizionare il grafo e per calcolare il percorso
from .graph_partitioning import load_full_graph, partition_graph
from .path_calculation import calculate_dcpp_route

def load_full_graph_from_data(graph_data):
    """
    Carica un grafo NetworkX da un dizionario contenente nodi e archi.

    Args:
        graph_data (dict): Dizionario con 'nodes' e 'edges' che rappresentano un grafo.

    Returns:
        nx.DiGraph: Il grafo diretto caricato con i nodi e gli archi.
    """
    G = nx.DiGraph()

    # Aggiunta dei nodi al grafo con attributi x, y, orientation
    for node in graph_data['nodes']:
        label = node['label']
        x = node['x']
        y = node['y']
        orientation = node.get('orientation', 0.0)
        G.add_node(label, x=x, y=y, orientation=orientation)

    # Aggiunta degli archi al grafo con relativo peso
    for edge in graph_data['edges']:
        u = edge['from']
        v = edge['to']
        weight = edge.get('weight', 1.0)
        G.add_edge(u, v, weight=weight)

    return G

class SlaveState:
    """
    Classe per gestire lo stato di uno slave. 
    In questo codice, gli slave secondari vengono gestiti, nel caso lo slave attuale diventasse master,
    potrebbe avere bisogno di tracciare anche altri slave.
    """

    def __init__(self, slave_ns):
        self.slave_ns = slave_ns
        self.assigned_waypoints = []  # Elenco dei waypoint assegnati a questo slave (se diventa master).
        self.current_waypoint_index = 0  # Indice del prossimo waypoint da raggiungere.
        self.last_seen_time = 0.0  # Ultima volta in cui abbiamo ricevuto un messaggio di heartbeat da questo slave.
        self.initial_x = None  # Posizione iniziale X dello slave.
        self.initial_y = None  # Posizione iniziale Y dello slave.
        self.initial_orientation = None  # Orientamento iniziale dello slave.
        self.publisher = None  # Publisher ROS per inviare messaggi a questo slave, se necessario.
        self.waiting = False  # Flag per indicare se lo slave è in attesa di un nuovo waypoint.

class SlaveNavigationNode(Node):
    """
    Nodo ROS 2 che rappresenta lo slave stesso.
    Questo nodo:
    - Attende di ricevere il grafo di navigazione per determinare la propria posizione iniziale.
    - Si registra con il master.
    - Riceve waypoint dal master e li esegue, inviando il proprio stato (es. reached, error).
    - Monitora la presenza del master via heartbeat.
    - Può diventare master se il master attuale viene perso e la procedura di rielezione lo elegge.
    """

    def __init__(self, robot_namespace, initial_node_label, initial_orientation_str):
        # robot_namespace: stringa unica per identificare lo slave (es. "robot_1")
        # initial_node_label: etichetta del nodo iniziale della mappa in cui si trova il robot.
        # initial_orientation_str: orientamento iniziale del robot (NORTH, EAST, SOUTH, WEST)
        self.robot_namespace = robot_namespace
        self.initial_node_label = initial_node_label
        self.initial_orientation_str = initial_orientation_str

        # Converte la stringa di orientamento in un valore in radianti
        self.initial_orientation = self.orientation_conversion(initial_orientation_str)

        # Inizializzazione del nodo ROS con il nome 'slave_navigation_node'
        super().__init__('slave_navigation_node')

        # Creazione dei Publisher e Subscriber base:
        # /slave_registration: per registrarsi al master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)
        # /slave_initial_positions: per inviare la propria posizione iniziale al master
        self.initial_position_publisher = self.create_publisher(String, '/slave_initial_positions', 10)
        # /navigation_commands: riceve i comandi di navigazione (waypoint) dal master
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.navigation_commands_callback, 10
        )
        # /navigation_status: invia il proprio stato di navigazione al master, riceve anche aggiornamenti
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)
        self.navigation_status_subscriber = self.create_subscription(
            String,
            '/navigation_status',
            self.navigation_status_callback,
            10
        )
        # /slave_heartbeat: invia heartbeat per segnalare che è attivo
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)
        # /master_heartbeat: riceve heartbeat dal master per verificare che sia attivo
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 2
        )
        # /slave_heartbeat: riceve heartbeat dagli altri slave
        self.slave_heartbeat_subscriber = self.create_subscription(
            String, '/slave_heartbeat', self.slave_heartbeat_callback, 10
        )
        # /navigation_graph: riceve il grafo di navigazione dal master
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.navigation_graph_callback, 10
        )

        # Timers:
        # Timer per pubblicare regolarmente la registrazione (ogni 1 secondo)
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        # Timer per pubblicare regolarmente l'heartbeat (ogni 1 secondo)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        # Timer per controllare se il master è ancora vivo (ogni 10 secondi)
        self.master_check_timer = self.create_timer(10.0, self.check_master_alive)
        # Timer per controllare se gli altri slave sono ancora vivi (ogni 2 secondi)
        self.slave_check_timer = self.create_timer(2.0, self.check_slave_alive)

        # Nota: non pubblichiamo subito la posizione iniziale perché non conosciamo ancora le coordinate (x,y).
        # Le ricaveremo dal grafo appena ricevuto.

        # Inizializzazione del TurtleBot4Navigator, uno strumento per navigare nel mondo.
        self.navigator = TurtleBot4Navigator()

        # Variabili per l'elezione del master
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 150.0  # Tempo di attesa prima di considerare il master disperso

        # Lista degli slave attivi (nel caso questo nodo diventi master)
        self.active_slaves = {}

        # Grafo di navigazione (inizialmente None, verrà impostato dopo la callback del grafo)
        self.navigation_graph = None

        # Publisher per comandi di navigazione degli altri slave (se diventa master)
        self.slave_command_publishers = {}

        # Posizioni iniziali degli altri slave (se diventa master)
        self.slave_initial_positions = {}

        # Flag per indicare se questo nodo è il master
        self.is_master = False
        self.master_graph_partitioned = False

        # Waypoints assegnati a se stesso se diventa master
        self.assigned_waypoints = []
        self.current_waypoint_index = 0

        # Variabili di navigazione corrente
        self.is_navigating = False
        self.current_goal_label = None
        self.current_goal_start_time = None

        # Timer per verificare regolarmente lo stato della navigazione (ogni 0.5 secondi)
        self.navigation_check_timer = self.create_timer(0.5, self.check_navigation_status)

        # Flag che indica se la posizione iniziale è stata pubblicata
        self.initial_position_published = False

        self.get_logger().info(
            f"[{self.robot_namespace}] Slave node initialized with initial node label '{self.initial_node_label}' and orientation {self.initial_orientation_str} ({self.initial_orientation} radians)."
        )

    def publish_registration(self):
        """
        Pubblica periodicamente un messaggio di registrazione al master, per fargli sapere che questo slave è attivo.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published registration.")

    def publish_initial_position(self):
        """
        Una volta note le coordinate (x, y) del nodo iniziale, pubblica la posizione iniziale al master.
        """
        if self.initial_x is None or self.initial_y is None:
            self.get_logger().error("Cannot publish initial position: coordinates not set.")
            return

        initial_position = {
            'robot_namespace': self.robot_namespace,
            'x': self.initial_x,
            'y': self.initial_y,
            'orientation': self.initial_orientation_str
        }
        msg = String()
        msg.data = json.dumps(initial_position)
        self.initial_position_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Published initial position: {initial_position}")
        self.initial_position_published = True

    def publish_heartbeat(self):
        """
        Pubblica periodicamente un messaggio di heartbeat per segnalare che lo slave è vivo.
        Il master si aspetta questi messaggi per sapere che lo slave non è andato offline.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published heartbeat.")

    def master_heartbeat_callback(self, msg):
        """
        Callback chiamata quando riceviamo un heartbeat dal master.
        Aggiorna il timestamp dell'ultimo heartbeat ricevuto.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received master heartbeat.")

    def slave_heartbeat_callback(self, msg):
        """
        Callback chiamata quando riceviamo un heartbeat da un altro slave.
        Se questo nodo diventa master, userà questa lista per monitorare gli altri slave.
        """
        slave_ns = msg.data.strip()
        current_time = time.time()
        if slave_ns != self.robot_namespace:
            # Se lo slave non è noto, lo aggiunge
            if slave_ns not in self.active_slaves:
                self.active_slaves[slave_ns] = SlaveState(slave_ns)
                self.get_logger().info(f"[{self.robot_namespace}] Detected new slave: {slave_ns}")
            self.active_slaves[slave_ns] = current_time
            self.get_logger().debug(f"[{self.robot_namespace}] Received heartbeat from slave {slave_ns}.")

    def check_master_alive(self):
        """
        Controlla se il master è vivo. Se non riceviamo heartbeat dal master entro heartbeat_timeout,
        avviamo la procedura di elezione di un nuovo master.
        """
        current_time = time.time()
        if self.master_alive:
            self.master_alive = False
        else:
            if current_time - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost. Initiating master election.")
                self.elect_new_master()

    def check_slave_alive(self):
        """
        Controlla se gli altri slave sono ancora vivi in base ai loro heartbeat.
        Rimuove quelli considerati persi.
        """
        current_time = time.time()
        for slave_ns in list(self.active_slaves.keys()):
            if current_time - self.active_slaves[slave_ns] > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Slave {slave_ns} heartbeat lost. Removing from active slaves.")
                del self.active_slaves[slave_ns]

    def elect_new_master(self):
        """
        Procedura di elezione di un nuovo master.
        Se il master è perso, tutti gli slave attivi (compreso questo) sono candidati.
        Quello con il namespace "minore" in ordine alfabetico diventa master.
        """
        candidates = list(self.active_slaves.keys()) + [self.robot_namespace]
        if not candidates:
            self.get_logger().error(f"[{self.robot_namespace}] No candidates available for master election.")
            return

        candidates_sorted = sorted(candidates)
        new_master = candidates_sorted[0]

        if new_master == self.robot_namespace:
            self.get_logger().info(f"[{self.robot_namespace}] Elected as the new master.")
            self.become_master()
        else:
            self.get_logger().info(f"[{self.robot_namespace}] New master is {new_master}.")

    def become_master(self):
        """
        Trasforma questo slave nel master.
        Come master, questo nodo dovrà:
        - Pubblicare il grafo
        - Partizionare il grafo e assegnare i waypoint
        - Monitorare gli slave
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] Now acting as the master.")
        self.available_informations()

        if self.navigation_graph is not None:
            self.publish_navigation_graph()
            self.partition_and_assign_waypoints()
        else:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot become master.")

    def available_informations(self):
        """
        Stampa informazioni sullo stato attuale: descrizione del grafo, slave disponibili.
        Utile per debug quando si diventa master.
        """
        self.get_logger().info("----- Available Information -----")
        self.get_logger().info("Graph description:")
        if self.navigation_graph is not None:
            for node, data in self.navigation_graph.nodes(data=True):
                self.get_logger().info(
                    f"  Node {node}: Position=({data['x']}, {data['y']}), Orientation={data.get('orientation',0.0)} radians"
                )
            for u, v, data in self.navigation_graph.edges(data=True):
                self.get_logger().info(f"  Edge from {u} to {v}, Weight: {data.get('weight',1.0)}")
        else:
            self.get_logger().info("No navigation graph available.")

        self.get_logger().info("Available slaves:")
        for slave_ns in self.active_slaves.keys():
            self.get_logger().info(f"  - {slave_ns}")

        self.get_logger().info("----- End of Available Information -----")

    def publish_navigation_graph(self):
        """
        Se questo nodo è master, pubblica il grafo di navigazione per informare tutti gli slave.
        """
        graph_msg = String()
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
        graph_msg.data = json.dumps(graph_data)
        self.status_publisher.publish(graph_msg)
        self.get_logger().info("Published navigation graph as master.")

    def navigation_graph_callback(self, msg):
        """
        Callback quando riceviamo il grafo di navigazione dal master.
        Una volta ricevuto, possiamo ricavare le coordinate del nodo iniziale e pubblicare la posizione iniziale.
        """
        try:
            graph_data = json.loads(msg.data)
            self.navigation_graph = load_full_graph_from_data(graph_data)
            # Ricaviamo le coordinate del nodo iniziale
            if self.initial_node_label in self.navigation_graph.nodes:
                node_data = self.navigation_graph.nodes[self.initial_node_label]
                self.initial_x = node_data['x']
                self.initial_y = node_data['y']
                if not self.initial_position_published:
                    self.publish_initial_position()

            # Se questo nodo è master e non abbiamo ancora partizionato il grafo, lo facciamo ora
            if self.is_master and not self.master_graph_partitioned:
                self.partition_and_assign_waypoints()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation graph: {e}")

    def navigation_commands_callback(self, msg):
        """
        Callback per i comandi di navigazione.
        Riceviamo un waypoint dal master e lo eseguiamo.
        """
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation command: {e}")
            return

        if isinstance(waypoint_data.get('orientation'), str):
            waypoint_data['orientation'] = self.orientation_conversion(waypoint_data['orientation'])

        self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")
        self.execute_navigation(waypoint_data)

    def execute_navigation(self, waypoint):
        """
        Esegue la navigazione verso il waypoint specificato usando il TurtleBot4Navigator.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']
        orientation_rad = waypoint['orientation']

        self.get_logger().info(f"[{self.robot_namespace}] Navigating to {label} at ({x}, {y}) with orientation {orientation_rad} radians.")

        goal_pose = self.navigator.getPoseStamped([x, y], orientation_rad)
        start_time = time.time()

        try:
            # Verifica se l'action server di navigazione è disponibile
            if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                error_message = f"Action server not available for {label}. Skipping this waypoint."
                self.get_logger().error(error_message)
                self.publish_status("error", error_message, 0.0, label)

                # Se siamo master, saltiamo il waypoint e passiamo al successivo
                if self.is_master:
                    self.get_logger().warn(f"[{self.robot_namespace}] Skipping waypoint {label} and moving to next.")
                    self.current_waypoint_index += 1
                    self.assign_next_waypoint_as_master()

                return

            # Inizia a navigare verso il goal
            self.navigator.startToPose(goal_pose)
        except Exception as e:
            error_message = f"Exception occurred while sending goal to {label}: {e}"
            self.get_logger().error(error_message)
            self.publish_status("error", error_message, 0.0, label)
            return

        # Attende il completamento della navigazione
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        time_taken = time.time() - start_time
        nav_result = self.navigator.getResult()

        # Controlla il risultato della navigazione
        if nav_result == TaskResult.SUCCEEDED:
            # Waypoint raggiunto
            self.get_logger().info(f"[{self.robot_namespace}] Reached {label} in {time_taken:.2f} seconds.")
            self.publish_status("reached", "", time_taken, label)
            self.current_waypoint_index += 1
            if self.is_master:
                self.assign_next_waypoint_as_master()
        else:
            # Navigazione fallita
            error_message = f"Navigation to {label} failed with result code {nav_result}."
            self.get_logger().error(error_message)
            self.publish_status("error", error_message, time_taken, label)

    def check_navigation_status(self):
        # Funzione placeholder per eventuali controlli sullo stato della navigazione non direttamente correlati.
        pass

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        """
        Pubblica lo stato della navigazione per informare il master sul risultato dell'ultimo waypoint.
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

    def navigation_status_callback(self, msg):
        """
        Callback per la ricezione dello stato di navigazione, se lo slave diventa master dovrà gestire
        anche gli stati degli altri slave.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_waypoint = data['current_waypoint']
            time_taken = data['time_taken']
            error_message = data.get('error_message', '')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"[{self.robot_namespace}] Invalid navigation status message: {e}")
            return

        current_time = time.time()

        if self.is_master:
            # Se è master, deve gestire gli stati degli altri slave (o di se stesso come master)
            # Nota: c'è un riferimento a self.lock che nel codice finale potrebbe non essere presente,
            # se non si utilizza multithreading. Se tolto self.lock in precedenza, si ignora questa parte.
            # Comunque lo scopo era sincronizzare l'accesso a self.active_slaves e strutture correlate.
            if slave_ns in self.active_slaves:
                slave = self.active_slaves[slave_ns]
            elif slave_ns == self.robot_namespace:
                slave = self
            else:
                self.get_logger().warn(f"[{self.robot_namespace}] Received status from unknown slave {slave_ns}.")
                return

            slave.last_seen_time = current_time

            if status == "reached":
                # Lo slave ha raggiunto un waypoint, libera il nodo se occupato e assegna il successivo
                if current_waypoint in self.occupied_nodes:
                    self.occupied_nodes.remove(current_waypoint)
                    self.get_logger().info(f"[{self.robot_namespace}] Node {current_waypoint} is now free.")
                else:
                    self.get_logger().warn(f"[{self.robot_namespace}] Node {current_waypoint} was not marked as occupied.")

                self.get_logger().info(f"[{self.robot_namespace}] Slave {slave_ns} has reached waypoint {current_waypoint}.")
                slave.waiting = False
                self.assign_next_waypoint(slave_ns)
                self.assign_waiting_slaves()

            elif status == "error":
                # Lo slave ha incontrato un errore
                self.get_logger().error(f"[{self.robot_namespace}] Slave {slave_ns} encountered an error: {error_message}")
                if current_waypoint in self.occupied_nodes:
                    self.occupied_nodes.remove(current_waypoint)
                    self.get_logger().info(f"[{self.robot_namespace}] Node {current_waypoint} is now free due to error.")
                if slave_ns in self.active_slaves:
                    del self.active_slaves[slave_ns]
                    self.get_logger().warn(f"[{self.robot_namespace}] Removing slave {slave_ns} due to error.")
                    self.partition_and_assign_waypoints()

        else:
            # Se non è master, non deve gestire lo stato di altri slave
            pass

    def print_subgraphs(self, subgraphs):
        # Funzione per stampare i sottografi dopo la partizione, utile se diventa master.
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

    def orientation_conversion(self, orientation_input):
        """
        Converte l'orientamento da stringa a radianti:
        - NORTH: 0.0 rad
        - EAST: -pi/2 rad
        - SOUTH: pi rad
        - WEST: pi/2 rad
        """
        orientations = {
            'NORTH': 0.0,
            'EAST': -math.pi / 2,
            'SOUTH': math.pi,
            'WEST': math.pi / 2
        }
        if isinstance(orientation_input, str):
            return orientations.get(orientation_input.upper(), 0.0)
        elif isinstance(orientation_input, (float, int)):
            return float(orientation_input)
        else:
            return 0.0

    def partition_and_assign_waypoints(self):
        """
        Se questo nodo diventa master, partiziona il grafo e assegna i waypoint.
        Stessa logica del master. Se rimane slave, potrebbe non essere mai chiamata.
        """
        if self.navigation_graph is None:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot partition and assign waypoints.")
            return

        full_graph = self.navigation_graph
        # All_slaves: tutti gli slave attivi + questo
        all_slaves = list(self.active_slaves.keys()) + [self.robot_namespace]
        all_slaves_sorted = sorted(all_slaves)
        num_slaves = len(all_slaves_sorted)

        start_positions = []
        for slave_ns in all_slaves_sorted:
            if slave_ns == self.robot_namespace:
                start_positions.append({'x': self.initial_x, 'y': self.initial_y})
            else:
                if slave_ns in self.slave_initial_positions:
                    pos = self.slave_initial_positions[slave_ns]
                    start_positions.append({'x': pos['x'], 'y': pos['y']})
                else:
                    self.get_logger().warn(
                        f"[{self.robot_namespace}] Initial position for slave '{slave_ns}' not available. Using master position as fallback."
                    )
                    start_positions.append({'x': self.initial_x, 'y': self.initial_y})

        try:
            subgraphs = partition_graph(full_graph, num_slaves, start_positions=start_positions)
            self.get_logger().info(
                f"[{self.robot_namespace}] Partitioned the graph into {len(subgraphs)} subgraphs."
            )
        except ValueError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to partition graph: {e}")
            return

        for idx, slave_ns in enumerate(all_slaves_sorted):
            subgraph = subgraphs[idx]
            waypoints = self.extract_waypoints(subgraph)

            if slave_ns == self.robot_namespace:
                # Assegna il percorso a se stesso come master
                self.assign_route_to_master(waypoints)
            else:
                # Crea un publisher per lo slave se non esiste
                if slave_ns not in self.slave_command_publishers:
                    topic_name = f"/{slave_ns}/navigation_commands"
                    publisher = self.create_publisher(String, topic_name, 10)
                    self.slave_command_publishers[slave_ns] = publisher
                    self.get_logger().info(
                        f"[{self.robot_namespace}] Created publisher for slave '{slave_ns}' on topic '{topic_name}'."
                    )

                publisher = self.slave_command_publishers[slave_ns]

                # Assegna tutti i waypoint allo slave
                for waypoint in waypoints:
                    if isinstance(waypoint['orientation'], str):
                        orientation_rad = self.orientation_conversion(waypoint['orientation'])
                    else:
                        orientation_rad = waypoint['orientation']

                    waypoint_msg = {
                        'label': waypoint['label'],
                        'x': waypoint['x'],
                        'y': waypoint['y'],
                        'orientation': orientation_rad
                    }
                    msg = String()
                    msg.data = json.dumps(waypoint_msg)
                    publisher.publish(msg)
                    self.get_logger().info(f"[{self.robot_namespace}] Assigned waypoint to {slave_ns}: {waypoint_msg}")

        self.master_graph_partitioned = True

    def assign_route_to_master(self, waypoints):
        """
        Assegna a se stesso, in quanto master, il percorso DCPP calcolato.
        """
        dcpp_route = calculate_dcpp_route(waypoints, self.navigation_graph, self.get_logger())
        self.assigned_waypoints = dcpp_route.copy()
        self.current_waypoint_index = 0

        self.get_logger().info(f"[{self.robot_namespace}] DCPP route assigned as master:")
        for wp in dcpp_route:
            self.get_logger().info(
                f" - {wp['label']} at ({wp['x']}, {wp['y']}) with orientation {wp['orientation']} radians"
            )

        self.assign_next_waypoint_as_master()
        self.get_logger().info(f"[{self.robot_namespace}] Assigned {len(dcpp_route)} waypoints as master.")

    def assign_next_waypoint_as_master(self):
        """
        Assegna il prossimo waypoint al master stesso.
        Se i waypoint finiscono, ricomincia da capo.
        """
        if not self.assigned_waypoints:
            self.get_logger().warn(f"[{self.robot_namespace}] No waypoints assigned. Cannot assign next waypoint.")
            return

        if self.current_waypoint_index < len(self.assigned_waypoints):
            waypoint = self.assigned_waypoints[self.current_waypoint_index]
            self.execute_navigation(waypoint)
        else:
            self.get_logger().info(f"[{self.robot_namespace}] All waypoints have been assigned. Restarting the route.")
            self.current_waypoint_index = 0
            waypoint = self.assigned_waypoints[self.current_waypoint_index]
            self.execute_navigation(waypoint)

def main(args=None):
    """
    Punto d'entrata principale del programma.
    Gestione degli argomenti da linea di comando per specificare:
    - robot_namespace
    - initial_node_label
    - initial_orientation
    """
    parser = argparse.ArgumentParser(description='Slave Navigation Node using TurtleBot4 with Master Replacement')
    parser.add_argument('--robot_namespace', type=str, required=True, help='Unique namespace of the robot (e.g., robot_1)')
    parser.add_argument('--initial_node_label', type=str, required=True, help='Initial node label (e.g., node_5)')
    parser.add_argument('--initial_orientation', type=str, required=True, help='Initial orientation (NORTH, EAST, SOUTH, WEST)')

    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=None)

    node = SlaveNavigationNode(
        robot_namespace=parsed_args.robot_namespace,
        initial_node_label=parsed_args.initial_node_label,
        initial_orientation_str=parsed_args.initial_orientation
    )

    try:
        # Avvia il nodo e rimane in ascolto finché non viene interrotto
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # All'uscita, distrugge il nodo e chiude ROS 2
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
