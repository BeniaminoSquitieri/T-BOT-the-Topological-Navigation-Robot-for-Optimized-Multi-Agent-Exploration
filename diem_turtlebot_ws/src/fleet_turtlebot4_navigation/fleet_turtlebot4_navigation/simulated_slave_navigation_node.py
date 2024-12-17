#!/usr/bin/env python3

import rclpy  # The ROS 2 Python client library
from rclpy.node import Node  # Base class for creating a ROS 2 node in Python
from std_msgs.msg import String  # Standard ROS message type for transmitting string data
import json  # For encoding/decoding JSON messages (graph data, statuses, positions)
import time  # For timestamps and measuring intervals (e.g., last heartbeat time)
import argparse  # For parsing command-line arguments passed at node startup
import math  # For mathematical operations (e.g., pi, trigonometric functions)
import threading  # For running certain tasks (like simulated navigation) in parallel threads
import networkx as nx  # For creating and handling complex graphs (nodes, edges, attributes)

# Placeholder implementations for the helper functions
def partition_graph(graph, num_partitions, start_positions=None):
    # Implementa la logica di partizionamento del grafo
    # Questo è solo un esempio fittizio
    subgraphs = []
    nodes = list(graph.nodes())
    partition_size = max(1, len(nodes) // num_partitions)
    for i in range(num_partitions):
        subgraph_nodes = nodes[i*partition_size:(i+1)*partition_size]
        subgraph = graph.subgraph(subgraph_nodes).copy()
        subgraphs.append(subgraph)
    return subgraphs

def calculate_dcpp_route(waypoints, subgraph, logger):
    # Implementa la logica per calcolare la rotta DCPP
    # Questo è solo un esempio fittizio
    return waypoints

def orientation_rad_to_str(orientation_rad):
    # Converti radianti in stringhe orientamento
    orientation_map = {
        0.0: "NORTH",
        -math.pi / 2: "EAST",
        math.pi: "SOUTH",
        math.pi / 2: "WEST"
    }
    return orientation_map.get(orientation_rad, "UNKNOWN")

class SlaveState:
    """
    Questa classe memorizza lo stato di un singolo robot slave dal punto di vista del master.

    Attributes:
        slave_ns (str): Il namespace unico dello slave (es. "robot_1").
        assigned_waypoints (list): Lista di waypoints assegnati a questo slave.
        current_waypoint_index (int): Indica quale waypoint lo slave sta attualmente seguendo.
        last_seen_time (float): Timestamp dell'ultima comunicazione (es. heartbeat) da questo slave.
        initial_x (float): Coordinata X iniziale dello slave (una volta nota).
        initial_y (float): Coordinata Y iniziale dello slave (una volta nota).
        initial_orientation (str): Orientamento iniziale in stringa (es. "NORTH").
        publisher: Publisher ROS per inviare comandi di navigazione a questo slave.
        waiting (bool): Flag che indica se questo slave sta aspettando un waypoint disponibile.
    """
    def __init__(self, slave_ns):
        self.slave_ns = slave_ns
        self.assigned_waypoints = []
        self.current_waypoint_index = 0
        self.last_seen_time = 0.0
        self.initial_x = None
        self.initial_y = None
        self.initial_orientation = None
        self.publisher = None
        self.waiting = False

class SlaveNavigationSimulator(Node):
    """
    Un nodo slave simulato, che imita il comportamento di un vero robot TurtleBot4.

    Questo nodo:
    - Si registra al master per segnalare la propria presenza.
    - Attende il grafo di navigazione dal master e determina le proprie coordinate iniziali.
    - Pubblica la posizione iniziale una volta note le coordinate.
    - Riceve comandi di navigazione (waypoints) e simula la navigazione.
    - Pubblica lo stato della navigazione (raggiunto, errore) al master.
    - Invia e riceve messaggi di heartbeat per monitorare la presenza di altri slave e del master.
    - Se il master viene perso, partecipa a un'elezione per diventare il nuovo master.
    - Se diventa master, partiziona il grafo di navigazione e assegna waypoints a tutti gli slave.

    Inoltre, ora implementa la pubblicazione continua della posizione iniziale anche dopo l'assunzione del ruolo di master.
    """

    def __init__(self, robot_namespace, initial_node_label, initial_orientation_str):
        # Memorizza i parametri forniti:
        # robot_namespace: nome unico per questo robot (es. "robot_simulator")
        # initial_node_label: etichetta del nodo nel grafo di navigazione dove il robot inizia
        # initial_orientation_str: orientamento iniziale come stringa (NORTH, EAST, SOUTH, WEST)
        self.robot_namespace = robot_namespace
        self.initial_node_label = initial_node_label
        self.initial_orientation_str = initial_orientation_str

        # Converti la stringa di orientamento in radianti per la rappresentazione interna
        self.initial_orientation = self.orientation_conversion(initial_orientation_str)

        # All'inizio, non conosciamo le coordinate iniziali (x, y) poiché abbiamo solo l'etichetta del nodo
        self.initial_x = None
        self.initial_y = None

        # Inizializza il nodo ROS con il namespace fornito
        super().__init__('slave_navigation_simulator_node', namespace=self.robot_namespace)

        # Crea i publisher e i subscriber necessari per la comunicazione con il master e gli altri slave:

        # Publisher per registrarsi al master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)

        # Publisher per inviare la posizione iniziale una volta nota
        self.initial_position_publisher = self.create_publisher(String, '/slave_initial_positions', 10)

        # Subscriber per ricevere comandi di navigazione (waypoints) dal master
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.navigation_commands_callback, 10
        )

        # Publisher per inviare aggiornamenti di stato della navigazione al master
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Publisher per inviare messaggi di heartbeat indicando che questo slave è attivo
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)

        # Subscriber per ricevere messaggi di heartbeat dal master
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 10
        )

        # Subscriber per ricevere messaggi di heartbeat da altri slave
        self.slave_heartbeat_subscriber = self.create_subscription(
            String, '/slave_heartbeat', self.slave_heartbeat_callback, 10
        )

        # Subscriber per ricevere il grafo di navigazione dal master
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.navigation_graph_callback, 10
        )

        # Publisher per inviare il grafo di navigazione se questo nodo diventa master
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)

        # Subscriber per ricevere le posizioni iniziali degli slave (NUOVA FUNZIONALITÀ)
        self.slave_initial_position_subscriber = self.create_subscription(
            String, '/slave_initial_positions', self.slave_initial_position_callback, 10
        )

        # Crea i timer per le attività periodiche:

        # 1. Pubblica la registrazione periodicamente
        self.registration_timer = self.create_timer(1.0, self.publish_registration)

        # 2. Pubblica l'heartbeat periodicamente
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # 3. Prova a pubblicare la posizione iniziale ogni 2 secondi fino al successo
        self.initial_position_timer = self.create_timer(2.0, self.try_publish_initial_position)

        # 4. Pubblica periodicamente la posizione iniziale
        self.periodic_position_publisher_timer = self.create_timer(5.0, self.periodic_position_publish)

        # Variabili per rilevare la presenza del master:
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 5.0  # Dopo 5 secondi senza heartbeat del master, considera il master perso

        # Dizionario degli slave attivi. Chiave: namespace dello slave, Valore: istanza di SlaveState
        self.active_slaves = {}

        # Il grafo di navigazione sarà impostato una volta ricevuto
        self.navigation_graph = None

        # Set dei nodi occupati, usato se assegniamo waypoints e consideriamo i nodi "occupati"
        self.occupied_nodes = set()

        # Flag per indicare se abbiamo già partizionato il grafo e assegnato waypoints
        self.partitioning_done = False

        # Indice del waypoint corrente per questo slave (se agisce come master o ha una rotta assegnata)
        self.current_waypoint_index = 0

        # Flag per tracciare se la posizione iniziale è stata pubblicata
        self.initial_position_published = False

        # Flag per indicare se questo nodo è il master
        self.is_master = False

        # Lock per gestire l'accesso concorrente alle strutture dati condivise come self.active_slaves
        self.lock = threading.Lock()

        # Subscriber per lo stato di navigazione (necessario se diventa master)
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )

        # Publisher per i master heartbeat (inizializzato a None; sarà creato quando diventa master)
        self.master_heartbeat_publisher = None  # # MODIFICHE

        # Timer per la pubblicazione dei master heartbeat (inizializzato a None; sarà creato quando diventa master)
        self.master_heartbeat_timer = None  # # MODIFICHE

        # Log delle informazioni di inizializzazione
        self.get_logger().info(
            f"[{self.robot_namespace}] Slave simulator inizializzato con etichetta nodo iniziale '{self.initial_node_label}' "
            f"e orientamento {self.initial_orientation_str} ({self.initial_orientation} radianti)."
        )

        # Timer per controllare la salute del master e degli slave:
        # Controlla se il master è ancora attivo ogni 10 secondi
        self.master_check_timer = self.create_timer(10.0, self.check_master_alive)

        # Controlla gli heartbeat degli altri slave ogni 2 secondi
        self.slave_check_timer = self.create_timer(2.0, self.check_slave_alive)

    def try_publish_initial_position(self):
        if self.initial_x is not None and self.initial_y is not None and not self.initial_position_published:
            # Ora conosciamo x e y, e non abbiamo ancora pubblicato la posizione iniziale
            initial_position = {
                'robot_namespace': self.robot_namespace,
                'x': self.initial_x,
                'y': self.initial_y,
                'orientation': self.initial_orientation_str
            }
            msg = String()
            msg.data = json.dumps(initial_position)
            self.initial_position_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Posizione iniziale pubblicata: {initial_position}")
            self.initial_position_published = True
            # Annulla il timer in modo da non continuare a tentare
            self.initial_position_timer.cancel()

            # Log di conferma
            self.get_logger().info(f"[{self.robot_namespace}] Posizione iniziale pubblicata correttamente.")
        else:
            self.get_logger().debug(f"[{self.robot_namespace}] In attesa che la posizione iniziale sia impostata.")

    def periodic_position_publish(self):
        """
        Pubblica periodicamente la posizione iniziale su '/slave_initial_positions'.
        Questo assicura che la posizione venga aggiornata regolarmente, anche dopo l'elezione a master.
        """
        if self.initial_x is not None and self.initial_y is not None:
            orientation_str = orientation_rad_to_str(self.initial_orientation)
            position = {
                'robot_namespace': self.robot_namespace,
                'x': self.initial_x,
                'y': self.initial_y,
                'orientation': orientation_str
            }
            msg = String()
            msg.data = json.dumps(position)
            self.initial_position_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Posizione iniziale pubblicata periodicamente: {position}")

    def publish_registration(self):
        """
        Pubblica periodicamente un messaggio di registrazione al master.
        Questo assicura che il master sappia che questo slave è attivo e disponibile.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Registrazione pubblicata.")

    def publish_heartbeat(self):
        """
        Pubblica periodicamente un messaggio di heartbeat per segnalare che questo slave è vivo.
        Il master e altri slave tracciano questi messaggi per rilevare se uno slave va offline.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Heartbeat pubblicato.")

    def master_heartbeat_callback(self, msg):
        """
        Callback invocata quando viene ricevuto un heartbeat dal master.
        Segna il master come vivo e aggiorna il timestamp dell'ultimo heartbeat.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Heartbeat del master ricevuto.")

    def slave_heartbeat_callback(self, msg):
        """
        Callback invocata quando viene ricevuto un heartbeat da un altro slave.
        Aggiorna il record degli slave attivi e il loro last_seen_time.
        Questo è utile se dobbiamo diventare master in seguito.
        """
        slave_ns = msg.data.strip()
        current_time = time.time()
        if slave_ns != self.robot_namespace:
            with self.lock:
                if slave_ns not in self.active_slaves:
                    self.active_slaves[slave_ns] = SlaveState(slave_ns)
                    self.get_logger().info(f"[{self.robot_namespace}] Slave rilevato: {slave_ns}")
                self.active_slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"[{self.robot_namespace}] Heartbeat ricevuto dallo slave {slave_ns}.")

    def slave_initial_position_callback(self, msg):
        """
        Callback per ricevere le posizioni iniziali degli slave.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            x = data['x']
            y = data['y']
            orientation = data['orientation']
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"[{self.robot_namespace}] Messaggio di posizione iniziale slave non valido: {e}")
            return

        # Ignora la posizione iniziale del master se pubblicata su /slave_initial_positions
        if slave_ns == self.robot_namespace:
            return

        with self.lock:
            if slave_ns not in self.active_slaves:
                self.active_slaves[slave_ns] = SlaveState(slave_ns)
                self.get_logger().info(f"[{self.robot_namespace}] Slave rilevato: {slave_ns}")
            slave = self.active_slaves[slave_ns]
            slave.initial_x = x
            slave.initial_y = y
            slave.initial_orientation = orientation
            self.get_logger().debug(f"[{self.robot_namespace}] Posizione iniziale aggiornata per {slave_ns}: ({x}, {y}, {orientation})")

    def check_master_alive(self):
        """
        Controlla periodicamente se il master è ancora vivo.
        Se non riceve un heartbeat del master entro il timeout, avvia un'elezione per un nuovo master.
        """
        current_time = time.time()
        if self.master_alive:
            # Resetta il flag; verrà impostato di nuovo se arriva un nuovo heartbeat
            self.master_alive = False
        else:
            # Nessun heartbeat dall'ultimo controllo
            if current_time - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Heartbeat del master perso. Avvio elezione master.")
                self.elect_new_master()

    def check_slave_alive(self):
        """
        Controlla periodicamente gli heartbeat degli altri slave.
        Rimuove quelli che non hanno comunicato entro il timeout.
        """
        current_time = time.time()
        with self.lock:
            for slave_ns in list(self.active_slaves.keys()):
                if current_time - self.active_slaves[slave_ns].last_seen_time > self.heartbeat_timeout:
                    self.get_logger().warn(f"[{self.robot_namespace}] Heartbeat dello slave {slave_ns} perso. Rimozione dallo stato attivo.")
                    del self.active_slaves[slave_ns]

    def elect_new_master(self):
        """
        Se il master è perso, elegge un nuovo master tra tutti gli slave attivi e questo nodo.
        Colui con il namespace lexicograficamente più piccolo vince.
        """
        with self.lock:
            candidates = list(self.active_slaves.keys()) + [self.robot_namespace]

        if not candidates:
            self.get_logger().error(f"[{self.robot_namespace}] Nessun candidato disponibile per l'elezione del master.")
            return

        candidates_sorted = sorted(candidates)
        new_master = candidates_sorted[0]

        if new_master == self.robot_namespace:
            self.get_logger().info(f"[{self.robot_namespace}] Eletto come nuovo master.")
            self.become_master()
        else:
            self.get_logger().info(f"[{self.robot_namespace}] Il nuovo master è {new_master}.")

    def become_master(self):
        """
        Trasforma questo slave in master.
        Come master, pubblica il grafo di navigazione, partiziona il grafo e assegna waypoints a tutti gli slave.
        """
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] Ora agisco come master.")

        # # MODIFICHE: Smonta il subscriber al master heartbeat
        if self.master_heartbeat_subscriber:
            self.destroy_subscription(self.master_heartbeat_subscriber)
            self.master_heartbeat_subscriber = None
            self.get_logger().info(f"[{self.robot_namespace}] Smontato il subscriber a /master_heartbeat.")

        # # MODIFICHE: Crea il publisher per i master heartbeat
        self.master_heartbeat_publisher = self.create_publisher(String, '/master_heartbeat', 10)
        self.get_logger().info(f"[{self.robot_namespace}] Creato il publisher per /master_heartbeat.")

        # # MODIFICHE: Crea un timer per pubblicare i master heartbeat
        self.master_heartbeat_timer = self.create_timer(1.0, self.publish_master_heartbeat)
        self.get_logger().info(f"[{self.robot_namespace}] Avviato il timer per i master heartbeat.")

        if self.navigation_graph is not None:
            self.publish_navigation_graph()
            self.get_logger().info(f"[{self.robot_namespace}] Grafo di navigazione pubblicato. Inizio partizionamento e assegnazione dei waypoints.")
            self.partition_and_assign_waypoints()
        else:
            self.get_logger().error(f"[{self.robot_namespace}] Grafo di navigazione non disponibile. Impossibile diventare master.")

    def publish_master_heartbeat(self):
        """
        Pubblica messaggi di heartbeat su /master_heartbeat per segnalare che questo nodo è il master.
        """
        if self.is_master and self.master_heartbeat_publisher:
            heartbeat_msg = String()
            heartbeat_msg.data = self.robot_namespace  # Identificatore del master
            self.master_heartbeat_publisher.publish(heartbeat_msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Heartbeat del master pubblicato.")

    def publish_navigation_graph(self):
        """
        Se siamo il master, pubblica il grafo di navigazione su '/navigation_graph'.
        Questo permette a tutti gli slave di conoscere la disposizione dell'ambiente.
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
        self.get_logger().debug(f"[{self.robot_namespace}] Grafo di navigazione pubblicato.")

    def navigation_graph_callback(self, msg):
        """
        Callback quando riceviamo il grafo di navigazione dal master.
        Una volta ricevuto, possiamo determinare le coordinate iniziali dal nodo iniziale.
        """
        try:
            graph_data = json.loads(msg.data)
            self.navigation_graph = self.load_full_graph_from_data(graph_data)
            self.get_logger().debug(f"[{self.robot_namespace}] Grafo di navigazione ricevuto.")

            # Trova le coordinate dell'etichetta del nodo iniziale
            if self.initial_node_label in self.navigation_graph.nodes:
                node_data = self.navigation_graph.nodes[self.initial_node_label]
                self.initial_x = node_data['x']
                self.initial_y = node_data['y']
                # Non pubblichiamo la posizione iniziale qui; affidiamoci al timer che tenta periodicamente
                self.get_logger().info(f"[{self.robot_namespace}] Coordinate iniziali determinate: ({self.initial_x}, {self.initial_y})")

            # Se siamo già master e non abbiamo ancora partizionato, partizioniamo ora
            if self.is_master and not self.partitioning_done:
                self.partition_and_assign_waypoints()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Decodifica del grafo di navigazione fallita: {e}")

    def navigation_commands_callback(self, msg):
        """
        Callback quando riceviamo un comando di navigazione (waypoint).
        Analizziamo il waypoint e iniziamo a simulare la navigazione verso di esso.
        """
        self.get_logger().debug(f"[{self.robot_namespace}] Comando di navigazione ricevuto: {msg.data}")
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Decodifica del comando di navigazione fallita: {e}")
            return

        # Converti l'orientamento se fornito come stringa
        if isinstance(waypoint_data.get('orientation'), str):
            waypoint_data['orientation'] = self.orientation_conversion(waypoint_data['orientation'])

        # Inizia la simulazione della navigazione in un thread separato per evitare di bloccare il loop principale
        threading.Thread(target=self.simulate_navigation, args=(waypoint_data,)).start()
        self.get_logger().debug(f"[{self.robot_namespace}] Avviato il thread di navigazione per il waypoint: {waypoint_data}")

    def simulate_navigation(self, waypoint):
        """
        Simula la navigazione verso il waypoint dato.
        Per scopi di test, dormiamo per un tempo fisso e poi riportiamo successo.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']
        orientation_rad = waypoint['orientation']

        self.get_logger().info(f"[{self.robot_namespace}] Simulazione navigazione verso {label} a ({x}, {y}) con orientamento {orientation_rad} radianti.")

        # Simula il tempo di viaggio, ad esempio 5 secondi
        simulated_navigation_time = 5.0
        self.get_logger().debug(f"[{self.robot_namespace}] Inizio simulazione navigazione per {label} (durata {simulated_navigation_time} secondi).")
        time.sleep(simulated_navigation_time)

        # Assume successo nella navigazione per semplicità
        nav_success = True

        if nav_success:
            self.get_logger().info(f"[{self.robot_namespace}] Raggiunto {label} in {simulated_navigation_time} secondi.")
            self.publish_status("reached", "", simulated_navigation_time, label)
            # MODIFICHE: Pubblica la posizione aggiornata dopo aver raggiunto il waypoint
            self.update_and_publish_position(x, y, orientation_rad)
            if self.is_master:
                # Se siamo master, gestisci l'assegnazione del prossimo waypoint per te stesso
                with self.lock:
                    self.current_waypoint_index += 1
                with self.lock:
                    if label in self.occupied_nodes:
                        self.occupied_nodes.remove(label)
                        self.get_logger().info(f"[{self.robot_namespace}] Nodo {label} ora è libero.")
                # Assegna il prossimo waypoint
                self.assign_next_waypoint(self.robot_namespace)
                # Assegna waypoints agli slave in attesa
                self.assign_waiting_slaves()
        else:
            # Simula un caso di errore se necessario
            error_message = f"Simulazione della navigazione verso {label} fallita."
            self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
            self.publish_status("error", error_message, simulated_navigation_time, label)

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        """
        Pubblica lo stato della navigazione al master per informarlo del risultato dell'ultimo tentativo di navigazione.
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
        self.get_logger().info(f"[{self.robot_namespace}] Stato pubblicato: {status_data}")

    def update_and_publish_position(self, x, y, orientation_rad):
        """
        Aggiorna la posizione corrente e la pubblica sul topic '/slave_initial_positions'.
        """
        self.initial_x = x
        self.initial_y = y
        self.initial_orientation = orientation_rad

        orientation_str = orientation_rad_to_str(orientation_rad)

        position = {
            'robot_namespace': self.robot_namespace,
            'x': self.initial_x,
            'y': self.initial_y,
            'orientation': orientation_str
        }
        msg = String()
        msg.data = json.dumps(position)
        self.initial_position_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Posizione aggiornata pubblicata: {position}")

    def navigation_status_callback(self, msg):
        """
        Gestisce il feedback dello stato della navigazione dagli slave se diventiamo master.
        Aggiorna i loro stati e eventualmente riassegna waypoints.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_waypoint = data['current_waypoint']
            time_taken = data['time_taken']
            error_message = data.get('error_message', '')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"[{self.robot_namespace}] Messaggio di stato della navigazione non valido: {e}")
            return

        current_time = time.time()

        if self.is_master:
            with self.lock:
                if slave_ns in self.active_slaves:
                    slave = self.active_slaves[slave_ns]
                elif slave_ns == self.robot_namespace:
                    # È noi stessi (il master) che riportiamo lo stato
                    slave = self
                else:
                    self.get_logger().warn(f"[{self.robot_namespace}] Stato ricevuto da slave sconosciuto {slave_ns}.")
                    return

                slave.last_seen_time = current_time

                if status == "reached":
                    # Lo slave ha raggiunto un waypoint, libera il nodo e assegna il prossimo
                    if current_waypoint in self.occupied_nodes:
                        self.occupied_nodes.remove(current_waypoint)
                        self.get_logger().info(f"[{self.robot_namespace}] Nodo {current_waypoint} ora è libero.")
                    else:
                        self.get_logger().warn(f"[{self.robot_namespace}] Nodo {current_waypoint} non era marcato come occupato.")

                    self.get_logger().info(f"[{self.robot_namespace}] Slave {slave_ns} ha raggiunto il waypoint {current_waypoint}.")
                    slave.waiting = False
                    self.assign_next_waypoint(slave_ns)
                    self.assign_waiting_slaves()

                elif status == "error":
                    # Lo slave ha riscontrato un errore
                    self.get_logger().error(f"[{self.robot_namespace}] Slave {slave_ns} ha riscontrato un errore: {error_message}")
                    if current_waypoint in self.occupied_nodes:
                        self.occupied_nodes.remove(current_waypoint)
                        self.get_logger().info(f"[{self.robot_namespace}] Nodo {current_waypoint} ora è libero a causa di un errore.")
                    if slave_ns in self.active_slaves:
                        del self.active_slaves[slave_ns]
                        self.get_logger().warn(f"[{self.robot_namespace}] Rimosso lo slave {slave_ns} a causa di un errore.")
                        self.partition_and_assign_waypoints()

    def partition_and_assign_waypoints(self):
        """
        Partiziona il grafo di navigazione tra tutti gli slave attivi (e questo nodo se master).
        Assegna una rotta DCPP a ciascun slave.
        Questo viene fatto solo se siamo il master.
        """
        if self.navigation_graph is None:
            self.get_logger().error(f"[{self.robot_namespace}] Grafo di navigazione non disponibile. Impossibile partizionare e assegnare waypoints.")
            return

        with self.lock:
            num_slaves = len(self.active_slaves) + 1  # Include se stesso come master
            if num_slaves == 0:
                self.get_logger().warn("Nessuno slave attivo trovato. In attesa che gli slave si registrino.")
                self.partitioning_done = False
                return

            # Raccogli le posizioni iniziali di tutti gli slave attivi
            start_positions = []
            for slave in self.active_slaves.values():
                if slave.initial_x is not None and slave.initial_y is not None:
                    start_positions.append({'x': slave.initial_x, 'y': slave.initial_y})
                else:
                    self.get_logger().warn(f"Slave {slave.slave_ns} posizione iniziale non disponibile.")

            # Aggiungi la propria posizione iniziale
            if self.initial_x is None or self.initial_y is None:
                # Se non abbiamo ancora ricevuto il grafo o qualcosa non va, non possiamo partizionare
                self.get_logger().error("Posizione iniziale del master non disponibile, impossibile partizionare.")
                return

            start_positions.append({'x': self.initial_x, 'y': self.initial_y})

            if len(start_positions) != num_slaves:
                self.get_logger().error("Non tutti gli slave hanno posizioni iniziali valide.")
                return

            # Partiziona il grafo
            try:
                subgraphs = partition_graph(self.navigation_graph, num_slaves, start_positions=start_positions)
                self.get_logger().info(f"Grafo partizionato in {len(subgraphs)} subgraph.")
                self.print_subgraphs(subgraphs)
            except ValueError as e:
                self.get_logger().error(f"Partizionamento del grafo fallito: {e}")
                return

            # Crea una lista ordinata di tutti gli slave inclusi il master
            all_slaves = list(self.active_slaves.keys()) + [self.robot_namespace]
            all_slaves_sorted = sorted(all_slaves)

            if len(subgraphs) != len(all_slaves_sorted):
                self.get_logger().error("Numero di subgraph non corrisponde al numero di slave attivi.")
                return

            # Assegna una rotta DCPP a ciascun slave
            for idx, slave_ns in enumerate(all_slaves_sorted):
                subgraph = subgraphs[idx]
                waypoints = self.extract_waypoints(subgraph)
                dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.get_logger())
                ordered_route = dcpp_route

                self.get_logger().info(f"Rotta DCPP per {slave_ns}:")
                for wp in ordered_route:
                    self.get_logger().info(f"  {wp}")

                if slave_ns == self.robot_namespace:
                    # Assegna la rotta a se stesso
                    self.assigned_waypoints = ordered_route
                    self.assign_next_waypoint(self.robot_namespace)
                else:
                    # Assegna la rotta ad altri slave
                    if slave_ns in self.active_slaves:
                        slave = self.active_slaves[slave_ns]
                        slave.assigned_waypoints = ordered_route
                        self.assign_next_waypoint(slave_ns)
                    else:
                        self.get_logger().warn(f"Slave {slave_ns} non trovato in active_slaves.")

            self.partitioning_done = True

    def assign_next_waypoint(self, slave_ns):
        """
        Assegna il prossimo waypoint nella rotta assegnata allo slave dato.
        Se il nodo è occupato, lo slave attende finché non è libero.
        """
        with self.lock:
            if slave_ns == self.robot_namespace:
                # Assegna a se stesso
                slave = self
            else:
                slave = self.active_slaves.get(slave_ns, None)

            if slave is None:
                self.get_logger().warn(f"Slave {slave_ns} non trovato.")
                return

            if len(slave.assigned_waypoints) == 0:
                self.get_logger().warn(f"Nessun waypoint assegnato allo slave {slave_ns}.")
                return

            waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
            node_label = waypoint['label']

            if node_label in self.occupied_nodes:
                self.get_logger().warn(f"Nodo {node_label} è già occupato. Non posso assegnarlo allo slave {slave_ns}.")
                slave.waiting = True
                return

            waypoint_msg = {
                'label': waypoint['label'],
                'x': waypoint['x'],
                'y': waypoint['y'],
                'orientation': orientation_rad_to_str(waypoint['orientation'])
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)

            if slave_ns == self.robot_namespace:
                # Assegna a se stesso, simula la ricezione del comando direttamente
                self.navigation_commands_callback(msg)
                self.get_logger().info(f"[Master {self.robot_namespace}] Waypoint assegnato a se stesso: {waypoint_msg}")
            else:
                # Invia il waypoint allo slave
                if slave.publisher is None:
                    slave.publisher = self.create_publisher(String, f'/{slave_ns}/navigation_commands', 10)
                slave.publisher.publish(msg)
                self.get_logger().info(f"[Master {self.robot_namespace}] Waypoint assegnato a {slave_ns}: {waypoint_msg}")

            self.occupied_nodes.add(node_label)
            slave.current_waypoint_index += 1
            if slave.current_waypoint_index >= len(slave.assigned_waypoints):
                slave.current_waypoint_index = 0

    def assign_waiting_slaves(self):
        """
        Assegna waypoints agli slave che erano in attesa di un nodo libero.
        Tenta di assegnare nuovamente se il nodo è ora libero.
        """
        with self.lock:
            candidates = list(self.active_slaves.keys()) + [self.robot_namespace]
            for slave_ns in sorted(candidates):
                if slave_ns == self.robot_namespace and self.is_master:
                    slave = self
                else:
                    slave = self.active_slaves.get(slave_ns, None)
                    if slave is None:
                        continue

                if slave.waiting:
                    if len(slave.assigned_waypoints) == 0:
                        self.get_logger().warn(f"Nessun waypoint assegnato allo slave {slave_ns}.")
                        continue

                    waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
                    node_label = waypoint['label']

                    # Controlla se il nodo è ora libero
                    if node_label not in self.occupied_nodes:
                        # Nodo libero, assegnalo ora
                        waypoint_msg = {
                            'label': waypoint['label'],
                            'x': waypoint['x'],
                            'y': waypoint['y'],
                            'orientation': orientation_rad_to_str(waypoint['orientation'])
                        }
                        msg = String()
                        msg.data = json.dumps(waypoint_msg)

                        if slave_ns == self.robot_namespace:
                            self.navigation_commands_callback(msg)
                            self.get_logger().info(f"[Master {self.robot_namespace}] Waypoint assegnato a se stesso: {waypoint_msg}")
                        else:
                            if slave.publisher is None:
                                slave.publisher = self.create_publisher(String, f'/{slave_ns}/navigation_commands', 10)
                            slave.publisher.publish(msg)
                            self.get_logger().info(f"[Master {self.robot_namespace}] Waypoint assegnato a {slave_ns}: {waypoint_msg}")

                        self.occupied_nodes.add(node_label)
                        slave.waiting = False
                        slave.current_waypoint_index += 1
                        if slave.current_waypoint_index >= len(slave.assigned_waypoints):
                            slave.current_waypoint_index = 0
                    else:
                        self.get_logger().warn(f"Nodo {node_label} è ancora occupato. Lo slave {slave_ns} rimane in stato di attesa.")

    def print_subgraphs(self, subgraphs):
        """
        Stampa i dettagli di ogni subgraph dopo la partizione per il debugging.
        Questo aiuta a verificare che la partizione funzioni correttamente.
        """
        self.get_logger().info("----- Subgraph Dopo Partizionamento -----")
        for idx, subgraph in enumerate(subgraphs):
            self.get_logger().info(f"Subgraph {idx+1}:")
            self.get_logger().info(f"  Nodi ({len(subgraph.nodes())}):")
            for node, data in subgraph.nodes(data=True):
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                orientation = data.get('orientation', 0.0)
                self.get_logger().info(f"    {node}: Posizione=({x}, {y}), Orientamento={orientation} radianti")
            self.get_logger().info(f"  Archi ({len(subgraph.edges())}):")
            for u, v, data in subgraph.edges(data=True):
                weight = data.get('weight', 1.0)
                self.get_logger().info(f"    Da {u} a {v}, Peso: {weight}")
        self.get_logger().info("----- Fine Subgraph -----")

    def extract_waypoints(self, subgraph):
        """
        Estrae i waypoints (nodi) da un subgraph come lista di dizionari.
        Ogni dizionario include label, x, y e orientamento.
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

    def orientation_conversion(self, orientation_input):
        """
        Converte un orientamento dato come stringa (NORTH, EAST, SOUTH, WEST) o float in radianti.
        Default a 0 se non riconosciuto.
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
            return float(orientation_input)
        else:
            return 0.0

    def load_full_graph_from_data(self, graph_data):
        """
        Carica un grafo diretto (DiGraph) da un dizionario con nodi e archi.
        Utilizzato quando riceviamo il grafo di navigazione come JSON.
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
    """
    Punto di ingresso principale.
    Analizza gli argomenti, inizializza ROS, crea il nodo e gira fino all'interruzione.
    """
    rclpy.init(args=args)

    # Analizza gli argomenti da linea di comando
    parser = argparse.ArgumentParser(description='Slave Navigation Simulator Node')
    parser.add_argument('--robot_namespace', type=str, default='robot_simulator', help='Namespace del robot')
    parser.add_argument('--initial_node_label', type=str, default='node_1', help='Etichetta del nodo iniziale dove il robot inizia')
    parser.add_argument('--initial_orientation', type=str, default='NORTH', help='Orientamento iniziale (NORTH, EAST, SOUTH, WEST)')

    # Ignora gli argomenti sconosciuti di ROS
    args, unknown = parser.parse_known_args()

    # Crea l'istanza del nodo con i parametri forniti
    node = SlaveNavigationSimulator(
        robot_namespace=args.robot_namespace,
        initial_node_label=args.initial_node_label,
        initial_orientation_str=args.initial_orientation
    )

    try:
        # Mantiene il nodo in esecuzione e rispondendo ai callback fino all'interruzione
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Pulizia all'uscita
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
