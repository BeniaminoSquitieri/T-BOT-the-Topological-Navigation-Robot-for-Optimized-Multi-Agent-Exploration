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

# Import delle funzioni ausiliarie dai moduli esistenti
from .graph_partitioning import load_full_graph, partition_graph
from .path_calculation import calculate_dcpp_route, orientation_rad_to_str

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

        # Publisher per inviare il grafo di navigazione se diventa master
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

        # Dizionario degli slave attivi (namespace e stato)
        self.active_slaves = {}  # Chiave: slave_ns, Valore: SlaveState

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

        # Subscriber per i messaggi di stato di navigazione (necessario solo come master)
        self.navigation_status_subscriber = self.create_subscription(
            String,
            '/navigation_status',
            self.navigation_status_callback,
            10
        )

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
                if slave_ns not in self.active_slaves:
                    # Crea uno stato per il nuovo slave se non esiste
                    self.active_slaves[slave_ns] = SlaveState(slave_ns)
                    self.get_logger().info(f"[{self.robot_namespace}] Detected new slave: {slave_ns}")
                self.active_slaves[slave_ns].last_seen_time = current_time
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
                if current_time - self.active_slaves[slave_ns].last_seen_time > self.heartbeat_timeout:
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
        simulated_navigation_time = 5.0
        time.sleep(simulated_navigation_time)

        # Simula il risultato della navigazione
        nav_success = True  # Cambia questo valore per simulare errori

        if nav_success:
            # Navigazione riuscita
            self.get_logger().info(f"[{self.robot_namespace}] Reached {label} in {simulated_navigation_time} seconds.")
            self.publish_status("reached", "", simulated_navigation_time, label)
            if self.is_master:
                # Se è master, gestisci gli occupati
                with self.lock:
                    self.current_waypoint_index += 1
                # Libera il nodo dopo aver raggiunto il waypoint
                with self.lock:
                    if label in self.occupied_nodes:
                        self.occupied_nodes.remove(label)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {label} is now free.")
                # Assegna il prossimo waypoint
                self.assign_next_waypoint(self.robot_namespace)
                # Assegna i waypoints agli altri slave in attesa
                self.assign_waiting_slaves()
            else:
                # Se è slave, semplicemente aggiorna il proprio stato
                pass
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

    def navigation_status_callback(self, msg):
        """
        Gestisce il feedback degli slave riguardo lo stato di navigazione.
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
            with self.lock:
                if slave_ns in self.active_slaves:
                    slave = self.active_slaves[slave_ns]
                elif slave_ns == self.robot_namespace:
                    slave = self  # Se stesso
                else:
                    self.get_logger().warn(f"[{self.robot_namespace}] Received status from unknown slave {slave_ns}.")
                    return

                slave.last_seen_time = current_time

                if status == "reached":
                    # Rimuovi il nodo dal set dei nodi occupati
                    if current_waypoint in self.occupied_nodes:
                        self.occupied_nodes.remove(current_waypoint)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {current_waypoint} is now free.")
                    else:
                        self.get_logger().warn(f"[{self.robot_namespace}] Node {current_waypoint} was not marked as occupied.")

                    self.get_logger().info(f"[{self.robot_namespace}] Slave {slave_ns} has reached waypoint {current_waypoint}.")
                    slave.waiting = False  # Lo slave non è più in attesa

                    # Assegna il prossimo waypoint
                    self.assign_next_waypoint(slave_ns)

                    # Dopo aver assegnato il prossimo waypoint, prova a assegnare waypoint agli slave in attesa
                    self.assign_waiting_slaves()

                elif status == "error":
                    # Gestione dello stato di errore
                    self.get_logger().error(f"[{self.robot_namespace}] Slave {slave_ns} encountered an error: {error_message}")
                    # Rimuovi il nodo dal set dei nodi occupati se necessario
                    if current_waypoint in self.occupied_nodes:
                        self.occupied_nodes.remove(current_waypoint)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {current_waypoint} is now free due to error.")
                    # Rimuovi lo slave
                    if slave_ns in self.active_slaves:
                        del self.active_slaves[slave_ns]
                        self.get_logger().warn(f"[{self.robot_namespace}] Removing slave {slave_ns} due to error.")
                        # Ripartizione del grafo
                        self.partition_and_assign_waypoints()
        else:
            # Se non è master, nessuna azione aggiuntiva
            pass

    def partition_and_assign_waypoints(self):
        """
        Partiziona il grafo di navigazione in base al numero di slave attivi e assegna un percorso DCPP a ciascuno slave.
        Questo metodo viene chiamato se lo slave assume il ruolo di master.
        """
        if self.navigation_graph is None:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot partition and assign waypoints.")
            return

        with self.lock:
            num_slaves = len(self.active_slaves) + 1  # Include se stesso

            if num_slaves == 0:
                self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
                self.partitioning_done = False
                return

            # Raccolta delle posizioni iniziali degli slave
            start_positions = []
            for slave_ns, slave in self.active_slaves.items():
                if slave.initial_x is not None and slave.initial_y is not None:
                    start_positions.append({'x': slave.initial_x, 'y': slave.initial_y})
                else:
                    self.get_logger().warn(f"Slave {slave_ns} initial position not available")

            # Aggiungi la posizione iniziale del master (se stesso)
            start_positions.append({'x': self.initial_x, 'y': self.initial_y})

            if len(start_positions) != num_slaves:
                self.get_logger().error("Not all slaves have valid initial positions.")
                return

            # Partiziona il grafo usando partition_graph
            try:
                subgraphs = partition_graph(self.navigation_graph, num_slaves, start_positions=start_positions)
                self.get_logger().info(f"Partitioned the graph into {len(subgraphs)} subgraphs.")
                # Stampa i sottografi
                self.print_subgraphs(subgraphs)
            except ValueError as e:
                self.get_logger().error(f"Failed to partition graph: {e}")
                return

            # Crea una lista di tutti gli slave, includendo se stesso
            all_slaves = list(self.active_slaves.keys()) + [self.robot_namespace]
            all_slaves_sorted = sorted(all_slaves)

            # Assicurati che il numero di sottografi corrisponda al numero di slave
            if len(subgraphs) != len(all_slaves_sorted):
                self.get_logger().error("Number of subgraphs does not match number of active slaves.")
                return

            # Assegna un percorso DCPP a ciascuno slave
            for idx, slave_ns in enumerate(all_slaves_sorted):
                subgraph = subgraphs[idx]

                # Estrai i waypoint dal sottografo
                waypoints = self.extract_waypoints(subgraph)

                # Calcola il percorso DCPP
                dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.get_logger())
                ordered_route = dcpp_route  # Supponendo che calculate_dcpp_route ritorni il percorso ordinato

                # Log del percorso DCPP calcolato
                self.get_logger().info(f"DCPP Route for {slave_ns}:")
                for wp in ordered_route:
                    self.get_logger().info(f"  {wp}")

                # Assegna i waypoints DCPP allo slave
                if slave_ns == self.robot_namespace:
                    # Assegna i waypoints a se stesso
                    self.assigned_waypoints = ordered_route
                    self.assign_next_waypoint(self.robot_namespace)
                else:
                    # Assegna i waypoints agli altri slave
                    if slave_ns in self.active_slaves:
                        slave = self.active_slaves[slave_ns]
                        slave.assigned_waypoints = ordered_route
                        self.assign_next_waypoint(slave_ns)
                    else:
                        self.get_logger().warn(f"Slave {slave_ns} not found in active_slaves.")

            self.partitioning_done = True

    def assign_next_waypoint(self, slave_ns):
        """
        Assegna il prossimo waypoint nella coda allo slave.

        Args:
            slave_ns (str): Namespace dello slave robot.
        """
        with self.lock:
            if slave_ns == self.robot_namespace:
                # Assegna a se stesso
                slave = self  # Riferimento a se stesso
            else:
                slave = self.active_slaves.get(slave_ns, None)

            if slave is None:
                self.get_logger().warn(f"Slave {slave_ns} not found.")
                return

            if len(slave.assigned_waypoints) == 0:
                self.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                return

            # Assegna il prossimo waypoint con loop
            waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
            node_label = waypoint['label']

            if node_label in self.occupied_nodes:
                self.get_logger().warn(f"Node {node_label} is already occupied. Cannot assign to slave {slave_ns}.")
                slave.waiting = True  # Mette lo slave in attesa
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
                # Simula la ricezione del messaggio chiamando direttamente il callback
                self.navigation_commands_callback(msg)
                self.get_logger().info(f"[Master {self.robot_namespace}] Assigned waypoint to itself: {waypoint_msg}")
            else:
                # Pubblica il comando di navigazione allo slave
                if slave.publisher is None:
                    slave.publisher = self.create_publisher(String, f'/{slave_ns}/navigation_commands', 10)
                slave.publisher.publish(msg)
                self.get_logger().info(f"[Master {self.robot_namespace}] Assigned waypoint to {slave_ns}: {waypoint_msg}")

            self.occupied_nodes.add(node_label)  # Segna il nodo come occupato

            # Incrementa l'indice del waypoint
            slave.current_waypoint_index += 1

            # Resetta l'indice per ricominciare il loop se necessario
            if slave.current_waypoint_index >= len(slave.assigned_waypoints):
                slave.current_waypoint_index = 0

    def assign_waiting_slaves(self):
        """
        Assegna i waypoint agli slave che sono in attesa.
        """
        with self.lock:
            for slave_ns in sorted(list(self.active_slaves.keys()) + [self.robot_namespace]):
                if slave_ns == self.robot_namespace and self.is_master:
                    slave = self  # Se stesso
                else:
                    slave = self.active_slaves.get(slave_ns, None)
                    if slave is None:
                        continue

                if slave.waiting:
                    if len(slave.assigned_waypoints) == 0:
                        self.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                        continue

                    waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
                    node_label = waypoint['label']

                    if node_label not in self.occupied_nodes:
                        # Assegna il waypoint
                        waypoint_msg = {
                            'label': waypoint['label'],
                            'x': waypoint['x'],
                            'y': waypoint['y'],
                            'orientation': orientation_rad_to_str(waypoint['orientation'])
                        }
                        msg = String()
                        msg.data = json.dumps(waypoint_msg)

                        if slave_ns == self.robot_namespace:
                            # Simula la ricezione del messaggio chiamando direttamente il callback
                            self.navigation_commands_callback(msg)
                            self.get_logger().info(f"[Master {self.robot_namespace}] Assigned waypoint to itself: {waypoint_msg}")
                        else:
                            # Pubblica il comando di navigazione allo slave
                            if slave.publisher is None:
                                slave.publisher = self.create_publisher(String, f'/{slave_ns}/navigation_commands', 10)
                            slave.publisher.publish(msg)
                            self.get_logger().info(f"[Master {self.robot_namespace}] Assigned waypoint to {slave_ns}: {waypoint_msg}")

                        self.occupied_nodes.add(node_label)  # Segna il nodo come occupato
                        slave.waiting = False  # Rimuove lo stato di attesa

                        # Incrementa l'indice del waypoint
                        slave.current_waypoint_index += 1

                        # Resetta l'indice per ricominciare il loop se necessario
                        if slave.current_waypoint_index >= len(slave.assigned_waypoints):
                            slave.current_waypoint_index = 0
                    else:
                        self.get_logger().warn(f"Node {node_label} is still occupied. Slave {slave_ns} remains in waiting state.")

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
