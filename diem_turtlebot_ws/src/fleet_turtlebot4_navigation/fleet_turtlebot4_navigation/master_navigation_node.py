#!/usr/bin/env python3

# Importazioni di base di ROS 2 e delle librerie Python necessarie
import rclpy  # Libreria Python per interfacciarsi con ROS 2
from rclpy.node import Node  # Classe base per creare nodi in ROS 2
from std_msgs.msg import String  # Tipo di messaggio ROS standard per stringhe
import json  # Per la manipolazione di dati in formato JSON
import math  # Per funzioni matematiche (non strettamente usate in questo file, ma utile se servisse)
from .graph_partitioning import load_full_graph, partition_graph  # Funzioni custom per caricare e partizionare un grafo
from .path_calculation import calculate_dcpp_route, orientation_rad_to_str  # Funzioni custom per calcolo percorso e conversione orientamenti
import os  # Per operazioni sul file system
import networkx as nx  # Libreria per la rappresentazione e gestione di grafi complessi

class SlaveState:
    """
    Classe che rappresenta lo stato interno di uno slave (un robot controllato dal master).
    Questa classe memorizza le informazioni fondamentali per gestire e tracciare un singolo slave.
    """

    def __init__(self, slave_ns, publisher):
        # slave_ns: stringa che identifica univocamente il robot slave (es. "robot_1")
        self.slave_ns = slave_ns

        # publisher: un publisher ROS 2 creato dal master per inviare comandi di navigazione a questo slave
        self.publisher = publisher

        # assigned_waypoints: una lista di waypoint (punti di navigazione) assegnati a questo slave.
        # Ogni waypoint è tipicamente un dizionario con 'label', 'x', 'y', 'orientation'.
        self.assigned_waypoints = []

        # current_waypoint_index: indice del prossimo waypoint da assegnare allo slave.
        # Quando lo slave completa un waypoint, l'indice incrementa per passare al successivo.
        self.current_waypoint_index = 0

        # last_seen_time: timestamp dell'ultima volta che abbiamo ricevuto un messaggio dallo slave.
        # Questo ci serve per verificare se lo slave è ancora attivo o se è "andato offline".
        self.last_seen_time = 0.0

        # initial_x, initial_y, initial_orientation: posizioni e orientamento iniziale dello slave.
        # Questi valori vengono aggiornati quando riceviamo il messaggio con la posizione iniziale dello slave.
        self.initial_x = None
        self.initial_y = None
        self.initial_orientation = None

        # waiting: flag booleano per indicare se lo slave è in attesa. Se True,
        # significa che lo slave è pronto per un prossimo waypoint ma il master non può ancora assegnarlo
        # (ad esempio perché il nodo è già occupato).
        self.waiting = False


class MasterNavigationNode(Node):
    """
    Nodo Master per la navigazione di una flotta di robot TurtleBot4.
    Questo nodo gestisce l'intero processo di:
    - Ricezione della registrazione degli slave
    - Ricezione delle posizioni iniziali degli slave
    - Pubblicazione e partizionamento del grafo di navigazione
    - Assegnazione dei waypoint agli slave
    - Gestione dei messaggi di stato inviati dagli slave (raggiunto waypoint, errori, ecc.)
    - Monitoraggio degli slave e gestione dei timeout
    """

    def __init__(self):
        # Inizializzazione del nodo master con il nome "master_navigation_node"
        super().__init__('master_navigation_node')

        # Dichiarazione dei parametri configurabili da linea di comando o file di launch
        # graph_path: percorso del file JSON contenente il grafo di navigazione
        self.declare_parameter('graph_path', '/home/beniamino/turtlebot4/diem_turtlebot_ws/src/fleet_turtlebot4_navigation/map/navigation_hardware_limitation.json')
        # check_interval: intervallo in secondi per controllare lo stato degli slave
        self.declare_parameter('check_interval', 2.0)
        # timeout: tempo in secondi dopo il quale uno slave inattivo viene considerato perso
        self.declare_parameter('timeout', 150.0)

        # Recupero dei parametri dichiarati
        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Verifica che il file del grafo esista
        if not os.path.exists(self.graph_path):
            self.get_logger().error(f"Graph file not found at {self.graph_path}")
            raise FileNotFoundError(f"Graph file not found at {self.graph_path}")

        # Carica il grafo completo
        self.full_graph = load_full_graph(self.graph_path)

        # Creazione di un publisher per inviare il grafo di navigazione agli slave
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)

        # Timer per pubblicare periodicamente il grafo di navigazione (ogni 5 secondi)
        self.graph_timer = self.create_timer(5.0, self.publish_navigation_graph)

        # Subscriber per ricevere la registrazione degli slave
        self.slave_registration_subscriber = self.create_subscription(
            String,
            '/slave_registration',
            self.slave_registration_callback,
            10
        )

        # Subscriber per ricevere la posizione iniziale degli slave
        self.initial_position_subscriber = self.create_subscription(
            String,
            '/slave_initial_positions',
            self.initial_position_callback,
            10
        )

        # Subscriber per ricevere lo stato di navigazione dagli slave
        self.navigation_status_subscriber = self.create_subscription(
            String,
            '/navigation_status',
            self.navigation_status_callback,
            10
        )

        # Publisher per inviare heartbeat che indica al mondo (e agli slave) che il master è vivo
        self.heartbeat_publisher = self.create_publisher(String, '/master_heartbeat', 10)

        # Timer che invia heartbeat ogni 1 secondo
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Timer per eseguire controlli periodici sugli slave (ogni check_interval secondi)
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Dizionario per gestire gli slave attivi
        # Chiave: namespace dello slave (es. "robot_1"), Valore: oggetto SlaveState
        self.slaves = {}

        # Flag per indicare se la partizione del grafo è stata completata
        self.partitioning_done = False

        # Set di nodi occupati (i nodi del grafo attualmente assegnati a qualche waypoint)
        self.occupied_nodes = set()

        # Master node inizializzato
        # (Il log è commentato per evitare output eccessivo)
        # self.get_logger().info("Master node initialized.")

    def publish_navigation_graph(self):
        """
        Pubblica il grafo di navigazione sul topic '/navigation_graph'.
        I nodi e gli archi del grafo vengono serializzati in formato JSON.
        Questo consente agli slave di conoscere la topologia della mappa.
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

        # Convertiamo il dizionario in una stringa JSON
        graph_msg.data = json.dumps(graph_data)
        self.graph_publisher.publish(graph_msg)
        # self.get_logger().info("Published navigation graph.")

    def publish_heartbeat(self):
        """
        Pubblica un messaggio di "heartbeat" per segnalare agli slave che il master è vivo.
        Gli slave usano questi heartbeat per capire se il master è ancora presente.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = "alive"
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug("Published heartbeat.")

    def slave_registration_callback(self, msg):
        """
        Callback chiamato quando uno slave si registra inviando il proprio namespace sul topic '/slave_registration'.
        """
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Se lo slave non è già registrato, creiamo un nuovo stato
        if slave_ns not in self.slaves:
            # Crea un publisher per inviare comandi di navigazione allo slave
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            self.slaves[slave_ns] = slave_state
            # Non ripartizioniamo subito, attendiamo la posizione iniziale dello slave
        else:
            # Aggiorna solo l'ora dell'ultima comunicazione ricevuta dallo slave
            self.slaves[slave_ns].last_seen_time = current_time

    def initial_position_callback(self, msg):
        """
        Callback chiamato quando uno slave invia la propria posizione iniziale.
        Il messaggio contiene: robot_namespace, x, y, orientation.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            initial_x = float(data['x'])
            initial_y = float(data['y'])
            orientation = data.get('orientation', 'NORTH')
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f"Invalid initial position message: {e}")
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Verifica che lo slave sia registrato
        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            # Aggiorna i dati dello slave con la posizione iniziale
            slave.last_seen_time = current_time
            slave.initial_x = initial_x
            slave.initial_y = initial_y
            slave.initial_orientation = orientation

            # Ora che abbiamo la posizione iniziale, possiamo procedere con la partizione del grafo
            self.repartition_and_assign_waypoints()
        else:
            # Se lo slave non era registrato prima, lo registriamo ora
            self.get_logger().info(f"Initial position received for unregistered slave {slave_ns}, registering now.")
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            slave_state.initial_x = initial_x
            slave_state.initial_y = initial_y
            slave_state.initial_orientation = orientation
            self.slaves[slave_ns] = slave_state
            self.get_logger().info(f"Created publisher for slave: {slave_ns}")

            # Avvia la partizione e l'assegnazione dei waypoint ora che abbiamo un nuovo slave
            self.repartition_and_assign_waypoints()

    def print_all_initial_positions(self):
        """
        Funzione di utilità per stampare le posizioni iniziali di tutti gli slave registrati.
        Utile per il debug.
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
        Callback per gestire i messaggi di stato della navigazione inviati dagli slave.
        Il messaggio contiene informazioni quali:
        - robot_namespace (lo slave che sta reportando)
        - status (es: "reached", "error")
        - current_waypoint (il waypoint corrente raggiunto o con errore)
        - time_taken (tempo impiegato per raggiungere o fallire il waypoint)
        - error_message (eventuale messaggio di errore)
        """

        # Stampa il messaggio ricevuto per debug
        self.get_logger().info(f"Received navigation status message: {msg.data}")

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

        # Stampa i dati decodificati per debug
        self.get_logger().info(f"Decoded navigation status data: {data}")

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Verifica se lo slave è noto
        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time

            # Gestione dei vari status
            if status == "reached":
                # Lo slave ha raggiunto il waypoint
                if current_waypoint in self.occupied_nodes:
                    # Libera il nodo occupato
                    self.occupied_nodes.remove(current_waypoint)
                    self.get_logger().info(f"Node {current_waypoint} is now free.")
                else:
                    self.get_logger().warn(f"Node {current_waypoint} was not marked as occupied.")

                self.get_logger().info(f"Slave {slave_ns} has reached waypoint {current_waypoint}.")
                slave.current_waypoint_index += 1
                slave.waiting = False

                # Assegna il prossimo waypoint
                self.assign_next_waypoint(slave_ns)

                # Assegna eventuali slave in attesa
                self.assign_waiting_slaves()

            elif status == "error":
                # Lo slave ha incontrato un errore nel raggiungere il waypoint
                self.get_logger().error(f"Slave {slave_ns} encountered an error: {error_message}")
                if current_waypoint in self.occupied_nodes:
                    self.occupied_nodes.remove(current_waypoint)
                    self.get_logger().info(f"Node {current_waypoint} is now free due to error.")

                # Rimuovi lo slave dall'elenco degli slave attivi
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removing slave {slave_ns} due to error.")

                # Ripartisci il grafo a causa del cambio nel numero di slave
                self.repartition_and_assign_waypoints()
        else:
            # Lo slave non è noto (potrebbe non essere registrato)
            self.get_logger().warn(f"Received status from unknown slave {slave_ns}.")

    def check_all_waypoints_reached(self):
        """
        Controlla se tutti i waypoint assegnati agli slave sono stati raggiunti.
        Se tutti gli slave hanno raggiunto tutti i loro waypoint, potremmo considerare
        l'operazione completa (se il percorso non è ciclico).

        Returns:
            bool: True se tutti i waypoint sono stati completati da tutti gli slave.
        """
        for slave in self.slaves.values():
            # Se uno slave ha ancora waypoint non completati, restituisce False
            if slave.current_waypoint_index < len(slave.assigned_waypoints):
                return False
        return True

    def repartition_and_assign_waypoints(self):
        """
        Funzione per partizionare il grafo e assegnare i percorsi agli slave.
        Il grafo viene diviso in sottografi, uno per ogni slave, in base alle loro posizioni iniziali.
        Viene quindi calcolato un percorso DCPP (copertura bilanciata dei nodi) e assegnato agli slave.
        """
        num_slaves = len(self.slaves)
        if num_slaves == 0:
            self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
            self.partitioning_done = False
            return

        # Raccoglie le posizioni iniziali di tutti gli slave
        start_positions = []
        for slave in self.slaves.values():
            if slave.initial_x is not None and slave.initial_y is not None:
                start_positions.append({'x': slave.initial_x, 'y': slave.initial_y})
            else:
                self.get_logger().warn(f"Slave {slave.slave_ns} lacks valid initial position.")

        # Se non tutti gli slave hanno una posizione iniziale valida, non possiamo partizionare
        if len(start_positions) != num_slaves:
            self.get_logger().error("Not all slaves have valid initial positions.")
            return

        # Partiziona il grafo
        try:
            subgraphs = partition_graph(self.full_graph, num_slaves, start_positions=start_positions)
            self.get_logger().info(f"Partitioned the graph into {len(subgraphs)} subgraphs.")
        except ValueError as e:
            self.get_logger().error(f"Failed to partition graph: {e}")
            return

        # Ordina gli slave per avere un ordine coerente di assegnazione
        slaves_sorted = sorted(self.slaves.keys())

        # Verifica che il numero di sottografi corrisponda al numero di slave
        if len(subgraphs) != len(slaves_sorted):
            self.get_logger().error("Number of subgraphs does not match number of active slaves.")
            return

        # Assegna un percorso DCPP a ciascuno slave
        for idx, slave_ns in enumerate(slaves_sorted):
            subgraph = subgraphs[idx]
            waypoints = self.extract_waypoints(subgraph)

            # Calcola il percorso DCPP (un percorso che visita tutti i nodi con copertura bilanciata)
            dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.get_logger())
            ordered_route = dcpp_route

            self.get_logger().info(f"DCPP Route for {slave_ns}:")
            for wp in ordered_route:
                self.get_logger().info(f"  {wp}")

            # Memorizza il percorso calcolato nello stato dello slave
            slave = self.slaves[slave_ns]
            slave.assigned_waypoints = ordered_route

            # Assegna il primo waypoint allo slave
            self.assign_next_waypoint(slave_ns)

        self.partitioning_done = True

    def assign_next_waypoint(self, slave_ns):
        """
        Assegna il prossimo waypoint allo slave specificato.
        Gestisce la logica per evitare di assegnare waypoint su nodi già occupati.
        """
        slave = self.slaves[slave_ns]
        if len(slave.assigned_waypoints) == 0:
            self.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
            return

        # Supporto per loop continuo: il prossimo waypoint è quello all'indice current_waypoint_index % numero_waypoints
        waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
        node_label = waypoint['label']

        # Se il nodo è già occupato da un altro slave
        if node_label in self.occupied_nodes:
            self.get_logger().warn(f"Node {node_label} is already occupied. Cannot assign to slave {slave_ns}.")
            slave.waiting = True
            return

        # Prepara il messaggio da inviare allo slave
        waypoint_msg = {
            'label': waypoint['label'],
            'x': waypoint['x'],
            'y': waypoint['y'],
            'orientation': orientation_rad_to_str(waypoint['orientation'])
        }
        msg = String()
        msg.data = json.dumps(waypoint_msg)

        # Pubblica il waypoint allo slave
        slave.publisher.publish(msg)
        self.occupied_nodes.add(node_label)  # Segna il nodo come occupato
        self.get_logger().info(f"Assigned waypoint to {slave.slave_ns}: {waypoint_msg}")

        # Se l'indice supera la lunghezza dei waypoint, resetta a 0 (loop continuo)
        if slave.current_waypoint_index >= len(slave.assigned_waypoints):
            slave.current_waypoint_index = 0

    def assign_waiting_slaves(self):
        """
        Assegna i waypoint agli slave che sono in attesa, se ora i nodi richiesti sono liberi.
        """
        for slave_ns in sorted(self.slaves.keys()):
            slave = self.slaves[slave_ns]
            if slave.waiting:
                if len(slave.assigned_waypoints) == 0:
                    self.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                    continue

                # Recupera il prossimo waypoint in coda
                waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
                node_label = waypoint['label']

                # Controlla se ora il nodo non è più occupato
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
                    slave.publisher.publish(msg)
                    self.occupied_nodes.add(node_label)
                    self.get_logger().info(f"Assigned waypoint to {slave.slave_ns}: {waypoint_msg}")
                    slave.waiting = False

                    # Incrementa l'indice del waypoint
                    slave.current_waypoint_index += 1

                    # Se l'indice ha superato il numero dei waypoint, resetta a 0
                    if slave.current_waypoint_index >= len(slave.assigned_waypoints):
                        slave.current_waypoint_index = 0
                else:
                    # Il nodo richiesto è ancora occupato, rimane in attesa
                    self.get_logger().warn(f"Node {node_label} is still occupied. Slave {slave.slave_ns} remains in waiting state.")
            else:
                # Non ci sono azioni da fare per slave non in attesa
                pass

    def print_subgraphs(self, subgraphs):
        """
        Stampa i dettagli dei sottografi dopo la partizione.
        Utile per il debug e per verificare la corretta partizione del grafo.
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
        Estrae una lista di waypoint (nodi con coordinate) da un sottografo.
        Ogni waypoint è un dizionario con 'label', 'x', 'y', 'orientation'.
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
        Callback periodico chiamato ogni 'check_interval' secondi.
        Esegue controlli sugli slave (timeout) e prova ad assegnare waypoint agli slave in attesa.
        """
        self.get_logger().debug("Timer callback triggered.")
        self.check_slaves_timeout()
        self.assign_waiting_slaves()

    def check_slaves_timeout(self):
        """
        Controlla se ci sono slave inattivi (che non comunicano da troppo tempo).
        Se uno slave supera il timeout, viene rimosso dall'elenco degli slave attivi
        e i nodi che stava occupando vengono liberati.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        slaves_to_remove = []

        for slave_ns, slave in self.slaves.items():
            # Se il tempo trascorso dall'ultima comunicazione supera il timeout
            if current_time - slave.last_seen_time > self.timeout:
                self.get_logger().warn(f"Slave {slave_ns} has timed out. Removing from active slaves.")
                slaves_to_remove.append(slave_ns)

        for slave_ns in slaves_to_remove:
            # Rimuovi eventuale nodo occupato da questo slave
            if slave_ns in self.slaves:
                slave = self.slaves[slave_ns]
                # Se lo slave aveva waypoint assegnati, libera il nodo corrente
                if slave.current_waypoint_index < len(slave.assigned_waypoints):
                    waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
                    node_label = waypoint['label']
                    if node_label in self.occupied_nodes:
                        self.occupied_nodes.remove(node_label)
                        self.get_logger().info(f"Node {node_label} is now free due to slave timeout.")

            # Rimuovi lo slave dall'elenco
            del self.slaves[slave_ns]
            self.get_logger().warn(f"Removed slave {slave_ns} due to timeout.")

        # Se abbiamo rimosso uno o più slave, è necessario ripartizionare il grafo
        if slaves_to_remove:
            self.repartition_and_assign_waypoints()

def load_full_graph_from_data(graph_data):
    """
    Funzione di utilità per caricare un grafo NetworkX da un dizionario di nodi ed archi.
    Questo può essere utile se si ha già il grafo in formato dizionario e lo si vuole trasformare in un DiGraph.
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
    Punto di ingresso principale del nodo master.
    Inizializza ROS 2, crea il nodo MasterNavigationNode e avvia lo spinning di ROS 2.
    """
    rclpy.init(args=args)  # Inizializzazione ROS 2
    node = MasterNavigationNode()  # Crea un'istanza del nodo master

    try:
        rclpy.spin(node)  # Mantiene il nodo attivo ed in ascolto dei messaggi
    except KeyboardInterrupt:
        pass

    # Shutdown di ROS 2 e distruzione del nodo
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
