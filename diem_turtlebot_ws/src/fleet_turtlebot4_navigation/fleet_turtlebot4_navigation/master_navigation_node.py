#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import networkx as nx
import os
from .graph_partitioning import load_full_graph, partition_graph
from .path_calculation import calculate_dcpp_route, orientation_conversion
import matplotlib.pyplot as plt  # Importato per la visualizzazione opzionale

class SlaveState:
    """
    Classe per gestire lo stato di ogni robot slave.
    """
    def __init__(self, slave_ns, publisher):
        self.slave_ns = slave_ns
        self.publisher = publisher
        self.assigned_waypoints = []  # Lista di waypoints assegnati
        self.current_waypoint_index = 0  # Indice del prossimo waypoint da assegnare
        self.last_seen_time = 0.0  # Ultimo momento in cui lo slave ha comunicato
        self.initial_x = None  # Posizione X iniziale
        self.initial_y = None  # Posizione Y iniziale
        self.initial_orientation = None  # Orientamento iniziale

class MasterNavigationNode(Node):
    def __init__(self):
        super().__init__('master_navigation_node')

        # Dichiarazione dei parametri ROS 2 con valori di default
        self.declare_parameter('graph_path', '')
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('timeout', 150.0)  # Timeout per slave inattivi in secondi

        # Recupero dei valori dei parametri
        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Caricamento del grafo completo dal file JSON specificato
        if not os.path.exists(self.graph_path):
            self.get_logger().error(f"File del grafo non trovato in {self.graph_path}")
            raise FileNotFoundError(f"File del grafo non trovato in {self.graph_path}")
        
        self.full_graph = load_full_graph(self.graph_path)
        self.get_logger().info("Nodo master ha caricato il grafo.")

        # Stampa il grafo caricato
        self.print_graph()

        # Visualizza il grafo (opzionale, può essere commentato se non necessario)
        # self.visualize_graph()

        # Pubblica il grafo di navigazione sul topic '/navigation_graph'
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)
        self.publish_navigation_graph()

        # Inizializzazione dei subscriber per la registrazione degli slave e le posizioni iniziali
        self.slave_registration_subscriber = self.create_subscription(
            String,
            '/slave_registration',
            self.slave_registration_callback,
            10
        )

        self.initial_position_subscriber = self.create_subscription(
            String,
            '/slave_initial_positions',
            self.initial_position_callback,
            10
        )

        # Subscriber per ricevere aggiornamenti sullo stato di navigazione dagli slave
        self.navigation_status_subscriber = self.create_subscription(
            String,
            '/navigation_status',
            self.navigation_status_callback,
            10
        )

        # Publisher per inviare messaggi di heartbeat che indicano che il master è attivo
        self.heartbeat_publisher = self.create_publisher(String, '/master_heartbeat', 10)

        # Timer per pubblicare messaggi di heartbeat ogni 1 secondo
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Timer per controllare lo stato degli slave e assegnare nuovi waypoints
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Dizionario per tracciare gli slave attivi e i loro stati
        self.slaves = {}  # Chiave: slave_ns, Valore: SlaveState

        # Flag che indica se la partizione del grafo e l'assegnazione dei waypoints sono state completate
        self.partitioning_done = False

    def print_graph(self, max_nodes=20, max_edges=20):
        """
        Stampa i nodi e gli archi del grafo completo per scopi di debug.
        Limita il numero di nodi e archi stampati per evitare log eccessivi.
        
        Args:
            max_nodes (int): Numero massimo di nodi da stampare.
            max_edges (int): Numero massimo di archi da stampare.
        """
        self.get_logger().info("----- Grafico Caricato -----")
        self.get_logger().info(f"Nodi (mostrati fino a {min(max_nodes, len(self.full_graph.nodes()))}/{len(self.full_graph.nodes())}):")
        for node, data in list(self.full_graph.nodes(data=True))[:max_nodes]:
            self.get_logger().info(f"  Nodo: {node}, Posizione: ({data['x']}, {data['y']}), Orientamento: {data.get('orientation', 0.0)} radians")
        
        self.get_logger().info(f"Archi (mostrati fino a {min(max_edges, len(self.full_graph.edges()))}/{len(self.full_graph.edges())}):")
        for u, v, data in list(self.full_graph.edges(data=True))[:max_edges]:
            self.get_logger().info(f"  Da {u} a {v}, Peso: {data.get('weight', 1.0)}")
        self.get_logger().info("----- Fine del Grafico -----")

    def visualize_graph(self):
        """
        Visualizza il grafo utilizzando matplotlib e NetworkX.
        Nota: Questa funzione blocca l'esecuzione fino a quando la finestra grafica non viene chiusa.
        Assicurati che l'ambiente in cui esegui il nodo supporti la visualizzazione grafica.
        """
        plt.figure(figsize=(10, 8))
        pos = {node: (data['x'], data['y']) for node, data in self.full_graph.nodes(data=True)}
        
        nx.draw_networkx_nodes(self.full_graph, pos, node_size=500, node_color='lightblue')
        nx.draw_networkx_edges(self.full_graph, pos, edge_color='gray', arrows=True)
        nx.draw_networkx_labels(self.full_graph, pos, font_size=10, font_weight='bold')
        
        plt.title("Visualizzazione del Grafo di Navigazione")
        plt.axis('off')
        plt.show()

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
        self.get_logger().info("Pubblicato grafo di navigazione.")

    def publish_heartbeat(self):
        """
        Pubblica un messaggio di heartbeat per indicare che il master è attivo.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = "alive"
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug("Pubblicato heartbeat.")

    def slave_registration_callback(self, msg):
        """
        Callback per gestire i messaggi di registrazione degli slave.
        Aggiorna il dizionario degli slave con il namespace e l'ora corrente.
        Inizializza le strutture dati per i nuovi slave.
        
        Args:
            msg (String): Messaggio contenente il namespace dello slave.
        """
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        if slave_ns not in self.slaves:
            self.get_logger().info(f"Nuovo slave registrato: {slave_ns}")

            # Crea un publisher per inviare comandi di navigazione allo slave
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            self.slaves[slave_ns] = slave_state

            # Partizione del grafo e assegnazione dei waypoints poiché è stato aggiunto un nuovo slave
            self.repartition_and_assign_waypoints()
        else:
            self.get_logger().info(f"Slave re-registrato: {slave_ns}")
            # Aggiorna l'ultimo tempo visto per lo slave
            self.slaves[slave_ns].last_seen_time = current_time

    def initial_position_callback(self, msg):
        """
        Callback per gestire i messaggi di posizione iniziale degli slave.
        Si aspetta una stringa JSON con 'robot_namespace', 'x', 'y', 'orientation'.
        
        Args:
            msg (String): Messaggio contenente la posizione iniziale dello slave.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            initial_x = data['x']
            initial_y = data['y']
            initial_orientation = data['orientation']
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Messaggio di posizione iniziale non valido: {e}")
            return

        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time
            slave.initial_x = initial_x
            slave.initial_y = initial_y
            slave.initial_orientation = initial_orientation
            self.get_logger().info(
                f"Posizione iniziale ricevuta da {slave_ns}: "
                f"({initial_x}, {initial_y}, {initial_orientation} radians)"
            )
        else:
            # Slave non registrato, registrarlo ora
            self.get_logger().info(f"Posizione iniziale ricevuta per lo slave non registrato {slave_ns}, registrazione in corso.")
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            slave_state.initial_x = initial_x
            slave_state.initial_y = initial_y
            slave_state.initial_orientation = initial_orientation
            self.slaves[slave_ns] = slave_state
            self.get_logger().info(f"Creato publisher per lo slave: {slave_ns}")

            # Partizione del grafo e assegnazione dei waypoints poiché è stato aggiunto un nuovo slave
            self.repartition_and_assign_waypoints()

    def timer_callback(self):
        """
        Callback periodico per:
        - Controllare gli slave inattivi basati sul timeout.
        - Rimuovere gli slave che hanno superato il timeout.
        - Partizionare nuovamente e assegnare waypoints se gli slave attivi sono cambiati.
        """
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        removed_slaves = []

        # Controlla i timeout degli slave
        for slave_ns in list(self.slaves.keys()):
            slave = self.slaves[slave_ns]
            if current_time - slave.last_seen_time > self.timeout:
                removed_slaves.append(slave_ns)
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Slave {slave_ns} ha superato il timeout e è stato rimosso dagli slave attivi.")

        if removed_slaves:
            # Partizione del grafo e assegnazione dei waypoints poiché gli slave attivi sono cambiati
            self.get_logger().info("Gli slave attivi sono cambiati. Partizionamento del grafo e riassegnazione dei waypoints.")
            self.repartition_and_assign_waypoints()

        # Opzionale: stampa lo stato corrente degli slave
        # self.print_current_status()

    def navigation_status_callback(self, msg):
        """
        Gestisce i feedback dagli slave riguardo allo stato di navigazione.
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
            self.get_logger().error(f"Messaggio di stato di navigazione non valido: {e}")
            return

        self.get_logger().info(
            f"Stato ricevuto da {slave_ns}: {status}, "
            f"Waypoint: {current_waypoint}, Tempo impiegato: {time_taken}s, Errore: {error_message}"
        )

        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time

            if status == "reached":
                # Lo slave ha raggiunto il waypoint corrente, assegna il successivo
                self.get_logger().info(f"Slave {slave_ns} ha raggiunto il waypoint {current_waypoint}.")
                slave.current_waypoint_index += 1
                self.assign_next_waypoint(slave)
            elif status == "error":
                # Gestisce lo stato di errore
                self.get_logger().error(f"Slave {slave_ns} ha riscontrato un errore: {error_message}")
                # Implementa logica di retry o gestione degli errori se necessario
                # Per ora, rimuove lo slave
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Rimozione dello slave {slave_ns} a causa di un errore.")
                self.repartition_and_assign_waypoints()
        else:
            self.get_logger().warn(f"Stato ricevuto da uno slave sconosciuto {slave_ns}.")

    def repartition_and_assign_waypoints(self):
        """
        Partiziona il grafo di navigazione basato sul numero di slave attivi e assegna waypoints a ciascuno slave.
        Calcola il percorso DCPP (Circuito Euleriano) per ogni sottografo.
        """
        num_slaves = len(self.slaves)
        if num_slaves == 0:
            self.get_logger().warn("Nessuno slave attivo trovato. In attesa che gli slave si registrino.")
            self.partitioning_done = False
            return

        # Raccogli le posizioni iniziali degli slave
        start_positions = []
        for slave in self.slaves.values():
            if slave.initial_x is not None and slave.initial_y is not None:
                start_positions.append({'x': slave.initial_x, 'y': slave.initial_y})
            else:
                self.get_logger().warn(f"Lo slave {slave.slave_ns} non ha una posizione iniziale valida.")

        if len(start_positions) != num_slaves:
            self.get_logger().error("Non tutti gli slave hanno posizioni iniziali valide.")
            return

        # Partiziona il grafo in sottografi basati sul numero di slave e le loro posizioni iniziali
        try:
            subgraphs = partition_graph(self.full_graph, num_slaves, start_positions=start_positions)
            self.get_logger().info(f"Partizionato il grafo in {len(subgraphs)} sottografi.")
        except ValueError as e:
            self.get_logger().error(f"Fallito nel partizionare il grafo: {e}")
            return

        # Ordina gli slave per garantire un'assegnazione consistente (es. ordine alfabetico)
        slaves_sorted = sorted(self.slaves.keys())

        # Assicurati che il numero di sottografi corrisponda al numero di slave
        if len(subgraphs) != len(slaves_sorted):
            self.get_logger().error("Il numero di sottografi non corrisponde al numero di slave attivi.")
            return

        # Assegna ogni sottografo a uno slave
        for idx, slave_ns in enumerate(slaves_sorted):
            subgraph = subgraphs[idx]
            waypoints = self.extract_waypoints(subgraph)

            # Assicurati che lo slave abbia una posizione iniziale
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
                self.get_logger().warn(f"Lo slave {slave_ns} manca dei dati di posizione iniziale.")

            # Calcola il percorso DCPP (Circuito Euleriano) per il sottografo
            dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.get_logger())
            # Assegna il percorso allo slave
            self.assign_route_to_slave(slave_ns, dcpp_route)

        self.partitioning_done = True

    def extract_waypoints(self, subgraph):
        """
        Estrae i waypoints da un sottografo.

        Args:
            subgraph (nx.Graph): Sottografo da cui estrarre i waypoints.

        Returns:
            list of dict: Lista di waypoints con 'label', 'x', 'y', e 'orientation'.
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
        Assegna un percorso di waypoints a uno specifico slave.

        Args:
            slave_ns (str): Namespace dello slave robot.
            route (list of dict): Lista ordinata di waypoints.
        """
        if slave_ns not in self.slaves:
            self.get_logger().error(f"Nessuno stato trovato per lo slave {slave_ns}. Impossibile assegnare il percorso.")
            return

        slave = self.slaves[slave_ns]
        slave.assigned_waypoints = route.copy()
        slave.current_waypoint_index = 0

        # Logging dettagliato del percorso assegnato
        self.get_logger().info(f"Percorso DCPP assegnato a {slave_ns}:")
        for wp in route:
            self.get_logger().info(f" - {wp['label']} a ({wp['x']}, {wp['y']}), Orientamento: {wp['orientation']} radians")

        # Assegna il primo waypoint
        self.assign_next_waypoint(slave)

        # Log del numero di waypoints assegnati
        self.get_logger().info(f"Assegnati {len(route)} waypoints a {slave_ns}.")

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
                'orientation': orientation_conversion(waypoint['orientation'])
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            slave.publisher.publish(msg)
            self.get_logger().info(f"Assegnato waypoint a {slave.slave_ns}: {waypoint_msg}")
        else:
            # Tutti i waypoints sono stati assegnati, ricomincia dal primo
            self.get_logger().info(f"Tutti i waypoints sono stati assegnati a {slave.slave_ns}. Ricomincio il percorso.")
            slave.current_waypoint_index = 0
            self.assign_next_waypoint(slave)  # Assegna nuovamente il primo waypoint

    def print_current_status(self):
        """
        Stampa lo stato attuale degli slave attivi.
        """
        num_active_slaves = len(self.slaves)
        self.get_logger().info(f"Slave attivi attualmente: {num_active_slaves}")
        for slave_ns in sorted(self.slaves.keys()):
            slave = self.slaves[slave_ns]
            status = "Assegnato" if slave.current_waypoint_index < len(slave.assigned_waypoints) else "Completato"
            self.get_logger().info(f"Slave {slave_ns}: Stato: {status}")

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

# Esegui il nodo master con il percorso corretto del grafo
# Esempio di comando:
# ros2 run fleet_turtlebot4_navigation master_navigation_node --ros-args -p graph_path:=/home/beniamino/turtlebot4/diem_turtlebot_ws/src/fleet_turtlebot4_navigation/map/navigation_hardware_limitation.json
