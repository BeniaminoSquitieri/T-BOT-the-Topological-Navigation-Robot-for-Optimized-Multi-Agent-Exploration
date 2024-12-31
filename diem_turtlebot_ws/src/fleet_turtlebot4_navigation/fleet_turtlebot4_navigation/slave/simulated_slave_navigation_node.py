#!/usr/bin/env python3

import rclpy  # La libreria client ROS 2 per Python
from rclpy.node import Node  # Classe base per creare un nodo ROS 2 in Python
from std_msgs.msg import String  # Tipo di messaggio ROS standard per trasmettere stringhe
import json  # Per codificare/decodificare messaggi JSON (grafi, stati, posizioni)
import time  # Per timestamp e misurazione degli intervalli
import argparse  # Per argomenti da riga di comando
import math  # Per operazioni matematiche (pi, trigonometria)
import threading  # Per eseguire task in parallelo
import networkx as nx  # Per costruire e gestire grafi (nodi, archi, attributi)


class SlaveNavigationSimulator(Node):
    """
    Nodo slave simulato, che imita il comportamento di un vero nodo di navigazione TurtleBot4.
    """

    def __init__(self, robot_namespace, initial_node_label):
        # Parametri iniziali
        self.robot_namespace = robot_namespace
        self.initial_node_label = initial_node_label

        # Nodo corrente
        self.current_node = initial_node_label

        # Inizializza il nodo ROS con il namespace specificato
        super().__init__('slave_navigation_simulator_node', namespace=self.robot_namespace)

        # Publisher e Subscriber
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.navigation_commands_callback, 10
        )
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 10
        )
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.navigation_graph_callback, 10
        )

        # Timers per attività periodiche
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Variabili di stato
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 5.0  # Timeout per l'heartbeat del master
        self.navigation_graph = None
        self.lock = threading.Lock()

        # Waypoints assegnati e indice corrente
        self.assigned_waypoints = []
        self.current_waypoint_index = 0

        # Flag per indicare se il grafo è stato ricevuto
        self.graph_received = False

        # Lista per eventuali comandi in coda
        self.pending_commands = []

        self.get_logger().info(
            f"[{self.robot_namespace}] Slave simulator inizializzato con label nodo iniziale '{self.initial_node_label}'."
        )

    def publish_registration(self):
        """
        Pubblica periodicamente un messaggio di registrazione per informare il master della propria presenza.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Pubblicata registrazione.")

    def publish_heartbeat(self):
        """
        Pubblica periodicamente un messaggio di heartbeat per segnalare che questo slave è attivo.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Pubblicato heartbeat.")

    def master_heartbeat_callback(self, msg):
        """
        Callback quando riceviamo un heartbeat dal master.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Ricevuto heartbeat dal master.")

    def navigation_graph_callback(self, msg):
        """
        Callback quando riceviamo il grafo di navigazione dal master.
        Lo decodifichiamo e lo carichiamo come nx.Graph (non diretto).
        """
        try:
            graph_data = json.loads(msg.data)
            new_graph = self.load_full_graph_from_data(graph_data)
            self.get_logger().debug(f"[{self.robot_namespace}] Ricevuto grafo di navigazione.")

            with self.lock:
                if not self.graph_received:
                    self.navigation_graph = new_graph
                    self.graph_received = True

            # Pubblica lo stato "ready" senza includere la posizione iniziale
            ready_status = {
                'robot_namespace': self.robot_namespace,
                'status': 'ready',
                'error_message': '',
                'time_taken': 0.0,
                'current_waypoint': self.current_node,  # Può essere 'None' se desiderato
                'traversed_edge': []
            }
            ready_msg = String()
            ready_msg.data = json.dumps(ready_status)
            self.status_publisher.publish(ready_msg)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Errore nel decodificare il grafo di navigazione: {e}")

    def navigation_commands_callback(self, msg):
        """
        Callback quando riceviamo un comando di navigazione (waypoint).
        """
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Errore nel decodificare il comando di navigazione: {e}")
            return

        # Controllo che il grafo sia disponibile
        if not self.graph_received or self.navigation_graph is None:
            self.get_logger().warn(f"[{self.robot_namespace}] Grafo non ancora ricevuto, impossibile navigare.")
            self.publish_status("error", "Grafo non ancora disponibile.", 0.0, waypoint_data.get('label', 'N/A'), traversed_edge=[])
            return

        # Assegna il waypoint
        with self.lock:
            self.assigned_waypoints.append(waypoint_data)
            # self.get_logger().info(f"[{self.robot_namespace}] Assegnato nuovo waypoint: {waypoint_data}")

        # Avvia la simulazione della navigazione in un thread separato per evitare il blocco del main loop
        threading.Thread(target=self.simulate_navigation, args=(waypoint_data,)).start()

    def simulate_navigation(self, waypoint):
        """
        Simula la navigazione verso il waypoint specificato.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']

        with self.lock:
            current_node = self.current_node
            if current_node == label:
                # Già sul nodo di destinazione
                self.get_logger().info(f"[{self.robot_namespace}] Già sul waypoint '{label}'.")
                self.publish_status("reached", "", 0.0, label, [current_node, label])
                return

            # Recuperiamo la posizione attuale e quella di destinazione
            if current_node not in self.navigation_graph.nodes or label not in self.navigation_graph.nodes:
                error_msg = f"Nodo corrente '{current_node}' o destinazione '{label}' non presente nel grafo."
                self.get_logger().error(f"[{self.robot_namespace}] {error_msg}")
                self.publish_status("error", error_msg, 0.0, label, [])
                return

            current_pos = (self.navigation_graph.nodes[current_node]['x'],
                           self.navigation_graph.nodes[current_node]['y'])
            target_pos = (x, y)

            # Calcola la distanza Euclidea
            distance = math.hypot(target_pos[0] - current_pos[0], target_pos[1] - current_pos[1])

            # Velocità di simulazione (es. 1.31 m/s) - puoi variare
            speed = 10.31
            travel_time = distance / speed

            traversed_edge = [current_node, label]

            # Stato "traversing"
            self.publish_status("traversing", "", 0.0, label, traversed_edge)
            self.get_logger().info(
                f"[{self.robot_namespace}] Navigazione simulata verso '{label}' con tempo di {travel_time:.2f}s."
            )

            # Imposta il nodo attuale come destinazione
            self.current_node = label

        # Simula il tempo di viaggio
        time.sleep(travel_time)

        # Supponiamo successo
        nav_success = True
        if nav_success:
            self.get_logger().info(f"[{self.robot_namespace}] Raggiunto waypoint '{label}' in {travel_time:.2f}s.")
            self.publish_status("reached", "", travel_time, label, traversed_edge)
        else:
            error_msg = f"Simulazione di navigazione verso '{label}' fallita."
            self.get_logger().error(f"[{self.robot_namespace}] {error_msg}")
            self.publish_status("error", error_msg, travel_time, label, traversed_edge)

    def publish_status(self, status, error_message, time_taken, current_waypoint, traversed_edge):
        """
        Pubblica lo stato della navigazione al master.
        """
        status_data = {
            'robot_namespace': self.robot_namespace,
            'status': status,
            'error_message': error_message,
            'time_taken': time_taken,
            'current_waypoint': current_waypoint,
            'traversed_edge': traversed_edge
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)
        # self.get_logger().info(f"[{self.robot_namespace}] Stato pubblicato: {status_data}")

    def load_full_graph_from_data(self, graph_data):
        """
        Carica un grafo *non diretto* (nx.Graph) da un dizionario con nodi e archi.
        """
        G = nx.Graph()
        # Caricamento nodi
        for node in graph_data['nodes']:
            label = node['label']
            x = node['x']
            y = node['y']
            G.add_node(label, x=x, y=y)

        # Caricamento archi (chiavi: 'source', 'target', 'distance')
        for edge in graph_data['edges']:
            u = edge['source']
            v = edge['target']
            if 'distance' not in edge:
                self.get_logger().warn(f"Arco ({u}, {v}) manca dell'attributo 'distance'. Imposto a 1.0.")
            dist = edge.get('distance', 1.0)
            G.add_edge(u, v, distance=dist)
            if 'distance' not in edge:
                self.get_logger().debug(f"Arco aggiunto con distanza di default: ({u}, {v}, distance={dist})")
            else:
                self.get_logger().debug(f"Arco aggiunto: ({u}, {v}, distance={dist})")

        return G

    def run(self):
        rclpy.spin(self)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Slave Navigation Simulator')
    parser.add_argument('--robot_namespace', type=str, default='robot1', help='Namespace of the robot')
    parser.add_argument('--initial_node_label', type=str, default='node_14', help='Initial node label where the robot starts')
    parsed_args, unknown = parser.parse_known_args()

    node = SlaveNavigationSimulator(
        robot_namespace=parsed_args.robot_namespace,
        initial_node_label=parsed_args.initial_node_label
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
