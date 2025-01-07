#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import argparse
import math
from threading import Lock, Event

# Navigazione TurtleBot4
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult

import networkx as nx  # Importato per la gestione dei grafi


class SlaveState:
    """
    Classe per gestire lo stato dello slave robot.

    Attributes:
        slave_ns (str): Namespace unico dello slave (es. "robot1").
        assigned_waypoints (list): Lista dei waypoints assegnati allo slave.
        current_waypoint_index (int): Indice del prossimo waypoint da raggiungere.
        publisher (rclpy.publisher.Publisher): Publisher ROS per inviare status di navigazione.
        waiting (bool): Flag che indica se lo slave è in attesa di un nuovo waypoint.
    """
    def __init__(self, slave_ns, publisher):
        self.slave_ns = slave_ns
        self.assigned_waypoints = []
        self.current_waypoint_index = 0
        self.publisher = publisher
        self.waiting = False


class SlaveNavigationNode(Node):
    """
    Nodo ROS2 per gestire la navigazione di uno slave robot.

    Questo nodo:
    - Si registra presso il master.
    - Riceve e esegue waypoints dal master (x, y, label).
    - Monitora i heartbeat del master.
    - Pubblica lo status della navigazione.
    - Gestisce il grafo di navigazione ricevuto dal master.
    - Gestisce la notifica del primo waypoint raggiunto.
    """

    def __init__(self, robot_namespace):
        """
        Inizializza il nodo SlaveNavigationNode.

        Args:
            robot_namespace (str): Namespace unico dello slave (es. "robot1").
        """
        super().__init__('slave_navigation_node', namespace=robot_namespace)

        # Parametri dello slave
        self.robot_namespace = robot_namespace

        # Publisher per registrare lo slave al master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)

        # Publisher per inviare lo status della navigazione al master
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Publisher per inviare heartbeat al master
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)

        # Publisher per notificare il primo waypoint raggiunto
        self.first_wp_reached_pub = self.create_publisher(String, '/first_waypoint_reached', 10)

        # Sottoscrizione ai heartbeat del master
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 10
        )

        # Sottoscrizione ai heartbeat di altri slave (opzionale)
        self.slave_heartbeat_subscriber = self.create_subscription(
            String, '/slave_heartbeat', self.slave_heartbeat_callback, 10
        )

        # Sottoscrizione ai comandi di navigazione specifici per lo slave
        navigation_commands_topic = f'/{self.robot_namespace}/navigation_commands'
        self.navigation_commands_subscriber = self.create_subscription(
            String, navigation_commands_topic, self.navigation_commands_callback, 10
        )
        self.get_logger().info(
            f"[{self.robot_namespace}] Subscribed to navigation commands on topic '{navigation_commands_topic}'."
        )

        # Sottoscrizione al grafo di navigazione
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.navigation_graph_callback, 10
        )

        # Timer per pubblicare regolarmente la registrazione dello slave
        self.registration_timer = self.create_timer(1.0, self.publish_registration)

        # Timer per pubblicare regolarmente gli heartbeat
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Inizializzazione del navigator per navigazione TurtleBot4
        self.navigator = TurtleBot4Navigator()

        # Stato del master
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 15.0  # Timeout in secondi

        # Flag per la navigazione
        self.is_navigating = False

        # Stato interno dello slave
        self.slave_state = SlaveState(
            slave_ns=robot_namespace,
            publisher=self.status_publisher
        )

        # Inizializzazione del nodo corrente senza posizione iniziale
        self.current_node = None

        # Flag per la notifica del primo waypoint
        self.first_wp_notification_sent = False

        # Lock per la navigazione
        self.navigation_lock = Lock()

        self.get_logger().info(f"[{self.robot_namespace}] Slave node initialized without an initial node.")

    def publish_registration(self):
        """
        Pubblica regolarmente un messaggio di registrazione per informare il master della presenza dello slave.
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published registration.")

    def publish_heartbeat(self):
        """
        Pubblica regolarmente un messaggio di heartbeat per indicare che lo slave è attivo.
        """
        hb_msg = String()
        hb_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(hb_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published heartbeat.")

    def master_heartbeat_callback(self, msg):
        """
        Callback per gestire i heartbeat ricevuti dal master.
        Aggiorna il timestamp dell'ultimo heartbeat ricevuto.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received master heartbeat.")

    def slave_heartbeat_callback(self, msg):
        """
        Callback per gestire i heartbeat ricevuti da altri slave (opzionale).
        """
        slave_ns = msg.data.strip()
        if slave_ns != self.robot_namespace:
            self.get_logger().debug(f"[{self.robot_namespace}] Received heartbeat from slave '{slave_ns}'.")

    def navigation_commands_callback(self, msg):
        """
        Callback per gestire i comandi di navigazione ricevuti dal master.
        Decodifica il messaggio JSON e avvia la navigazione verso il waypoint.
        Il waypoint contiene almeno: {'label': str, 'x': float, 'y': float}.
        """
        try:
            waypoint_data = json.loads(msg.data)
            self.get_logger().debug(f"[{self.robot_namespace}] Decoded waypoint: {waypoint_data}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding waypoint: {e}")
            return

        # Verifica che i campi 'label', 'x' e 'y' siano presenti
        if not all(k in waypoint_data for k in ('label', 'x', 'y')):
            self.get_logger().error(f"[{self.robot_namespace}] Invalid waypoint format: {waypoint_data}")
            return

        # Log del waypoint ricevuto
        self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")

        # Stampa dettagliata dei waypoint per debugging
        self.get_logger().info(
            f"[{self.robot_namespace}] Waypoint Details - Label: {waypoint_data['label']}, X: {waypoint_data['x']}, Y: {waypoint_data['y']}"
        )

        # Aggiungi il waypoint alla lista
        self.slave_state.assigned_waypoints.append(waypoint_data)

        # Log dell'assegnazione del waypoint
        self.get_logger().info(
            f"[{self.robot_namespace}] Assigned waypoint '{waypoint_data['label']}' to navigation queue."
        )

        # Se non è già in navigazione, avvia la navigazione verso il waypoint
        if not self.is_navigating:
            self.is_navigating = True
            self.execute_navigation(waypoint_data)

    def navigation_graph_callback(self, msg):
        """
        Callback per gestire il grafo di navigazione ricevuto dal master.
        Carica il grafo e imposta il flag di ricezione.
        """
        try:
            graph_data = json.loads(msg.data)
            self.navigation_graph = self.load_full_graph_from_data(graph_data)
            self.graph_received = True
            self.get_logger().info(f"[{self.robot_namespace}] Navigation graph received and loaded.")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding navigation graph: {e}")
        except Exception as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error loading navigation graph: {e}")

    def load_full_graph_from_data(self, data):
        """
        Funzione di utilità per caricare un grafo completo da dati JSON.

        Args:
            data (dict): Dati del grafo in formato node-link.

        Returns:
            networkx.Graph: Grafo di navigazione caricato.
        """
        try:
            # Specifica esplicitamente il parametro edges per evitare avvisi e errori
            return nx.node_link_graph(data, edges="links")
        except Exception as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to load graph: {e}")
            return None

    def execute_navigation(self, waypoint):
        """
        Naviga verso un waypoint specificato usando il TurtleBot4Navigator.

        La funzione:
        - Inizia la navigazione verso le coordinate (x, y) specificate.
        - Monitora il processo di navigazione e attende il completamento.
        - Pubblica un messaggio di stato (successo o fallimento) al master dopo la navigazione.

        Args:
            waypoint (dict): Un dizionario contenente i dati del waypoint:
                - 'label': Label del waypoint.
                - 'x': Coordinata X del waypoint.
                - 'y': Coordinata Y del waypoint.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']

        # Se non ho current_node, imposta il nodo corrente al primo waypoint senza navigare
        if self.current_node is None:
            self.current_node = label
            self.get_logger().info(f"[{self.robot_namespace}] Set current_node to '{label}' without navigation.")
            self.publish_status("reached", "", 0.0, label)
            self.publish_first_waypoint_notification()
            self.is_navigating = False
            return

        # Log la navigazione
        self.get_logger().info(f"[{self.robot_namespace}] Navigating to {label} at ({x}, {y})")

        # Crea il goal pose per la navigazione
        # Correggi qui la chiamata a getPoseStamped
        # **Modifica**: Usa il parametro corretto, presumibilmente una lista completa [x, y, theta]
        goal_pose = self.navigator.getPoseStamped([x, y], 0.0)  # Passa 'rotation' come secondo argomento


        start_time = time.time()

        try:
            # Controlla se l'action server di navigazione è disponibile
            if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                error_message = f"Action server not available for {label}. Skipping this waypoint."
                self.get_logger().error(f"[{self.robot_namespace}] {error_message}")

                # Pubblica uno stato di errore
                self.publish_status("error", error_message, 0.0, label)

                return

            # Invia il goal di navigazione all'action server
            nav_future = self.navigator.startToPose(goal_pose)
            self.get_logger().debug(f"[{self.robot_namespace}] Navigation started towards '{label}'...")

            # Attende il completamento della navigazione
            rclpy.spin_until_future_complete(self, nav_future)
            nav_result = nav_future.result()  # Ottiene il risultato della navigazione

            # Calcola il tempo impiegato per la navigazione
            time_taken = time.time() - start_time

            if nav_result is None:
                # Se nessun risultato è restituito, logga un errore e pubblica uno stato di errore
                error_message = f"No result received for navigation to '{label}'."
                self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
                self.publish_status("error", error_message, 0.0, label)
                return

            if nav_result == TaskResult.SUCCEEDED:
                # Se la navigazione ha avuto successo, aggiorna il nodo corrente e pubblica uno stato di successo
                self.get_logger().info(f"[{self.robot_namespace}] Successfully reached '{label}' in {time_taken:.2f} seconds.")
                self.current_node = label
                self.publish_status("reached", "", time_taken, label)

                # Pubblica la notifica del primo waypoint raggiunto (solo una volta)
                self.publish_first_waypoint_notification()
            else:
                # Se la navigazione ha fallito, logga un errore e pubblica uno stato di errore
                error_message = f"Navigation to '{label}' failed with result code {nav_result}."
                self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
                self.publish_status("error", error_message, time_taken, label)

        except Exception as e:
            # Gestisce eventuali eccezioni durante la navigazione
            error_message = f"Exception during navigation to '{label}': {e}"
            self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
            self.publish_status("error", error_message, 0.0, label)

        finally:
            # Reset del flag di navigazione
            self.is_navigating = False

    def publish_first_waypoint_notification(self):
        """
        Pubblica una notifica una volta raggiunto il primo waypoint.
        Assicura che la notifica sia inviata solo una volta.
        """
        if not self.first_wp_notification_sent and self.current_node is not None:
            notif_data = {"robot_namespace": self.robot_namespace}
            notif_msg = String()
            notif_msg.data = json.dumps(notif_data)
            self.first_wp_reached_pub.publish(notif_msg)  # Pubblica la notifica
            self.first_wp_notification_sent = True  # Assicura che la notifica sia inviata solo una volta
            self.get_logger().info(f"[{self.robot_namespace}] Published first_waypoint_reached notification.")

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        """
        Pubblica lo status della navigazione al master.

        Args:
            status (str): Stato della navigazione ("reached", "error", etc.).
            error_message (str): Messaggio di errore, se presente.
            time_taken (float): Tempo impiegato per raggiungere il waypoint.
            current_waypoint (str): Label del waypoint attuale.
        """
        st_data = {
            'robot_namespace': self.robot_namespace,
            'status': status,
            'error_message': error_message,
            'time_taken': time_taken,
            'current_waypoint': current_waypoint
        }
        msg = String()
        msg.data = json.dumps(st_data)
        self.status_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Published status: {st_data}")

    def run_navigation_loop(self):
        """
        Esegue il loop di navigazione principale.
        """
        rclpy.spin(self)

    def destroy_node(self):
        """
        Pulisce le risorse prima di spegnere il nodo.
        """
        super().destroy_node()


def main(args=None):
    """
    Punto di ingresso principale per il nodo SlaveNavigationNode.
    """
    parser = argparse.ArgumentParser(description='Real Slave Navigation Node')
    parser.add_argument(
        '--robot_namespace',
        type=str,
        required=True,
        help='Unique namespace of the robot (e.g., "robot1").'
    )

    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)
    node = SlaveNavigationNode(
        robot_namespace=parsed_args.robot_namespace
    )

    try:
        node.run_navigation_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
