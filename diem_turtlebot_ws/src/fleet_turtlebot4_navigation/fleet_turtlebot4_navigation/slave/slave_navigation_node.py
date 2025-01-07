#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import argparse
import math

# Navigazione TurtleBot4
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult


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
    """

    def __init__(self, robot_namespace):
        """
        Inizializza il nodo SlaveNavigationNode.

        Args:
            robot_namespace (str): Namespace unico dello slave (es. "robot1").
        """
        super().__init__('slave_navigation_node')

        # Parametri dello slave
        self.robot_namespace = robot_namespace

        # Publisher per registrare lo slave al master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)

        # Publisher per inviare lo status della navigazione al master
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Publisher per inviare heartbeat al master
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)

        # Sottoscrizione ai heartbeat del master
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 10
        )

        # Sottoscrizione ai heartbeat di altri slave (se necessario)
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

        self.get_logger().info(f"[{self.robot_namespace}] Slave node initialized.")

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
        self.get_logger().info(f"[{self.robot_namespace}] Waypoint Details - Label: {waypoint_data['label']}, X: {waypoint_data['x']}, Y: {waypoint_data['y']}")

        # Aggiungi il waypoint alla lista
        self.slave_state.assigned_waypoints.append(waypoint_data)

        # Se non è già in navigazione, avvia la navigazione verso il waypoint
        if not self.is_navigating:
            self.is_navigating = True
            self.execute_navigation(waypoint_data)

    def execute_navigation(self, waypoint):
        """
        Naviga verso il waypoint specificato (x, y) con TurtleBot4Navigator.

        Args:
            waypoint (dict): Dizionario contenente 'label', 'x', 'y'.
        """
        label = waypoint.get('label', '???')
        x = waypoint.get('x')
        y = waypoint.get('y')

        # Log della navigazione
        self.get_logger().info(
            f"[{self.robot_namespace}] Navigating to '{label}' => (x={x}, y={y})."
        )

        # Crea il goal pose con orientamento di default (0.0)
        try:
            goal_pose = self.navigator.getPoseStamped([x, y], 0.0)
            self.get_logger().debug(f"[{self.robot_namespace}] Created goal_pose: {goal_pose}")
        except Exception as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error creating goal_pose: {e}")
            self.publish_status("error", f"Error creating goal_pose: {e}", 0.0, label)
            self.is_navigating = False
            return

        start_time = time.time()

        try:
            # Verifica se l'action server è disponibile
            if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                error_message = f"Action server not available for '{label}'."
                self.get_logger().error(f"[{self.robot_namespace}] {error_message}")

                # Pubblica uno status di errore
                self.publish_status("error", error_message, 0.0, label)

                # Reset dello stato di navigazione
                self.is_navigating = False
                return

            # Avvia il task di navigazione
            nav_future = self.navigator.startToPose(goal_pose)
            self.get_logger().debug(f"[{self.robot_namespace}] Navigation started => {label}.")

        except Exception as e:
            # Gestisci eventuali eccezioni durante l'avvio della navigazione
            error_message = f"Exception while sending goal to '{label}': {e}"
            self.get_logger().error(f"[{self.robot_namespace}] {error_message}")

            # Pubblica uno status di errore
            self.publish_status("error", error_message, 0.0, label)

            # Reset dello stato di navigazione
            self.is_navigating = False
            return

        # Attendi che il task di navigazione sia completato
        rclpy.spin_until_future_complete(self, nav_future)
        nav_result = nav_future.result()

        # Calcola il tempo impiegato per la navigazione
        time_taken = time.time() - start_time

        # Gestisci il risultato del task di navigazione
        if nav_result is None:
            # Nessun risultato dall'action server
            error_message = f"No result => '{label}'."
            self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
            self.publish_status("error", error_message, time_taken, label)
            self.is_navigating = False
            return

        if nav_result == TaskResult.SUCCEEDED:
            # Log del successo e pubblica uno status di successo
            self.get_logger().info(
                f"[{self.robot_namespace}] Reached '{label}' successfully in {time_taken:.2f}s."
            )
            self.publish_status("reached", "", time_taken, label)

            # Passa al prossimo waypoint se presente
            self.slave_state.current_waypoint_index += 1
            if self.slave_state.current_waypoint_index < len(self.slave_state.assigned_waypoints):
                next_wpt = self.slave_state.assigned_waypoints[self.slave_state.current_waypoint_index]
                self.get_logger().info(f"[{self.robot_namespace}] Proceeding to next waypoint: {next_wpt}")
                self.execute_navigation(next_wpt)
            else:
                # Se non ci sono altri waypoints, resetta lo stato di navigazione
                self.is_navigating = False
                self.get_logger().info(
                    f"[{self.robot_namespace}] All waypoints have been navigated."
                )

        else:
            # Log del fallimento e pubblica uno status di errore
            error_message = f"Navigation to '{label}' failed. Code={nav_result}"
            self.get_logger().error(f"[{self.robot_namespace}] {error_message}")
            self.publish_status("error", error_message, time_taken, label)
            self.is_navigating = False

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


def main(args=None):
    """
    Punto di ingresso principale per il nodo SlaveNavigationNode.
    """
    parser = argparse.ArgumentParser(description='Minimal Slave Navigation Node')
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
