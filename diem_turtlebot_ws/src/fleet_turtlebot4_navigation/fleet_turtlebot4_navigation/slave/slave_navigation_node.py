#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import argparse
import math
import time

# Classe che fornisce le API di navigazione reale
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult


class SlaveNavigationNode(Node):
    """
    Nodo ROS2 'reale' che si comporta come lo 'slave':
    - Pubblica la propria presenza (registrazione, heartbeat)
    - Riceve comandi di navigazione (in formato JSON) su /navigation_commands
    - Usa TurtleBot4Navigator per raggiungere la destinazione
    - Pubblica lo stato della navigazione su /navigation_status
    - È completamente indipendente dal nodo di partenza o da un grafo di navigazione.
    """

    def __init__(self, robot_namespace):
        # Inizializza il nodo ROS con un certo nome e namespace
        super().__init__('slave_navigation_node', namespace=robot_namespace)

        self.robot_namespace = robot_namespace

        # --------------------------------------------------
        # PUBLISHER e SUBSCRIBER
        # --------------------------------------------------
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)
        self.navigation_commands_subscriber = self.create_subscription(
            String, '/navigation_commands',
            self.navigation_commands_callback,
            10
        )
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat',
            self.master_heartbeat_callback,
            10
        )

        # --------------------------------------------------
        # TIMER
        # --------------------------------------------------
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # --------------------------------------------------
        # VARIABILI DI STATO
        # --------------------------------------------------
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 5.0

        # Inizializza il TurtleBot4Navigator per la navigazione reale
        self.navigator = TurtleBot4Navigator()

        # Messaggio di avvio
        self.get_logger().info(
            f"[{self.robot_namespace}] Slave navigation node (REALE) inizializzato e indipendente dal nodo di partenza."
        )

    # --------------------------------------------------
    # PUBBLICAZIONE REGISTRAZIONE
    # --------------------------------------------------
    def publish_registration(self):
        """
        Pubblica periodicamente un messaggio di registrazione (slave presente).
        """
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Pubblicata registrazione su /slave_registration.")

    # --------------------------------------------------
    # PUBBLICAZIONE HEARTBEAT
    # --------------------------------------------------
    def publish_heartbeat(self):
        """
        Pubblica periodicamente un messaggio di heartbeat (slave attivo).
        """
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Pubblicato heartbeat su /slave_heartbeat.")

    # --------------------------------------------------
    # CALLBACK HEARTBEAT MASTER
    # --------------------------------------------------
    def master_heartbeat_callback(self, msg):
        """
        Gestisce la ricezione dell'heartbeat del master.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Ricevuto heartbeat dal master. (master_alive=True)")

    # --------------------------------------------------
    # CALLBACK COMANDI DI NAVIGAZIONE
    # --------------------------------------------------
    def navigation_commands_callback(self, msg):
        """
        Gestisce i comandi di navigazione ricevuti dal master, in formato JSON:
        {
          "label": <str>,
          "x": <float>,
          "y": <float>
        }
        """
        self.get_logger().debug(f"[{self.robot_namespace}] Ricevuto su /navigation_commands: {msg.data}")
        try:
            waypoint_data = json.loads(msg.data)
            self.get_logger().debug(f"[{self.robot_namespace}] Decodificato JSON: {waypoint_data}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Errore nel decodificare JSON di navigazione: {e}")
            return

        self.get_logger().info(f"[{self.robot_namespace}] Comando di navigazione ricevuto: {waypoint_data}")
        self.execute_navigation(waypoint_data)

    # --------------------------------------------------
    # NAVIGAZIONE REALE
    # --------------------------------------------------
    def execute_navigation(self, waypoint):
        """
        Usa TurtleBot4Navigator per muovere il robot a (x, y).
        Ignora qualsiasi informazione di partenza. È indipendente dal nodo di partenza.
        """
        label = waypoint.get('label', 'N/A')
        x = waypoint.get('x')
        y = waypoint.get('y')

        if x is None or y is None:
            err = f"Waypoint '{label}' mancante di coordinate x o y."
            self.get_logger().error(f"[{self.robot_namespace}] {err}")
            self.publish_status("error", err, 0.0, label, traversed_edge=[])
            return

        self.get_logger().info(
            f"[{self.robot_namespace}] Inizio navigazione REALE verso '{label}' a ({x}, {y})."
        )
        print(f"[{self.robot_namespace}] [REALE] Navigating to waypoint '{label}' at ({x}, {y}).")

        # Crea la goal pose (non usiamo orientamento)
        goal_pose = self.navigator.getPoseStamped([x, y])

        # Attende che l'action server sia disponibile
        if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            err_msg = f"Action server non disponibile per waypoint '{label}'."
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, traversed_edge=[])
            return

        # Avvia la navigazione
        nav_future = self.navigator.startToPose(goal_pose)
        self.get_logger().debug(f"[{self.robot_namespace}] Avviata navigazione verso '{label}'. Attendiamo completamento...")

        # Blocca finché il task non è completo
        rclpy.spin_until_future_complete(self, nav_future)

        # Risultato
        nav_result = nav_future.result()
        if nav_result is None:
            err_msg = f"Nessun risultato (nav_result is None) per waypoint '{label}'."
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, traversed_edge=[])
            return

        if nav_result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"[{self.robot_namespace}] Waypoint '{label}' raggiunto con successo!")
            self.publish_status("reached", "", 0.0, label, traversed_edge=[])
        else:
            err_msg = f"Navigazione fallita verso '{label}', codice {nav_result}."
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, traversed_edge=[])

    # --------------------------------------------------
    # PUBBLICAZIONE STATO
    # --------------------------------------------------
    def publish_status(self, status, error_message, time_taken, current_waypoint, traversed_edge):
        """
        Pubblica lo stato su /navigation_status, come nella versione simulata.
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
        self.get_logger().debug(f"[{self.robot_namespace}] Stato pubblicato: {status_data}")

    # --------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------
    def run(self):
        rclpy.spin(self)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Slave Navigation Node (REALE), indipendente dal nodo di partenza.')
    parser.add_argument(
        '--robot_namespace', type=str, default='robot1',
        help='Namespace del robot (es: "robot1")'
    )
    parsed_args, unknown = parser.parse_known_args()

    node = SlaveNavigationNode(robot_namespace=parsed_args.robot_namespace)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
