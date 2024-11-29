#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import argparse
import math
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult

class SlaveNavigationNode(Node):
    def __init__(self, robot_namespace, initial_x, initial_y, initial_orientation_str):
        # Inizializza il nodo senza specificare il namespace
        super().__init__('slave_navigation_node')

        self.robot_namespace = robot_namespace
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.initial_orientation_str = initial_orientation_str
        self.initial_orientation = self.orientation_conversion(initial_orientation_str)

        # Publisher per registrare lo slave con il master
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)

        # Publisher per inviare la posizione iniziale al master
        self.initial_position_publisher = self.create_publisher(String, '/slave_initial_positions', 10)

        # Subscriber per ricevere i comandi di navigazione dal master
        self.navigation_commands_subscriber = self.create_subscription(
            String,
            'navigation_commands',
            self.navigation_commands_callback,
            10
        )

        # Publisher per inviare lo stato della navigazione al master
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)

        # Timer per pubblicare regolarmente i messaggi di registrazione
        self.registration_timer = self.create_timer(1.0, self.publish_registration)

        # Pubblica la posizione iniziale una volta all'avvio
        self.publish_initial_position()

        # Inizializza la classe TurtleBot4Navigator senza parametri aggiuntivi
        self.navigator = TurtleBot4Navigator()

        # Log dell'inizializzazione dello slave
        self.get_logger().info(f"[{self.robot_namespace}] Slave node initialized at ({self.initial_x}, {self.initial_y}) with orientation {self.initial_orientation_str} ({self.initial_orientation} radians).")

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
        initial_position = {
            'robot_namespace': self.robot_namespace,
            'x': self.initial_x,
            'y': self.initial_y,
            'orientation': self.initial_orientation_str  # Invia l'orientamento come stringa
        }
        msg = String()
        msg.data = json.dumps(initial_position)
        self.initial_position_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Published initial position: {initial_position}")

    def navigation_commands_callback(self, msg):
        """
        Callback funzione attivata quando viene ricevuto un nuovo waypoint dal master.
        """
        waypoint_data = json.loads(msg.data)
        self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")
        self.execute_navigation(waypoint_data)

    def execute_navigation(self, waypoint):
        """
        Utilizza il TurtleBot4Navigator per muovere il robot verso il waypoint specificato.
        """
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']
        orientation_str = waypoint['orientation']
        orientation = self.orientation_conversion(orientation_str)

        # Log del compito di navigazione
        self.get_logger().info(f"[{self.robot_namespace}] Navigating to {label} at ({x}, {y}) with orientation {orientation_str} ({orientation} radians).")

        # Crea un goal pose usando il navigator
        goal_pose = self.navigator.getPoseStamped([x, y], orientation)

        # Inizia il tempo per la navigazione
        self.start_time = time.time()

        try:
            # Verifica se l'action server è disponibile
            if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                error_message = f"Action server not available for {label}."
                self.get_logger().error(error_message)
                self.publish_status("error", error_message, 0.0, label)
                return

            # Inizia la navigazione verso il goal pose
            self.navigator.startToPose(goal_pose)
        except Exception as e:
            # Se si verifica un'eccezione, logga un errore e pubblica lo stato al master
            error_message = f"Exception occurred while sending goal to {label}: {e}"
            self.get_logger().error(error_message)
            self.publish_status("error", error_message, 0.0, label)
            return

        # Attende fino al completamento della navigazione
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            # Opzionalmente, puoi aggiungere controlli per preemption o timeout qui

        # Calcola il tempo impiegato per la navigazione
        time_taken = time.time() - self.start_time

        # Controlla il risultato della navigazione
        nav_result = self.navigator.getResult()

        if nav_result == TaskResult.SUCCEEDED:
            # Navigazione riuscita
            self.get_logger().info(f"[{self.robot_namespace}] Reached {label} in {time_taken:.2f} seconds.")
            self.publish_status("reached", "", time_taken, label)
        else:
            # Navigazione fallita
            error_message = f"Navigation to {label} failed with result code {nav_result}."
            self.get_logger().error(error_message)
            self.publish_status("error", error_message, time_taken, label)

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

    def orientation_conversion(self, orientation_str):
        """
        Converte un orientamento stringa in radianti.

        Args:
            orientation_str (str): Orientamento come stringa ('NORTH', 'EAST', 'SOUTH', 'WEST').

        Returns:
            float: Orientamento in radianti.
        """
        orientation_map = {
            "NORTH": 0.0,
            "EAST": -math.pi / 2,
            "SOUTH": math.pi,
            "WEST": math.pi / 2
        }
        return orientation_map.get(orientation_str.upper(), 0.0)

def main(args=None):
    parser = argparse.ArgumentParser(description='Slave Navigation Node using TurtleBot4')
    parser.add_argument('--robot_namespace', type=str, required=True, help='Unique namespace of the robot (e.g., robot_1)')
    parser.add_argument('--initial_x', type=float, required=True, help='Initial x coordinate')
    parser.add_argument('--initial_y', type=float, required=True, help='Initial y coordinate')
    parser.add_argument('--initial_orientation', type=str, required=True, help='Initial orientation (NORTH, EAST, SOUTH, WEST)')

    # Parse gli argomenti passati da linea di comando
    parsed_args, unknown = parser.parse_known_args()

    # Inizializza ROS 2
    rclpy.init(args=args)

    # Crea l'istanza del nodo SlaveNavigationNode
    node = SlaveNavigationNode(
        robot_namespace=parsed_args.robot_namespace,
        initial_x=parsed_args.initial_x,
        initial_y=parsed_args.initial_y,
        initial_orientation_str=parsed_args.initial_orientation
    )

    try:
        # Mantiene il nodo attivo e in ascolto dei callback
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Distrugge il nodo e ferma ROS 2
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
