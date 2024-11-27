# multi_nav_pkg/slave_navigation_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multi_nav_pkg.msg import Waypoint, NavigationStatus
import json
import math
import time
import logging
import socket
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class SlaveNavigationNode(Node):
    def __init__(self, robot_namespace, output_dir, listen_topic='navigation_commands'):
        super().__init__(robot_namespace + '_slave_navigation_node')
        
        self.robot_namespace = robot_namespace
        self.output_dir = output_dir
        self.listen_topic = f"{robot_namespace}/navigation_commands"

        # Publisher per registrarsi come slave
        self.slave_registration_publisher = self.create_publisher(String, 'slave_registration', 10)

        # Publisher per inviare lo stato della navigazione
        self.status_publisher = self.create_publisher(NavigationStatus, 'navigation_status', 10)

        # Subscriber per ricevere i comandi di navigazione dal master
        self.navigation_commands_subscriber = self.create_subscription(
            Waypoint,
            self.listen_topic,
            self.navigation_commands_callback,
            10
        )

        # Timer per pubblicare regolarmente la registrazione
        self.registration_timer = self.create_timer(1.0, self.publish_registration)

        # Inizializza il navigatore
        self.navigator = TurtleBot4Navigator()

        self.current_waypoint = None
        self.start_time = None

        # Configurazione del logger per inviare i log a Logstash
        self.logstash_host = '192.168.1.100'  # Indirizzo IP del server Logstash
        self.logstash_port = 5000            # Porta configurata in Logstash

        # Configura il logger
        self.logger = logging.getLogger('SlaveNavigationNode')
        self.logger.setLevel(logging.DEBUG)

        # Crea un handler socket
        try:
            self.socket_handler = logging.handlers.SocketHandler(self.logstash_host, self.logstash_port)
            self.logger.addHandler(self.socket_handler)
            self.logger.debug("Socket handler for Logstash successfully added.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Logstash: {e}")

        # Override dei metodi di logging per inviare i log anche a Logstash
        self.get_logger().info = self.custom_info
        self.get_logger().warn = self.custom_warn
        self.get_logger().error = self.custom_error

        self.get_logger().info(f"[{self.robot_namespace}] Slave node initialized.")

    def custom_info(self, msg):
        self.logger.info(msg)
        super().get_logger().info(msg)

    def custom_warn(self, msg):
        self.logger.warning(msg)
        super().get_logger().warn(msg)

    def custom_error(self, msg):
        self.logger.error(msg)
        super().get_logger().error(msg)

    def publish_registration(self):
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"Published registration for {self.robot_namespace}")

    def navigation_commands_callback(self, msg):
        self.current_waypoint = msg
        self.get_logger().info(f"Received waypoint: {msg.label} at ({msg.x}, {msg.y}) with orientation {msg.orientation} radians.")
        self.execute_navigation(msg)

    def execute_navigation(self, waypoint):
        label = waypoint.label
        x = waypoint.x
        y = waypoint.y
        orientation = waypoint.orientation

        # Log della navigazione
        self.get_logger().info(f"Navigating to {label} at ({x}, {y}) with orientation {orientation} radians.")

        # Crea la pose di destinazione
        goal_pose = self.navigator.getPoseStamped([x, y], orientation)

        # Avvia la navigazione
        try:
            self.start_time = time.time()
            result = self.navigator.startToPose(goal_pose)
            if result is None:
                raise Exception("Failed to send goal.")

            # Attendi il completamento della navigazione
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Calcola il tempo impiegato
            time_taken = time.time() - self.start_time

            # Log del completamento
            self.get_logger().info(f"Reached {label} in {time_taken:.2f} seconds.")
            self.publish_status("reached", "", time_taken, label)
        
        except Exception as e:
            # Log dell'errore
            self.get_logger().error(f"Error navigating to {label}: {str(e)}")
            self.publish_status("error", str(e), 0.0, label)

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        status_msg = NavigationStatus()
        status_msg.robot_namespace = self.robot_namespace
        status_msg.status = status
        status_msg.error_message = error_message
        status_msg.time_taken = time_taken
        status_msg.current_waypoint = current_waypoint

        self.status_publisher.publish(status_msg)
        self.get_logger().info(f"Published status: {status}, Waypoint: {current_waypoint}, Time Taken: {time_taken}s, Error: {error_message}")

def main(args=None):
    rclpy.init(args=args)

    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('--robot_namespace', type=str, required=True, help='Namespace of the robot')
    parser.add_argument('--output_dir', type=str, default='/tmp', help='Directory to save subgraphs')

    args, unknown = parser.parse_known_args()

    slave_node = SlaveNavigationNode(
        robot_namespace=args.robot_namespace,
        output_dir=args.output_dir
    )

    try:
        rclpy.spin(slave_node)
    except KeyboardInterrupt:
        pass
    finally:
        slave_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
