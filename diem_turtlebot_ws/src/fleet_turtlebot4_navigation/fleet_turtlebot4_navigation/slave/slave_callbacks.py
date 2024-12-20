# src/fleet_turtlebot4_navigation/fleet_turtlebot4_navigation/slave/slave_callbacks.py

import json
import time  # Import necessario per utilizzare time.time()
from std_msgs.msg import String
from fleet_turtlebot4_navigation.master.slave_state import SlaveState  # Assumiamo che SlaveState sia definito nel master

class SlaveCallbacks:
    """
    Mixin class to encapsulate all callback functions for the SlaveNavigationSimulator node.
    """

    def publish_heartbeat(self):
        """
        Publish a heartbeat message to indicate that this slave is active.
        """
        current_time = time.time()
        heartbeat_msg = {
            'robot_namespace': self.robot_namespace,
            'status': 'alive',
            'timestamp': current_time  # Utilizza un float serializzabile
        }
        msg = String()
        msg.data = json.dumps(heartbeat_msg)
        self.heartbeat_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published heartbeat: {heartbeat_msg}")

    def publish_registration(self):
        """
        Periodically publish a registration message to inform the master of this slave's existence.
        """
        current_time = time.time()
        registration_msg = {
            'robot_namespace': self.robot_namespace,
            'status': 'registered',
            'timestamp': current_time  # Utilizza un float serializzabile
        }
        msg = String()
        msg.data = json.dumps(registration_msg)
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published registration: {registration_msg}")

    def master_heartbeat_callback(self, msg):
        """
        Callback invoked when a heartbeat from the master is received.
        Marks the master as alive and updates the timestamp of the last heartbeat.
        """
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received master heartbeat.")

    def slave_heartbeat_callback(self, msg):
        """
        Callback invoked when a heartbeat from another slave is received.
        Updates the state of the active slaves.
        """
        slave_ns = msg.data.strip()
        current_time = time.time()
        if slave_ns != self.robot_namespace:
            with self.lock:
                if slave_ns not in self.active_slaves:
                    self.active_slaves[slave_ns] = SlaveState(slave_ns)
                    self.get_logger().info(f"[{self.robot_namespace}] Detected new slave: {slave_ns}")
                self.active_slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"[{self.robot_namespace}] Received heartbeat from slave {slave_ns}.")
