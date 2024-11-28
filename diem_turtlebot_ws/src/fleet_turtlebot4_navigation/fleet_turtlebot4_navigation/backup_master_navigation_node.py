#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import networkx as nx
import os
import time
from .graph_partitioning import load_full_graph, partition_graph, save_subgraphs

class BackupMasterNavigationNode(Node):
    def __init__(self):
        super().__init__('backup_master_navigation_node')

        # Declare parameters
        self.declare_parameter('graph_path', '')
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('heartbeat_timeout', 3.0)

        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value

        # Load the full graph
        self.full_graph = load_full_graph(self.graph_path)
        self.get_logger().info("Backup Master node loaded the graph.")

        # Subscriptions and Publishers
        self.slave_registration_subscriber = self.create_subscription(
            String,
            'slave_registration',
            self.slave_registration_callback,
            10
        )

        # Publisher for navigation commands to each slave
        self.navigation_command_publishers = {}

        # Dictionary to track active slaves: {namespace: last_seen_time}
        self.active_slaves = {}

        # Dictionary to track assigned waypoints: {namespace: [waypoints]}
        self.assigned_waypoints = {}

        # Dictionary to track if the slave is navigating: {namespace: bool}
        self.slave_busy = {}

        # Set to track assigned waypoints globally (to avoid duplication)
        self.assigned_waypoints_set = set()

        # Timer for periodic slave checks
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Subscriber to receive feedback from slaves
        self.navigation_status_subscriber = self.create_subscription(
            String,
            'navigation_status',
            self.navigation_status_callback,
            10
        )

        # Subscriber for heartbeat from the primary master
        self.heartbeat_subscriber = self.create_subscription(
            String,
            'master_heartbeat',
            self.heartbeat_callback,
            10
        )

        # Variable to track the last heartbeat received
        self.last_heartbeat = None

        # Timer to check heartbeat
        self.heartbeat_check_timer = self.create_timer(1.0, self.check_master_alive)

        # Flag to avoid multiple activations
        self.is_active_master = False

        # Publisher for heartbeat (becomes active if necessary)
        self.heartbeat_publisher = None
        self.heartbeat_timer = None

        # Flag to indicate whether the graph has been partitioned and waypoints assigned
        self.partitioning_done = False

    def heartbeat_callback(self, msg):
        if msg.data.strip() == "alive":
            self.last_heartbeat = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().debug("Received heartbeat from primary master.")

    def check_master_alive(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_heartbeat is None:
            # Has not received any heartbeat yet
            return
        if current_time - self.last_heartbeat > self.heartbeat_timeout:
            if not self.is_active_master:
                self.get_logger().warn("Primary master is down. Becoming active master.")
                self.is_active_master = True
                self.activate_master_functions()

    def activate_master_functions(self):
        # Publisher for heartbeat (now becomes master)
        self.heartbeat_publisher = self.create_publisher(String, 'master_heartbeat', 10)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Start the primary master functions
        # This includes partitioning and assigning waypoints
        self.repartition_and_assign_waypoints()

    def publish_heartbeat(self):
        heartbeat_msg = String()
        heartbeat_msg.data = "alive"
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug("Backup Master published heartbeat.")

    def slave_registration_callback(self, msg):
        if not self.is_active_master:
            return  # Only the active master handles registrations
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().nanoseconds / 1e9  # Current time in seconds
        new_slave = False
        if slave_ns not in self.active_slaves:
            new_slave = True
        self.active_slaves[slave_ns] = current_time
        self.get_logger().info(f"Received registration from slave: {slave_ns}")
        # Initialize data structures for the new slave
        if slave_ns not in self.assigned_waypoints:
            self.assigned_waypoints[slave_ns] = []
            self.slave_busy[slave_ns] = False

        # If partitioning is already done and a new slave has registered, repartition
        if self.partitioning_done and new_slave:
            self.get_logger().info("New slave registered. Repartitioning graph and reassigning waypoints.")
            self.repartition_and_assign_waypoints()

    def timer_callback(self):
        if not self.is_active_master:
            return  # Only the active master performs these operations

        # Current time
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Remove slaves that have timed out
        removed_slaves = []
        for slave_ns in list(self.active_slaves.keys()):
            last_seen = self.active_slaves[slave_ns]
            if current_time - last_seen > self.heartbeat_timeout:
                removed_slaves.append(slave_ns)
                del self.active_slaves[slave_ns]
                self.get_logger().warn(f"Slave {slave_ns} timed out and removed from active slaves.")
                # Remove publisher if exists
                if slave_ns in self.navigation_command_publishers:
                    del self.navigation_command_publishers[slave_ns]
                # Remove assigned waypoints
                if slave_ns in self.assigned_waypoints:
                    # Release the assigned waypoints
                    for wp in self.assigned_waypoints[slave_ns]:
                        wp_id = (wp['label'], wp['x'], wp['y'])
                        self.assigned_waypoints_set.discard(wp_id)
                    del self.assigned_waypoints[slave_ns]
                if slave_ns in self.slave_busy:
                    del self.slave_busy[slave_ns]

        # If slaves have been removed, repartition and reassign
        if removed_slaves:
            self.get_logger().info("Active slaves changed. Repartitioning graph and reassigning waypoints.")
            self.repartition_and_assign_waypoints()

        # Assign waypoints to available slaves
        for slave_ns in self.active_slaves:
            if not self.slave_busy.get(slave_ns, False):
                if self.assigned_waypoints.get(slave_ns, []):
                    next_waypoint = self.assigned_waypoints[slave_ns].pop(0)
                    # Remove the waypoint from the global assignment set
                    wp_id = (next_waypoint['label'], next_waypoint['x'], next_waypoint['y'])
                    self.assigned_waypoints_set.discard(wp_id)
                    self.send_waypoint(slave_ns, next_waypoint)

    def repartition_and_assign_waypoints(self):
        num_slaves = len(self.active_slaves)
        if num_slaves == 0:
            self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
            return

        # Clear assigned waypoints and assigned waypoints set
        self.assigned_waypoints = {}
        self.assigned_waypoints_set.clear()
        # Note: Do not reset self.slave_busy here

        # Partition the graph
        subgraphs = partition_graph(self.full_graph, num_slaves)
        self.get_logger().info(f"Partitioned the graph into {num_slaves} subgraphs.")

        # Save the subgraphs to files
        subgraph_paths = save_subgraphs(subgraphs, self.output_dir)
        self.get_logger().info(f"Saved subgraphs to {subgraph_paths}")

        # Assign each subgraph to a slave
        slaves = sorted(self.active_slaves.keys())  # Sort for deterministic assignment
        for idx, slave_ns in enumerate(slaves):
            subgraph_file = subgraph_paths[idx]
            waypoints = self.extract_waypoints(subgraph_file)
            # Assign waypoints to the slave
            self.assigned_waypoints[slave_ns] = waypoints.copy()
            # Update the global set of assigned waypoints
            for wp in waypoints:
                wp_id = (wp['label'], wp['x'], wp['y'])
                self.assigned_waypoints_set.add(wp_id)
            # Maintain current busy status
            if slave_ns not in self.slave_busy:
                self.slave_busy[slave_ns] = False
            self.get_logger().info(f"Assigned {len(waypoints)} waypoints to {slave_ns}")

        self.partitioning_done = True  # Set the partitioning flag

    def extract_waypoints(self, subgraph_file_path):
        # Same as in master node
        with open(subgraph_file_path, 'r') as f:
            data = json.load(f)

        waypoints = []
        for node in data['nodes']:
            waypoint = {
                'label': node['label'],
                'x': node['x'],
                'y': node['y'],
                'orientation': node.get('orientation', 0.0)
            }
            waypoints.append(waypoint)
        return waypoints

    def send_waypoint(self, slave_ns, waypoint):
        # Create or retrieve the publisher for the slave
        if slave_ns not in self.navigation_command_publishers:
            publisher = self.create_publisher(String, f"{slave_ns}/navigation_commands", 10)
            self.navigation_command_publishers[slave_ns] = publisher
            self.get_logger().info(f"Created publisher for slave: {slave_ns}")
        else:
            publisher = self.navigation_command_publishers[slave_ns]

        # Create the message as JSON
        waypoint_msg = {
            'label': waypoint['label'],
            'x': waypoint['x'],
            'y': waypoint['y'],
            'orientation': waypoint['orientation']
        }

        msg = String()
        msg.data = json.dumps(waypoint_msg)

        # Publish the waypoint
        publisher.publish(msg)
        self.get_logger().info(f"Sent waypoint to {slave_ns}: {waypoint_msg}")

        # Mark the slave as busy
        self.slave_busy[slave_ns] = True

    def navigation_status_callback(self, msg):
        if not self.is_active_master:
            return  # Only the active master handles feedback
        data = json.loads(msg.data)
        slave_ns = data['robot_namespace']
        status = data['status']
        current_waypoint = data['current_waypoint']
        time_taken = data['time_taken']
        error_message = data['error_message']

        self.get_logger().info(f"Received status from {slave_ns}: {status}, Waypoint: {current_waypoint}, Time Taken: {time_taken}s, Error: {error_message}")

        if status == "reached":
            # Mark the slave as available for the next waypoint
            self.slave_busy[slave_ns] = False
        elif status == "error":
            # Handle errors, e.g., by reassigning waypoints
            self.get_logger().error(f"Slave {slave_ns} encountered an error: {error_message}")
            self.slave_busy[slave_ns] = False
            # Optionally, repartition and reassign waypoints
            self.repartition_and_assign_waypoints()

def main(args=None):
    rclpy.init(args=args)
    node = BackupMasterNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#ros2 run fleet_turtlebot4_navigation backup_master_navigation_node --ros-args -p graph_path:=/path/to/your/graph.json -p output_dir:=/tmp
