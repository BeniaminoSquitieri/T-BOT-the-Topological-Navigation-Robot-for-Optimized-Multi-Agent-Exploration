# master_callbacks.py

import json
import math
from std_msgs.msg import String

from .graph_utils import partition_graph_wrapper
from .path_calculation import calculate_dcpp_route, orientation_rad_to_str
from .slave_state import SlaveState
from .utils import find_node_from_position

class MasterCallbacks:
    """
    A mixin class that encapsulates all callback functions for the MasterNavigationNode.
    This promotes modularity by separating callback logic from the main node implementation.
    """
    
    def publish_navigation_graph(self):
        """
        Publish the navigation graph to the '/navigation_graph' topic.
        This allows slaves to know the topology of the environment.
        """
        graph_msg = String()
        graph_data = {
            'nodes': [
                {
                    'label': node,
                    'x': data['x'],
                    'y': data['y'],
                    'orientation': data.get('orientation', 0.0)
                } for node, data in self.full_graph.nodes(data=True)
            ],
            'edges': [
                {
                    'from': u,
                    'to': v,
                    'weight': data.get('weight', 1.0)
                } for u, v, data in self.full_graph.edges(data=True)
            ]
        }
        graph_msg.data = json.dumps(graph_data)
        self.graph_publisher.publish(graph_msg)
        self.get_logger().debug("Navigation graph published.")
    
    def slave_registration_callback(self, msg):
        """
        Callback function triggered when a slave sends a registration message.
        This message informs the master about the slave's existence.

        Args:
            msg (String): ROS message containing the slave's namespace.
        """
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns not in self.slaves:
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            self.slaves[slave_ns] = slave_state
            self.get_logger().info(f"Registered new slave: {slave_ns}")
        else:
            self.slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"Updated last seen time for slave: {slave_ns}")
    
    def initial_position_callback(self, msg):
        """
        Callback function triggered when a slave sends its initial position.
        This message contains the slave's coordinates and orientation.

        Args:
            msg (String): ROS message containing the initial position in JSON format.
        """
        try:
            with open("/tmp/initial_position_messages.log", "a") as f:
                f.write(f"Received message: {msg.data}\n")
        except Exception as e:
            self.get_logger().error(f"Failed to write received message to log: {e}")

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

        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time
            slave.initial_x = initial_x
            slave.initial_y = initial_y
            slave.initial_orientation = orientation
            slave.current_node = find_node_from_position(self.full_graph, initial_x, initial_y)

            if slave.current_node is None:
                self.get_logger().error(f"Could not find a node matching the initial position of slave {slave_ns}")
            else:
                self.get_logger().info(
                    f"Received initial position from {slave_ns}: "
                    f"({initial_x}, {initial_y}) on node {slave.current_node} with orientation {orientation}"
                )
            self.waypoint_manager.repartition_and_assign_waypoints()
        else:
            self.get_logger().info(f"Initial position received for unregistered slave {slave_ns}, registering now.")
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            slave_state.initial_x = initial_x
            slave_state.initial_y = initial_y
            slave_state.initial_orientation = orientation
            slave_state.current_node = find_node_from_position(self.full_graph, initial_x, initial_y)

            if slave_state.current_node is None:
                self.get_logger().error(f"Could not find a node matching the initial position of slave {slave_ns}")
            else:
                self.get_logger().info(f"Created publisher for slave: {slave_ns} at node {slave_state.current_node}")

            self.slaves[slave_ns] = slave_state
            self.waypoint_manager.repartition_and_assign_waypoints()
    
    def navigation_status_callback(self, msg):
        """
        Processes navigation status updates from slaves.

        Args:
            msg (String): JSON-formatted string containing the navigation status of a slave.
        """
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

        self.get_logger().info(f"Decoded navigation status data: {data}")

        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time

            if status == "reached":
                from_node = slave.current_node
                to_node = current_waypoint
                edge = (from_node, to_node)

                if edge in self.occupied_edges:
                    self.occupied_edges.remove(edge)
                    self.get_logger().info(f"Edge {edge} is now free.")
                else:
                    self.get_logger().warn(f"Edge {edge} was not occupied. Check logic.")

                slave.current_node = current_waypoint
                slave.current_waypoint_index += 1
                slave.waiting = False

                self.waypoint_manager.assign_next_waypoint(slave_ns)
                self.waypoint_manager.assign_waiting_slaves()

            elif status == "error":
                self.get_logger().error(f"Slave {slave_ns} encountered an error: {error_message}")

                from_node = slave.current_node
                to_node = current_waypoint
                edge = (from_node, to_node)
                if edge in self.occupied_edges:
                    self.occupied_edges.remove(edge)
                    self.get_logger().info(f"Edge {edge} is now free due to error.")

                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removed slave {slave_ns} due to error.")

                self.waypoint_manager.repartition_and_assign_waypoints()
        else:
            self.get_logger().warn(f"Received status from unknown slave {slave_ns}.")
    
    def timer_callback(self):
        """
        Periodic callback function triggered at intervals defined by `check_interval`.

        This method performs two primary tasks:
        1. Checks if any slaves have timed out due to inactivity (exceeding the timeout period).
        2. Attempts to reassign waypoints to slaves that are waiting for free edges.

        Purpose:
        - Ensures that inactive slaves are removed, and their resources (edges) are freed.
        - Improves efficiency by attempting to resolve waiting states for active slaves.

        This function is periodically called by a ROS timer.
        """
        # Log debug information for the timer execution
        self.get_logger().debug("Timer callback triggered.")
        
        # Check for and remove inactive slaves (exceeding timeout period)
        self.check_slaves_timeout()
        
        # Attempt to assign waypoints to slaves that are waiting for an edge to become free
        self.waypoint_manager.assign_waiting_slaves()
