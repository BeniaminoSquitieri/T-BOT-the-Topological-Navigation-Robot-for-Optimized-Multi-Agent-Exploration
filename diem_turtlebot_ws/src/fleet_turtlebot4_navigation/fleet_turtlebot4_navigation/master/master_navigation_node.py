#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .master_callbacks import MasterCallbacks
from .heartbeat_manager import HeartbeatManager
from .waypoint_manager import WaypointManager
from .graph_utils import load_full_graph
from .slave_state import SlaveState


class MasterNavigationNode(Node, MasterCallbacks):
    """
    MasterNavigationNode is a ROS2 node responsible for managing a fleet of slave robots.
    
    This node performs the following key functions:
      - Loads and publishes the navigation graph.
      - Registers and tracks slave robots.
      - Computes and assigns waypoints to slaves.
      - Monitors slave heartbeats to detect and handle inactive slaves.
      - Manages the occupancy of edges in the navigation graph to prevent task conflicts.
    
    The node inherits from both `Node` (a fundamental ROS2 class) and `MasterCallbacks` 
    (which provides callback methods for handling slave interactions and navigation tasks).
    
    Attributes:
        graph_path (str): Path to the JSON file containing the navigation graph.
        check_interval (float): Interval in seconds for periodic checks (e.g., slave timeouts).
        timeout (float): Time in seconds after which a slave is considered inactive if no heartbeat is received.
        full_graph (nx.MultiGraph): The complete navigation graph loaded from the JSON file.
        graph_publisher (Publisher): Publisher for broadcasting the navigation graph.
        graph_timer (Timer): Timer to periodically publish the navigation graph.
        slave_registration_subscriber (Subscriber): Subscriber for receiving slave registration messages.
        navigation_status_subscriber (Subscriber): Subscriber for receiving navigation status updates from slaves.
        heartbeat_manager (HeartbeatManager): Manages heartbeat publishing for the master node.
        waypoint_manager (WaypointManager): Manages waypoint assignments to slaves.
        timer (Timer): Timer for performing periodic maintenance tasks.
        slaves (dict): Dictionary mapping slave namespaces to their respective `SlaveState` instances.
        partitioning_done (bool): Flag indicating whether graph partitioning has been completed.
        occupied_edges (set): Set of edges currently assigned to slaves to prevent overlapping tasks.
        edge_occupants (dict): Dictionary mapping occupied edges to the slave namespaces that occupy them.
            Queste due variabili lavorano in sinergia per garantire una gestione efficiente e senza conflitti dei compiti di navigazione all'interno della flotta di robot. Assicurandoti 
            che gli edge siano correttamente assegnati e gestiti, puoi prevenire problemi come l'assegnazione di edge fittizi o sovrapposti, migliorando la robustezza e l'affidabilità del sistema 
            di navigazione.
        edge_last_visit_time (dict): Dictionary tracking the last time each edge was traversed (optional).
    """
    
    def __init__(self):
        """
        Initializes the MasterNavigationNode.
        
        Steps performed during initialization:
          1. Initializes the ROS2 node with the name 'master_navigation_node'.
          2. Initializes the MasterCallbacks class to set up callback methods.
          3. Declares and retrieves ROS2 parameters: graph_path, check_interval, and timeout.
          4. Validates the existence of the navigation graph file.
          5. Loads the navigation graph using `load_full_graph`.
          6. Sets up publishers and subscribers for graph distribution and slave management.
          7. Initializes HeartbeatManager and WaypointManager for managing heartbeats and waypoints.
          8. Sets up a periodic timer for maintenance tasks such as checking slave timeouts and assigning waypoints.
          9. Initializes tracking structures for slaves and edge occupancy.
          10. Resets occupied edges and occupants to ensure a clean start.
          11. Subscribes to '/first_waypoint_reached' to handle first waypoint notifications from slaves.
          
        Raises:
            FileNotFoundError: If the navigation graph file does not exist at the specified path.
        """
        # Initialize the ROS2 node with the name 'master_navigation_node'
        Node.__init__(self, 'master_navigation_node')
        
        # Initialize the MasterCallbacks class to set up callback methods
        MasterCallbacks.__init__(self)

        # ---------------------------
        # Parameter Declaration and Retrieval
        # ---------------------------
        # Declare a ROS2 parameter 'graph_path' with a default value pointing to the navigation graph JSON file
        self.declare_parameter(
            'graph_path',
            '/home/utente/percorso_al_file/result_graph.json'
        )
        # Declare a ROS2 parameter 'check_interval' with a default value of 2.0 seconds
        self.declare_parameter('check_interval', 2.0)
        # Declare a ROS2 parameter 'timeout' with a default value of 150.0 seconds
        self.declare_parameter('timeout', 150.0)
    
        # Retrieve the value of 'graph_path' parameter as a string
        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        # Retrieve the value of 'check_interval' parameter as a double (float)
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        # Retrieve the value of 'timeout' parameter as a double (float)
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
    
        # ---------------------------
        # Navigation Graph Validation and Loading
        # ---------------------------
        # Check if the navigation graph file exists at the specified path
        if not os.path.exists(self.graph_path):
            # Log an error message if the graph file is not found
            self.get_logger().error(f"Graph file not found at {self.graph_path}")
            # Raise a FileNotFoundError to halt execution
            raise FileNotFoundError(f"Graph file not found at {self.graph_path}")
    
        # Load the navigation graph as an undirected MultiGraph using the provided utility function
        self.full_graph = load_full_graph(self.graph_path)
        # Log an informational message indicating successful graph loading
        self.get_logger().info(f"Loaded undirected graph from {self.graph_path}.")
    
        # ---------------------------
        # Publisher and Subscriber Setup
        # ---------------------------
        # Initialize a publisher for broadcasting the navigation graph on the '/navigation_graph' topic
        self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)
        # Initialize a timer to periodically publish the navigation graph every second
        self.graph_timer = self.create_timer(1.0, self.publish_navigation_graph)
    
        # Initialize a subscriber to receive slave registration messages from the '/slave_registration' topic
        self.slave_registration_subscriber = self.create_subscription(
            String, '/slave_registration', self.slave_registration_callback, 10
        )
    
        # Initialize a subscriber to receive navigation status updates from slaves on the '/navigation_status' topic
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )
    
        # ---------------------------
        # Manager Initialization
        # ---------------------------
        # Initialize the HeartbeatManager to handle heartbeat publishing for the master node
        self.heartbeat_manager = HeartbeatManager(self)
        self.heartbeat_manager.start_publishing()
    
        # Initialize the WaypointManager to handle waypoint assignments to slaves
        self.waypoint_manager = WaypointManager(self)
    
        # ---------------------------
        # Timer for Periodic Maintenance
        # ---------------------------
        # Set up a periodic timer based on 'check_interval' to perform maintenance tasks
        self.timer = self.create_timer(self.check_interval, self.timer_callback)
    
        # ---------------------------
        # Slave Tracking Structures
        # ---------------------------
        # Initialize a dictionary to track registered slaves and their states
        self.slaves = {}            # { namespace_slave: SlaveState(...) }
        # Flag to indicate whether graph partitioning has been completed
        self.partitioning_done = False
    
        # Initialize a set to keep track of edges currently occupied by slaves
        self.occupied_edges = set()
        # Initialize a dictionary to map occupied edges to the slave namespaces that occupy them
        self.edge_occupants = {}
    
        # Reset occupied_edges and edge_occupants to ensure no residual data from previous runs
        self.reset_occupied_edges()
    
        # Initialize a dictionary to optionally track the last time each edge was traversed
        self.edge_last_visit_time = {}
    
        # Log an informational message indicating successful initialization
        self.get_logger().info("Master node initialized with undirected CPP approach.")
    
        # ---------------------------
        # Subscription to '/first_waypoint_reached'
        # ---------------------------
        # Initialize a subscriber to receive first waypoint reached notifications from slaves
        self.first_wp_reached_subscriber = self.create_subscription(
            String,
            '/first_waypoint_reached',
            self.on_first_waypoint_reached,  # Callback defined in MasterCallbacks
            10
        )
        self.get_logger().info("Subscribed to '/first_waypoint_reached' for first waypoint notifications.")
    
        # Initialize tracking structures for first waypoint notifications
        self.slaves_that_reported_first_wp = set()
        self.waiting_for_first_waypoints = False
    
    def check_slaves_timeout(self):
        """
        Identifies and removes slaves that have exceeded the heartbeat timeout.
        
        This method iterates through all registered slaves and checks if the time elapsed since 
        their last heartbeat exceeds the predefined timeout threshold (`self.timeout`). Slaves that 
        have timed out are removed from the active slaves list, and any edges they were traversing 
        are freed to allow reassignment to other slaves.
        """
        # Get the current time in seconds using the ROS2 clock
        current_time = self.get_clock().now().nanoseconds / 1e9
        # List to store namespaces of slaves that need to be removed due to timeout
        slaves_to_remove = []
    
        # Iterate over all registered slaves to check for timeouts
        for slave_ns, slave in self.slaves.items():
            # Calculate the elapsed time since the slave's last heartbeat
            if current_time - slave.last_seen_time > self.timeout:
                # Log a warning indicating the slave has timed out
                self.get_logger().warn(f"Slave {slave_ns} has timed out. Removing from active slaves.")
                # Mark the slave for removal
                slaves_to_remove.append(slave_ns)
    
        # Remove each timed-out slave from the tracking structures
        for slave_ns in slaves_to_remove:
            if slave_ns in self.slaves:
                s = self.slaves[slave_ns]
    
                # If the slave was traversing an edge, free that edge
                if s.current_edge is not None and s.current_edge in self.occupied_edges:
                    self.occupied_edges.remove(s.current_edge)
                    occupant = self.edge_occupants.pop(s.current_edge, None)
                    self.get_logger().info(
                        f"Freed edge {s.current_edge} from occupant={occupant} due to slave timeout."
                    )
    
                # Remove the slave from the tracking dictionary
                del self.slaves[slave_ns]
                # Log the removal action with a warning
                self.get_logger().warn(f"Removed slave {slave_ns} due to timeout.")
    
        # If any slaves were removed, initiate waypoint repartitioning and reassignment
        if slaves_to_remove:
            self.get_logger().info("Repartitioning and reassigning waypoints after slave removal.")
            # Call the WaypointManager to handle repartitioning and assigning new waypoints
            self.waypoint_manager.repartition_and_assign_waypoints()
    
    def timer_callback(self):
        """
        Periodic callback triggered by a ROS2 timer to perform maintenance tasks.
        
        Actions performed during each timer callback:
          1. Check for and handle any slaves that have timed out.
          2. Assign waypoints to any waiting slaves that are ready to receive new tasks.
        
        This ensures continuous monitoring of slave health and efficient distribution of navigation tasks.
        """
        # Log that the timer callback has been triggered at the DEBUG level
        self.get_logger().debug("Master timer callback triggered.")
        # Invoke the method to check and handle slave timeouts
        self.check_slaves_timeout()
        # Assign waypoints to any slaves that are ready and awaiting tasks
        self.waypoint_manager.assign_waiting_slaves()
    
    def reset_occupied_edges(self):
        """
        Resets the tracking of occupied edges and their occupants.
        
        This method clears both the `occupied_edges` set and the `edge_occupants` dictionary,
        ensuring that no edges are marked as occupied at the start. This is crucial for preventing
        task assignment conflicts and ensuring accurate tracking of edge usage.
        """
        # Clear the set of occupied edges
        self.occupied_edges.clear()
        # Clear the dictionary mapping edges to their occupants
        self.edge_occupants.clear()
        # Log the reset action at the INFO level
        self.get_logger().info("Reset all occupied_edges and edge_occupants.")


def main(args=None):
    """
    The main entry point for the MasterNavigationNode ROS2 node.
    
    Steps performed:
      1. Initializes the ROS2 Python client library.
      2. Instantiates the MasterNavigationNode.
      3. Spins the node to process callbacks and handle ROS2 events.
      4. Handles graceful shutdown upon receiving a keyboard interrupt.
    
    Parameters:
        args (list, optional): Command-line arguments passed to the node. Defaults to None.
    """
    # Initialize the ROS2 client library
    rclpy.init(args=args)
    # Instantiate the MasterNavigationNode
    node = MasterNavigationNode()
    try:
        # Set the flag to wait for all slaves to reach their first waypoint
        node.waiting_for_first_waypoints = True
        # Clear any previous records of slaves that reported reaching the first waypoint
        node.slaves_that_reported_first_wp.clear()
        # Assign initial offsets along the global CPP route to slaves
        node.waypoint_manager.assign_offsets_along_route()
    
        # Spin the node to keep it active and processing callbacks
        node.get_logger().info("MasterNavigationNode is now spinning.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow the node to be interrupted gracefully via keyboard (e.g., Ctrl+C)
        pass
    finally:
        # Destroy the node explicitly (optional but recommended)
        node.destroy_node()
        # Shutdown the ROS2 client library
        rclpy.shutdown()


if __name__ == '__main__':
    # Execute the main function if the script is run directly
    main()
