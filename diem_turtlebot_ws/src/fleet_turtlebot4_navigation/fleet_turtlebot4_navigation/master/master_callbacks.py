import json
from std_msgs.msg import String

from .slave_state import SlaveState
from .path_calculation import calculate_undirected_cpp_route


class MasterCallbacks:
    """
    Callback methods for the Master node.

    This class encapsulates the essential functionalities required by a master node in a
    distributed robot fleet system. It handles tasks such as publishing the navigation graph,
    computing global routes, registering new slaves, processing navigation status updates from
    slaves, and managing timeouts for inactive slaves.

    By separating these callbacks into their own class, the code promotes modularity and
    reusability, allowing different master node implementations to inherit and utilize
    these core functionalities.
    """

    def __init__(self):
        """
        Initializes the MasterCallbacks class.

        Sets up initial attributes required for managing the global CPP (Closed Path Planning)
        route. This route is shared across all slave nodes to coordinate their navigation tasks.
        """
        self.global_cpp_route = []  # Will hold the common Eulerian path for navigation
        self.first_waypoint_phase = True
        self.slaves_reached_first = set()

    def publish_navigation_graph(self):
        """
        Publishes the navigation graph to a ROS2 topic.

        This method serializes the current navigation graph (`self.full_graph`) into a JSON
        format and publishes it on a designated ROS2 topic (`self.graph_publisher`). The
        graph includes nodes with their spatial coordinates and edges with their associated
        traversal distances.

        The 'weight' attribute of each edge is used to represent traversal time, calculated
        as distance divided by a constant speed factor (0.31 units/sec).
        """
        # Initialize a String message for the graph
        graph_msg = String()

        # Prepare the graph data in JSON-serializable format
        graph_data = {
            'nodes': [],
            'edges': []
        }

        # Iterate over all nodes in the full graph and add their details to graph_data
        for node, data in self.full_graph.nodes(data=True):
            graph_data['nodes'].append({
                'label': node,
                'x': data.get('x', 0.0),
                'y': data.get('y', 0.0),
                # Orientation can be added if needed
                # 'orientation': data.get('orientation', 0.0)
            })

        # Iterate over all edges in the full graph and add their details to graph_data
        for u, v, data in self.full_graph.edges(data=True):
            if u < v:  # Avoid duplicates in an undirected graph
                dist_value = data.get('weight', 1.0)  # Default distance if not specified
                graph_data['edges'].append({
                    'source': u,
                    'target': v,
                    'distance': dist_value
                })

        # Serialize the graph data to JSON and assign to the message
        graph_msg.data = json.dumps(graph_data)

        # Publish the serialized graph message
        self.graph_publisher.publish(graph_msg)

        # Log the publishing action at the DEBUG level
        self.get_logger().debug("Navigation graph published.")

    def compute_global_cpp_route(self):
        """
        Computes the global Closed Path Planning (CPP) route on the navigation graph.

        This method generates a list of waypoints (`self.global_cpp_route`) representing an Eulerian
        circuit that traverses each edge exactly once. It utilizes the `calculate_undirected_cpp_route`
        function to compute this route based on the nodes and the navigation graph.

        If a valid Eulerian circuit is found, it logs the computed route. Otherwise, it logs an error.
        """
        waypoints = []

        # Extract waypoint information from each node in the graph
        for node, data in self.full_graph.nodes(data=True):
            waypoint = {
                'label': node,
                'x': data['x'],
                'y': data['y']
            }
            waypoints.append(waypoint)

        # Calculate the CPP route using the provided path calculation utility
        route_nodes = calculate_undirected_cpp_route(waypoints, self.full_graph, self.get_logger())

        # Assign the computed route to the class attribute
        self.global_cpp_route = route_nodes

        # Log the outcome of the route computation
        if self.global_cpp_route:
            self.get_logger().info(f"Global CPP route computed: {self.global_cpp_route}")
        else:
            self.get_logger().error("Global CPP route is empty or Euler circuit not found.")

    def slave_registration_callback(self, msg):
        """
        Callback that handles the registration of a new slave robot.

        When a new slave robot publishes its namespace to the `/slave_registration` topic,
        this callback is invoked to register the slave within the master's tracking system.
        It performs the following actions:

        1. Parses the slave's namespace from the incoming message.
        2. Checks if the slave is already registered.
           - If not registered:
             a. Creates a publisher for sending navigation commands to the slave.
             b. Initializes a `SlaveState` instance to track the slave's status.
             c. Adds the slave to the master's slave dictionary.
             d. Computes the global CPP route if not already done.
             e. Initiates waypoint repartitioning and assignment.
           - If already registered:
             a. Updates the slave's last seen time to prevent timeout.

        Parameters:
        - msg (std_msgs.msg.String): The incoming message containing the slave's namespace.
        """
        # Extract the slave's namespace from the message, removing any surrounding whitespace
        slave_ns = msg.data.strip()

        # Get the current time in seconds (from ROS2 clock)
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Check if the slave is already registered
        if slave_ns not in self.slaves:
            # Create a ROS2 publisher for sending navigation commands to this slave
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)

            # Initialize a new SlaveState instance to track the slave's status and communication
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time  # Record the registration time

            # Add the new slave to the master's slave tracking dictionary
            self.slaves[slave_ns] = slave_state

            # Log the successful registration of the new slave
            self.get_logger().info(f"Registered new slave: {slave_ns}")

            # If the global CPP route has not been computed yet, compute it now
            if not self.global_cpp_route:
                self.compute_global_cpp_route()

            # Initiate waypoint repartitioning and assignment to slaves
            self.waypoint_manager.repartition_and_assign_waypoints()

        else:
            # If the slave is already registered, update its last seen time to prevent timeout
            self.slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"Updated last seen time for slave: {slave_ns}")

    def navigation_status_callback(self, msg):
        """
        Processes navigation status updates received from slaves.

        Slaves publish their navigation status on the `/navigation_status` topic. This callback
        handles these updates, performing actions based on the reported status, such as:

        - Marking slaves as ready.
        - Tracking traversal of edges.
        - Handling errors reported by slaves.
        - Assigning the next waypoint to slaves upon successful navigation.

        Parameters:
        - msg (std_msgs.msg.String): The incoming message containing the slave's navigation status in JSON format.
        """
        try:
            # Parse the incoming JSON message
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_wpt = data['current_waypoint']
            error_message = data.get('error_message', '')
            traversed = data.get('traversed_edge', [])

            # Normalize the traversed edge as a sorted tuple to ensure consistency
            if len(traversed) == 2:
                edge_key = tuple(sorted(traversed))
            else:
                edge_key = None

        except (json.JSONDecodeError, KeyError) as e:
            # Log an error if the message is malformed or missing required fields
            self.get_logger().error(f"Invalid navigation status message: {e}")
            return

        # Check if the slave is registered; if not, optionally handle or ignore
        if slave_ns not in self.slaves:
            # Optionally, handle unregistered slaves, such as auto-registering them
            # self.get_logger().warn(f"Received status from unknown slave '{slave_ns}'. Ignoring message.")
            return

        # Retrieve the SlaveState instance for the reporting slave
        slave = self.slaves[slave_ns]

        # Handle different statuses reported by the slave
        if status == "ready":
            # If the slave has a currently occupied edge, ignore redundant 'ready' statuses
            if slave.current_edge is not None:
                self.get_logger().debug(
                    f"[READY] Slave '{slave_ns}' still has current_edge={slave.current_edge}. Ignoring extra 'ready'."
                )
                return

            # If the slave was previously not ready, mark it as ready
            if not slave.ready:
                slave.ready = True
                self.get_logger().info(f"Slave '{slave_ns}' is ready.")

                # Assign waypoints after confirming the slave is ready
                self.waypoint_manager.repartition_and_assign_waypoints()
            else:
                # If the slave was already marked as ready, log it at the debug level
                self.get_logger().debug(f"Slave '{slave_ns}' was already ready.")

        elif status == "reached":
            # Log the successful arrival of the slave at a waypoint
            self.get_logger().info(
                f"Slave '{slave_ns}' reached waypoint '{current_wpt}'"
            )
            if edge_key:
                # Log the traversed edge
                # self.get_logger().info(f"Traversed edge: {edge_key}")
                self.get_logger().info(
                    f"Current occupied edges before freeing: {self.occupied_edges}"
                )
                if edge_key in self.occupied_edges:
                    # Remove the edge from the set of occupied edges
                    self.occupied_edges.remove(edge_key)
                    # Remove the occupant mapping for this edge
                    occupant = self.edge_occupants.pop(edge_key, None)
                    self.get_logger().info(
                        f"Edge {edge_key} freed by slave '{slave_ns}'. (Previously occupant={occupant})"
                    )
                else:
                    # Warn if the edge was not marked as occupied
                    self.get_logger().warn(
                        f"Received 'reached' for edge {edge_key} from slave '{slave_ns}' but it was NOT in occupied_edges."
                    )
            else:
                # Warn if no valid edge was traversed
                self.get_logger().warn(f"Slave '{slave_ns}' says 'reached' but no valid edge_key.")

            # Update the slave's current node to the reached waypoint
            slave.current_node = current_wpt
            slave.current_edge = None

            # Assign the next waypoint to the slave
            self.waypoint_manager.assign_next_waypoint(slave_ns)

        elif status == "error":
            # Log the error reported by the slave
            self.get_logger().error(f"Slave '{slave_ns}' error: {error_message}")
            # If an edge was being traversed, free it
            if edge_key and edge_key in self.occupied_edges:
                self.occupied_edges.remove(edge_key)
                occupant = self.edge_occupants.pop(edge_key, None)
                self.get_logger().info(
                    f"Edge {edge_key} freed by error from '{slave_ns}'. occupant was={occupant}"
                )

            # Remove the slave from the tracking system due to the error
            del self.slaves[slave_ns]
            self.get_logger().warn(f"Removed slave '{slave_ns}' due to error.")

        elif status == "traversing":
            # Log that the slave is currently traversing toward a waypoint
            self.get_logger().debug(
                f"Slave '{slave_ns}' is traversing toward '{current_wpt}'."
            )
            # Additional state updates or actions can be implemented here

        else:
            # Warn if an unhandled status is received
            self.get_logger().warn(f"Unhandled status '{status}' from '{slave_ns}'.")

    def on_first_waypoint_reached(self, msg: String):
        """
        Callback to handle notifications from slaves when they reach their first waypoint.

        This method tracks which slaves have reported reaching their first waypoint. Once all active
        slaves have reported, it proceeds with assigning subsequent waypoints and stops listening for
        further first waypoint notifications.

        Parameters:
            msg (std_msgs.msg.String): The incoming message containing data in JSON format with the key 'robot_namespace'.
        """
        # Attempt to decode the JSON message
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
        except (json.JSONDecodeError, KeyError):
            self.get_logger().error("Invalid /first_waypoint_reached message (JSON or missing keys).")
            return

        # If not waiting for first waypoints, ignore the notification
        if not self.waiting_for_first_waypoints:
            self.get_logger().debug(f"Ignoring notification from {slave_ns}, not waiting anymore.")
            return

        # Add the slave_ns to the set of slaves that have reached their first waypoint
        self.slaves_that_reported_first_wp.add(slave_ns)
        self.get_logger().info(f"Slave '{slave_ns}' has reached its first waypoint.")

        # Get the current set of registered slaves
        current_slaves = set(self.slaves.keys())

        # Check if all active slaves have reported reaching their first waypoint
        if current_slaves.issubset(self.slaves_that_reported_first_wp) and len(current_slaves) > 0:
            self.get_logger().info("All active slaves have reached their first waypoint. Starting subsequent assignments.")

            # Set the flag to stop waiting for first waypoints
            self.waiting_for_first_waypoints = False

            # Unsubscribe from the '/first_waypoint_reached' topic as it's no longer needed
            if self.first_wp_reached_subscriber is not None:
                self.destroy_subscription(self.first_wp_reached_subscriber)
                self.first_wp_reached_subscriber = None
                self.get_logger().info("Unsubscribed from '/first_waypoint_reached'.")

            # Proceed with assigning subsequent waypoints
            self.waypoint_manager.repartition_and_assign_waypoints()
