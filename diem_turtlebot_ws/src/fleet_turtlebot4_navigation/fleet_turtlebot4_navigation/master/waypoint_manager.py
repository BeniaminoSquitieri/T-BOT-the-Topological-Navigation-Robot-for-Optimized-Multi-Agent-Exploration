import json
import networkx as nx
from std_msgs.msg import String

from .path_calculation import calculate_undirected_cpp_route


class WaypointManager:
    """
    Manages waypoint assignments to slaves, ensuring no conflicts in edge usage.
    
    The `WaypointManager` is responsible for assigning navigation waypoints to slave robots,
    ensuring that no two slaves are assigned to traverse the same edge simultaneously. It handles
    initial waypoint assignments, subsequent waypoint assignments, and manages slaves that are
    waiting due to edge occupancy conflicts. Additionally, it can distribute slaves along the
    global Closed Postman Problem (CPP) route with uniform offsets to ensure efficient coverage.
    
    Attributes:
        node (MasterNavigationNode):
            A reference to the `MasterNavigationNode` instance, providing access to the master node's
            attributes and methods, such as logging, graph data, and slave tracking.
        
        assigned_nodes (set):
            A set that keeps track of nodes that have already been assigned to slaves to prevent
            duplicate assignments. (Note: In the current implementation, this set is initialized
            but not utilized. It can be leveraged in future enhancements for more granular control.)
    """
    
    def __init__(self, node):
        """
        Initializes the WaypointManager.
        
        Parameters:
            node (MasterNavigationNode):
                A reference to the master node, allowing the WaypointManager to access and modify
                the master's state, such as the navigation graph, slave states, and logging facilities.
        """
        self.node = node  # Reference to the MasterNavigationNode
        self.assigned_nodes = set()  # Tracks nodes that have been assigned to slaves to prevent duplication
    
    # ===================================================
    # Initial Waypoint Assignment
    # ===================================================
    
    def assign_initial_waypoint(self, slave_ns: str):
        """
        Assigns the first waypoint to a newly registered slave.
        
        This method is invoked when a new slave robot registers with the master node. It assigns
        the first waypoint from the slave's assigned waypoints list, ensuring that the edge between
        the slave's current node and the first waypoint is not already occupied by another slave.
        If the desired edge is occupied, the slave is marked as waiting until the edge becomes available.
        
        Parameters:
            slave_ns (str):
                The unique namespace identifier of the slave robot to which the initial waypoint is to be assigned.
        """
        # Log the current state of occupied edges and their occupants before assignment
        self.node.get_logger().info(f"==== OCCUPIED EDGES BEFORE ASSIGNING FIRST WAYPOINT ====")
        self.node.get_logger().info(f"Occupied edges: {self.node.occupied_edges}")
        self.node.get_logger().info(f"Edge occupants: {self.node.edge_occupants}")
        self.node.get_logger().info(f"========================================================")
    
        # Directly call assign_next_waypoint to handle the actual assignment
        self.assign_next_waypoint(slave_ns)
        # Retrieve the SlaveState instance for the specified slave
        slave = self.node.slaves[slave_ns]
    
        # Check if there are any waypoints assigned to the slave
        if not slave.assigned_waypoints:
            self.node.get_logger().warn(f"No waypoints to assign to slave {slave_ns}.")
            return
    
        # Determine the next waypoint label based on the current waypoint index
        waypoint_label = slave.assigned_waypoints[slave.current_waypoint_index]
        # Retrieve the slave's current node (location)
        from_node = slave.current_node
        # The next node to assign as a waypoint
        to_node = waypoint_label
    
        # Create the edge key as a sorted tuple to maintain consistency in edge representation
        edge_key = tuple(sorted([from_node, to_node]))
    
        # Check if the desired edge is already occupied by another slave
        if edge_key in self.node.occupied_edges:
            # If the edge is occupied, log a warning and mark the slave as waiting
            self.node.get_logger().warn(
                f"Edge {edge_key} is occupied. {slave_ns} must wait for initial waypoint."
            )
            slave.waiting = True
            return
        else:
            # If the edge is free, occupy it by adding it to the occupied_edges set
            self.node.occupied_edges.add(edge_key)
            # Update the slave's current_edge attribute to reflect the occupied edge
            slave.current_edge = edge_key
    
        # Create and publish the waypoint message to the slave
        data = self.node.full_graph.nodes[to_node]
        waypoint_msg = {
            'label': to_node,
            'x': data['x'],
            'y': data['y'],
        }
        msg = String()
        msg.data = json.dumps(waypoint_msg)
        slave.publisher.publish(msg)
    
        # Log the successful assignment of the initial waypoint
        self.node.get_logger().info(
            f"Assigned initial waypoint '{to_node}' to slave '{slave_ns}'. "
            f"Waypoint data: {waypoint_msg}"
        )
    
        # Update the slave's current_waypoint_index to point to the next waypoint in the list
        slave.current_waypoint_index = (slave.current_waypoint_index + 1) % len(slave.assigned_waypoints)
    
    # ===================================================
    # Next Waypoint Assignment
    # ===================================================
    
    def assign_next_waypoint(self, slave_ns: str):
        """
        Assigns the next waypoint to the specified slave.
        
        This method handles the assignment of the subsequent waypoint in the slave's assigned waypoints list.
        It ensures that the edge between the slave's current node and the next waypoint is free before assignment.
        If the edge is occupied by another slave, the current slave is marked as waiting.
        
        Parameters:
            slave_ns (str):
                The unique namespace identifier of the slave robot to which the next waypoint is to be assigned.
        """
        # Retrieve the SlaveState instance for the specified slave
        slave = self.node.slaves[slave_ns]
    
        # Check if the slave has any waypoints assigned
        if not slave.assigned_waypoints:
            self.node.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
            return
    
        # If the slave has traversed all assigned waypoints, optionally restart the route
        if slave.current_waypoint_index >= len(slave.assigned_waypoints):
            self.node.get_logger().info(f"All waypoints assigned to {slave_ns} have been traversed. Restarting route.")
            slave.current_waypoint_index = 0
    
        # Determine the next node to assign as a waypoint
        next_node = slave.assigned_waypoints[slave.current_waypoint_index]
        # Retrieve the slave's current node (location)
        from_node = slave.current_node
    
        # If the next waypoint is the same as the current node, skip assignment to avoid redundancy
        if next_node == from_node:
            self.node.get_logger().warn(
                f"Slave '{slave_ns}' next_node == current_node ({next_node}); skipping to next index."
            )
            # Increment the waypoint index to move to the next waypoint
            slave.current_waypoint_index += 1
            # Recursively call assign_next_waypoint to assign the subsequent waypoint
            self.assign_next_waypoint(slave_ns)
            return
    
        # Create the edge key as a sorted tuple to maintain consistency
        edge_key = tuple(sorted([from_node, next_node]))
    
        # Check if the desired edge is already occupied by another slave
        if edge_key in self.node.occupied_edges:
            # Retrieve the occupant of the edge
            occupant = self.node.edge_occupants.get(edge_key, "Unknown")
            if occupant != slave_ns:
                # If the edge is occupied by a different slave, log a warning and mark the current slave as waiting
                self.node.get_logger().warn(
                    f"Edge {edge_key} is occupied by '{occupant}'. {slave_ns} must wait."
                )
                slave.waiting = True
                return
            else:
                # If the edge is already occupied by the current slave, proceed with assignment
                self.node.get_logger().info(
                    f"Edge {edge_key} is already occupied by '{slave_ns}'. Proceeding with assignment."
                )
    
        # If the edge is not occupied, proceed to assign it to the slave
        if edge_key not in self.node.occupied_edges:
            # Occupy the edge by adding it to the occupied_edges set
            self.node.occupied_edges.add(edge_key)
            # Map the edge to the occupying slave
            self.node.edge_occupants[edge_key] = slave_ns
            # Update the slave's current_edge attribute
            slave.current_edge = edge_key
            # Mark the slave as not waiting
            slave.waiting = False
    
        # Create and publish the waypoint message to the slave
        data = self.node.full_graph.nodes[next_node]
        waypoint_msg = {
            'label': next_node,
            'x': data['x'],
            'y': data['y'],
        }
        msg = String()
        msg.data = json.dumps(waypoint_msg)
        slave.publisher.publish(msg)
    
        # Log the successful assignment of the next waypoint
        self.node.get_logger().info(
            f"Assigned next waypoint '{next_node}' to slave '{slave_ns}'. "
            f"Edge {edge_key} is now occupied by {slave_ns}."
        )
    
        # Increment the slave's current_waypoint_index to point to the next waypoint
        slave.current_waypoint_index += 1
    
    # ===================================================
    # Assigning Waypoints to Waiting Slaves
    # ===================================================
    
    def assign_waiting_slaves(self):
        """
        Assigns waypoints to slaves that are currently waiting because an edge was occupied.
        
        This method iterates through all registered slaves and checks if any are marked as waiting
        for a waypoint assignment due to edge occupancy conflicts. For each waiting slave, it attempts
        to assign the next waypoint if the required edge becomes available.
        """
        # Iterate through all slaves in a sorted order based on their namespaces
        for slave_ns in sorted(self.node.slaves.keys()):
            # Retrieve the SlaveState instance for the current slave
            slave = self.node.slaves[slave_ns]
            # Skip slaves that are not waiting
            if not slave.waiting:
                continue
    
            # Check if the slave has any waypoints assigned
            if not slave.assigned_waypoints:
                self.node.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                continue
    
            # Determine the next waypoint label based on the current waypoint index
            waypoint_label = slave.assigned_waypoints[slave.current_waypoint_index]
            # Retrieve the slave's current node (location)
            from_node = slave.current_node
            # The next node to assign as a waypoint
            to_node = waypoint_label
            # Create the edge key as a sorted tuple to maintain consistency
            edge_key = tuple(sorted([from_node, to_node]))
    
            # Check if the desired edge has been freed (no longer occupied)
            if edge_key not in self.node.occupied_edges:
                # Occupy the edge by adding it to the occupied_edges set
                self.node.occupied_edges.add(edge_key)
                # Map the edge to the occupying slave
                self.node.edge_occupants[edge_key] = slave_ns
                # Update the slave's current_edge attribute
                slave.current_edge = edge_key
                # Mark the slave as not waiting
                slave.waiting = False
    
                # Create and publish the waypoint message to the slave
                data = self.node.full_graph.nodes[to_node]
                waypoint_msg = {
                    'label': to_node,
                    'x': data['x'],
                    'y': data['y'],
                }
                msg = String()
                msg.data = json.dumps(waypoint_msg)
                slave.publisher.publish(msg)
    
                # Log the successful assignment of the waiting waypoint
                self.node.get_logger().info(
                    f"Assigned waiting waypoint '{to_node}' to slave '{slave_ns}'. "
                    f"Waypoint data: {waypoint_msg}"
                )
    
                # Update the slave's current_node to the newly assigned waypoint
                slave.current_node = to_node
                # Increment the slave's current_waypoint_index to point to the next waypoint
                slave.current_waypoint_index += 1
            else:
                # If the edge is still occupied, log a warning and leave the slave in the waiting state
                self.node.get_logger().warn(
                    f"Edge {edge_key} is still occupied. {slave_ns} remains waiting."
                )
    
    # ===================================================
    # Assigning Offsets Along the Global CPP Route
    # ===================================================
    
    def assign_offsets_along_route(self):
        """
        Distributes robots along the global CPP route with uniform offsets.
        
        This method assigns each active slave robot to a specific starting point along the global
        CPP route based on the total number of slaves and the length of the route. By distributing
        slaves with uniform offsets, the fleet ensures efficient and comprehensive coverage of the
        navigation area without overlapping tasks.
        """
        # Log the start of the offset assignment process
        self.node.get_logger().info("==== assign_offsets_along_route() START ====")
        
        # Check if a global CPP route has been computed
        if not self.node.global_cpp_route:
            self.node.get_logger().warn("No global CPP route available.")
            return
    
        # Retrieve a list of active slaves
        active_slaves = list(self.node.slaves.values())
        # Determine the number of active slaves
        k = len(active_slaves)
        # If there are no active slaves, log a warning and exit
        if k == 0:
            self.node.get_logger().warn("No slaves to assign offset.")
            return
    
        # Determine the total length of the global CPP route
        route_length = len(self.node.global_cpp_route)
        # If there are fewer route nodes than slaves, log a warning about potential overlapping starting points
        if route_length < k:
            self.node.get_logger().warn("Fewer route nodes than slaves. Some slaves may start at the same node.")
    
        # Iterate through each slave and assign a starting offset based on their index
        for i, slave in enumerate(active_slaves):
            # Calculate the offset index for the current slave to distribute them uniformly along the route
            offset_index = int(round(i * (route_length / k))) % route_length
            # Assign the entire global CPP route to the slave, starting at the calculated offset
            slave.assigned_waypoints = self.node.global_cpp_route.copy()
            # Set the slave's current_waypoint_index to the offset index
            slave.current_waypoint_index = offset_index
            
            # Determine the starting node based on the offset index
            start_label = self.node.global_cpp_route[offset_index]
            # Update the slave's current_node to the starting node
            slave.current_node = start_label
    
            # Determine the next waypoint in the route
            next_index = (offset_index + 1) % route_length
            next_label = self.node.global_cpp_route[next_index]
    
            # Log detailed information about the assigned offset for debugging and monitoring
            self.node.get_logger().info(f"--- OFFSET for {slave.slave_ns} ---")
            self.node.get_logger().info(f"  offset_index = {offset_index}")
            self.node.get_logger().info(f"  start_node   = {start_label}")
            self.node.get_logger().info(f"  next_waypoint= {next_label}")
            self.node.get_logger().info(f"  assigned_waypoints: {slave.assigned_waypoints}")
            self.node.get_logger().info(f"----------------------")
    
            # Create the edge key between the starting node and the next waypoint
            edge_key = tuple(sorted([start_label, next_label]))
            # Check if the edge is free to be occupied
            if edge_key not in self.node.occupied_edges:
                # Occupy the edge by adding it to the occupied_edges set
                self.node.occupied_edges.add(edge_key)
                # Map the edge to the occupying slave
                self.node.edge_occupants[edge_key] = slave.slave_ns
    
            # Create and publish the first waypoint message to the slave
            data = self.node.full_graph.nodes[next_label]
            waypoint_msg = {
                'label': next_label,
                'x': data['x'],
                'y': data['y'],
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            slave.publisher.publish(msg)
    
            # Update the slave's current_waypoint_index to point to the next waypoint
            slave.current_waypoint_index = next_index
    
            # Log the successful assignment of the initial waypoint with offset information
            self.node.get_logger().info(
                f"Assigned offset waypoint '{next_label}' to slave '{slave.slave_ns}' "
                f"(offset_index={offset_index}). "
                f"--- Edges Occupied Now: {self.node.occupied_edges}"
            )
    
        # Mark that partitioning has been completed
        self.node.partitioning_done = True
        # Log the end of the offset assignment process
        self.node.get_logger().info("==== assign_offsets_along_route() END ====")
    
    # ===================================================
    # Repartitioning and Assigning Waypoints
    # ===================================================
    
    def repartition_and_assign_waypoints(self):
        """
        Assigns offsets to robots based on the total number of waypoints and active robots.
        
        This method recalculates the distribution of waypoints among active slaves to ensure that
        all robots are efficiently covering the navigation graph without overlapping tasks. It takes
        into account the total number of waypoints in the global CPP route and the number of active
        slaves to determine appropriate offsets.
        
        The method performs the following steps:
          1. Determines the number of active slaves and the total number of waypoints.
          2. Clears the current tracking of occupied edges to reset the state.
          3. Calculates an offset based on the number of slaves and the length of the route.
          4. Assigns each slave an offset route and the corresponding starting waypoint.
          5. Assigns the next waypoint to each slave by invoking `assign_next_waypoint`.
        """
        # Determine the number of active slaves
        num_slaves = len(self.node.slaves)
        # If there are no active slaves, log a warning and exit
        if num_slaves == 0:
            self.node.get_logger().warn("No slaves to assign waypoints.")
            return
    
        # Determine the total number of waypoints in the global CPP route
        total_waypoints = len(self.node.global_cpp_route)
        # If the global CPP route is empty, log a warning and exit
        if total_waypoints == 0:
            self.node.get_logger().warn("Global CPP route is empty.")
            return
    
        # # Log that the occupied edges are being cleared before repartitioning
        # self.node.get_logger().info("Clearing occupied_edges before repartition/offset assignment...")
        # # Clear the set of occupied edges
        # self.node.occupied_edges.clear()
        # # Clear the dictionary mapping edges to their occupants
        # self.node.edge_occupants.clear()
    
        # Calculate the offset based on the number of slaves and the total waypoints
        offset = total_waypoints // num_slaves
        # Iterate through each slave to assign an offset route
        for idx, (slave_ns, slave) in enumerate(self.node.slaves.items()):
            # Determine the starting index for the current slave based on its position in the list
            start_index = (idx * offset) % total_waypoints
            # Assign the entire global CPP route to the slave, starting at the calculated offset
            assigned_route = self.node.global_cpp_route[start_index:] + self.node.global_cpp_route[:start_index]
            # Update the slave's assigned_waypoints with the newly calculated route
            slave.assigned_waypoints = assigned_route
            # Reset the slave's current_waypoint_index to start at the beginning of the assigned route
            slave.current_waypoint_index = 0
    
            # Set the slave's current_node to the first node in the assigned route
            first_node = assigned_route[0]
            slave.current_node = first_node
    
            # Log detailed information about the assigned offset for debugging and monitoring
            self.node.get_logger().info(
                f"Assigned offset={offset} route to slave '{slave_ns}'. Starting at index={start_index} "
                f"first_node={first_node}"
            )
    
            # Assign the next waypoint to the slave by invoking the assign_next_waypoint method
            self.assign_next_waypoint(slave_ns)
    
    # ===================================================
    # Extracting Waypoints from a Subgraph (Deprecated)
    # ===================================================
    
    def extract_waypoints_from_subgraph(self, subgraph: nx.Graph) -> list:
        """
        Extracts waypoints from a subgraph.
        
        **Note**: This method is currently not utilized in the offset-based strategy but is retained
        for potential future use or alternative waypoint assignment strategies.
        
        Parameters:
            subgraph (networkx.Graph):
                The subgraph from which to extract waypoints. Each node in the subgraph should have
                attributes like 'label', 'x', and 'y' representing its identifier and spatial coordinates.
        
        Returns:
            list:
                A list of waypoints, where each waypoint is a dictionary containing 'label', 'x', and 'y'.
                Example:
                    [
                        {'label': 'node_1', 'x': 1.0, 'y': 2.0},
                        {'label': 'node_2', 'x': 3.0, 'y': 4.0},
                        # ... more waypoints ...
                    ]
        """
        waypoints = []
        # Iterate through all nodes in the subgraph
        for node, data in subgraph.nodes(data=True):
            # Create a waypoint dictionary with the node's label and coordinates
            waypoint = {
                'label': node,
                'x': data['x'],
                'y': data['y'],
            }
            # Append the waypoint to the waypoints list
            waypoints.append(waypoint)
        # Return the list of extracted waypoints
        return waypoints
    
    # ===================================================
    # Debugging: Printing Subgraphs
    # ===================================================
    
    def print_subgraphs(self, subgraphs: list):
        """
        Logs details of each subgraph for debugging purposes.
        
        **Note**: This method is currently not essential in the offset-based strategy but can be useful
        for debugging graph partitioning and waypoint assignments in alternative strategies.
        
        Parameters:
            subgraphs (list of networkx.Graph):
                A list of subgraphs, where each subgraph represents a partition of the main navigation graph.
                Each subgraph should contain nodes with attributes like 'label', 'x', and 'y', and edges
                with attributes such as 'distance'.
        """
        # Log the start of subgraph details
        self.node.get_logger().info("----- Subgraphs After Partition -----")
        # Iterate through each subgraph with its index
        for idx, subg in enumerate(subgraphs):
            # Log the subgraph number
            self.node.get_logger().info(f"Subgraph {idx+1}:")
            # Log the number of nodes in the subgraph
            self.node.get_logger().info(f"  Nodes ({len(subg.nodes())}):")
            # Iterate through all nodes in the subgraph and log their details
            for n, d in subg.nodes(data=True):
                self.node.get_logger().info(f"    {n} -> (x={d.get('x')}, y={d.get('y')})")
            # Log the number of edges in the subgraph
            self.node.get_logger().info(f"  Edges ({len(subg.edges())}):")
            # Iterate through all edges in the subgraph and log their details
            for u, v, ed in subg.edges(data=True):
                self.node.get_logger().info(
                    f"    {u}-{v}, distance={ed.get('distance')}"
                )
        # Log the end of subgraph details
        self.node.get_logger().info("----- End of Subgraphs -----")
