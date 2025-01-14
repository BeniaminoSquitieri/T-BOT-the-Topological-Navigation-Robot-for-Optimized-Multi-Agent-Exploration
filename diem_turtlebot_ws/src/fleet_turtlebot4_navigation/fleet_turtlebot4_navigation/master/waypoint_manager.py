# waypoint_manager.py

import json
from std_msgs.msg import String
import networkx as nx

from .path_calculation import calculate_undirected_cpp_route
from .slave_state import SlaveState

class WaypointManager:
    """
    Manages waypoint assignments to slave robots, ensuring no conflicts in edge usage.

    The `WaypointManager` is responsible for assigning navigation waypoints to slave robots,
    ensuring that no two slaves are assigned to traverse the same edge simultaneously. It handles
    initial waypoint assignments, subsequent waypoint assignments, and manages slaves that are
    waiting due to edge occupancy conflicts. Additionally, it distributes slaves along the
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
    # Assigning Offsets Along the Global CPP Route
    # ===================================================
    
    def assign_offsets_along_route(self):
        """
        Assigns offsets to all active slaves based on the total number of waypoints and active slaves.

        This method recalculates the distribution of waypoints among all active slaves to ensure that
        each slave is evenly spaced along the global CPP route. It assigns unique starting points (offsets)
        to each slave and resets their waypoint assignments accordingly.

        Steps:
          1. Determines the number of active slaves and the total number of waypoints.
          2. Calculates an offset for each slave to distribute them evenly along the route.
          3. Assigns the entire global CPP route to each slave, starting from their respective offsets.
          4. Publishes the first waypoint to each slave without checking edge occupancy.
        """
        # self.node.get_logger().info("==== assign_offsets_along_route() START ====")

        # Check if a global CPP route exists
        if not self.node.global_cpp_route:
            self.node.get_logger().warn("No global CPP route available.")
            return

        active_slaves = list(self.node.slaves.values())
        num_slaves = len(active_slaves)
        if num_slaves == 0:
            self.node.get_logger().warn("No active slaves to assign waypoints.")
            return

        route_length = len(self.node.global_cpp_route)

        # Calculate the offset index for each slave to distribute them evenly along the route
        for i, slave in enumerate(active_slaves):
            offset_index = int(round(i * (route_length / num_slaves))) % route_length
            slave.assigned_waypoints = self.node.global_cpp_route.copy()
            slave.current_waypoint_index = offset_index
            # Assign only the first waypoint without checking edge occupancy
            self.assign_first_waypoint_to_slave(slave)

        self.node.partitioning_done = True
        # self.node.get_logger().info("==== assign_offsets_along_route() END ====")
    
    def assign_first_waypoint_to_slave(self, slave):
        """
        Assigns the first waypoint to a slave without checking edge occupancy.
        
        Parameters:
            slave (SlaveState):
                The `SlaveState` instance representing the slave to assign the first waypoint to.
        """
        idx = slave.current_waypoint_index
        route = slave.assigned_waypoints
        if idx < 0 or idx >= len(route):
            self.node.get_logger().error(f"Invalid offset index {idx} for slave {slave.slave_ns}.")
            return

        next_label = route[idx]
        data = self.node.full_graph.nodes[next_label]
        waypoint_msg = {
            'label': next_label,
            'x': data['x'],
            'y': data['y']
        }
        msg = String()
        msg.data = json.dumps(waypoint_msg)
        slave.publisher.publish(msg)
        self.node.get_logger().info(
            f"Assigned first waypoint '{next_label}' to slave '{slave.slave_ns}' (offset_idx={idx})."
        )
        # Do not set `has_first_waypoint_assigned` yet; it will be set when the slave reaches the waypoint

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
                The unique namespace identifier of the slave robot to assign the next waypoint to.
        """
        # Retrieve the SlaveState instance for the specified slave
        slave = self.node.slaves[slave_ns]

        # Check if the slave has any waypoints assigned
        if not slave.assigned_waypoints:
            self.node.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
            return

        # If the slave has traversed all assigned waypoints, optionally restart the route
        if slave.current_waypoint_index >= len(slave.assigned_waypoints):
            self.node.get_logger().info(f"All assigned waypoints for {slave_ns} have been traversed. Restarting route.")
            slave.current_waypoint_index = 0

        # Determine the next waypoint to assign
        next_node = slave.assigned_waypoints[slave.current_waypoint_index]
        from_node = slave.current_node

        # Prevent assigning a waypoint that is the same as the current node (self-edge)
        if next_node == from_node:
            self.node.get_logger().warn(
                f"Slave '{slave_ns}' next_node ({next_node}) is the same as current_node; skipping to next waypoint."
            )
            # Increment the waypoint index to skip the current (invalid) waypoint
            slave.current_waypoint_index = (slave.current_waypoint_index + 1) % len(slave.assigned_waypoints)
            # Recursively call assign_next_waypoint to assign the next valid waypoint
            self.assign_next_waypoint(slave_ns)
            return

        # Create an ordered tuple for the edge to maintain consistency
        edge_key = tuple(sorted([from_node, next_node]))

        # Check if the desired edge is already occupied by another slave
        if edge_key in self.node.occupied_edges:
            occupant = self.node.edge_occupants.get(edge_key, "Unknown")
            if occupant != slave_ns:
                # If the edge is occupied by a different slave, mark this slave as waiting
                self.node.get_logger().warn(
                    f"Edge {edge_key} is occupied by '{occupant}'. Slave '{slave_ns}' must wait."
                )
                slave.waiting = True
                return
            # If the edge is occupied by the same slave, proceed with assignment

        # If the edge is not occupied, proceed to assign it to the slave
        if edge_key not in self.node.occupied_edges:
            # Occupy the edge by adding it to the set of occupied edges
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
            f"Edge {edge_key} is now occupied by '{slave_ns}'."
        )

        # Increment the waypoint index for the next assignment
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
        # Iterate through all slaves in alphabetical order based on their namespaces
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

            # Determine the next waypoint based on the current waypoint index
            next_node = slave.assigned_waypoints[slave.current_waypoint_index]
            from_node = slave.current_node
            # Create an ordered tuple for the edge
            edge_key = tuple(sorted([from_node, next_node]))

            # Check if the desired edge has been freed (no longer occupied)
            if edge_key not in self.node.occupied_edges:
                # Occupy the edge by adding it to the set of occupied edges
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

                # Log the successful assignment of the waiting waypoint
                self.node.get_logger().info(
                    f"Assigned waiting waypoint '{next_node}' to slave '{slave_ns}'. "
                    f"Edge {edge_key} is now occupied by '{slave_ns}'."
                )

                # Update the slave's current node to the newly assigned waypoint
                slave.current_node = next_node
                # Increment the waypoint index for the next assignment
                slave.current_waypoint_index += 1
            else:
                # If the edge is still occupied, log a warning and leave the slave in the waiting state
                self.node.get_logger().warn(
                    f"Edge {edge_key} is still occupied. Slave '{slave_ns}' remains waiting."
                )

    # ===================================================
    # Repartitioning and Assigning Waypoints
    # ===================================================
    
    def repartition_and_assign_waypoints(self):
        """
        Repartitions and assigns waypoints to all active slaves.
        
        This method recalculates the distribution of waypoints among all active slaves, assigning
        unique offsets to each slave to ensure even distribution along the global CPP route. It
        effectively redistributes waypoints whenever a new slave is added or an existing slave is removed.
        """
        # self.node.get_logger().info("Repartitioning and assigning waypoints in progress...")
        self.assign_offsets_along_route()
