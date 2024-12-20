# waypoint_manager.py

import json
from std_msgs.msg import String
import networkx as nx

from .path_calculation import calculate_dcpp_route, orientation_rad_to_str
from .graph_utils import partition_graph_wrapper
from .slave_state import SlaveState

class WaypointManager:
    """
    Manages waypoint assignments to slaves, ensuring no conflicts in edge occupation.
    """
    def __init__(self, node):
        """
        Initialize the WaypointManager.

        Args:
            node (Node): The ROS 2 node instance.
        """
        self.node = node
        self.occupied_edges = set()

    def repartition_and_assign_waypoints(self):
        """
        Partitions the graph and assigns waypoints to active slaves.
        """
        num_slaves = len(self.node.slaves)
        if num_slaves == 0:
            self.node.get_logger().warn("No active slaves found. Waiting for slaves to register.")
            self.node.partitioning_done = False
            return

        # Collect the initial positions of all active slaves
        start_positions = []
        for slave in self.node.slaves.values():
            if slave.initial_x is not None and slave.initial_y is not None:
                start_positions.append({'x': slave.initial_x, 'y': slave.initial_y})
            else:
                self.node.get_logger().warn(f"Slave {slave.slave_ns} lacks valid initial position.")

        # Ensure all slaves have valid initial positions
        if len(start_positions) != num_slaves:
            self.node.get_logger().error("Not all slaves have valid initial positions.")
            return

        # Partition the graph into balanced subgraphs based on the number of slaves
        try:
            subgraphs = partition_graph_wrapper(self.node.full_graph, num_slaves, start_positions=start_positions)
            self.node.get_logger().info(f"Partitioned the graph into {len(subgraphs)} subgraphs.")
            self.print_subgraphs(subgraphs)
        except ValueError as e:
            self.node.get_logger().error(f"Failed to partition graph: {e}")
            return

        # Sort the slave namespaces to ensure consistent assignment
        slaves_sorted = sorted(self.node.slaves.keys())

        # Ensure the number of subgraphs matches the number of slaves
        if len(subgraphs) != len(slaves_sorted):
            self.node.get_logger().error("Number of subgraphs does not match number of active slaves.")
            return

        # Assign each subgraph to a slave
        for idx, slave_ns in enumerate(slaves_sorted):
            subgraph = subgraphs[idx]

            # Extract waypoints from the subgraph
            waypoints = self.extract_waypoints(subgraph)  # Ora definito internamente

            # Calculate an Eulerian path through the waypoints
            dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.node.get_logger())

            # Log the calculated route for debugging
            self.node.get_logger().info(f"DCPP Route for {slave_ns}:")
            for wp in dcpp_route:
                self.node.get_logger().info(f"  {wp}")

            # Assign the calculated route to the slave
            slave = self.node.slaves[slave_ns]
            slave.assigned_waypoints = dcpp_route
            self.assign_next_waypoint(slave_ns)

        # Mark partitioning as done
        self.node.partitioning_done = True


    def extract_waypoints(self, subgraph: nx.DiGraph) -> list:
        """
        Extracts the waypoints (nodes) from a subgraph as a list of dictionaries.
        Each dictionary includes label, x, y, and orientation.

        Args:
            subgraph (nx.DiGraph): A subgraph containing nodes and edges.

        Returns:
            list: A list of dictionaries representing the waypoints.
        """
        waypoints = []
        for node, data in subgraph.nodes(data=True):
            waypoint = {
                'label': node,
                'x': data['x'],
                'y': data['y'],
                'orientation': data.get('orientation', 0.0)
            }
            waypoints.append(waypoint)
        return waypoints

    def assign_next_waypoint(self, slave_ns: str):
        """
        Assign the next waypoint from the slave's route to the slave for navigation.

        Args:
            slave_ns (str): The namespace of the slave robot.
        """
        slave = self.node.slaves[slave_ns]

        if len(slave.assigned_waypoints) == 0:
            self.node.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
            return

        waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
        from_node = slave.current_node
        to_node = waypoint['label']
        edge = (from_node, to_node)

        if edge in self.occupied_edges:
            self.node.get_logger().warn(f"Edge {edge} is already occupied. Slave {slave_ns} must wait.")
            slave.waiting = True
            return
        else:
            self.occupied_edges.add(edge)
            self.node.get_logger().info(f"Assigned edge {edge} to {slave_ns}.")

            waypoint_msg = {
                'label': waypoint['label'],
                'x': waypoint['x'],
                'y': waypoint['y'],
                'orientation': orientation_rad_to_str(waypoint['orientation'])
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)

            slave.publisher.publish(msg)

    def assign_waiting_slaves(self):
        """
        Assigns waypoints to slaves that were waiting for a free edge.
        """
        for slave_ns in sorted(self.node.slaves.keys()):
            slave = self.node.slaves[slave_ns]

            if not slave.waiting:
                continue

            if len(slave.assigned_waypoints) == 0:
                self.node.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                continue

            waypoint = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
            from_node = slave.current_node
            to_node = waypoint['label']
            edge = (from_node, to_node)

            if edge not in self.occupied_edges:
                self.occupied_edges.add(edge)
                slave.waiting = False
                self.node.get_logger().info(f"Assigned edge {edge} to slave {slave_ns} (previously waiting).")

                waypoint_msg = {
                    'label': waypoint['label'],
                    'x': waypoint['x'],
                    'y': waypoint['y'],
                    'orientation': orientation_rad_to_str(waypoint['orientation'])
                }
                msg = String()
                msg.data = json.dumps(waypoint_msg)

                slave.publisher.publish(msg)
            else:
                self.node.get_logger().warn(f"Edge {edge} is still occupied. Slave {slave_ns} remains waiting.")

    def print_subgraphs(self, subgraphs: list):
        """
        Log details of each subgraph created during graph partitioning.

        Args:
            subgraphs (list of nx.Graph): A list of subgraphs created during partitioning.
        """
        self.node.get_logger().info("----- Subgraphs After Partition -----")

        for idx, subgraph in enumerate(subgraphs):
            self.node.get_logger().info(f"Subgraph {idx+1}:")
            self.node.get_logger().info(f"  Nodes ({len(subgraph.nodes())}):")
            for node, data in subgraph.nodes(data=True):
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                orientation = data.get('orientation', 0.0)
                self.node.get_logger().info(f"    {node}: Position=({x}, {y}), Orientation={orientation} radians")
            self.node.get_logger().info(f"  Edges ({len(subgraph.edges())}):")
            for u, v, data in subgraph.edges(data=True):
                weight = data.get('weight', 1.0)
                self.node.get_logger().info(f"    From {u} to {v}, Weight: {weight}")

        self.node.get_logger().info("----- End of Subgraphs -----")
