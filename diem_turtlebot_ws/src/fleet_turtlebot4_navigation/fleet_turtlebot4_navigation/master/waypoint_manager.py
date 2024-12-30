# waypoint_manager.py

import json
from std_msgs.msg import String
import networkx as nx

from .path_calculation import calculate_undirected_cpp_route
from .utils import orientation_rad_to_str

class WaypointManager:
    """
    Manages waypoint assignments to slaves, ensuring no conflicts in edge usage.
    """
    def __init__(self, node):
        self.node = node  # Reference to the MasterNavigationNode
        self.assigned_nodes = set()  # Track nodes already assigned to slaves

    def assign_initial_waypoint(self, slave_ns: str):
        """
        Assigns the first waypoint to a newly registered slave.
        """
        slave = self.node.slaves[slave_ns]

        # Verifica che ci siano waypoints assegnati
        if not slave.assigned_waypoints:
            self.node.get_logger().warn(f"No waypoints to assign to slave {slave_ns}.")
            return

        # Determina il waypoint da assegnare
        waypoint_label = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
        from_node = slave.current_node
        to_node = waypoint_label

        # Crea e pubblica il messaggio di waypoint
        data = self.node.full_graph.nodes[to_node]
        orientation_str = orientation_rad_to_str(data.get('orientation', 0.0))
        waypoint_msg = {
            'label': to_node,
            'x': data['x'],
            'y': data['y'],
            'orientation': orientation_str
        }
        msg = String()
        msg.data = json.dumps(waypoint_msg)
        slave.publisher.publish(msg)

        # Log di Assegnazione
        self.node.get_logger().info(
            f"Assigned initial waypoint '{to_node}' to slave '{slave_ns}'. Waypoint data: {waypoint_msg}"
        )

        # Aggiorna l'indice del waypoint successivo
        slave.current_waypoint_index = (slave.current_waypoint_index + 1) % len(slave.assigned_waypoints)

    def assign_next_waypoint(self, slave_ns: str):
        """
        Assigns the next waypoint to a slave after it has reached the previous one.
        """
        slave = self.node.slaves[slave_ns]

        # Verifica che ci siano waypoints assegnati
        if not slave.assigned_waypoints:
            self.node.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
            return

        # Reset dell'indice se abbiamo raggiunto la fine della lista
        if slave.current_waypoint_index >= len(slave.assigned_waypoints):
            self.node.get_logger().info(f"All waypoints in the global CPP route have been assigned to {slave_ns}.")
            self.node.get_logger().info("Starting again from the first waypoint.")
            slave.current_waypoint_index = 0

        # Determina il prossimo waypoint
        waypoint_label = slave.assigned_waypoints[slave.current_waypoint_index]
        from_node = slave.current_node
        to_node = waypoint_label
        edge_key = tuple(sorted([from_node, to_node]))

        self.node.get_logger().debug(f"Slave {slave_ns} attempting to assign waypoint '{to_node}' at index {slave.current_waypoint_index}.")

        # Controlla se l'arco è occupato
        if edge_key in self.node.occupied_edges:
            self.node.get_logger().warn(f"Edge {edge_key} is occupied. {slave_ns} must wait.")
            slave.waiting = True
            return
        else:
            # Assegna l'arco e pubblica il waypoint
            self.node.occupied_edges.add(edge_key)
            slave.current_edge = edge_key
            slave.waiting = False

            # Crea e pubblica il messaggio di waypoint
            data = self.node.full_graph.nodes[to_node]
            orientation_str = orientation_rad_to_str(data.get('orientation', 0.0))
            waypoint_msg = {
                'label': to_node,
                'x': data['x'],
                'y': data['y'],
                'orientation': orientation_str
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            slave.publisher.publish(msg)

            # Log di Assegnazione
            self.node.get_logger().info(
                f"Assigned next waypoint '{to_node}' to slave '{slave_ns}'. Waypoint data: {waypoint_msg}"
            )

            # Aggiorna l'indice del waypoint successivo
            slave.current_waypoint_index += 1

    def assign_waiting_slaves(self):
        """
        Assigns waypoints to slaves that are currently waiting due to occupied edges.
        """
        for slave_ns in sorted(self.node.slaves.keys()):
            slave = self.node.slaves[slave_ns]
            if not slave.waiting:
                continue

            if not slave.assigned_waypoints:
                self.node.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                continue

            # Determina il waypoint da assegnare
            waypoint_label = slave.assigned_waypoints[slave.current_waypoint_index % len(slave.assigned_waypoints)]
            from_node = slave.current_node
            to_node = waypoint_label
            edge_key = tuple(sorted([from_node, to_node]))

            # Controlla se l'arco è libero
            if edge_key not in self.node.occupied_edges:
                # Assegna l'arco e pubblica il waypoint
                self.node.occupied_edges.add(edge_key)
                slave.current_edge = edge_key
                slave.waiting = False

                # Crea e pubblica il messaggio di waypoint
                data = self.node.full_graph.nodes[to_node]
                orientation_str = orientation_rad_to_str(data.get('orientation', 0.0))
                waypoint_msg = {
                    'label': to_node,
                    'x': data['x'],
                    'y': data['y'],
                    'orientation': orientation_str
                }
                msg = String()
                msg.data = json.dumps(waypoint_msg)
                slave.publisher.publish(msg)

                # Log di Assegnazione
                self.node.get_logger().info(
                    f"Assigned waiting waypoint '{to_node}' to slave '{slave_ns}'. Waypoint data: {waypoint_msg}"
                )

                # Aggiorna il nodo corrente e l'indice del waypoint
                slave.current_node = to_node
                slave.current_waypoint_index += 1
            else:
                self.node.get_logger().warn(f"Edge {edge_key} is still occupied. {slave_ns} remains waiting.")

    def assign_offsets_along_route(self):
        """
        Distributes robots along the global CPP route with uniform offsets.
        """
        if not self.node.global_cpp_route:
            self.node.get_logger().warn("No global CPP route available.")
            return

        active_slaves = list(self.node.slaves.values())
        k = len(active_slaves)
        if k == 0:
            self.node.get_logger().warn("No slaves to assign offset.")
            return

        route_length = len(self.node.global_cpp_route)
        if route_length < k:
            self.node.get_logger().warn("Fewer route nodes than slaves. Some slaves may start at same node.")

        for i, slave in enumerate(active_slaves):
            # Calcola l'indice di offset
            offset_index = int(round(i * (route_length / k))) % route_length
            slave.assigned_waypoints = self.node.global_cpp_route.copy()
            slave.current_waypoint_index = offset_index

            start_label = self.node.global_cpp_route[offset_index]
            slave.current_node = start_label
            next_index = (offset_index + 1) % route_length
            next_label = self.node.global_cpp_route[next_index]

            # Controlla e occupa l'arco
            edge_key = tuple(sorted([start_label, next_label]))
            if edge_key not in self.node.occupied_edges:
                self.node.occupied_edges.add(edge_key)
                slave.current_edge = edge_key

            # Crea e pubblica il messaggio di waypoint
            data = self.node.full_graph.nodes[next_label]
            orientation_str = orientation_rad_to_str(data.get('orientation', 0.0))
            waypoint_msg = {
                'label': next_label,
                'x': data['x'],
                'y': data['y'],
                'orientation': orientation_str
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            slave.publisher.publish(msg)
            slave.current_waypoint_index = next_index

            # Log di Assegnazione degli Offset
            self.node.get_logger().info(
                f"Assigned offset waypoint '{next_label}' to slave '{slave.slave_ns}' "
                f"(offset_index={offset_index}, start_node={start_label}). Waypoint data: {waypoint_msg}"
            )

        self.node.partitioning_done = True

    def repartition_and_assign_waypoints(self):
        """
        Partitions the graph and assigns waypoints to active slaves.
        """
        num_ready_slaves = sum(1 for slave in self.node.slaves.values() if slave.ready)
        if num_ready_slaves == 0:
            self.node.get_logger().warn("No ready slaves found. Waiting for slaves to be ready.")
            self.node.partitioning_done = False
            return

        # Reset assigned nodes
        self.assigned_nodes = set()

        # Collect ready slaves
        ready_slaves = [slave for slave in self.node.slaves.values() if slave.ready]

        # Partiziona il grafo in sottografi per ogni ready slave
        from .graph_utils import partition_graph_wrapper
        subgraphs_with_start = partition_graph_wrapper(self.node.full_graph, num_ready_slaves)

        self.node.get_logger().info(f"Partitioned the graph into {len(subgraphs_with_start)} subgraphs.")
        self.print_subgraphs([sg for sg, _ in subgraphs_with_start])

        # Assegna ogni subgraph a uno slave
        for idx, slave in enumerate(ready_slaves):
            if idx >= len(subgraphs_with_start):
                self.node.get_logger().warn(f"More ready slaves ({len(ready_slaves)}) than subgraphs ({len(subgraphs_with_start)}).")
                break

            subgraph, starting_node = subgraphs_with_start[idx]
            slave.subgraph = subgraph
            slave.starting_node = starting_node
            slave.current_node = starting_node  # Imposta il nodo corrente al nodo iniziale

            # Segna i nodi come assegnati
            self.assigned_nodes.update(subgraph.nodes())

            # Calcola il percorso CPP
            waypoints = self.extract_waypoints_from_subgraph(subgraph)
            route = calculate_undirected_cpp_route(waypoints, subgraph, self.node.get_logger())
            slave.assigned_waypoints = route
            slave.current_waypoint_index = 0

            # Assegna il primo waypoint
            self.assign_initial_waypoint(slave.slave_ns)

        self.node.partitioning_done = True

    def extract_waypoints_from_subgraph(self, subgraph: nx.Graph) -> list:
        """
        Converts the nodes of the subgraph into a list of dicts (label, x, y, orientation).
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

    def print_subgraphs(self, subgraphs: list):
        """
        Logs details of each subgraph for debugging purposes.
        """
        self.node.get_logger().info("----- Subgraphs After Partition -----")
        for idx, subg in enumerate(subgraphs):
            self.node.get_logger().info(f"Subgraph {idx+1}:")
            self.node.get_logger().info(f"  Nodes ({len(subg.nodes())}):")
            for n, d in subg.nodes(data=True):
                self.node.get_logger().info(f"    {n} -> (x={d.get('x')}, y={d.get('y')})")
            self.node.get_logger().info(f"  Edges ({len(subg.edges())}):")
            for u, v, ed in subg.edges(data=True):
                self.node.get_logger().info(f"    {u}-{v}, distance={ed.get('distance')}")
        self.node.get_logger().info("----- End of Subgraphs -----")
