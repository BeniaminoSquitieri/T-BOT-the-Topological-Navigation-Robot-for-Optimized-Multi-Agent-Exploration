import json
import networkx as nx
from std_msgs.msg import String

from .path_calculation import calculate_undirected_cpp_route


class WaypointManager:
    """
    Manages waypoint assignments to slaves, ensuring no conflicts in edge usage.
    """
    def __init__(self, node):
        self.node = node  # Reference to the MasterNavigationNode
        self.assigned_nodes = set()

    def assign_initial_waypoint(self, slave_ns: str):
        """
        Assigns the first waypoint to a newly registered slave.
        """
        self.node.get_logger().info(f"==== OCCUPIED EDGES BEFORE ASSIGNING FIRST WAYPOINT ====")
        self.node.get_logger().info(f"Occupied edges: {self.node.occupied_edges}")
        self.node.get_logger().info(f"Edge occupants: {self.node.edge_occupants}")
        self.node.get_logger().info(f"========================================================")

        # Chiama direttamente assign_next_waypoint
        self.assign_next_waypoint(slave_ns)
        slave = self.node.slaves[slave_ns]

        if not slave.assigned_waypoints:
            self.node.get_logger().warn(f"No waypoints to assign to slave {slave_ns}.")
            return

        # Determina il waypoint
        waypoint_label = slave.assigned_waypoints[slave.current_waypoint_index]
        from_node = slave.current_node
        to_node = waypoint_label

        # Creiamo la edge_key in modo ordinato
        edge_key = tuple(sorted([from_node, to_node]))

        # Se è libero, occupalo, altrimenti metti lo slave in attesa
        if edge_key in self.node.occupied_edges:
            self.node.get_logger().warn(
                f"Edge {edge_key} is occupied. {slave_ns} must wait for initial waypoint."
            )
            slave.waiting = True
            return
        else:
            self.node.occupied_edges.add(edge_key)
            slave.current_edge = edge_key

        # Crea e pubblica il messaggio di waypoint
        data = self.node.full_graph.nodes[to_node]
        waypoint_msg = {
            'label': to_node,
            'x': data['x'],
            'y': data['y'],
        }
        msg = String()
        msg.data = json.dumps(waypoint_msg)
        slave.publisher.publish(msg)

        self.node.get_logger().info(
            f"Assigned initial waypoint '{to_node}' to slave '{slave_ns}'. "
            f"Waypoint data: {waypoint_msg}"
        )

        # Aggiorna l'indice
        slave.current_waypoint_index = (slave.current_waypoint_index + 1) % len(slave.assigned_waypoints)


    def assign_next_waypoint(self, slave_ns: str):
        """
        Assegna il prossimo waypoint allo slave specificato.
        """
        # NOTA: Qui dobbiamo riferirci a self.node (che è il MasterNavigationNode)
        slave = self.node.slaves[slave_ns]

        # Se non abbiamo waypoints assegnati
        if not slave.assigned_waypoints:
            self.node.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
            return

        # Se abbiamo terminato il percorso, eventualmente ricominciamo
        if slave.current_waypoint_index >= len(slave.assigned_waypoints):
            self.node.get_logger().info(f"All waypoints assigned to {slave_ns} have been traversed. Restarting route.")
            slave.current_waypoint_index = 0

        next_node = slave.assigned_waypoints[slave.current_waypoint_index]
        from_node = slave.current_node

        # 1) Se il next_node == from_node, salta l'assegnazione
        if next_node == from_node:
            self.node.get_logger().warn(
                f"Slave '{slave_ns}' next_node == current_node ({next_node}); skipping to next index."
            )
            slave.current_waypoint_index += 1
            # Riprova ad assegnare il prossimo waypoint
            self.assign_next_waypoint(slave_ns)
            return

        edge_key = tuple(sorted([from_node, next_node]))

        # 2) Verifica se l'arco è già occupato
        if edge_key in self.node.occupied_edges:
            occupant = self.node.edge_occupants.get(edge_key, "Unknown")
            if occupant != slave_ns:
                self.node.get_logger().warn(
                    f"Edge {edge_key} is occupied by '{occupant}'. {slave_ns} must wait."
                )
                slave.waiting = True
                return
            else:
                self.node.get_logger().info(
                    f"Edge {edge_key} is already occupied by '{slave_ns}'. Proceeding with assignment."
                )

        # 3) Assegna l'arco se non è occupato
        if edge_key not in self.node.occupied_edges:
            self.node.occupied_edges.add(edge_key)
            self.node.edge_occupants[edge_key] = slave_ns
            slave.current_edge = edge_key
            slave.waiting = False

        # 4) Crea e pubblica il messaggio di waypoint
        data = self.node.full_graph.nodes[next_node]
        waypoint_msg = {
            'label': next_node,
            'x': data['x'],
            'y': data['y'],
        }
        msg = String()
        msg.data = json.dumps(waypoint_msg)
        slave.publisher.publish(msg)

        self.node.get_logger().info(
            f"Assigned next waypoint '{next_node}' to slave '{slave_ns}'. "
            f"Edge {edge_key} is now occupied by {slave_ns}."
        )

        # 5) Avanza l'indice del waypoint
        slave.current_waypoint_index += 1


    def assign_waiting_slaves(self):
        """
        Assigns waypoints to slaves that are currently waiting because an edge was occupied.
        """
        for slave_ns in sorted(self.node.slaves.keys()):
            slave = self.node.slaves[slave_ns]
            if not slave.waiting:
                continue

            if not slave.assigned_waypoints:
                self.node.get_logger().warn(f"No waypoints assigned to slave {slave_ns}.")
                continue

            waypoint_label = slave.assigned_waypoints[slave.current_waypoint_index]
            from_node = slave.current_node
            to_node = waypoint_label
            edge_key = tuple(sorted([from_node, to_node]))

            # Se l'arco si è liberato...
            if edge_key not in self.node.occupied_edges:
                self.node.occupied_edges.add(edge_key)
                self.node.edge_occupants[edge_key] = slave_ns
                slave.current_edge = edge_key
                slave.waiting = False

                data = self.node.full_graph.nodes[to_node]
                waypoint_msg = {
                    'label': to_node,
                    'x': data['x'],
                    'y': data['y'],
                }
                msg = String()
                msg.data = json.dumps(waypoint_msg)
                slave.publisher.publish(msg)

                self.node.get_logger().info(
                    f"Assigned waiting waypoint '{to_node}' to slave '{slave_ns}'. "
                    f"Waypoint data: {waypoint_msg}"
                )

                slave.current_node = to_node
                slave.current_waypoint_index += 1
            else:
                # L'arco è ancora occupato
                self.node.get_logger().warn(
                    f"Edge {edge_key} is still occupied. {slave_ns} remains waiting."
                )

    def assign_offsets_along_route(self):
        """
        Distributes robots along the global CPP route with uniform offsets.
        """
        self.node.get_logger().info("==== assign_offsets_along_route() START ====")
        
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
            self.node.get_logger().warn("Fewer route nodes than slaves. Some slaves may start at the same node.")

        for i, slave in enumerate(active_slaves):
            offset_index = int(round(i * (route_length / k))) % route_length
            # Assegna la route intera
            slave.assigned_waypoints = self.node.global_cpp_route.copy()
            slave.current_waypoint_index = offset_index
            
            start_label = self.node.global_cpp_route[offset_index]
            slave.current_node = start_label

            next_index = (offset_index + 1) % route_length
            next_label = self.node.global_cpp_route[next_index]

            # Log di debug
            self.node.get_logger().info(f"--- OFFSET for {slave.slave_ns} ---")
            self.node.get_logger().info(f"  offset_index = {offset_index}")
            self.node.get_logger().info(f"  start_node   = {start_label}")
            self.node.get_logger().info(f"  next_waypoint= {next_label}")
            self.node.get_logger().info(f"  assigned_waypoints: {slave.assigned_waypoints}")
            self.node.get_logger().info(f"----------------------")

            # Controlla e occupa l'arco (start_label, next_label)
            edge_key = tuple(sorted([start_label, next_label]))
            if edge_key not in self.node.occupied_edges:
                self.node.occupied_edges.add(edge_key)
                self.node.edge_occupants[edge_key] = slave.slave_ns
                slave.current_edge = edge_key

            # Pubblica subito il next_label come primo waypoint
            data = self.node.full_graph.nodes[next_label]
            waypoint_msg = {
                'label': next_label,
                'x': data['x'],
                'y': data['y'],
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            slave.publisher.publish(msg)

            # Aggiorniamo l’indice
            slave.current_waypoint_index = next_index

            # Log conclusivo
            self.node.get_logger().info(
                f"Assigned offset waypoint '{next_label}' to slave '{slave.slave_ns}' "
                f"(offset_index={offset_index}). "
                f"--- Edges Occupied Now: {self.node.occupied_edges}"
            )

        self.node.partitioning_done = True
        self.node.get_logger().info("==== assign_offsets_along_route() END ====")

    def repartition_and_assign_waypoints(self):
        """
        Assegna gli offset ai robot in base al numero totale di waypoints (self.node.global_cpp_route)
        e al numero di robot attivi.
        """
        num_slaves = len(self.node.slaves)
        if num_slaves == 0:
            self.node.get_logger().warn("No slaves to assign waypoints.")
            return

        total_waypoints = len(self.node.global_cpp_route)
        if total_waypoints == 0:
            self.node.get_logger().warn("Global CPP route is empty.")
            return

        # Pulisci i set/dict occupati prima di riassegnare
        self.node.get_logger().info("Clearing occupied_edges before repartition/offset assignment...")
        self.node.occupied_edges.clear()
        self.node.edge_occupants.clear()

        offset = total_waypoints // num_slaves
        # Assegna offset a ciascuno slave
        for idx, (slave_ns, slave) in enumerate(self.node.slaves.items()):
            start_index = (idx * offset) % total_waypoints
            assigned_route = self.node.global_cpp_route[start_index:] + self.node.global_cpp_route[:start_index]
            slave.assigned_waypoints = assigned_route
            slave.current_waypoint_index = 0

            # Impostiamo la current_node sul primo waypoint
            first_node = assigned_route[0]
            slave.current_node = first_node

            self.node.get_logger().info(
                f"Assigned offset={offset} route to slave '{slave_ns}'. Starting at index={start_index} "
                f"first_node={first_node}"
            )

            # Assegniamo il prossimo waypoint
            self.assign_next_waypoint(slave_ns)


    def extract_waypoints_from_subgraph(self, subgraph: nx.Graph) -> list:
        """
        Se volessi estrarre i waypoints da un subgrafo (non più necessario nella strategia con offset).
        """
        waypoints = []
        for node, data in subgraph.nodes(data=True):
            waypoint = {
                'label': node,
                'x': data['x'],
                'y': data['y'],
            }
            waypoints.append(waypoint)
        return waypoints

    def print_subgraphs(self, subgraphs: list):
        """
        Logs details of each subgraph for debugging purposes (non più essenziale in strategia offset).
        """
        self.node.get_logger().info("----- Subgraphs After Partition -----")
        for idx, subg in enumerate(subgraphs):
            self.node.get_logger().info(f"Subgraph {idx+1}:")
            self.node.get_logger().info(f"  Nodes ({len(subg.nodes())}):")
            for n, d in subg.nodes(data=True):
                self.node.get_logger().info(f"    {n} -> (x={d.get('x')}, y={d.get('y')})")
            self.node.get_logger().info(f"  Edges ({len(subg.edges())}):")
            for u, v, ed in subg.edges(data=True):
                self.node.get_logger().info(
                    f"    {u}-{v}, distance={ed.get('distance')}"
                )
        self.node.get_logger().info("----- End of Subgraphs -----")
