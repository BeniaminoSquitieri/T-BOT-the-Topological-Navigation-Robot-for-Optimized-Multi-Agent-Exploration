import json
import networkx as nx
from std_msgs.msg import String

from .slave_state import SlaveState
from .path_calculation import calculate_undirected_cpp_route

class MasterCallbacks:
    """
    Callback methods for the Master node.
    """

    def __init__(self):
        self.global_cpp_route = []  # Sarà la rotta euleriana comune
        # Nel Node effettivo avremo: self.full_graph, self.slaves, etc.

    def publish_navigation_graph(self):
        graph_msg = String()
        graph_data = {
            'nodes': [],
            'edges': []
        }
        for node, data in self.full_graph.nodes(data=True):
            graph_data['nodes'].append({
                'label': node,
                'x': data.get('x', 0.0),
                'y': data.get('y', 0.0),
                'orientation': data.get('orientation', 0.0)
            })
        for u, v, data in self.full_graph.edges(data=True):
            if u < v:
                dist_value = data.get('weight', 1.0)
                graph_data['edges'].append({
                    'source': u,
                    'target': v,
                    'distance': dist_value
                })

        graph_msg.data = json.dumps(graph_data)
        self.graph_publisher.publish(graph_msg)
        self.get_logger().debug("Navigation graph published.")

    def compute_global_cpp_route(self):
        """
        Calcola l'intero percorso Eulero (CPP) sul grafo globale e lo salva in self.global_cpp_route.
        """
        waypoints = []
        for node, data in self.full_graph.nodes(data=True):
            wpt = {
                'label': node,
                'x': data['x'],
                'y': data['y']
            }
            waypoints.append(wpt)
        route_nodes = calculate_undirected_cpp_route(waypoints, self.full_graph, self.get_logger())
        self.global_cpp_route = route_nodes
        if route_nodes:
            self.get_logger().info(f"Global CPP route computed: {route_nodes}")
        else:
            self.get_logger().error("Global CPP route is empty or Euler circuit not found.")

    def slave_registration_callback(self, msg):
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns not in self.slaves:
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            self.slaves[slave_ns] = slave_state
            self.get_logger().info(f"Registered new slave: {slave_ns}")

            # Re-partiziona e riassegna (opzionale) OPPURE
            # semplicemente rifai la route globale se vuoi usare la strategia B (unica rotta globale + offset).
            # Qui implementiamo la Strategia B: UNICA rotta e offset diversi.

            if not self.global_cpp_route:
                self.compute_global_cpp_route()

            # Assegniamo offset in base al numero di slave registrati
            self.assign_offsets_along_route()
        else:
            self.slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"Updated last seen time for slave: {slave_ns}")

    def assign_offsets_along_route(self):
        """
        Distribuisce i robot lungo la rotta globale con offset uniformi.
        """
        # Se non abbiamo global_cpp_route, non facciamo nulla
        if not self.global_cpp_route:
            self.get_logger().warn("No global CPP route available.")
            return

        # Prendiamo la lista dei robot ready (o tutti i robot?)
        active_slaves = list(self.slaves.values())

        # Numero di posizioni
        k = len(active_slaves)
        if k == 0:
            self.get_logger().warn("No slaves to assign offset.")
            return

        # Calcoliamo la lunghezza della route come numero di nodi (oppure potremmo usare la "length" in metri).
        # Per semplicità usiamo len(self.global_cpp_route)
        route_length = len(self.global_cpp_route)
        if route_length < k:
            self.get_logger().warn("Fewer route nodes than slaves. Some slaves may start at same node.")
        
        # Assegna offset = i*(route_length/k)
        # Nota: potremmo farlo in modo float, ma i nodi sono discreti; usiamo int() rounding.
        for i, slave in enumerate(active_slaves):
            offset_index = int(round(i * (route_length / k))) % route_length
            # Assegniamo la rotta "completa" al robot, e settiamo current_waypoint_index = offset
            slave.assigned_waypoints = self.global_cpp_route.copy()
            slave.current_waypoint_index = offset_index

            # settiamo la current_node in base a offset_index
            start_label = self.global_cpp_route[offset_index]
            slave.current_node = start_label  # come se fosse lì all'avvio
            # Non occupiamo l'arco in partenza (non c'è from_node).
            # Mandiamo un "primo waypoint" al robot, cioè lo inviamo al node successivo
            next_index = (offset_index + 1) % route_length
            next_label = self.global_cpp_route[next_index]

            # Pubblica il next_label come waypoint
            if start_label == next_label:
                # Se la rotta di offset ha node uguale... strano, ma può succedere se route ha archi duplicati consecutivi
                self.get_logger().warn("start_label == next_label. Possibly a single-node circuit??")
            
            # Non occupiamo l'arco in anticipo, aspettiamo che lo slave ci dica "ready"?
            # Oppure assegniamo subito:
            # Esempio: assegniamo subito l'arco (start_label, next_label).
            from_node = start_label
            to_node = next_label
            edge_key = tuple(sorted([from_node, to_node]))
            if edge_key not in self.occupied_edges:
                self.occupied_edges.add(edge_key)
                slave.current_edge = edge_key

            # Invio
            data = self.full_graph.nodes[next_label]
            waypoint_msg = {
                'label': next_label,
                'x': data['x'],
                'y': data['y'],
            }
            msg = String()
            import json
            msg.data = json.dumps(waypoint_msg)
            slave.publisher.publish(msg)
            slave.current_waypoint_index = next_index  # lo slave partirà da offset_index e va al next_index

            self.get_logger().info(
                f"[Offset] Slave {slave.slave_ns} assigned offset_index={offset_index}, start_node={start_label}, next_waypoint={next_label}."
            )

    def navigation_status_callback(self, msg):
        """
        Process navigation status updates from slaves.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_waypoint = data['current_waypoint']
            error_message = data.get('error_message', '')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid navigation status message: {e}")
            return

        if slave_ns not in self.slaves:
            self.get_logger().warn(f"Received status from unknown slave {slave_ns}.")
            return

        slave = self.slaves[slave_ns]

        if status == "ready":
            if not slave.ready:
                slave.ready = True
                self.get_logger().info(f"Slave {slave_ns} is ready.")
                # Già fatto l'offset assignment in slave_registration_callback.  
                # Oppure potresti farlo *solo* adesso. Logiche alternative.

        elif status == "reached":
            # L'arco si libera
            from_node = slave.current_node
            to_node = current_waypoint
            edge_key = tuple(sorted([from_node, to_node]))
            if edge_key in self.occupied_edges:
                self.occupied_edges.remove(edge_key)
            # Aggiorna current_node
            slave.current_node = to_node

            # Passa al prossimo index (ciclico) e assegna
            route = slave.assigned_waypoints
            next_index = (slave.current_waypoint_index + 1) % len(route)
            next_label = route[next_index]
            slave.current_waypoint_index = next_index

            # Occupiamo l'arco e mandiamo il comando
            edge_key2 = tuple(sorted([to_node, next_label]))
            if edge_key2 in self.occupied_edges:
                self.get_logger().warn(f"Edge {edge_key2} is occupied. {slave_ns} must wait.")
                slave.waiting = True
                return
            else:
                self.occupied_edges.add(edge_key2)
                slave.current_edge = edge_key2

            data_node = self.full_graph.nodes[next_label]
            waypoint_msg = {
                'label': next_label,
                'x': data_node['x'],
                'y': data_node['y'],
            }
            m2 = String()
            m2.data = json.dumps(waypoint_msg)
            slave.publisher.publish(m2)

        elif status == "error":
            self.get_logger().error(f"Slave {slave_ns} encountered an error: {error_message}")
            # Libera l'arco e rimuovi lo slave
            from_node = slave.current_node
            to_node = current_waypoint
            edge_key = tuple(sorted([from_node, to_node])) if from_node and to_node else None
            if edge_key and edge_key in self.occupied_edges:
                self.occupied_edges.remove(edge_key)
            del self.slaves[slave_ns]
            self.get_logger().warn(f"Removed slave {slave_ns} due to error.")

        # Altri stati possibili ("traversing", ecc...) da gestire se desiderato.

    def timer_callback(self):
        """
        Controlli periodici (timeout e assegnazione edges in attesa).
        """
        self.get_logger().debug("Master timer callback triggered.")
        self.check_slaves_timeout()
        self.waypoint_manager.assign_waiting_slaves()
