import json
from std_msgs.msg import String

from .slave_state import SlaveState
from .path_calculation import calculate_undirected_cpp_route


class MasterCallbacks:
    """
    Callback methods for the Master node.
    """

    def __init__(self):
        """
        Inizializza la classe MasterCallbacks.
        """
        self.global_cpp_route = []  # Sarà la rotta euleriana comune

    def publish_navigation_graph(self):
        """
        Pubblica il grafo di navigazione su un topic ROS.
        """
        graph_msg = String()
        graph_data = {
            'nodes': [],
            'edges': []
        }
        # Scorriamo tutti i nodi del grafo
        for node, data in self.full_graph.nodes(data=True):
            graph_data['nodes'].append({
                'label': node,
                'x': data.get('x', 0.0),
                'y': data.get('y', 0.0),
            })
        # Scorriamo tutti gli archi del grafo
        for u, v, data in self.full_graph.edges(data=True):
            if u < v:  # Evita duplicati su grafo NON diretto
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
        Calcola il percorso CPP globale sul grafo e lo salva in self.global_cpp_route.
        """
        waypoints = []
        for node, data in self.full_graph.nodes(data=True):
            waypoint = {
                'label': node,
                'x': data['x'],
                'y': data['y']
            }
            waypoints.append(waypoint)

        # Calcolo del route con DCPP
        route_nodes = calculate_undirected_cpp_route(waypoints, self.full_graph, self.get_logger())
        self.global_cpp_route = route_nodes
        if route_nodes:
            self.get_logger().info(f"Global CPP route computed: {route_nodes}")
        else:
            self.get_logger().error("Global CPP route is empty or Euler circuit not found.")

    def slave_registration_callback(self, msg):
        """
        Callback che gestisce la registrazione di un nuovo slave.
        """
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().nanoseconds / 1e9

        if slave_ns not in self.slaves:
            # Crea un publisher per inviare waypoint a questo slave
            publisher = self.create_publisher(String, f"/{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            self.slaves[slave_ns] = slave_state

            self.get_logger().info(f"Registered new slave: {slave_ns}")

            # Se la rotta globale non è ancora stata calcolata, calcolala
            if not self.global_cpp_route:
                self.compute_global_cpp_route()

            # Assegna gli offset e i waypoints usando il WaypointManager
            self.waypoint_manager.repartition_and_assign_waypoints()

        else:
            # Aggiorna last_seen_time
            self.slaves[slave_ns].last_seen_time = current_time
            self.get_logger().debug(f"Updated last seen time for slave: {slave_ns}")

    def navigation_status_callback(self, msg):
        """
        Processes navigation status updates from slaves.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_wpt = data['current_waypoint']
            error_message = data.get('error_message', '')
            traversed = data.get('traversed_edge', [])

            # Normalizziamo l'arco (se esattamente due nodi)
            if len(traversed) == 2:
                edge_key = tuple(sorted(traversed))
            else:
                edge_key = None

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid navigation status message: {e}")
            return

        # Se slave non registrato, gestiscilo oppure ignora
        if slave_ns not in self.slaves:
            self.get_logger().warn(f"Received status from unknown slave '{slave_ns}'. Registering slave.")
            return

        slave = self.slaves[slave_ns]

        if status == "ready":
            # Se c'è un edge occupato ancora...
            if slave.current_edge is not None:
                self.get_logger().debug(
                    f"[READY] Slave '{slave_ns}' still has current_edge={slave.current_edge}. Ignoring extra 'ready'."
                )
                return

            if not slave.ready:
                slave.ready = True
                self.get_logger().info(f"Slave '{slave_ns}' is ready.")

                # Assegna i waypoint solo dopo che lo slave è pronto
                self.waypoint_manager.repartition_and_assign_waypoints()
            else:
                self.get_logger().debug(f"Slave '{slave_ns}' was already ready.")

        elif status == "reached":
            self.get_logger().info(
                f"Slave '{slave_ns}' reached waypoint '{current_wpt}'. traversed_edge={traversed}"
            )
            if edge_key:
                self.get_logger().info(f"Traversed edge: {edge_key}")
                self.get_logger().info(
                    f"Current occupied edges before freeing: {self.occupied_edges}"
                )
                if edge_key in self.occupied_edges:
                    # Libera dal set
                    self.occupied_edges.remove(edge_key)
                    # Recuperiamo chi lo occupava
                    occupant = self.edge_occupants.pop(edge_key, None)
                    self.get_logger().info(
                        f"Edge {edge_key} freed by slave '{slave_ns}'. (Previously occupant={occupant})"
                    )
                else:
                    self.get_logger().warn(
                        f"Received 'reached' for edge {edge_key} from slave '{slave_ns}' but it was NOT in occupied_edges."
                    )
            else:
                self.get_logger().warn(f"Slave '{slave_ns}' says 'reached' but no valid edge_key.")

            # Aggiorna la posizione corrente dello slave
            slave.current_node = current_wpt
            slave.current_edge = None

            # Passa al prossimo waypoint
            self.waypoint_manager.assign_next_waypoint(slave_ns)

        elif status == "error":
            self.get_logger().error(f"Slave '{slave_ns}' error: {error_message}")
            # Se l'arco è occupato, liberiamolo
            if edge_key and edge_key in self.occupied_edges:
                self.occupied_edges.remove(edge_key)
                occupant = self.edge_occupants.pop(edge_key, None)
                self.get_logger().info(
                    f"Edge {edge_key} freed by error from '{slave_ns}'. occupant was={occupant}"
                )

            # Rimuovi lo slave
            del self.slaves[slave_ns]
            self.get_logger().warn(f"Removed slave '{slave_ns}' due to error.")

        elif status == "traversing":
            self.get_logger().debug(
                f"Slave '{slave_ns}' is traversing toward '{current_wpt}'."
            )
            # eventuali update di stato

        else:
            self.get_logger().warn(f"Unhandled status '{status}' from '{slave_ns}'.")


    def timer_callback(self):
        """
        Funzione periodica per:
        - check_slaves_timeout
        - assign_waiting_slaves
        """
        self.get_logger().debug("Master timer callback triggered.")
        self.check_slaves_timeout()
        self.waypoint_manager.assign_waiting_slaves()

    def check_slaves_timeout(self):
        """
        Identifica e rimuove gli slave che hanno superato il periodo di timeout.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        slaves_to_remove = []

        for slave_ns, slave in self.slaves.items():
            if current_time - slave.last_seen_time > self.timeout:
                self.get_logger().warn(f"Slave {slave_ns} has timed out. Removing from active slaves.")
                slaves_to_remove.append(slave_ns)

        for slave_ns in slaves_to_remove:
            if slave_ns in self.slaves:
                s = self.slaves[slave_ns]
                # Libera l'edge occupato, se presente
                if s.current_edge is not None and s.current_edge in self.occupied_edges:
                    self.occupied_edges.remove(s.current_edge)
                    occupant = self.edge_occupants.pop(s.current_edge, None)
                    self.get_logger().info(
                        f"Freed edge {s.current_edge} from occupant={occupant} due to slave timeout."
                    )

                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removed slave {slave_ns} due to timeout.")

        if slaves_to_remove:
            self.get_logger().info("Repartitioning and reassigning waypoints after slave removal.")
            self.waypoint_manager.repartition_and_assign_waypoints()
