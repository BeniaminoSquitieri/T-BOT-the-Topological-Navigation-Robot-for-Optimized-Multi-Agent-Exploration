#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import argparse
import math
import networkx as nx
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult
from .graph_partitioning import load_full_graph, partition_graph
from .path_calculation import calculate_dcpp_route

def load_full_graph_from_data(graph_data):
    """
    Carica un grafo NetworkX da un dizionario contenente nodi e archi.

    Args:
        graph_data (dict): Dizionario con 'nodes' e 'edges'.

    Returns:
        nx.DiGraph: Il grafo diretto caricato.
    """
    G = nx.DiGraph()

    for node in graph_data['nodes']:
        label = node['label']
        x = node['x']
        y = node['y']
        orientation = node.get('orientation', 0.0)
        G.add_node(label, x=x, y=y, orientation=orientation)

    for edge in graph_data['edges']:
        u = edge['from']
        v = edge['to']
        weight = edge.get('weight', 1.0)
        G.add_edge(u, v, weight=weight)

    return G

class SlaveState:
    """
    Classe per gestire lo stato di ciascun slave robot.
    """
    def __init__(self, slave_ns):
        self.slave_ns = slave_ns
        self.assigned_waypoints = []
        self.current_waypoint_index = 0
        self.last_seen_time = 0.0
        self.initial_x = None
        self.initial_y = None
        self.initial_orientation = None
        self.publisher = None
        self.waiting = False

class SlaveNavigationNode(Node):
    def __init__(self, robot_namespace, initial_x, initial_y, initial_orientation_str):
        self.robot_namespace = robot_namespace
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.initial_orientation_str = initial_orientation_str
        self.initial_orientation = self.orientation_conversion(initial_orientation_str)
        super().__init__('slave_navigation_node')

        # Publisher e Subscriber di base
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)
        self.initial_position_publisher = self.create_publisher(String, '/slave_initial_positions', 10)
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.navigation_commands_callback, 10
        )
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)
        self.navigation_status_subscriber = self.create_subscription(
            String,
            '/navigation_status',
            self.navigation_status_callback,
            10
        )
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 2
        )
        self.slave_heartbeat_subscriber = self.create_subscription(
            String, '/slave_heartbeat', self.slave_heartbeat_callback, 10
        )
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.navigation_graph_callback, 10
        )

        # Timers
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        self.master_check_timer = self.create_timer(10.0, self.check_master_alive)
        self.slave_check_timer = self.create_timer(2.0, self.check_slave_alive)
        # Timer per pubblicare regolarmente i messaggi di posizione iniziale
        self.initial_position_timer = self.create_timer(2.0, self.publish_initial_position)

        # Pubblica la posizione iniziale una volta all'avvio
        self.publish_initial_position()

        # Inizializza il TurtleBot4Navigator
        self.navigator = TurtleBot4Navigator()

        # Variabili per l'elezione del master
        self.master_alive = False
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 150.0

        # Lista degli slave attivi
        self.active_slaves = {}

        # Grafo di navigazione
        self.navigation_graph = None

        # Publisher per comandi di navigazione degli altri slave
        self.slave_command_publishers = {}

        # Posizioni iniziali degli altri slave
        self.slave_initial_positions = {}

        # Flag del ruolo di master
        self.is_master = False
        self.master_graph_partitioned = False

        # Waypoints assegnati a se stesso come master
        self.assigned_waypoints = []
        self.current_waypoint_index = 0

        # Variabili di navigazione corrente
        self.is_navigating = False
        self.current_goal_label = None
        self.current_goal_start_time = None

        # Timer per verificare stato navigazione
        self.navigation_check_timer = self.create_timer(0.5, self.check_navigation_status)

        self.get_logger().info(
            f"[{self.robot_namespace}] Slave node initialized at "
            f"({self.initial_x}, {self.initial_y}) with orientation {self.initial_orientation_str} ({self.initial_orientation} radians)."
        )

    def publish_registration(self):
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published registration.")

    def publish_initial_position(self):
        initial_position = {
            'robot_namespace': self.robot_namespace,
            'x': self.initial_x,
            'y': self.initial_y,
            'orientation': self.initial_orientation_str
        }
        msg = String()
        msg.data = json.dumps(initial_position)
        self.initial_position_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Published initial position: {initial_position}")
        self.initial_position_published = True
        self.initial_position_timer.cancel()  # Ferma il timer dopo la prima pubblicazione
    
    def publish_heartbeat(self):
        heartbeat_msg = String()
        heartbeat_msg.data = self.robot_namespace
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published heartbeat.")

    def master_heartbeat_callback(self, msg):
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received master heartbeat.")

    def slave_heartbeat_callback(self, msg):
        slave_ns = msg.data.strip()
        current_time = time.time()
        if slave_ns != self.robot_namespace:
            if slave_ns not in self.active_slaves:
                # Crea uno stato per il nuovo slave se non esiste
                self.active_slaves[slave_ns] = SlaveState(slave_ns)
                self.get_logger().info(f"[{self.robot_namespace}] Detected new slave: {slave_ns}")
            self.active_slaves[slave_ns] = current_time
            self.get_logger().debug(f"[{self.robot_namespace}] Received heartbeat from slave {slave_ns}.")

    def check_master_alive(self):
        current_time = time.time()
        if self.master_alive:
            self.master_alive = False
        else:
            if current_time - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost. Initiating master election.")
                self.elect_new_master()

    def check_slave_alive(self):
        current_time = time.time()
        for slave_ns in list(self.active_slaves.keys()):
            if current_time - self.active_slaves[slave_ns] > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Slave {slave_ns} heartbeat lost. Removing from active slaves.")
                del self.active_slaves[slave_ns]

    def elect_new_master(self):
        candidates = list(self.active_slaves.keys()) + [self.robot_namespace]
        if not candidates:
            self.get_logger().error(f"[{self.robot_namespace}] No candidates available for master election.")
            return

        candidates_sorted = sorted(candidates)
        new_master = candidates_sorted[0]

        if new_master == self.robot_namespace:
            self.get_logger().info(f"[{self.robot_namespace}] Elected as the new master.")
            self.become_master()
        else:
            self.get_logger().info(f"[{self.robot_namespace}] New master is {new_master}.")

    def become_master(self):
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] Now acting as the master.")
        # Stampiamo le informazioni disponibili come richiesto
        self.available_informations()

        if self.navigation_graph is not None:
            self.publish_navigation_graph()
            self.partition_and_assign_waypoints()
        else:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot become master.")

    def available_informations(self):
        """
        Stampa le informazioni disponibili: descrizione del grafo e slave disponibili.
        """
        self.get_logger().info("----- Available Information -----")
        self.get_logger().info("Graph description:")
        if self.navigation_graph is not None:
            for node, data in self.navigation_graph.nodes(data=True):
                self.get_logger().info(
                    f"  Node {node}: Position=({data['x']}, {data['y']}), Orientation={data.get('orientation',0.0)} radians"
                )
            for u, v, data in self.navigation_graph.edges(data=True):
                self.get_logger().info(f"  Edge from {u} to {v}, Weight: {data.get('weight',1.0)}")
        else:
            self.get_logger().info("No navigation graph available.")

        self.get_logger().info("Available slaves:")
        for slave_ns in self.active_slaves.keys():
            self.get_logger().info(f"  - {slave_ns}")

        self.get_logger().info("----- End of Available Information -----")

    def publish_navigation_graph(self):
        graph_msg = String()
        graph_data = {
            'nodes': [
                {'label': node, 'x': data['x'], 'y': data['y'], 'orientation': data.get('orientation', 0.0)}
                for node, data in self.navigation_graph.nodes(data=True)
            ],
            'edges': [
                {'from': u, 'to': v, 'weight': data.get('weight', 1.0)}
                for u, v, data in self.navigation_graph.edges(data=True)
            ]
        }
        graph_msg.data = json.dumps(graph_data)
        self.status_publisher.publish(graph_msg)
        self.get_logger().info("Published navigation graph as master.")

    def navigation_graph_callback(self, msg):
        try:
            graph_data = json.loads(msg.data)
            self.navigation_graph = load_full_graph_from_data(graph_data)
            if self.is_master and not self.master_graph_partitioned:
                self.partition_and_assign_waypoints()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation graph: {e}")

    def navigation_commands_callback(self, msg):
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to decode navigation command: {e}")
            return

        if isinstance(waypoint_data.get('orientation'), str):
            waypoint_data['orientation'] = self.orientation_conversion(waypoint_data['orientation'])

        self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")
        # Chiamiamo direttamente execute_navigation senza thread o asyncio
        self.execute_navigation(waypoint_data)

    def execute_navigation(self, waypoint):
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']
        orientation_rad = waypoint['orientation']

        self.get_logger().info(f"[{self.robot_namespace}] Navigating to {label} at ({x}, {y}) with orientation {orientation_rad} radians.")

        goal_pose = self.navigator.getPoseStamped([x, y], orientation_rad)
        start_time = time.time()

        try:
            # Verifica disponibilità action server
            if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                error_message = f"Action server not available for {label}. Skipping this waypoint."
                self.get_logger().error(error_message)
                # Pubblica uno stato di errore in modo che il master sappia che non possiamo raggiungere questo waypoint
                self.publish_status("error", error_message, 0.0, label)

                # Se siamo il master, passiamo subito al prossimo waypoint
                if self.is_master:
                    self.get_logger().warn(f"[{self.robot_namespace}] Skipping waypoint {label} and moving to next.")
                    self.current_waypoint_index += 1
                    self.assign_next_waypoint_as_master()

                return

            self.navigator.startToPose(goal_pose)
        except Exception as e:
            error_message = f"Exception occurred while sending goal to {label}: {e}"
            self.get_logger().error(error_message)
            self.publish_status("error", error_message, 0.0, label)
            return

        # Attende fino a completamento del task
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        time_taken = time.time() - start_time
        nav_result = self.navigator.getResult()

        if nav_result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"[{self.robot_namespace}] Reached {label} in {time_taken:.2f} seconds.")
            self.publish_status("reached", "", time_taken, label)
            self.current_waypoint_index += 1
            if self.is_master:
                self.assign_next_waypoint_as_master()
        else:
            error_message = f"Navigation to {label} failed with result code {nav_result}."
            self.get_logger().error(error_message)
            self.publish_status("error", error_message, time_taken, label)


    def check_navigation_status(self):
        if not self.is_navigating:
            return

        if self.navigator.isTaskComplete():
            time_taken = time.time() - self.current_goal_start_time
            nav_result = self.navigator.getResult()

            if nav_result == TaskResult.SUCCEEDED:
                self.get_logger().info(
                    f"[{self.robot_namespace}] Reached {self.current_goal_label} in {time_taken:.2f} seconds."
                )
                self.publish_status("reached", "", time_taken, self.current_goal_label)
                if self.is_master:
                    self.assign_next_waypoint_as_master()
            else:
                error_message = f"Navigation to {self.current_goal_label} failed with result code {nav_result}."
                self.get_logger().error(error_message)
                self.publish_status("error", error_message, time_taken, self.current_goal_label)

            self.is_navigating = False
            self.current_goal_label = None
            self.current_goal_start_time = None

    def publish_status(self, status, error_message, time_taken, current_waypoint):
        status_data = {
            'robot_namespace': self.robot_namespace,
            'status': status,
            'error_message': error_message,
            'time_taken': time_taken,
            'current_waypoint': current_waypoint
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)
        # Uncomment the line below for detailed status logging
        self.get_logger().info(f"[{self.robot_namespace}] Published status: {status_data}")

    def navigation_status_callback(self, msg):
        """
        Gestisce il feedback degli slave riguardo lo stato di navigazione.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_waypoint = data['current_waypoint']
            time_taken = data['time_taken']
            error_message = data.get('error_message', '')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"[{self.robot_namespace}] Invalid navigation status message: {e}")
            return

        current_time = time.time()

        if self.is_master:
            with self.lock:
                if slave_ns in self.active_slaves:
                    slave = self.active_slaves[slave_ns]
                elif slave_ns == self.robot_namespace:
                    slave = self  # Se stesso
                else:
                    self.get_logger().warn(f"[{self.robot_namespace}] Received status from unknown slave {slave_ns}.")
                    return

                slave.last_seen_time = current_time

                if status == "reached":
                    # Rimuovi il nodo dal set dei nodi occupati
                    if current_waypoint in self.occupied_nodes:
                        self.occupied_nodes.remove(current_waypoint)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {current_waypoint} is now free.")
                    else:
                        self.get_logger().warn(f"[{self.robot_namespace}] Node {current_waypoint} was not marked as occupied.")

                    self.get_logger().info(f"[{self.robot_namespace}] Slave {slave_ns} has reached waypoint {current_waypoint}.")
                    slave.waiting = False  # Lo slave non è più in attesa

                    # Assegna il prossimo waypoint
                    self.assign_next_waypoint(slave_ns)

                    # Dopo aver assegnato il prossimo waypoint, prova a assegnare waypoint agli slave in attesa
                    self.assign_waiting_slaves()

                elif status == "error":
                    # Gestione dello stato di errore
                    self.get_logger().error(f"[{self.robot_namespace}] Slave {slave_ns} encountered an error: {error_message}")
                    # Rimuovi il nodo dal set dei nodi occupati se necessario
                    if current_waypoint in self.occupied_nodes:
                        self.occupied_nodes.remove(current_waypoint)
                        self.get_logger().info(f"[{self.robot_namespace}] Node {current_waypoint} is now free due to error.")
                    # Rimuovi lo slave
                    if slave_ns in self.active_slaves:
                        del self.active_slaves[slave_ns]
                        self.get_logger().warn(f"[{self.robot_namespace}] Removing slave {slave_ns} due to error.")
                        # Ripartizione del grafo
                        self.partition_and_assign_waypoints()
        else:
            # Se non è master, nessuna azione aggiuntiva
            pass


    def print_subgraphs(self, subgraphs):
        """
        Stampa i dettagli di ciascun sottografo dopo la partizione.

        Args:
            subgraphs (list of nx.Graph): Lista di sottografi risultanti dalla partizione.
        """
        self.get_logger().info("----- Subgraphs After Partition -----")
        for idx, subgraph in enumerate(subgraphs):
            self.get_logger().info(f"Subgraph {idx+1}:")
            self.get_logger().info(f"  Nodes ({len(subgraph.nodes())}):")
            for node, data in subgraph.nodes(data=True):
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                orientation = data.get('orientation', 0.0)
                self.get_logger().info(f"    {node}: Position=({x}, {y}), Orientation={orientation} radians")
            self.get_logger().info(f"  Edges ({len(subgraph.edges())}):")
            for u, v, data in subgraph.edges(data=True):
                weight = data.get('weight', 1.0)
                self.get_logger().info(f"    From {u} to {v}, Weight: {weight}")
        self.get_logger().info("----- End of Subgraphs -----")



    def orientation_conversion(self, orientation_input):
        orientations = {
            'NORTH': 0.0,
            'EAST': -math.pi / 2,
            'SOUTH': math.pi,
            'WEST': math.pi / 2
        }
        if isinstance(orientation_input, str):
            return orientations.get(orientation_input.upper(), 0.0)
        elif isinstance(orientation_input, (float, int)):
            return float(orientation_input)
        else:
            return 0.0

    def partition_and_assign_waypoints(self):
        if self.navigation_graph is None:
            self.get_logger().error(f"[{self.robot_namespace}] Navigation graph not available. Cannot partition and assign waypoints.")
            return

        full_graph = self.navigation_graph
        all_slaves = list(self.active_slaves.keys()) + [self.robot_namespace]
        all_slaves_sorted = sorted(all_slaves)
        num_slaves = len(all_slaves_sorted)

        start_positions = []
        for slave_ns in all_slaves_sorted:
            if slave_ns == self.robot_namespace:
                start_positions.append({'x': self.initial_x, 'y': self.initial_y})
            else:
                if slave_ns in self.slave_initial_positions:
                    pos = self.slave_initial_positions[slave_ns]
                    start_positions.append({'x': pos['x'], 'y': pos['y']})
                else:
                    self.get_logger().warn(
                        f"[{self.robot_namespace}] Initial position for slave '{slave_ns}' not available. Using master position as fallback."
                    )
                    start_positions.append({'x': self.initial_x, 'y': self.initial_y})

        try:
            subgraphs = partition_graph(full_graph, num_slaves, start_positions=start_positions)
            self.get_logger().info(
                f"[{self.robot_namespace}] Partitioned the graph into {len(subgraphs)} subgraphs."
            )
        except ValueError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Failed to partition graph: {e}")
            return

        for idx, slave_ns in enumerate(all_slaves_sorted):
            subgraph = subgraphs[idx]
            waypoints = self.extract_waypoints(subgraph)

            if slave_ns == self.robot_namespace:
                self.assign_route_to_master(waypoints)
            else:
                if slave_ns not in self.slave_command_publishers:
                    topic_name = f"/{slave_ns}/navigation_commands"
                    publisher = self.create_publisher(String, topic_name, 10)
                    self.slave_command_publishers[slave_ns] = publisher
                    self.get_logger().info(
                        f"[{self.robot_namespace}] Created publisher for slave '{slave_ns}' on topic '{topic_name}'."
                    )

                publisher = self.slave_command_publishers[slave_ns]

                for waypoint in waypoints:
                    if isinstance(waypoint['orientation'], str):
                        orientation_rad = self.orientation_conversion(waypoint['orientation'])
                    else:
                        orientation_rad = waypoint['orientation']

                    waypoint_msg = {
                        'label': waypoint['label'],
                        'x': waypoint['x'],
                        'y': waypoint['y'],
                        'orientation': orientation_rad
                    }
                    msg = String()
                    msg.data = json.dumps(waypoint_msg)
                    publisher.publish(msg)
                    self.get_logger().info(f"[{self.robot_namespace}] Assigned waypoint to {slave_ns}: {waypoint_msg}")

        self.master_graph_partitioned = True

    def assign_route_to_master(self, waypoints):
        dcpp_route = calculate_dcpp_route(waypoints, self.navigation_graph, self.get_logger())
        self.assigned_waypoints = dcpp_route.copy()
        self.current_waypoint_index = 0

        self.get_logger().info(f"[{self.robot_namespace}] DCPP route assigned as master:")
        for wp in dcpp_route:
            self.get_logger().info(
                f" - {wp['label']} at ({wp['x']}, {wp['y']}) with orientation {wp['orientation']} radians"
            )

        self.assign_next_waypoint_as_master()
        self.get_logger().info(f"[{self.robot_namespace}] Assigned {len(dcpp_route)} waypoints as master.")

    def assign_next_waypoint_as_master(self):
        if not self.assigned_waypoints:
            self.get_logger().warn(f"[{self.robot_namespace}] No waypoints assigned. Cannot assign next waypoint.")
            return

        if self.current_waypoint_index < len(self.assigned_waypoints):
            waypoint = self.assigned_waypoints[self.current_waypoint_index]
            self.execute_navigation(waypoint)
        else:
            self.get_logger().info(f"[{self.robot_namespace}] All waypoints have been assigned. Restarting the route.")
            self.current_waypoint_index = 0
            waypoint = self.assigned_waypoints[self.current_waypoint_index]
            self.execute_navigation(waypoint)

    def extract_waypoints(self, subgraph):
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

def main(args=None):
    parser = argparse.ArgumentParser(description='Slave Navigation Node using TurtleBot4 with Master Replacement')
    parser.add_argument('--robot_namespace', type=str, required=True, help='Unique namespace of the robot (e.g., robot_1)')
    parser.add_argument('--initial_x', type=float, required=True, help='Initial x coordinate')
    parser.add_argument('--initial_y', type=float, required=True, help='Initial y coordinate')
    parser.add_argument('--initial_orientation', type=str, required=True, help='Initial orientation (NORTH, EAST, SOUTH, WEST)')

    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)

    node = SlaveNavigationNode(
        robot_namespace=parsed_args.robot_namespace,
        initial_x=parsed_args.initial_x,
        initial_y=parsed_args.initial_y,
        initial_orientation_str=parsed_args.initial_orientation
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
