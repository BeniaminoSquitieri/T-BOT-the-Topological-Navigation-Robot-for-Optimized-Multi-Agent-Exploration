#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import networkx as nx
from .graph_partitioning import load_full_graph, partition_graph
from .path_calculation import calculate_dcpp_route, orientation_conversion

class SlaveState:
    """
    Classe per gestire lo stato di ciascun slave robot.
    """
    def __init__(self, slave_ns, publisher):
        self.slave_ns = slave_ns
        self.publisher = publisher
        self.assigned_waypoints = []  # Lista di waypoint assegnati
        self.current_waypoint_index = 0  # Indice del prossimo waypoint da assegnare
        self.last_seen_time = 0.0  # Ultima volta che lo slave ha comunicato

class MasterNavigationNode(Node):
    def __init__(self):
        super().__init__('master_navigation_node')

        # Dichiarazione dei parametri ROS 2 con valori predefiniti
        self.declare_parameter('graph_path', '')
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('timeout', 10.0)  # Timeout per slave inattivi in secondi

        # Recupero dei valori dei parametri
        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Caricamento del grafo completo dal file JSON specificato
        self.full_graph = load_full_graph(self.graph_path)
        self.get_logger().info("Master node loaded the graph.")

        # Stampa di tutti i nodi del grafo caricato
        # self.print_all_graph_nodes()

        # Inizializzazione dei subscriber per la registrazione degli slave e le posizioni iniziali
        self.slave_registration_subscriber = self.create_subscription(
            String,
            'slave_registration',
            self.slave_registration_callback,
            10
        )

        self.initial_position_subscriber = self.create_subscription(
            String,
            'slave_initial_positions',
            self.initial_position_callback,
            10
        )

        # Subscriber per ricevere aggiornamenti sullo stato della navigazione dagli slave
        self.navigation_status_subscriber = self.create_subscription(
            String,
            'navigation_status',
            self.navigation_status_callback,
            10
        )

        # Publisher per inviare messaggi di heartbeat indicando che il master è attivo
        self.heartbeat_publisher = self.create_publisher(String, 'master_heartbeat', 10)

        # Creazione di un timer che pubblica messaggi di heartbeat ogni 1 secondo
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Timer per controllare lo stato dei slave e assegnare nuovi waypoints
        self.timer = self.create_timer(self.check_interval, self.timer_callback)

        # Dizionario per tracciare gli slave attivi e il loro stato
        self.slaves = {}  # Chiave: slave_ns, Valore: SlaveState

        # Flag per indicare se il partizionamento del grafo e l'assegnazione dei waypoint sono stati completati
        self.partitioning_done = False

    def print_all_graph_nodes(self):
        """
        Stampa tutti i nodi del grafo con le loro coordinate.
        """
        self.get_logger().info("Printing all graph nodes:")
        for node, data in self.full_graph.nodes(data=True):
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            orientation = data.get('orientation', 0.0) 
            self.get_logger().info(f"Node {node}: x={x}, y={y}, orientation={orientation} radians")

    def publish_heartbeat(self):
        """
        Pubblica un messaggio di heartbeat per indicare che il master è attivo.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = "alive"
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug("Published heartbeat.")

    def slave_registration_callback(self, msg):
        """
        Callback funzione attivata quando viene ricevuto un messaggio di registrazione di uno slave.
        Aggiorna il dizionario slaves con il namespace dello slave e l'ora corrente.
        Inizializza le strutture dati per nuovi slave.
        """
        slave_ns = msg.data.strip()
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        if slave_ns not in self.slaves:
            self.get_logger().info(f"New slave registered: {slave_ns}")

            # Crea un publisher per inviare comandi di navigazione allo slave
            publisher = self.create_publisher(String, f"{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            self.slaves[slave_ns] = slave_state

            # Ripartizione del grafo e assegnazione dei waypoint poiché un nuovo slave è stato aggiunto
            self.repartition_and_assign_waypoints()
        else:
            self.get_logger().info(f"Slave re-registered: {slave_ns}")
            # Aggiorna il tempo dell'ultimo aggiornamento dello slave
            self.slaves[slave_ns].last_seen_time = current_time

    def initial_position_callback(self, msg):
        """
        Callback funzione attivata quando viene ricevuta la posizione iniziale di uno slave.
        Si aspetta una stringa JSON con 'robot_namespace', 'x', 'y', 'orientation'.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            initial_x = data['x']
            initial_y = data['y']
            initial_orientation = data['orientation']
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid initial position message: {e}")
            return

        current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        if slave_ns in self.slaves:
            self.slaves[slave_ns].last_seen_time = current_time
            self.get_logger().info(
                f"Received initial position from {slave_ns}: "
                f"({initial_x}, {initial_y}, {initial_orientation} radians)"
            )
        else:
            # Slave non registrato ancora, registralo ora
            self.get_logger().info(f"Initial position received for unregistered slave {slave_ns}, registering now.")
            publisher = self.create_publisher(String, f"{slave_ns}/navigation_commands", 10)
            slave_state = SlaveState(slave_ns, publisher)
            slave_state.last_seen_time = current_time
            self.slaves[slave_ns] = slave_state
            self.get_logger().info(f"Created publisher for slave: {slave_ns}")

            # Ripartizione del grafo e assegnazione dei waypoint poiché un nuovo slave è stato aggiunto
            self.repartition_and_assign_waypoints()

    def timer_callback(self):
        """
        Callback funzione periodica per:
        - Controllare se ci sono slave inattivi basati sul timeout.
        - Rimuovere gli slave che hanno superato il timeout.
        - Ripartizionare e ri-assegnare i waypoint se gli slave attivi sono cambiati.
        """
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        removed_slaves = []

        # Controllo dei timeout degli slave
        for slave_ns in list(self.slaves.keys()):
            slave = self.slaves[slave_ns]
            if current_time - slave.last_seen_time > self.timeout:
                removed_slaves.append(slave_ns)
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Slave {slave_ns} timed out and removed from active slaves.")

        if removed_slaves:
            # Ripartizione del grafo e ri-assegnazione dei waypoint poiché gli slave attivi sono cambiati
            self.get_logger().info("Active slaves changed. Repartitioning graph and reassigning waypoints.")
            self.repartition_and_assign_waypoints()

        # Stampa lo stato attuale degli slave
        # self.print_current_status()

    def navigation_status_callback(self, msg):
        """
        Gestisce il feedback degli slave riguardo lo stato della navigazione.
        Si aspetta una stringa JSON con 'robot_namespace', 'status', 'current_waypoint', 'time_taken', 'error_message'.
        """
        try:
            data = json.loads(msg.data)
            slave_ns = data['robot_namespace']
            status = data['status']
            current_waypoint = data['current_waypoint']
            time_taken = data['time_taken']
            error_message = data.get('error_message', '')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid navigation status message: {e}")
            return

        self.get_logger().info(
            f"Received status from {slave_ns}: {status}, "
            f"Waypoint: {current_waypoint}, Time Taken: {time_taken}s, Error: {error_message}"
        )

        current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        if slave_ns in self.slaves:
            slave = self.slaves[slave_ns]
            slave.last_seen_time = current_time

            if status == "reached":
                # Slave ha raggiunto il waypoint corrente, assegna il successivo
                self.get_logger().info(f"Slave {slave_ns} has reached waypoint {current_waypoint}.")
                slave.current_waypoint_index += 1
                self.assign_next_waypoint(slave)
            elif status == "error":
                # Gestione dello stato di errore
                self.get_logger().error(f"Slave {slave_ns} encountered an error: {error_message}")
                # Implementa la logica di retry o gestione degli errori se necessario
                # Per ora, rimuoviamo lo slave
                del self.slaves[slave_ns]
                self.get_logger().warn(f"Removing slave {slave_ns} due to error.")
                self.repartition_and_assign_waypoints()
        else:
            self.get_logger().warn(f"Received status from unknown slave {slave_ns}.")

    def repartition_and_assign_waypoints(self):
        """
        Partiziona il grafo di navigazione in base al numero di slave attivi e assegna waypoint a ciascuno slave.
        Calcola il percorso DCPP (circuito Euleriano) per ogni sottografo.
        """
        num_slaves = len(self.slaves)
        if num_slaves == 0:
            self.get_logger().warn("No active slaves found. Waiting for slaves to register.")
            self.partitioning_done = False
            return

        # Partiziona il grafo in sottografi basati sul numero di slave
        try:
            subgraphs = partition_graph(self.full_graph, num_slaves)
            self.get_logger().info(f"Partitioned the graph into {len(subgraphs)} subgraphs.")
        except ValueError as e:
            self.get_logger().error(f"Failed to partition graph: {e}")
            return

        # Ordina gli slave per garantire un'assegnazione coerente
        slaves = sorted(self.slaves.keys())

        # Assicurati che il numero di sottografi corrisponda al numero di slave
        if len(subgraphs) != len(slaves):
            self.get_logger().error("Number of subgraphs does not match number of active slaves.")
            return

        # Assegna ciascun sottografo a uno slave
        for idx, slave_ns in enumerate(slaves):
            subgraph = subgraphs[idx]
            waypoints = self.extract_waypoints(subgraph)
            # Calcola il percorso DCPP (circuito Euleriano) per il sottografo
            dcpp_route = calculate_dcpp_route(waypoints, subgraph, self.get_logger())
            # Assegna il percorso allo slave
            self.assign_route_to_slave(slave_ns, dcpp_route)

        self.partitioning_done = True

    def extract_waypoints(self, subgraph):
        """
        Estrae i waypoint da un sottografo.

        Args:
            subgraph (nx.Graph): Sottografo da cui estrarre i waypoint.

        Returns:
            list of dict: Lista di waypoint con 'label', 'x', 'y', e 'orientation'.
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

    def assign_route_to_slave(self, slave_ns, route):
        """
        Assegna un percorso di waypoint a uno slave specifico.

        Args:
            slave_ns (str): Namespace dello slave robot.
            route (list of dict): Lista ordinata di waypoint.
        """
        if slave_ns not in self.slaves:
            self.get_logger().error(f"No slave state found for {slave_ns}. Cannot assign route.")
            return

        slave = self.slaves[slave_ns]
        slave.assigned_waypoints = route.copy()
        slave.current_waypoint_index = 0

        # Assegna il primo waypoint
        self.assign_next_waypoint(slave)

        # Log del numero di waypoint assegnati
        self.get_logger().info(f"Assigned {len(route)} waypoints to {slave_ns}.")

    def assign_next_waypoint(self, slave):
        """
        Assegna il prossimo waypoint nella coda dello slave.

        Args:
            slave (SlaveState): Lo slave a cui assegnare il waypoint.
        """
        if slave.current_waypoint_index < len(slave.assigned_waypoints):
            waypoint = slave.assigned_waypoints[slave.current_waypoint_index]
            waypoint_msg = {
                'label': waypoint['label'],
                'x': waypoint['x'],
                'y': waypoint['y'],
                'orientation': orientation_conversion(waypoint['orientation'])
            }
            msg = String()
            msg.data = json.dumps(waypoint_msg)
            slave.publisher.publish(msg)
            self.get_logger().info(f"Assigned waypoint to {slave.slave_ns}: {waypoint_msg}")
        else:
            # Tutti i waypoints sono stati assegnati, ricomincia dal primo
            self.get_logger().info(f"All waypoints have been assigned to {slave.slave_ns}. Restarting the route.")
            slave.current_waypoint_index = 0
            self.assign_next_waypoint(slave)  # Assegna di nuovo il primo waypoint

    def print_current_status(self):
        """
        Stampa lo stato attuale degli slave attivi.
        """
        num_active_slaves = len(self.slaves)
        self.get_logger().info(f"Current active slaves: {num_active_slaves}")
        for slave_ns in sorted(self.slaves.keys()):
            slave = self.slaves[slave_ns]
            status = "Assigned" if slave.current_waypoint_index < len(slave.assigned_waypoints) else "Completed"
            self.get_logger().info(f"Slave {slave_ns}: Status: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = MasterNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#ros2 run fleet_turtlebot4_navigation master_navigation_node --ros-args -p graph_path:=/home/beniamino/turtlebot4/diem_turtlebot_ws/src/fleet_turtlebot4_navigation/map/navigation_hardware_limitation.json 
