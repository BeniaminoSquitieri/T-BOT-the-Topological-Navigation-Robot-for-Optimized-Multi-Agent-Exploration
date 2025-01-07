#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math
import random
from threading import Lock, Event

# Master Tools (import solo se necessari per la parte "MasterCallbacks" - puoi eliminarli se non utilizzi la logica da master)
from fleet_turtlebot4_navigation.master.master_callbacks import MasterCallbacks
from fleet_turtlebot4_navigation.master.heartbeat_manager import HeartbeatManager
from fleet_turtlebot4_navigation.master.waypoint_manager import WaypointManager
from fleet_turtlebot4_navigation.master.graph_utils import load_full_graph_from_data

class SimulatedSlaveNavigationNode(Node, MasterCallbacks):
    """
    Simulated Slave Navigation Node that can elect itself as Master in case of Master failure.
    
    In questa versione, il nodo slave:
      - Non ha alcuna dipendenza dal parametro iniziale initial_node_label.
      - Parte con self.current_node = None, per indicare che non c’è una posizione (nodo) iniziale conosciuta.
      - Quando riceve il primo waypoint dal Master, imposta self.current_node alla destinazione, se necessario.
      - Può trasformarsi in Master usando l’algoritmo di elezione (Bully).
    """

    def __init__(self, robot_namespace='robot1'):
        """
        Inizializza il SimulatedSlaveNavigationNode in modalità SLAVE.
        
        Parametri:
            robot_namespace (str):
                Namespace identificativo di questo robot. Di default 'robot1'.
        """
        super().__init__('simulated_slave_navigation_node', namespace=robot_namespace)

        # Inizializza MasterCallbacks (solo se vuoi ereditare logiche di master).
        MasterCallbacks.__init__(self)
        
        # Namespace di questo slave
        self.robot_namespace = robot_namespace
        
        # Flag e stato principale
        self.is_master = False
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.heartbeat_timeout = 10.0
        
        # Parametro di timeout (per controllo slave)
        self.declare_parameter('timeout', 5.0)
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # Insieme di slave attivi (thread-safe)
        self.active_slaves = set()
        self.active_slaves_lock = Lock()
        
        # Stato di navigazione
        self.graph_received = False
        self.navigation_graph = None
        self.current_node = None  # <<-- NIENTE più nodo iniziale
        self.assigned_waypoints = []
        self.is_navigating = False
        self.navigation_lock = Lock()
        
        # Strutture tipiche del Master
        self.slaves = {}
        self.occupied_edges = set()
        self.edge_occupants = {}
        self.partitioning_done = False
        self.heartbeat_manager = None
        self.waypoint_manager = None
        
        # Parametro di intervallo per i timer da master
        self.declare_parameter('check_interval', 5.0)
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        
        # Publisher e Subscriber da SLAVE
        self.slave_registration_publisher = self.create_publisher(String, '/slave_registration', 10)
        self.heartbeat_publisher = self.create_publisher(String, '/slave_heartbeat', 10)
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)
        
        self.election_publisher = self.create_publisher(String, '/election', 10)
        self.coordinator_publisher = self.create_publisher(String, '/coordinator', 10)
        
        self.navigation_commands_subscriber = self.create_subscription(
            String, 'navigation_commands', self.slave_navigation_commands_callback, 10
        )
        self.master_heartbeat_subscriber = self.create_subscription(
            String, '/master_heartbeat', self.master_heartbeat_callback, 1
        )
        self.graph_subscriber = self.create_subscription(
            String, '/navigation_graph', self.slave_navigation_graph_callback, 10
        )
        self.election_subscriber = self.create_subscription(
            String, '/election', self.election_callback, 10
        )
        self.coordinator_subscriber = self.create_subscription(
            String, '/coordinator', self.coordinator_callback, 10
        )
        
        # Timer da SLAVE
        self.registration_timer = self.create_timer(1.0, self.publish_registration)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        self.check_master_timer = self.create_timer(1.0, self.check_master_alive)
        
        # Flag e eventi per l’elezione
        self.election_in_progress = False
        self.election_event = Event()
        
        # Ritardo random in startup
        startup_delay = random.uniform(0, 2)
        self.get_logger().info(
            f"[{self.robot_namespace}] Starting with a delay of {startup_delay:.2f}s to reduce election conflicts."
        )
        time.sleep(startup_delay)
        
        # Log di avvio
        self.get_logger().info(
            f"[{self.robot_namespace}] Started as SLAVE simulator with NO initial node."
        )
        # Nel costruttore __init__ dello slave
        self.first_wp_notification_sent = False
        self.first_wp_reached_pub = self.create_publisher(String, '/first_waypoint_reached', 10)

    # =========================================================================
    # Metodi di pubblicazione e callback SLAVE
    # =========================================================================
    
    def publish_registration(self):
        msg = String()
        msg.data = self.robot_namespace
        self.slave_registration_publisher.publish(msg)
        self.get_logger().debug(f"[{self.robot_namespace}] Published slave registration.")
    
    def publish_heartbeat(self):
        hb = String()
        hb.data = self.robot_namespace
        self.heartbeat_publisher.publish(hb)
        self.get_logger().debug(f"[{self.robot_namespace}] Published slave heartbeat.")
    
    def master_heartbeat_callback(self, msg):
        self.master_alive = True
        self.last_master_heartbeat = time.time()
        self.get_logger().debug(f"[{self.robot_namespace}] Received Master heartbeat.")
    
    def check_master_alive(self):
        if self.is_master:
            return
        
        if self.master_alive:
            self.master_alive = False
        else:
            now = time.time()
            if now - self.last_master_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(f"[{self.robot_namespace}] Master heartbeat lost => Initiating Election!")
                self.elect_new_master()

    # =========================================================================
    # Elezioni (Bully)
    # =========================================================================
    
    def elect_new_master(self):
        if self.election_in_progress:
            self.get_logger().info(f"[{self.robot_namespace}] Election already in progress.")
            return
        
        self.election_in_progress = True
        self.election_event.clear()
        
        higher_nodes = [node for node in self.active_slaves if node > self.robot_namespace]
        
        if not higher_nodes:
            self.get_logger().info(f"[{self.robot_namespace}] No higher nodes found. Becoming MASTER!")
            self.become_master()
            self.election_in_progress = False
            self.election_event.set()
            return
        
        for node in higher_nodes:
            election_msg = {"type": "ELECTION", "sender": self.robot_namespace}
            msg = String()
            msg.data = json.dumps(election_msg)
            self.election_publisher.publish(msg)
            self.get_logger().debug(f"[{self.robot_namespace}] Sent ELECTION to '{node}'.")
        
        wait_time = 5
        self.get_logger().info(f"[{self.robot_namespace}] Waiting {wait_time}s for OK responses.")
        self.election_event.wait(wait_time)
        
        if not self.election_in_progress:
            self.get_logger().info(f"[{self.robot_namespace}] Another node took over the election.")
            return
        
        self.get_logger().info(f"[{self.robot_namespace}] No OK received => Becoming MASTER!")
        self.become_master()
        self.election_in_progress = False
        self.election_event.set()

    def election_callback(self, msg):
        try:
            election_msg = json.loads(msg.data)
            if election_msg.get("type") != "ELECTION":
                return
            sender = election_msg.get("sender")
            if sender and sender < self.robot_namespace:
                ok_msg = {"type": "OK", "sender": self.robot_namespace}
                response = String()
                response.data = json.dumps(ok_msg)
                self.election_publisher.publish(response)
                self.get_logger().info(f"[{self.robot_namespace}] Received ELECTION from '{sender}', sent OK.")
                self.elect_new_master()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding ELECTION message: {e}")

    def coordinator_callback(self, msg):
        try:
            coord_msg = json.loads(msg.data)
            if coord_msg.get("type") != "COORDINATOR":
                return
            sender = coord_msg.get("sender")
            if sender:
                self.get_logger().info(f"[{self.robot_namespace}] COORDINATOR from '{sender}'. That is the new Master.")
                self.is_master = False
                self.master_alive = True
                self.last_master_heartbeat = time.time()
                self.election_in_progress = False
                self.election_event.set()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding COORDINATOR message: {e}")

    # =========================================================================
    # Gestione del Grafo in modalità SLAVE
    # =========================================================================
    
    def slave_navigation_graph_callback(self, msg):
        if self.is_master:
            return
        
        if self.graph_received:
            self.get_logger().debug(f"[{self.robot_namespace}] Graph already received, ignoring.")
            return
        
        try:
            data = json.loads(msg.data)
            self.navigation_graph = load_full_graph_from_data(data)
            self.graph_received = True
            # Non settiamo un "initial_node", self.current_node resta None
            
            # Pubblico status "ready"
            ready_data = {
                'robot_namespace': self.robot_namespace,
                'status': 'ready',
                'error_message': '',
                'time_taken': 0.0,
                'current_waypoint': str(self.current_node),  # None come stringa
                'traversed_edge': []
            }
            rmsg = String()
            rmsg.data = json.dumps(ready_data)
            self.status_publisher.publish(rmsg)
            self.get_logger().info(f"[{self.robot_namespace}] Graph received, published 'ready' status.")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding graph as slave: {e}")

    # =========================================================================
    # Ricezione dei Waypoints in modalità SLAVE
    # =========================================================================
    
    def slave_navigation_commands_callback(self, msg):
        if self.is_master:
            return
        
        try:
            waypoint_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[{self.robot_namespace}] Error decoding waypoint: {e}")
            return
        
        if not self.graph_received or self.navigation_graph is None:
            self.get_logger().warn(f"[{self.robot_namespace}] No graph available => cannot navigate.")
            self.publish_status("error", "NoGraph", 0.0, waypoint_data.get('label','?'), [])
            return

        with self.navigation_lock:
            self.assigned_waypoints.append(waypoint_data)
            self.get_logger().info(f"[{self.robot_namespace}] Received waypoint: {waypoint_data}")
            if not self.is_navigating:
                self.is_navigating = True
                self.execute_waypoints_in_sequence()

    def execute_waypoints_in_sequence(self):
        while self.assigned_waypoints:
            wpt = self.assigned_waypoints.pop(0)
            self.simulate_navigation(wpt)
        self.is_navigating = False

    def simulate_navigation(self, waypoint):
        label = waypoint['label']
        x = waypoint['x']
        y = waypoint['y']

        # Se non ho current_node, mi teletrasporto:
        if self.current_node is None:
            self.current_node = label
            # Pubblico "reached"
            self.publish_status("reached", "", 0.0, label, [label, label])

            # Pubblico la notifica del primo waypoint raggiunto (solo se non l’ho già fatto)
            if not self.first_wp_notification_sent:
                notif = {"robot_namespace": self.robot_namespace}
                msg_notif = String()
                msg_notif.data = json.dumps(notif)
                self.first_wp_reached_pub.publish(msg_notif)
                self.first_wp_notification_sent = True
            
            return
            
        if self.current_node == label:
            self.get_logger().info(f"[{self.robot_namespace}] Already at '{label}'.")
            return
        
        if (self.current_node not in self.navigation_graph.nodes) or (label not in self.navigation_graph.nodes):
            err_msg = f"Current node='{self.current_node}' or destination='{label}' not in the graph"
            self.get_logger().error(f"[{self.robot_namespace}] {err_msg}")
            self.publish_status("error", err_msg, 0.0, label, [])
            return

        cx = self.navigation_graph.nodes[self.current_node]['x']
        cy = self.navigation_graph.nodes[self.current_node]['y']
        dist = math.hypot(x - cx, y - cy)
        speed = 10.31
        ttime = dist / speed
        
        trav_edge = [self.current_node, label]
        self.publish_status("traversing", "", 0.0, label, trav_edge)
        self.get_logger().info(
            f"[{self.robot_namespace}] Navigating from '{self.current_node}' to '{label}' ~{ttime:.2f}s."
        )
        
        time.sleep(ttime)  # Simulazione dell'attraversamento
        self.current_node = label
        self.get_logger().info(f"[{self.robot_namespace}] Reached '{label}' in {ttime:.2f}s.")
        self.publish_status("reached", "", ttime, label, trav_edge)
        # Se sto raggiungendo il PRIMO waypoint “reale” e non ho ancora inviato la notifica:
        if not self.first_wp_notification_sent:
            notif = {"robot_namespace": self.robot_namespace}
            msg_notif = String()
            msg_notif.data = json.dumps(notif)
            self.first_wp_reached_pub.publish(msg_notif)
            self.first_wp_notification_sent = True




    def publish_status(self, status, error_message, time_taken, current_waypoint, trav):
        st = {
            'robot_namespace': self.robot_namespace,
            'status': status,
            'error_message': error_message,
            'time_taken': time_taken,
            'current_waypoint': current_waypoint,
            'traversed_edge': trav
        }
        s = String()
        s.data = json.dumps(st)
        self.status_publisher.publish(s)
        self.get_logger().debug(f"[{self.robot_namespace}] Published status: {st}")

    # =========================================================================
    # Diventare MASTER
    # =========================================================================
    
    def become_master(self):
        self.is_master = True
        self.get_logger().info(f"[{self.robot_namespace}] => MASTER MODE activated")
        
        self.heartbeat_manager = HeartbeatManager(self)
        self.heartbeat_manager.start_publishing()
        
        self.waypoint_manager = WaypointManager(self)
        
        self.slaves = {}
        self.occupied_edges.clear()
        self.edge_occupants.clear()
        
        self.slave_registration_subscriber = self.create_subscription(
            String, '/slave_registration', self.slave_registration_callback, 10
        )
        self.navigation_status_subscriber = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10
        )
        
        if self.navigation_graph is not None:
            self.full_graph = self.navigation_graph
            self.get_logger().info(f"[{self.robot_namespace}] (MASTER) Using navigation_graph as full_graph.")
            
            self.graph_publisher = self.create_publisher(String, '/navigation_graph', 10)
            self.graph_timer = self.create_timer(1.0, self.publish_navigation_graph)
            
            self.compute_global_cpp_route()
            self.get_logger().info(f"Global CPP route computed: {self.global_cpp_route}")
            
            self.waypoint_manager.repartition_and_assign_waypoints()
        else:
            self.get_logger().warn(f"[{self.robot_namespace}] No navigation graph => cannot assign routes as MASTER.")
        
        self.master_timer = self.create_timer(self.check_interval, self.master_timer_callback)
        
        self.heartbeat_timer.cancel()
        self.get_logger().info(f"[{self.robot_namespace}] Stopped publishing slave heartbeat.")
        
        with self.active_slaves_lock:
            if self.robot_namespace in self.active_slaves:
                self.active_slaves.remove(self.robot_namespace)
                self.get_logger().info(f"[{self.robot_namespace}] Removed itself from active_slaves.")
        
        self.get_logger().info(f"[{self.robot_namespace}] => Master setup complete, ready to listen to slaves.")

    def master_timer_callback(self):
        self.check_slaves_timeout()
        self.waypoint_manager.assign_waiting_slaves()

    # =========================================================================
    # Override dei metodi di MasterCallbacks (se is_master == True)
    # =========================================================================
    
    def slave_registration_callback(self, msg):
        if not self.is_master:
            return
        super().slave_registration_callback(msg)

    def navigation_status_callback(self, msg):
        if not self.is_master:
            return
        super().navigation_status_callback(msg)

    def send_coordinator_message(self):
        coord_msg = {"type": "COORDINATOR", "sender": self.robot_namespace}
        msg = String()
        msg.data = json.dumps(coord_msg)
        self.coordinator_publisher.publish(msg)
        self.get_logger().info(f"[{self.robot_namespace}] Sent COORDINATOR message.")

    # =========================================================================
    # Esecuzione
    # =========================================================================
    
    def run(self):
        rclpy.spin(self)

    def destroy_node(self):
        if self.is_master and self.heartbeat_manager:
            self.heartbeat_manager.stop_publishing()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser(description='Simulated Slave Node (can become Master) without an initial node.')
    parser.add_argument('--robot_namespace', type=str, default='robot1', help='Namespace of this slave')
    parsed_args, _ = parser.parse_known_args()

    node = SimulatedSlaveNavigationNode(robot_namespace=parsed_args.robot_namespace)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()