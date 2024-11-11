import os
import argparse
import time
import rclpy
from rclpy.node import Node
import json
import math
import networkx as nx
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class NavigationNode(Node):
    """
    Nodo ROS2 per la navigazione utilizzando una struttura a grafo ottimizzata con il Problema del Postino Cinese Orientato.

    Attributi:
        nx_graph (networkx.DiGraph): Rappresentazione del grafo diretto utilizzando NetworkX.
        navigator (TurtleBot4Navigator): Istanza di TurtleBot4Navigator per la navigazione del robot.
        dcpp_route (list): Percorso calcolato dal DCPP come lista di coppie di nodi (u, v).
        current_node_label (str): Etichetta del nodo corrente in cui si trova il robot.
    """
    def __init__(self, graph_file_path, starting_point):
        """
        Inizializza il nodo di navigazione.

        Args:
            graph_file_path (str): Percorso al file JSON contenente la struttura del grafo.
            starting_point (dict): Punto di partenza con chiavi 'x' e 'y' che rappresentano le coordinate.
        """
        super().__init__('navigation_node')  # Costruttore della classe base Node con nome 'navigation_node'

        self.navigator = TurtleBot4Navigator()  # Crea un'istanza del navigatore per controllare il robot

        # Carica il grafo dal file JSON e costruisce il grafo diretto
        self.load_graph(graph_file_path)

        # Calcola il percorso del Problema del Postino Cinese Orientato (DCPP)
        self.dcpp_route = self.compute_dcpp_route(self.nx_graph)

        if not self.dcpp_route:
            self.get_logger().error("Impossibile calcolare il percorso DCPP.")
            return

        # Trova il nodo più vicino al punto di partenza
        self.current_node_label = self.find_nearest_node(starting_point)
        self.get_logger().info(f"Punto di partenza: {self.current_node_label}")

        # Imposta la posa iniziale del robot al nodo di partenza
        self.set_initial_pose(self.current_node_label)

        # Naviga attraverso il percorso DCPP a partire dal nodo corrente
        self.navigate_dcpp_route()

    def load_graph(self, graph_file_path):
        """
        Carica il grafo da un file JSON e costruisce un DiGraph di NetworkX.

        Args:
            graph_file_path (str): Percorso al file JSON contenente la struttura del grafo.
        """
        # Carica i dati del grafo dal file JSON
        with open(graph_file_path, 'r') as f:
            data = json.load(f)

        # Crea un grafo diretto utilizzando NetworkX
        self.nx_graph = nx.DiGraph()  # Costruttore per un grafo diretto

        # Aggiunge i nodi al grafo con le loro coordinate come attributi
        for node in data['nodes']:
            label = node['label']  # Etichetta del nodo (es. 'node_1')
            x = node['x']          # Coordinata X del nodo
            y = node['y']          # Coordinata Y del nodo
            # Aggiunge il nodo al grafo con gli attributi
            self.nx_graph.add_node(label, x=x, y=y)

        # Aggiunge gli archi al grafo con pesi calcolati (distanza euclidea)
        for edge in data['edges']:
            u = edge['from']  # Nodo di partenza dell'arco
            v = edge['to']    # Nodo di arrivo dell'arco
            x1, y1 = self.nx_graph.nodes[u]['x'], self.nx_graph.nodes[u]['y']  # Coordinate del nodo u
            x2, y2 = self.nx_graph.nodes[v]['x'], self.nx_graph.nodes[v]['y']  # Coordinate del nodo v
            weight = math.hypot(x2 - x1, y2 - y1)  # Calcola il peso come distanza euclidea
            # Aggiunge l'arco al grafo con il peso calcolato
            self.nx_graph.add_edge(u, v, weight=weight)

    def compute_dcpp_route(self, G):
        """
        Calcola il percorso DCPP per il grafo diretto G fornito.

        Args:
            G (networkx.DiGraph): Il grafo diretto su cui calcolare il percorso DCPP.

        Returns:
            list: Il percorso DCPP come lista di archi (u, v).
        """
        # Verifica se il grafo è fortemente connesso
        if not nx.is_strongly_connected(G):
            self.get_logger().error("Il grafo diretto non è fortemente connesso.")
            return None

        # Calcola lo sbilanciamento per ogni nodo (grado uscente - grado entrante)
        imbalances = {v: G.out_degree(v) - G.in_degree(v) for v in G.nodes()}
        positive_nodes = [v for v, imbalance in imbalances.items() if imbalance > 0]  # Nodi con sbilanciamento positivo
        negative_nodes = [v for v, imbalance in imbalances.items() if imbalance < 0]  # Nodi con sbilanciamento negativo

        # Crea una lista di coppie di nodi sbilanciati con i costi per bilanciare il grafo
        imbalance_pairs = []
        for u in positive_nodes:
            for v in negative_nodes:
                x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
                x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
                distance = math.hypot(x2 - x1, y2 - y1)  # Calcola il costo come distanza euclidea
                imbalance_pairs.append((u, v, distance))

        # Crea un nuovo grafo M per rappresentare il flusso minimo di costo
        M = nx.DiGraph()
        M.add_nodes_from(G.nodes(data=True))  # Aggiunge tutti i nodi da G a M
        M.add_edges_from(G.edges(data=True))  # Aggiunge tutti gli archi esistenti da G a M

        # Aggiunge archi tra i nodi sbilanciati con capacità e peso
        for u, v, cost in imbalance_pairs:
            capacity = min(imbalances[u], -imbalances[v])  # Capacità massima possibile tra u e v
            if capacity > 0:
                M.add_edge(u, v, capacity=capacity, weight=cost)  # Aggiunge l'arco con capacità e costo

        # Aggiunge un super sorgente e un super pozzo al grafo M
        M.add_node('super_source')
        M.add_node('super_sink')

        # Collega il super sorgente ai nodi con sbilanciamento positivo
        for v in positive_nodes:
            M.add_edge('super_source', v, capacity=imbalances[v], weight=0)

        # Collega i nodi con sbilanciamento negativo al super pozzo
        for v in negative_nodes:
            M.add_edge(v, 'super_sink', capacity=-imbalances[v], weight=0)

        # Calcola il flusso di costo minimo nel grafo per bilanciare il grafo originale
        flowDict = nx.algorithms.flow.min_cost_flow(M, 'super_source', 'super_sink')

        # Aggiunge gli archi necessari al grafo originale G in base al flusso calcolato
        for u in G.nodes():
            for v in G.nodes():
                flow = flowDict.get(u, {}).get(v, 0)
                if flow > 0:
                    for _ in range(flow):
                        # Aggiunge l'arco con lo stesso peso dell'arco originale
                        G.add_edge(u, v, weight=G[u][v]['weight'])

        # Dopo il bilanciamento, verifica se il grafo è ora Euleriano
        if nx.is_eulerian(G):
            self.get_logger().info("Il grafo è stato bilanciato ed è ora Euleriano.")
            circuit = list(nx.eulerian_circuit(G))  # Trova il circuito Euleriano nel grafo bilanciato
            return circuit
        else:
            self.get_logger().error("Impossibile bilanciare il grafo per renderlo Euleriano.")
            return None

    def find_nearest_node(self, point):
        """
        Trova il nodo più vicino nel grafo al punto dato.

        Args:
            point (dict): Un punto con chiavi 'x' e 'y'.

        Returns:
            str: L'etichetta del nodo più vicino.
        """
        min_distance = float('inf')
        nearest_node_label = None
        x0, y0 = point['x'], point['y']  # Coordinate del punto di partenza
        for node_label, data in self.nx_graph.nodes(data=True):
            x1, y1 = data['x'], data['y']  # Coordinate del nodo
            distance = math.hypot(x1 - x0, y1 - y0)  # Calcola la distanza euclidea
            if distance < min_distance:
                min_distance = distance
                nearest_node_label = node_label
        return nearest_node_label

    def set_initial_pose(self, node_label):
        """
        Imposta la posa iniziale del robot al nodo dato.

        Args:
            node_label (str): L'etichetta del nodo.
        """
        x, y = self.nx_graph.nodes[node_label]['x'], self.nx_graph.nodes[node_label]['y']  # Coordinate del nodo
        initial_pose = self.navigator.getPoseStamped([x, y], 0.0)  # Crea un PoseStamped con orientamento 0 radianti
        self.navigator.setInitialPose(initial_pose)  # Imposta la posa iniziale utilizzando il navigatore
        time.sleep(1.0)

    def navigate_dcpp_route(self):
        """
        Naviga attraverso il percorso DCPP calcolato a partire dal nodo corrente.
        """
        # Riordina il circuito per iniziare dal nodo corrente
        circuit = self.reorder_circuit(self.dcpp_route, self.current_node_label)
        self.get_logger().info("Inizio navigazione attraverso il percorso DCPP.")

        for u, v in circuit:
            # Determina il prossimo nodo in base al nodo corrente
            if u == self.current_node_label:
                next_node_label = v
            elif v == self.current_node_label:
                next_node_label = u
            else:
                continue  # Salta gli archi non collegati al nodo corrente

            # Ottiene le coordinate del prossimo nodo
            x, y = self.nx_graph.nodes[next_node_label]['x'], self.nx_graph.nodes[next_node_label]['y']

            # Calcola l'orientamento verso il prossimo nodo
            orientation = self.compute_orientation(self.current_node_label, next_node_label)

            # Naviga verso il prossimo nodo
            self.navigate_to_node(next_node_label, orientation)

            # Aggiorna il nodo corrente
            self.current_node_label = next_node_label

        self.get_logger().info("Navigazione attraverso il percorso DCPP completata.")

    def reorder_circuit(self, circuit, start_node_label):
        """
        Riordina il circuito per iniziare dal nodo specificato.

        Args:
            circuit (list): Il circuito Euleriano come lista di archi (u, v).
            start_node_label (str): L'etichetta del nodo di partenza.

        Returns:
            list: Il circuito riordinato.
        """
        for i, (u, v) in enumerate(circuit):
            if u == start_node_label:
                return circuit[i:] + circuit[:i]  # Riordina il circuito per iniziare da start_node_label
        return circuit  # Se il nodo di partenza non è trovato, restituisce il circuito così com'è

    def compute_orientation(self, u_label, v_label):
        """
        Calcola l'angolo di orientamento dal nodo u al nodo v.

        Args:
            u_label (str): Etichetta del nodo u.
            v_label (str): Etichetta del nodo v.

        Returns:
            float: Angolo di orientamento in radianti.
        """
        x1, y1 = self.nx_graph.nodes[u_label]['x'], self.nx_graph.nodes[u_label]['y']  # Coordinate del nodo u
        x2, y2 = self.nx_graph.nodes[v_label]['x'], self.nx_graph.nodes[v_label]['y']  # Coordinate del nodo v
        angle = math.atan2(y2 - y1, x2 - x1)  # Calcola l'angolo tra i due nodi
        return angle

    def navigate_to_node(self, node_label, orientation_angle):
        """
        Naviga verso il nodo specificato.

        Args:
            node_label (str): Identificatore del nodo verso cui navigare.
            orientation_angle (float): Angolo di orientamento in radianti.
        """
        x, y = self.nx_graph.nodes[node_label]['x'], self.nx_graph.nodes[node_label]['y']  # Coordinate del nodo
        goal_pose = self.navigator.getPoseStamped([x, y], orientation_angle)  # Crea il goal pose
        self.navigator.startToPose(goal_pose)  # Avvia la navigazione verso il goal
        self.get_logger().info(f"Navigazione verso il nodo {node_label} a ({x}, {y}) con orientamento {orientation_angle:.2f} radianti.")
        # Attende fino a quando il robot raggiunge l'obiettivo
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)
        time.sleep(1.0)  # Attende un po' prima di passare al nodo successivo

def main(args=None):
    """
    Funzione principale per inizializzare ed eseguire il nodo di navigazione.
    """
    rclpy.init(args=args)  

    # Parsing degli argomenti da linea di comando per ottenere le coordinate di partenza
    parser = argparse.ArgumentParser(description='Nodo di Navigazione utilizzando DCPP')
    parser.add_argument('--start_x', type=float, required=True, help='Coordinata x di partenza')
    parser.add_argument('--start_y', type=float, required=True, help='Coordinata y di partenza')
    args = parser.parse_args()

    starting_point = {'x': args.start_x, 'y': args.start_y}  # Crea un dizionario per il punto di partenza

    # Ottiene il percorso assoluto della directory corrente dello script
    script_path = os.path.dirname(os.path.abspath(__file__))

    # Percorso al file JSON del grafo
    graph_path = os.path.join(script_path, '/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/map_transformation_phase/graph/navigation_graph_simplified.json')

    # Crea un'istanza del nodo di navigazione con il percorso del grafo e il punto di partenza
    navigation_node = NavigationNode(graph_path, starting_point)

    # Non è necessario eseguire rclpy.spin poiché non ci sono sottoscrizioni a topic
    # rclpy.spin(navigation_node)

    navigation_node.destroy_node()  # Distrugge il nodo
    rclpy.shutdown()  # Termina ROS2

if __name__ == '__main__':
    main()
