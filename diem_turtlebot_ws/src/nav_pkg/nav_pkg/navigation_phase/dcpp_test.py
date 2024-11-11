import json
import math
import networkx as nx
import argparse
import os

def load_graph(graph_file_path):
    """
    Carica il grafo da un file JSON e costruisce un DiGraph di NetworkX.

    Args:
        graph_file_path (str): Percorso al file JSON contenente la struttura del grafo.

    Returns:
        networkx.DiGraph: Il grafo diretto con pesi sugli archi.
    """
    with open(graph_file_path, 'r') as f:
        data = json.load(f)

    # Crea un grafo diretto utilizzando NetworkX
    G = nx.DiGraph()

    # Aggiungi i nodi con le loro coordinate come attributi
    for node in data['nodes']:
        G.add_node(node['label'], x=node['x'], y=node['y'])

    # Aggiungi gli archi con peso calcolato in base alla distanza euclidea
    for edge in data['edges']:
        u, v = edge['from'], edge['to']
        x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        weight = math.hypot(x2 - x1, y2 - y1)
        G.add_edge(u, v, weight=weight)

    return G

def compute_dcpp_route(G):
    """
    Calcola il percorso DCPP per il grafo diretto G fornito.

    Args:
        G (networkx.DiGraph): Il grafo diretto su cui calcolare il percorso DCPP.

    Returns:
        list: Il percorso DCPP come lista di archi (u, v).
    """
    if not nx.is_strongly_connected(G):
        print("Errore: Il grafo non è fortemente connesso.")
        return None

    # Bilanciamento dei nodi sbilanciati
    imbalances = {v: G.out_degree(v) - G.in_degree(v) for v in G.nodes()}
    positive_nodes = [v for v, imbalance in imbalances.items() if imbalance > 0]
    negative_nodes = [v for v, imbalance in imbalances.items() if imbalance < 0]

    imbalance_pairs = []
    for u in positive_nodes:
        for v in negative_nodes:
            x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
            x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
            distance = math.hypot(x2 - x1, y2 - y1)
            imbalance_pairs.append((u, v, distance))

    M = nx.DiGraph()
    M.add_nodes_from(G.nodes(data=True))
    M.add_edges_from(G.edges(data=True))

    for u, v, cost in imbalance_pairs:
        capacity = min(imbalances[u], -imbalances[v])
        if capacity > 0:
            M.add_edge(u, v, capacity=capacity, weight=cost)

    M.add_node('super_source')
    M.add_node('super_sink')
    for v in positive_nodes:
        M.add_edge('super_source', v, capacity=imbalances[v], weight=0)
    for v in negative_nodes:
        M.add_edge(v, 'super_sink', capacity=-imbalances[v], weight=0)

    flowDict = nx.algorithms.flow.min_cost_flow(M, 'super_source', 'super_sink')

    for u in G.nodes():
        for v in G.nodes():
            flow = flowDict.get(u, {}).get(v, 0)
            if flow > 0:
                for _ in range(flow):
                    G.add_edge(u, v, weight=G[u][v]['weight'])

    if nx.is_eulerian(G):
        circuit = list(nx.eulerian_circuit(G))
        return circuit
    else:
        print("Errore: Impossibile bilanciare il grafo per renderlo Euleriano.")
        return None

def print_route(route, G, start_node_label):
    """
    Stampa i nodi e le direzioni attraversati nel percorso DCPP.

    Args:
        route (list): Lista di coppie di nodi (u, v) che rappresentano il percorso DCPP.
        G (networkx.DiGraph): Grafo contenente i nodi e le loro coordinate.
        start_node_label (str): Nodo di partenza per il percorso.
    """
    # Riordina il circuito per iniziare dal nodo di partenza
    reordered_route = reorder_route(route, start_node_label)

    for u, v in reordered_route:
        print(f"Da {u} a {v}")

def reorder_route(route, start_node_label):
    """
    Riordina il percorso DCPP per iniziare dal nodo di partenza specificato.

    Args:
        route (list): Lista di coppie di nodi (u, v) che rappresentano il percorso DCPP.
        start_node_label (str): Nodo di partenza desiderato.

    Returns:
        list: Percorso riordinato per iniziare dal nodo specificato.
    """
    for i, (u, v) in enumerate(route):
        if u == start_node_label:
            return route[i:] + route[:i]
    return route

def main():
    # Parsing degli argomenti della riga di comando per il nodo di partenza
    parser = argparse.ArgumentParser(description="Navigazione DCPP in un grafo.")
    parser.add_argument('--start_node', type=str, required=True, help="Nodo di partenza (es. 'node_1')")
    args = parser.parse_args()
    start_node_label = args.start_node

    # Percorso al file JSON del grafo
    script_path = os.path.dirname(os.path.abspath(__file__))
    graph_path = os.path.join(script_path, '/home/beniamino/turtlebot4/diem_turtlebot_ws/src/map/map_transformation_phase/graph/navigation_graph_simplified.json')

    # Carica il grafo dal file JSON
    G = load_graph(graph_path)

    # Calcola il percorso DCPP per il grafo
    route = compute_dcpp_route(G)

    # Se il percorso è stato calcolato, stampalo
    if route:
        print("\nPercorso DCPP (nodo e direzione):")
        print_route(route, G, start_node_label)
    else:
        print("Non è stato possibile calcolare il percorso DCPP.")

if __name__ == "__main__":
    main()
