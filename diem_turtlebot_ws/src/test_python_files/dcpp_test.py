import argparse
import json
import logging
import math
import networkx as nx
from pathlib import Path

# Import the functions from the provided code file
# Assumiamo che il codice fornito sia in un file chiamato 'path_calculation.py'
# e che si trovi nella stessa directory di questo test.
# Se si trova altrove, bisogna aggiungere al PYTHONPATH la directory corrispondente o modificare l'import di conseguenza.
from path_calculation import calculate_dcpp_route, orientation_str_to_rad, orientation_rad_to_str

def load_graph_from_json(graph_path):
    """
    Carica un grafo da un file JSON.
    Il file deve avere la struttura:
    {
        "nodes": [
            {"label": "node_1", "x": 0.0, "y": 0.0},
            {"label": "node_2", "x": 1.0, "y": 1.0},
            ...
        ],
        "edges": [
            {"from": "node_1", "to": "node_2"},
            {"from": "node_2", "to": "node_3"},
            ...
        ]
    }

    Args:
        graph_path (str): Il percorso del file JSON contenente il grafo.

    Returns:
        nx.DiGraph: Il grafo diretto con nodi (label, x, y) e archi con pesi calcolati.
    """
    with open(graph_path, 'r') as f:
        data = json.load(f)

    G = nx.DiGraph()

    # Aggiungiamo i nodi
    for node in data['nodes']:
        label = node['label']
        x = node['x']
        y = node['y']
        G.add_node(label, x=x, y=y)

    # Aggiungiamo gli archi con il peso come distanza euclidea
    for edge in data['edges']:
        u = edge['from']
        v = edge['to']
        x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        dist = math.hypot(x2 - x1, y2 - y1)
        G.add_edge(u, v, weight=dist)

    return G

def build_waypoints_from_graph(G, start_node_label):
    """
    Crea la lista dei waypoints a partire dai nodi del grafo.
    Il primo waypoint sarà lo start_node_label, e gli altri verranno ordinati semplicemente
    nell'ordine in cui appaiono i nodi, oppure potete anche ordinarli alfabeticamente.

    Per semplicità, assumiamo che tutti i waypoint abbiano orientamento 'NORTH' di default.
    In caso si voglia un orientamento differente si potrebbe fare un mapping ad hoc.

    Args:
        G (nx.DiGraph): Il grafo da cui prendere i nodi.
        start_node_label (str): Il nodo di partenza.
    
    Returns:
        list of dict: Lista di waypoint con 'label', 'x', 'y', 'orientation'
    """
    waypoints = []
    # Mettiamo lo start_node_label come primo waypoint
    if start_node_label not in G.nodes:
        raise ValueError(f"Start node {start_node_label} not found in graph.")

    # start waypoint
    waypoints.append({
        'label': start_node_label,
        'x': G.nodes[start_node_label]['x'],
        'y': G.nodes[start_node_label]['y'],
        'orientation': orientation_str_to_rad('NORTH')  # default NORTH
    })

    # Aggiungiamo gli altri nodi come waypoints (escludendo lo start se già aggiunto)
    for n in G.nodes:
        if n != start_node_label:
            waypoints.append({
                'label': n,
                'x': G.nodes[n]['x'],
                'y': G.nodes[n]['y'],
                'orientation': orientation_str_to_rad('NORTH')
            })

    return waypoints

def main():
    parser = argparse.ArgumentParser(description="Test per il calcolo del DCPP route.")
    parser.add_argument('--graph_path', type=str, required=True, help="Percorso del file JSON contenente il grafo.")
    parser.add_argument('--start_node', type=str, required=True, help="Nodo di partenza per il percorso DCPP.")
    args = parser.parse_args()

    graph_path = args.graph_path
    start_node_label = args.start_node

    # Creiamo un logger semplice
    logger = logging.getLogger("DCPP_Test")
    logger.setLevel(logging.INFO)
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    formatter = logging.Formatter('[%(levelname)s] %(message)s')
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    # Carichiamo il grafo
    if not Path(graph_path).exists():
        logger.error(f"File {graph_path} not found.")
        return

    logger.info(f"Loading graph from {graph_path}")
    G = load_graph_from_json(graph_path)

    # Creiamo i waypoints dal grafo
    try:
        waypoints = build_waypoints_from_graph(G, start_node_label)
    except ValueError as e:
        logger.error(str(e))
        return

    # Calcoliamo il percorso DCPP
    try:
        dcpp_route = calculate_dcpp_route(waypoints, G, logger)
        logger.info("DCPP Route calculated successfully.")
        logger.info("Final route (with orientations in radians):")
        for idx, wp in enumerate(dcpp_route, start=1):
            direction_str = orientation_rad_to_str(wp['orientation'])
            logger.info(f"Waypoint {idx}: {wp['label']} ({wp['x']}, {wp['y']}) Direction: {direction_str}")
    except ValueError as e:
        logger.error(str(e))
        logger.error("Failed to calculate DCPP route.")

if __name__ == '__main__':
    main()
