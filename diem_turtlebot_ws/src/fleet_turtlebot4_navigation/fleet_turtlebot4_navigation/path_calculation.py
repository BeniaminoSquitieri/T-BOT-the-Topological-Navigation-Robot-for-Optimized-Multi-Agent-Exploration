# path_calculation.py

import networkx as nx
import math
import json

def calculate_dcpp_route(waypoints, subgraph, logger):
    """
    Calcola il percorso DCPP (circuito Euleriano) per una lista di waypoint.

    Args:
        waypoints (list of dict): Lista di waypoint.
        subgraph (nx.DiGraph): Sottografo assegnato allo slave.
        logger (rclpy.logging.Logger): Logger ROS 2 per logging.

    Returns:
        list of dict: Lista ordinata di waypoint che costituiscono il percorso DCPP.
    """
    # Crea un grafo NetworkX a partire dai waypoint
    G = nx.DiGraph()
    for wp in waypoints:
        G.add_node(wp['label'], x=wp['x'], y=wp['y'], orientation=wp['orientation'])

    # Aggiungi gli archi dal sottografo
    for u, v, data in subgraph.edges(data=True):
        weight = data.get('weight', 1.0)
        G.add_edge(u, v, weight=weight)

    # Calcola il circuito Euleriano
    try:
        # Try to find an Eulerian circuit starting from the first waypoint
        euler_circuit = list(nx.eulerian_circuit(G, source=waypoints[0]['label']))
        route_labels = [waypoints[0]['label']]
        for u, v in euler_circuit:
            route_labels.append(v)
    except nx.NetworkXError as e:
        logger.error(f"Errore nel calcolo del circuito Euleriano: {e}")
        # Fallback: sequenza lineare dei waypoint
        route_labels = [wp['label'] for wp in waypoints]

    # Mappa i label dei nodi al dizionario dei waypoint
    label_to_wp = {wp['label']: wp for wp in waypoints}
    ordered_route = [label_to_wp[label] for label in route_labels if label in label_to_wp]

    return ordered_route

def orientation_conversion(orientation_radians):
    """
    Converte un angolo in radianti nella stringa di orientamento corrispondente.

    Args:
        orientation_radians (float): Angolo in radianti.

    Returns:
        str: Orientamento come stringa ('NORTH', 'EAST', 'SOUTH', 'WEST').
    """
    orientation_map = {
        0.0: 'NORTH',
        -math.pi / 2: 'EAST',
        math.pi: 'SOUTH',
        math.pi / 2: 'WEST'
    }

    # Tolleranza per la corrispondenza degli angoli
    tolerance = 0.1  # Radianti (~5.7 gradi)

    for angle, direction in orientation_map.items():
        if abs(orientation_radians - angle) < tolerance:
            return direction

    # Se nessuna corrispondenza esatta, assegnare la direzione più vicina
    closest_angle = min(orientation_map.keys(), key=lambda k: abs(k - orientation_radians))
    return orientation_map[closest_angle]
