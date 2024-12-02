# fleet_turtlebot4_navigation/path_calculation.py

import networkx as nx
import math
import logging

def calculate_dcpp_route(waypoints, subgraph, logger):
    """
    Calculates the Directed Chinese Postman Problem (DCPP) route for the given subgraph.

    Args:
        waypoints (list of dict): List of waypoints with 'label', 'x', 'y', and 'orientation'.
        subgraph (nx.DiGraph): The subgraph for which to calculate the DCPP route.
        logger (logging.Logger): Logger for logging information.

    Returns:
        list of dict: Ordered list of waypoints representing the DCPP route.
    """
    # Calculate imbalances
    imbalance = {}
    for node in subgraph.nodes():
        imbalance[node] = subgraph.out_degree(node) - subgraph.in_degree(node)

    # Nodes with positive imbalance need additional incoming edges
    # Nodes with negative imbalance need additional outgoing edges
    pos_imbalance = [node for node, imbal in imbalance.items() if imbal > 0]
    neg_imbalance = [node for node, imbal in imbalance.items() if imbal < 0]

    # Calculate the number of additional edges needed
    additional_edges = {}
    for u, v in zip(neg_imbalance, pos_imbalance):
        additional_edges.setdefault(u, []).append(v)

    # Add the additional edges to make the graph Eulerian
    for u, vs in additional_edges.items():
        for v in vs:
            if subgraph.has_edge(u, v):
                subgraph.add_edge(u, v, weight=subgraph[u][v]['weight'])
            else:
                # If edge doesn't exist, find the shortest path and add edges
                try:
                    path = nx.shortest_path(subgraph, source=u, target=v, weight='weight')
                    for i in range(len(path) - 1):
                        s, t = path[i], path[i + 1]
                        subgraph.add_edge(s, t, weight=subgraph[s][t]['weight'])
                except nx.NetworkXNoPath:
                    logger.error(f"No path found to add edge from {u} to {v} for DCPP.")
                    raise

    # Now, the graph should be Eulerian
    if not nx.is_eulerian(subgraph):
        logger.error("Failed to make the subgraph Eulerian for DCPP.")
        raise ValueError("Subgraph is not Eulerian after adding additional edges.")

    # Find the Eulerian circuit
    euler_circuit = list(nx.eulerian_circuit(subgraph))

    # Map the circuit to waypoints
    route_labels = [waypoints[0]['label']]
    for u, v in euler_circuit:
        route_labels.append(v)

    # Convert labels back to waypoints
    label_to_wp = {wp['label']: wp for wp in waypoints}
    ordered_route = [label_to_wp[label] for label in route_labels if label in label_to_wp]

    # Log the calculated route
    logger.info("Calculated DCPP route:")
    for wp in ordered_route:
        logger.info(f" - {wp['label']} at ({wp['x']}, {wp['y']}) with orientation {wp['orientation']} radians")
    
    # **Aggiunta della stampa del percorso DCPP**
    print("\n===== Calculated DCPP Route =====")
    for idx, wp in enumerate(ordered_route, start=1):
        print(f"Waypoint {idx}: {wp['label']} at ({wp['x']}, {wp['y']}), Orientation: {wp['orientation']} radians")
    print("===== End of DCPP Route =====\n")

    return ordered_route

def orientation_str_to_rad(orientation_str):
    """
    Converte un orientamento in formato stringa in radianti.

    Args:
        orientation_str (str): Orientamento come stringa ('NORTH', 'EAST', 'SOUTH', 'WEST').

    Returns:
        float: Orientamento in radianti.
    """
    orientations = {
        'NORTH': 0.0,
        'EAST': math.pi / 2,
        'SOUTH': math.pi,
        'WEST': 3 * math.pi / 2
    }
    orientation_rad = orientations.get(orientation_str.upper(), 0.0)
    return orientation_rad

def orientation_rad_to_str(orientation_radians):
    """
    Converts an angle in radians to the corresponding cardinal direction string.

    Args:
        orientation_radians (float): Angle in radians.

    Returns:
        str: Corresponding direction ('NORTH', 'EAST', 'SOUTH', 'WEST').
    """
    orientation_map = {
        0.0: 'NORTH',
        math.pi / 2: 'EAST',
        math.pi: 'SOUTH',
        3 * math.pi / 2: 'WEST'
    }

    # Tolerance for matching angles
    tolerance = 0.1  # Radians (~5.7 degrees)

    for angle, direction in orientation_map.items():
        if abs(orientation_radians - angle) < tolerance:
            return direction

    # If no exact match, return the closest direction
    closest_angle = min(orientation_map.keys(), key=lambda k: abs(k - orientation_radians))
    return orientation_map[closest_angle]
