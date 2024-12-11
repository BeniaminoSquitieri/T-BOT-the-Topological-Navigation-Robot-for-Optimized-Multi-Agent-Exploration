import networkx as nx
import math

def calculate_dcpp_route(waypoints, subgraph, logger):
    """
    Calculate the Directed Chinese Postman Problem (DCPP) route for a given subgraph.

    Args:
        waypoints (list of dict): Waypoints with 'label', 'x', 'y', and 'orientation'.
        subgraph (nx.DiGraph): Subgraph for which the DCPP route is calculated.
        logger (logging.Logger): Logger for debugging and logging progress.

    Returns:
        list of dict: Ordered waypoints representing the DCPP route.
    """
    # Calculate imbalances in in-degree and out-degree for each node
    imbalance = {node: subgraph.out_degree(node) - subgraph.in_degree(node) for node in subgraph.nodes()}

    # Separate nodes into positive (excess outgoing edges) and negative (insufficient incoming edges)
    pos_imbalance = [node for node, imbal in imbalance.items() if imbal > 0]
    neg_imbalance = [node for node, imbal in imbalance.items() if imbal < 0]

    # Prepare to add additional edges to balance the graph
    additional_edges = {}
    for u, v in zip(neg_imbalance, pos_imbalance):
        additional_edges.setdefault(u, []).append(v)

    # Add required edges to balance the graph and make it Eulerian
    for u, vs in additional_edges.items():
        for v in vs:
            if subgraph.has_edge(u, v):
                # Edge already exists; increase its weight
                subgraph.add_edge(u, v, weight=subgraph[u][v]['weight'])
            else:
                # Edge doesn't exist; find the shortest path and add edges
                try:
                    path = nx.shortest_path(subgraph, source=u, target=v, weight='weight')
                    for i in range(len(path) - 1):
                        s, t = path[i], path[i + 1]
                        subgraph.add_edge(s, t, weight=subgraph[s][t]['weight'])
                except nx.NetworkXNoPath:
                    # Log error if no path is found
                    logger.error(f"No path found to add edge from {u} to {v} for DCPP.")
                    raise

    # Ensure the graph is Eulerian before proceeding
    if not nx.is_eulerian(subgraph):
        logger.error("Failed to make the subgraph Eulerian for DCPP.")
        raise ValueError("Subgraph is not Eulerian after adding additional edges.")

    # Calculate Eulerian circuit
    euler_circuit = list(nx.eulerian_circuit(subgraph))

    # Map Eulerian circuit to waypoints
    route_labels = [waypoints[0]['label']]
    for u, v in euler_circuit:
        route_labels.append(v)

    # Match route labels with actual waypoint details
    label_to_wp = {wp['label']: wp for wp in waypoints}
    ordered_route = [label_to_wp[label] for label in route_labels if label in label_to_wp]

    # Log the calculated route
    logger.info("Calculated DCPP route:")
    for idx, wp in enumerate(ordered_route, start=1):
        logger.info(f" - Waypoint {idx}: {wp['label']} at ({wp['x']}, {wp['y']}) Orientation: {wp['orientation']} radians")

    return ordered_route

def orientation_str_to_rad(orientation_str):
    """
    Convert a cardinal direction string to radians.

    Args:
        orientation_str (str): Direction as string ('NORTH', 'EAST', 'SOUTH', 'WEST').

    Returns:
        float: Corresponding angle in radians.
    """
    orientations = {
        'NORTH': 0.0,
        'EAST': math.pi / 2,
        'SOUTH': math.pi,
        'WEST': 3 * math.pi / 2
    }
    return orientations.get(orientation_str.upper(), 0.0)

def orientation_rad_to_str(orientation_radians):
    """
    Convert an angle in radians to a cardinal direction string.

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
    tolerance = 0.1  # Allow for small differences due to floating-point precision

    for angle, direction in orientation_map.items():
        if abs(orientation_radians - angle) < tolerance:
            return direction

    # If no exact match, return the closest direction
    closest_angle = min(orientation_map.keys(), key=lambda k: abs(k - orientation_radians))
    return orientation_map[closest_angle]
