import networkx as nx
import math

def calculate_dcpp_route(waypoints, subgraph, logger):
    """
    Calculate the DCPP (Directed Chinese Postman Problem) route for a given subgraph using a min cost flow approach.
    This method balances the imbalances between each node's in-degree and out-degree by introducing additional flows,
    which correspond to adding paths (edges) to make the graph Eulerian.

    Brief procedure:
    1. Check if the subgraph is strongly connected.
       If not, solving the DCPP easily is not possible.
    2. Compute imbalances for each node: out_degree(node) - in_degree(node).
       - Nodes with a positive imbalance have more outgoing edges than incoming.
       - Nodes with a negative imbalance have more incoming edges than outgoing.
    3. Create an auxiliary graph M for the min cost flow problem:
       - Add a super_source and a super_sink.
       - From the super_source, connect to nodes with a positive imbalance (capacity = imbalance, weight = 0).
       - From nodes with a negative imbalance, connect edges to the super_sink (capacity = -imbalance, weight = 0).
       - Copy the original edges with infinite capacity and weight equal to their cost (distance) to allow paths.
    4. Solve the min cost flow on M to find the cheapest way to compensate the imbalances.
    5. Use the obtained flow to add extra edges in G where needed (in practice, consider them as additional traversals on the same edges).
    6. Now G should be Eulerian. Check with nx.is_eulerian(G).
    7. Compute an eulerian_circuit on the resulting graph.
    8. Map the Eulerian circuit back onto the initial waypoints.

    Args:
        waypoints (list of dict): A list of waypoints, each with 'label', 'x', 'y', and 'orientation'.
        subgraph (nx.DiGraph): The subgraph on which to compute the DCPP route.
        logger (logging.Logger): Logger for debugging and process logging.

    Returns:
        list of dict: An ordered list of waypoints representing the DCPP route.
    """

    G = subgraph  # For readability

    # Check if the subgraph is strongly connected
    if not nx.is_strongly_connected(G):
        logger.error("The subgraph is not strongly connected, cannot solve DCPP.")
        raise ValueError("Subgraph not strongly connected.")

    # Compute imbalances: out_degree(node) - in_degree(node)
    imbalances = {v: G.out_degree(v) - G.in_degree(v) for v in G.nodes()}

    # Identify nodes with positive and negative imbalances
    positive_nodes = [v for v, imbalance in imbalances.items() if imbalance > 0]
    negative_nodes = [v for v, imbalance in imbalances.items() if imbalance < 0]

    # Create an auxiliary graph M for the min cost flow
    M = nx.DiGraph()

    # Copy the original nodes into M
    M.add_nodes_from(G.nodes(data=True))

    # Copy the original edges into M with capacity = inf and weight = original distance
    for (u, v, data) in G.edges(data=True):
        M.add_edge(u, v, capacity=float('inf'), weight=data.get('weight', 1.0))

    # Add super_source and super_sink
    M.add_node('super_source')
    M.add_node('super_sink')

    # Add edges from super_source to positive imbalance nodes
    for v in positive_nodes:
        M.add_edge('super_source', v, capacity=imbalances[v], weight=0)

    # Add edges from negative imbalance nodes to super_sink
    for v in negative_nodes:
        M.add_edge(v, 'super_sink', capacity=(-imbalances[v]), weight=0)

    # Solve the min cost flow on M
    try:
        flowDict = nx.min_cost_flow(M)
    except nx.NetworkXUnfeasible:
        logger.error("Min cost flow problem is infeasible, cannot make the subgraph Eulerian.")
        raise ValueError("Could not find a feasible min cost flow solution.")

    # Use the computed flow to add extra edges in G
    # Practically, if there's flow from u to v in flowDict, we need to "duplicate" that path
    # as many times as the units of flow.
    for u in G.nodes():
        for v, flow in flowDict.get(u, {}).items():
            if v not in G.nodes() or flow <= 0:
                continue
            # Add edges corresponding to the flow
            for _ in range(flow):
                if not G.has_edge(u, v):
                    # If the edge does not exist in G (a rare case), we create it
                    # with the estimated weight as the Euclidean distance
                    dist = math.hypot(G.nodes[v]['x'] - G.nodes[u]['x'],
                                      G.nodes[v]['y'] - G.nodes[u]['y'])
                    G.add_edge(u, v, weight=dist)
                else:
                    # The edge already exists; adding flow means considering multiple traversals.
                    # The Eulerian circuit will handle multiplicities implicitly.
                    pass

    # Now G should be Eulerian
    if not nx.is_eulerian(G):
        logger.error("Failed to make the subgraph Eulerian after min cost flow balancing.")
        raise ValueError("Subgraph not Eulerian after min cost flow balancing.")

    # Compute the eulerian_circuit on the balanced graph
    euler_circuit = list(nx.eulerian_circuit(G))

    # Map the eulerian_circuit onto the waypoints
    # The first waypoint in the route is the first of the provided waypoints.
    route_labels = [waypoints[0]['label']]
    for u, v in euler_circuit:
        route_labels.append(v)

    # Associate labels with actual waypoints
    label_to_wp = {wp['label']: wp for wp in waypoints}
    ordered_route = [label_to_wp[label] for label in route_labels if label in label_to_wp]

    return ordered_route


def orientation_str_to_rad(orientation_str):
    """
    Convert a cardinal direction string to radians.

    Args:
        orientation_str (str): Direction as a string ('NORTH', 'EAST', 'SOUTH', 'WEST').

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
    tolerance = 0.1  # Tolerance for floating-point approximations

    for angle, direction in orientation_map.items():
        if abs(orientation_radians - angle) < tolerance:
            return direction

    # If no exact match, return the closest direction
    closest_angle = min(orientation_map.keys(), key=lambda k: abs(k - orientation_radians))
    return orientation_map[closest_angle]
