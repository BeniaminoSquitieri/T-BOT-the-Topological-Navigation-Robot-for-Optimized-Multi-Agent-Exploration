import networkx as nx


def calculate_undirected_cpp_route(waypoints, subgraph: nx.MultiGraph, logger):
    """
    Calculates a Closed Postman Problem (CPP) route on an undirected MultiGraph.

    The Chinese Postman Problem seeks the shortest possible route that traverses every edge of a graph
    at least once. For an undirected graph, if it is already Eulerian (all nodes have even degrees), 
    the Eulerian circuit is the optimal CPP route. If not, additional edges (duplicated paths) are 
    added to make all node degrees even, thereby enabling an Eulerian circuit.

    This function performs the following steps:
      1. Checks if the subgraph is a MultiGraph (required for handling multiple edges).
      2. Identifies nodes with odd degrees.
      3. If the graph is Eulerian, directly computes the Eulerian circuit.
      4. If not, computes a minimum-weight matching between odd-degree nodes to duplicate edges.
      5. Adds the matched edges to the subgraph, making it Eulerian.
      6. Computes the Eulerian circuit on the modified subgraph.
      7. Constructs an ordered list of node labels representing the CPP route.

    Parameters:
        waypoints (list of dict): 
            A list containing waypoint information, where each waypoint is a dictionary with keys 
            like 'label', 'x', 'y', etc. Example:
                [
                    {'label': 'node_1', 'x': 1.0, 'y': 2.0},
                    {'label': 'node_2', 'x': 3.0, 'y': 4.0},
                    ...
                ]
        subgraph (networkx.MultiGraph): 
            The subgraph assigned to the slave robot for route calculation. It represents the portion
            of the navigation graph the slave is responsible for.
        logger (rclpy.logging): 
            The logger instance from the master node used for logging informational and error messages.

    Returns:
        list: 
            An ordered list of node labels representing the route that covers all edges of the subgraph 
            at least once. Example:
                ['node_1', 'node_2', 'node_3', 'node_1']
    """
    # ---------------------------
    # Step 1: Validate Subgraph Type
    # ---------------------------
    # Ensure that the provided subgraph is indeed a MultiGraph, which allows multiple edges between nodes.
    # This is crucial for correctly handling scenarios where multiple paths exist between two nodes.
    if not isinstance(subgraph, nx.MultiGraph):
        logger.error(f"The graph is not a MultiGraph, but a {type(subgraph)}. Cannot compute DCPP.")
        return []

    # ---------------------------
    # Step 2: Identify Odd-Degree Nodes
    # ---------------------------
    # In graph theory, a graph has an Eulerian circuit if and only if it is connected and every vertex has an even degree.
    # Nodes with odd degrees disrupt this property, so we need to pair them up to make their degrees even.
    odd_degree_nodes = [node for node, deg in subgraph.degree() if deg % 2 != 0]
    # logger.info(f"Found {len(odd_degree_nodes)} odd-degree nodes: {odd_degree_nodes}")

    # ---------------------------
    # Step 3: Check if Graph is Already Eulerian
    # ---------------------------
    # If there are no odd-degree nodes, the graph is already Eulerian, and we can directly compute the Eulerian circuit.
    if len(odd_degree_nodes) == 0:
        try:
            # Compute the Eulerian circuit using NetworkX's built-in function.
            # This function returns an iterator of edge tuples (u, v).
            euler_circuit = list(nx.eulerian_circuit(subgraph))
            # logger.info("The graph is already Eulerian. Computed Eulerian circuit.")
        except nx.NetworkXError as e:
            # Handle the error if the Eulerian circuit cannot be computed for some reason.
            logger.error(f"Error computing Eulerian circuit: {e}")
            return []
    else:
        # ---------------------------
        # Step 4: Make Graph Eulerian by Adding Matching Edges
        # ---------------------------
        # Since the graph is not Eulerian, we need to make it Eulerian by pairing up the odd-degree nodes
        # and adding the shortest possible paths between them. This ensures that all nodes have even degrees.

        # Create a complete graph of the odd-degree nodes where the edge weights represent the shortest path distances.
        odd_complete = nx.Graph()

        # Iterate over all unique pairs of odd-degree nodes to calculate the shortest path distances between them.
        for i, u in enumerate(odd_degree_nodes):
            for v in odd_degree_nodes[i + 1:]:
                try:
                    # Calculate the length of the shortest path between nodes u and v, using edge weights.
                    dist = nx.shortest_path_length(subgraph, source=u, target=v, weight='weight')
                    # Add an edge between u and v in the complete graph with the distance as the weight.
                    odd_complete.add_edge(u, v, weight=dist)
                except nx.NetworkXNoPath:
                    # If there is no path between u and v, log an error and skip this pair.
                    logger.error(f"No path between {u} and {v} in the subgraph.")
                    continue

        # Check if the complete graph has any edges; if not, it's impossible to perform matching.
        if odd_complete.number_of_edges() == 0:
            logger.error("Cannot compute matching between odd-degree nodes; no edges available.")
            return []

        # Compute the minimum-weight matching on the complete graph of odd-degree nodes.
        # This finds the pairing of nodes that minimizes the total added distance.
        matching = nx.min_weight_matching(odd_complete, weight='weight')
        # logger.info(f"Found minimum weight matching between odd-degree nodes: {matching}")

        # ---------------------------
        # Step 5: Duplicate Edges in Subgraph Based on Matching
        # ---------------------------
        # For each pair in the matching, add the corresponding edges to the subgraph to make all node degrees even.
        for (u, v) in matching:
            if subgraph.has_edge(u, v):
                # If an edge already exists between u and v, retrieve its original weight.
                # Since it's a MultiGraph, there might be multiple edges; take the first one.
                original_weight = list(subgraph.get_edge_data(u, v).values())[0]['weight']
            else:
                # If no direct edge exists, calculate the shortest path distance between u and v.
                original_weight = nx.shortest_path_length(subgraph, source=u, target=v, weight='weight')
            # Add an edge between u and v with the original weight.
            # In a MultiGraph, this allows multiple edges between the same pair of nodes.
            subgraph.add_edge(u, v, weight=original_weight)

        # After adding the necessary edges, the graph should now be Eulerian.
        try:
            # Compute the Eulerian circuit on the modified subgraph.
            euler_circuit = list(nx.eulerian_circuit(subgraph))
            # logger.info("Eulerian path computed after adding matching edges.")
        except nx.NetworkXError as e:
            # Handle any errors that occur during the computation of the Eulerian circuit.
            logger.error(f"Error computing Eulerian circuit after adding edges: {e}")
            return []

    # ---------------------------
    # Step 6: Construct the Ordered Route from the Eulerian Circuit
    # ---------------------------
    # Convert the sequence of edges in the Eulerian circuit into an ordered list of node labels.
    ordered_route = []
    for u, v in euler_circuit:
        if not ordered_route:
            # If the ordered_route list is empty, append the first node of the circuit.
            ordered_route.append(u)
        # Append the target node of the current edge to the route.
        ordered_route.append(v)

    # ---------------------------
    # Step 7: Return the Final CPP Route
    # ---------------------------
    # The ordered_route now represents a path that traverses every edge at least once.
    # This is the solution to the Chinese Postman Problem for the given subgraph.
    return ordered_route
