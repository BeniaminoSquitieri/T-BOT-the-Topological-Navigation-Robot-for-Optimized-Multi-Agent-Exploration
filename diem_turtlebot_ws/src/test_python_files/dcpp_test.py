import json
import math
import networkx as nx
import argparse
import os
import random

def load_graph(graph_file_path):
    """
    Loads a graph from a JSON file and constructs a NetworkX DiGraph.

    Args:
        graph_file_path (str): Path to the JSON file containing the graph structure.

    Returns:
        networkx.DiGraph: A directed graph with weighted edges.
    """
    with open(graph_file_path, 'r') as f:
        data = json.load(f)

    # Create a directed graph using NetworkX
    G = nx.DiGraph()

    # Add nodes with coordinates as attributes
    for node in data['nodes']:
        G.add_node(node['label'], x=node['x'], y=node['y'])

    # Add edges with weights calculated based on Euclidean distance
    for edge in data['edges']:
        u, v = edge['from'], edge['to']
        x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        weight = math.hypot(x2 - x1, y2 - y1)
        G.add_edge(u, v, weight=weight)

    return G

def compute_dcpp_route(G):
    """
    Computes the Directed Chinese Postman Problem (DCPP) route for the provided directed graph G.

    Args:
        G (networkx.DiGraph): The directed graph for which to calculate the DCPP route.

    Returns:
        list: The DCPP route as a list of edges (u, v).
    """
    if not nx.is_strongly_connected(G):
        print("Error: The graph is not strongly connected.")
        return None

    # Balancing unbalanced nodes
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
        print("Error: Unable to balance the graph to make it Eulerian.")
        return None

def print_route(route, G, start_node_label):
    """
    Prints the nodes and directions traversed in the DCPP route.

    Args:
        route (list): List of node pairs (u, v) representing the DCPP route.
        G (networkx.DiGraph): Graph containing the nodes and their coordinates.
        start_node_label (str): Starting node for the route.
    """
    # Reorder the circuit to start from the specified starting node
    reordered_route = reorder_route(route, start_node_label)

    for u, v in reordered_route:
        print(f"From {u} to {v}")

def reorder_route(route, start_node_label):
    """
    Reorders the DCPP route to start from the specified starting node.

    Args:
        route (list): List of node pairs (u, v) representing the DCPP route.
        start_node_label (str): Desired starting node.

    Returns:
        list: Reordered route starting from the specified node.
    """
    for i, (u, v) in enumerate(route):
        if u == start_node_label:
            return route[i:] + route[:i]
    return route

def main():
    # Command line argument parsing for the start node and graph file path
    parser = argparse.ArgumentParser(description="DCPP Navigation on a Graph.")
    parser.add_argument('--start_node', type=str, required=True, help="Starting node (e.g., 'node_1')")
    parser.add_argument('--graph_path', type=str, required=True, help="Path to the JSON file containing the graph.")
    args = parser.parse_args()
    start_node_label = args.start_node
    graph_path = args.graph_path

    # Load the graph from the JSON file
    G = load_graph(graph_path)

    # Check if the start node is in the graph
    if start_node_label not in G.nodes():
        # Select a random node from the graph
        start_node_label = random.choice(list(G.nodes()))
        print(f"Warning: The specified start node was not found in the graph. Using random node '{start_node_label}' instead.")

    # Compute the DCPP route for the graph
    route = compute_dcpp_route(G)

    # If the route was computed, print it
    if route:
        print("\nDCPP Route (node and direction):")
        print_route(route, G, start_node_label)
    else:
        print("Unable to compute the DCPP route.")

if __name__ == "__main__":
    main()
