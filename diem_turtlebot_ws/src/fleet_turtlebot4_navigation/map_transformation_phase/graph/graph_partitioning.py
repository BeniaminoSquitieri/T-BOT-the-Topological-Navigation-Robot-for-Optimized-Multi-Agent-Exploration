# nav_pkg/map/map_transformation_phase/graph/graph_partitioning.py

import os
import json
import math
import networkx as nx
import numpy as np
from sklearn.cluster import KMeans

def load_full_graph(graph_path):
    """
    Loads the full graph from a JSON file and returns a NetworkX DiGraph (directed graph).
    """
    # Open the JSON file containing the graph data
    with open(graph_path, 'r') as f:
        data = json.load(f)

    # Create an empty directed NetworkX graph
    full_graph = nx.DiGraph()

    # Add nodes to the graph with their attributes
    for node in data['nodes']:
        label = node['label']  # Get the node label (e.g., "node_1")
        x = node['x']          # Get the x-coordinate of the node
        y = node['y']          # Get the y-coordinate of the node
        full_graph.add_node(label, x=x, y=y)  # Add the node with its attributes

    # Add edges to the graph, considering their direction and calculating weights
    for edge in data['edges']:
        u = edge['from']  # Source node of the edge
        v = edge['to']    # Target node of the edge
        # Calculate Euclidean distance between the nodes
        x1, y1 = full_graph.nodes[u]['x'], full_graph.nodes[u]['y']
        x2, y2 = full_graph.nodes[v]['x'], full_graph.nodes[v]['y']
        weight = math.hypot(x2 - x1, y2 - y1)
        # Add directed edge with weight
        full_graph.add_edge(u, v, weight=weight)
        # Optionally, you can store the direction if needed
        if 'direction' in edge:
            full_graph[u][v]['direction'] = edge['direction']

    return full_graph

def partition_graph(full_graph, num_partitions, start_positions=None, capacities=None):
    """
    Partitions the graph into subgraphs based on spatial clustering and robot capacities,
    considering the starting positions of the robots.

    Args:
        full_graph (nx.DiGraph): The original directed graph.
        num_partitions (int): Number of partitions/robots.
        start_positions (list of dict, optional): List of starting positions with 'x' and 'y'.
        capacities (list of int, optional): List of capacities for each robot.

    Returns:
        list of nx.DiGraph: List of subgraphs.
    """
    node_labels = list(full_graph.nodes())
    positions = np.array([[full_graph.nodes[node]['x'], full_graph.nodes[node]['y']] for node in node_labels])

    if start_positions and len(start_positions) == num_partitions:
        # Assign clusters based on proximity to starting positions
        centroids = np.array([[pos['x'], pos['y']] for pos in start_positions])
        kmeans = KMeans(n_clusters=num_partitions, init=centroids, n_init=1)
        labels = kmeans.fit_predict(positions)
    else:
        # Fallback to K-Means clustering
        kmeans = KMeans(n_clusters=num_partitions, random_state=42)
        labels = kmeans.fit_predict(positions)

    # Group nodes into clusters based on K-Means labels
    clusters = {}
    for node_label, cluster_label in zip(node_labels, labels):
        clusters.setdefault(cluster_label, set()).add(node_label)

    # Se le capacità non sono fornite, assumiamo capacità uguali
    if capacities and len(capacities) == num_partitions:
        cluster_capacities = capacities
    else:
        cluster_capacities = [len(clusters[i]) for i in range(num_partitions)]

    # Inizializza lista di sottografi
    subgraphs = []
    for cluster_idx, (cluster_label, cluster_nodes) in enumerate(clusters.items()):
        capacity = cluster_capacities[cluster_idx] if cluster_capacities else len(cluster_nodes)

        # Raccogli tutte le connessioni per il sottografo
        edges = []
        nodes = set(cluster_nodes)
        for u, v, data in full_graph.edges(data=True):
            if u in cluster_nodes or v in cluster_nodes:
                edges.append((u, v, data))
                nodes.update([u, v])

        # Crea il sottografo
        subgraph = nx.DiGraph()
        for node in nodes:
            subgraph.add_node(node, **full_graph.nodes[node])
        subgraph.add_edges_from([(u, v, data) for u, v, data in edges])

        # Duplica gli edge comuni
        subgraph = duplicate_common_edges(subgraph, full_graph, cluster_idx)

        # Assicura che il sottografo sia fortemente connesso
        if not nx.is_strongly_connected(subgraph):
            subgraph = make_subgraph_strongly_connected(subgraph, full_graph)

        # Rende il sottografo Euleriano se necessario
        subgraph = make_subgraph_eulerian(subgraph, full_graph)

        # Limita il numero di nodi in base alla capacità
        if capacity < len(subgraph.nodes()):
            # Implementa una logica per limitare i nodi, ad esempio rimuovendo quelli più lontani
            # Questo è un esempio semplice; potrebbe essere migliorato
            sorted_nodes = sorted(subgraph.nodes(data=True), key=lambda x: (x[1]['x'], x[1]['y']))
            limited_nodes = [node[0] for node in sorted_nodes[:capacity]]
            subgraph = subgraph.subgraph(limited_nodes).copy()

        subgraphs.append(subgraph)

    return subgraphs


def duplicate_common_edges(subgraph, full_graph, cluster_idx):
    """
    Duplicates edges that are common between subgraphs to ensure each subgraph
    can form a closed tour independently.
    """
    # Mark all edges in the subgraph as belonging to the current cluster
    for u, v in subgraph.edges():
        subgraph[u][v]['cluster'] = cluster_idx  # Add cluster identifier

    return subgraph

def make_subgraph_strongly_connected(subgraph, full_graph):
    """
    Adds edges from the full graph to the subgraph to make it strongly connected.
    """
    # Find strongly connected components
    sccs = list(nx.strongly_connected_components(subgraph))
    if len(sccs) == 1:
        return subgraph  # Already strongly connected

    # Build a simple cycle through the SCCs
    scc_list = [scc for scc in sccs]
    num_scc = len(scc_list)
    for i in range(num_scc):
        scc_from = scc_list[i]
        scc_to = scc_list[(i + 1) % num_scc]

        # Find nodes to connect scc_from to scc_to
        added_edge = False
        for u in scc_from:
            for v in scc_to:
                if full_graph.has_edge(u, v):
                    subgraph.add_edge(u, v, **full_graph.get_edge_data(u, v))
                    added_edge = True
                    break
            if added_edge:
                break
        if not added_edge:
            # If no direct edge exists, try to find a path
            try:
                path = nx.shortest_path(full_graph, source=list(scc_from)[0], target=list(scc_to)[0])
                for u, v in zip(path[:-1], path[1:]):
                    if not subgraph.has_node(u):
                        subgraph.add_node(u, **full_graph.nodes[u])
                    if not subgraph.has_node(v):
                        subgraph.add_node(v, **full_graph.nodes[v])
                    if not subgraph.has_edge(u, v):
                        subgraph.add_edge(u, v, **full_graph.get_edge_data(u, v))
            except nx.NetworkXNoPath:
                print("Unable to find path to make subgraph strongly connected.")
    return subgraph

def make_subgraph_eulerian(subgraph, full_graph):
    """
    Modifies the subgraph to make it Eulerian by balancing in-degree and out-degree.
    """
    # First, check if the subgraph is Eulerian
    if nx.is_eulerian(subgraph):
        return subgraph

    # Calculate the imbalance of in-degree and out-degree for each node
    imbalance = {}
    for node in subgraph.nodes():
        in_deg = subgraph.in_degree(node)
        out_deg = subgraph.out_degree(node)
        imbalance[node] = out_deg - in_deg

    # Nodes with positive imbalance need additional incoming edges
    # Nodes with negative imbalance need additional outgoing edges

    positive_imbalance_nodes = [node for node, imbal in imbalance.items() if imbal > 0]
    negative_imbalance_nodes = [node for node, imbal in imbalance.items() if imbal < 0]

    # Pair nodes to balance the degrees
    while positive_imbalance_nodes and negative_imbalance_nodes:
        u = positive_imbalance_nodes.pop()
        v = negative_imbalance_nodes.pop()

        # Add edge from v to u
        if full_graph.has_edge(v, u):
            subgraph.add_edge(v, u, **full_graph.get_edge_data(v, u))
        else:
            # If the edge doesn't exist in the full graph, find a path
            try:
                path = nx.shortest_path(full_graph, source=v, target=u)
                for s, t in zip(path[:-1], path[1:]):
                    if not subgraph.has_node(s):
                        subgraph.add_node(s, **full_graph.nodes[s])
                    if not subgraph.has_node(t):
                        subgraph.add_node(t, **full_graph.nodes[t])
                    if not subgraph.has_edge(s, t):
                        subgraph.add_edge(s, t, **full_graph.get_edge_data(s, t))
            except nx.NetworkXNoPath:
                print(f"Cannot find path from {v} to {u} to balance degrees.")

    return subgraph

def save_subgraphs(subgraphs, output_dir):
    """
    Saves each subgraph to a JSON file in the specified output directory.
    """
    subgraph_paths = []
    for idx, subgraph in enumerate(subgraphs):
        subgraph_data = {
            "nodes": [
                {"label": node, "x": data["x"], "y": data["y"], "orientation": data.get("orientation", 0.0)}
                for node, data in subgraph.nodes(data=True)
            ],
            "edges": []
        }

        # Add edges, including duplication info if applicable
        for u, v, data in subgraph.edges(data=True):
            edge_info = {
                "from": u,
                "to": v,
                "weight": data.get("weight", 1.0)
            }
            if 'duplex' in data:
                edge_info['duplex'] = data['duplex']
            if 'direction' in data:
                edge_info['direction'] = data['direction']
            if 'cluster' in data:
                edge_info['cluster'] = data['cluster']
            subgraph_data["edges"].append(edge_info)

        # Write the subgraph to a JSON file
        subgraph_file_path = os.path.join(output_dir, f"subgraph_{idx}.json")
        with open(subgraph_file_path, 'w') as f:
            json.dump(subgraph_data, f, indent=4)

        subgraph_paths.append(subgraph_file_path)  # Track saved file paths

    return subgraph_paths
