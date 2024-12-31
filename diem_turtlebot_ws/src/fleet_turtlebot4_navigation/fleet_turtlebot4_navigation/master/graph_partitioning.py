import os
import json
import math
import networkx as nx
import numpy as np
from sklearn.cluster import KMeans


#MODULO VECCHIO NON PIU USATO
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
        x = node['x']  # Get the x-coordinate of the node
        y = node['y']  # Get the y-coordinate of the node
        full_graph.add_node(label, x=x, y=y)  # Add the node with its attributes

    # Add edges to the graph, considering their direction and calculating weights
    for edge in data['edges']:
        u = edge['from']  # Source node of the edge
        v = edge['to']  # Target node of the edge
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

def partition_graph(full_graph, num_partitions, start_positions=None):
    """
    Partitions the graph into subgraphs based on spatial clustering,
    including all edges connected to cluster nodes, and duplicates common edges.
    After partitioning, ensures each subgraph is strongly connected by adding necessary edges.
    """
    # Extract node positions as a list of coordinates
    node_labels = list(full_graph.nodes())
    positions = np.array([[full_graph.nodes[node]['x'], full_graph.nodes[node]['y']] for node in node_labels])

    if start_positions and len(start_positions) == num_partitions:
        # Use initial positions as centroids for K-Means
        centroids = np.array([[pos['x'], pos['y']] for pos in start_positions])
        kmeans = KMeans(n_clusters=num_partitions, init=centroids, n_init=1)
        labels = kmeans.fit_predict(positions)
    else:
        # Perform standard K-Means clustering on the node positions
        kmeans = KMeans(n_clusters=num_partitions, random_state=42)
        labels = kmeans.fit_predict(positions)
    # Group nodes into clusters based on K-Means labels
    clusters = {}
    for node_label, cluster_label in zip(node_labels, labels):
        clusters.setdefault(cluster_label, set()).add(node_label)

    # Initialize list to store subgraphs
    subgraphs = []
    for cluster_idx, cluster_nodes in clusters.items():
        # Collect all edges connected to nodes in the cluster
        edges = []
        nodes = set(cluster_nodes)  # Nodes in the current cluster
        for u, v, data in full_graph.edges(data=True):
            if u in cluster_nodes or v in cluster_nodes:  # Include edges connected to the cluster
                edges.append((u, v, data))
                nodes.update([u, v])  # Ensure both nodes of the edge are included

        # Create a subgraph from the collected nodes and edges
        subgraph = nx.DiGraph()
        for node in nodes:
            subgraph.add_node(node, **full_graph.nodes[node])
        subgraph.add_edges_from([(u, v, data) for u, v, data in edges])

        # Duplicate edges that are shared between subgraphs
        subgraph = duplicate_common_edges(subgraph, full_graph, cluster_idx)

        # Ensure the subgraph is strongly connected
        if not nx.is_strongly_connected(subgraph):
            subgraph = make_subgraph_strongly_connected(subgraph, full_graph)

        # Adjust the subgraph to make it Eulerian (for closed tours), if needed
        # Note: In directed graphs, an Eulerian circuit requires the graph to be strongly connected and balanced
        subgraph = make_subgraph_eulerian(subgraph, full_graph)

        # Add the subgraph to the list
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
            # If the edge doesn't exist in the full graph, we can create it or find a path
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
                {"label": node, "x": data["x"], "y": data["y"]}
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
            subgraph_data["edges"].append(edge_info)

        # Write the subgraph to a JSON file
        subgraph_file_path = os.path.join(output_dir, f"subgraph_{idx}.json")
        with open(subgraph_file_path, 'w') as f:
            json.dump(subgraph_data, f, indent=4)

        subgraph_paths.append(subgraph_file_path)  # Track saved file paths

    return subgraph_paths



def load_full_graph_from_data(graph_data):
    """
    Carica un grafo NetworkX da un dizionario contenente nodi e archi.

    Args:
        graph_data (dict): Dizionario con 'nodes' e 'edges'.

    Returns:
        nx.DiGraph: Il grafo diretto caricato.
    """
    G = nx.DiGraph()

    for node in graph_data['nodes']:
        label = node['label']
        x = node['x']
        y = node['y']
        G.add_node(label, x=x, y=y)

    for edge in graph_data['edges']:
        u = edge['from']
        v = edge['to']
        weight = edge.get('weight', 1.0)
        G.add_edge(u, v, weight=weight)