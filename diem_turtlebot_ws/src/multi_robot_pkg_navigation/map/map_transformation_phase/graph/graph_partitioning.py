import os
import json
import math
import networkx as nx
import numpy as np
from sklearn.cluster import KMeans

def load_full_graph(graph_path):
    """
    Loads the full graph from a JSON file and returns a NetworkX graph.
    """
    # Open the JSON file containing the graph data
    with open(graph_path, 'r') as f:
        data = json.load(f)

    # Create an empty undirected NetworkX graph
    full_graph = nx.Graph()

    # Add nodes to the graph with their attributes
    for node in data['nodes']:
        label = node['label']  # Get the node label (e.g., "node_1")
        x = node['x']  # Get the x-coordinate of the node
        y = node['y']  # Get the y-coordinate of the node
        full_graph.add_node(label, x=x, y=y)  # Add the node with its attributes

    # Add edges to the graph, calculating their weights (distances)
    for edge in data['edges']:
        u = edge['from']  # Source node of the edge
        v = edge['to']  # Target node of the edge
        # Calculate Euclidean distance between the nodes
        x1, y1 = full_graph.nodes[u]['x'], full_graph.nodes[u]['y']
        x2, y2 = full_graph.nodes[v]['x'], full_graph.nodes[v]['y']
        weight = math.hypot(x2 - x1, y2 - y1)
        full_graph.add_edge(u, v, weight=weight)  # Add edge with weight

    return full_graph

def partition_graph(full_graph, num_partitions):
    """
    Partitions the graph into connected subgraphs based on spatial clustering,
    including all edges connected to cluster nodes, and duplicates common edges.
    """
    # Extract node positions as a list of coordinates
    node_labels = list(full_graph.nodes())
    positions = np.array([[full_graph.nodes[node]['x'], full_graph.nodes[node]['y']] for node in node_labels])

    # Perform K-Means clustering on the node positions
    kmeans = KMeans(n_clusters=num_partitions, random_state=42)
    labels = kmeans.fit_predict(positions)  # Cluster assignments for each node

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
        subgraph = nx.Graph()
        for node in nodes:
            subgraph.add_node(node, **full_graph.nodes[node])
        subgraph.add_edges_from([(u, v, data) for u, v, data in edges])

        # Duplicate edges that are shared between subgraphs
        subgraph = duplicate_common_edges(subgraph, full_graph, cluster_idx)

        # Ensure the subgraph is fully connected
        if not nx.is_connected(subgraph):
            subgraph = connect_subgraph(subgraph, full_graph)

        # Adjust the subgraph to make it Eulerian (for closed tours)
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

def connect_subgraph(subgraph, full_graph):
    """
    Connects disconnected components in the subgraph using edges from the full graph.
    """
    # Identify connected components of the subgraph
    components = list(nx.connected_components(subgraph))
    while len(components) > 1:  # Repeat until only one component remains
        min_distance = float('inf')  # Track the shortest path length
        pair_to_connect = None  # Track the nodes to connect

        # Find the shortest path between any two disconnected components
        for comp_a in components:
            for comp_b in components:
                if comp_a != comp_b:  # Avoid self-comparison
                    for node_a in comp_a:
                        for node_b in comp_b:
                            try:
                                # Calculate shortest path length in the full graph
                                path_length = nx.shortest_path_length(full_graph, source=node_a, target=node_b, weight='weight')
                                if path_length < min_distance:
                                    min_distance = path_length
                                    pair_to_connect = (node_a, node_b)  # Store the pair
                            except nx.NetworkXNoPath:
                                continue  # Ignore if no path exists

        # If a valid pair is found, connect the components
        if pair_to_connect:
            path = nx.shortest_path(full_graph, source=pair_to_connect[0], target=pair_to_connect[1], weight='weight')
            edges_in_path = list(zip(path[:-1], path[1:]))
            for u, v in edges_in_path:
                if not subgraph.has_node(u):  # Add missing nodes
                    subgraph.add_node(u, **full_graph.nodes[u])
                if not subgraph.has_node(v):
                    subgraph.add_node(v, **full_graph.nodes[v])
                if not subgraph.has_edge(u, v):  # Add missing edges
                    subgraph.add_edge(u, v, **full_graph.get_edge_data(u, v))
                    subgraph[u][v]['cluster'] = subgraph.graph.get('cluster_idx', -1)
        else:
            print("Unable to connect components in subgraph.")
            break

        components = list(nx.connected_components(subgraph))  # Recalculate components

    return subgraph

def make_subgraph_eulerian(subgraph, full_graph):
    """
    Modifies the subgraph to make it Eulerian by adding duplicate edges.
    """
    if nx.is_eulerian(subgraph):
        return subgraph  # Return if already Eulerian

    # Find nodes with odd degree
    odd_degree_nodes = [node for node in subgraph.nodes() if subgraph.degree(node) % 2 == 1]

    # Pair nodes with odd degree and connect them
    while len(odd_degree_nodes) >= 2:
        node_u = odd_degree_nodes.pop()
        node_v = odd_degree_nodes.pop()

        # Find the shortest path between the pair
        try:
            path = nx.shortest_path(subgraph, source=node_u, target=node_v, weight='weight')
        except nx.NetworkXNoPath:
            path = nx.shortest_path(full_graph, source=node_u, target=node_v, weight='weight')  # Use full graph if needed
            for u, v in zip(path[:-1], path[1:]):
                if not subgraph.has_node(u):
                    subgraph.add_node(u, **full_graph.nodes[u])
                if not subgraph.has_node(v):
                    subgraph.add_node(v, **full_graph.nodes[v])
                if not subgraph.has_edge(u, v):
                    subgraph.add_edge(u, v, **full_graph.get_edge_data(u, v))
                    subgraph[u][v]['cluster'] = subgraph.graph.get('cluster_idx', -1)

        # Add duplicate edges for Eulerian property
        edges_in_path = list(zip(path[:-1], path[1:]))
        for u, v in edges_in_path:
            if subgraph.has_edge(u, v):
                if 'duplex' in subgraph[u][v]:
                    subgraph[u][v]['duplex'] += 1  # Increment duplication count
                else:
                    subgraph[u][v]['duplex'] = 2  # Initialize duplication count
            else:
                subgraph.add_edge(u, v, **full_graph.get_edge_data(u, v))
                subgraph[u][v]['duplex'] = 1  # Mark as duplicated
                subgraph[u][v]['cluster'] = subgraph.graph.get('cluster_idx', -1)

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
            subgraph_data["edges"].append(edge_info)

        # Write the subgraph to a JSON file
        subgraph_file_path = os.path.join(output_dir, f"subgraph_{idx}.json")
        with open(subgraph_file_path, 'w') as f:
            json.dump(subgraph_data, f, indent=4)

        subgraph_paths.append(subgraph_file_path)  # Track saved file paths

    return subgraph_paths
