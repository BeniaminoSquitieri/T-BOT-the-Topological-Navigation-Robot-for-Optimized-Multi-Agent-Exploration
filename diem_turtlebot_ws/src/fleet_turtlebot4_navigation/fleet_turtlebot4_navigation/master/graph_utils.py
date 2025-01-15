#!/usr/bin/env python3

import math
import networkx as nx
import numpy as np
from sklearn.cluster import KMeans
import json

def load_full_graph(graph_path: str) -> nx.MultiGraph:
    """
    Loads an undirected MultiGraph from a JSON file.

    The JSON file should have the following structure:
      {
        "nodes": [
          { "label": "node_1", "x": 1.0, "y": 2.0 },
          { "label": "node_2", "x": 3.0, "y": 4.0 },
          ...
        ],
        "edges": [
          { "source": "node_1", "target": "node_2", "distance": 5.0 },
          { "source": "node_2", "target": "node_3", "distance": 7.0 },
          ...
        ]
      }

    After loading, the function sets the 'weight' attribute for each edge
    as distance divided by 0.31, representing traversal time based on a fixed speed.

    Parameters:
    - graph_path (str): Path to the JSON file containing the graph data.

    Returns:
    - nx.MultiGraph: A NetworkX MultiGraph object representing the navigation graph.
    """
    # Open and read the JSON file containing the graph data
    with open(graph_path, 'r') as f:
        data = json.load(f)
    # Utilize the helper function to construct the graph from the loaded data
    return load_full_graph_from_data(data)


def load_full_graph_from_data(graph_data):
    """
    Constructs a NetworkX MultiGraph from a dictionary containing graph data.

    This function parses the provided graph data, adding nodes and edges to the
    NetworkX MultiGraph. Each edge's 'weight' attribute is calculated as distance
    divided by 0.31, which may represent traversal time given a constant speed.

    Parameters:
    - graph_data (dict): Dictionary containing 'nodes' and 'edges' lists.

    Returns:
    - nx.MultiGraph: A NetworkX MultiGraph object constructed from the input data.
    """
    # Initialize an empty MultiGraph
    G = nx.MultiGraph()

    # Iterate over each node in the 'nodes' list of the graph data
    for node in graph_data['nodes']:
        label = node['label']      # Unique identifier for the node
        x = node['x']              # X-coordinate in the navigation space
        y = node['y']              # Y-coordinate in the navigation space
        # Add the node to the graph with its attributes
        G.add_node(label, x=x, y=y)

    # Iterate over each edge in the 'edges' list of the graph data
    for edge in graph_data['edges']:
        u = edge['source']         # Source node label
        v = edge['target']         # Target node label
        distance = edge.get('distance', 1.0)  # Distance between nodes; default to 1.0 if not provided
        turtlebot4_max_velocity=0.31
        weight = distance / turtlebot4_max_velocity   # Calculate traversal time based on a fixed speed (0.31 units/sec)
        # Add the edge to the graph with the computed weight
        G.add_edge(u, v, weight=weight)

    return G  # Return the constructed MultiGraph

###
#This function is used if we want to use an other strategy to partition the graph divided by the slaves during the visit. We divide
#the graph in a number of subgraphs characterized by the number of slaves in the network and then we assign each cpp route on each 
#subgraph to every slave in order to maximize the frequency to which is trasversed by a robot. 
###


def partition_graph(full_graph: nx.MultiGraph, num_partitions: int):
    """
    Partitions the full navigation graph into a specified number of clusters using K-Means clustering.

    The partitioning is based on the spatial positions (x, y coordinates) of the nodes.
    After clustering, each partition is represented as a subgraph along with a designated
    starting node, typically the node closest to the cluster's centroid.

    Parameters:
    - full_graph (nx.MultiGraph): The complete navigation graph to be partitioned.
    - num_partitions (int): The number of clusters (partitions) to divide the graph into.

    Returns:
    - List[Tuple[nx.MultiGraph, str]]: A list where each element is a tuple containing:
        - A subgraph (nx.MultiGraph) representing a cluster.
        - A starting node label (str) for that subgraph.
    """
    # Extract all node labels from the full graph
    node_labels = list(full_graph.nodes())
    
    # Create a NumPy array of node positions for clustering
    positions = np.array([
        [full_graph.nodes[node]['x'], full_graph.nodes[node]['y']] for node in node_labels
    ])
    
    # Initialize K-Means clustering with the desired number of partitions
    kmeans = KMeans(n_clusters=num_partitions, random_state=42)
    # Perform clustering and obtain cluster labels for each node
    labels = kmeans.fit_predict(positions)
    
    # Organize nodes into clusters based on their assigned labels
    clusters = {}
    for node_label, clab in zip(node_labels, labels):
        clusters.setdefault(clab, set()).add(node_label)
    
    subgraphs_with_start = []  # List to store subgraphs and their starting nodes

    # Iterate over each cluster to construct subgraphs
    for cluster_idx, cluster_nodes in clusters.items():
        sg_nodes = set(cluster_nodes)  # Nodes belonging to the current cluster
        edges = []  # List to store edges within the cluster

        # Iterate over all edges in the full graph
        for u, v, data in full_graph.edges(data=True):
            # Check if either node of the edge belongs to the current cluster
            if u in sg_nodes or v in sg_nodes:
                edges.append((u, v, data))  # Add the edge to the cluster's edge list
                sg_nodes.update([u, v])     # Ensure both nodes are included in the cluster
        
        # Initialize a new MultiGraph for the current cluster
        sg = nx.MultiGraph()
        # Add all nodes in the cluster to the subgraph with their attributes
        for n in sg_nodes:
            sg.add_node(n, **full_graph.nodes[n])
        # Add all relevant edges to the subgraph with their attributes
        for (u, v, d) in edges:
            sg.add_edge(u, v, **d)
        
        # Determine the centroid of the current cluster from K-Means results
        centroid = kmeans.cluster_centers_[cluster_idx]
        # Calculate the Euclidean distance of each node in the subgraph to the centroid
        distances = {node: math.hypot(full_graph.nodes[node]['x'] - centroid[0],
                                     full_graph.nodes[node]['y'] - centroid[1]) for node in sg.nodes()}
        # Select the node closest to the centroid as the starting node for this subgraph
        starting_node = min(distances, key=distances.get)
        
        # Append the subgraph and its starting node to the result list
        subgraphs_with_start.append((sg, starting_node))
    
    return subgraphs_with_start  # Return the list of subgraphs with their starting nodes