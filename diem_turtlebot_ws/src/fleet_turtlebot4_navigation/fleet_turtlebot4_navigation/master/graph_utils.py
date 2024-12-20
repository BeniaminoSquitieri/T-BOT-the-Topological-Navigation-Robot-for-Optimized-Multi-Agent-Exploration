# graph_utils.py

import json
import networkx as nx
from .graph_partitioning import partition_graph  # Local import to prevent circular dependencies

def load_full_graph(graph_path: str) -> nx.DiGraph:
    """
    Load the navigation graph from a JSON file.

    Args:
        graph_path (str): Path to the navigation graph JSON file.

    Returns:
        nx.DiGraph: The loaded directed graph.
    """
    with open(graph_path, 'r') as file:
        graph_data = json.load(file)
    return load_full_graph_from_data(graph_data)

def load_full_graph_from_data(graph_data: dict) -> nx.DiGraph:
    """
    Load a directed graph from a dictionary containing nodes and edges.

    Args:
        graph_data (dict): Dictionary containing 'nodes' and 'edges'.

    Returns:
        nx.DiGraph: The loaded directed graph.
    """
    G = nx.DiGraph()
    for node in graph_data['nodes']:
        label = node['label']
        x = node['x']
        y = node['y']
        orientation = node.get('orientation', 0.0)
        G.add_node(label, x=x, y=y, orientation=orientation)
    for edge in graph_data['edges']:
        u = edge['from']
        v = edge['to']
        weight = edge.get('weight', 1.0)
        G.add_edge(u, v, weight=weight)
    return G

def partition_graph_wrapper(full_graph: nx.DiGraph, num_slaves: int, start_positions: list) -> list:
    """
    Wrapper function for partitioning the graph.

    Args:
        full_graph (nx.DiGraph): The complete navigation graph.
        num_slaves (int): Number of slaves to partition the graph for.
        start_positions (list): List of starting positions for each slave.

    Returns:
        list: List of subgraphs corresponding to each slave.
    """
    return partition_graph(full_graph, num_slaves, start_positions=start_positions)
