import math
import networkx as nx
import numpy as np
from sklearn.cluster import KMeans
import json
def load_full_graph(graph_path: str) -> nx.MultiGraph:
    """
    Carica un grafo non diretto (MultiGraph) da un file JSON con struttura:
      {
        "nodes": [{ "label":..., "x":..., "y":..., "orientation":... }, ...],
        "edges": [{ "source":..., "target":..., "distance":... }, ...]
      }
    e imposta subgraph[u][v]['weight'] = distance/0.31 (tempo di percorrenza).
    """
    with open(graph_path, 'r') as f:
        data = json.load(f)
    return load_full_graph_from_data(data)

def load_full_graph_from_data(graph_data):
    G = nx.MultiGraph()
    for node in graph_data['nodes']:
        label = node['label']
        x = node['x']
        y = node['y']
        orientation = node.get('orientation', 0.0)
        G.add_node(label, x=x, y=y, orientation=orientation)

    for edge in graph_data['edges']:
        u = edge['source']
        v = edge['target']
        distance = edge.get('distance', 1.0)
        weight = distance / 0.31
        G.add_edge(u, v, weight=weight)

    return G

def partition_graph(full_graph: nx.MultiGraph, num_partitions: int):
    """
    Partiziona il grafo in 'num_partitions' cluster (K-Means) e restituisce
    una lista di tuple (subgraph, starting_node).
    """
    node_labels = list(full_graph.nodes())
    positions = np.array([
        [full_graph.nodes[node]['x'], full_graph.nodes[node]['y']] for node in node_labels
    ])

    kmeans = KMeans(n_clusters=num_partitions, random_state=42)
    labels = kmeans.fit_predict(positions)

    clusters = {}
    for node_label, clab in zip(node_labels, labels):
        clusters.setdefault(clab, set()).add(node_label)

    subgraphs_with_start = []
    for cluster_idx, cluster_nodes in clusters.items():
        # Costruiamo il MultiGraph
        sg_nodes = set(cluster_nodes)
        edges = []
        for u, v, data in full_graph.edges(data=True):
            if u in sg_nodes or v in sg_nodes:
                edges.append((u, v, data))
                sg_nodes.update([u, v])

        sg = nx.MultiGraph()
        for n in sg_nodes:
            sg.add_node(n, **full_graph.nodes[n])
        for (u, v, d) in edges:
            sg.add_edge(u, v, **d)

        # Scegli un nodo iniziale (es. quello più vicino al centro di cluster)
        centroid = kmeans.cluster_centers_[cluster_idx]
        distances = {node: math.hypot(full_graph.nodes[node]['x'] - centroid[0],
                                     full_graph.nodes[node]['y'] - centroid[1]) for node in sg.nodes()}
        starting_node = min(distances, key=distances.get)

        subgraphs_with_start.append((sg, starting_node))

    return subgraphs_with_start

def partition_graph_wrapper(full_graph: nx.MultiGraph, num_slaves: int, start_positions: list = None):
    """
    Wrapper per partition_graph, ignoriamo start_positions
    e ritorniamo (subgraph, starting_node) per ogni slave.
    """
    return partition_graph(full_graph, num_slaves)
