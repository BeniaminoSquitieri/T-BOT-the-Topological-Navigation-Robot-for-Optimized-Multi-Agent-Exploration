# fleet_turtlebot4_navigation/graph_partitioning.py

import os
import json
import math
import networkx as nx
import numpy as np
from sklearn.cluster import KMeans

def load_full_graph(graph_path):
    """
    Carica il grafo completo da un file JSON, calcola la distanza euclidea per ogni arco
    e imposta il peso dell'arco in base a questa distanza. Restituisce un NetworkX DiGraph (grafo diretto).
    
    Args:
        graph_path (str): Percorso al file JSON contenente il grafo.
    
    Returns:
        nx.DiGraph: Il grafo diretto caricato con pesi aggiornati.
    """
    with open(graph_path, 'r') as f:
        data = json.load(f)

    full_graph = nx.DiGraph()

    # Aggiungi nodi al grafo
    for node in data['nodes']:
        label = node['label']
        x = node['x']
        y = node['y']
        orientation = node.get('orientation', 0.0)
        full_graph.add_node(label, x=x, y=y, orientation=orientation)

    # Aggiungi archi al grafo con pesi basati sulla distanza euclidea
    for edge in data['edges']:
        u = edge['from']
        v = edge['to']
        # Calcola la distanza euclidea tra i nodi u e v
        if full_graph.has_node(u) and full_graph.has_node(v):
            x1, y1 = full_graph.nodes[u]['x'], full_graph.nodes[u]['y']
            x2, y2 = full_graph.nodes[v]['x'], full_graph.nodes[v]['y']
            distance = math.hypot(x2 - x1, y2 - y1)
        else:
            # Se uno dei nodi non esiste, imposta una distanza predefinita
            distance = edge.get('weight', 1.0)
        
        full_graph.add_edge(u, v, weight=distance)

    return full_graph

def partition_graph(full_graph, num_partitions, start_positions=None, capacities=None):
    """
    Partiziona il grafo in sottografi connessi basati su K-Means clustering,
    utilizzando le posizioni iniziali dei robot come centroidi se fornite.

    Args:
        full_graph (nx.DiGraph): Il grafo completo.
        num_partitions (int): Numero di partizioni/robot.
        start_positions (list of dict, optional): Lista di posizioni iniziali con 'x' e 'y'.
        capacities (list of int, optional): Lista di capacità per ciascun robot.

    Returns:
        list of nx.DiGraph: Lista di sottografi.
    """
    node_labels = list(full_graph.nodes())
    positions = np.array([[full_graph.nodes[node]['x'], full_graph.nodes[node]['y']] for node in node_labels])

    if start_positions and len(start_positions) == num_partitions:
        # Utilizza le posizioni iniziali come centroidi per K-Means
        centroids = np.array([[pos['x'], pos['y']] for pos in start_positions])
        kmeans = KMeans(n_clusters=num_partitions, init=centroids, n_init=1)
        labels = kmeans.fit_predict(positions)
    else:
        # Fallback a K-Means standard
        kmeans = KMeans(n_clusters=num_partitions, random_state=42)
        labels = kmeans.fit_predict(positions)

    # Raggruppa i nodi in cluster basati sui label di K-Means
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
            if u in cluster_nodes and v in cluster_nodes:
                edges.append((u, v, data))

        # Crea il sottografo
        subgraph = nx.DiGraph()
        for node in nodes:
            subgraph.add_node(node, **full_graph.nodes[node])
        subgraph.add_edges_from([(u, v, data) for u, v, data in edges])

        # Duplica gli archi comuni se necessario
        subgraph = duplicate_common_edges(subgraph, full_graph, cluster_idx)

        # Assicura che il sottografo sia fortemente connesso
        if not nx.is_strongly_connected(subgraph):
            subgraph = make_subgraph_strongly_connected(subgraph, full_graph)

        # Rendi il sottografo Euleriano
        subgraph = make_subgraph_eulerian(subgraph, full_graph)

        # Limita il numero di nodi in base alla capacità
        if capacity < len(subgraph.nodes()):
            # Logica per limitare i nodi, ad esempio rimuovendo quelli più vicini al centroide
            centroid = np.mean([[full_graph.nodes[node]['x'], full_graph.nodes[node]['y']] for node in cluster_nodes], axis=0)
            sorted_nodes = sorted(subgraph.nodes(data=True), key=lambda x: math.hypot(x[1]['x'] - centroid[0], x[1]['y'] - centroid[1]))
            limited_nodes = [node[0] for node in sorted_nodes[:capacity]]
            subgraph = subgraph.subgraph(limited_nodes).copy()

        # Log dei nodi nel sottografo
        print(f"Subgraph {cluster_idx} contiene {len(subgraph.nodes())} nodi: {list(subgraph.nodes())}")

        subgraphs.append(subgraph)

    return subgraphs

def duplicate_common_edges(subgraph, full_graph, cluster_idx):
    """
    Duplica gli archi che sono comuni tra sottografi per garantire che ogni sottografo
    possa formare un tour chiuso indipendentemente.
    """
    # Implementa la logica per duplicare gli archi comuni se necessario
    # Al momento, questa funzione non aggiunge nulla. Puoi implementarla secondo le tue esigenze.
    return subgraph

def make_subgraph_strongly_connected(subgraph, full_graph):
    """
    Aggiunge archi dal grafo completo al sottografo per renderlo fortemente connesso.
    """
    # Trova le componenti fortemente connesse
    sccs = list(nx.strongly_connected_components(subgraph))
    if len(sccs) == 1:
        return subgraph  # Già fortemente connesso

    # Costruisci un semplice ciclo attraverso le SCC
    scc_list = [scc for scc in sccs]
    num_scc = len(scc_list)
    for i in range(num_scc):
        scc_from = scc_list[i]
        scc_to = scc_list[(i + 1) % num_scc]

        # Trova un arco dal grafo completo da scc_from a scc_to
        found = False
        for u in scc_from:
            for v in scc_to:
                if full_graph.has_edge(u, v):
                    subgraph.add_edge(u, v, **full_graph.get_edge_data(u, v))
                    found = True
                    break
            if found:
                break
        if not found:
            # Se non esiste un arco diretto, prova a trovare un percorso
            try:
                path = nx.shortest_path(full_graph, source=list(scc_from)[0], target=list(scc_to)[0])
                for s, t in zip(path[:-1], path[1:]):
                    if not subgraph.has_edge(s, t):
                        subgraph.add_edge(s, t, **full_graph.get_edge_data(s, t))
            except nx.NetworkXNoPath:
                raise ValueError(f"Impossibile trovare un percorso da {list(scc_from)[0]} a {list(scc_to)[0]} per rendere il sottografo fortemente connesso.")

    return subgraph

def make_subgraph_eulerian(subgraph, full_graph):
    """
    Modifica il sottografo per renderlo Euleriano bilanciando in-degree e out-degree.
    """
    # Verifica se il sottografo è già Euleriano
    if nx.is_eulerian(subgraph):
        return subgraph

    # Calcola l'imbalanced per ogni nodo
    imbalance = {}
    for node in subgraph.nodes():
        in_deg = subgraph.in_degree(node)
        out_deg = subgraph.out_degree(node)
        imbalance[node] = out_deg - in_deg

    # Nodi con imbalance positivo necessitano di ulteriori incoming edges
    # Nodi con imbalance negativo necessitano di ulteriori outgoing edges
    positive_imbalance_nodes = [node for node, imbal in imbalance.items() if imbal > 0]
    negative_imbalance_nodes = [node for node, imbal in imbalance.items() if imbal < 0]

    # Accoppia i nodi per bilanciare gli imbalances
    while positive_imbalance_nodes and negative_imbalance_nodes:
        u = positive_imbalance_nodes.pop()
        v = negative_imbalance_nodes.pop()

        # Aggiungi arco da v a u
        if subgraph.has_edge(v, u):
            subgraph.add_edge(v, u, **full_graph.get_edge_data(v, u))
        else:
            # Se l'arco non esiste, trova un percorso e aggiungi gli archi necessari
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
                raise ValueError(f"Impossibile trovare un percorso da {v} a {u} per bilanciare gli imbalances.")

    return subgraph

def save_subgraphs(subgraphs, output_dir):
    """
    Salva ciascun sottografo in un file JSON nella directory di output specificata.
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

        for u, v, data in subgraph.edges(data=True):
            edge_info = {
                "from": u,
                "to": v,
                "weight": data.get("weight", 1.0)
            }
            if 'cluster' in data:
                edge_info['cluster'] = data['cluster']
            subgraph_data["edges"].append(edge_info)

        # Scrivi il sottografo in un file JSON
        subgraph_file_path = os.path.join(output_dir, f"subgraph_{idx}.json")
        with open(subgraph_file_path, 'w') as f:
            json.dump(subgraph_data, f, indent=4)

        subgraph_paths.append(subgraph_file_path)

    return subgraph_paths
